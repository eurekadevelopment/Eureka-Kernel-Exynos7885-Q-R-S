/*
 * SVC Greybus driver.
 *
 * Copyright 2015 Google Inc.
 * Copyright 2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#include <linux/workqueue.h>

#include "greybus.h"

#define CPORT_FLAGS_E2EFC       BIT(0)
#define CPORT_FLAGS_CSD_N       BIT(1)
#define CPORT_FLAGS_CSV_N       BIT(2)

struct gb_svc {
	struct gb_connection	*connection;
};

struct svc_hotplug {
	struct work_struct work;
	struct gb_connection *connection;
	struct gb_svc_intf_hotplug_request data;
};

static struct ida greybus_svc_device_id_map;

/*
 * AP's SVC cport is required early to get messages from the SVC. This happens
 * even before the Endo is created and hence any modules or interfaces.
 *
 * This is a temporary connection, used only at initial bootup.
 */
struct gb_connection *
gb_ap_svc_connection_create(struct greybus_host_device *hd)
{
	struct gb_connection *connection;

	connection = gb_connection_create_range(hd, NULL, hd->parent,
						GB_SVC_CPORT_ID,
						GREYBUS_PROTOCOL_SVC,
						GB_SVC_CPORT_ID,
						GB_SVC_CPORT_ID + 1);

	return connection;
}

/*
 * We know endo-type and AP's interface id now, lets create a proper svc
 * connection (and its interface/bundle) now and get rid of the initial
 * 'partially' initialized one svc connection.
 */
static struct gb_interface *
gb_ap_interface_create(struct greybus_host_device *hd,
		       struct gb_connection *connection, u8 interface_id)
{
	struct gb_interface *intf;
	struct device *dev = &hd->endo->dev;

	intf = gb_interface_create(hd, interface_id);
	if (!intf) {
		dev_err(dev, "%s: Failed to create interface with id %hhu\n",
			__func__, interface_id);
		return NULL;
	}

	intf->device_id = GB_DEVICE_ID_AP;
	svc_update_connection(intf, connection);

	/* Its no longer a partially initialized connection */
	hd->initial_svc_connection = NULL;

	return intf;
}

static int gb_svc_intf_device_id(struct gb_svc *svc, u8 intf_id, u8 device_id)
{
	struct gb_svc_intf_device_id_request request;

	request.intf_id = intf_id;
	request.device_id = device_id;

	return gb_operation_sync(svc->connection, GB_SVC_TYPE_INTF_DEVICE_ID,
				 &request, sizeof(request), NULL, 0);
}

int gb_svc_intf_reset(struct gb_svc *svc, u8 intf_id)
{
	struct gb_svc_intf_reset_request request;

	request.intf_id = intf_id;

	return gb_operation_sync(svc->connection, GB_SVC_TYPE_INTF_RESET,
				 &request, sizeof(request), NULL, 0);
}
EXPORT_SYMBOL_GPL(gb_svc_intf_reset);

int gb_svc_dme_peer_get(struct gb_svc *svc, u8 intf_id, u16 attr, u16 selector,
			u32 *value)
{
	struct gb_svc_dme_peer_get_request request;
	struct gb_svc_dme_peer_get_response response;
	u16 result;
	int ret;

	request.intf_id = intf_id;
	request.attr = cpu_to_le16(attr);
	request.selector = cpu_to_le16(selector);

	ret = gb_operation_sync(svc->connection, GB_SVC_TYPE_DME_PEER_GET,
				&request, sizeof(request),
				&response, sizeof(response));
	if (ret) {
		dev_err(&svc->connection->dev,
			"failed to get DME attribute (%hhu %hx %hu) %d\n",
			intf_id, attr, selector, ret);
		return ret;
	}

	result = le16_to_cpu(response.result_code);
	if (result) {
		dev_err(&svc->connection->dev,
			"Unipro error %hu while getting DME attribute (%hhu %hx %hu)\n",
			result, intf_id, attr, selector);
		return -EINVAL;
	}

	if (value)
		*value = le32_to_cpu(response.attr_value);

	return 0;
}
EXPORT_SYMBOL_GPL(gb_svc_dme_peer_get);

int gb_svc_dme_peer_set(struct gb_svc *svc, u8 intf_id, u16 attr, u16 selector,
			u32 value)
{
	struct gb_svc_dme_peer_set_request request;
	struct gb_svc_dme_peer_set_response response;
	u16 result;
	int ret;

	request.intf_id = intf_id;
	request.attr = cpu_to_le16(attr);
	request.selector = cpu_to_le16(selector);
	request.value = cpu_to_le32(value);

	ret = gb_operation_sync(svc->connection, GB_SVC_TYPE_DME_PEER_SET,
				&request, sizeof(request),
				&response, sizeof(response));
	if (ret) {
		dev_err(&svc->connection->dev,
			"failed to set DME attribute (%hhu %hx %hu %u) %d\n",
			intf_id, attr, selector, value, ret);
		return ret;
	}

	result = le16_to_cpu(response.result_code);
	if (result) {
		dev_err(&svc->connection->dev,
			"Unipro error %hu while setting DME attribute (%hhu %hx %hu %u)\n",
			result, intf_id, attr, selector, value);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(gb_svc_dme_peer_set);

/*
 * T_TstSrcIncrement is written by the module on ES2 as a stand-in for boot
 * status attribute. AP needs to read and clear it, after reading a non-zero
 * value from it.
 *
 * FIXME: This is module-hardware dependent and needs to be extended for every
 * type of module we want to support.
 */
static int gb_svc_read_and_clear_module_boot_status(struct gb_interface *intf)
{
	struct greybus_host_device *hd = intf->hd;
	int ret;
	u32 value;

	/* Read and clear boot status in T_TstSrcIncrement */
	ret = gb_svc_dme_peer_get(hd->svc, intf->interface_id,
				  DME_ATTR_T_TST_SRC_INCREMENT,
				  DME_ATTR_SELECTOR_INDEX, &value);

	if (ret)
		return ret;

	/*
	 * A nonzero boot status indicates the module has finished
	 * booting. Clear it.
	 */
	if (!value) {
		dev_err(&intf->dev, "Module not ready yet\n");
		return -ENODEV;
	}

	return gb_svc_dme_peer_set(hd->svc, intf->interface_id,
				   DME_ATTR_T_TST_SRC_INCREMENT,
				   DME_ATTR_SELECTOR_INDEX, 0);
}

int gb_svc_connection_create(struct gb_svc *svc,
				u8 intf1_id, u16 cport1_id,
				u8 intf2_id, u16 cport2_id)
{
	struct gb_svc_conn_create_request request;

	request.intf1_id = intf1_id;
	request.cport1_id = cport1_id;
	request.intf2_id = intf2_id;
	request.cport2_id = cport2_id;
	/*
	 * XXX: fix connections paramaters to TC0 and all CPort flags
	 * for now.
	 */
	request.tc = 0;
	request.flags = CPORT_FLAGS_CSV_N | CPORT_FLAGS_E2EFC;

	return gb_operation_sync(svc->connection, GB_SVC_TYPE_CONN_CREATE,
				 &request, sizeof(request), NULL, 0);
}
EXPORT_SYMBOL_GPL(gb_svc_connection_create);

void gb_svc_connection_destroy(struct gb_svc *svc, u8 intf1_id, u16 cport1_id,
			       u8 intf2_id, u16 cport2_id)
{
	struct gb_svc_conn_destroy_request request;
	struct gb_connection *connection = svc->connection;
	int ret;

	request.intf1_id = intf1_id;
	request.cport1_id = cport1_id;
	request.intf2_id = intf2_id;
	request.cport2_id = cport2_id;

	ret = gb_operation_sync(connection, GB_SVC_TYPE_CONN_DESTROY,
				&request, sizeof(request), NULL, 0);
	if (ret) {
		dev_err(&connection->dev,
			"failed to destroy connection (%hhx:%hx %hhx:%hx) %d\n",
			intf1_id, cport1_id, intf2_id, cport2_id, ret);
	}
}
EXPORT_SYMBOL_GPL(gb_svc_connection_destroy);

/* Creates bi-directional routes between the devices */
static int gb_svc_route_create(struct gb_svc *svc, u8 intf1_id, u8 dev1_id,
			       u8 intf2_id, u8 dev2_id)
{
	struct gb_svc_route_create_request request;

	request.intf1_id = intf1_id;
	request.dev1_id = dev1_id;
	request.intf2_id = intf2_id;
	request.dev2_id = dev2_id;

	return gb_operation_sync(svc->connection, GB_SVC_TYPE_ROUTE_CREATE,
				 &request, sizeof(request), NULL, 0);
}

/* Destroys bi-directional routes between the devices */
static void gb_svc_route_destroy(struct gb_svc *svc, u8 intf1_id, u8 intf2_id)
{
	struct gb_svc_route_destroy_request request;
	int ret;

	request.intf1_id = intf1_id;
	request.intf2_id = intf2_id;

	ret = gb_operation_sync(svc->connection, GB_SVC_TYPE_ROUTE_DESTROY,
				&request, sizeof(request), NULL, 0);
	if (ret) {
		dev_err(&svc->connection->dev,
			"failed to destroy route (%hhx %hhx) %d\n",
			intf1_id, intf2_id, ret);
	}
}

static int gb_svc_version_request(struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct gb_protocol_version_response *version;
	struct device *dev = &connection->dev;

	version = op->request->payload;

	if (version->major > GB_SVC_VERSION_MAJOR) {
		dev_err(&connection->dev,
			"unsupported major version (%hhu > %hhu)\n",
			version->major, GB_SVC_VERSION_MAJOR);
		return -ENOTSUPP;
	}

	connection->module_major = version->major;
	connection->module_minor = version->minor;

	if (!gb_operation_response_alloc(op, sizeof(*version), GFP_KERNEL)) {
		dev_err(dev, "%s: error allocating response\n",
				__func__);
		return -ENOMEM;
	}

	version = op->response->payload;
	version->major = connection->module_major;
	version->minor = connection->module_minor;

	return 0;
}

static int gb_svc_hello(struct gb_operation *op)
{
	struct gb_connection *connection = op->connection;
	struct greybus_host_device *hd = connection->hd;
	struct gb_svc_hello_request *hello_request;
	struct device *dev = &connection->dev;
	struct gb_interface *intf;
	u16 endo_id;
	u8 interface_id;
	int ret;

	/* Hello message should be received only during early bootup */
	WARN_ON(hd->initial_svc_connection != connection);

	/*
	 * SVC sends information about the endo and interface-id on the hello
	 * request, use that to create an endo.
	 */
	if (op->request->payload_size < sizeof(*hello_request)) {
		dev_err(dev, "%s: Illegal size of hello request (%zu < %zu)\n",
			__func__, op->request->payload_size,
			sizeof(*hello_request));
		return -EINVAL;
	}

	hello_request = op->request->payload;
	endo_id = le16_to_cpu(hello_request->endo_id);
	interface_id = hello_request->interface_id;

	/* Setup Endo */
	ret = greybus_endo_setup(hd, endo_id, interface_id);
	if (ret)
		return ret;

	/*
	 * Endo and its modules are ready now, fix AP's partially initialized
	 * svc protocol and its connection.
	 */
	intf = gb_ap_interface_create(hd, connection, interface_id);
	if (!intf) {
		gb_endo_remove(hd->endo);
		return ret;
	}

	return 0;
}

/*
 * 'struct svc_hotplug' should be freed by svc_process_hotplug() before it
 * returns, irrespective of success or Failure in bringing up the module.
 */
static void svc_process_hotplug(struct work_struct *work)
{
	struct svc_hotplug *svc_hotplug = container_of(work, struct svc_hotplug,
						       work);
	struct gb_svc_intf_hotplug_request *hotplug = &svc_hotplug->data;
	struct gb_connection *connection = svc_hotplug->connection;
	struct gb_svc *svc = connection->private;
	struct greybus_host_device *hd = connection->hd;
	struct device *dev = &connection->dev;
	struct gb_interface *intf;
	u8 intf_id, device_id;
	int ret;

	/*
	 * Grab the information we need.
	 */
	intf_id = hotplug->intf_id;

	intf = gb_interface_create(hd, intf_id);
	if (!intf) {
		dev_err(dev, "%s: Failed to create interface with id %hhu\n",
			__func__, intf_id);
		goto free_svc_hotplug;
	}

	ret = gb_svc_read_and_clear_module_boot_status(intf);
	if (ret)
		goto destroy_interface;

	intf->unipro_mfg_id = le32_to_cpu(hotplug->data.unipro_mfg_id);
	intf->unipro_prod_id = le32_to_cpu(hotplug->data.unipro_prod_id);
	intf->vendor_id = le32_to_cpu(hotplug->data.ara_vend_id);
	intf->product_id = le32_to_cpu(hotplug->data.ara_prod_id);

	/*
	 * Create a device id for the interface:
	 * - device id 0 (GB_DEVICE_ID_SVC) belongs to the SVC
	 * - device id 1 (GB_DEVICE_ID_AP) belongs to the AP
	 *
	 * XXX Do we need to allocate device ID for SVC or the AP here? And what
	 * XXX about an AP with multiple interface blocks?
	 */
	device_id = ida_simple_get(&greybus_svc_device_id_map,
				   GB_DEVICE_ID_MODULES_START, 0, GFP_KERNEL);
	if (device_id < 0) {
		ret = device_id;
		dev_err(dev, "%s: Failed to allocate device id for interface with id %hhu (%d)\n",
			__func__, intf_id, ret);
		goto destroy_interface;
	}

	ret = gb_svc_intf_device_id(svc, intf_id, device_id);
	if (ret) {
		dev_err(dev, "%s: Device id operation failed, interface %hhu device_id %hhu (%d)\n",
			__func__, intf_id, device_id, ret);
		goto ida_put;
	}

	/*
	 * Create a two-way route between the AP and the new interface
	 */
	ret = gb_svc_route_create(svc, hd->endo->ap_intf_id, GB_DEVICE_ID_AP,
				  intf_id, device_id);
	if (ret) {
		dev_err(dev, "%s: Route create operation failed, interface %hhu device_id %hhu (%d)\n",
			__func__, intf_id, device_id, ret);
		goto svc_id_free;
	}

	ret = gb_interface_init(intf, device_id);
	if (ret) {
		dev_err(dev, "%s: Failed to initialize interface, interface %hhu device_id %hhu (%d)\n",
			__func__, intf_id, device_id, ret);
		goto destroy_route;
	}

	goto free_svc_hotplug;

destroy_route:
	gb_svc_route_destroy(svc, hd->endo->ap_intf_id, intf_id);
svc_id_free:
	/*
	 * XXX Should we tell SVC that this id doesn't belong to interface
	 * XXX anymore.
	 */
ida_put:
	ida_simple_remove(&greybus_svc_device_id_map, device_id);
destroy_interface:
	gb_interface_remove(hd, intf_id);
free_svc_hotplug:
	kfree(svc_hotplug);
}

/*
 * Bringing up a module can be time consuming, as that may require lots of
 * initialization on the module side. Over that, we may also need to download
 * the firmware first and flash that on the module.
 *
 * In order to make other hotplug events to not wait for all this to finish,
 * handle most of module hotplug stuff outside of the hotplug callback, with
 * help of a workqueue.
 */
static int gb_svc_intf_hotplug_recv(struct gb_operation *op)
{
	struct gb_message *request = op->request;
	struct svc_hotplug *svc_hotplug;

	if (request->payload_size < sizeof(svc_hotplug->data)) {
		dev_err(&op->connection->dev,
			"%s: short hotplug request received (%zu < %zu)\n",
			__func__, request->payload_size,
			sizeof(svc_hotplug->data));
		return -EINVAL;
	}

	svc_hotplug = kmalloc(sizeof(*svc_hotplug), GFP_KERNEL);
	if (!svc_hotplug)
		return -ENOMEM;

	svc_hotplug->connection = op->connection;
	memcpy(&svc_hotplug->data, op->request->payload, sizeof(svc_hotplug->data));

	INIT_WORK(&svc_hotplug->work, svc_process_hotplug);
	queue_work(system_unbound_wq, &svc_hotplug->work);

	return 0;
}

static int gb_svc_intf_hot_unplug_recv(struct gb_operation *op)
{
	struct gb_message *request = op->request;
	struct gb_svc_intf_hot_unplug_request *hot_unplug = request->payload;
	struct greybus_host_device *hd = op->connection->hd;
	struct device *dev = &op->connection->dev;
	struct gb_svc *svc = op->connection->private;
	u8 device_id;
	struct gb_interface *intf;
	u8 intf_id;

	if (request->payload_size < sizeof(*hot_unplug)) {
		dev_err(dev, "short hot unplug request received (%zu < %zu)\n",
			request->payload_size, sizeof(*hot_unplug));
		return -EINVAL;
	}

	intf_id = hot_unplug->intf_id;

	intf = gb_interface_find(hd, intf_id);
	if (!intf) {
		dev_err(dev, "%s: Couldn't find interface for id %hhu\n",
			__func__, intf_id);
		return -EINVAL;
	}

	device_id = intf->device_id;
	gb_interface_remove(hd, intf_id);

	/*
	 * Destroy the two-way route between the AP and the interface.
	 */
	gb_svc_route_destroy(svc, hd->endo->ap_intf_id, intf_id);

	ida_simple_remove(&greybus_svc_device_id_map, device_id);

	return 0;
}

static int gb_svc_intf_reset_recv(struct gb_operation *op)
{
	struct gb_message *request = op->request;
	struct gb_svc_intf_reset_request *reset;
	u8 intf_id;

	if (request->payload_size < sizeof(*reset)) {
		dev_err(&op->connection->dev,
			"short reset request received (%zu < %zu)\n",
			request->payload_size, sizeof(*reset));
		return -EINVAL;
	}
	reset = request->payload;

	intf_id = reset->intf_id;

	/* FIXME Reset the interface here */

	return 0;
}

static int gb_svc_request_recv(u8 type, struct gb_operation *op)
{
	switch (type) {
	case GB_SVC_TYPE_PROTOCOL_VERSION:
		return gb_svc_version_request(op);
	case GB_SVC_TYPE_SVC_HELLO:
		return gb_svc_hello(op);
	case GB_SVC_TYPE_INTF_HOTPLUG:
		return gb_svc_intf_hotplug_recv(op);
	case GB_SVC_TYPE_INTF_HOT_UNPLUG:
		return gb_svc_intf_hot_unplug_recv(op);
	case GB_SVC_TYPE_INTF_RESET:
		return gb_svc_intf_reset_recv(op);
	default:
		dev_err(&op->connection->dev,
			"unsupported request: %hhu\n", type);
		return -EINVAL;
	}
}

static int gb_svc_connection_init(struct gb_connection *connection)
{
	struct gb_svc *svc;

	svc = kzalloc(sizeof(*svc), GFP_KERNEL);
	if (!svc)
		return -ENOMEM;

	connection->hd->svc = svc;
	svc->connection = connection;
	connection->private = svc;

	WARN_ON(connection->hd->initial_svc_connection);
	connection->hd->initial_svc_connection = connection;

	ida_init(&greybus_svc_device_id_map);

	return 0;
}

static void gb_svc_connection_exit(struct gb_connection *connection)
{
	struct gb_svc *svc = connection->private;

	connection->hd->svc = NULL;
	connection->private = NULL;
	kfree(svc);
}

static struct gb_protocol svc_protocol = {
	.name			= "svc",
	.id			= GREYBUS_PROTOCOL_SVC,
	.major			= GB_SVC_VERSION_MAJOR,
	.minor			= GB_SVC_VERSION_MINOR,
	.connection_init	= gb_svc_connection_init,
	.connection_exit	= gb_svc_connection_exit,
	.request_recv		= gb_svc_request_recv,
};
gb_builtin_protocol_driver(svc_protocol);
