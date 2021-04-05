/****************************************************************************
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef _SCSC_CORE_H
#define _SCSC_CORE_H

#include <linux/types.h>
#include <linux/notifier.h>
#include "scsc_mifram.h"

#define SCSC_PANIC_CODE_FW	0
#define SCSC_PANIC_CODE_HOST	1
/* offset from 0x80000000, the start of BAAW1 region */
#define SCSC_BAAW1_OFFSET		0x400000

#define	SCSC_FW_EVENT_FAILURE			0
#define	SCSC_FW_EVENT_MOREDUMP_COMPLETE		1

struct device;
struct firmware;
struct scsc_mx;

enum scsc_service_id {
	SCSC_SERVICE_ID_NULL = 0,
	SCSC_SERVICE_ID_WLAN = 1,
	SCSC_SERVICE_ID_BT = 2,
	SCSC_SERVICE_ID_ANT = 3,
	SCSC_SERVICE_ID_R4DBG = 4,
	SCSC_SERVICE_ID_ECHO = 5,
	SCSC_SERVICE_ID_DBG_SAMPLER = 6,
	SCSC_SERVICE_ID_CLK20MHZ = 7,
	SCSC_SERVICE_ID_INVALID = 0xff,
};

enum scsc_module_client_reason {
	SCSC_MODULE_CLIENT_REASON_HW_PROBE = 0,
	SCSC_MODULE_CLIENT_REASON_HW_REMOVE = 1,
	SCSC_MODULE_CLIENT_REASON_RECOVERY = 2,
	SCSC_MODULE_CLIENT_REASON_INVALID = 0xff,
};

/* Core Driver Module registration */

struct scsc_mx_module_client {
	char *name;
	void (*probe)(struct scsc_mx_module_client *module_client, struct scsc_mx *mx, enum scsc_module_client_reason reason);
	void (*remove)(struct scsc_mx_module_client *module_client, struct scsc_mx *mx, enum scsc_module_client_reason reason);
};

/* Service Client interface */

struct scsc_service_client;

struct scsc_service_client {
	/** Called on Maxwell failure. The Client should Stop all SDRAM & MIF
	 * Mailbox access as fast as possible and inform the Manager by calling
	 * client_stopped() */
	void (*stop_on_failure)(struct scsc_service_client *client);
	/** Called when Maxwell failure has handled and the Maxwell has been
	 * reset. The Client should assume that any Maxwell resources it held are
	 * invalid */
	void (*failure_reset)(struct scsc_service_client *client, u16 scsc_panic_code);
	/* called when AP processor is going into suspend. */
	int (*suspend)(struct scsc_service_client *client);
	/* called when AP processor has resumed */
	int (*resume)(struct scsc_service_client *client);
	/* called when log collection has been triggered */
	void (*log)(struct scsc_service_client *client, u16 reason);
};


#define PANIC_RECORD_SIZE			64
#define PANIC_RECORD_DUMP_BUFFER_SZ		4096
/* WARNING: THIS IS INTERRUPT CONTEXT!
 * here: some serious warnings about not blocking or doing anything lengthy at all
 */
typedef void (*scsc_mifintrbit_handler)(int which_bit, void *data);

/*
 * Core Module Inteface
 */
int scsc_mx_module_register_client_module(struct scsc_mx_module_client *module_client);
void scsc_mx_module_unregister_client_module(struct scsc_mx_module_client *module_client);
int scsc_mx_module_reset(void);

/*
 *  Core Instance interface
 */
/** 1st thing to do is call open and return service managment interface*/
struct scsc_service *scsc_mx_service_open(struct scsc_mx *mx, enum scsc_service_id id, struct scsc_service_client *client, int *status);

/*
 * Service interface
 */
/** pass a portable dram reference and returns kernel pointer (basically is dealing with the pointers) */
void *scsc_mx_service_mif_addr_to_ptr(struct scsc_service *service, scsc_mifram_ref ref);
void *scsc_mx_service_mif_addr_to_phys(struct scsc_service *service, scsc_mifram_ref ref);
int scsc_mx_service_mif_ptr_to_addr(struct scsc_service *service, void *mem_ptr, scsc_mifram_ref *ref);

int scsc_mx_service_start(struct scsc_service *service, scsc_mifram_ref ref);
int scsc_mx_service_stop(struct scsc_service *service);
int scsc_mx_service_close(struct scsc_service *service);
int scsc_mx_service_mif_dump_registers(struct scsc_service *service);
int scsc_mx_service_get_abox_shared_mem(struct scsc_service *service, void **data);
/** Signal a failure detected by the Client. This will trigger the systemwide
 * failure handling procedure: _All_ Clients will be called back via
 * their stop_on_failure() handler as a side-effect. */
void scsc_mx_service_service_failed(struct scsc_service *service, const char *reason);


/* MEMORY Interface*/
/** Allocate a contiguous block of SDRAM accessible to Client Driver */
int scsc_mx_service_mifram_alloc(struct scsc_service *service, size_t nbytes, scsc_mifram_ref *ref, u32 align);
/** Free a contiguous block of SDRAM */
void scsc_mx_service_mifram_free(struct scsc_service *service, scsc_mifram_ref ref);

/* MBOX Interface */
/** Allocate n contiguous mailboxes. Outputs index of first mbox, returns FALSE if can’t allocate n contiguous mailboxes. */
bool scsc_mx_service_alloc_mboxes(struct scsc_service *service, int n, int *first_mbox_index);
/** Free n contiguous mailboxes. */
void scsc_service_free_mboxes(struct scsc_service *service, int n, int first_mbox_index);

/** Get kernel-space pointer to a mailbox.
 * The pointer can be cached as it is guaranteed not to change between service start & stop.
 **/
u32 *scsc_mx_service_get_mbox_ptr(struct scsc_service *service, int mbox_index);

/* IRQ Interface */
/* Getters/Setters */

/* From R4/M4 */
int scsc_service_mifintrbit_bit_mask_status_get(struct scsc_service *service);
int scsc_service_mifintrbit_get(struct scsc_service *service);
void scsc_service_mifintrbit_bit_clear(struct scsc_service *service, int which_bit);
void scsc_service_mifintrbit_bit_mask(struct scsc_service *service, int which_bit);
void scsc_service_mifintrbit_bit_unmask(struct scsc_service *service, int which_bit);

/* To R4/M4 */
enum scsc_mifintr_target {
	SCSC_MIFINTR_TARGET_R4 = 0,
	SCSC_MIFINTR_TARGET_M4 = 1
};

void scsc_service_mifintrbit_bit_set(struct scsc_service *service, int which_bit, enum scsc_mifintr_target dir);

/* Register an interrupt handler -TOHOST direction.
 * Function returns the IRQ associated , -EIO if all interrupts have been assigned */
int scsc_service_mifintrbit_register_tohost(struct scsc_service *service, scsc_mifintrbit_handler handler, void *data);
/* Unregister an interrupt handler associated with a bit -TOHOST direction */
int scsc_service_mifintrbit_unregister_tohost(struct scsc_service *service, int which_bit);

/* Get an interrupt bit associated with the target (R4/M4) -FROMHOST direction
 * Function returns the IRQ bit associated , -EIO if error */
int scsc_service_mifintrbit_alloc_fromhost(struct scsc_service *service, enum scsc_mifintr_target dir);
/* Free an interrupt bit associated with the target (R4/M4) -FROMHOST direction
 * Function returns the 0 if succedes , -EIO if error */
int scsc_service_mifintrbit_free_fromhost(struct scsc_service *service, int which_bit, enum scsc_mifintr_target dir);
/*
 * Return a kernel device associated 1:1 with the Maxwell instance.
 * This is published only for the purpose of associating service drivers
 * with a Maxwell instance for logging purposes. Clients should not make
 * any assumptions about the device type. In some configurations this may
 * be the associated host-interface device (AXI/PCIe),
 * but this may change in future.
 */
struct device *scsc_service_get_device(struct scsc_service *service);

int scsc_service_force_panic(struct scsc_service *service);

/*
 * API to share /sys/wifi kobject between core and wifi driver modules.
 * Depending upon the order of loading respective drivers, a kobject is
 * created and shared with the other driver. This convoluted implementation
 * is required as we need the common kobject associated with "/sys/wifi" directory
 * when creating a file underneth. core driver (mxman.c) need to create "memdump"
 * and wifi driver (dev.c,mgt.c) needs to create "mac_addr" files respectively.
 */
struct kobject *mxman_wifi_kobject_ref_get(void);
void mxman_wifi_kobject_ref_put(void);

/* MXLOGGER API */
/* If there is no service/mxman associated, register the observer as global (will affect all the mx instanes)*/
/* Users of these functions should ensure that the registers/unregister functions are balanced (i.e. if observer is registed as global,
 * it _has_ to unregister as global) */
int scsc_service_register_observer(struct scsc_service *service, char *name);
/* Unregister an observer */
int scsc_service_unregister_observer(struct scsc_service *service, char *name);

/* Reads a configuration file into memory.
 *
 * Path is relative to the currently selected firmware configuration
 * subdirectory.
 * Returns pointer to data or NULL if file not found.
 * Call mx140_file_release_conf()to release the memory.
 */
int mx140_file_request_conf(struct scsc_mx *mx, const struct firmware **conf, const char *config_path, const char *filename);

/* Reads a debug configuration file into memory.
 *
 * Path is relative to the currently selected firmware configuration
 * subdirectory.
 * Returns pointer to data or NULL if file not found.
 * Call mx140_file_release_conf()to release the memory.
 */
int mx140_file_request_debug_conf(struct scsc_mx *mx, const struct firmware **conf, const char *config_path);

/* Read device configuration file into memory.
 *
 * Path is relative to the device configuration directory.
 * Returns pointer to data or NULL if file not found.
 * Call mx140_file_release_conf() to release the memory.
 * This call is only used for configuration files that are
 * device instance specific (e.g. mac addresses)
 */
int mx140_file_request_device_conf(struct scsc_mx *mx, const struct firmware **conf, const char *config_path);

/* Release configuration file memory
 *
 * If conf is NULL, has no effect.
 */
void mx140_file_release_conf(struct scsc_mx *mx, const struct firmware *conf);

/* Read device configuration file into memory.
 *
 * Path is absolute.
 * Returns pointer to data or NULL if file not found.
 * Call mx140_release_file() to release the memory.
 */
int mx140_request_file(struct scsc_mx *mx, char *path, const struct firmware **firmp);

/* Release configuration file memory allocated with mx140_request_file()
 *
 * If firmp is NULL, has no effect.
 */
int mx140_release_file(struct scsc_mx *mx, const struct firmware *firmp);

/* 20 MHz clock API.
 * The mx140 device uses a clock that is also required by the USB driver.
 * This API allows the USB/clock driver to inform the mx140 driver that the
 * clock is required and that it must boot and/or keep the clock running.
 */

enum mx140_clk20mhz_status {
	MX140_CLK_SUCCESS = 0,	/* Returned successfully */
	MX140_CLK_STARTED,	/* mx140 has started the clock */
	MX140_CLK_STOPPED,	/* mx140 has stopped the clock */
	MX140_CLK_NOT_STARTED,	/* failed to start the clock */
	MX140_CLK_NOT_STOPPED,	/* failed to stop the clock */
	MX140_CLK_ASYNC_FAIL,	/* mx140 failure, async call */
};

/* Register for 20 MHz clock API callbacks
 *
 * Parameters:
 * client_cb:
 *  If client provides non-NULL client_cb, the request is asynchronous and
 *  the client will be called back when the clock service is started.
 *  If client_cb is NULL, the request is blocking.
 * data:
 *  opaque context for the client, and will be passed back in any callback
 *
 * Note it is possible that the callback may be made in the context of the
 * calling request/release function.
 *
 * Returns 0 on success
 */
int mx140_clk20mhz_register(void (*client_cb)(void *data, enum mx140_clk20mhz_status event), void *data);

/* Unregister for 20 MHz clock API callbacks.
 * After this call is made, the mx140 driver will no longer call back.
 */
void mx140_clk20mhz_unregister(void);

/* Client request that the clock be available.
 *
 * If a callback was installed via mx140_clk20mhz_register(), the mx140 driver
 * will call back when the clock is available. If no callback was installed,
 * the request is blocking and will return when the clock is running.
 *
 * Returns:
 *  mx140_clk20mhz_status if a blocking attempt was made to start the clock,
 *  MX140_CLK_SUCCESS if the request will happen asynchronously, or,
 *  -ve error code on other error.
 *
 */
int mx140_clk20mhz_request(void);

/* Client informs that the clock is no longer needed
 *
 * Returns:
 *  mx140_clk20mhz_status if a blocking attempt was made to stop the clock,
 *  MX140_CLK_SUCCESS if the request will happen asynchronously, or,
 *  -ve error code on other error.
 */
int mx140_clk20mhz_release(void);

/*
 * for set test mode.
 *
 */
bool slsi_is_rf_test_mode_enabled(void);

int mx140_log_dump(void);

/* Shared Data BT-ABOX */
struct scsc_btabox_data {
	unsigned long btaboxmem_start;
	size_t        btaboxmem_size;
};

int mxman_register_firmware_notifier(struct notifier_block *nb);
int mxman_unregister_firmware_notifier(struct notifier_block *nb);
void mxman_get_fw_version(char *version, size_t ver_sz);
void mxman_get_driver_version(char *version, size_t ver_sz);

/* Status of WLBT autorecovery on the platform
 *
 * Returns:
 *  false - enabled, true disabled
 */
bool mxman_recovery_disabled(void);

/* function to provide string representation of uint8 trigger code */
static inline const char *scsc_get_trigger_str(int code)
{
	switch (code) {
	case 1:	return "scsc_log_fw_panic";
	case 2:	return "scsc_log_user";
	case 3:	return "scsc_log_fw";
	case 4:	return "scsc_log_dumpstate";
	case 5:	return "scsc_log_host_wlan";
	case 6:	return "scsc_log_host_bt";
	case 7:	return "scsc_log_host_common";
	case 0:
	default:
		return "unknown";
	}
};

#endif
