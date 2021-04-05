/*****************************************************************************
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include "cfg80211_ops.h"
#include "debug.h"
#include "mgt.h"
#include "mlme.h"

struct net_device *slsi_nan_get_netdev(struct slsi_dev *sdev)
{
#if CONFIG_SCSC_WLAN_MAX_INTERFACES >= 4
	return slsi_get_netdev(sdev, SLSI_NET_INDEX_NAN);
#else
	return NULL;
#endif
}

static int slsi_nan_get_new_id(u32 id_map, int max_ids)
{
	int i;

	for (i = 1; i <= max_ids; i++) {
		if (!(id_map & BIT(i)))
			return i;
	}
	return 0;
}

static int slsi_nan_get_new_publish_id(struct netdev_vif *ndev_vif)
{
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	return slsi_nan_get_new_id(ndev_vif->nan.service_id_map, SLSI_NAN_MAX_SERVICE_ID);
}

static int slsi_nan_get_new_subscribe_id(struct netdev_vif *ndev_vif)
{
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	return slsi_nan_get_new_id(ndev_vif->nan.service_id_map, SLSI_NAN_MAX_SERVICE_ID);
}

static bool slsi_nan_is_publish_id_active(struct netdev_vif *ndev_vif, u32 id)
{
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	return ndev_vif->nan.service_id_map & BIT(id);
}

static bool slsi_nan_is_subscribe_id_active(struct netdev_vif *ndev_vif, u32 id)
{
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	return ndev_vif->nan.service_id_map & BIT(id);
}

static int slsi_nan_get_new_ndp_id(struct netdev_vif *ndev_vif)
{
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	return slsi_nan_get_new_id(ndev_vif->nan.ndp_id_map, SLSI_NAN_MAX_NDP_INSTANCES);
}

static void slsi_nan_pre_check(struct slsi_dev *sdev, struct net_device *dev, int *ret, int *reply_status)
{
	*ret = WIFI_HAL_SUCCESS;
	*reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;
	if (!dev) {
		SLSI_ERR(sdev, "No NAN interface\n");
		*ret = -WIFI_HAL_ERROR_NOT_SUPPORTED;
		*reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
	}

	if (!slsi_dev_nan_supported(sdev)) {
		SLSI_ERR(sdev, "NAN not allowed(mib:%d)\n", sdev->nan_enabled);
		*ret = WIFI_HAL_ERROR_NOT_SUPPORTED;
		*reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
	}
}

int slsi_nan_ndp_new_entry(struct slsi_dev *sdev, struct net_device *dev, u32 ndp_id,
			   u16 ndl_vif_id, u8 *local_ndi, u8 *peer_nmi)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16 ndl_id;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));

	if (ndl_vif_id < SLSI_NAN_DATA_IFINDEX_START ||
	    ndl_vif_id >= SLSI_NAN_DATA_IFINDEX_START + SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_ERR(sdev, "Invalid ndl_vif:%d\n", ndl_vif_id);
		return 1;
	}

	if (ndp_id == 0 || ndp_id > SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_ERR(sdev, "Invalid ndp:%d\n", ndp_id);
		return 1;
	}

	ndl_id = ndl_vif_id - SLSI_NAN_DATA_IFINDEX_START;
	if (ndev_vif->nan.ndl_list[ndl_id].ndp_count < 0 ||
	    ndev_vif->nan.ndl_list[ndl_id].ndp_count > SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_WARN(sdev, "improper ndp count(%d) for vif_id(%d)\n",
			  ndev_vif->nan.ndl_list[ndl_id].ndp_count, ndl_vif_id);
	}

	ndev_vif->nan.ndp_id_map |= (u32)BIT(ndp_id);
	if (peer_nmi)
		ether_addr_copy(ndev_vif->nan.ndl_list[ndl_id].peer_nmi, peer_nmi);
	ndev_vif->nan.ndl_list[ndl_id].ndp_count++;
	if (local_ndi)
		ether_addr_copy(ndev_vif->nan.ndp_ndi[ndp_id - 1], local_ndi);
	ndev_vif->nan.ndp_id2ndl_vif[ndp_id - 1] = ndl_vif_id;
	ndev_vif->nan.ndp_state[ndp_id - 1] = ndp_slot_status_in_use;
	return 0;
}

void slsi_nan_ndp_del_entry(struct slsi_dev *sdev, struct net_device *dev, u32 ndp_id)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16 ndl_vif_id =0 , ndl_id;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));

	if (ndp_id == 0 || ndp_id > SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_WARN(sdev, "Invalid ndp:%d\n", ndp_id);
		return;
	}

	ndl_vif_id = ndev_vif->nan.ndp_id2ndl_vif[ndp_id - 1];
	if (ndl_vif_id < SLSI_NAN_DATA_IFINDEX_START ||
	    ndl_vif_id >= SLSI_NAN_DATA_IFINDEX_START + SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_WARN(sdev, "Invalid ndl_vif:%d\n", ndl_vif_id);
		return;
	}

	ndl_id = ndl_vif_id - SLSI_NAN_DATA_IFINDEX_START;
	ndev_vif->nan.ndp_id_map &= ~(u32)BIT(ndp_id);
	ndev_vif->nan.ndl_list[ndl_id].ndp_count--;
	if (ndev_vif->nan.ndl_list[ndl_id].ndp_count == 0)
		slsi_eth_zero_addr(ndev_vif->nan.ndl_list[ndl_id].peer_nmi);
	ndev_vif->nan.ndp_id2ndl_vif[ndp_id - 1] = 0;
	slsi_eth_zero_addr(ndev_vif->nan.ndp_ndi[ndp_id - 1]);
	if (ndev_vif->nan.ndl_list[ndl_id].ndp_count < 0)
		SLSI_WARN(sdev, "ndp_count is negative %d for ndl idx %d\n",
			  ndev_vif->nan.ndl_list[ndl_id].ndp_count, ndl_id);
	ndev_vif->nan.ndp_state[ndp_id - 1] = ndp_slot_status_free;
}

u16 slsi_nan_ndp_get_ndl_vif_id(u8 *peer_mni, struct slsi_nan_ndl_info *ndl_list)
{
	u16 i, free_idx = SLSI_NAN_MAX_NDP_INSTANCES + SLSI_NAN_DATA_IFINDEX_START;

	for (i = 0; i < SLSI_NAN_MAX_NDP_INSTANCES; i++) {
		if (ether_addr_equal(peer_mni, ndl_list[i].peer_nmi))
			return i + SLSI_NAN_DATA_IFINDEX_START;
		if (free_idx == SLSI_NAN_MAX_NDP_INSTANCES + SLSI_NAN_DATA_IFINDEX_START &&
		    ndl_list[i].ndp_count == 0)
			free_idx = i + SLSI_NAN_DATA_IFINDEX_START;
	}
	return free_idx;
}

void slsi_nan_get_mac(struct slsi_dev *sdev, char *nan_mac_addr)
{
	memset(nan_mac_addr, 0, ETH_ALEN);
#if CONFIG_SCSC_WLAN_MAX_INTERFACES >= 4
	if (slsi_dev_nan_supported(sdev))
		ether_addr_copy(nan_mac_addr, sdev->netdev_addresses[SLSI_NET_INDEX_NAN]);
#endif
}

static void slsi_vendor_nan_command_reply(struct wiphy *wiphy, u32 status, u32 error, u32 response_type,
					  u16 id, struct slsi_hal_nan_capabilities *capabilities, u16 req_id)
{
	int reply_len;
	struct sk_buff  *reply;

	reply_len = SLSI_NL_VENDOR_REPLY_OVERHEAD + SLSI_NL_ATTRIBUTE_U32_LEN *
		    (3 + sizeof(struct slsi_hal_nan_capabilities));
	reply = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, reply_len);
	if (!reply) {
		SLSI_WARN_NODEV("SKB alloc failed for vendor_cmd reply\n");
		return;
	}

	nla_put_u32(reply, NAN_REPLY_ATTR_STATUS_TYPE, status);
	nla_put_u32(reply, NAN_REPLY_ATTR_VALUE, error);
	nla_put_u32(reply, NAN_REPLY_ATTR_RESPONSE_TYPE, response_type);
	nla_put_u16(reply, NAN_REPLY_ATTR_HAL_TRANSACTION_ID, req_id);

	if (capabilities) {
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_CONCURRENT_CLUSTER,
			    capabilities->max_concurrent_nan_clusters);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_PUBLISHES, capabilities->max_publishes);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_SUBSCRIBES, capabilities->max_subscribes);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_SERVICE_NAME_LEN, capabilities->max_service_name_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_MATCH_FILTER_LEN, capabilities->max_match_filter_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_TOTAL_MATCH_FILTER_LEN,
			    capabilities->max_total_match_filter_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_SERVICE_SPECIFIC_INFO_LEN,
			    capabilities->max_service_specific_info_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_VSA_DATA_LEN, capabilities->max_vsa_data_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_MESH_DATA_LEN, capabilities->max_mesh_data_len);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_NDI_INTERFACES, capabilities->max_ndi_interfaces);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_NDP_SESSIONS, capabilities->max_ndp_sessions);
		nla_put_u32(reply, NAN_REPLY_ATTR_CAP_MAX_APP_INFO_LEN, capabilities->max_app_info_len);
	} else if (id) {
		if (response_type < NAN_DP_INTERFACE_CREATE)
			nla_put_u16(reply, NAN_REPLY_ATTR_PUBLISH_SUBSCRIBE_TYPE, id);
		else
			nla_put_u16(reply, NAN_REPLY_ATTR_NDP_INSTANCE_ID, id);
	}

	if (cfg80211_vendor_cmd_reply(reply))
		SLSI_ERR_NODEV("FAILED to reply nan coammnd. response_type:%d\n", response_type);
}

static int slsi_nan_get_sdea_params_nl(struct slsi_dev *sdev, struct slsi_nan_sdea_ctrl_params *sdea_params,
				       const struct nlattr *iter, int nl_attr_id)
{
	switch (nl_attr_id) {
	case NAN_REQ_ATTR_SDEA_PARAM_NDP_TYPE:
		if (slsi_util_nla_get_u8(iter, &(sdea_params->ndp_type)))
			return -EINVAL;
		sdea_params->config_nan_data_path = 1;
		break;
	case NAN_REQ_ATTR_SDEA_PARAM_SECURITY_CFG:
		if (slsi_util_nla_get_u8(iter, &(sdea_params->security_cfg)))
			return -EINVAL;
		sdea_params->config_nan_data_path = 1;
		break;
	case NAN_REQ_ATTR_SDEA_PARAM_RANGING_STATE:
		if (slsi_util_nla_get_u8(iter, &(sdea_params->ranging_state)))
			return -EINVAL;
		sdea_params->config_nan_data_path = 1;
		break;
	case NAN_REQ_ATTR_SDEA_PARAM_RANGE_REPORT:
		if (slsi_util_nla_get_u8(iter, &(sdea_params->range_report)))
			return -EINVAL;
		sdea_params->config_nan_data_path = 1;
		break;
	case NAN_REQ_ATTR_SDEA_PARAM_QOS_CFG:
		if (slsi_util_nla_get_u8(iter, &(sdea_params->qos_cfg)))
			return -EINVAL;
		sdea_params->config_nan_data_path = 1;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int slsi_nan_get_ranging_cfg_nl(struct slsi_dev *sdev, struct slsi_nan_ranging_cfg *ranging_cfg,
				       const struct nlattr *iter, int nl_attr_id)
{
	switch (nl_attr_id) {
	case NAN_REQ_ATTR_RANGING_CFG_INTERVAL:
		if (slsi_util_nla_get_u32(iter, &(ranging_cfg->ranging_interval_msec)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_RANGING_CFG_INDICATION:
		if (slsi_util_nla_get_u32(iter, &(ranging_cfg->config_ranging_indications)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_RANGING_CFG_INGRESS_MM:
		if (slsi_util_nla_get_u32(iter, &(ranging_cfg->distance_ingress_mm)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_RANGING_CFG_EGRESS_MM:
		if (slsi_util_nla_get_u32(iter, &(ranging_cfg->distance_egress_mm)))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int slsi_nan_get_security_info_nl(struct slsi_dev *sdev, struct slsi_nan_security_info *sec_info,
					 const struct nlattr *iter, int nl_attr_id)
{
	u32 len = 0;

	switch (nl_attr_id) {
	case NAN_REQ_ATTR_CIPHER_TYPE:
		if (slsi_util_nla_get_u32(iter, &(sec_info->cipher_type)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_SECURITY_KEY_TYPE:
		if (slsi_util_nla_get_u8(iter, &(sec_info->key_info.key_type)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_SECURITY_PMK_LEN:
		if (slsi_util_nla_get_u32(iter, &len))
			return -EINVAL;
		sec_info->key_info.body.pmk_info.pmk_len = len;
		break;
	case NAN_REQ_ATTR_SECURITY_PMK:
		if (slsi_util_nla_get_data(iter, sec_info->key_info.body.pmk_info.pmk_len, sec_info->key_info.body.pmk_info.pmk))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_SECURITY_PASSPHRASE_LEN:
		if (slsi_util_nla_get_u32(iter, &len))
			return -EINVAL;
		sec_info->key_info.body.passphrase_info.passphrase_len = len;
		break;
	case NAN_REQ_ATTR_SECURITY_PASSPHRASE:
		if (slsi_util_nla_get_data(iter, sec_info->key_info.body.passphrase_info.passphrase_len,
			sec_info->key_info.body.passphrase_info.passphrase))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_SCID_LEN:
		if (slsi_util_nla_get_u32(iter, &(sec_info->scid_len)))
			return -EINVAL;
		break;
	case NAN_REQ_ATTR_SCID:
		if (sec_info->scid_len > sizeof(sec_info->scid))
			sec_info->scid_len = sizeof(sec_info->scid);
		if (slsi_util_nla_get_data(iter, sec_info->scid_len, sec_info->scid))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int slsi_nan_get_range_resp_cfg_nl(struct slsi_dev *sdev, struct slsi_nan_range_response_cfg *cfg,
					  const struct nlattr *iter, int nl_attr_id)
{
	switch (nl_attr_id) {
	case NAN_REQ_ATTR_RANGE_RESPONSE_CFG_PUBLISH_ID:
		if (slsi_util_nla_get_u16(iter, &(cfg->publish_id)))
			return -EINVAL;
		break;

	case NAN_REQ_ATTR_RANGE_RESPONSE_CFG_REQUESTOR_ID:
		if (slsi_util_nla_get_u32(iter, &(cfg->requestor_instance_id)))
			return -EINVAL;
		break;

	case NAN_REQ_ATTR_RANGE_RESPONSE_CFG_PEER_ADDR:
		if (slsi_util_nla_get_data(iter, ETH_ALEN, cfg->peer_addr))
			return -EINVAL;
		break;

	case NAN_REQ_ATTR_RANGE_RESPONSE_CFG_RANGING_RESPONSE:
		if (slsi_util_nla_get_u8(iter, &(cfg->ranging_response)))
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* NAN HAL REQUESTS */

static int slsi_nan_enable_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_enable_req *hal_req,
					 const void *data, int len)
{
	int type, tmp;
	const struct nlattr *iter;
	u8 val = 0;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_MASTER_PREF:
			if (slsi_util_nla_get_u8(iter, &(hal_req->master_pref)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_CLUSTER_LOW:
			if (slsi_util_nla_get_u16(iter, &(hal_req->cluster_low)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_CLUSTER_HIGH:
			if (slsi_util_nla_get_u16(iter, &(hal_req->cluster_high)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUPPORT_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->support_5g_val)))
				return -EINVAL;
			hal_req->config_support_5g = 1;
			break;

		case NAN_REQ_ATTR_SID_BEACON_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->sid_beacon_val)))
				return -EINVAL;
			hal_req->config_sid_beacon = 1;
			break;

		case NAN_REQ_ATTR_RSSI_CLOSE_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_close_2dot4g_val)))
				return -EINVAL;
			hal_req->config_2dot4g_rssi_close = 1;
			break;

		case NAN_REQ_ATTR_RSSI_MIDDLE_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_middle_2dot4g_val)))
				return -EINVAL;
			hal_req->config_2dot4g_rssi_middle = 1;
			break;

		case NAN_REQ_ATTR_RSSI_PROXIMITY_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_proximity_2dot4g_val)))
				return -EINVAL;
			hal_req->config_2dot4g_rssi_proximity = 1;
			break;

		case NAN_REQ_ATTR_HOP_COUNT_LIMIT_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->hop_count_limit_val)))
				return -EINVAL;
			hal_req->config_hop_count_limit = 1;
			break;

		case NAN_REQ_ATTR_SUPPORT_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->support_2dot4g_val)))
				return -EINVAL;
			hal_req->config_2dot4g_support = 1;
			break;

		case NAN_REQ_ATTR_BEACONS_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->beacon_2dot4g_val)))
				return -EINVAL;
			hal_req->config_2dot4g_beacons = 1;
			break;

		case NAN_REQ_ATTR_SDF_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->sdf_2dot4g_val)))
				 return -EINVAL;
			hal_req->config_2dot4g_sdf = 1;
			break;

		case NAN_REQ_ATTR_BEACON_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->beacon_5g_val)))
				return -EINVAL;
			hal_req->config_5g_beacons = 1;
			break;

		case NAN_REQ_ATTR_SDF_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->sdf_5g_val)))
				 return -EINVAL;
			hal_req->config_5g_sdf = 1;
			break;

		case NAN_REQ_ATTR_RSSI_CLOSE_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_close_5g_val)))
				return -EINVAL;
			hal_req->config_5g_rssi_close = 1;
			break;

		case NAN_REQ_ATTR_RSSI_MIDDLE_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_middle_5g_val)))
				return -EINVAL;
			hal_req->config_5g_rssi_middle = 1;
			break;

		case NAN_REQ_ATTR_RSSI_CLOSE_PROXIMITY_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_close_proximity_5g_val)))
				return -EINVAL;
			hal_req->config_5g_rssi_close_proximity = 1;
			break;

		case NAN_REQ_ATTR_RSSI_WINDOW_SIZE_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_window_size_val)))
				return -EINVAL;
			hal_req->config_rssi_window_size = 1;
			break;

		case NAN_REQ_ATTR_OUI_VAL:
			if (slsi_util_nla_get_u32(iter, &(hal_req->oui_val)))
				return -EINVAL;
			hal_req->config_oui = 1;
			break;

		case NAN_REQ_ATTR_MAC_ADDR_VAL:
			if (slsi_util_nla_get_data(iter, ETH_ALEN, hal_req->intf_addr_val))
				return -EINVAL;
			hal_req->config_intf_addr = 1;
			break;

		case NAN_REQ_ATTR_CLUSTER_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->config_cluster_attribute_val)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SOCIAL_CH_SCAN_DWELL_TIME:
			if (slsi_util_nla_get_data(iter, sizeof(hal_req->scan_params_val.dwell_time), hal_req->scan_params_val.dwell_time))
				return -EINVAL;
			hal_req->config_scan_params = 1;
			break;

		case NAN_REQ_ATTR_SOCIAL_CH_SCAN_PERIOD:
			if (slsi_util_nla_get_data(iter, sizeof(hal_req->scan_params_val.scan_period), hal_req->scan_params_val.scan_period))
				return -EINVAL;
			hal_req->config_scan_params = 1;
			break;

		case NAN_REQ_ATTR_RANDOM_FACTOR_FORCE_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->random_factor_force_val)))
				return -EINVAL;
			hal_req->config_random_factor_force = 1;
			break;

		case NAN_REQ_ATTR_HOP_COUNT_FORCE_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->hop_count_force_val)))
				return -EINVAL;
			hal_req->config_hop_count_force = 1;
			break;

		case NAN_REQ_ATTR_CHANNEL_2G4_MHZ_VAL:
			if (slsi_util_nla_get_u32(iter, &(hal_req->channel_24g_val)))
				return -EINVAL;
			hal_req->config_24g_channel = 1;
			break;

		case NAN_REQ_ATTR_CHANNEL_5G_MHZ_VAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->channel_5g_val = (int)val;
			hal_req->config_5g_channel = 1;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SID_BEACON_VAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->subscribe_sid_beacon_val = (u32)val;
			hal_req->config_subscribe_sid_beacon = 1;
			break;

		case NAN_REQ_ATTR_DW_2G4_INTERVAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->dw_2dot4g_interval_val = (u32)val;
			/* valid range for 2.4G is 1-5 */
			if (hal_req->dw_2dot4g_interval_val > 0  && hal_req->dw_2dot4g_interval_val < 5)
				hal_req->config_2dot4g_dw_band = 1;
			break;

		case NAN_REQ_ATTR_DW_5G_INTERVAL:
			if (slsi_util_nla_get_u8(iter, &val))
				 return -EINVAL;
			hal_req->dw_5g_interval_val = (u32)val;
			/* valid range for 5g is 0-5 */
			if (hal_req->dw_5g_interval_val < 5)
				hal_req->config_5g_dw_band = 1;
			break;

		case NAN_REQ_ATTR_DISC_MAC_ADDR_RANDOM_INTERVAL:
			if (slsi_util_nla_get_u32(iter, &(hal_req->disc_mac_addr_rand_interval_sec)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			SLSI_ERR(sdev, "Unexpected NAN enable attribute TYPE:%d\n", type);
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_enable(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct slsi_hal_nan_enable_req hal_req;
	int ret;
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	u8 nan_vif_mac_address[ETH_ALEN];
	u8 broadcast_mac[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	hal_req.transaction_id = 0;
	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);

	reply_status = slsi_nan_enable_get_nl_params(sdev, &hal_req, data, len);
	if (reply_status != SLSI_HAL_NAN_STATUS_SUCCESS) {
		ret = -EINVAL;
		goto exit;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (ndev_vif->activated) {
		ret = -EINVAL;
		SLSI_DBG1(sdev, SLSI_GSCAN, "Already Enabled. Req Rejected\n");
		goto exit_with_mutex;
	}
	ndev_vif->vif_type = FAPI_VIFTYPE_NAN;

	slsi_net_randomize_nmi_ndi(sdev);

	if (hal_req.config_intf_addr)
		ether_addr_copy(nan_vif_mac_address, hal_req.intf_addr_val);
	else
		slsi_nan_get_mac(sdev, nan_vif_mac_address);

	ret = slsi_mlme_add_vif(sdev, dev, nan_vif_mac_address, broadcast_mac);
	if (ret) {
		reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		SLSI_ERR(sdev, "failed to set unsync vif. Cannot start NAN\n");
	} else {
		ret = slsi_mlme_nan_enable(sdev, dev, &hal_req);
		if (ret) {
			SLSI_ERR(sdev, "failed to enable NAN.\n");
			reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
			if (slsi_mlme_del_vif(sdev, dev) != 0)
				SLSI_NET_ERR(dev, "slsi_mlme_del_vif failed\n");
			ndev_vif->activated = false;
			ndev_vif->nan.service_id_map = 0;
		} else {
			slsi_vif_activated(sdev, dev);
			ndev_vif->nan.master_pref_value = hal_req.master_pref;
			ether_addr_copy(ndev_vif->nan.local_nmi, nan_vif_mac_address);
			ndev_vif->nan.state = 1;
			if (hal_req.config_24g_channel)
				ndev_vif->nan.operating_channel[0] = hal_req.channel_24g_val;
			if (hal_req.config_5g_channel)
				ndev_vif->nan.operating_channel[1] = hal_req.channel_5g_val;
			if (hal_req.config_hop_count_limit)
				ndev_vif->nan.hopcount = hal_req.hop_count_limit_val;
			slsi_eth_zero_addr(ndev_vif->nan.cluster_id);
			ndev_vif->nan.random_mac_interval_sec = hal_req.disc_mac_addr_rand_interval_sec;
			SLSI_INFO(sdev,
				  "trans_id:%d master_pref:%d 2gChan:%d 5gChan:%d mac_random_interval:%d\n",
				  hal_req.transaction_id, hal_req.master_pref, hal_req.channel_24g_val,
				  hal_req.channel_5g_val, hal_req.disc_mac_addr_rand_interval_sec);
		}
	}
	ether_addr_copy(ndev_vif->nan.local_nmi, nan_vif_mac_address);

exit_with_mutex:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_ENABLED, 0, NULL, hal_req.transaction_id);
	return ret;
}

int slsi_nan_disable(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct net_device *data_dev;
	struct netdev_vif *ndev_vif, *data_ndev_vif;
	u8 disable_cluster_merge, i;
	int type, tmp;
	const struct nlattr *iter;
	u16 transaction_id = 0;

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(transaction_id)))
				return -EINVAL;
			break;
		default:
			break;
		}
	}

	SLSI_INFO(sdev, "transaction_id:%d\n", transaction_id);
	if (dev) {
		ndev_vif = netdev_priv(dev);
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		if (ndev_vif->activated) {
			if (slsi_mlme_del_vif(sdev, dev) != 0)
				SLSI_NET_ERR(dev, "slsi_mlme_del_vif failed\n");
			ndev_vif->activated = false;
		} else {
			SLSI_WARN(sdev, "NAN FWif not active!!");
		}
		for (i = SLSI_NAN_DATA_IFINDEX_START; i < CONFIG_SCSC_WLAN_MAX_INTERFACES + 1; i++) {
			data_dev = slsi_get_netdev(sdev, i);
			if (data_dev) {
				data_ndev_vif = netdev_priv(data_dev);
				SLSI_MUTEX_LOCK(data_ndev_vif->vif_mutex);
				slsi_vif_cleanup(sdev, data_dev, true);
				SLSI_MUTEX_UNLOCK(data_ndev_vif->vif_mutex);
			}
		}
		disable_cluster_merge = ndev_vif->nan.disable_cluster_merge;
		memset(&ndev_vif->nan, 0, sizeof(ndev_vif->nan));
		ndev_vif->nan.disable_cluster_merge = disable_cluster_merge;
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	} else {
		SLSI_WARN(sdev, "No NAN interface!!");
	}

	slsi_vendor_nan_command_reply(wiphy, SLSI_HAL_NAN_STATUS_SUCCESS, 0, NAN_RESPONSE_DISABLED, 0, NULL, transaction_id);

	return 0;
}

static int slsi_nan_publish_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_publish_req *hal_req,
					  const void *data, int len)
{
	int type, tmp, r;
	const struct nlattr *iter;
	u16 val = 0;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_PUBLISH_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->publish_id)))
				return -EINVAL;
			break;
		case NAN_REQ_ATTR_PUBLISH_TTL:
			if (slsi_util_nla_get_u16(iter, &(hal_req->ttl)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_PERIOD:
			if (slsi_util_nla_get_u16(iter, &(hal_req->period)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_TYPE:
			if (slsi_util_nla_get_u16(iter, &val))
				return -EINVAL;
			hal_req->publish_type = (u8)val;
			break;

		case NAN_REQ_ATTR_PUBLISH_TX_TYPE:
			if (slsi_util_nla_get_u16(iter, &val))
				return -EINVAL;
			hal_req->tx_type = (u8)val;
			break;

		case NAN_REQ_ATTR_PUBLISH_COUNT:
			if (slsi_util_nla_get_u8(iter, &(hal_req->publish_count)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SERVICE_NAME_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->service_name_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SERVICE_NAME:
			if (slsi_util_nla_get_data(iter, hal_req->service_name_len, hal_req->service_name))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_MATCH_ALGO:
			if (slsi_util_nla_get_u8(iter, &(hal_req->publish_match_indicator)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SERVICE_INFO_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SERVICE_INFO:
			if (slsi_util_nla_get_data(iter, hal_req->service_specific_info_len, hal_req->service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_RX_MATCH_FILTER_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->rx_match_filter_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_RX_MATCH_FILTER:
			if (slsi_util_nla_get_data(iter,  hal_req->rx_match_filter_len, hal_req->rx_match_filter))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_TX_MATCH_FILTER_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->tx_match_filter_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_TX_MATCH_FILTER:
			if (slsi_util_nla_get_data(iter, hal_req->tx_match_filter_len, hal_req->tx_match_filter))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_RSSI_THRESHOLD_FLAG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_threshold_flag)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_CONN_MAP:
			if (slsi_util_nla_get_u8(iter, &(hal_req->connmap)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_RECV_IND_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->recv_indication_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->sdea_service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA:
			if (slsi_util_nla_get_data(iter, hal_req->sdea_service_specific_info_len, hal_req->sdea_service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_RANGING_AUTO_RESPONSE:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ranging_auto_response)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			r = slsi_nan_get_sdea_params_nl(sdev, &hal_req->sdea_params, iter, type);
			if (r)
				r = slsi_nan_get_ranging_cfg_nl(sdev, &hal_req->ranging_cfg, iter, type);
			if (r)
				r = slsi_nan_get_security_info_nl(sdev, &hal_req->sec_info, iter, type);
			if (r)
				r = slsi_nan_get_range_resp_cfg_nl(sdev, &hal_req->range_response_cfg, iter, type);
			if (r) {
				SLSI_ERR(sdev, "Unexpected NAN publish attribute TYPE:%d\n", type);
				return SLSI_HAL_NAN_STATUS_INVALID_PARAM;
			}
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_publish(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct slsi_hal_nan_publish_req *hal_req;
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	int ret;
	u32 reply_status;
	u32 publish_id = 0;
	u16 transaction_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	hal_req = kmalloc(sizeof(*hal_req), GFP_KERNEL);
	if (!hal_req) {
		SLSI_ERR(sdev, "failed to alloc hal_req\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = -ENOMEM;
		goto exit;
	}

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_publish_get_nl_params(sdev, hal_req, data, len);
	if (reply_status != SLSI_HAL_NAN_STATUS_SUCCESS) {
		kfree(hal_req);
		ret = -EINVAL;
		goto exit;
	}
	transaction_id = hal_req->transaction_id;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}

	if (!hal_req->publish_id) {
		hal_req->publish_id = slsi_nan_get_new_publish_id(ndev_vif);
	} else if (!slsi_nan_is_publish_id_active(ndev_vif, hal_req->publish_id)) {
		SLSI_WARN(sdev, "Publish id %d not found. map:%x\n", hal_req->publish_id,
			  ndev_vif->nan.service_id_map);
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		ret = -EINVAL;
		goto exit_with_lock;
	}

	if (hal_req->publish_id) {
		ret = slsi_mlme_nan_publish(sdev, dev, hal_req, hal_req->publish_id);
		if (ret) {
			reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		} else {
			publish_id = hal_req->publish_id;
			SLSI_INFO(sdev,
				  "trans_id:%d, publish_id:%d type:%d rec_ind_cfg:0x%x\n",
				  hal_req->transaction_id, hal_req->publish_id, hal_req->publish_type,
				  hal_req->recv_indication_cfg);
		}
	} else {
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		SLSI_WARN(sdev, "Too Many concurrent PUBLISH REQ(map:%x)\n",
			  ndev_vif->nan.service_id_map);
		ret = -ENOTSUPP;
	}
exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(hal_req);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_PUBLISH, publish_id, NULL, transaction_id);
	return ret;
}

int slsi_nan_publish_cancel(struct wiphy *wiphy, struct wireless_dev *wdev,
			    const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	int type, tmp, ret = 0;
	u16 publish_id = 0, transaction_id = 0;
	const struct nlattr *iter;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_PUBLISH_ID:
			if (slsi_util_nla_get_u16(iter, &(publish_id)))
				return -EINVAL;
			break;
		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(transaction_id)))
				return -EINVAL;
			break;

		default:
			SLSI_ERR(sdev, "Unexpected NAN publishcancel attribute TYPE:%d\n", type);
		}
	}
	SLSI_INFO(sdev, "transaction_id:%d publish_id:%d\n", transaction_id, publish_id);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}
	if (!publish_id || !slsi_nan_is_publish_id_active(ndev_vif, publish_id)) {
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		SLSI_WARN(sdev, "Publish_id(%d) not active. map:%x\n",
			  publish_id, ndev_vif->nan.service_id_map);
	} else {
		ret = slsi_mlme_nan_publish(sdev, dev, NULL, publish_id);
		if (ret)
			reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
	}
exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_PUBLISH_CANCEL, publish_id, NULL, transaction_id);
	return ret;
}

static int slsi_nan_subscribe_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_subscribe_req *hal_req,
					    const void *data, int len)
{
	int type, tmp, r;
	const struct nlattr *iter;
	u16 val = 0;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_SUBSCRIBE_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->subscribe_id)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_TTL:
			if (slsi_util_nla_get_u16(iter, &(hal_req->ttl)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_PERIOD:
			if (slsi_util_nla_get_u16(iter, &(hal_req->period)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_TYPE:
			if (slsi_util_nla_get_u8(iter, &(hal_req->subscribe_type)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RESP_FILTER_TYPE:
			if (slsi_util_nla_get_u16(iter, &val))
				return -EINVAL;
			hal_req->service_response_filter = (u8)val;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RESP_INCLUDE:
			if (slsi_util_nla_get_u8(iter, &(hal_req->service_response_include)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_USE_RESP_FILTER:
			if (slsi_util_nla_get_u8(iter, &(hal_req->use_service_response_filter)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SSI_REQUIRED:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ssi_required_for_match_indication)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_MATCH_INDICATOR:
			if (slsi_util_nla_get_u8(iter, &(hal_req->subscribe_match_indicator)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_COUNT:
			if (slsi_util_nla_get_u8(iter, &(hal_req->subscribe_count)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SERVICE_NAME_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->service_name_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SERVICE_NAME:
			if (slsi_util_nla_get_data(iter, hal_req->service_name_len, hal_req->service_name))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SERVICE_INFO_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SERVICE_INFO:
			if (slsi_util_nla_get_data(iter, hal_req->service_specific_info_len, hal_req->service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RX_MATCH_FILTER_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->rx_match_filter_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RX_MATCH_FILTER:
			if (slsi_util_nla_get_data(iter, hal_req->rx_match_filter_len, hal_req->rx_match_filter))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_TX_MATCH_FILTER_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->tx_match_filter_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_TX_MATCH_FILTER:
			if (slsi_util_nla_get_data(iter, hal_req->tx_match_filter_len, hal_req->tx_match_filter))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RSSI_THRESHOLD_FLAG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_threshold_flag)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_CONN_MAP:
			if (slsi_util_nla_get_u8(iter, &(hal_req->connmap)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_NUM_INTF_ADDR_PRESENT:
			if (slsi_util_nla_get_u8(iter, &(hal_req->num_intf_addr_present)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_INTF_ADDR:
			if (slsi_util_nla_get_data(iter, (hal_req->num_intf_addr_present * ETH_ALEN), hal_req->intf_addr))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_RECV_IND_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->recv_indication_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->sdea_service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA:
			if (slsi_util_nla_get_data(iter, hal_req->sdea_service_specific_info_len,
				hal_req->sdea_service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_RANGING_AUTO_RESPONSE:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ranging_auto_response)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			r = slsi_nan_get_sdea_params_nl(sdev, &hal_req->sdea_params, iter, type);
			if (r)
				r = slsi_nan_get_ranging_cfg_nl(sdev, &hal_req->ranging_cfg, iter, type);
			if (r)
				r = slsi_nan_get_security_info_nl(sdev, &hal_req->sec_info, iter, type);
			if (r)
				r = slsi_nan_get_range_resp_cfg_nl(sdev, &hal_req->range_response_cfg, iter, type);
			if (r) {
				SLSI_ERR(sdev, "Unexpected NAN subscribe attribute TYPE:%d\n", type);
				return SLSI_HAL_NAN_STATUS_INVALID_PARAM;
			}
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_subscribe(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	struct slsi_hal_nan_subscribe_req *hal_req;
	int ret;
	u32 reply_status;
	u32 subscribe_id = 0;
	u16 transaction_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	hal_req = kmalloc(sizeof(*hal_req), GFP_KERNEL);
	if (!hal_req) {
		SLSI_ERR(sdev, "Failed to alloc hal_req structure!!!\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = -ENOMEM;
		goto exit;
	}

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_subscribe_get_nl_params(sdev, hal_req, data, len);
	if (reply_status != SLSI_HAL_NAN_STATUS_SUCCESS) {
		kfree(hal_req);
		ret = -EINVAL;
		goto exit;
	}
	transaction_id = hal_req->transaction_id;
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}

	if (!hal_req->subscribe_id) {
		hal_req->subscribe_id = slsi_nan_get_new_subscribe_id(ndev_vif);
	} else if (!slsi_nan_is_subscribe_id_active(ndev_vif, hal_req->subscribe_id)) {
		SLSI_WARN(sdev, "Subscribe id %d not found. map:%x\n", hal_req->subscribe_id,
			  ndev_vif->nan.service_id_map);
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		ret = -EINVAL;
		goto exit_with_lock;
	}

	ret = slsi_mlme_nan_subscribe(sdev, dev, hal_req, hal_req->subscribe_id);
	if (ret) {
		reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
	} else {
		SLSI_INFO(sdev, "trans_id:%d subscribe_id:%d type:%d\n",
			  hal_req->transaction_id, hal_req->subscribe_id, hal_req->subscribe_type);
		subscribe_id = hal_req->subscribe_id;
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(hal_req);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_SUBSCRIBE, subscribe_id, NULL, transaction_id);
	return ret;
}

int slsi_nan_subscribe_cancel(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	int type, tmp, ret = WIFI_HAL_ERROR_UNKNOWN;
	u16 subscribe_id = 0, transaction_id = 0;
	const struct nlattr *iter;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_SUBSCRIBE_ID:
			if (slsi_util_nla_get_u16(iter, &(subscribe_id))) {
				reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
				goto exit;
			}
			break;
		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(transaction_id))) {
				reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
				goto exit;
			}
			break;
		default:
			SLSI_ERR(sdev, "Unexpected NAN subscribecancel attribute TYPE:%d\n", type);
			reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
			goto exit;
		}
	}
	SLSI_INFO(sdev, "trans_id:%d subscribe_id:%d\n", transaction_id, subscribe_id);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (ndev_vif->activated) {
		if (!subscribe_id || !slsi_nan_is_subscribe_id_active(ndev_vif, subscribe_id)) {
			SLSI_WARN(sdev, "subscribe_id(%d) not active. map:%x\n",
				  subscribe_id, ndev_vif->nan.service_id_map);
			reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		} else {
			ret = slsi_mlme_nan_subscribe(sdev, dev, NULL, subscribe_id);
			if (ret)
				reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		}
	} else {
		SLSI_ERR(sdev, "vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_SUBSCRIBE_CANCEL, subscribe_id, NULL, transaction_id);
	return ret;
}

static int slsi_nan_followup_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_transmit_followup_req *hal_req,
					   const void *data, int len)
{
	int type, tmp;
	const struct nlattr *iter;
	u16 val = 0;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_FOLLOWUP_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->publish_subscribe_id)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_REQUESTOR_ID:
			if (slsi_util_nla_get_u16(iter, &val))
				return -EINVAL;
			hal_req->requestor_instance_id = (u32)val;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_ADDR:
			if (slsi_util_nla_get_data(iter, ETH_ALEN, hal_req->addr))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_PRIORITY:
			if (slsi_util_nla_get_u8(iter, &(hal_req->priority)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_TX_WINDOW:
			if (slsi_util_nla_get_u8(iter, &(hal_req->dw_or_faw)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_SERVICE_NAME_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_SERVICE_NAME:
			if (slsi_util_nla_get_data(iter, hal_req->service_specific_info_len, hal_req->service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_FOLLOWUP_RECV_IND_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->recv_indication_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->sdea_service_specific_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_PUBLISH_SDEA:
			if (slsi_util_nla_get_data(iter, hal_req->sdea_service_specific_info_len,
				hal_req->sdea_service_specific_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			SLSI_ERR(sdev, "Unexpected NAN followup attribute TYPE:%d\n", type);
			return SLSI_HAL_NAN_STATUS_INVALID_PARAM;
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_transmit_followup(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	struct slsi_hal_nan_transmit_followup_req hal_req;
	int ret = 0;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	hal_req.transaction_id = 0;
	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_followup_get_nl_params(sdev, &hal_req, data, len);
	if (reply_status) {
		ret = -EINVAL;
		goto exit;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}

	if (!hal_req.publish_subscribe_id ||
	    !(slsi_nan_is_subscribe_id_active(ndev_vif, hal_req.publish_subscribe_id) ||
	    slsi_nan_is_publish_id_active(ndev_vif, hal_req.publish_subscribe_id))) {
		SLSI_WARN(sdev, "publish/Subscribe id %d not found. map:%x\n", hal_req.publish_subscribe_id,
			  ndev_vif->nan.service_id_map);
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PUBLISH_SUBSCRIBE_ID;
		ret = -EINVAL;
		goto exit_with_lock;
	}

	ret = slsi_mlme_nan_tx_followup(sdev, dev, &hal_req);
	if (ret) {
		if (ret == SLSI_HAL_NAN_STATUS_FOLLOWUP_QUEUE_FULL) {
			reply_status = ret;
			ret = 0;
		} else {
			reply_status = ret == SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		}
	} else {
		SLSI_INFO(sdev,
			  "transId:%d serviceId:%d instanceId:%d\n",
			  hal_req.transaction_id, hal_req.publish_subscribe_id, hal_req.requestor_instance_id);
		ndev_vif->nan.followup_trans_id = hal_req.transaction_id;
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_TRANSMIT_FOLLOWUP, 0, NULL, hal_req.transaction_id);
	return ret;
}

static int slsi_nan_config_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_config_req *hal_req,
					 const void *data, int len)
{
	int type, type1, tmp, tmp1, disc_attr_idx = 0, famchan_idx = 0;
	const struct nlattr *iter, *iter1;
	struct slsi_hal_nan_post_discovery_param *disc_attr;
	struct slsi_hal_nan_further_availability_channel *famchan;
	u8 val = 0;

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_SID_BEACON_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->sid_beacon)))
				return -EINVAL;
			hal_req->config_sid_beacon = 1;
			break;

		case NAN_REQ_ATTR_RSSI_PROXIMITY_2G4_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_proximity)))
				return -EINVAL;
			hal_req->config_rssi_proximity = 1;
			break;

		case NAN_REQ_ATTR_MASTER_PREF:
			if (slsi_util_nla_get_u8(iter, &(hal_req->master_pref)))
				return -EINVAL;
			hal_req->config_master_pref = 1;
			break;

		case NAN_REQ_ATTR_RSSI_CLOSE_PROXIMITY_5G_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->rssi_close_proximity_5g_val)))
				return -EINVAL;
			hal_req->config_5g_rssi_close_proximity = 1;
			break;

		case NAN_REQ_ATTR_RSSI_WINDOW_SIZE_VAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->rssi_window_size_val = (u16)val;
			hal_req->config_rssi_window_size = 1;
			break;

		case NAN_REQ_ATTR_CLUSTER_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->config_cluster_attribute_val)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SOCIAL_CH_SCAN_DWELL_TIME:
			if (slsi_util_nla_get_data(iter, sizeof(hal_req->scan_params_val.dwell_time),
				hal_req->scan_params_val.dwell_time))
				return -EINVAL;
			hal_req->config_scan_params = 1;
			break;

		case NAN_REQ_ATTR_SOCIAL_CH_SCAN_PERIOD:
			if (slsi_util_nla_get_data(iter, sizeof(hal_req->scan_params_val.scan_period),
				hal_req->scan_params_val.scan_period))
				return -EINVAL;
			hal_req->config_scan_params = 1;
			break;

		case NAN_REQ_ATTR_RANDOM_FACTOR_FORCE_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->random_factor_force_val)))
				return -EINVAL;
			hal_req->config_random_factor_force = 1;
			break;

		case NAN_REQ_ATTR_HOP_COUNT_FORCE_VAL:
			if (slsi_util_nla_get_u8(iter, &(hal_req->hop_count_force_val)))
				return -EINVAL;
			hal_req->config_hop_count_force = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_PAYLOAD_TX:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.payload_transmit_flag)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_WFD:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.is_wfd_supported)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_WFDS:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.is_wfds_supported)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_TDLS:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.is_tdls_supported)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_MESH:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.is_mesh_supported)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_IBSS:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.is_ibss_supported)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_CONN_CAPABILITY_WLAN_INFRA:
			if (slsi_util_nla_get_u8(iter, &(hal_req->conn_capability_val.wlan_infra_field)))
				return -EINVAL;
			hal_req->config_conn_capability = 1;
			break;

		case NAN_REQ_ATTR_DISCOVERY_ATTR_NUM_ENTRIES:
			if (slsi_util_nla_get_u8(iter, &(hal_req->num_config_discovery_attr)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_DISCOVERY_ATTR_VAL:
			if (disc_attr_idx >= hal_req->num_config_discovery_attr) {
				SLSI_ERR(sdev,
					 "disc attr(%d) > num disc attr(%d)\n",
					 disc_attr_idx + 1, hal_req->num_config_discovery_attr);
				return -EINVAL;
			}
			disc_attr = &hal_req->discovery_attr_val[disc_attr_idx];
			disc_attr_idx++;
			nla_for_each_nested(iter1, iter, tmp1) {
				type1 = nla_type(iter1);
				switch (type1) {
				case NAN_REQ_ATTR_CONN_TYPE:
					if (slsi_util_nla_get_u8(iter1, &(disc_attr->type)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_NAN_ROLE:
					if (slsi_util_nla_get_u8(iter1, &(disc_attr->role)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_TRANSMIT_FREQ:
					if (slsi_util_nla_get_u8(iter1, &(disc_attr->transmit_freq)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_AVAILABILITY_DURATION:
					if (slsi_util_nla_get_u8(iter1, &(disc_attr->duration)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_AVAILABILITY_INTERVAL:
					if (slsi_util_nla_get_u32(iter1, &(disc_attr->avail_interval_bitmap)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_MAC_ADDR_VAL:
					if (slsi_util_nla_get_data(iter1, ETH_ALEN, disc_attr->addr))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_MESH_ID_LEN:
					if (slsi_util_nla_get_u16(iter1, &(disc_attr->mesh_id_len)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_MESH_ID:
					if (slsi_util_nla_get_data(iter1, disc_attr->mesh_id_len, disc_attr->mesh_id))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_INFRASTRUCTURE_SSID_LEN:
					if (slsi_util_nla_get_u16(iter1, &(disc_attr->infrastructure_ssid_len)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_INFRASTRUCTURE_SSID:
					if (slsi_util_nla_get_data(iter1, disc_attr->infrastructure_ssid_len,
						disc_attr->infrastructure_ssid_val))
						return -EINVAL;
					break;
				}
			}
			break;

		case NAN_REQ_ATTR_FURTHER_AVAIL_NUM_ENTRIES:
			if (slsi_util_nla_get_u8(iter, &(hal_req->fam_val.numchans)))
				return -EINVAL;
			hal_req->config_fam = 1;
			break;

		case NAN_REQ_ATTR_FURTHER_AVAIL_VAL:
			hal_req->config_fam = 1;
			if (famchan_idx >= hal_req->fam_val.numchans) {
				SLSI_ERR(sdev,
					 "famchan attr(%d) > numchans(%d)\n",
					 famchan_idx + 1, hal_req->fam_val.numchans);
				return -EINVAL;
			}
			famchan = &hal_req->fam_val.famchan[famchan_idx];
			famchan_idx++;
			nla_for_each_nested(iter1, iter, tmp1) {
				type1 = nla_type(iter1);
				switch (type1) {
				case NAN_REQ_ATTR_FURTHER_AVAIL_ENTRY_CTRL:
					if (slsi_util_nla_get_u8(iter1, &famchan->entry_control))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_FURTHER_AVAIL_CHAN_CLASS:
					if (slsi_util_nla_get_u8(iter1, &(famchan->class_val)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_FURTHER_AVAIL_CHAN:
					if (slsi_util_nla_get_u8(iter1, &(famchan->channel)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_FURTHER_AVAIL_CHAN_MAPID:
					if (slsi_util_nla_get_u8(iter1, &(famchan->mapid)))
						return -EINVAL;
					break;

				case NAN_REQ_ATTR_FURTHER_AVAIL_INTERVAL_BITMAP:
					if (slsi_util_nla_get_u32(iter1, &(famchan->avail_interval_bitmap)))
						return -EINVAL;
					break;
				}
			}
			break;

		case NAN_REQ_ATTR_SUBSCRIBE_SID_BEACON_VAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->subscribe_sid_beacon_val = (u32)val;
			hal_req->config_subscribe_sid_beacon = 1;
			break;

		case NAN_REQ_ATTR_DW_2G4_INTERVAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->dw_2dot4g_interval_val = (u32)val;
			/* valid range for 2.4G is 1-5 */
			if (hal_req->dw_2dot4g_interval_val > 0  && hal_req->dw_2dot4g_interval_val < 6)
				hal_req->config_2dot4g_dw_band = 1;
			break;

		case NAN_REQ_ATTR_DW_5G_INTERVAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->dw_5g_interval_val = (u32)val;
			/* valid range for 5g is 0-5 */
			if (hal_req->dw_5g_interval_val < 6)
				hal_req->config_5g_dw_band = 1;
			break;

		case NAN_REQ_ATTR_DISC_MAC_ADDR_RANDOM_INTERVAL:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->disc_mac_addr_rand_interval_sec = (u32)val;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			SLSI_ERR(sdev, "Unexpected NAN config attribute TYPE:%d\n", type);
			return SLSI_HAL_NAN_STATUS_INVALID_PARAM;
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_set_config(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	u16 transaction_id = 0;
	int ret;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	reply_status = slsi_nan_config_get_nl_params(sdev, &ndev_vif->nan.config, data, len);
	if (reply_status) {
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		ret = -EINVAL;
		goto exit;
	}
	transaction_id = ndev_vif->nan.config.transaction_id;
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
	} else {
		ret = slsi_mlme_nan_set_config(sdev, dev, &ndev_vif->nan.config);
		if (ret) {
			reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		} else {
			if (ndev_vif->nan.config.config_master_pref)
				ndev_vif->nan.master_pref_value = ndev_vif->nan.config.master_pref;
			ndev_vif->nan.random_mac_interval_sec = ndev_vif->nan.config.disc_mac_addr_rand_interval_sec;
			SLSI_INFO(sdev, "transId:%d masterPref:%d\n",
				  ndev_vif->nan.config.transaction_id, ndev_vif->nan.config.master_pref);
		}
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_CONFIG, 0, NULL, transaction_id);
	return ret;
}

int slsi_nan_get_capabilities(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;
	struct slsi_hal_nan_capabilities nan_capabilities;
	int ret = 0, i;
	struct slsi_mib_value *values = NULL;
	struct slsi_mib_data mibrsp = { 0, NULL };
	struct slsi_mib_get_entry get_values[] = {{ SLSI_PSID_UNIFI_NAN_MAX_CONCURRENT_CLUSTERS, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_CONCURRENT_PUBLISHES, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_CONCURRENT_SUBSCRIBES, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_SERVICE_NAME_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_MATCH_FILTER_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_TOTAL_MATCH_FILTER_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_SERVICE_SPECIFIC_INFO_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_VSA_DATA_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_MESH_DATA_LENGTH, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_NDI_INTERFACES, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_NDP_SESSIONS, { 0, 0 } },
						  { SLSI_PSID_UNIFI_NAN_MAX_APP_INFO_LENGTH, { 0, 0 } } };
	u32 *capabilities_mib_val[] = { &nan_capabilities.max_concurrent_nan_clusters,
					&nan_capabilities.max_publishes,
					&nan_capabilities.max_subscribes,
					&nan_capabilities.max_service_name_len,
					&nan_capabilities.max_match_filter_len,
					&nan_capabilities.max_total_match_filter_len,
					&nan_capabilities.max_service_specific_info_len,
					&nan_capabilities.max_vsa_data_len,
					&nan_capabilities.max_mesh_data_len,
					&nan_capabilities.max_ndi_interfaces,
					&nan_capabilities.max_ndp_sessions,
					&nan_capabilities.max_app_info_len };
	int type, tmp;
	const struct nlattr *iter;
	u16 transaction_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(transaction_id)))
				return -EINVAL;
			break;
		default:
			break;
		}
	}

	ndev_vif = netdev_priv(dev);

	/* Expect each mib length in response is 11 */
	mibrsp.dataLength = 11 * ARRAY_SIZE(get_values);
	mibrsp.data = kmalloc(mibrsp.dataLength, GFP_KERNEL);
	if (!mibrsp.data) {
		SLSI_ERR(sdev, "Cannot kmalloc %d bytes\n", mibrsp.dataLength);
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = -ENOMEM;
		goto exit;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	values = slsi_read_mibs(sdev, NULL, get_values, ARRAY_SIZE(get_values), &mibrsp);
	if (!values) {
		ret = 0xFFFFFFFF;
		reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		goto exit_with_mibrsp;
	}

	for (i = 0; i < (int)ARRAY_SIZE(get_values); i++) {
		if (values[i].type == SLSI_MIB_TYPE_UINT) {
			*capabilities_mib_val[i] = values[i].u.uintValue;
			SLSI_DBG2(sdev, SLSI_GSCAN, "MIB value = %u\n", *capabilities_mib_val[i]);
		} else {
			SLSI_ERR(sdev, "invalid type(%d). iter:%d\n", values[i].type, i);
			ret = 0xFFFFFFFF;
			reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
			*capabilities_mib_val[i] = 0;
		}
	}

	if (!nan_capabilities.max_ndi_interfaces)
		nan_capabilities.max_ndi_interfaces = slsi_get_nan_max_ndi_ifaces();
	if (!nan_capabilities.max_ndp_sessions)
		nan_capabilities.max_ndi_interfaces = slsi_get_nan_max_ndp_instances();

	if (nan_capabilities.max_ndi_interfaces > SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_ERR(sdev, "max ndp if's:%d but supported:%d\n", nan_capabilities.max_ndi_interfaces,
			 SLSI_NAN_MAX_NDP_INSTANCES);
		nan_capabilities.max_ndi_interfaces = SLSI_NAN_MAX_NDP_INSTANCES;
	}

	if (nan_capabilities.max_ndp_sessions > SLSI_NAN_MAX_NDP_INSTANCES) {
		SLSI_ERR(sdev, "max ndp if's:%d but supported:%d\n", nan_capabilities.max_ndp_sessions,
			 SLSI_NAN_MAX_NDP_INSTANCES);
		nan_capabilities.max_ndi_interfaces = SLSI_NAN_MAX_NDP_INSTANCES;
	}

	SLSI_INFO(sdev, "transId:%d\n", transaction_id);

	kfree(values);
exit_with_mibrsp:
	kfree(mibrsp.data);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_RESPONSE_GET_CAPABILITIES, 0, &nan_capabilities, transaction_id);
	return ret;
}

int slsi_nan_data_iface_create(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	u8 *iface_name = NULL;
	int ret = 0, if_idx, type, tmp, err;
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct net_device *dev_ndp = NULL;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;
	const struct nlattr *iter;
	u16 transaction_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		if (type == NAN_REQ_ATTR_DATA_INTERFACE_NAME) {
			/* 16 is the interface length from net_device
			 * structure.
			 */
			if (nla_len(iter) > IFNAMSIZ)
				return -EINVAL;
			iface_name = nla_data(iter);
		} else if (type == NAN_REQ_ATTR_HAL_TRANSACTION_ID)
			if (slsi_util_nla_get_u16(iter, &transaction_id))
				return -EINVAL;
	}
	if (!iface_name) {
		SLSI_ERR(sdev, "No NAN data interface name\n");
		ret = WIFI_HAL_ERROR_INVALID_ARGS;
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
		goto exit;
	}

	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	/* Find unused netdev idx */
	for (if_idx = SLSI_NAN_DATA_IFINDEX_START; if_idx < CONFIG_SCSC_WLAN_MAX_INTERFACES + 1; if_idx++) {
		dev_ndp = slsi_get_netdev_locked(sdev, if_idx);
		if (!dev_ndp)
			break;
	}

	if (if_idx >= CONFIG_SCSC_WLAN_MAX_INTERFACES + 1) {
		SLSI_ERR(sdev, "NAN no free NAN data interfaces\n");
		ret = WIFI_HAL_ERROR_TOO_MANY_REQUESTS;
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
		goto exit_with_lock;
	}

	err = slsi_netif_add_locked(sdev, iface_name, if_idx);
	if (err) {
		SLSI_ERR(sdev, "NAN fail net_if_add if_name:%s, if_idx:%d\n", iface_name, if_idx);
		ret = WIFI_HAL_ERROR_OUT_OF_MEMORY;
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		goto exit_with_lock;
	}

	dev_ndp = slsi_get_netdev_locked(sdev, if_idx);
	err = slsi_netif_register_locked(sdev, dev_ndp);
	if (err) {
		SLSI_ERR(sdev, "NAN fail netdev err:%d if_name:%s, if_idx:%d\n", err);
		ret = WIFI_HAL_ERROR_UNKNOWN;
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
	} else {
		SLSI_INFO(sdev, "trans_id:%d, if_name:%s, if_idx:%d\n", transaction_id, iface_name, if_idx);
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_DP_INTERFACE_CREATE, 0, NULL, transaction_id);
	return ret;
}

int slsi_nan_data_iface_delete(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	u8 *iface_name = NULL;
	int ret = 0, if_idx, type, tmp;
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct net_device *dev_ndp = NULL;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;
	const struct nlattr *iter;
	u16 transaction_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		if (type == NAN_REQ_ATTR_DATA_INTERFACE_NAME) {
			/* 16 is the interface length from net_device
			 * structure.
			 */
			if (nla_len(iter) > IFNAMSIZ)
				return -EINVAL;
			iface_name = nla_data(iter);
		} else if (type == NAN_REQ_ATTR_HAL_TRANSACTION_ID)
			if (slsi_util_nla_get_u16(iter, &(transaction_id)))
				return -EINVAL;
	}
	if (!iface_name) {
		SLSI_ERR(sdev, "No NAN data interface name\n");
		ret = WIFI_HAL_ERROR_INVALID_ARGS;
		reply_status = SLSI_HAL_NAN_STATUS_INVALID_PARAM;
		goto exit;
	}

	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	for (if_idx = SLSI_NAN_DATA_IFINDEX_START; if_idx < CONFIG_SCSC_WLAN_MAX_INTERFACES + 1; if_idx++) {
		dev_ndp = slsi_get_netdev_locked(sdev, if_idx);
		if (dev_ndp && strcmp(iface_name, dev_ndp->name) == 0)
			break;
		dev_ndp = NULL;
	}

	if (dev_ndp) {
		slsi_netif_remove_locked(sdev, dev_ndp);
		SLSI_INFO(sdev, "Success transId:%d ifaceName:%s\n", transaction_id, iface_name);
	}

	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_DP_INTERFACE_DELETE, 0, NULL, transaction_id);
	return ret;
}

int slsi_nan_ndp_initiate_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_data_path_initiator_req *hal_req,
					const void *data, int len)
{
	int type, tmp, r;
	const struct nlattr *iter;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_REQ_INSTANCE_ID:
			if (slsi_util_nla_get_u32(iter, &(hal_req->requestor_instance_id)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_CHAN_REQ_TYPE:
			if (slsi_util_nla_get_u8(iter, &(hal_req->channel_request_type)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_CHAN:
			if (slsi_util_nla_get_u32(iter, &(hal_req->channel)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_MAC_ADDR_VAL:
			if (nla_len(iter) < ETH_ALEN)
				return -EINVAL;
			ether_addr_copy(hal_req->peer_disc_mac_addr, nla_data(iter));
			break;

		case NAN_REQ_ATTR_DATA_INTERFACE_NAME:
			if (slsi_util_nla_get_data(iter, IFNAMSIZ, hal_req->ndp_iface))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_DATA_INTERFACE_NAME_LEN:
			break;

		case NAN_REQ_ATTR_SDEA_PARAM_SECURITY_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ndp_cfg.security_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SDEA_PARAM_QOS_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ndp_cfg.qos_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_APP_INFO_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->app_info.ndp_app_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_APP_INFO:
			if (slsi_util_nla_get_data(iter, hal_req->app_info.ndp_app_info_len, hal_req->app_info.ndp_app_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SERVICE_NAME_LEN:
			if (slsi_util_nla_get_u32(iter, &(hal_req->service_name_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SERVICE_NAME:
			if (slsi_util_nla_get_data(iter, hal_req->service_name_len, hal_req->service_name))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			r = slsi_nan_get_security_info_nl(sdev, &hal_req->key_info, iter, type);
			if (r)
				SLSI_ERR(sdev, "Unexpected NAN ndp attribute TYPE:%d\n", type);
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_ndp_initiate(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	struct slsi_hal_nan_data_path_initiator_req *hal_req = NULL;
	int ret;
	u32 ndp_id = 0;
	u16 ndl_vif_id, transaction_id = 0;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	hal_req = kmalloc(sizeof(*hal_req), GFP_KERNEL);
	if (!hal_req) {
		SLSI_ERR(sdev, "Failed to alloc hal_req structure!!!\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = WIFI_HAL_ERROR_OUT_OF_MEMORY;
		goto exit;
	}

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_ndp_initiate_get_nl_params(sdev, hal_req, data, len);
	if (reply_status) {
		ret = WIFI_HAL_ERROR_INVALID_ARGS;
		goto exit;
	}
	transaction_id = hal_req->transaction_id;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}

	ndp_id = slsi_nan_get_new_ndp_id(ndev_vif);
	if (!ndp_id) {
		SLSI_WARN(sdev, "NAN no free ndp slots\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = WIFI_HAL_ERROR_TOO_MANY_REQUESTS;
		goto exit_with_lock;
	}

	ndl_vif_id = slsi_nan_ndp_get_ndl_vif_id(hal_req->peer_disc_mac_addr, ndev_vif->nan.ndl_list);
	if (ndl_vif_id >= SLSI_NAN_MAX_NDP_INSTANCES + SLSI_NAN_DATA_IFINDEX_START) {
		SLSI_WARN(sdev, "NAN no free ndl slots\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = WIFI_HAL_ERROR_TOO_MANY_REQUESTS;
		goto exit_with_lock;
	}
	ret = slsi_mlme_ndp_request(sdev, dev, hal_req, ndp_id, ndl_vif_id);
	if (ret) {
		reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		ret = WIFI_HAL_ERROR_UNKNOWN;
	} else {
		SLSI_INFO(sdev, "transId:%d ndpId:%d ndlVifId:%d iface:%s\n",
			  hal_req->transaction_id, ndp_id, ndl_vif_id, hal_req->ndp_iface);
		ret = WIFI_HAL_SUCCESS;
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_DP_INITIATOR_RESPONSE, (u16)ndp_id, NULL, transaction_id);
	kfree(hal_req);
	return ret;
}

int slsi_nan_ndp_respond_get_nl_param(struct slsi_dev *sdev, struct slsi_hal_nan_data_path_indication_response *hal_req,
				      const void *data, int len)
{
	int type, tmp, r;
	const struct nlattr *iter;
	u8 val = 0;
	u32 value = 0;

	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		switch (type) {
		case NAN_REQ_ATTR_NDP_INSTANCE_ID:
			if (slsi_util_nla_get_u32(iter, &(hal_req->ndp_instance_id)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_DATA_INTERFACE_NAME:
			if (slsi_util_nla_get_data(iter, IFNAMSIZ, hal_req->ndp_iface))
				return -EINVAL;
			break;
		case NAN_REQ_ATTR_DATA_INTERFACE_NAME_LEN:
			break;

		case NAN_REQ_ATTR_SDEA_PARAM_SECURITY_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ndp_cfg.security_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_SDEA_PARAM_QOS_CFG:
			if (slsi_util_nla_get_u8(iter, &(hal_req->ndp_cfg.qos_cfg)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_APP_INFO_LEN:
			if (slsi_util_nla_get_u16(iter, &(hal_req->app_info.ndp_app_info_len)))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_APP_INFO:
			if (slsi_util_nla_get_data(iter, hal_req->app_info.ndp_app_info_len, hal_req->app_info.ndp_app_info))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_NDP_RESPONSE_CODE:
			if (slsi_util_nla_get_u8(iter, &val))
				return -EINVAL;
			hal_req->rsp_code = (u32)val;
			break;

		case NAN_REQ_ATTR_SERVICE_NAME_LEN:
			if (slsi_util_nla_get_u32(iter, &value))
				return -EINVAL;
			hal_req->service_name_len = (u8)value;
			break;

		case NAN_REQ_ATTR_SERVICE_NAME:
			if (slsi_util_nla_get_data(iter, hal_req->service_name_len, hal_req->service_name))
				return -EINVAL;
			break;

		case NAN_REQ_ATTR_HAL_TRANSACTION_ID:
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
			break;

		default:
			r = 0;
			r = slsi_nan_get_security_info_nl(sdev, &hal_req->key_info, iter, type);
			if (r)
				SLSI_ERR(sdev, "Unexpected NAN ndp attribute TYPE:%d\n", type);
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_ndp_respond(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	struct slsi_hal_nan_data_path_indication_response *hal_req = NULL;
	int ret;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;
	u16 transaction_id = 0, ndp_instance_id = 0;

	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	hal_req = kmalloc(sizeof(*hal_req), GFP_KERNEL);
	if (!hal_req) {
		SLSI_ERR(sdev, "Failed to alloc hal_req structure!!!\n");
		reply_status = SLSI_HAL_NAN_STATUS_NO_RESOURCE_AVAILABLE;
		ret = WIFI_HAL_ERROR_OUT_OF_MEMORY;
		goto exit;
	}

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_ndp_respond_get_nl_param(sdev, hal_req, data, len);
	transaction_id = hal_req->transaction_id;
	ndp_instance_id = hal_req->ndp_instance_id;
	if (reply_status) {
		ret = WIFI_HAL_ERROR_INVALID_ARGS;
		goto exit;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		reply_status = SLSI_HAL_NAN_STATUS_NAN_NOT_ALLOWED;
		ret = WIFI_HAL_ERROR_NOT_AVAILABLE;
		goto exit_with_lock;
	}

	ret = slsi_mlme_ndp_response(sdev, dev, hal_req, ndev_vif->nan.ndp_local_ndp_id[hal_req->ndp_instance_id - 1]);
	if (ret) {
		reply_status = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		ret = WIFI_HAL_ERROR_UNKNOWN;
	} else {
		SLSI_INFO(sdev, "transId:%d ndpId:%d iface:%s\n",
			  hal_req->transaction_id, hal_req->ndp_instance_id, hal_req->ndp_iface);
		ret = WIFI_HAL_SUCCESS;
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_DP_RESPONDER_RESPONSE,
				      ndp_instance_id, NULL, transaction_id);
	kfree(hal_req);
	return ret;
}

int slsi_nan_ndp_end_get_nl_params(struct slsi_dev *sdev, struct slsi_hal_nan_data_end *hal_req,
				   const void *data, int len)
{
	int type, tmp;
	const struct nlattr *iter;
	u32 val = 0;
	memset(hal_req, 0, sizeof(*hal_req));
	nla_for_each_attr(iter, data, len, tmp) {
		type = nla_type(iter);
		if (type == NAN_REQ_ATTR_NDP_INSTANCE_ID && hal_req->num_ndp_instances < SLSI_NAN_MAX_NDP_INSTANCES) {
			if (slsi_util_nla_get_u32(iter, &val))
				return -EINVAL;
			hal_req->ndp_instance_id[hal_req->num_ndp_instances++] = val;
		} else if (type == NAN_REQ_ATTR_HAL_TRANSACTION_ID) {
			if (slsi_util_nla_get_u16(iter, &(hal_req->transaction_id)))
				return -EINVAL;
		}
	}
	return SLSI_HAL_NAN_STATUS_SUCCESS;
}

int slsi_nan_ndp_end(struct wiphy *wiphy, struct wireless_dev *wdev, const void *data, int len)
{
	struct slsi_dev *sdev = SDEV_FROM_WIPHY(wiphy);
	struct net_device *dev = slsi_nan_get_netdev(sdev);
	struct netdev_vif *ndev_vif;
	struct slsi_hal_nan_data_end hal_req;
	int ret, i;
	u32 reply_status = SLSI_HAL_NAN_STATUS_SUCCESS;

	hal_req.transaction_id = 0;
	slsi_nan_pre_check(sdev, dev, &ret, &reply_status);
	if (ret != WIFI_HAL_SUCCESS)
		goto exit;

	ndev_vif = netdev_priv(dev);
	reply_status = slsi_nan_ndp_end_get_nl_params(sdev, &hal_req, data, len);

	if (reply_status) {
		ret = WIFI_HAL_ERROR_INVALID_ARGS;
		goto exit;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_WARN(sdev, "NAN vif not activated\n");
		slsi_nan_ndp_termination_handler(sdev, dev, 0, 0, NULL);
		ret = WIFI_HAL_SUCCESS;
		goto exit_with_lock;
	}
	for (i = 0; i < hal_req.num_ndp_instances; i++)
		if (hal_req.ndp_instance_id[i] > 0 && hal_req.ndp_instance_id[i] <= SLSI_NAN_MAX_NDP_INSTANCES) {
			ndev_vif->nan.ndp_state[hal_req.ndp_instance_id[i] - 1] = ndp_slot_status_terminating;
			slsi_mlme_ndp_terminate(sdev, dev, hal_req.ndp_instance_id[i]);
			SLSI_INFO(sdev, "transId:%d ndpId:%d [%d/%d]\n",
				  hal_req.transaction_id, hal_req.ndp_instance_id[i], i, hal_req.num_ndp_instances);
		} else {
			SLSI_ERR(sdev, "Ignore invalid ndp_id:%d\n", hal_req.ndp_instance_id[i]);
		}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
exit:
	slsi_vendor_nan_command_reply(wiphy, reply_status, ret, NAN_DP_END, 0, NULL, hal_req.transaction_id);
	return ret;
}

/* NAN HAL EVENTS */

void slsi_nan_event(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct sk_buff *nl_skb = NULL;
	int res = 0;
	u16 event, identifier, evt_reason;
	u8 *mac_addr;
	u16 hal_event;
	struct netdev_vif *ndev_vif;
	enum slsi_nan_disc_event_type disc_event_type = 0;

	ndev_vif = netdev_priv(dev);
	event = fapi_get_u16(skb, u.mlme_nan_event_ind.event);
	identifier = fapi_get_u16(skb, u.mlme_nan_event_ind.identifier);
	mac_addr = fapi_get_buff(skb, u.mlme_nan_event_ind.address_or_identifier);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	switch (fapi_get_u16(skb, u.mlme_nan_event_ind.reason_code)) {
	case FAPI_REASONCODE_NAN_SERVICE_TERMINATED_TIMEOUT:
	case FAPI_REASONCODE_NAN_SERVICE_TERMINATED_COUNT_REACHED:
	case FAPI_REASONCODE_NAN_SERVICE_TERMINATED_DISCOVERY_SHUTDOWN:
	case FAPI_REASONCODE_NAN_SERVICE_TERMINATED_USER_REQUEST:
	case FAPI_REASONCODE_NAN_TRANSMIT_FOLLOWUP_SUCCESS:
		evt_reason = SLSI_HAL_NAN_STATUS_SUCCESS;
		break;
	case FAPI_REASONCODE_NAN_TRANSMIT_FOLLOWUP_FAILURE:
		evt_reason = SLSI_HAL_NAN_STATUS_PROTOCOL_FAILURE;
		break;
	default:
		evt_reason = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
		break;
	}

	switch (event) {
	case FAPI_EVENT_WIFI_EVENT_NAN_PUBLISH_TERMINATED:
		ndev_vif->nan.service_id_map &= (u32)~BIT(identifier);
		if (ndev_vif->nan.nan_sdf_flags[identifier] & FAPI_NANSDFCONTROL_PUBLISH_END_EVENT)
			goto exit;
		hal_event = SLSI_NL80211_NAN_PUBLISH_TERMINATED_EVENT;
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_MATCH_EXPIRED:
		if (ndev_vif->nan.nan_sdf_flags[identifier] & FAPI_NANSDFCONTROL_MATCH_EXPIRED_EVENT)
			goto exit;
		hal_event = SLSI_NL80211_NAN_MATCH_EXPIRED_EVENT;
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_SUBSCRIBE_TERMINATED:
		ndev_vif->nan.service_id_map &= (u32)~BIT(identifier);
		if (ndev_vif->nan.nan_sdf_flags[identifier] & FAPI_NANSDFCONTROL_SUBSCRIBE_END_EVENT)
			goto exit;
		hal_event = SLSI_NL80211_NAN_SUBSCRIBE_TERMINATED_EVENT;
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_ADDRESS_CHANGED:
		disc_event_type = NAN_EVENT_ID_DISC_MAC_ADDR;
		hal_event = SLSI_NL80211_NAN_DISCOVERY_ENGINE_EVENT;
		ether_addr_copy(ndev_vif->nan.local_nmi, mac_addr);
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_CLUSTER_STARTED:
		disc_event_type = NAN_EVENT_ID_STARTED_CLUSTER;
		hal_event = SLSI_NL80211_NAN_DISCOVERY_ENGINE_EVENT;
		ether_addr_copy(ndev_vif->nan.cluster_id, mac_addr);
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_CLUSTER_JOINED:
		disc_event_type = NAN_EVENT_ID_JOINED_CLUSTER;
		hal_event = SLSI_NL80211_NAN_DISCOVERY_ENGINE_EVENT;
		ether_addr_copy(ndev_vif->nan.cluster_id, mac_addr);
		break;
	case FAPI_EVENT_WIFI_EVENT_NAN_TRANSMIT_FOLLOWUP:
		if (ndev_vif->nan.nan_sdf_flags[identifier] & FAPI_NANSDFCONTROL_FOLLOWUP_TRANSMIT_STATUS)
			goto exit;
		hal_event = SLSI_NL80211_NAN_TRANSMIT_FOLLOWUP_STATUS;
		break;
	default:
		goto exit;
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(hal_event), hal_event);
#endif

#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, hal_event, GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, hal_event, GFP_KERNEL);
#endif
	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		goto exit;
	}

	res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_STATUS, evt_reason);
	switch (hal_event) {
	case SLSI_NL80211_NAN_PUBLISH_TERMINATED_EVENT:
		res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_PUBLISH_ID, identifier);
		break;
	case SLSI_NL80211_NAN_MATCH_EXPIRED_EVENT:
		res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_MATCH_PUBLISH_SUBSCRIBE_ID, identifier);
		break;
	case SLSI_NL80211_NAN_SUBSCRIBE_TERMINATED_EVENT:
		res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_SUBSCRIBE_ID, identifier);
		break;
	case SLSI_NL80211_NAN_DISCOVERY_ENGINE_EVENT:
		res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_DISCOVERY_ENGINE_EVT_TYPE, disc_event_type);
		res |= nla_put(nl_skb, NAN_EVT_ATTR_DISCOVERY_ENGINE_MAC_ADDR, ETH_ALEN, mac_addr);
		break;
	case SLSI_NL80211_NAN_TRANSMIT_FOLLOWUP_STATUS:
		res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_HAL_TRANSACTION_ID, ndev_vif->nan.followup_trans_id);
		ndev_vif->nan.followup_trans_id = 0;
		break;
	}

	if (res) {
		SLSI_ERR(sdev, "Error in nla_put*:%x\n", res);
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		goto exit;
	}

	cfg80211_vendor_event(nl_skb, GFP_KERNEL);

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_nan_followup_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 tag_id, tag_len;
	u8  *ptr;
	struct slsi_hal_nan_followup_ind *hal_evt;
	struct sk_buff *nl_skb;
	int res;
	int sig_data_len;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_DBG3(sdev, SLSI_GSCAN, "\n");

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	sig_data_len = fapi_get_datalen(skb);

	hal_evt = kmalloc(sizeof(*hal_evt), GFP_KERNEL);
	if (!hal_evt) {
		SLSI_ERR(sdev, "No memory for followup_ind\n");
		goto exit;
	}
	memset(hal_evt, 0, sizeof(*hal_evt));

	hal_evt->publish_subscribe_id = fapi_get_u16(skb, u.mlme_nan_followup_ind.session_id);
	hal_evt->requestor_instance_id = fapi_get_u16(skb, u.mlme_nan_followup_ind.match_id);
	ether_addr_copy(hal_evt->addr,
			fapi_get_buff(skb, u.mlme_nan_followup_ind.peer_nan_management_interface_address));

	ptr = fapi_get_data(skb);
	if (ptr) {
		tag_id = le16_to_cpu(*(u16 *)ptr);
		tag_len = le16_to_cpu(*(u16 *)(ptr + 2));

		while (sig_data_len >= tag_len + 4) {
			if (tag_id == SLSI_NAN_TLV_TAG_SERVICE_SPECIFIC_INFO) {
				hal_evt->service_specific_info_len = tag_len > SLSI_HAL_NAN_MAX_SERVICE_SPECIFIC_INFO_LEN ?
							SLSI_HAL_NAN_MAX_SERVICE_SPECIFIC_INFO_LEN : tag_len;
				memcpy(hal_evt->service_specific_info, ptr + 4, hal_evt->service_specific_info_len);
			} else if (tag_id == SLSI_NAN_TLV_TAG_EXT_SERVICE_SPECIFIC_INFO) {
				if (tag_len > SLSI_HAL_NAN_MAX_SDEA_SERVICE_SPEC_INFO_LEN)
					hal_evt->sdea_service_specific_info_len = SLSI_HAL_NAN_MAX_SDEA_SERVICE_SPEC_INFO_LEN;
				else
					hal_evt->sdea_service_specific_info_len = tag_len;
				memcpy(hal_evt->sdea_service_specific_info, ptr + 4, hal_evt->sdea_service_specific_info_len);
			} else {
				SLSI_WARN(sdev, "Skip processing TLV %d\n", tag_id);
			}
			sig_data_len -= tag_len + 4;
			ptr += tag_len + 4;
			if (sig_data_len > 4) {
				tag_id = le16_to_cpu(*(u16 *)ptr);
				tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
			} else {
				tag_id = 0;
				tag_len = 0;
			}
		}
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(SLSI_NL80211_NAN_FOLLOWUP_EVENT), SLSI_NL80211_NAN_FOLLOWUP_EVENT);
#endif
#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, SLSI_NL80211_NAN_FOLLOWUP_EVENT,
					     GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, SLSI_NL80211_NAN_FOLLOWUP_EVENT,
					     GFP_KERNEL);
#endif

	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		kfree(hal_evt);
		goto exit;
	}

	res = nla_put_be16(nl_skb, NAN_EVT_ATTR_FOLLOWUP_PUBLISH_SUBSCRIBE_ID,
			   cpu_to_le16(hal_evt->publish_subscribe_id));
	res |= nla_put_be16(nl_skb, NAN_EVT_ATTR_FOLLOWUP_REQUESTOR_INSTANCE_ID,
			    cpu_to_le16(hal_evt->requestor_instance_id));
	res |= nla_put(nl_skb, NAN_EVT_ATTR_FOLLOWUP_ADDR, ETH_ALEN, hal_evt->addr);
	res |= nla_put_u8(nl_skb, NAN_EVT_ATTR_FOLLOWUP_DW_OR_FAW, hal_evt->dw_or_faw);
	res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_FOLLOWUP_SERVICE_SPECIFIC_INFO_LEN, hal_evt->service_specific_info_len);
	if (hal_evt->service_specific_info_len)
		res |= nla_put(nl_skb, NAN_EVT_ATTR_FOLLOWUP_SERVICE_SPECIFIC_INFO, hal_evt->service_specific_info_len,
			       hal_evt->service_specific_info);
	res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_SDEA_LEN, hal_evt->sdea_service_specific_info_len);
	if (hal_evt->sdea_service_specific_info_len)
		res |= nla_put(nl_skb, NAN_EVT_ATTR_SDEA, hal_evt->sdea_service_specific_info_len,
			       hal_evt->sdea_service_specific_info);

	if (res) {
		SLSI_ERR(sdev, "Error in nla_put*:%x\n", res);
		kfree(hal_evt);
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		goto exit;
	}

	cfg80211_vendor_event(nl_skb, GFP_KERNEL);
	kfree(hal_evt);
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_nan_service_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 tag_id, tag_len;
	u8  *ptr;
	const u8 *tag_data_ptr;
	int sig_data_len;
	struct slsi_hal_nan_match_ind *hal_evt;
	struct sk_buff *nl_skb;
	int res;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_DBG3(sdev, SLSI_GSCAN, "\n");
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	sig_data_len = fapi_get_datalen(skb);
	if (sig_data_len <= 4) {
		SLSI_ERR(sdev, "Invalid data len(%d)\n", sig_data_len);
		goto exit;
	}

	hal_evt = kmalloc(sizeof(*hal_evt), GFP_KERNEL);
	if (!hal_evt) {
		SLSI_ERR(sdev, "No memory for service_ind\n");
		goto exit;
	}

	memset(hal_evt, 0, sizeof(*hal_evt));
	hal_evt->publish_subscribe_id = fapi_get_u16(skb, u.mlme_nan_service_ind.session_id);
	hal_evt->requestor_instance_id = fapi_get_u16(skb, u.mlme_nan_service_ind.match_id);
	hal_evt->ranging_event_type = fapi_get_u16(skb, u.mlme_nan_service_ind.rangingindicationtype);
	hal_evt->range_measurement_mm = 10 * fapi_get_u16(skb, u.mlme_nan_service_ind.ranging_measurement);

	ptr = fapi_get_data(skb);
	tag_id = le16_to_cpu(*(u16 *)ptr);
	tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
	tag_data_ptr = ptr + 4;

	while (sig_data_len >= tag_len + 4) {
		switch (tag_id) {
		case SLSI_NAN_TLV_TAG_MATCH_IND:
			if (tag_len < 0x11) {
				SLSI_WARN(sdev, "Invalid taglen(%d) for SLSI_NAN_TLV_TAG_MATCH_IND\n", tag_len);
				break;
			}
			ether_addr_copy(hal_evt->addr, tag_data_ptr);
			tag_data_ptr += ETH_ALEN;
			hal_evt->match_occurred_flag = le16_to_cpu(*(u16 *)tag_data_ptr);
			tag_data_ptr += 2;
			hal_evt->out_of_resource_flag = le16_to_cpu(*(u16 *)tag_data_ptr);
			tag_data_ptr += 2;
			hal_evt->rssi_value = *tag_data_ptr;
			tag_data_ptr++;
			hal_evt->sec_info.cipher_type = *tag_data_ptr;
			break;
		case SLSI_NAN_TLV_TAG_SERVICE_SPECIFIC_INFO:
			hal_evt->service_specific_info_len = tag_len > SLSI_HAL_NAN_MAX_SERVICE_SPECIFIC_INFO_LEN ?
						SLSI_HAL_NAN_MAX_SERVICE_SPECIFIC_INFO_LEN : tag_len;
			memcpy(hal_evt->service_specific_info, tag_data_ptr, hal_evt->service_specific_info_len);
			break;
		case SLSI_NAN_TLV_TAG_EXT_SERVICE_SPECIFIC_INFO:
			if (tag_len > SLSI_HAL_NAN_MAX_SDEA_SERVICE_SPEC_INFO_LEN)
				hal_evt->sdea_service_specific_info_len = SLSI_HAL_NAN_MAX_SDEA_SERVICE_SPEC_INFO_LEN;
			else
				hal_evt->sdea_service_specific_info_len = tag_len;
			memcpy(hal_evt->sdea_service_specific_info, tag_data_ptr, hal_evt->sdea_service_specific_info_len);
			break;
		case SLSI_NAN_TLV_TAG_DATA_PATH_SECURITY:
			if (tag_len < 7) {
				SLSI_WARN(sdev, "Invalid taglen(%d) for SLSI_NAN_TLV_TAG_DATA_PATH_SECURITY\n", tag_len);
				break;
			}
			hal_evt->sec_info.key_info.key_type = *tag_data_ptr;
			tag_data_ptr++;
			hal_evt->sec_info.cipher_type = *tag_data_ptr;
			tag_data_ptr++;
			break;
		case SLSI_NAN_TLV_TAG_MATCH_FILTER:
			if (tag_len > SLSI_HAL_NAN_MAX_MATCH_FILTER_LEN)
				hal_evt->sdf_match_filter_len = SLSI_HAL_NAN_MAX_MATCH_FILTER_LEN;
			else
				hal_evt->sdf_match_filter_len = tag_len;
			memcpy(hal_evt->sdf_match_filter, tag_data_ptr, hal_evt->sdf_match_filter_len);
			break;
		default:
			SLSI_WARN(sdev, "Skip processing TLV %d\n", tag_id);
			break;
		}

		sig_data_len -= tag_len + 4;
		ptr += tag_len + 4;
		if (sig_data_len > 4) {
			tag_id = le16_to_cpu(*(u16 *)ptr);
			tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
			tag_data_ptr = ptr + 4;
		} else {
			tag_id = 0;
			tag_len = 0;
		}
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(SLSI_NL80211_NAN_MATCH_EVENT), SLSI_NL80211_NAN_MATCH_EVENT);
#endif
#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, SLSI_NL80211_NAN_MATCH_EVENT,
					     GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, SLSI_NL80211_NAN_MATCH_EVENT, GFP_KERNEL);
#endif
	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		kfree(hal_evt);
		goto exit;
	}
	res = nla_put_u16(nl_skb, NAN_EVT_ATTR_MATCH_PUBLISH_SUBSCRIBE_ID, hal_evt->publish_subscribe_id);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_MATCH_REQUESTOR_INSTANCE_ID, hal_evt->requestor_instance_id);
	res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_ADDR, ETH_ALEN, hal_evt->addr);
	res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_MATCH_SERVICE_SPECIFIC_INFO_LEN, hal_evt->service_specific_info_len);
	if (hal_evt->service_specific_info_len)
		res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_SERVICE_SPECIFIC_INFO, hal_evt->service_specific_info_len,
			       hal_evt->service_specific_info);
	res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_MATCH_SDF_MATCH_FILTER_LEN, hal_evt->sdf_match_filter_len);
	if (hal_evt->sdf_match_filter_len)
		res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_SDF_MATCH_FILTER, hal_evt->sdf_match_filter_len,
			       hal_evt->sdf_match_filter);
	res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_SDEA_LEN, hal_evt->sdea_service_specific_info_len);
	if (hal_evt->sdea_service_specific_info_len)
		res |= nla_put(nl_skb, NAN_EVT_ATTR_SDEA, hal_evt->sdea_service_specific_info_len,
			       hal_evt->sdea_service_specific_info);

	res |= nla_put_u8(nl_skb, NAN_EVT_ATTR_MATCH_MATCH_OCCURRED_FLAG, hal_evt->match_occurred_flag);
	res |= nla_put_u8(nl_skb, NAN_EVT_ATTR_MATCH_OUT_OF_RESOURCE_FLAG, hal_evt->out_of_resource_flag);
	res |= nla_put_u8(nl_skb, NAN_EVT_ATTR_MATCH_RSSI_VALUE, hal_evt->rssi_value);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_RANGE_MEASUREMENT_MM, hal_evt->range_measurement_mm);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_RANGEING_EVENT_TYPE, hal_evt->ranging_event_type);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_SECURITY_CIPHER_TYPE, hal_evt->sec_info.cipher_type);

	if (res) {
		SLSI_ERR(sdev, "Error in nla_put*:%x\n", res);
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		kfree(hal_evt);
		goto exit;
	}

	cfg80211_vendor_event(nl_skb, GFP_KERNEL);
	kfree(hal_evt);
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

static void slsi_nan_get_resp_status_code(u16 fapi_result_code, u16 *resp_code, u16 *status_code)
{
	switch (fapi_result_code) {
	case FAPI_RESULTCODE_SUCCESS:
		*resp_code = NAN_DP_REQUEST_ACCEPT;
		*status_code = SLSI_HAL_NAN_STATUS_SUCCESS;
		break;
	case FAPI_RESULTCODE_NDP_REJECTED:
		*resp_code = NAN_DP_REQUEST_REJECT;
		*status_code = SLSI_HAL_NAN_STATUS_SUCCESS;
		break;
	case FAPI_RESULTCODE_NAN_NO_OTA_ACK:
		*resp_code = NAN_DP_REQUEST_REJECT;
		*status_code = SLSI_HAL_NAN_STATUS_NO_OTA_ACK;
		break;
	case FAPI_RESULTCODE_NAN_INVALID_AVAILABILITY:
	case FAPI_RESULTCODE_NAN_IMMUTABLE_UNACCEPTABLE:
	case FAPI_RESULTCODE_NAN_REJECTED_SECURITY_POLICY:
	case FAPI_RESULTCODE_NDL_UNACCEPTABLE:
		*resp_code = NAN_DP_REQUEST_REJECT;
		*status_code = SLSI_HAL_NAN_STATUS_PROTOCOL_FAILURE;
		break;
	case FAPI_RESULTCODE_TRANSMISSION_FAILURE:
	default:
		*resp_code = NAN_DP_REQUEST_REJECT;
		*status_code = SLSI_HAL_NAN_STATUS_INTERNAL_FAILURE;
	}
}

u32 slsi_nan_get_ndp_from_ndl_local_ndi(struct net_device *dev, u16 ndl_vif_id, u8 *local_ndi)
{
	int i, j;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	for (i = 0; i < SLSI_NAN_MAX_NDP_INSTANCES; i++) {
		if (ndev_vif->nan.ndp_id2ndl_vif[i] == ndl_vif_id) {
			for (j = 0; j < SLSI_NAN_MAX_NDP_INSTANCES; j++)
				if (ether_addr_equal(ndev_vif->nan.ndp_ndi[j], local_ndi))
					return i + 1;
		}
	}
	return SLSI_NAN_MAX_NDP_INSTANCES + 1;
}

static int slsi_nan_put_ndp_req_ind_params(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb,
					   struct sk_buff *nl_skb, u8 **peer_ndi, u16 *resp_code, u16 *ndp_id)
{
	int res;
	u16 ndl_vif_id, status_code;
	u8 *local_ndi;

	ndl_vif_id = fapi_get_u16(skb, u.mlme_ndp_request_ind.ndl_vif_index);
	*peer_ndi = fapi_get_buff(skb, u.mlme_ndp_request_ind.peer_ndp_interface_address);
	local_ndi = fapi_get_buff(skb, u.mlme_ndp_request_ind.local_ndp_interface_address);
	slsi_nan_get_resp_status_code(fapi_get_u16(skb, u.mlme_ndp_request_ind.result_code), resp_code, &status_code);
	*ndp_id = slsi_nan_get_ndp_from_ndl_local_ndi(dev, ndl_vif_id, local_ndi);

	res = nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_INSTANCE_ID, *ndp_id);
	res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_ADDR, ETH_ALEN, *peer_ndi);

	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_RSP_CODE, *resp_code);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_STATUS_CODE, status_code);

	return res;
}

static int slsi_nan_put_ndp_resp_ind_params(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb,
					    struct sk_buff *nl_skb, u8 **peer_ndi, u16 *resp_code, u16 *ndp_id)
{
	int res;
	u16 ndl_vif_id, status_code;
	u8 *local_ndi;

	ndl_vif_id = fapi_get_u16(skb, u.mlme_ndp_response_ind.ndl_vif_index);
	*peer_ndi = fapi_get_buff(skb, u.mlme_ndp_response_ind.peer_ndp_interface_address);
	local_ndi = fapi_get_buff(skb, u.mlme_ndp_response_ind.local_ndp_interface_address);
	slsi_nan_get_resp_status_code(fapi_get_u16(skb, u.mlme_ndp_response_ind.result_code), resp_code, &status_code);
	*ndp_id = slsi_nan_get_ndp_from_ndl_local_ndi(dev, ndl_vif_id, local_ndi);

	res = nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_INSTANCE_ID, *ndp_id);
	res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_ADDR, ETH_ALEN, *peer_ndi);

	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_RSP_CODE, *resp_code);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_STATUS_CODE, status_code);

	return res;
}

void slsi_nan_ndp_setup_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb, bool is_req_ind)
{
	u16 tag_id, tag_len;
	u8  *ptr;
	const u8 *tag_data_ptr;
	int sig_data_len;
	struct sk_buff *nl_skb;
	int res;
	u8 *peer_ndi;
	u16 ndp_setup_response, ndp_id;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_DBG3(sdev, SLSI_GSCAN, "\n");
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	sig_data_len = fapi_get_datalen(skb);

#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_CFM,
					     GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_CFM, GFP_KERNEL);
#endif
	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		goto exit;
	}

	if (is_req_ind)
		res = slsi_nan_put_ndp_req_ind_params(sdev, dev, skb, nl_skb, &peer_ndi, &ndp_setup_response, &ndp_id);
	else
		res = slsi_nan_put_ndp_resp_ind_params(sdev, dev, skb, nl_skb, &peer_ndi, &ndp_setup_response, &ndp_id);

	if (ndp_setup_response != NAN_DP_REQUEST_ACCEPT)
		slsi_nan_ndp_del_entry(sdev, dev, ndp_id);

	ptr = fapi_get_data(skb);
	if (ptr) {
		tag_id = le16_to_cpu(*(u16 *)ptr);
		tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
		tag_data_ptr = ptr + 4;

		while (sig_data_len >= tag_len + 4) {
			if (tag_id == SLSI_NAN_TLV_TAG_APP_INFO) {
				res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_APP_INFO_LEN, tag_len);
				res |= nla_put(nl_skb, NAN_EVT_ATTR_APP_INFO, tag_len, tag_data_ptr);
				break;
			}
			sig_data_len -= tag_len + 4;
			ptr += tag_len + 4;
			if (sig_data_len > 4) {
				tag_id = le16_to_cpu(*(u16 *)ptr);
				tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
				tag_data_ptr = ptr + 4;
			} else {
				tag_id = 0;
				tag_len = 0;
			}
		}
	}

	if (res) {
		SLSI_ERR(sdev, "Error in nla_put*:%x\n", res);
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		goto exit;
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(SLSI_NAN_EVENT_NDP_CFM), SLSI_NAN_EVENT_NDP_CFM);
#endif

	cfg80211_vendor_event(nl_skb, GFP_KERNEL);
	if (ndp_setup_response == NAN_DP_REQUEST_ACCEPT) {
		struct netdev_vif *ndev_data_vif;
		struct net_device *data_dev = slsi_get_netdev_by_mac_addr(sdev, ndev_vif->nan.ndp_ndi[ndp_id],
									  SLSI_NAN_DATA_IFINDEX_START);
		struct slsi_peer *peer = NULL;

		if (ndp_id == 0 || ndp_id > SLSI_NAN_MAX_NDP_INSTANCES + 1) {
			SLSI_ERR(sdev, "Invalid ndp_id:%d\n", ndp_id);
			goto exit;
		}

		data_dev = slsi_get_netdev_by_mac_addr(sdev, ndev_vif->nan.ndp_ndi[ndp_id - 1],
						       SLSI_NAN_DATA_IFINDEX_START);

		if (!data_dev) {
			SLSI_ERR(sdev, "no data_dev for ndp:%d ndi[%pM]\n", ndp_id, ndev_vif->nan.ndp_ndi[ndp_id - 1]);
			goto exit;
		}
		ndev_data_vif = netdev_priv(data_dev);
		SLSI_MUTEX_LOCK(ndev_data_vif->vif_mutex);
		peer = slsi_peer_add(sdev, data_dev, peer_ndi, ndp_id);
		if (peer) {
			peer->connected_state = SLSI_STA_CONN_STATE_CONNECTED;
			slsi_ps_port_control(sdev, data_dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
			peer->ndl_vif = ndev_vif->nan.ndp_id2ndl_vif[ndp_id - 1];
			peer->qos_enabled = true;
		} else {
			SLSI_ERR(sdev, "no peer for ndp:%d ndi[%d]\n", ndp_id, ndev_vif->nan.ndp_ndi[ndp_id - 1]);
		}
		SLSI_MUTEX_UNLOCK(ndev_data_vif->vif_mutex);
	}
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_nan_ndp_requested_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 tag_id, tag_len = 0, ndl_vif_id, local_ndp_id;
	u8  *ptr, *peer_nmi;
	const u8 *tag_data_ptr;
	int sig_data_len, res;
	struct sk_buff *nl_skb;
	u32 ndp_id;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_DBG3(sdev, SLSI_GSCAN, "\n");
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	sig_data_len = fapi_get_datalen(skb);

#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_REQ,
					     GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_REQ, GFP_KERNEL);
#endif
	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		goto exit;
	}

	local_ndp_id = fapi_get_u16(skb, u.mlme_ndp_requested_ind.request_id);
	ndp_id = slsi_nan_get_new_ndp_id(ndev_vif);
	peer_nmi = fapi_get_buff(skb, u.mlme_ndp_requested_ind.peer_nan_management_interface_address);
	res = nla_put_u16(nl_skb, NAN_EVT_ATTR_SERVICE_INSTANCE_ID,
			  fapi_get_u16(skb, u.mlme_ndp_requested_ind.session_id));
	res |= nla_put(nl_skb, NAN_EVT_ATTR_MATCH_ADDR, ETH_ALEN, peer_nmi);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_INSTANCE_ID, ndp_id);
	res |= nla_put_u32(nl_skb, NAN_EVT_ATTR_SDEA_PARAM_SECURITY_CONFIG,
			   fapi_get_u16(skb, u.mlme_ndp_requested_ind.security_required));

	ptr = fapi_get_data(skb);
	if (ptr) {
		tag_id = le16_to_cpu(*(u16 *)ptr);
		tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
		tag_data_ptr = ptr + 4;

		while (sig_data_len >= tag_len + 4) {
			if (tag_id == SLSI_NAN_TLV_TAG_APP_INFO) {
				res |= nla_put_u16(nl_skb, NAN_EVT_ATTR_APP_INFO_LEN, tag_len);
				res |= nla_put(nl_skb, NAN_EVT_ATTR_APP_INFO, tag_len, tag_data_ptr);
				break;
			}
			sig_data_len -= tag_len + 4;
			ptr += tag_len + 4;
			if (sig_data_len > 4) {
				tag_id = le16_to_cpu(*(u16 *)ptr);
				tag_len = le16_to_cpu(*(u16 *)(ptr + 2));
				tag_data_ptr = ptr + 4;
			} else {
				tag_id = 0;
				tag_len = 0;
			}
		}
	}

	if (res) {
		SLSI_ERR(sdev, "Error in nla_put*:%x\n", res);
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		goto exit;
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(SLSI_NAN_EVENT_NDP_REQ), SLSI_NAN_EVENT_NDP_REQ);
#endif
	ndl_vif_id = slsi_nan_ndp_get_ndl_vif_id(peer_nmi, ndev_vif->nan.ndl_list);
	if (slsi_nan_ndp_new_entry(sdev, dev, ndp_id, ndl_vif_id, NULL, peer_nmi) == 0) {
		cfg80211_vendor_event(nl_skb, GFP_KERNEL);
		ndev_vif->nan.ndp_local_ndp_id[ndp_id - 1] = local_ndp_id;
	} else {
		struct slsi_hal_nan_data_path_indication_response response_req;

		kfree_skb(nl_skb);
		SLSI_ERR(sdev, "invalid ndl_vifid:%d ndp_id:%d\n", ndl_vif_id, ndp_id);
		memset(&response_req, 0, sizeof(response_req));
		response_req.rsp_code = NAN_DP_REQUEST_REJECT;
		slsi_mlme_ndp_response(sdev, dev, &response_req, local_ndp_id);
	}

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_nan_del_peer(struct slsi_dev *sdev, struct net_device *dev, u8 *local_ndi, u16 ndp_id)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct net_device *data_dev;
	struct netdev_vif *ndev_data_vif;
	struct slsi_peer *peer = NULL;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));

	if (!local_ndi)
		return;

	data_dev = slsi_get_netdev_by_mac_addr_locked(sdev, local_ndi, SLSI_NAN_DATA_IFINDEX_START);
	if (!data_dev)
		return;

	if (ndp_id == 0 || ndp_id > SLSI_NAN_MAX_NDP_INSTANCES)
		return;

	ndev_data_vif = netdev_priv(data_dev);
	SLSI_MUTEX_LOCK(ndev_data_vif->vif_mutex);
	peer = ndev_vif->peer_sta_record[ndp_id - 1];
	if (peer) {
		slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DISCONNECTED);
		slsi_spinlock_lock(&ndev_data_vif->peer_lock);
		slsi_peer_remove(sdev, dev, peer);
		slsi_spinlock_unlock(&ndev_data_vif->peer_lock);
	} else {
		SLSI_ERR(sdev, "no peer for ndp:%d ndi[%pM]\n", ndp_id, ndev_vif->nan.ndp_ndi[ndp_id - 1]);
	}
	SLSI_MUTEX_UNLOCK(ndev_data_vif->vif_mutex);
}

void slsi_nan_ndp_termination_handler(struct slsi_dev *sdev, struct net_device *dev, u16 ndp_id, u16 ndl_vif, u8 *ndi)
{
	struct sk_buff *nl_skb;

	slsi_nan_ndp_del_entry(sdev, dev, ndp_id);
	slsi_nan_del_peer(sdev, dev, ndi, ndp_id);

#if (KERNEL_VERSION(4, 1, 0) <= LINUX_VERSION_CODE)
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_END,
					     GFP_KERNEL);
#else
	nl_skb = cfg80211_vendor_event_alloc(sdev->wiphy, NLMSG_DEFAULT_SIZE, SLSI_NAN_EVENT_NDP_END, GFP_KERNEL);
#endif
	if (!nl_skb) {
		SLSI_ERR(sdev, "NO MEM for nl_skb!!!\n");
		return;
	}

	if (nla_put_u32(nl_skb, NAN_EVT_ATTR_NDP_INSTANCE_ID, ndp_id)) {
		SLSI_ERR(sdev, "Error in nla_put_u32\n");
		/* Dont use slsi skb wrapper for this free */
		kfree_skb(nl_skb);
		return;
	}
#ifdef CONFIG_SCSC_WLAN_DEBUG
	SLSI_INFO(sdev, "Event: %s(%d)\n",
		  slsi_print_event_name(SLSI_NAN_EVENT_NDP_END), SLSI_NAN_EVENT_NDP_END);
#endif
	cfg80211_vendor_event(nl_skb, GFP_KERNEL);
}

void slsi_nan_ndp_termination_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb,
				  bool is_terminated_ind)
{
	u16 ndl_vif_id, ndp_id;
	u8 *local_ndi;
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_DBG3(sdev, SLSI_GSCAN, "\n");
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (is_terminated_ind) {
		ndl_vif_id = fapi_get_u16(skb, u.mlme_ndp_terminated_ind.ndl_vif_index);
		local_ndi = fapi_get_buff(skb, u.mlme_ndp_terminated_ind.local_ndp_interface_address);
	} else {
		ndl_vif_id = fapi_get_u16(skb, u.mlme_ndp_terminate_ind.ndl_vif_index);
		local_ndi = fapi_get_buff(skb, u.mlme_ndp_terminate_ind.local_ndp_interface_address);
	}

	ndp_id = slsi_nan_get_ndp_from_ndl_local_ndi(dev, ndl_vif_id, local_ndi);
	if (ndp_id <= SLSI_NAN_MAX_NDP_INSTANCES)
		slsi_nan_ndp_termination_handler(sdev, dev, ndp_id, ndl_vif_id, local_ndi);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}
