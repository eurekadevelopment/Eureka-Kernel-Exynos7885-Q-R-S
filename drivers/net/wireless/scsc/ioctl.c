/****************************************************************************
 *
 * Copyright (c) 2012 - 2018 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include "ioctl.h"
#include "debug.h"
#include "mlme.h"
#include "mgt.h"
#include "cac.h"
#include "hip.h"
#include "netif.h"
#include <net/netlink.h>
#include <linux/netdevice.h>
#include <linux/ieee80211.h>
#include "mib.h"
#include <scsc/scsc_mx.h>
#include <scsc/scsc_log_collector.h>
#include "dev.h"
#include "fapi.h"

#define CMD_RXFILTERADD         "RXFILTER-ADD"
#define CMD_RXFILTERREMOVE              "RXFILTER-REMOVE"
#define CMD_RXFILTERSTART               "RXFILTER-START"
#define CMD_RXFILTERSTOP                "RXFILTER-STOP"
#define CMD_SETCOUNTRYREV               "SETCOUNTRYREV"
#define CMD_GETCOUNTRYREV               "GETCOUNTRYREV"
#define CMD_SETROAMTRIGGER              "SETROAMTRIGGER"
#define CMD_GETROAMTRIGGER              "GETROAMTRIGGER"
#define CMD_SETSUSPENDMODE              "SETSUSPENDMODE"
#define CMD_SETROAMDELTA                "SETROAMDELTA"
#define CMD_GETROAMDELTA                "GETROAMDELTA"
#define CMD_SETROAMSCANPERIOD           "SETROAMSCANPERIOD"
#define CMD_GETROAMSCANPERIOD           "GETROAMSCANPERIOD"
#define CMD_SETFULLROAMSCANPERIOD               "SETFULLROAMSCANPERIOD"
#define CMD_GETFULLROAMSCANPERIOD               "GETFULLROAMSCANPERIOD"
#define CMD_SETSCANCHANNELTIME          "SETSCANCHANNELTIME"
#define CMD_GETSCANCHANNELTIME          "GETSCANCHANNELTIME"
#define CMD_SETSCANNPROBES              "SETSCANNPROBES"
#define CMD_GETSCANNPROBES              "GETSCANNPROBES"
#define CMD_SETROAMMODE         "SETROAMMODE"
#define CMD_GETROAMMODE         "GETROAMMODE"
#define CMD_SETROAMINTRABAND            "SETROAMINTRABAND"
#define CMD_GETROAMINTRABAND            "GETROAMINTRABAND"
#define CMD_SETROAMBAND         "SETROAMBAND"
#define CMD_GETROAMBAND         "GETROAMBAND"
#define CMD_SETROAMSCANCONTROL          "SETROAMSCANCONTROL"
#define CMD_GETROAMSCANCONTROL          "GETROAMSCANCONTROL"
#define CMD_SETSCANHOMETIME             "SETSCANHOMETIME"
#define CMD_GETSCANHOMETIME             "GETSCANHOMETIME"
#define CMD_SETSCANHOMEAWAYTIME         "SETSCANHOMEAWAYTIME"
#define CMD_GETSCANHOMEAWAYTIME         "GETSCANHOMEAWAYTIME"
#define CMD_SETOKCMODE          "SETOKCMODE"
#define CMD_GETOKCMODE          "GETOKCMODE"
#define CMD_SETWESMODE          "SETWESMODE"
#define CMD_GETWESMODE          "GETWESMODE"
#define CMD_SET_PMK             "SET_PMK"
#define CMD_HAPD_GET_CHANNEL			"HAPD_GET_CHANNEL"
#define CMD_SET_SAP_CHANNEL_LIST                "SET_SAP_CHANNEL_LIST"
#define CMD_REASSOC             "REASSOC"
#define CMD_SETROAMSCANCHANNELS         "SETROAMSCANCHANNELS"
#define CMD_GETROAMSCANCHANNELS         "GETROAMSCANCHANNELS"
#define CMD_ADDROAMSCANCHANNELS         "ADDROAMSCANCHANNELS"
#define CMD_SENDACTIONFRAME             "SENDACTIONFRAME"
#define CMD_CERTSENDACTIONFRAME         "CERTSENDACTIONFRAME"
#define CMD_GETNCHOMODE                 "GETNCHOMODE"
#define CMD_SETNCHOMODE                 "SETNCHOMODE"
#define CMD_GETDFSSCANMODE                 "GETDFSSCANMODE"
#define CMD_SETDFSSCANMODE                 "SETDFSSCANMODE"
#define CMD_HAPD_MAX_NUM_STA            "HAPD_MAX_NUM_STA"
#define CMD_COUNTRY            "COUNTRY"
#define CMD_SEND_GK                               "SEND_GK"
#define CMD_SETAPP2PWPSIE "SET_AP_P2P_WPS_IE"
#define CMD_P2PSETPS "P2P_SET_PS"
#define CMD_P2PSETNOA "P2P_SET_NOA"
#define CMD_P2PECSA "P2P_ECSA"
#define CMD_P2PLOSTART "P2P_LO_START"
#define CMD_P2PLOSTOP "P2P_LO_STOP"
#define CMD_TDLSCHANNELSWITCH  "TDLS_CHANNEL_SWITCH"
#define CMD_SETROAMOFFLOAD     "SETROAMOFFLOAD"
#define CMD_SETROAMOFFLAPLIST  "SETROAMOFFLAPLIST"
#ifdef CONFIG_SCSC_WLAN_LOW_LATENCY_MODE
#define CMD_SET_LATENCY_MODE "SET_LATENCY_MODE"
#define CMD_SET_POWER_MGMT "SET_POWER_MGMT"
#endif
#define CMD_SET_LATENCY_CRITICAL "SET_LATENCY_CRITICAL"
#define CMD_SET_DISCONNECT_IES "SET_DISCONNECT_IES"

#define CMD_SETBAND "SETBAND"
#define CMD_GETBAND "GETBAND"
#define CMD_SET_FCC_CHANNEL "SET_FCC_CHANNEL"

#define CMD_FAKEMAC "FAKEMAC"

#define CMD_GETBSSRSSI "GET_BSS_RSSI"
#define CMD_GETBSSINFO "GETBSSINFO"
#define CMD_GETSTAINFO "GETSTAINFO"
#define CMD_GETASSOCREJECTINFO "GETASSOCREJECTINFO"

#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
#define CMD_BEACON_RECV "BEACON_RECV"
#endif
#ifdef CONFIG_SCSC_WLAN_STA_ENHANCED_ARP_DETECT
#define CMD_SET_ENHANCED_ARP_TARGET "SET_ENHANCED_ARP_TARGET"
#define CMD_GET_ENHANCED_ARP_COUNTS "GET_ENHANCED_ARP_COUNTS"
#endif

/* Known commands from framework for which no handlers */
#define CMD_AMPDU_MPDU "AMPDU_MPDU"
#define CMD_BTCOEXMODE "BTCOEXMODE"
#define CMD_BTCOEXSCAN_START "BTCOEXSCAN-START"
#define CMD_BTCOEXSCAN_STOP "BTCOEXSCAN-STOP"
#define CMD_CHANGE_RL "CHANGE_RL"
#define CMD_INTERFACE_CREATE "INTERFACE_CREATE"
#define CMD_INTERFACE_DELETE "INTERFACE_DELETE"
#define CMD_SET_INDOOR_CHANNELS "SET_INDOOR_CHANNELS"
#define CMD_GET_INDOOR_CHANNELS "GET_INDOOR_CHANNELS"
#define CMD_LTECOEX "LTECOEX"
#define CMD_MIRACAST "MIRACAST"
#define CMD_RESTORE_RL "RESTORE_RL"
#define CMD_RPSMODE "RPSMODE"
#define CMD_SETCCXMODE "SETCCXMODE"
#define CMD_SETDFSSCANMODE "SETDFSSCANMODE"
#define CMD_SETJOINPREFER "SETJOINPREFER"
#define CMD_SETSINGLEANT "SETSINGLEANT"
#define CMD_SET_TX_POWER_CALLING "SET_TX_POWER_CALLING"

#define CMD_DRIVERDEBUGDUMP "DEBUG_DUMP"
#define CMD_DRIVERDEBUGCOMMAND "DEBUG_COMMAND"
#define CMD_TESTFORCEHANG "SLSI_TEST_FORCE_HANG"
#define CMD_GETREGULATORY "GETREGULATORY"

#define CMD_SET_TX_POWER_SAR "SET_TX_POWER_SAR"
#define CMD_GET_TX_POWER_SAR "GET_TX_POWER_SAR"

#ifdef CONFIG_SCSC_WLAN_ENHANCED_PKT_FILTER
#define CMD_ENHANCED_PKT_FILTER "ENHANCED_PKT_FILTER"
#endif

#ifdef CONFIG_SCSC_WLAN_MAX_LINK_SPEED
#define CMD_GET_MAX_LINK_SPEED "GET_MAX_LINK_SPEED"
#endif

#ifdef CONFIG_SCSC_WLAN_SET_NUM_ANTENNAS
#define CMD_SET_NUM_ANTENNAS "SET_NUM_ANTENNAS"
#endif

#define ROAMOFFLAPLIST_MIN 1
#define ROAMOFFLAPLIST_MAX 100

struct slsi_ioctl_args *slsi_get_private_command_args(char *buffer, int buf_len, int max_arg_count)
{
	struct slsi_ioctl_args *ioctl_args = NULL;
	char *pos                          = buffer;

	ioctl_args = kmalloc(sizeof(*ioctl_args) + sizeof(u8 *) * max_arg_count, GFP_KERNEL);
	if (!ioctl_args)
		return NULL;
	memset(ioctl_args->args, '\0', sizeof(u8 *) * max_arg_count);

	ioctl_args->arg_count = 0;
	while (buf_len > 0 && ioctl_args->arg_count < max_arg_count) {
		pos = strchr(pos, ' ');
		if (!pos)
			break;
		buf_len = buf_len - (pos - buffer + 1);
		if (buf_len <= 0)
			break;
		*pos = '\0';
		pos++;
		while (*pos == ' ')
			pos++;
		buffer = pos;
		ioctl_args->args[ioctl_args->arg_count++] = pos;
	}
	return ioctl_args;
}

static int slsi_parse_hex(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return 0;
}

void slsi_convert_space_seperation(char *buf, int buf_len)
{
	int i = 0;

	while (buf[i] != '\0' && i < buf_len) {
		if (buf[i] == ',' || buf[i] == '=')
			buf[i] = ' ';
		i++;
	}
}

static void slsi_machexstring_to_macarray(char *mac_str, u8 *mac_arr)
{
	mac_arr[0] = slsi_parse_hex(mac_str[0]) << 4 | slsi_parse_hex(mac_str[1]);
	mac_arr[1] = slsi_parse_hex(mac_str[3]) << 4 | slsi_parse_hex(mac_str[4]);
	mac_arr[2] = slsi_parse_hex(mac_str[6]) << 4 | slsi_parse_hex(mac_str[7]);
	mac_arr[3] = slsi_parse_hex(mac_str[9]) << 4 | slsi_parse_hex(mac_str[10]);
	mac_arr[4] = slsi_parse_hex(mac_str[12]) << 4 | slsi_parse_hex(mac_str[13]);
	mac_arr[5] = slsi_parse_hex(mac_str[15]) << 4 | slsi_parse_hex(mac_str[16]);
}

static ssize_t slsi_set_suspend_mode(struct net_device *dev, char *command, int cmd_len)
{
	struct netdev_vif *netdev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = netdev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               user_suspend_mode;
	int               previous_suspend_mode;
	u8                host_state;
	int               ret = 0;
	int vif;

	ioctl_args = slsi_get_private_command_args(command, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	user_suspend_mode = *ioctl_args->args[0] - '0';
	if (user_suspend_mode != 0 && user_suspend_mode != 1) {
		SLSI_ERR(sdev, "Invalid value of user_suspend_mode %d\n", user_suspend_mode);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	previous_suspend_mode = sdev->device_config.user_suspend_mode;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (user_suspend_mode != previous_suspend_mode) {
		SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
		for (vif = 1; vif <= CONFIG_SCSC_WLAN_MAX_INTERFACES; vif++) {
			struct net_device *dev = slsi_get_netdev_locked(sdev, vif);
			struct netdev_vif *ndev_vif;

			if (!dev)
				continue;

			ndev_vif = netdev_priv(dev);
			SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
			if (ndev_vif->activated &&
			    ndev_vif->vif_type == FAPI_VIFTYPE_STATION &&
			    ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED) {
				if (user_suspend_mode)
					ret = slsi_update_packet_filters(sdev, dev);
				else
					ret = slsi_clear_packet_filters(sdev, dev);
				if (ret != 0)
					SLSI_NET_ERR(dev, "Error in updating /clearing the packet filters,ret=%d", ret);
			}

			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		}
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	sdev->device_config.user_suspend_mode = user_suspend_mode;
	host_state = sdev->device_config.host_state;

	if (!sdev->device_config.user_suspend_mode)
		host_state = host_state | SLSI_HOSTSTATE_LCD_ACTIVE;
	else
		host_state = host_state & ~SLSI_HOSTSTATE_LCD_ACTIVE;
	sdev->device_config.host_state = host_state;

	ret = slsi_mlme_set_host_state(sdev, dev, host_state);
	if (ret != 0)
		SLSI_NET_ERR(dev, "Error in setting the Host State, ret=%d", ret);

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_set_p2p_oppps(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	unsigned int      ct_param;
	unsigned int      legacy_ps;
	unsigned int      opp_ps;
	int               result = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 3);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (ioctl_args->arg_count < 3) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		result = -EINVAL;
		goto exit;
	}

	/* The NOA param shall be added only after P2P-VIF is active */
	if (!ndev_vif->activated || ndev_vif->iftype != NL80211_IFTYPE_P2P_GO) {
		SLSI_ERR_NODEV("P2P GO vif not activated\n");
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &legacy_ps)) {
		SLSI_ERR(sdev, "legacy_ps: failed to read from string: '%s'\n", ioctl_args->args[0]);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[1], &opp_ps)) {
		SLSI_ERR(sdev, "opp_ps: failed to read from string: '%s'\n", ioctl_args->args[1]);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[2], &ct_param)) {
		SLSI_ERR(sdev, "ct_param: failed to read from string: '%s'\n", ioctl_args->args[2]);
		result = -EINVAL;
		goto exit;
	}

	if (opp_ps == 0)
		result = slsi_mlme_set_ctwindow(sdev, dev, opp_ps);
	else if (ct_param < (unsigned int)ndev_vif->ap.beacon_interval)
		result = slsi_mlme_set_ctwindow(sdev, dev, ct_param);
	else
		SLSI_DBG1(sdev, SLSI_CFG80211, "p2p ct window = %d is out of range for beacon interval(%d)\n", ct_param, ndev_vif->ap.beacon_interval);
exit:
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return result;
}

static ssize_t slsi_p2p_set_noa_params(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  result = 0;
	unsigned int         noa_count;
	unsigned int         duration;
	unsigned int         interval;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 3);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (ioctl_args->arg_count < 3) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		result = -EINVAL;
		goto exit;
	}

	/* The NOA param shall be added only after P2P-VIF is active */
	if (!ndev_vif->activated || ndev_vif->iftype != NL80211_IFTYPE_P2P_GO) {
		SLSI_ERR_NODEV("P2P GO vif not activated\n");
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &noa_count)) {
		SLSI_ERR(sdev, "noa_count: failed to read string: '%s'\n", ioctl_args->args[0]);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[1], &interval)) {
		SLSI_ERR(sdev, "interval: failed to read string: '%s'\n", ioctl_args->args[1]);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[2], &duration)) {
		SLSI_ERR(sdev, "duration: failed to read string: '%s'\n", ioctl_args->args[2]);
		result = -EINVAL;
		goto exit;
	}

	/* Skip start time */
	result = slsi_mlme_set_p2p_noa(sdev, dev, noa_count, interval, duration);

exit:
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return result;
}

static ssize_t slsi_p2p_ecsa(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct netdev_vif *group_dev_vif;
	struct net_device *group_dev = NULL;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  result = 0;
	unsigned int         channel;
	unsigned int         bandwidth;
	u16 center_freq = 0;
	u16 chan_info = 0;
	struct cfg80211_chan_def chandef;
	enum ieee80211_band band;
	enum nl80211_channel_type chan_type = NL80211_CHAN_NO_HT;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 2);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (ioctl_args->arg_count < 2) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	group_dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2PX_SWLAN);
	if (!group_dev) {
		SLSI_INFO(sdev, "No Group net_dev found\n");
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &channel)) {
		SLSI_ERR(sdev, "channel: failed to read string: '%s'\n", ioctl_args->args[0]);
		result = -EINVAL;
		goto exit;
	}

	if (channel < 1 || channel > 165) {
		SLSI_ERR(sdev, "Invalid channel : %d\n", channel);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[1], &bandwidth)) {
		SLSI_ERR(sdev, "bandwidth: failed to read string: '%s'\n", ioctl_args->args[1]);
		result = -EINVAL;
		goto exit;
	}

	band = (channel <= 14) ? IEEE80211_BAND_2GHZ : IEEE80211_BAND_5GHZ;
	center_freq = ieee80211_channel_to_frequency(channel, band);
	SLSI_DBG1(sdev, SLSI_CFG80211, "p2p ecsa_params (center_freq)= (%d)\n", center_freq);
	chandef.chan = ieee80211_get_channel(sdev->wiphy, center_freq);
	chandef.width = (band  == IEEE80211_BAND_2GHZ) ? NL80211_CHAN_WIDTH_20_NOHT : NL80211_CHAN_WIDTH_80;

#ifndef SSB_4963_FIXED
	/* Default HT40 configuration */
	if (sdev->band_5g_supported) {
		if (bandwidth == 80) {
			chandef.width = NL80211_CHAN_WIDTH_40;
			bandwidth = 40;
			if (channel == 36 || channel == 44 || channel == 149 || channel == 157)
				chan_type = NL80211_CHAN_HT40PLUS;
			else
				chan_type = NL80211_CHAN_HT40MINUS;
		}
	}
#endif
	if (channel == 165 && bandwidth != 20) {
		bandwidth = 20;
		chan_type = NL80211_CHAN_WIDTH_20;
	}
	cfg80211_chandef_create(&chandef, chandef.chan, chan_type);
	chan_info = slsi_get_chann_info(sdev, &chandef);
	if (bandwidth != 20)
		center_freq = slsi_get_center_freq1(sdev, chan_info, center_freq);
	group_dev_vif = netdev_priv(group_dev);
	SLSI_MUTEX_LOCK(group_dev_vif->vif_mutex);
	result = slsi_mlme_channel_switch(sdev, group_dev, center_freq, chan_info);
	SLSI_MUTEX_UNLOCK(group_dev_vif->vif_mutex);

exit:
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return result;
}

static ssize_t slsi_ap_vendor_ies_write(struct slsi_dev *sdev, struct net_device *dev, u8 *ie,
					size_t ie_len, u16 purpose)
{
	u8                *vendor_ie = NULL;
	int               result = 0;
	struct netdev_vif *ndev_vif;

	ndev_vif = netdev_priv(dev);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	/* During AP start before mlme_start_req, supplicant calls set_ap_wps_ie() to send the vendor IEs for each
	 * beacon, probe response and association response. As we get all of them in mlme_start_req, ignoring the
	 * same which comes before adding GO VIF
	 */
	if (!ndev_vif->activated) {
		SLSI_DBG1(sdev, SLSI_CFG80211, "vif not activated\n");
		result = 0;
		goto exit;
	}
	if (!(ndev_vif->iftype == NL80211_IFTYPE_P2P_GO || ndev_vif->iftype == NL80211_IFTYPE_AP)) {
		SLSI_ERR(sdev, "Not AP or P2P interface. interfaceType:%d\n", ndev_vif->iftype);
		result = -EINVAL;
		goto exit;
	}

	vendor_ie = kmalloc(ie_len, GFP_KERNEL);
	if (!vendor_ie) {
		SLSI_ERR(sdev, "kmalloc failed\n");
		result = -ENOMEM;
		goto exit;
	}
	memcpy(vendor_ie, ie, ie_len);

	slsi_clear_cached_ies(&ndev_vif->ap.add_info_ies, &ndev_vif->ap.add_info_ies_len);
	result = slsi_ap_prepare_add_info_ies(ndev_vif, vendor_ie, ie_len);

	if (result == 0)
		result = slsi_mlme_add_info_elements(sdev, dev, purpose, ndev_vif->ap.add_info_ies, ndev_vif->ap.add_info_ies_len);

	slsi_clear_cached_ies(&ndev_vif->ap.add_info_ies, &ndev_vif->ap.add_info_ies_len);
	kfree(vendor_ie);

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return result;
}

static ssize_t slsi_set_ap_p2p_wps_ie(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               result = 0;
	u8                *params;
	int               offset = 0;
	enum if_type {
		IF_TYPE_NONE,
		IF_TYPE_P2P_DEVICE,
		IF_TYPE_AP_P2P
	} iftype = IF_TYPE_NONE;
	enum frame_type {
		FRAME_TYPE_NONE,
		FRAME_TYPE_BEACON,
		FRAME_TYPE_PROBE_RESPONSE,
		FRAME_TYPE_ASSOC_RESPONSE
	} frametype = FRAME_TYPE_NONE;
	int params_len = buf_len - strlen(CMD_SETAPP2PWPSIE) - 1;

	params = command + strlen(CMD_SETAPP2PWPSIE) + 1;
	ioctl_args = slsi_get_private_command_args(command, buf_len, 2);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (ioctl_args->arg_count < 2) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		result = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], (int *)&frametype)) {
		SLSI_ERR(sdev, "Failed to read frame type string: '%s'\n", ioctl_args->args[0]);
		result = -EINVAL;
		goto exit;
	}
	offset = offset + strlen(ioctl_args->args[0]) + 1;

	if (!slsi_str_to_int(ioctl_args->args[1], (int *)&iftype)) {
		SLSI_ERR(sdev, "Failed to read iftype string: '%s'\n", ioctl_args->args[1]);
		result = -EINVAL;
		goto exit;
	}
	offset = offset + strlen(ioctl_args->args[1]) + 1;
	params_len = params_len - offset;

	SLSI_NET_DBG2(dev, SLSI_NETDEV,
		      "command=%s, frametype=%d, iftype=%d, total buf_len=%d, params_len=%d\n",
		      command, frametype, iftype, buf_len, params_len);

	/* check the net device interface type */
	if (iftype == IF_TYPE_P2P_DEVICE) {
		u8                *probe_resp_ie = NULL; /* params+offset; */

		if (frametype != FRAME_TYPE_PROBE_RESPONSE) {
			SLSI_NET_ERR(dev, "Wrong frame type received\n");
			result = -EINVAL;
			goto exit;
		}
		probe_resp_ie = kmalloc(params_len, GFP_KERNEL);
		if (!probe_resp_ie) {
			SLSI_ERR(sdev, "Malloc for IEs failed\n");
			kfree(ioctl_args);
			return -ENOMEM;
		}

		memcpy(probe_resp_ie, params + offset, params_len);

		result = slsi_p2p_dev_probe_rsp_ie(sdev, dev, probe_resp_ie, params_len);
	} else if (iftype == IF_TYPE_AP_P2P) {
		if (frametype == FRAME_TYPE_BEACON)
			result = slsi_ap_vendor_ies_write(sdev, dev, params + offset, params_len, FAPI_PURPOSE_BEACON);
		else if (frametype == FRAME_TYPE_PROBE_RESPONSE)
			result = slsi_ap_vendor_ies_write(sdev, dev, params + offset, params_len, FAPI_PURPOSE_PROBE_RESPONSE);
		else if (frametype == FRAME_TYPE_ASSOC_RESPONSE)
			result = slsi_ap_vendor_ies_write(sdev, dev, params + offset, params_len, FAPI_PURPOSE_ASSOCIATION_RESPONSE);
	}
exit:
	kfree(ioctl_args);
	return result;
}

/**
 * P2P_LO_START handling.
 * Add unsync vif, register for action frames and set the listen channel.
 * The probe response IEs would be configured later.
 */
static int slsi_p2p_lo_start(struct net_device *dev, char *command, int cmd_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct ieee80211_channel *chan = NULL;
	struct slsi_ioctl_args *ioctl_args = NULL;
	unsigned int channel, duration, interval, count;
	int  ret = 0;
	int  freq;
	enum nl80211_band band;

	ioctl_args = slsi_get_private_command_args(command, cmd_len, 4);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (ioctl_args->arg_count < 4) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	/* Reject LO if other operations are in progress. Back to back LO can be received.
	 * In such a case, if state is Listening then the listen offload flag should be true else
	 * reject the request as the Listening state would then be due to ROC.
	 */
	if (sdev->p2p_state == P2P_SCANNING || sdev->p2p_state > P2P_LISTENING ||
	    (sdev->p2p_state == P2P_LISTENING && !ndev_vif->unsync.listen_offload)) {
		SLSI_NET_ERR(dev, "Reject LO due to ongoing P2P operation (state: %s)\n", slsi_p2p_state_text(sdev->p2p_state));
		ret = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &channel)) {
		SLSI_ERR(sdev, "channel: failed to read string: '%s'\n", ioctl_args->args[0]);
		ret = -EINVAL;
		goto exit;
	}

	if (channel < 1 || channel > 165) {
		SLSI_ERR(sdev, "Invalid channel : %d\n", channel);
		ret = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[1], &duration)) {
		SLSI_ERR(sdev, "duration: failed to read string: '%s'\n", ioctl_args->args[1]);
		ret = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[2], &interval)) {
		SLSI_ERR(sdev, "interval: failed to read string: '%s'\n", ioctl_args->args[2]);
		ret = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[3], &count)) {
		SLSI_ERR(sdev, "count: failed to read string: '%s'\n", ioctl_args->args[3]);
		ret = -EINVAL;
		goto exit;
	}

	if (!ndev_vif->activated) {
		ret = slsi_mlme_add_vif(sdev, dev, dev->dev_addr, dev->dev_addr);
		if (ret != 0) {
			SLSI_NET_ERR(dev, "Unsync vif addition failed\n");
			goto exit;
		}

		ndev_vif->activated = true;
		ndev_vif->mgmt_tx_data.exp_frame = SLSI_PA_INVALID;
		SLSI_P2P_STATE_CHANGE(sdev, P2P_IDLE_VIF_ACTIVE);

		ret = slsi_mlme_register_action_frame(sdev, dev, SLSI_ACTION_FRAME_PUBLIC, SLSI_ACTION_FRAME_PUBLIC);
		if (ret != 0) {
			SLSI_NET_ERR(dev, "Action frame registration for unsync vif failed\n");
			goto exit_with_vif_deactivate;
		}
	}

	/* Send set_channel irrespective of the values of LO parameters as they are not cached
	 * in driver to check whether they have changed.
	 */
	band = (channel <= 14) ? IEEE80211_BAND_2GHZ : IEEE80211_BAND_5GHZ;
	freq = ieee80211_channel_to_frequency(channel, band);
	chan = ieee80211_get_channel(sdev->wiphy, freq);
	if (!chan) {
		SLSI_NET_ERR(dev, "Incorrect channel: %u - Listen Offload failed\n", channel);
		ret = -EINVAL;
		goto exit_with_vif_deactivate;
	}

	ret = slsi_mlme_set_channel(sdev, dev, chan, duration, interval, count);
	if (ret != 0) {
		SLSI_NET_ERR(dev, "Set channel for unsync vif failed\n");
		goto exit_with_vif_deactivate;
	} else {
		ndev_vif->chan = chan;
	}
	/* If framework sends the values for listen offload as 1,500,5000 and 6,
	 * where 5000ms (5 seconds) is the listen interval which needs to be repeated
	 * 6 times(i.e. count). Hence listen_end_ind comes after 30 seconds
	 * (6 * 5000 = 30000ms) Hence host should wait 31 seconds to delete the
	 * unsync VIF for one such P2P listen offload request.
	 */
	slsi_p2p_queue_unsync_vif_del_work(ndev_vif, interval * count + 1000);
	ndev_vif->unsync.listen_offload = true;
	SLSI_P2P_STATE_CHANGE(ndev_vif->sdev, P2P_LISTENING);
	goto exit;

exit_with_vif_deactivate:
	slsi_p2p_vif_deactivate(sdev, dev, true);
exit:
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return ret;
}

/**
 * P2P_LO_STOP handling.
 * Clear listen offload flag.
 * Delete the P2P unsynchronized vif.
 */
static int slsi_p2p_lo_stop(struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	WARN_ON((!ndev_vif->unsync.listen_offload) || (ndev_vif->sdev->p2p_state != P2P_LISTENING));

	ndev_vif->unsync.listen_offload = false;

	/* Deactivating the p2p unsynchronized vif */
	if (ndev_vif->sdev->p2p_state == P2P_LISTENING)
		slsi_p2p_vif_deactivate(ndev_vif->sdev, ndev_vif->wdev.netdev, true);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return 0;
}

static ssize_t slsi_rx_filter_num_write(struct net_device *dev, char *buffer, int buf_len, int add_remove)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               ret = 0;
	int               filter_num = 0;

	if (add_remove == 0) {
		SLSI_MUTEX_LOCK(sdev->device_config_mutex);
		sdev->device_config.rx_filter_num = 0;
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return ret;
	}

	ioctl_args = slsi_get_private_command_args(buffer, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	filter_num = *ioctl_args->args[0] - '0';
	if (filter_num < 0 || filter_num > 3) {
		SLSI_ERR(sdev, "Invalid value of filter_num %d\n", filter_num);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	sdev->device_config.rx_filter_num = filter_num;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return ret;
}

#ifdef CONFIG_SCSC_WLAN_WIFI_SHARING
#if !defined(CONFIG_SCSC_WLAN_MHS_STATIC_INTERFACE) || (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION < 9)
static ssize_t slsi_create_interface(struct net_device *dev, char *buffer, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	struct net_device   *ap_dev;
	char *intf_name = NULL;

	ioctl_args = slsi_get_private_command_args(buffer, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	intf_name = ioctl_args->args[0];
	if (strcmp(CONFIG_SCSC_AP_INTERFACE_NAME, intf_name) != 0) {
		SLSI_NET_ERR(dev, "Creation of %s not allowed!\n", intf_name);
		kfree(ioctl_args);
		return -EINVAL;
	}
	ap_dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2PX_SWLAN);
	if (ap_dev && (strcmp(ap_dev->name, intf_name) == 0)) {
		SLSI_NET_ERR(dev, "%s already created\n", intf_name);
		kfree(ioctl_args);
		return -EINVAL;
	}

	ap_dev = slsi_dynamic_interface_create(sdev->wiphy, intf_name, NL80211_IFTYPE_AP, NULL);
	if (ap_dev) {
		sdev->netdev_ap = ap_dev;
		kfree(ioctl_args);
		return 0;
	}

	SLSI_NET_ERR(dev, "Failed to create AP interface %s\n", intf_name);
	kfree(ioctl_args);
	return -EINVAL;
}

static ssize_t slsi_delete_interface(struct net_device *dev, char *buffer, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	char *intf_name = NULL;

	ioctl_args = slsi_get_private_command_args(buffer, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	intf_name = ioctl_args->args[0];
	if (strcmp(CONFIG_SCSC_AP_INTERFACE_NAME, intf_name) != 0) {
		SLSI_NET_ERR(dev, "Deletion of %s not allowed!\n", intf_name);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (strcmp(intf_name, CONFIG_SCSC_AP_INTERFACE_NAME) == 0)
		dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2PX_SWLAN);

	if (!dev) {
		SLSI_WARN(sdev, "AP dev is NULL");
		kfree(ioctl_args);
		return -EINVAL;
	}
	ndev_vif = netdev_priv(dev);

	if (ndev_vif->activated)
		slsi_stop_net_dev(sdev, dev);
	slsi_netif_remove_rtlnl_locked(sdev, dev);

	sdev->netdev_ap = NULL;
	SLSI_DBG1_NODEV(SLSI_MLME, "Successfully deleted AP interface %s ", intf_name);

	kfree(ioctl_args);
	return 0;
}
#endif

static ssize_t slsi_get_indoor_channels(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	char op[150] = "";
	char int_string[30] = "";
	int i;
	int len = 0;

	SLSI_DBG1_NODEV(SLSI_MLME, "GET_INDOOR_CHANNELS : %d ", sdev->num_5g_restricted_channels);

	for (i = 0; i < sdev->num_5g_restricted_channels; i++) {
		sprintf(int_string, "%d", sdev->wifi_sharing_5g_restricted_channels[i]);
		strcat(op, int_string);
		strcat(op, " ");
	}

	len = snprintf(command, buf_len, "%d %s", sdev->num_5g_restricted_channels, op);

	return len;
}
#endif

static ssize_t slsi_set_country_rev(struct net_device *dev, char *command, int cmd_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	char              alpha2_rev[] = {0, 0, 0, 0};
	int               status = 0;
	char *country_code = NULL;
	struct slsi_ioctl_args *ioctl_args = NULL;

	ioctl_args = slsi_get_private_command_args(command, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	country_code = ioctl_args->args[0];
	memcpy(alpha2_rev, country_code, strlen(country_code) < 4 ? strlen(country_code) : 4);

	status = slsi_set_country_update_regd(sdev, alpha2_rev, strlen(country_code) < 4 ? strlen(country_code) : 4);

	kfree(ioctl_args);
	return status;
}

static ssize_t slsi_get_country_rev(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	u8                buf[5];
	int               len = 0;

	memset(buf, 0, sizeof(buf));

	len = snprintf(command, buf_len, "%s %c%c %d", CMD_GETCOUNTRYREV,
		       sdev->device_config.domain_info.regdomain->alpha2[0],
		       sdev->device_config.domain_info.regdomain->alpha2[1],
		       sdev->device_config.domain_info.regdomain->dfs_region);

	return len;
}

static ssize_t slsi_freq_band_write(struct net_device *dev, char *command, int cmd_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	uint band = 0;
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
	int                ret = 0;
#endif
	struct slsi_ioctl_args *ioctl_args = NULL;

	ioctl_args = slsi_get_private_command_args(command, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	band = *ioctl_args->args[0] - '0';
	if (band > 2) {
		SLSI_ERR(sdev, "Invalid value : Band Must be 0/1/2 band %d\n", band);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (slsi_is_test_mode_enabled()) {
		slsi_band_update(sdev, band);
		/* Convert to correct Mib value (intra_band:1, all_band:2) */
		kfree(ioctl_args);
		return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_BAND, (band == SLSI_FREQ_BAND_AUTO) ? 2 : 1);
	}

#ifdef CONFIG_SCSC_WLAN_WES_NCHO
	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (sdev->device_config.supported_band == band) {
		SLSI_DBG1_NODEV(SLSI_MLME, "band is already %d\n", band);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return ret;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ret = slsi_mlme_set_band_req(sdev, dev, band);
	if (ret == -EIO) {
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return ret;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	sdev->device_config.supported_band = band;
	slsi_band_cfg_update(sdev, band);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return ret;
#else
	SLSI_ERR(sdev, "NCHO is not supported\n");

	kfree(ioctl_args);
	return -EINVAL;
#endif
}

static ssize_t slsi_freq_band_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               pos = 0;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

#ifdef CONFIG_SCSC_WLAN_WES_NCHO
	if (slsi_is_test_mode_enabled() || sdev->device_config.ncho_mode) {
#else
	if (sdev->device_config.ncho_mode) {
#endif
		pos = scnprintf(command, buf_len, "Band %d", sdev->device_config.supported_band);
	} else {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		pos = -EINVAL;
	}

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	return pos;
}

#ifdef CONFIG_SCSC_WLAN_WES_NCHO
static ssize_t slsi_roam_scan_trigger_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mib_value > -50 || mib_value < -100) {
		SLSI_ERR(sdev, "Invalid trigger_value: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ret = slsi_mlme_set_roaming_parameters(sdev, dev, SLSI_PSID_UNIFI_ROAM_NCHO_RSSI_TRIGGER, mib_value, 1);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_roam_scan_trigger_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_NCHO_RSSI_TRIGGER, &mib_value);
	if (res)
		return res;
	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMTRIGGER, mib_value);
	return res;
}

static ssize_t slsi_roam_delta_trigger_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mib_value > 100 || mib_value < 0) {
		SLSI_ERR(sdev, "Invalid delta trigger_value: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ret = slsi_mlme_set_roaming_parameters(sdev, dev, SLSI_PSID_UNIFI_ROAM_NCHO_RSSI_DELDA, mib_value, 1);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_roam_delta_trigger_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_NCHO_RSSI_DELDA, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMDELTA, mib_value);
	return res;
}

static ssize_t slsi_reassoc_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif   *ndev_vif = netdev_priv(dev);
	struct slsi_dev     *sdev = ndev_vif->sdev;
	u8                  bssid[6] = { 0 };
	int                 channel = 0;
	int                 freq = 0;
	enum nl80211_band band = NL80211_BAND_2GHZ;
	int                 r = 0;
	struct slsi_ioctl_args *ioctl_args = NULL;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 2);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	if (ioctl_args->arg_count != 2) {
		SLSI_ERR(sdev, "Not enough arguments\n");
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (strlen(ioctl_args->args[0]) != 17) {
		SLSI_ERR(sdev, "Invalid MAC address length :%d\n", (int)strlen(ioctl_args->args[0]));
		kfree(ioctl_args);
		return -EINVAL;
	}

	slsi_machexstring_to_macarray(ioctl_args->args[0], bssid);

	if (!slsi_str_to_int(ioctl_args->args[1], &channel)) {
		SLSI_ERR(sdev, "Invalid channel string: '%s'\n", ioctl_args->args[1]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (channel < 1 || channel > 165) {
		SLSI_ERR(sdev, "Invalid channel : %d\n", channel);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (channel > 14)
		band = NL80211_BAND_5GHZ;
	freq = (u16)ieee80211_channel_to_frequency(channel, band);

	ndev_vif = netdev_priv(dev);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	r = slsi_mlme_roam(sdev, dev, bssid, freq);

	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return r;
}

static ssize_t slsi_cached_channel_scan_period_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mib_value > 60 || mib_value < 0) {
		SLSI_ERR(sdev, "Invalid roam scan period: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ret = slsi_mlme_set_roaming_parameters(sdev, dev, SLSI_PSID_UNIFI_NCHO_CACHED_SCAN_PERIOD, mib_value * 1000000, 4);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_cached_channel_scan_period_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_NCHO_CACHED_SCAN_PERIOD, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMSCANPERIOD, mib_value / 1000000);

	return res;
}

static ssize_t slsi_full_roam_scan_period_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value > 600 || mib_value < 0) {
		SLSI_ERR(sdev, "Invalid full roam scan period: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ret = slsi_mlme_set_roaming_parameters(sdev, dev, SLSI_PSID_UNIFI_ROAM_NCHO_FULL_SCAN_PERIOD, mib_value * 1000000, 4);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_full_roam_scan_period_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_NCHO_FULL_SCAN_PERIOD, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETFULLROAMSCANPERIOD, mib_value / 1000000);

	return res;
}

static ssize_t slsi_roam_scan_max_active_channel_time_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value > 300 || mib_value < 3) {
		SLSI_ERR(sdev, "Invalid scan channel time: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_MAX_ACTIVE_CHANNEL_TIME, mib_value);
}

static ssize_t slsi_roam_scan_max_active_channel_time_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_MAX_ACTIVE_CHANNEL_TIME, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETSCANCHANNELTIME, mib_value);

	return res;
}

static ssize_t slsi_roam_scan_probe_interval_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value > 10 || mib_value < 1) {
		SLSI_ERR(sdev, "Invalid scan probes: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_NPROBE, mib_value);
}

static ssize_t slsi_roam_scan_probe_interval_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_NPROBE, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETSCANNPROBES, mib_value);

	return res;
}

static ssize_t slsi_roam_mode_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (slsi_is_rf_test_mode_enabled()) {
		SLSI_DBG1_NODEV(SLSI_MLME, "SLSI_PSID_UNIFI_ROAM_MODE is not supported because of rf test mode.\n");
		kfree(ioctl_args);
		return -ENOTSUPP;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value < 0 || mib_value > 2) {
		SLSI_ERR(sdev, "Invalid roam mode value: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_MODE, mib_value);
}

static ssize_t slsi_roam_mode_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_MODE, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMMODE, mib_value);

	return res;
}

static int slsi_roam_offload_ap_list(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif          *ndev_vif   = netdev_priv(dev);
	struct slsi_dev            *sdev       = ndev_vif->sdev;
	struct cfg80211_acl_data   *mac_acl;
	struct slsi_ioctl_args     *ioctl_args = NULL;
	int                        ap_count    = 0;
	int                        i, r;
	int                        malloc_len;

	slsi_convert_space_seperation(command, buf_len);
	ioctl_args = slsi_get_private_command_args(command, buf_len, 101);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	/* command format:
	 *     x,aa:bb:cc:dd:ee:ff,xx:yy:zz:qq:ww:ee...
	 *     x = 1 to 100
	 *     each mac address id 17 bytes and every mac address is separated by ','
	 */
	if (!slsi_str_to_int(ioctl_args->args[0], &ap_count)) {
			SLSI_ERR(sdev, "Invalid ap_count string: '%s'\n", ioctl_args->args[0]);
			kfree(ioctl_args);
			return -EINVAL;
	}
	if (ap_count < ROAMOFFLAPLIST_MIN || ap_count > ROAMOFFLAPLIST_MAX) {
		SLSI_ERR(sdev, "Invalid ap_count: %d\n", ap_count);
		kfree(ioctl_args);
		return -EINVAL;
	}
	/* each mac address takes 18 bytes(17 for mac address and 1 for ',') except the last one.
	 * the last mac address is just 17 bytes(without a coma)
	 */
	if ((ioctl_args->arg_count - 1) < ap_count) {
		SLSI_ERR(sdev, "Buffer doesn't have enough fields ap_count: %d\n", ap_count);
		kfree(ioctl_args);
		return -EINVAL;
	}
	malloc_len = sizeof(struct cfg80211_acl_data) + sizeof(struct mac_address) * ap_count;
	mac_acl = kmalloc(malloc_len, GFP_KERNEL);
	if (!mac_acl) {
		SLSI_ERR(sdev, "MEM fail for size:%ld\n", sizeof(struct cfg80211_acl_data) + sizeof(struct mac_address) * ap_count);
		kfree(ioctl_args);
		return -ENOMEM;
	}

	for (i = 1; i <= ap_count; i++) {
		if (strlen(ioctl_args->args[i]) != 17) {
			SLSI_ERR(sdev, "Invalid MAC address length :%d\n", (int)strlen(ioctl_args->args[i]));
			kfree(ioctl_args);
			kfree(mac_acl);
			return -EINVAL;
		}
		slsi_machexstring_to_macarray(ioctl_args->args[i], mac_acl->mac_addrs[i].addr);
		SLSI_DBG3_NODEV(SLSI_MLME, "[%pM]", mac_acl->mac_addrs[i].addr);
	}
	mac_acl->acl_policy = NL80211_ACL_POLICY_DENY_UNLESS_LISTED;
	mac_acl->n_acl_entries = ap_count;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	r = slsi_mlme_set_acl(sdev, dev, ndev_vif->ifnum, mac_acl);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(mac_acl);
	kfree(ioctl_args);
	return r;
}

static ssize_t slsi_roam_scan_band_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value != 1 && mib_value != 2) {
		SLSI_ERR(sdev, "Invalid roam scan band value: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_BAND, mib_value);
}

static ssize_t slsi_roam_scan_band_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_BAND, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMINTRABAND, mib_value);

	return res;
}

static ssize_t slsi_roam_scan_control_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif      *ndev_vif   = netdev_priv(dev);
	struct slsi_dev        *sdev       = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                    mode        = 0;
	int                    res         = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		res = -EINVAL;
		goto exit;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		res = -EINVAL;
		goto exit;
	}

	if (mode == 0 || mode == 1) {
		sdev->device_config.roam_scan_mode = mode;
	} else {
		SLSI_ERR(sdev, "Invalid roam Mode: Must be 0 or, 1 Not '%d'\n", mode);
		res = -EINVAL;
		goto exit;
	}

	res = slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_CONTROL, sdev->device_config.roam_scan_mode);
	if (res)
		goto exit;

	/* If the mode is 0, Clear the roam cache */
	if (!mode) {
		memset(&sdev->device_config.wes_roam_scan_list, 0, sizeof(struct slsi_wes_mode_roam_scan_channels));
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		res = slsi_mlme_set_cached_channels(sdev, dev, 0, NULL);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return res;
	}

	exit:
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	kfree(ioctl_args);
	return res;
}

static ssize_t slsi_roam_scan_control_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_CONTROL, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETROAMSCANCONTROL, mib_value);

	return res;
}

static ssize_t slsi_roam_scan_home_time_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value > 300 || mib_value < 3) {
		SLSI_ERR(sdev, "Invalid scan home time: '%d'\n", mib_value);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_HOME_TIME, mib_value);
}

static ssize_t slsi_roam_scan_home_time_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_HOME_TIME, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETSCANHOMETIME, mib_value);

	return res;
}

static ssize_t slsi_roam_scan_home_away_time_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int mib_value = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	if (!slsi_str_to_int(ioctl_args->args[0], &mib_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (mib_value > 300 || mib_value < 3) {
		SLSI_ERR(sdev, "Invalid scan home away time: '%d'\n", mib_value);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	return slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_HOME_AWAY_TIME, mib_value);
}

static ssize_t slsi_roam_scan_home_away_time_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_SCAN_HOME_AWAY_TIME, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETSCANHOMEAWAYTIME, mib_value);

	return res;
}

static ssize_t slsi_roam_scan_channels_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               result = 0;
	int               i, channel_count = 0;
	int               channels[SLSI_NCHO_MAX_CHANNEL_LIST];
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 21);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &channel_count)) {
		SLSI_ERR(sdev, "Invalid channel_count string: '%s'\n", ioctl_args->args[0]);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (channel_count < 0) {
		SLSI_ERR(sdev, "Invalid channel count : %d\n", channel_count);
		kfree(ioctl_args);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	if (channel_count > SLSI_NCHO_MAX_CHANNEL_LIST)
		channel_count = SLSI_NCHO_MAX_CHANNEL_LIST;

	if ((ioctl_args->arg_count - 1) < channel_count) {
		SLSI_ERR(sdev, "Buffer doesn't have enough fields channel_count: %d\n", channel_count);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	sdev->device_config.wes_roam_scan_list.n = channel_count;

	for (i = 0; i < channel_count; i++) {
		if (!slsi_str_to_int(ioctl_args->args[i + 1], &channels[i])) {
			SLSI_ERR(sdev, "failed to read a numeric value\n");
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			kfree(ioctl_args);
			return -EINVAL;
		}
		if (channels[i] < 1 || channels[i] > 165) {
			SLSI_ERR(sdev, "Invalid channel : %d\n", channels[i]);
			kfree(ioctl_args);
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			return -EINVAL;
		}
		sdev->device_config.wes_roam_scan_list.channels[i] = channels[i];
	}

	if (!sdev->device_config.roam_scan_mode) {
		ret = slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_SCAN_CONTROL, 1);
		if (ret != SLSI_MIB_STATUS_SUCCESS) {
			kfree(ioctl_args);
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			return -EINVAL;
		}
		sdev->device_config.roam_scan_mode = 1;
	}

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	result = slsi_mlme_set_cached_channels(sdev, dev, channel_count, sdev->device_config.wes_roam_scan_list.channels);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return result;
}

static ssize_t slsi_roam_scan_channels_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               pos = 0;
	int               i;
	int               channel_count = 0;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}

	channel_count = sdev->device_config.wes_roam_scan_list.n;
	pos = scnprintf(command, buf_len, "%s %d", CMD_GETROAMSCANCHANNELS, channel_count);
	for (i = 0; i < channel_count; i++)
		pos += scnprintf(command + pos, buf_len - pos, " %d", sdev->device_config.wes_roam_scan_list.channels[i]);

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	return pos;
}

static ssize_t slsi_roam_add_scan_channels(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               result = 0;
	int               i, j, new_channel_count = 0;
	int               new_channels[SLSI_NCHO_MAX_CHANNEL_LIST];
	int               curr_channel_count = 0;
	int               found = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 21);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (sdev->device_config.roam_scan_mode) {
		SLSI_ERR(sdev, "ROAM Scan Control must be 0, roam mode = %d\n", sdev->device_config.roam_scan_mode);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (sdev->device_config.wes_roam_scan_list.n == SLSI_NCHO_MAX_CHANNEL_LIST) {
		SLSI_ERR(sdev, "Roam scan list is already full\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	/* Adding Connected channel to wes_roam_scan_list */
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED &&
	    sdev->device_config.wes_roam_scan_list.n == 0) {
		sdev->device_config.wes_roam_scan_list.n = 1;
		sdev->device_config.wes_roam_scan_list.channels[0] = ndev_vif->chan->hw_value;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	if (!slsi_str_to_int(ioctl_args->args[0], &new_channel_count)) {
		SLSI_ERR(sdev, "Invalid channel_count string: '%s'\n", ioctl_args->args[0]);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	if (new_channel_count < 0 || new_channel_count > 20) {
		SLSI_ERR(sdev, "Invalid channel count : %d Range: [0,20]\n", new_channel_count);
		kfree(ioctl_args);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}

	if ((ioctl_args->arg_count - 1) < new_channel_count) {
		SLSI_ERR(sdev, "Buffer doesn't have enough fields channel_count: %d\n", new_channel_count);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	curr_channel_count = sdev->device_config.wes_roam_scan_list.n;

	for (i = 0; i < new_channel_count; i++) {
		if (!slsi_str_to_int(ioctl_args->args[i + 1], &new_channels[i])) {
			SLSI_ERR(sdev, "failed to read a numeric value\n");
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			kfree(ioctl_args);
			return -EINVAL;
		}
		if (new_channels[i] < 1 || new_channels[i] > 165) {
			SLSI_ERR(sdev, "Invalid channel : %d\n", new_channels[i]);
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			kfree(ioctl_args);
			return -EINVAL;
		}
		for (j = 0; j < curr_channel_count; j++) {
			found = 0;
			if (sdev->device_config.wes_roam_scan_list.channels[j] == new_channels[i]) {
				found = 1;
				break;
			}
		}
		if (!found) {
			sdev->device_config.wes_roam_scan_list.channels[curr_channel_count] = new_channels[i];
			curr_channel_count++;
		}
		if (curr_channel_count >= SLSI_NCHO_MAX_CHANNEL_LIST) {
			curr_channel_count = SLSI_NCHO_MAX_CHANNEL_LIST;
			break;
		}
	}

	sdev->device_config.wes_roam_scan_list.n = curr_channel_count;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	result = slsi_mlme_set_cached_channels(sdev, dev, curr_channel_count, sdev->device_config.wes_roam_scan_list.channels);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return result;
}

static ssize_t slsi_okc_mode_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int mode = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode == 0 || mode == 1) {
		sdev->device_config.okc_mode = mode;
	} else {
		SLSI_ERR(sdev, "Invalid OKC Mode: Must be 0 or, 1 Not '%d'\n", mode);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	return 0;
}

static ssize_t slsi_okc_mode_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               okc_mode;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}

	okc_mode = sdev->device_config.okc_mode;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = snprintf(command, buf_len, "%s %d", CMD_GETOKCMODE, okc_mode);

	return res;
}

static ssize_t slsi_wes_mode_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               result = 0;
	u32               action_frame_bmap = SLSI_STA_ACTION_FRAME_BITMAP;
	int mode = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode == 0 || mode == 1) {
		sdev->device_config.wes_mode = mode;
	} else {
		SLSI_ERR(sdev, "Invalid WES Mode: Must be 0 or 1 Not '%d'\n", mode);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (ndev_vif->activated && ndev_vif->vif_type == FAPI_VIFTYPE_STATION &&
	    ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED) {
		if (sdev->device_config.wes_mode)
			action_frame_bmap |= SLSI_ACTION_FRAME_VENDOR_SPEC;

		result = slsi_mlme_register_action_frame(sdev, dev, action_frame_bmap, action_frame_bmap);
	}

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	kfree(ioctl_args);
	return result;
}

static ssize_t slsi_wes_mode_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               wes_mode;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}

	wes_mode = sdev->device_config.wes_mode;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = snprintf(command, buf_len, "%s %d", CMD_GETWESMODE, wes_mode);

	return res;
}

static ssize_t slsi_set_ncho_mode(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	struct sk_buff    *req;
	struct sk_buff    *cfm;
	int               ret = 0;
	int               mode = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode != 0 && mode != 1) {
		SLSI_ERR(sdev, "Invalid NCHO Mode: Must be 0 or 1, mode = %d\n", mode);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (sdev->device_config.ncho_mode == mode) {
		SLSI_INFO(sdev, "ncho_mode is already %d\n", mode);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return ret;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_NET_ERR(dev, "sta is not in connected state\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EPERM;
	}

	SLSI_DBG1_NODEV(SLSI_MLME, "mlme_set_roaming_type_req(vif:%u mode:%u)\n", ndev_vif->ifnum, mode);

	req = fapi_alloc(mlme_set_roaming_type_req, MLME_SET_ROAMING_TYPE_REQ, ndev_vif->ifnum, 0);
	if (!req) {
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EIO;
	}
	fapi_set_u16(req, u.mlme_set_roaming_type_req.vif, ndev_vif->ifnum);
	fapi_set_u16(req, u.mlme_set_roaming_type_req.roaming_type, mode);
	cfm = slsi_mlme_req_cfm(sdev, dev, req, MLME_SET_ROAMING_TYPE_CFM);
	if (!cfm) {
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EIO;
	}

	if (fapi_get_u16(cfm, u.mlme_set_roaming_type_cfm.result_code) != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "mlme_set_roaming_type_cfm(result:0x%04x) ERROR\n",
			     fapi_get_u16(cfm, u.mlme_set_roaming_type_cfm.result_code));
		ret = -EINVAL;
	}

	slsi_kfree_skb(cfm);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	sdev->device_config.ncho_mode = mode;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return ret;
}

static ssize_t slsi_get_ncho_mode(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               ret;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	ret = snprintf(command, buf_len, "%s %d", CMD_GETNCHOMODE, sdev->device_config.ncho_mode);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	return ret;
}

static ssize_t slsi_set_dfs_scan_mode(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int mode = 0;
	int res;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);

	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode >= 0 && mode <= 2) {
		sdev->device_config.dfs_scan_mode = mode;
	} else {
		SLSI_ERR(sdev, "Invalid dfs scan mode: Must be 0/1 or 2, Not '%d'\n", mode);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}

	kfree(ioctl_args);
	res = slsi_set_mib_roam(sdev, NULL, SLSI_PSID_UNIFI_ROAM_DFS_SCAN_MODE, sdev->device_config.dfs_scan_mode);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	return res;
}

static ssize_t slsi_get_dfs_scan_mode(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               mib_value = 0;
	int               res;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.ncho_mode) {
		SLSI_INFO(sdev, "Command not allowed, NCHO is disabled\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	res = slsi_get_mib_roam(sdev, SLSI_PSID_UNIFI_ROAM_DFS_SCAN_MODE, &mib_value);
	if (res)
		return res;

	res = snprintf(command, buf_len, "%s %d", CMD_GETDFSSCANMODE, mib_value);

	return res;
}

#endif

static ssize_t slsi_set_pmk(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	u8                pmk[33] = {0};
	int               result = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (strlen(ioctl_args->args[0]) < 32) {
		kfree(ioctl_args);
		return -EINVAL;
	}

	memcpy((u8 *)pmk, ioctl_args->args[0], 32);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	result = slsi_mlme_set_pmk(sdev, dev, pmk, 32);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	kfree(ioctl_args);
	return result;
}

static ssize_t slsi_auto_chan_read(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int             ap_auto_chan;
	int result = 0;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	ap_auto_chan = sdev->device_config.ap_auto_chan;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	result = snprintf(command, buf_len, "%d\n", ap_auto_chan);
	return result;
}

static ssize_t slsi_auto_chan_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif        *ndev_vif = netdev_priv(dev);
	struct slsi_dev          *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args   *ioctl_args = NULL;
	int                      n_channels = 0;
	struct ieee80211_channel *channels[SLSI_NO_OF_SCAN_CHANLS_FOR_AUTO_CHAN_MAX] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };
	int                      count_channels;
	int                      chan;
#ifdef CONFIG_SCSC_WLAN_WIFI_SHARING
	struct net_device *sta_dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_WLAN);
	struct netdev_vif *ndev_sta_vif  = netdev_priv(sta_dev);
	int sta_frequency;
#endif

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &n_channels)) {
		SLSI_ERR(sdev, "channel count: failed to read a numeric value");
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (n_channels < 1 || n_channels > SLSI_NO_OF_SCAN_CHANLS_FOR_AUTO_CHAN_MAX) {
		SLSI_ERR(sdev, "Invalid channel count:%d", n_channels);
		kfree(ioctl_args);
		return -EINVAL;
	}

	/* If "1 6 11" are passed, scan all "1 - 14" channels. If "1 6" are passed, scan "1 - 9" channels */
	if (n_channels == 3)
		n_channels = 14;
	else if (n_channels == 2)
		n_channels = 9;
	count_channels = 0;
	for (chan = 1; chan <= n_channels; chan++) {
		int center_freq;

		center_freq = ieee80211_channel_to_frequency(chan, NL80211_BAND_2GHZ);
		channels[count_channels] = ieee80211_get_channel(sdev->wiphy, center_freq);
		if (!channels[count_channels])
			SLSI_WARN(sdev, "channel number:%d invalid\n", chan);
		else
			count_channels++;

	}

	SLSI_DBG3(sdev, SLSI_INIT_DEINIT, "Number of channels for autochannel selection= %d\n", count_channels);

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	sdev->device_config.ap_auto_chan = 0;
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

#ifdef CONFIG_SCSC_WLAN_WIFI_SHARING
	if (ndev_sta_vif->activated && ndev_sta_vif->vif_type == FAPI_VIFTYPE_STATION &&
	    (ndev_sta_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTING ||
	     ndev_sta_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED)) {
		sta_frequency = ndev_sta_vif->chan->center_freq;
		SLSI_MUTEX_LOCK(sdev->device_config_mutex);
		if ((sta_frequency / 1000) == 2)
			sdev->device_config.ap_auto_chan = ieee80211_frequency_to_channel(sta_frequency);
		else
			sdev->device_config.ap_auto_chan = 1;
		SLSI_INFO(sdev, "Channel selected = %d", sdev->device_config.ap_auto_chan);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return 0;
	}
#endif /*wifi sharing*/
	kfree(ioctl_args);
	return slsi_auto_chan_select_scan(sdev, count_channels, channels);
}

static ssize_t slsi_send_action_frame(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	char                 *temp;
	u8                   bssid[6] = { 0 };
	int                  channel = 0;
	int                  freq = 0;
	enum ieee80211_band  band = IEEE80211_BAND_2GHZ;
	int                  r = 0;
	u16                  host_tag = slsi_tx_mgmt_host_tag(sdev);
	u32                  dwell_time;
	struct ieee80211_hdr *hdr;
	u8                   *buf = NULL;
	u8                   *final_buf = NULL;
	u8                   temp_byte;
	int                  len = 0;
	int                  final_length = 0;
	int                  i = 0, j = 0;
	struct slsi_ioctl_args *ioctl_args = NULL;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated || ndev_vif->vif_type != FAPI_VIFTYPE_STATION ||
	    ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_ERR(sdev, "Not a STA vif or status is not CONNECTED\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	ioctl_args = slsi_get_private_command_args(command, buf_len, 5);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (ioctl_args->arg_count < 5) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (strlen(ioctl_args->args[0]) != 17) {
		SLSI_ERR(sdev, "Invalid mac address: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	slsi_machexstring_to_macarray(ioctl_args->args[0], bssid);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!SLSI_ETHER_EQUAL(bssid, ndev_vif->sta.bssid)) {
		SLSI_ERR(sdev, "Wrong Bssid = %pM Connected Bssid = %pM\n", bssid, ndev_vif->sta.bssid);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	if (!slsi_str_to_int(ioctl_args->args[1], &channel)) {
		SLSI_ERR(sdev, "Invalid channel string: '%s'\n", ioctl_args->args[1]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (channel < 1 || channel > 165) {
		SLSI_ERR(sdev, "Invalid channel : %d\n", channel);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (channel > 14)
		band = IEEE80211_BAND_5GHZ;
	freq = (u16)ieee80211_channel_to_frequency(channel, band);
	if (!freq) {
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[2], &dwell_time)) {
		SLSI_ERR(sdev, "Invalid dwell time string: '%s'\n", ioctl_args->args[2]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[3], &len)) {
		SLSI_ERR(sdev, "Invalid length string: '%s'\n", ioctl_args->args[3]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	/* if length is less than 512 or greater than 1024 driver will ignore the command */
	if (len < 512 || len > 1024) {
		if (len < 0 || len > 1024) {
			SLSI_ERR(sdev, "Invalid buffer length:%d\n", len);
			kfree(ioctl_args);
			return -EINVAL;
		}
		SLSI_INFO(sdev, "Frame Body of Vendor Specific buffer length = %d so ignoring the command\n", len);
		kfree(ioctl_args);
		return 0;
	}

	buf = kmalloc((len + 1) / 2, GFP_KERNEL);

	if (!buf) {
		SLSI_ERR(sdev, "Malloc  failed\n");
		kfree(ioctl_args);
		return -ENOMEM;
	}

	/*We receive a char buffer, convert to hex*/
	temp = ioctl_args->args[4];
	for (i = 0, j = 0; j < len; j += 2) {
		if (j + 1 == len)
			temp_byte = slsi_parse_hex(temp[j]);
		else
			temp_byte = slsi_parse_hex(temp[j]) << 4 | slsi_parse_hex(temp[j + 1]);
		buf[i++] = temp_byte;
	}
	len = i;

	final_length = len + IEEE80211_HEADER_SIZE;
	final_buf = kmalloc(final_length, GFP_KERNEL);
	if (!final_buf) {
		SLSI_ERR(sdev, "Malloc  failed\n");
		kfree(ioctl_args);
		kfree(buf);
		return -ENOMEM;
	}

	hdr = (struct ieee80211_hdr *)final_buf;
	hdr->frame_control = IEEE80211_FC(IEEE80211_FTYPE_MGMT, IEEE80211_STYPE_ACTION);
	SLSI_ETHER_COPY(hdr->addr1, bssid);
	SLSI_ETHER_COPY(hdr->addr2, dev->dev_addr);
	SLSI_ETHER_COPY(hdr->addr3, bssid);
	memcpy(final_buf + IEEE80211_HEADER_SIZE, buf, len);

	kfree(buf);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	r = slsi_mlme_send_frame_mgmt(sdev, dev, final_buf, final_length, FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME, FAPI_MESSAGETYPE_IEEE80211_ACTION, host_tag, SLSI_FREQ_HOST_TO_FW(freq), dwell_time * 1000, 0);

	kfree(final_buf);
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return r;
}

static int slsi_send_action_frame_cert(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	char                 *temp;
	int                  r = 0;
	u16                  host_tag = slsi_tx_mgmt_host_tag(sdev);
	struct ieee80211_hdr *hdr;
	u8                   *buf = NULL;
	u8                   *final_buf = NULL;
	u8                   temp_byte;
	int                  len = 0;
	int                  final_length = 0;
	int                  i = 0, j = 0;
	struct slsi_ioctl_args *ioctl_args = NULL;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated || ndev_vif->vif_type != FAPI_VIFTYPE_STATION ||
	    ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_ERR(sdev, "Not a STA vif or status is not CONNECTED\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return -EINVAL;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	ioctl_args = slsi_get_private_command_args(command, buf_len, 2);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (ioctl_args->arg_count < 2) {
		SLSI_ERR(sdev, "Invalid argument count = %d\n", ioctl_args->arg_count);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (!slsi_str_to_int(ioctl_args->args[0], &len)) {
		SLSI_ERR(sdev, "Invalid length string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	buf = kmalloc((len + 1) / 2, GFP_KERNEL);

	if (!buf) {
		SLSI_ERR(sdev, "Malloc failed\n");
		kfree(ioctl_args);
		return -ENOMEM;
	}

	/*We receive a char buffer, convert to hex*/
	temp = ioctl_args->args[1];
	for (i = 0, j = 0; j < len; j += 2) {
		if (j + 1 == len)
			temp_byte = slsi_parse_hex(temp[j]);
		else
			temp_byte = slsi_parse_hex(temp[j]) << 4 | slsi_parse_hex(temp[j + 1]);
		buf[i++] = temp_byte;
	}
	len = i;

	final_length = len + IEEE80211_HEADER_SIZE;
	final_buf = kmalloc(final_length, GFP_KERNEL);
	if (!final_buf) {
		SLSI_ERR(sdev, "Malloc failed\n");
		kfree(ioctl_args);
		kfree(buf);
		return -ENOMEM;
	}

	hdr = (struct ieee80211_hdr *)final_buf;
	hdr->frame_control = IEEE80211_FC(IEEE80211_FTYPE_MGMT, IEEE80211_STYPE_ACTION);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	SLSI_ETHER_COPY(hdr->addr1, ndev_vif->sta.bssid);
	SLSI_ETHER_COPY(hdr->addr3, ndev_vif->sta.bssid);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	SLSI_ETHER_COPY(hdr->addr2, sdev->hw_addr);
	memcpy(final_buf + IEEE80211_HEADER_SIZE, buf, len);

	kfree(buf);

	r = slsi_mlme_send_frame_mgmt(sdev, dev, final_buf, final_length, FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME, FAPI_MESSAGETYPE_IEEE80211_ACTION, host_tag, 0, 0, 0);

	kfree(final_buf);
	kfree(ioctl_args);
	return r;
}

static ssize_t slsi_setting_max_sta_write(struct net_device *dev, char *command, int cmd_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_mib_data mib_data = { 0, NULL };
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  sta_number = 0;
	int                  result = 0;

	slsi_convert_space_seperation(command, cmd_len);
	ioctl_args = slsi_get_private_command_args(command, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &sta_number)) {
		SLSI_ERR(sdev, "Invalid max num sta sting: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (sta_number > 10 || sta_number < 1) {
		kfree(ioctl_args);
		return -EINVAL;
	}

	result = slsi_mib_encode_uint(&mib_data, SLSI_PSID_UNIFI_MAX_CLIENT, sta_number, 0);
	if (result != SLSI_MIB_STATUS_SUCCESS || mib_data.dataLength == 0) {
		kfree(ioctl_args);
		return -ENOMEM;
	}
	result = slsi_mlme_set(sdev, dev, mib_data.data, mib_data.dataLength);
	if (result != 0)
		SLSI_ERR(sdev, "max_sta: mlme_set_req failed: Result code: %d\n", result);
	kfree(ioctl_args);
	kfree(mib_data.data);

	return result;
}

static ssize_t slsi_country_write(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *netdev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = netdev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	char              alpha2_code[SLSI_COUNTRY_CODE_LEN];
	int               status;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (strlen(ioctl_args->args[0]) < 2) {
		kfree(ioctl_args);
		return -EINVAL;
	}

	memcpy(alpha2_code, ioctl_args->args[0], 2);
	alpha2_code[2] = ' '; /* set 3rd byte of countrycode to ASCII space */

	status = slsi_set_country_update_regd(sdev, alpha2_code, SLSI_COUNTRY_CODE_LEN);

	kfree(ioctl_args);
	return status;
}

#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
static ssize_t slsi_forward_beacon(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif *netdev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = netdev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int               intended_action = 0;
	int               ret = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (strlen(ioctl_args->args[0]) == 4 && strncasecmp(ioctl_args->args[0], "stop", 4) == 0) {
		intended_action = FAPI_WIPSACTION_STOP;
	} else if (strlen(ioctl_args->args[0]) == 5 && strncasecmp(ioctl_args->args[0], "start", 5) == 0) {
		intended_action = FAPI_WIPSACTION_START;
	} else {
		SLSI_NET_ERR(dev, "BEACON_RECV should be used with start or stop\n");
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_NET_DBG2(dev, SLSI_MLME, "BEACON_RECV %s!!\n", intended_action ? "START" : "STOP");
	SLSI_MUTEX_LOCK(netdev_vif->vif_mutex);

	if (!netdev_vif->activated || netdev_vif->vif_type != FAPI_VIFTYPE_STATION ||
	    netdev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_ERR(sdev, "Not a STA vif or status is not CONNECTED\n");
		ret = -EINVAL;
		goto exit_vif_mutex;
	}

	if (((intended_action == FAPI_WIPSACTION_START) && netdev_vif->is_wips_running) ||
	    ((intended_action == FAPI_WIPSACTION_STOP) && !netdev_vif->is_wips_running)) {
		SLSI_NET_INFO(dev, "Forwarding beacon is already %s!!\n",
			      netdev_vif->is_wips_running ? "running" : "stopped");
		ret = 0;
		goto exit_vif_mutex;
	}

	SLSI_MUTEX_LOCK(netdev_vif->scan_mutex);
	if (intended_action == FAPI_WIPSACTION_START &&
	    (netdev_vif->scan[SLSI_SCAN_HW_ID].scan_req || netdev_vif->sta.roam_in_progress)) {
		SLSI_NET_ERR(dev, "Rejecting BEACON_RECV start as scan/roam is running\n");
		ret = -EBUSY;
		goto exit_scan_mutex;
	}

	ret = slsi_mlme_set_forward_beacon(sdev, dev, intended_action);
exit_scan_mutex:
	SLSI_MUTEX_UNLOCK(netdev_vif->scan_mutex);
exit_vif_mutex:
	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(netdev_vif->vif_mutex);
	return ret;
}
#endif

static ssize_t slsi_update_rssi_boost(struct net_device *dev, char *buffer, int buf_len)
{
	struct netdev_vif *netdev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = netdev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int digit1, digit2, band, lendigit1, lendigit2;
	int boost = 0, length = 0, i = 0;
	char *rssi_boost_string = NULL;

	ioctl_args = slsi_get_private_command_args(buffer, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	rssi_boost_string = ioctl_args->args[0];
	if (strlen(rssi_boost_string) < 8) {
		kfree(ioctl_args);
		return -EINVAL;
	}
	for (i = 0; i < (strlen(rssi_boost_string) - 4);) {
		if (rssi_boost_string[i] == '0' &&
		    rssi_boost_string[i + 1] == '4') {
			if (rssi_boost_string[i + 2] == '0' &&
			    rssi_boost_string[i + 3] == '2' &&
			    ((i + 7) < strlen(rssi_boost_string))) {
				i = i + 4;
			} else {
				kfree(ioctl_args);
				return -EINVAL;
			}
			digit1 = slsi_parse_hex(rssi_boost_string[i]);
			digit2 = slsi_parse_hex(rssi_boost_string[i + 1]);
			boost = (digit1 * 16) + digit2;
			band = rssi_boost_string[i + 3] - '0';
			SLSI_MUTEX_LOCK(sdev->device_config_mutex);
			if (band == 0) {
				sdev->device_config.rssi_boost_2g = 0;
				sdev->device_config.rssi_boost_5g = 0;
			} else if (band == 1) {
				sdev->device_config.rssi_boost_2g = 0;
				sdev->device_config.rssi_boost_5g = boost;
			} else if (band == 2) {
				sdev->device_config.rssi_boost_2g = boost;
				sdev->device_config.rssi_boost_5g = 0;
			} else {
				SLSI_ERR(sdev, "Invalid band value %d\n", band);
				SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
				kfree(ioctl_args);
				return -EINVAL;
			}
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			if (netdev_vif->activated &&
			    netdev_vif->vif_type == FAPI_VIFTYPE_STATION) {
				kfree(ioctl_args);
				return slsi_set_boost(sdev, dev);
			}
			kfree(ioctl_args);
			return 0;
		}
		i = i + 2;
		lendigit1 = slsi_parse_hex(rssi_boost_string[i]);
		lendigit2 = slsi_parse_hex(rssi_boost_string[i + 1]);
		length = (lendigit1 * 16) + lendigit2;
		i = i + (length * 2) + 2;
	}
	kfree(ioctl_args);
	return -EINVAL;
}

int slsi_set_tx_power_calling(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  mode = 0;
	int                  error = 0;
	u8                   host_state;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	/*
	 * 0       : Enable SAR Active Host State
	 * 1 or -1 : Disable SAR Active Host State
	 */
	if (mode != 0 && mode != 1 && mode != -1) {
		SLSI_ERR(sdev, "Invalid mode: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	host_state = sdev->device_config.host_state;

	if (!mode)
		host_state = host_state | SLSI_HOSTSTATE_SAR_ACTIVE;
	else
		host_state = host_state & ~SLSI_HOSTSTATE_SAR_ACTIVE;

	error = slsi_mlme_set_host_state(sdev, dev, host_state);
	if (!error)
		sdev->device_config.host_state = host_state;

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return error;
}

int slsi_set_tx_power_sar(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  mode = 0;
	int                  error = 0;
	u8                   host_state;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode < 0 || mode > 4) {
		SLSI_ERR(sdev, "Invalid mode: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	host_state = sdev->device_config.host_state;
	host_state &= ~(SLSI_HOSTSTATE_SAR_ACTIVE | BIT(3) | BIT(4));

	if (mode)
		host_state |= ((mode - 1) << 3) | SLSI_HOSTSTATE_SAR_ACTIVE;

	error = slsi_mlme_set_host_state(sdev, dev, host_state);
	if (!error)
		sdev->device_config.host_state = host_state;

	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return error;
}

int slsi_get_tx_power_sar(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif        *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	int len = 0;
	u8                   host_state, index;

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	host_state = sdev->device_config.host_state;

	if (host_state & SLSI_HOSTSTATE_SAR_ACTIVE)
		index = ((host_state >> 3) & 3) + 1;
	else
		index = 0;

	len = snprintf(command, buf_len, "%u", index);
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	return len;
}

static int slsi_print_regulatory(struct slsi_802_11d_reg_domain *domain_info, char *buf, int buf_len, struct slsi_supported_channels *supported_channels, int supp_chan_length)
{
	int  cur_pos = 0;
	int  i, j, k;
	char *dfs_region_str[] = {"unknown", "FCC", "ETSI", "JAPAN", "GLOBAL", "CHINA"};
	u8   dfs_region_index;
	struct ieee80211_reg_rule *reg_rule;
	int  channel_start_freq = 0;
	int  channel_end_freq = 0;
	int  channel_start_num = 0;
	int  channel_end_num = 0;
	int  channel_count = 0;
	int  channel_increment = 0;
	int  channel_band = 0;
	bool display_pattern = false;

	cur_pos = snprintf(buf, buf_len, "country %c%c:", domain_info->regdomain->alpha2[0],
			   domain_info->regdomain->alpha2[1]);
	dfs_region_index = domain_info->regdomain->dfs_region <= 5 ? domain_info->regdomain->dfs_region : 0;
	cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, "DFS-%s\n", dfs_region_str[dfs_region_index]);
	for (i = 0; i < domain_info->regdomain->n_reg_rules; i++) {
		reg_rule = &domain_info->regdomain->reg_rules[i];
		cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, "\t(%d-%d @ %d), (N/A, %d)",
					reg_rule->freq_range.start_freq_khz/1000,
					reg_rule->freq_range.end_freq_khz/1000,
					reg_rule->freq_range.max_bandwidth_khz/1000,
					MBM_TO_DBM(reg_rule->power_rule.max_eirp));
		if (reg_rule->flags) {
			if (reg_rule->flags & NL80211_RRF_DFS)
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", DFS");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 9))
			if (reg_rule->flags & NL80211_RRF_NO_OFDM)
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", NO_OFDM");
#endif
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
			if (reg_rule->flags & (NL80211_RRF_PASSIVE_SCAN | NL80211_RRF_NO_IBSS))
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", NO_IR");
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
			if (reg_rule->flags & (NL80211_RRF_NO_IR))
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", NO_IR");
#endif
			if (reg_rule->flags & NL80211_RRF_NO_INDOOR)
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", NO_INDOOR");
			if (reg_rule->flags & NL80211_RRF_NO_OUTDOOR)
				cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", NO_OUTDOOR");
		}
		cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, "\n");
	}

	/* Display of Supported Channels for 2.4GHz and 5GHz */
	cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, "Channels:");

	for (i = 0; i < supp_chan_length; i++) {
		channel_start_num = supported_channels[i].start_chan_num;
		channel_count = supported_channels[i].channel_count;
		channel_increment = supported_channels[i].increment;
		channel_band = supported_channels[i].band;
		channel_end_num = channel_start_num + ((channel_count - 1) * channel_increment);
		for (j = channel_start_num; j <= channel_end_num; j += channel_increment) {
			channel_start_freq = (ieee80211_channel_to_frequency(j, channel_band) * 1000) - 10000;
			channel_end_freq = (ieee80211_channel_to_frequency(j, channel_band) * 1000) + 10000;
			for (k = 0; k < domain_info->regdomain->n_reg_rules; k++) {
				reg_rule = &domain_info->regdomain->reg_rules[k];
				if (reg_rule->freq_range.start_freq_khz <= channel_start_freq &&
				    reg_rule->freq_range.end_freq_khz >= channel_end_freq) {
					if (display_pattern)
						cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, ", %d", j);
					else
						cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, " %d", j);
					display_pattern = true;
					break;
				}
			}
		}
	}
	cur_pos += snprintf(buf + cur_pos, buf_len - cur_pos, "\n");
	return cur_pos;
}

static int slsi_get_supported_channels(struct slsi_dev *sdev, struct net_device *dev, struct slsi_supported_channels *supported_channels)
{
	struct slsi_mib_data      mibrsp = { 0, NULL };
	struct slsi_mib_data      supported_chan_mib;
	struct slsi_mib_value     *values = NULL;
	struct slsi_mib_get_entry get_values[] = {{SLSI_PSID_UNIFI_SUPPORTED_CHANNELS, { 0, 0 } } };
	int                       i, chan_count, chan_start;
	int			  supp_chan_length = 0;

	/* Expect each mib length in response is <= 16. So assume 16 bytes for each MIB */
	mibrsp.dataLength = 16;
	mibrsp.data = kmalloc(mibrsp.dataLength, GFP_KERNEL);
	if (!mibrsp.data) {
		SLSI_ERR(sdev, "Cannot kmalloc %d bytes\n", mibrsp.dataLength);
		return 0;
	}
	values = slsi_read_mibs(sdev, dev, get_values, 1, &mibrsp);
	if (!values)
		goto exit_with_mibrsp;

	if (values[0].type != SLSI_MIB_TYPE_OCTET) {
		SLSI_ERR(sdev, "Supported_Chan invalid type.");
		goto exit_with_values;
	}

	supported_chan_mib = values[0].u.octetValue;
	for (i = 0; i < supported_chan_mib.dataLength / 2; i++) {
		chan_start = supported_chan_mib.data[i * 2];
		chan_count = supported_chan_mib.data[i * 2 + 1];
		if (chan_start == 1) { /* for 2.4GHz */
			supported_channels[supp_chan_length].start_chan_num = 1;
			if (!(sdev->device_config.host_state & SLSI_HOSTSTATE_CELLULAR_ACTIVE) &&
			    chan_count > 11 && sdev->device_config.disable_ch12_ch13) {
				chan_count = 11;
				SLSI_DBG1(sdev, SLSI_CFG80211, "Channels 12 and 13 have been disabled");
			}
			supported_channels[supp_chan_length].channel_count = chan_count;
			supported_channels[supp_chan_length].increment = 1;
			supported_channels[supp_chan_length].band = NL80211_BAND_2GHZ;
			supp_chan_length = supp_chan_length + 1;
		} else { /* for 5GHz */
			supported_channels[supp_chan_length].start_chan_num = chan_start;
			supported_channels[supp_chan_length].channel_count = chan_count;
			supported_channels[supp_chan_length].increment = 4;
			supported_channels[supp_chan_length].band = NL80211_BAND_5GHZ;
			supp_chan_length = supp_chan_length + 1;
		}
	}
exit_with_values:
	kfree(values);
exit_with_mibrsp:
	kfree(mibrsp.data);
	return supp_chan_length;
}

static int slsi_get_regulatory(struct net_device *dev, char *buf, int buf_len)
{
	struct netdev_vif              *ndev_vif = netdev_priv(dev);
	struct slsi_dev                *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                            mode = 0;
	int                            cur_pos = 0;
	int                            status;
	u8                             alpha2[3];
	struct slsi_supported_channels supported_channels[5];
	int			       supp_chan_length;

	ioctl_args = slsi_get_private_command_args(buf, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode != 0 && mode != 1) {
		SLSI_ERR(sdev, "Invalid mode: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode == 1) {
		struct slsi_802_11d_reg_domain domain_info;

		memset(&domain_info, 0, sizeof(struct slsi_802_11d_reg_domain));
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		if (!ndev_vif->activated || ndev_vif->vif_type != FAPI_VIFTYPE_STATION || !ndev_vif->sta.sta_bss) {
			snprintf(buf, buf_len, "Station not connected");
			SLSI_ERR(sdev, "station not connected. vif.activated:%d, vif.type:%d, vif.bss:%s\n",
				 ndev_vif->activated, ndev_vif->vif_type, ndev_vif->sta.sta_bss ? "yes" : "no");
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			kfree(ioctl_args);
			return -EINVAL;
		}
		/* read vif specific country code, index = vifid+1 */
		status = slsi_read_default_country(sdev, alpha2, ndev_vif->ifnum + 1);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		if (status) {
			kfree(ioctl_args);
			return status;
		}

		/* max 20 rules */
		domain_info.regdomain = kmalloc(sizeof(*domain_info.regdomain) + sizeof(struct ieee80211_reg_rule) * 20, GFP_KERNEL);
		if (!domain_info.regdomain) {
			SLSI_ERR(sdev, "no memory size:%lu\n",
				 sizeof(struct ieee80211_regdomain) + sizeof(struct ieee80211_reg_rule) * 20);
			kfree(ioctl_args);
			return -ENOMEM;
		}

		/* get regulatory rules based on country code */
		domain_info.countrylist = sdev->device_config.domain_info.countrylist;
		domain_info.country_len = sdev->device_config.domain_info.country_len;
		status = slsi_read_regulatory_rules(sdev, &domain_info, alpha2);
		if (status) {
			kfree(domain_info.regdomain);
			kfree(ioctl_args);
			return status;
		}
		/* get supported channels based on country code */
		supp_chan_length = slsi_get_supported_channels(sdev, dev, &supported_channels[0]);
		cur_pos += slsi_print_regulatory(&domain_info, buf + cur_pos, buf_len - cur_pos, &supported_channels[0], supp_chan_length);
		kfree(domain_info.regdomain);
	} else if (mode == 0) {
		SLSI_MUTEX_LOCK(sdev->device_config_mutex);
		supp_chan_length = slsi_get_supported_channels(sdev, dev, &supported_channels[0]);
		cur_pos += slsi_print_regulatory(&sdev->device_config.domain_info, buf + cur_pos, buf_len - cur_pos, &supported_channels[0], supp_chan_length);
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
	}
	/* Buf is somewhere close to 4Kbytes. so expect some spare space. If there is no spare
	 * space we might have missed printing some text in buf.
	 */
	kfree(ioctl_args);
	if (buf_len - cur_pos)
		return cur_pos;
	else
		return -ENOMEM;
}

void slsi_disable_ch12_13(struct slsi_dev *sdev)
{
	struct wiphy *wiphy = sdev->wiphy;
	struct ieee80211_channel *chan;

	if (wiphy->bands[0]) {
		chan = &wiphy->bands[0]->channels[11];
		chan->flags |= IEEE80211_CHAN_DISABLED;
		chan = &wiphy->bands[0]->channels[12];
		chan->flags |= IEEE80211_CHAN_DISABLED;
	}

	SLSI_DBG1(sdev, SLSI_CFG80211, "Channels 12 and 13 have been disabled");
}

int slsi_set_fcc_channel(struct net_device *dev, char *cmd, int cmd_len)
{
	struct netdev_vif      *ndev_vif = netdev_priv(dev);
	struct slsi_dev        *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                    status;
	int                    fcc_channel_value;
	u8                     host_state;

	ioctl_args = slsi_get_private_command_args(cmd, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &fcc_channel_value)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (fcc_channel_value != 0 && fcc_channel_value != -1) {
		SLSI_ERR(sdev, "Invalid value of flight_mode_ena: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	host_state = sdev->device_config.host_state;

	/* SET_FCC_CHANNEL 0 indicates flight mode is enabled */
	/* SET_FCC_CHANNEL -1 indicates flight mode is disabled */
	if (fcc_channel_value == 0)
		host_state = host_state & ~SLSI_HOSTSTATE_CELLULAR_ACTIVE;
	else
		host_state = host_state | SLSI_HOSTSTATE_CELLULAR_ACTIVE;
	sdev->device_config.host_state = host_state;

	status = slsi_mlme_set_host_state(sdev, dev, host_state);
	if (status) {
		SLSI_ERR(sdev, "Err setting MMaxPowerEna. error = %d\n", status);
	} else {
		if (fcc_channel_value == 0 && sdev->device_config.disable_ch12_ch13)
			slsi_disable_ch12_13(sdev);
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	kfree(ioctl_args);
	return status;
}

int slsi_fake_mac_write(struct net_device *dev, char *cmd, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_mib_data mib_data = { 0, NULL };
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  status;
	bool                enable;

	ioctl_args = slsi_get_private_command_args(cmd, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (strlen(ioctl_args->args[0]) == 2 && strncmp(ioctl_args->args[0], "ON", strlen("ON")) == 0) {
		enable = 1;
	} else if (strlen(ioctl_args->args[0]) == 3 && strncmp(ioctl_args->args[0], "OFF", strlen("OFF")) == 0) {
		enable = 0;
	} else {
		SLSI_ERR(sdev, "Invalid parameter: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	status = slsi_mib_encode_bool(&mib_data, SLSI_PSID_UNIFI_MAC_ADDRESS_RANDOMISATION, enable, 0);
	if (status != SLSI_MIB_STATUS_SUCCESS) {
		SLSI_ERR(sdev, "FAKE MAC FAIL: no mem for MIB\n");
		kfree(ioctl_args);
		return -ENOMEM;
	}

	status = slsi_mlme_set(sdev, NULL, mib_data.data, mib_data.dataLength);

	kfree(mib_data.data);

	if (status)
		SLSI_ERR(sdev, "Err setting unifiMacAddrRandomistaion MIB. error = %d\n", status);

	kfree(ioctl_args);
	return status;
}

static char *slsi_get_assoc_status(u16 fw_result_code)
{
	char *assoc_status_label = "unspecified_failure";

	switch (fw_result_code) {
	case FAPI_RESULTCODE_SUCCESS:
		assoc_status_label = "success";
		break;
	case FAPI_RESULTCODE_TRANSMISSION_FAILURE:
		assoc_status_label = "transmission_failure";
		break;
	case FAPI_RESULTCODE_HOST_REQUEST_SUCCESS:
		assoc_status_label = "host_request_success";
		break;
	case FAPI_RESULTCODE_HOST_REQUEST_FAILED:
		assoc_status_label = "host_request_failed";
		break;
	case FAPI_RESULTCODE_PROBE_TIMEOUT:
		assoc_status_label = "probe_timeout";
		break;
	case FAPI_RESULTCODE_AUTH_TIMEOUT:
		assoc_status_label = "auth_timeout";
		break;
	case FAPI_RESULTCODE_ASSOC_TIMEOUT:
		assoc_status_label = "assoc_timeout";
		break;
	case FAPI_RESULTCODE_ASSOC_ABORT:
		assoc_status_label = "assoc_abort";
		break;
	}
	return assoc_status_label;
}

int slsi_get_sta_info(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif   *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	int                       len;
#ifdef CONFIG_SCSC_WLAN_WIFI_SHARING
	struct net_device *ap_dev;
	struct netdev_vif *ndev_ap_vif;

	ap_dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2PX_SWLAN);

	if (ap_dev) {
		ndev_ap_vif = netdev_priv(ap_dev);
		SLSI_MUTEX_LOCK(ndev_ap_vif->vif_mutex);
		if (SLSI_IS_VIF_INDEX_MHS(sdev, ndev_ap_vif))
			ndev_vif = ndev_ap_vif;
		SLSI_MUTEX_UNLOCK(ndev_ap_vif->vif_mutex);
	}
#endif

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (!ndev_vif->activated || ndev_vif->vif_type != FAPI_VIFTYPE_AP) {
		SLSI_ERR(sdev, "slsi_get_sta_info: AP is not up.Command not allowed vif.activated:%d, vif.type:%d\n",
			 ndev_vif->activated, ndev_vif->vif_type);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return -EINVAL;
	}

#if defined(SCSC_SEP_VERSION) && (SCSC_SEP_VERSION >= 9)
	len = snprintf(command, buf_len, "GETSTAINFO %pM Rx_Retry_Pkts=%d Rx_BcMc_Pkts=%d CAP=%04x %02x:%02x:%02x ",
		       ndev_vif->ap.last_disconnected_sta.address,
		       ndev_vif->ap.last_disconnected_sta.rx_retry_packets,
		       ndev_vif->ap.last_disconnected_sta.rx_bc_mc_packets,
		       ndev_vif->ap.last_disconnected_sta.capabilities,
		       ndev_vif->ap.last_disconnected_sta.address[0],
		       ndev_vif->ap.last_disconnected_sta.address[1],
		       ndev_vif->ap.last_disconnected_sta.address[2]);

	len += snprintf(&command[len], (buf_len - len), "%d %d %d %d %d %d %d %u %d",
		       ieee80211_frequency_to_channel(ndev_vif->ap.channel_freq),
		       ndev_vif->ap.last_disconnected_sta.bandwidth, ndev_vif->ap.last_disconnected_sta.rssi,
		       ndev_vif->ap.last_disconnected_sta.tx_data_rate, ndev_vif->ap.last_disconnected_sta.mode,
		       ndev_vif->ap.last_disconnected_sta.antenna_mode,
		       ndev_vif->ap.last_disconnected_sta.mimo_used, ndev_vif->ap.last_disconnected_sta.reason,
		       ndev_vif->ap.last_disconnected_sta.support_mode);
#else
	len = snprintf(command, buf_len, "wl_get_sta_info : %02x%02x%02x %u %d %d %d %d %d %d %u ",
		       ndev_vif->ap.last_disconnected_sta.address[0], ndev_vif->ap.last_disconnected_sta.address[1],
		       ndev_vif->ap.last_disconnected_sta.address[2], ndev_vif->ap.channel_freq,
		       ndev_vif->ap.last_disconnected_sta.bandwidth, ndev_vif->ap.last_disconnected_sta.rssi,
		       ndev_vif->ap.last_disconnected_sta.tx_data_rate, ndev_vif->ap.last_disconnected_sta.mode,
		       ndev_vif->ap.last_disconnected_sta.antenna_mode,
		       ndev_vif->ap.last_disconnected_sta.mimo_used, ndev_vif->ap.last_disconnected_sta.reason);
#endif

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return len;
}

static int slsi_get_bss_rssi(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif        *ndev_vif = netdev_priv(dev);
	int len = 0;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	len = snprintf(command, buf_len, "%d", ndev_vif->sta.last_connected_bss.rssi);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return len;
}

static int slsi_get_bss_info(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif        *ndev_vif = netdev_priv(dev);
	int len = 0;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	len = snprintf(command, buf_len, "%02x:%02x:%02x %u %u %d %u %u %u %u %u %d %d %u %u %u %u",
		       ndev_vif->sta.last_connected_bss.address[0],  ndev_vif->sta.last_connected_bss.address[1],
		       ndev_vif->sta.last_connected_bss.address[2],
		       ndev_vif->sta.last_connected_bss.channel_freq, ndev_vif->sta.last_connected_bss.bandwidth,
		       ndev_vif->sta.last_connected_bss.rssi, ndev_vif->sta.last_connected_bss.tx_data_rate,
		       ndev_vif->sta.last_connected_bss.mode, ndev_vif->sta.last_connected_bss.antenna_mode,
		       ndev_vif->sta.last_connected_bss.mimo_used,
		       ndev_vif->sta.last_connected_bss.passpoint_version, ndev_vif->sta.last_connected_bss.snr,
		       ndev_vif->sta.last_connected_bss.noise_level, ndev_vif->sta.last_connected_bss.roaming_akm,
		       ndev_vif->sta.last_connected_bss.roaming_count, ndev_vif->sta.last_connected_bss.kv,
		       ndev_vif->sta.last_connected_bss.kvie);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return len;
}

#ifdef CONFIG_SCSC_WLAN_MAX_LINK_SPEED
static int slsi_get_linkspeed(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	int len = 0;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated) {
		SLSI_ERR(sdev, "Not Activated, Command not allowed vif.activated:%d, vif.type:%d\n",
			 ndev_vif->activated, ndev_vif->vif_type);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return -EINVAL;
	}

	if (ndev_vif->vif_type != FAPI_VIFTYPE_STATION && ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_NET_ERR(dev, "sta is not in connected state\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return -EPERM;
	}

	len = snprintf(command, buf_len, "MAX_SPEED %lu CURRENT_LINK_SPEED %lu",
		       ndev_vif->sta.max_rate_mbps, ndev_vif->sta.data_rate_mbps);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	return len;
}
#endif

static int slsi_get_assoc_reject_info(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	int len = 0;

	len = snprintf(command, buf_len, "assoc_reject.status : %d %s", sdev->assoc_result_code,
		       slsi_get_assoc_status(sdev->assoc_result_code));

	return len;
}

#ifdef CONFIG_SCSC_WLAN_LOW_LATENCY_MODE
static int slsi_set_power_mode(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int                  status;
	u16                  power_mode;
	int                  mode;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &mode)) {
		SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	if (mode != 0 && mode != 1) {
		SLSI_ERR(sdev, "Invalid power_mode: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	power_mode = (mode == 0) ? FAPI_POWERMANAGEMENTMODE_ACTIVE_MODE : FAPI_POWERMANAGEMENTMODE_POWER_SAVE;
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->activated || ndev_vif->vif_type != FAPI_VIFTYPE_STATION ||
	    !(ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED)) {
		SLSI_ERR(sdev, "Command not allowed vif.activated:%d, vif.type:%d, ndev_vif->sta.vif_status:%d\n",
			 ndev_vif->activated, ndev_vif->vif_type, ndev_vif->sta.vif_status);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EINVAL;
	}
	status = slsi_mlme_powermgt(sdev, dev, power_mode);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	kfree(ioctl_args);
	return status;
}
#endif

int slsi_set_disconnect_ies(struct net_device *dev, char *cmd, int cmd_len)
{
	struct netdev_vif       *ndev_vif = netdev_priv(dev);
	struct slsi_dev         *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args  *ioctl_args = NULL;
	char                    *disconnect_ies = cmd + strlen(CMD_SET_DISCONNECT_IES) + 1;
	int                     ie_len = 0;
	u8                      *disconnect_ies_bin;
	u8                      temp_byte;
	int                     i;
	int                     j;
	int                     len;

	SLSI_DBG1(sdev, SLSI_CFG80211, "Setting disconnect IE's\n");
	ioctl_args = slsi_get_private_command_args(cmd, cmd_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	disconnect_ies = ioctl_args->args[0];
	ie_len = strlen(ioctl_args->args[0]);

	/* ie_len has been trimmed to even, as odd length would mean that ie is invalid */
	ie_len &= (~0x01);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ndev_vif->sta.vendor_disconnect_ies_len = (ie_len / 2);
	disconnect_ies_bin = kmalloc(ndev_vif->sta.vendor_disconnect_ies_len, GFP_KERNEL);
	if (!disconnect_ies_bin) {
		SLSI_ERR(sdev, "Malloc  failed\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -ENOMEM;
	}

	for (i = 0, j = 0; j < ie_len; j += 2) {
		temp_byte = slsi_parse_hex(disconnect_ies[j]) << 4 | slsi_parse_hex(disconnect_ies[j + 1]);
		disconnect_ies_bin[i++] = temp_byte;
	}

	/* check if the IE is valid */
	for (i = 0; i < ndev_vif->sta.vendor_disconnect_ies_len;) {
		if (disconnect_ies_bin[i] == 0xdd) {
			len = disconnect_ies_bin[i + 1];
			if ((ndev_vif->sta.vendor_disconnect_ies_len - (i + 2)) < len) {
				SLSI_WARN(sdev, "The length of disconnect IE's is not proper\n");
				ndev_vif->sta.vendor_disconnect_ies_len = 0;
				kfree(disconnect_ies_bin);
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
				kfree(ioctl_args);
				return -EINVAL;
			}

			i = i + 2 + len;
		} else {
			SLSI_WARN(sdev, "The tag of disconnect IE's is not proper\n");
			ndev_vif->sta.vendor_disconnect_ies_len = 0;
			kfree(disconnect_ies_bin);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			kfree(ioctl_args);
			return -EINVAL;
		}
	}
	ndev_vif->sta.vendor_disconnect_ies = disconnect_ies_bin;
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	kfree(ioctl_args);
	return 0;
}

#ifdef CONFIG_SCSC_WLAN_STA_ENHANCED_ARP_DETECT
static int slsi_enhanced_arp_start_stop(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif    *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	struct slsi_ioctl_args *ioctl_args = NULL;
	int result = 0;
	int readbyte = 0;
	int readvalue = 0;
	int i = 0;

	ioctl_args = slsi_get_private_command_args(command, buf_len, 1);
	SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

	if (!slsi_str_to_int(ioctl_args->args[0], &readvalue)) {
		SLSI_ERR(sdev, "Invalid start/stop string: '%s'\n", ioctl_args->args[0]);
		kfree(ioctl_args);
		return -EINVAL;
	}

	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	if (!sdev->device_config.fw_enhanced_arp_detect_supported) {
		SLSI_ERR(sdev, "Enhanced ARP Detect Feature is not supported.\n");
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		kfree(ioctl_args);
		return -ENOTSUPP;
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (ndev_vif->vif_type != FAPI_VIFTYPE_STATION) {
		SLSI_ERR(sdev, "Not in STA mode\n");
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		kfree(ioctl_args);
		return -EPERM;
	}

	SLSI_DBG1(sdev, SLSI_CFG80211, "Enhanced ARP Start/Stop\n");

	memset(ndev_vif->target_ip_addr, 0, sizeof(ndev_vif->target_ip_addr));
	if (readvalue != 0) { /* parse IP address */
		for (i = 0; i < 4 ; i++) {
			readbyte = slsi_str_to_int(ioctl_args->args[0], &readvalue);
			if (!readbyte) {
				SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
				memset(ndev_vif->target_ip_addr, 0, sizeof(ndev_vif->target_ip_addr));
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
				kfree(ioctl_args);
				return -EINVAL;
			}
			if (readvalue < 0 || readvalue > 255) {
				SLSI_ERR(sdev, "Invalid value of IP address byte\n");
				memset(ndev_vif->target_ip_addr, 0, sizeof(ndev_vif->target_ip_addr));
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
				kfree(ioctl_args);
				return -EINVAL;
			}
			ndev_vif->target_ip_addr[i] = readvalue;
			if (i == 3)
				break;
			if (strlen(ioctl_args->args[0]) - readbyte - 1 <= 0) {
				SLSI_ERR(sdev, "Invalid IP address\n");
				memset(ndev_vif->target_ip_addr, 0, sizeof(ndev_vif->target_ip_addr));
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
				kfree(ioctl_args);
				return -EINVAL;
			}
			ioctl_args->args[0] = ioctl_args->args[0] + readbyte + 1;
		}
	}

	if (ndev_vif->target_ip_addr[0] != 0) { /* start enhanced arp detect */
		/* reset all the counters in host and firmware */
		slsi_read_enhanced_arp_rx_count_by_lower_mac(sdev, dev, SLSI_PSID_UNIFI_ARP_DETECT_RESPONSE_COUNTER);
		memset(&ndev_vif->enhanced_arp_stats, 0, sizeof(ndev_vif->enhanced_arp_stats));
		memset(ndev_vif->enhanced_arp_host_tag, 0, sizeof(ndev_vif->enhanced_arp_host_tag));
		ndev_vif->enhanced_arp_detect_enabled = true;
		result = slsi_mlme_arp_detect_request(sdev, dev, FAPI_ACTION_START, ndev_vif->target_ip_addr);
	} else { /* stop enhanced arp detect */
		ndev_vif->enhanced_arp_detect_enabled = false;
		result = slsi_mlme_arp_detect_request(sdev, dev, FAPI_ACTION_STOP, ndev_vif->target_ip_addr);
	}

	kfree(ioctl_args);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return result;
}

static int slsi_enhanced_arp_get_stats(struct net_device *dev, char *command, int buf_len)
{
	struct netdev_vif                      *ndev_vif = netdev_priv(dev);
	struct slsi_dev      *sdev = ndev_vif->sdev;
	int                                    r = 0;
	int                                    len = 0;

	r = slsi_read_enhanced_arp_rx_count_by_lower_mac(sdev, dev, SLSI_PSID_UNIFI_ARP_DETECT_RESPONSE_COUNTER);

	if (r == 0) {
		len = snprintf(command, buf_len, "%d %d %d %d %d %d %d %d %d %d",
			       ndev_vif->enhanced_arp_stats.arp_req_count_from_netdev,
			       ndev_vif->enhanced_arp_stats.arp_req_count_to_lower_mac,
			       ndev_vif->enhanced_arp_stats.arp_req_rx_count_by_lower_mac,
			       ndev_vif->enhanced_arp_stats.arp_req_count_tx_success,
			       ndev_vif->enhanced_arp_stats.arp_rsp_rx_count_by_lower_mac,
			       ndev_vif->enhanced_arp_stats.arp_rsp_rx_count_by_upper_mac,
			       ndev_vif->enhanced_arp_stats.arp_rsp_count_to_netdev,
			       ndev_vif->enhanced_arp_stats.arp_rsp_count_out_of_order_drop,
			       0, /*ap_link_active not supported, set as 0*/
			       ndev_vif->enhanced_arp_stats.is_duplicate_addr_detected);
		SLSI_DBG2(sdev, SLSI_CFG80211, "Enhanced ARP Stats: %s\n", command);
	}

	/*clear all the counters*/
	memset(&ndev_vif->enhanced_arp_stats, 0, sizeof(ndev_vif->enhanced_arp_stats));

	return len;
}
#endif

int slsi_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#define MAX_LEN_PRIV_COMMAND    4096 /*This value is the max reply size set in supplicant*/
	struct android_wifi_priv_cmd priv_cmd;
	int                          ret = 0;
	u8                           *command = NULL;

	if (!dev) {
		ret = -ENODEV;
		goto exit;
	}

	if (!rq->ifr_data) {
		ret = -EINVAL;
		goto exit;
	}
	if (copy_from_user((void *)&priv_cmd, (void *)rq->ifr_data, sizeof(struct android_wifi_priv_cmd))) {
		ret = -EFAULT;
		SLSI_NET_ERR(dev, "ifr data failed\n");
		goto exit;
	}

	if (priv_cmd.total_len > MAX_LEN_PRIV_COMMAND || priv_cmd.total_len < 0) {
		ret = -EINVAL;
		SLSI_NET_ERR(dev, "Length mismatch total_len = %d\n", priv_cmd.total_len);
		goto exit;
	}
	command = kmalloc((priv_cmd.total_len + 1), GFP_KERNEL);
	if (!command) {
		ret = -ENOMEM;
		SLSI_NET_ERR(dev, "No memory\n");
		goto exit;
	}
	if (copy_from_user(command, priv_cmd.buf, priv_cmd.total_len)) {
		ret = -EFAULT;
		SLSI_NET_ERR(dev, "Buffer copy fail\n");
		goto exit;
	}
	command[priv_cmd.total_len] = '\0';

	SLSI_INFO_NODEV("command: %.*s\n", priv_cmd.total_len, command);

	if (strncasecmp(command, CMD_SETSUSPENDMODE, strlen(CMD_SETSUSPENDMODE)) == 0) {
			ret = slsi_set_suspend_mode(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETJOINPREFER, strlen(CMD_SETJOINPREFER)) == 0) {
		ret = slsi_update_rssi_boost(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_RXFILTERADD, strlen(CMD_RXFILTERADD)) == 0) {
		ret = slsi_rx_filter_num_write(dev, command, priv_cmd.total_len, 1);
	} else if (strncasecmp(command, CMD_RXFILTERREMOVE, strlen(CMD_RXFILTERREMOVE)) == 0) {
		ret = slsi_rx_filter_num_write(dev, command, priv_cmd.total_len, 0);
#ifdef CONFIG_SCSC_WLAN_WIFI_SHARING
#if !defined(CONFIG_SCSC_WLAN_MHS_STATIC_INTERFACE) || (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION < 9)
	} else if (strncasecmp(command, CMD_INTERFACE_CREATE, strlen(CMD_INTERFACE_CREATE)) == 0) {
		ret = slsi_create_interface(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_INTERFACE_DELETE, strlen(CMD_INTERFACE_DELETE)) == 0) {
		ret = slsi_delete_interface(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SET_INDOOR_CHANNELS, strlen(CMD_SET_INDOOR_CHANNELS)) == 0) {
		ret = slsi_set_mib_wifi_sharing_5ghz_channel(dev, SLSI_PSID_UNIFI_WI_FI_SHARING5_GHZ_CHANNEL, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GET_INDOOR_CHANNELS, strlen(CMD_GET_INDOOR_CHANNELS)) == 0) {
		ret = slsi_get_indoor_channels(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SETCOUNTRYREV, strlen(CMD_SETCOUNTRYREV)) == 0) {
		ret = slsi_set_country_rev(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETCOUNTRYREV, strlen(CMD_GETCOUNTRYREV)) == 0) {
		ret = slsi_get_country_rev(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMBAND, strlen(CMD_SETROAMBAND)) == 0) {
		ret = slsi_freq_band_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETBAND, strlen(CMD_SETBAND)) == 0) {
		ret = slsi_freq_band_write(dev, command, priv_cmd.total_len);
	} else if ((strncasecmp(command, CMD_GETROAMBAND, strlen(CMD_GETROAMBAND)) == 0) ||
		   (strncasecmp(command, CMD_GETBAND, strlen(CMD_GETBAND)) == 0)) {
			ret = slsi_freq_band_read(dev, command, priv_cmd.total_len);
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
	} else if (strncasecmp(command, CMD_SETROAMTRIGGER, strlen(CMD_SETROAMTRIGGER)) == 0) {
		ret = slsi_roam_scan_trigger_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMTRIGGER, strlen(CMD_GETROAMTRIGGER)) == 0) {
		ret = slsi_roam_scan_trigger_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMDELTA, strlen(CMD_SETROAMDELTA)) == 0) {
		ret = slsi_roam_delta_trigger_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMDELTA, strlen(CMD_GETROAMDELTA)) == 0) {
		ret = slsi_roam_delta_trigger_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMSCANPERIOD, strlen(CMD_SETROAMSCANPERIOD)) == 0) {
		ret = slsi_cached_channel_scan_period_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMSCANPERIOD, strlen(CMD_GETROAMSCANPERIOD)) == 0) {
		ret = slsi_cached_channel_scan_period_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETFULLROAMSCANPERIOD, strlen(CMD_SETFULLROAMSCANPERIOD)) == 0) {
		ret = slsi_full_roam_scan_period_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETFULLROAMSCANPERIOD, strlen(CMD_GETFULLROAMSCANPERIOD)) == 0) {
		ret = slsi_full_roam_scan_period_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETSCANCHANNELTIME, strlen(CMD_SETSCANCHANNELTIME)) == 0) {
		ret = slsi_roam_scan_max_active_channel_time_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETSCANCHANNELTIME, strlen(CMD_GETSCANCHANNELTIME)) == 0) {
		ret = slsi_roam_scan_max_active_channel_time_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETSCANNPROBES, strlen(CMD_SETSCANNPROBES)) == 0) {
		ret = slsi_roam_scan_probe_interval_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETSCANNPROBES, strlen(CMD_GETSCANNPROBES)) == 0) {
		ret = slsi_roam_scan_probe_interval_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMMODE, strlen(CMD_SETROAMMODE)) == 0) {
		ret = slsi_roam_mode_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMMODE, strlen(CMD_GETROAMMODE)) == 0) {
		ret = slsi_roam_mode_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMINTRABAND, strlen(CMD_SETROAMINTRABAND)) == 0) {
		ret = slsi_roam_scan_band_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMINTRABAND, strlen(CMD_GETROAMINTRABAND)) == 0) {
		ret = slsi_roam_scan_band_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMSCANCONTROL, strlen(CMD_SETROAMSCANCONTROL)) == 0) {
		ret = slsi_roam_scan_control_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMSCANCONTROL, strlen(CMD_GETROAMSCANCONTROL)) == 0) {
		ret = slsi_roam_scan_control_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETSCANHOMETIME, strlen(CMD_SETSCANHOMETIME)) == 0) {
		ret = slsi_roam_scan_home_time_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETSCANHOMETIME, strlen(CMD_GETSCANHOMETIME)) == 0) {
		ret = slsi_roam_scan_home_time_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETSCANHOMEAWAYTIME, strlen(CMD_SETSCANHOMEAWAYTIME)) == 0) {
		ret = slsi_roam_scan_home_away_time_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETSCANHOMEAWAYTIME, strlen(CMD_GETSCANHOMEAWAYTIME)) == 0) {
		ret = slsi_roam_scan_home_away_time_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETOKCMODE, strlen(CMD_SETOKCMODE)) == 0) {
		ret = slsi_okc_mode_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETOKCMODE, strlen(CMD_GETOKCMODE)) == 0) {
		ret = slsi_okc_mode_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETWESMODE, strlen(CMD_SETWESMODE)) == 0) {
		ret = slsi_wes_mode_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETWESMODE, strlen(CMD_GETWESMODE)) == 0) {
		ret = slsi_wes_mode_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMSCANCHANNELS, strlen(CMD_SETROAMSCANCHANNELS)) == 0) {
		ret = slsi_roam_scan_channels_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETROAMSCANCHANNELS, strlen(CMD_GETROAMSCANCHANNELS)) == 0) {
		ret = slsi_roam_scan_channels_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_ADDROAMSCANCHANNELS, strlen(CMD_ADDROAMSCANCHANNELS)) == 0) {
		ret = slsi_roam_add_scan_channels(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETNCHOMODE, strlen(CMD_GETNCHOMODE)) == 0) {
		ret = slsi_get_ncho_mode(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETNCHOMODE, strlen(CMD_SETNCHOMODE)) == 0) {
		ret = slsi_set_ncho_mode(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETDFSSCANMODE, strlen(CMD_SETDFSSCANMODE)) == 0) {
		ret = slsi_set_dfs_scan_mode(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETDFSSCANMODE, strlen(CMD_GETDFSSCANMODE)) == 0) {
		ret = slsi_get_dfs_scan_mode(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_REASSOC, strlen(CMD_REASSOC)) == 0) {
		ret = slsi_reassoc_write(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SET_PMK, strlen(CMD_SET_PMK)) == 0) {
		ret = slsi_set_pmk(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_HAPD_GET_CHANNEL, strlen(CMD_HAPD_GET_CHANNEL)) == 0) {
		ret = slsi_auto_chan_read(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SET_SAP_CHANNEL_LIST, strlen(CMD_SET_SAP_CHANNEL_LIST)) == 0) {
		ret = slsi_auto_chan_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SENDACTIONFRAME, strlen(CMD_SENDACTIONFRAME)) == 0) {
		ret = slsi_send_action_frame(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_CERTSENDACTIONFRAME, strlen(CMD_CERTSENDACTIONFRAME)) == 0) {
		ret = slsi_send_action_frame_cert(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_HAPD_MAX_NUM_STA, strlen(CMD_HAPD_MAX_NUM_STA)) == 0) {
		ret = slsi_setting_max_sta_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_COUNTRY, strlen(CMD_COUNTRY)) == 0) {
		ret = slsi_country_write(dev, command, priv_cmd.total_len);
#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
	} else if (strncasecmp(command, CMD_BEACON_RECV, strlen(CMD_BEACON_RECV)) == 0) {
		ret = slsi_forward_beacon(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SETAPP2PWPSIE, strlen(CMD_SETAPP2PWPSIE)) == 0) {
		ret = slsi_set_ap_p2p_wps_ie(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_P2PSETPS, strlen(CMD_P2PSETPS)) == 0) {
		ret = slsi_set_p2p_oppps(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_P2PSETNOA, strlen(CMD_P2PSETNOA)) == 0) {
		ret = slsi_p2p_set_noa_params(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_P2PECSA, strlen(CMD_P2PECSA)) == 0) {
		ret = slsi_p2p_ecsa(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_P2PLOSTART, strlen(CMD_P2PLOSTART)) == 0) {
		ret = slsi_p2p_lo_start(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_P2PLOSTOP, strlen(CMD_P2PLOSTOP)) == 0) {
		ret = slsi_p2p_lo_stop(dev);
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
	} else if (strncasecmp(command, CMD_SETROAMOFFLOAD, strlen(CMD_SETROAMOFFLOAD)) == 0) {
		ret = slsi_roam_mode_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SETROAMOFFLAPLIST, strlen(CMD_SETROAMOFFLAPLIST)) == 0) {
		ret = slsi_roam_offload_ap_list(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SET_TX_POWER_CALLING, strlen(CMD_SET_TX_POWER_CALLING)) == 0) {
		ret = slsi_set_tx_power_calling(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_SET_TX_POWER_SAR, strlen(CMD_SET_TX_POWER_SAR)) == 0) {
		ret = slsi_set_tx_power_sar(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GET_TX_POWER_SAR, strlen(CMD_GET_TX_POWER_SAR)) == 0) {
		ret = slsi_get_tx_power_sar(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETREGULATORY, strlen(CMD_GETREGULATORY)) == 0) {
		ret = slsi_get_regulatory(dev, command, priv_cmd.total_len);
#ifdef CONFIG_SCSC_WLAN_HANG_TEST
	} else if (strncasecmp(command, CMD_TESTFORCEHANG, strlen(CMD_TESTFORCEHANG)) == 0) {
		ret = slsi_test_send_hanged_vendor_event(dev);
#endif
	} else if (strncasecmp(command, CMD_SET_FCC_CHANNEL, strlen(CMD_SET_FCC_CHANNEL)) == 0) {
		ret = slsi_set_fcc_channel(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_FAKEMAC, strlen(CMD_FAKEMAC)) == 0) {
		ret = slsi_fake_mac_write(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETBSSRSSI, strlen(CMD_GETBSSRSSI)) == 0) {
		ret = slsi_get_bss_rssi(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETBSSINFO, strlen(CMD_GETBSSINFO)) == 0) {
		ret = slsi_get_bss_info(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETSTAINFO, strlen(CMD_GETSTAINFO)) == 0) {
		ret = slsi_get_sta_info(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GETASSOCREJECTINFO, strlen(CMD_GETASSOCREJECTINFO)) == 0) {
		ret = slsi_get_assoc_reject_info(dev, command, priv_cmd.total_len);
#ifdef CONFIG_SCSC_WLAN_LOW_LATENCY_MODE
	} else if (strncasecmp(command, CMD_SET_LATENCY_MODE, strlen(CMD_SET_LATENCY_MODE)) == 0) {
		struct netdev_vif *ndev_vif = netdev_priv(dev);
		struct slsi_dev   *sdev = ndev_vif->sdev;
		struct slsi_ioctl_args *ioctl_args = NULL;
		int latency_mode = 0;

		ioctl_args = slsi_get_private_command_args(command, priv_cmd.total_len, 1);
		SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

		if (!slsi_str_to_int(ioctl_args->args[0], &latency_mode)) {
			SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
			ret = -EINVAL;
		} else {
			if (latency_mode != 0 && latency_mode != 1) {
				SLSI_ERR(sdev, "Invalid latency_mode: '%s'\n", ioctl_args->args[0]);
				ret = -EINVAL;
			} else {
				ret = slsi_set_latency_mode(dev, latency_mode, priv_cmd.total_len);
			}
		}
		kfree(ioctl_args);
	} else if (strncasecmp(command, CMD_SET_POWER_MGMT, strlen(CMD_SET_POWER_MGMT)) == 0) {
		ret = slsi_set_power_mode(dev, command, priv_cmd.total_len);
#endif
	} else if (strncasecmp(command, CMD_SET_LATENCY_CRITICAL, strlen(CMD_SET_LATENCY_CRITICAL)) == 0) {
		struct netdev_vif *ndev_vif = netdev_priv(dev);
		struct slsi_dev   *sdev = ndev_vif->sdev;
		struct slsi_ioctl_args *ioctl_args = NULL;
		int latency_mode = 0;

		ioctl_args = slsi_get_private_command_args(command, priv_cmd.total_len, 1);
		SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

		if (!slsi_str_to_int(ioctl_args->args[0], &latency_mode)) {
			SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
			ret = -EINVAL;
		} else {
			if (latency_mode != 0 && latency_mode != 1) {
				SLSI_ERR(sdev, "Invalid latency_mode: '%s'\n", ioctl_args->args[0]);
				ret = -EINVAL;
			} else {
				ret = slsi_set_latency_mode(dev, latency_mode, priv_cmd.total_len);
			}
		}
		kfree(ioctl_args);
	} else if (strncasecmp(command, CMD_SET_DISCONNECT_IES, strlen(CMD_SET_DISCONNECT_IES)) == 0) {
		ret = slsi_set_disconnect_ies(dev, command, priv_cmd.total_len);
#ifdef CONFIG_SCSC_WLAN_STA_ENHANCED_ARP_DETECT
	} else if (strncasecmp(command, CMD_SET_ENHANCED_ARP_TARGET, strlen(CMD_SET_ENHANCED_ARP_TARGET)) == 0) {
		ret = slsi_enhanced_arp_start_stop(dev, command, priv_cmd.total_len);
	} else if (strncasecmp(command, CMD_GET_ENHANCED_ARP_COUNTS, strlen(CMD_SET_ENHANCED_ARP_TARGET)) == 0) {
		ret = slsi_enhanced_arp_get_stats(dev, command, priv_cmd.total_len);
#endif
	} else if ((strncasecmp(command, CMD_RXFILTERSTART, strlen(CMD_RXFILTERSTART)) == 0) ||
			(strncasecmp(command, CMD_RXFILTERSTOP, strlen(CMD_RXFILTERSTOP)) == 0) ||
			(strncasecmp(command, CMD_BTCOEXMODE, strlen(CMD_BTCOEXMODE)) == 0) ||
			(strncasecmp(command, CMD_BTCOEXSCAN_START, strlen(CMD_BTCOEXSCAN_START)) == 0) ||
			(strncasecmp(command, CMD_BTCOEXSCAN_STOP, strlen(CMD_BTCOEXSCAN_STOP)) == 0) ||
			(strncasecmp(command, CMD_MIRACAST, strlen(CMD_MIRACAST)) == 0)) {
		ret = 0;
	} else if ((strncasecmp(command, CMD_AMPDU_MPDU, strlen(CMD_AMPDU_MPDU)) == 0) ||
			(strncasecmp(command, CMD_CHANGE_RL, strlen(CMD_CHANGE_RL)) == 0) ||
			(strncasecmp(command, CMD_INTERFACE_CREATE, strlen(CMD_INTERFACE_CREATE)) == 0) ||
			(strncasecmp(command, CMD_INTERFACE_DELETE, strlen(CMD_INTERFACE_DELETE)) == 0) ||
			(strncasecmp(command, CMD_LTECOEX, strlen(CMD_LTECOEX)) == 0) ||
			(strncasecmp(command, CMD_RESTORE_RL, strlen(CMD_RESTORE_RL)) == 0) ||
			(strncasecmp(command, CMD_RPSMODE, strlen(CMD_RPSMODE)) == 0) ||
			(strncasecmp(command, CMD_SETCCXMODE, strlen(CMD_SETCCXMODE)) == 0) ||
			(strncasecmp(command, CMD_SETDFSSCANMODE, strlen(CMD_SETDFSSCANMODE)) == 0) ||
			(strncasecmp(command, CMD_SETSINGLEANT, strlen(CMD_SETSINGLEANT)) == 0)) {
		ret  = -ENOTSUPP;
#ifndef SLSI_TEST_DEV
	} else if ((strncasecmp(command, CMD_DRIVERDEBUGDUMP, strlen(CMD_DRIVERDEBUGDUMP)) == 0) ||
		(strncasecmp(command, CMD_DRIVERDEBUGCOMMAND, strlen(CMD_DRIVERDEBUGCOMMAND)) == 0)) {
		slsi_dump_stats(dev);
#ifdef CONFIG_SCSC_LOG_COLLECTION
		scsc_log_collector_schedule_collection(SCSC_LOG_DUMPSTATE, SCSC_LOG_DUMPSTATE_REASON_DRIVERDEBUGDUMP);
#else
		ret = mx140_log_dump();
#endif
#endif
#ifdef CONFIG_SCSC_WLAN_SET_NUM_ANTENNAS
	} else if (strncasecmp(command, CMD_SET_NUM_ANTENNAS, strlen(CMD_SET_NUM_ANTENNAS)) == 0) {
		struct netdev_vif *ndev_vif = netdev_priv(dev);
		struct slsi_dev   *sdev = ndev_vif->sdev;
		int num_of_antennas;
		struct slsi_ioctl_args *ioctl_args = NULL;

		ioctl_args = slsi_get_private_command_args(command, priv_cmd.total_len, 1);
		SLSI_VERIFY_IOCTL_ARGS(sdev, ioctl_args);

		if (!slsi_str_to_int(ioctl_args->args[0], &num_of_antennas)) {
			SLSI_ERR(sdev, "Invalid string: '%s'\n", ioctl_args->args[0]);
			ret = -EINVAL;
		} else {
			/* We cannot lock in slsi_set_num_antennas as
			 *  this is also called in slsi_start_ap with netdev_vif lock.
			 */
			SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
			ret = slsi_set_num_antennas(dev, num_of_antennas);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		}
		kfree(ioctl_args);
#endif
#ifdef CONFIG_SCSC_WLAN_ENHANCED_PKT_FILTER
	} else if ((strncasecmp(command, CMD_ENHANCED_PKT_FILTER, strlen(CMD_ENHANCED_PKT_FILTER)) == 0)) {
		ret = slsi_set_enhanced_pkt_filter(dev, command, priv_cmd.total_len);
#endif
#ifdef CONFIG_SCSC_WLAN_MAX_LINK_SPEED
	} else if ((strncasecmp(command, CMD_GET_MAX_LINK_SPEED, strlen(CMD_GET_MAX_LINK_SPEED)) == 0)) {
		ret = slsi_get_linkspeed(dev, command, priv_cmd.total_len);
#endif
	} else {
		ret  = -ENOTSUPP;
	}
	if (strncasecmp(command, CMD_SETROAMBAND, strlen(CMD_SETROAMBAND)) != 0 && strncasecmp(command, CMD_SETBAND, strlen(CMD_SETBAND)) != 0 && copy_to_user(priv_cmd.buf, command, priv_cmd.total_len)) {
		ret = -EFAULT;
		SLSI_NET_ERR(dev, "Buffer copy fail\n");
	}

exit:
	kfree(command);
	return ret;
}
