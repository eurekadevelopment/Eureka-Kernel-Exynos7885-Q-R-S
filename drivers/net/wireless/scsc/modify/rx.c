/*****************************************************************************
 *
 * Copyright (c) 2012 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/etherdevice.h>
#include <linux/time.h>

fdjnfdjknfdjn


#include "debug.h"
#include "dev.h"
#include "mgt.h"
#include "mlme.h"
#include "src_sink.h"
#include "const.h"
#include "ba.h"
#include "mib.h"
#include "cac.h"
#include "nl80211_vendor.h"

#if (LINUX_VERSION_CODE <  KERNEL_VERSION(3, 14, 0))
#include "porting_imx.h"
#endif

struct ieee80211_channel *slsi_find_scan_channel(struct slsi_dev *sdev, struct ieee80211_mgmt *mgmt, size_t mgmt_len, u16 freq)
{
	int      ielen = mgmt_len - (mgmt->u.beacon.variable - (u8 *)mgmt);
	const u8 *scan_ds = cfg80211_find_ie(WLAN_EID_DS_PARAMS, mgmt->u.beacon.variable, ielen);
	const u8 *scan_ht = cfg80211_find_ie(WLAN_EID_HT_OPERATION, mgmt->u.beacon.variable, ielen);
	u8       chan = 0;

	/* Use the DS or HT channel where possible as the Offchannel results mean the RX freq is not reliable */
	if (scan_ds)
		chan = scan_ds[2];
	else if (scan_ht)
		chan = scan_ht[2];

	if (chan) {
		enum ieee80211_band band = IEEE80211_BAND_2GHZ;

		if (chan > 14)
			band = IEEE80211_BAND_5GHZ;
		freq = (u16)ieee80211_channel_to_frequency(chan, band);
	}
	if (!freq)
		return NULL;

	return ieee80211_get_channel(sdev->wiphy, freq);
}

static struct ieee80211_mgmt *slsi_rx_scan_update_ssid(struct slsi_dev *sdev, struct net_device *dev,
						       struct ieee80211_mgmt *mgmt, size_t mgmt_len, size_t *new_len)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u8 *new_mgmt;
	int offset;
	const u8 *mgmt_pos;
	const u8 *ssid;
	int     i;

	if (!SLSI_IS_VIF_INDEX_WLAN(ndev_vif))
		return NULL;

	/* update beacon, not probe response as probe response will always have actual ssid.*/
	if (!ieee80211_is_beacon(mgmt->frame_control))
		return NULL;

	ssid = cfg80211_find_ie(WLAN_EID_SSID, mgmt->u.beacon.variable,
				mgmt_len - (mgmt->u.beacon.variable - (u8 *)mgmt));
	if (!ssid) {
		SLSI_WARN(sdev, "beacon with NO SSID IE\n");
		return NULL;
	}
	/* update beacon only if hidden ssid. So, Skip if not hidden ssid*/
	if ((ssid[1] > 0) && (ssid[2] != '\0'))
		return NULL;

	/* check we have a known ssid for a bss */
	for (i = 0; i < SLSI_SCAN_SSID_MAP_MAX; i++) {
		if (SLSI_ETHER_EQUAL(sdev->ssid_map[i].bssid, mgmt->bssid)) {
			new_mgmt = kmalloc(mgmt_len + 34, GFP_KERNEL);
			if (!new_mgmt) {
				SLSI_ERR_NODEV("malloc failed(len:%ld)\n", mgmt_len + 34);
				return NULL;
			}

			/* copy frame till ssid element */
			memcpy(new_mgmt, mgmt, ssid - (u8 *)mgmt);
			offset = ssid - (u8 *)mgmt;
			/* copy bss ssid into new frame */
			new_mgmt[offset++] = WLAN_EID_SSID;
			new_mgmt[offset++] = sdev->ssid_map[i].ssid_len;
			memcpy(new_mgmt + offset, sdev->ssid_map[i].ssid, sdev->ssid_map[i].ssid_len);
			offset += sdev->ssid_map[i].ssid_len;
			/* copy rest of the frame following ssid */
			mgmt_pos = ssid + ssid[1] + 2;
			memcpy(new_mgmt + offset, mgmt_pos, mgmt_len - (mgmt_pos - (u8 *)mgmt));
			offset += mgmt_len - (mgmt_pos - (u8 *)mgmt);
			*new_len = offset;

			return (struct ieee80211_mgmt *)new_mgmt;
		}
	}
	return NULL;
}

void slsi_rx_scan_pass_to_cfg80211(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct ieee80211_mgmt    *mgmt = fapi_get_mgmt(skb);
	size_t                   mgmt_len = fapi_get_mgmtlen(skb);
	s32                      signal = fapi_get_s16(skb, u.mlme_scan_ind.rssi) * 100;
	u16                      freq = SLSI_FREQ_FW_TO_HOST(fapi_get_u16(skb, u.mlme_scan_ind.channel_frequency));
	struct ieee80211_channel *channel = slsi_find_scan_channel(sdev, mgmt, mgmt_len, freq);
	struct timespec uptime;

	get_monotonic_boottime(&uptime);
	SLSI_UNUSED_PARAMETER_NOT_DEBUG(dev);

	/* update timestamp with device uptime in micro sec */
	mgmt->u.beacon.timestamp = (uptime.tv_sec * 1000000) + (uptime.tv_nsec / 1000);

	SLSI_NET_DBG4(dev, SLSI_MLME, "freq:%d\n", fapi_get_u16(skb, u.mlme_scan_ind.channel_frequency) / 2);

	if (channel) {
		struct cfg80211_bss *bss;
		struct ieee80211_mgmt *mgmt_new;
		size_t mgmt_new_len = 0;

		mgmt_new = slsi_rx_scan_update_ssid(sdev, dev, mgmt, mgmt_len, &mgmt_new_len);
		if (mgmt_new)
			bss = cfg80211_inform_bss_frame(sdev->wiphy, channel, mgmt_new, mgmt_new_len, signal, GFP_KERNEL);
		else
			bss = cfg80211_inform_bss_frame(sdev->wiphy, channel, mgmt, mgmt_len, signal, GFP_KERNEL);
		slsi_cfg80211_put_bss(sdev->wiphy, bss);
		kfree(mgmt_new);
	} else {
		SLSI_NET_DBG1(dev, SLSI_MLME, "No Channel info found for freq:%d\n", freq);
	}

	slsi_kfree_skb(skb);
}

static int slsi_add_to_scan_list(struct slsi_dev *sdev, struct netdev_vif *ndev_vif,
				 struct sk_buff *skb, const u8 *scan_ssid, u16 scan_id)
{
	struct slsi_scan_result *head;
	struct slsi_scan_result *scan_result, *current_result, *prev = NULL;
	struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
	bool found = 0, skb_stored = 0;
	int current_rssi;

	SLSI_MUTEX_LOCK(ndev_vif->scan_result_mutex);
	head = ndev_vif->scan[scan_id].scan_results;
	scan_result = head;
	current_rssi =  fapi_get_s16(skb, u.mlme_scan_ind.rssi);

	while (scan_result) {
		if (SLSI_ETHER_EQUAL(scan_result->bssid, mgmt->bssid)) {
			/*entry exists for bssid*/
			if (!scan_result->probe_resp && ieee80211_is_probe_resp(mgmt->frame_control)) {
				scan_result->probe_resp = skb;
				skb_stored = 1;
			} else if (!scan_result->beacon && ieee80211_is_beacon(mgmt->frame_control)) {
				scan_result->beacon = skb;
				skb_stored = 1;
				if (!scan_ssid || !scan_ssid[1] || scan_ssid[2] == '\0')
					scan_result->hidden = 1;
			}

			/* Use the best RSSI value from all beacons/probe resp for a bssid. If no improvment
			  * in RSSI and beacon and probe response exist, ignore this result
			  */
			if (current_rssi < scan_result->rssi) {
				if (!skb_stored)
					slsi_kfree_skb(skb);
				SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
				return 0;
			}

			scan_result->rssi = current_rssi;
			if (!skb_stored) {
				if (ieee80211_is_beacon(mgmt->frame_control)) {
					slsi_kfree_skb(scan_result->beacon);
					scan_result->beacon = skb;
				} else {
					slsi_kfree_skb(scan_result->probe_resp);
					scan_result->probe_resp = skb;
				}
			}

			/*No change in position if rssi is still less than prev node*/
			if (!prev || (prev->rssi > current_rssi)) {
				SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
				return 0;
			}

			/*remove and re-insert*/
			found = 1;
			prev->next = scan_result->next;
			scan_result->next = NULL;
			current_result = scan_result;

			break;
		}

		prev = scan_result;
		scan_result = scan_result->next;
	}

	if (!found) {
		/*add_new node*/
		current_result = kzalloc(sizeof(*current_result), GFP_KERNEL);
		if (!current_result) {
			SLSI_ERR(sdev, "Failed to allocate node for scan result\n");
			SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
			return -1;
		}
		SLSI_ETHER_COPY(current_result->bssid, mgmt->bssid);

		current_result->rssi = current_rssi;

		if (ieee80211_is_beacon(mgmt->frame_control)) {
			current_result->beacon = skb;
			if (!scan_ssid || !scan_ssid[1] || scan_ssid[2] == '\0')
				current_result->hidden = 1;
		} else {
			current_result->probe_resp = skb;
		}
		current_result->next = NULL;

		if (!head) { /*first node*/
			ndev_vif->scan[scan_id].scan_results = current_result;
			SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
			return 0;
		}
	}

	scan_result = head;
	prev = NULL;
	/* insert based on rssi in descending order*/
	while (scan_result) {
		if (current_result->rssi > scan_result->rssi) {
			current_result->next = scan_result;
			if (prev)
				prev->next = current_result;
			else
				ndev_vif->scan[scan_id].scan_results = current_result;
			break;
		}
		prev = scan_result;
		scan_result = scan_result->next;
	}
	if (!scan_result) {
		/*insert at the end*/
		prev->next = current_result;
		current_result->next = NULL;
	}

	SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
	return 0;
}

static int slsi_add_to_p2p_scan_list(struct slsi_dev *sdev, struct netdev_vif *ndev_vif,
				     struct sk_buff *skb, u16 scan_id)
{
	struct slsi_scan_result *current_result;
	struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
	struct slsi_scan *scan;

	/*add_new node*/
	current_result = kzalloc(sizeof(*current_result), GFP_KERNEL);
	if (!current_result) {
		SLSI_ERR(sdev, "Failed to allocate node for scan result\n");
		return -1;
	}
	SLSI_ETHER_COPY(current_result->bssid, mgmt->bssid);

	SLSI_MUTEX_LOCK(ndev_vif->scan_result_mutex);
	scan = &ndev_vif->scan[scan_id];
	if (ieee80211_is_beacon(mgmt->frame_control))
		current_result->beacon = skb;
	else
		current_result->probe_resp = skb;

	if (!scan->scan_results) {
		scan->scan_results = current_result;
		current_result->next = NULL;
	} else {
		current_result->next = scan->scan_results;
		scan->scan_results = current_result;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);

	return 0;
}

void slsi_rx_scan_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16               scan_id = fapi_get_u16(skb, u.mlme_scan_ind.scan_id);
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
	size_t mgmt_len = fapi_get_mgmtlen(skb);
	size_t ie_len = mgmt_len - offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
	const u8 *scan_ssid = NULL;

	SLSI_NET_DBG3(dev, SLSI_MLME, "mlme_scan_ind(interface:%d, scan_id:%d) bssid:%pM\n", ndev_vif->ifnum, scan_id, fapi_get_mgmt(skb)->bssid);

#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
	if (slsi_is_gscan_id(scan_id)) {
		SLSI_NET_DBG3(dev, SLSI_GSCAN, "scan_id:%#x bssid:%pM\n", scan_id, fapi_get_mgmt(skb)->bssid);
		SLSI_MUTEX_LOCK(ndev_vif->scan_mutex);
		slsi_gscan_handle_scan_result(sdev, dev, skb, scan_id, false);
		SLSI_MUTEX_UNLOCK(ndev_vif->scan_mutex);
		return;
	}
#endif

	scan_ssid = cfg80211_find_ie(WLAN_EID_SSID, mgmt->u.probe_resp.variable, ie_len);

	if (sdev->p2p_certif && (ndev_vif->iftype == NL80211_IFTYPE_P2P_CLIENT) && (scan_id == (ndev_vif->ifnum << 8 | SLSI_SCAN_HW_ID))) {
		/* When supplicant receives a peer GO probe response with selected registrar set and group capability as 0,
		 * which is invalid, it is unable to store persistent network block. Hence such probe response is getting ignored here.
		 * This is mainly for an inter-op with Realtek P2P GO in P2P certification
		 */
		if (scan_ssid && scan_ssid[1] > 7) {
			const u8 *p2p_ie = NULL;

			p2p_ie = cfg80211_find_vendor_ie(WLAN_OUI_WFA, WLAN_OUI_TYPE_WFA_P2P, mgmt->u.probe_resp.variable, ie_len);
#define P2P_GROUP_CAPAB_PERSISTENT_GROUP BIT(1)
			if (p2p_ie && !(p2p_ie[10] & P2P_GROUP_CAPAB_PERSISTENT_GROUP)) {
				SLSI_NET_INFO(dev, "Ignoring a peer GO probe response with group_capab as 0\n");
				slsi_kfree_skb(skb);
				return;
			}
		}
	}

	scan_id = (scan_id & 0xFF);

	if (WARN_ON(scan_id >= SLSI_SCAN_MAX)) {
		slsi_kfree_skb(skb);
		return;
	}

	/* Blocking scans already taken scan mutex.
	 * So scan mutex only incase of non blocking scans.
	 */
	if (!ndev_vif->scan[scan_id].is_blocking_scan)
		SLSI_MUTEX_LOCK(ndev_vif->scan_mutex);

	if (fapi_get_vif(skb) != 0 && fapi_get_u16(skb, u.mlme_scan_ind.scan_id) == 0) {
		/* Connect/Roaming scan data : Save for processing later */
		SLSI_NET_DBG1(dev, SLSI_MLME, "Connect/Roaming scan indication received, bssid:%pM\n", fapi_get_mgmt(skb)->bssid);
		slsi_kfree_skb(ndev_vif->sta.mlme_scan_ind_skb);
		ndev_vif->sta.mlme_scan_ind_skb = skb;
	} else {
		slsi_roam_channel_cache_add(sdev, dev, skb);
		if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif))
			slsi_add_to_scan_list(sdev, ndev_vif, skb, scan_ssid, scan_id);
		else
			slsi_add_to_p2p_scan_list(sdev, ndev_vif, skb, scan_id);
	}

	if (!ndev_vif->scan[scan_id].is_blocking_scan)
		SLSI_MUTEX_UNLOCK(ndev_vif->scan_mutex);
}

static void slsi_scan_update_ssid_map(struct slsi_dev *sdev, struct net_device *dev, u16 scan_id)
{
	struct netdev_vif     *ndev_vif = netdev_priv(dev);
	struct ieee80211_mgmt *mgmt;
	const u8              *ssid_ie = NULL, *connected_ssid = NULL;
	int                   i, found = 0, is_connected = 0;
	struct slsi_scan_result	*scan_result = NULL;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->scan_result_mutex));

	if (ndev_vif->activated && ndev_vif->vif_type == FAPI_VIFTYPE_STATION && ndev_vif->sta.sta_bss) {
		is_connected = 1;
		connected_ssid = cfg80211_find_ie(WLAN_EID_SSID, ndev_vif->sta.sta_bss->ies->data, ndev_vif->sta.sta_bss->ies->len);
	}

	/* sanitize map: [remove any old entries] */
	for (i = 0; i < SLSI_SCAN_SSID_MAP_MAX; i++) {
		found = 0;
		if (!sdev->ssid_map[i].ssid_len)
			continue;

		/* We are connected to this hidden AP. So no need to check if this AP is present in scan results */
		if (is_connected && SLSI_ETHER_EQUAL(ndev_vif->sta.sta_bss->bssid, sdev->ssid_map[i].bssid))
			continue;

		/* If this entry AP is found to be non-hidden, remove entry. */
		scan_result = ndev_vif->scan[scan_id].scan_results;
		while (scan_result) {
			if (SLSI_ETHER_EQUAL(sdev->ssid_map[i].bssid, scan_result->bssid)) {
				/* AP is no more hidden. OR AP is hidden but did not
				 * receive probe resp. Go for expiry.
				 */
				if (!scan_result->hidden || (scan_result->hidden && !scan_result->probe_resp))
					sdev->ssid_map[i].age = SLSI_SCAN_SSID_MAP_EXPIRY_AGE;
				else
					found = 1;
				break;
			}
			scan_result = scan_result->next;
		}

		if (!found) {
			sdev->ssid_map[i].age++;
			if (sdev->ssid_map[i].age > SLSI_SCAN_SSID_MAP_EXPIRY_AGE) {
				sdev->ssid_map[i].ssid_len = 0;
				sdev->ssid_map[i].age = 0;
			}
		}
	}

	scan_result = ndev_vif->scan[scan_id].scan_results;
	/* update/add hidden bss with known ssid */
	while (scan_result) {
		ssid_ie = NULL;

		if (scan_result->hidden) {
			if (is_connected && SLSI_ETHER_EQUAL(ndev_vif->sta.sta_bss->bssid, scan_result->bssid)) {
				ssid_ie = connected_ssid;
			} else if (scan_result->probe_resp) {
				mgmt = fapi_get_mgmt(scan_result->probe_resp);
				ssid_ie = cfg80211_find_ie(WLAN_EID_SSID, mgmt->u.beacon.variable, fapi_get_mgmtlen(scan_result->probe_resp) - (mgmt->u.beacon.variable - (u8 *)mgmt));
			}
		}

		if (!ssid_ie) {
			scan_result = scan_result->next;
			continue;
		}

		found = 0;
		/* if this bss is in map, update map */
		for (i = 0; i < SLSI_SCAN_SSID_MAP_MAX; i++) {
			if (!sdev->ssid_map[i].ssid_len)
				continue;
			if (SLSI_ETHER_EQUAL(scan_result->bssid, sdev->ssid_map[i].bssid)) {
				sdev->ssid_map[i].ssid_len = ssid_ie[1];
				memcpy(sdev->ssid_map[i].ssid, &ssid_ie[2], ssid_ie[1]);
				found = 1;
				break;
			}
		}
		if (!found) {
			/* add a new entry in map */
			for (i = 0; i < SLSI_SCAN_SSID_MAP_MAX; i++) {
				if (sdev->ssid_map[i].ssid_len)
					continue;
				SLSI_ETHER_COPY(sdev->ssid_map[i].bssid, scan_result->bssid);
				sdev->ssid_map[i].age = 0;
				sdev->ssid_map[i].ssid_len = ssid_ie[1];
				memcpy(sdev->ssid_map[i].ssid, &ssid_ie[2], ssid_ie[1]);
				break;
			}
		}
		scan_result = scan_result->next;
	}
}

void slsi_scan_complete(struct slsi_dev *sdev, struct net_device *dev, u16 scan_id, bool aborted)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *scan;
	int count = 0;
	int *result_count = NULL, max_count = 0;

	if (WARN_ON(scan_id >= SLSI_SCAN_MAX))
		return;

	SLSI_NET_DBG3(dev, SLSI_MLME, "mlme_scan_done_ind(interface:%d, scan_id:%d)\n", ndev_vif->ifnum, scan_id);

	if (scan_id == SLSI_SCAN_HW_ID && !ndev_vif->scan[scan_id].scan_req)
		return;

	if (WARN_ON(scan_id == SLSI_SCAN_SCHED_ID && !ndev_vif->scan[scan_id].sched_req))
		return;

	SLSI_MUTEX_LOCK(ndev_vif->scan_result_mutex);
	if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
		slsi_scan_update_ssid_map(sdev, dev, scan_id);
		result_count = &count;
		max_count  = slsi_dev_get_scan_result_count();
	}
	scan = slsi_dequeue_cached_scan_result(&ndev_vif->scan[scan_id], result_count);
	while (scan) {
		SLSI_NET_DBG3(dev, SLSI_MLME, "mlme_scan_ind(scan_id:%d, %pM, freq:%d, rssi:%d)\n",
			      fapi_get_u16(scan, u.mlme_scan_ind.scan_id),
			      fapi_get_mgmt(scan)->bssid,
			      SLSI_FREQ_FW_TO_HOST(fapi_get_u16(scan, u.mlme_scan_ind.channel_frequency)),
			      fapi_get_s16(scan, u.mlme_scan_ind.rssi));

		/* skb freed inside slsi_rx_scan_pass_to_cfg80211 */
		slsi_rx_scan_pass_to_cfg80211(sdev, dev, scan);

		if ((SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) && (*result_count >= max_count)) {
			SLSI_NET_DBG3(dev, SLSI_MLME,
				      "mlme_scan_done_ind: dropping results - threshold reached: %d\n", *result_count);
			slsi_purge_scan_results_locked(ndev_vif, scan_id);
			break;
		}
		scan = slsi_dequeue_cached_scan_result(&ndev_vif->scan[scan_id], result_count);
	}

	slsi_roam_channel_cache_prune(dev, SLSI_ROAMING_CHANNEL_CACHE_TIMEOUT);

	if (scan_id == SLSI_SCAN_HW_ID) {
		if (SLSI_IS_VIF_INDEX_P2P(ndev_vif) && (!SLSI_IS_P2P_GROUP_STATE(sdev))) {
			/* Check for unsync vif as it could be present during the cycle of social channel scan and listen */
			if (ndev_vif->activated)
				SLSI_P2P_STATE_CHANGE(sdev, P2P_IDLE_VIF_ACTIVE);
			else
				SLSI_P2P_STATE_CHANGE(sdev, P2P_IDLE_NO_VIF);
		}

		cfg80211_scan_done(ndev_vif->scan[scan_id].scan_req, aborted);
		ndev_vif->scan[scan_id].scan_req = NULL;
		ndev_vif->scan[scan_id].requeue_timeout_work = false;
	}

	if (scan_id == SLSI_SCAN_SCHED_ID)
		cfg80211_sched_scan_results(sdev->wiphy);
	SLSI_MUTEX_UNLOCK(ndev_vif->scan_result_mutex);
}

void slsi_rx_scan_done_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16               scan_id = fapi_get_u16(skb, u.mlme_scan_done_ind.scan_id);
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	SLSI_MUTEX_LOCK(ndev_vif->scan_mutex);

#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
	if (slsi_is_gscan_id(scan_id)) {
		SLSI_NET_DBG3(dev, SLSI_GSCAN, "scan_id:%#x\n", scan_id);

		slsi_gscan_handle_scan_result(sdev, dev, skb, scan_id, true);

		SLSI_MUTEX_UNLOCK(ndev_vif->scan_mutex);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		return;
	}
#endif
	scan_id = (scan_id & 0xFF);

	if (scan_id == SLSI_SCAN_HW_ID && ndev_vif->scan[SLSI_SCAN_HW_ID].scan_req)
		cancel_delayed_work(&ndev_vif->scan_timeout_work);

	slsi_scan_complete(sdev, dev, scan_id, false);

	SLSI_MUTEX_UNLOCK(ndev_vif->scan_mutex);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void __slsi_rx_blockack_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_peer  *peer;

	SLSI_NET_DBG1(dev, SLSI_MLME, "ma_blockack_ind(vif:%d, peer_qsta_address:%pM, parameter_set:%d, sequence_number:%d, reason_code:%d, direction:%d)\n",
		      fapi_get_vif(skb),
		      fapi_get_buff(skb, u.ma_blockack_ind.peer_qsta_address),
		      fapi_get_u16(skb, u.ma_blockack_ind.blockack_parameter_set),
		      fapi_get_u16(skb, u.ma_blockack_ind.sequence_number),
		      fapi_get_u16(skb, u.ma_blockack_ind.reason_code),
		      fapi_get_u16(skb, u.ma_blockack_ind.direction));

	peer = slsi_get_peer_from_mac(sdev, dev, fapi_get_buff(skb, u.ma_blockack_ind.peer_qsta_address));
	WARN_ON(!peer);

	if (peer) {
		/* Buffering of frames before the mlme_connected_ind */
		if ((ndev_vif->vif_type == FAPI_VIFTYPE_AP) && (peer->connected_state == SLSI_STA_CONN_STATE_CONNECTING)) {
			SLSI_DBG3(sdev, SLSI_MLME, "Buffering MA-BlockAck.Indication\n");
			slsi_skb_queue_tail(&peer->buffered_frames, skb);
			return;
		}
		slsi_handle_blockack(
			dev,
			peer,
			fapi_get_vif(skb),
			fapi_get_buff(skb, u.ma_blockack_ind.peer_qsta_address),
			fapi_get_u16(skb, u.ma_blockack_ind.blockack_parameter_set),
			fapi_get_u16(skb, u.ma_blockack_ind.sequence_number),
			fapi_get_u16(skb, u.ma_blockack_ind.reason_code),
			fapi_get_u16(skb, u.ma_blockack_ind.direction)
			);
	}

	slsi_kfree_skb(skb);
}

void slsi_rx_blockack_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	__slsi_rx_blockack_ind(sdev, dev, skb);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
}

static bool get_wmm_ie_from_resp_ie(struct slsi_dev *sdev, struct net_device *dev, u8 *resp_ie, size_t resp_ie_len, const u8 **wmm_elem, size_t *wmm_elem_len)
{
	struct ieee80211_vendor_ie *ie;

	SLSI_UNUSED_PARAMETER(sdev);

	if (!resp_ie) {
		SLSI_NET_ERR(dev, "Received invalid pointer to the ie's of the association response\n");
		return false;
	}

	*wmm_elem = resp_ie;
	while (*wmm_elem && (*wmm_elem - resp_ie < resp_ie_len)) {
		/* parse response ie elements and return the wmm ie */
		*wmm_elem = cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WMM, *wmm_elem,
						    resp_ie_len - (*wmm_elem - resp_ie));
		/* re-assoc-res can contain wmm parameter IE and wmm TSPEC IE.
		 * we want wmm parameter Element)
		 */
		if (*wmm_elem && (*wmm_elem)[1] > 6 && (*wmm_elem)[6] == WMM_OUI_SUBTYPE_PARAMETER_ELEMENT)
			break;
		if (*wmm_elem)
			*wmm_elem += (*wmm_elem)[1];
	}

	if (!(*wmm_elem)) {
		SLSI_NET_DBG2(dev, SLSI_MLME, "No WMM IE\n");
		return false;
	}
	ie = (struct ieee80211_vendor_ie *)*wmm_elem;
	*wmm_elem_len = ie->len + 2;

	SLSI_NET_DBG3(dev, SLSI_MLME, "WMM IE received and parsed successfully\n");
	return true;
}

static bool sta_wmm_update_uapsd(struct slsi_dev *sdev, struct net_device *dev, struct slsi_peer *peer, u8 *assoc_req_ie, size_t assoc_req_ie_len)
{
	const u8 *wmm_information_ie;

	if (!assoc_req_ie) {
		SLSI_NET_ERR(dev, "null reference to IE\n");
		return false;
	}

	wmm_information_ie = cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WMM, assoc_req_ie, assoc_req_ie_len);
	if (!wmm_information_ie) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "no WMM IE\n");
		return false;
	}

	peer->uapsd = wmm_information_ie[8];
	SLSI_NET_DBG1(dev, SLSI_MLME, "peer->uapsd = 0x%x\n", peer->uapsd);
	return true;
}

static bool sta_wmm_update_wmm_ac_ies(struct slsi_dev *sdev, struct net_device *dev, struct slsi_peer *peer,
				      u8 *assoc_rsp_ie, size_t assoc_rsp_ie_len)
{
	size_t   left;
	const u8 *pos;
	const u8 *wmm_elem = NULL;
	size_t   wmm_elem_len = 0;
	struct netdev_vif  *ndev_vif = netdev_priv(dev);
	struct slsi_wmm_ac *wmm_ac = &ndev_vif->sta.wmm_ac[0];

	if (!get_wmm_ie_from_resp_ie(sdev, dev, assoc_rsp_ie, assoc_rsp_ie_len, &wmm_elem, &wmm_elem_len)) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "No WMM IE received\n");
		return false;
	}

	if (wmm_elem_len < 10 || wmm_elem[7] /* version */ != 1) {
		SLSI_NET_WARN(dev, "Invalid WMM IE: wmm_elem_len=%lu, wmm_elem[7]=%d\n", (unsigned long int)wmm_elem_len, (int)wmm_elem[7]);
		return false;
	}

	pos = wmm_elem + 10;
	left = wmm_elem_len - 10;

	for (; left >= 4; left -= 4, pos += 4) {
		int aci = (pos[0] >> 5) & 0x03;
		int acm = (pos[0] >> 4) & 0x01;

		memcpy(wmm_ac, pos, sizeof(struct slsi_wmm_ac));

		switch (aci) {
		case 1:                                            /* AC_BK */
			if (acm)
				peer->wmm_acm |= BIT(1) | BIT(2);  /* BK/- */
			break;
		case 2:                                            /* AC_VI */
			if (acm)
				peer->wmm_acm |= BIT(4) | BIT(5);  /* CL/VI */
			break;
		case 3:                                            /* AC_VO */
			if (acm)
				peer->wmm_acm |= BIT(6) | BIT(7);  /* VO/NC */
			break;
		case 0:                                            /* AC_BE */
		default:
			if (acm)
				peer->wmm_acm |= BIT(0) | BIT(3); /* BE/EE */
			break;
		}
		wmm_ac++;
	}

	SLSI_NET_DBG3(dev, SLSI_MLME, "WMM ies have been updated successfully\n");
	return true;
}

#ifdef CONFIG_SCSC_WLAN_KEY_MGMT_OFFLOAD
enum slsi_wlan_vendor_attr_roam_auth {
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_INVALID = 0,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_KEY_REPLAY_CTR,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KCK,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KEK,
	/* keep last */
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST,
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_MAX =
	SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST - 1
};

int slsi_send_roam_vendor_event(struct slsi_dev *sdev,
				const u8 *bssid,
				const u8 *req_ie, size_t req_ie_len,
				const u8 *resp_ie, size_t resp_ie_len,
				bool authorized)
{
	int                                    length = ETH_ALEN + 32 + req_ie_len + resp_ie_len;
	bool                                   is_secured_bss;
	struct sk_buff                         *skb = NULL;

	is_secured_bss = cfg80211_find_ie(WLAN_EID_RSN, req_ie, req_ie_len) ||
					cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WPA, req_ie, req_ie_len);

	SLSI_DBG2(sdev, SLSI_MLME, "authorized:%d, is_secured_bss:%d\n", authorized, is_secured_bss);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	skb = cfg80211_vendor_event_alloc(sdev->wiphy, NULL, length, SLSI_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH, GFP_KERNEL);
#else
	skb = cfg80211_vendor_event_alloc(sdev->wiphy, length, SLSI_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH, GFP_KERNEL);
#endif
	if (!skb) {
		SLSI_ERR_NODEV("Failed to allocate skb for VENDOR Roam event\n");
		return -ENOMEM;
	}
	if (nla_put(skb, SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID, ETH_ALEN, bssid) ||
	    nla_put(skb, SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED, 1, &authorized) ||
	   (req_ie && nla_put(skb, SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE, req_ie_len, req_ie)) ||
	   (resp_ie && nla_put(skb, SLSI_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE, resp_ie_len, resp_ie))) {
		SLSI_ERR_NODEV("Failed nla_put 1\n");
		slsi_kfree_skb(skb);
		return -EINVAL;
	}

	SLSI_DBG3_NODEV(SLSI_MLME, "Event: KEY_MGMT_ROAM_AUTH(%d)\n", SLSI_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH);
	cfg80211_vendor_event(skb, GFP_KERNEL);
	return 0;
}
#endif /* offload */

void slsi_rx_roamed_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif      *ndev_vif = netdev_priv(dev);
	struct ieee80211_mgmt  *mgmt = fapi_get_mgmt(skb);
	struct slsi_peer       *peer;
	u16                    temporal_keys_required = fapi_get_u16(skb, u.mlme_roamed_ind.temporal_keys_required);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	enum ieee80211_privacy bss_privacy;
#endif

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_roamed_ind(vif:%d) Roaming to %pM\n",
		      fapi_get_vif(skb),
		      mgmt->bssid);

	peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
	if (WARN_ON(!peer))
		goto exit;

	if (WARN_ON(!ndev_vif->sta.sta_bss))
		goto exit;

	slsi_rx_ba_stop_all(dev, peer);

	SLSI_ETHER_COPY(peer->address, mgmt->bssid);

	if (ndev_vif->sta.mlme_scan_ind_skb) {
		/* saved skb [mlme_scan_ind] freed inside slsi_rx_scan_pass_to_cfg80211 */
		slsi_rx_scan_pass_to_cfg80211(sdev, dev, ndev_vif->sta.mlme_scan_ind_skb);
		ndev_vif->sta.mlme_scan_ind_skb = NULL;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	if (ndev_vif->sta.sta_bss->capability & WLAN_CAPABILITY_PRIVACY)
		bss_privacy = IEEE80211_PRIVACY_ON;
	else
		bss_privacy = IEEE80211_PRIVACY_OFF;
#endif

	slsi_cfg80211_put_bss(sdev->wiphy, ndev_vif->sta.sta_bss);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
	ndev_vif->sta.sta_bss = cfg80211_get_bss(sdev->wiphy, NULL, peer->address, NULL, 0,
						 IEEE80211_BSS_TYPE_ANY, bss_privacy);
#else
	ndev_vif->sta.sta_bss = cfg80211_get_bss(sdev->wiphy, NULL, peer->address, NULL, 0,  0, 0);
#endif

	if (!ndev_vif->sta.sta_bss || !ndev_vif->sta.roam_mlme_procedure_started_ind) {
		WARN(!ndev_vif->sta.sta_bss, "bss not updated in cfg80211");
		WARN(!ndev_vif->sta.roam_mlme_procedure_started_ind, "proc-started-ind not received before roamed-ind");
		netif_carrier_off(dev);
		slsi_mlme_disconnect(sdev, dev, peer->address, 0, true);
		slsi_handle_disconnect(sdev, dev, peer->address, 0);
	} else {
		u8  *assoc_ie = NULL;
		int assoc_ie_len = 0;
		u8  *assoc_rsp_ie = NULL;
		int assoc_rsp_ie_len = 0;

		slsi_peer_reset_stats(sdev, dev, peer);
		slsi_peer_update_assoc_req(sdev, dev, peer, ndev_vif->sta.roam_mlme_procedure_started_ind);
		slsi_peer_update_assoc_rsp(sdev, dev, peer, skb);

		/* skb is consumed by slsi_peer_update_assoc_rsp. So do not access this anymore. */
		skb = NULL;

		if (peer->assoc_ie) {
			assoc_ie = peer->assoc_ie->data;
			assoc_ie_len = peer->assoc_ie->len;
		}

		if (peer->assoc_resp_ie) {
			assoc_rsp_ie = peer->assoc_resp_ie->data;
			assoc_rsp_ie_len = peer->assoc_resp_ie->len;
		}

		/* this is the right place to initialize the bitmasks for
		 * acm bit and tspec establishment
		 */
		peer->wmm_acm = 0;
		peer->tspec_established = 0;
		peer->uapsd = 0;

		/* update the uapsd bitmask according to the bit values
		 * in wmm information element of association request
		 */
		if (!sta_wmm_update_uapsd(sdev, dev, peer, assoc_ie, assoc_ie_len))
			SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM uapsd\n");

		/* update the acm bitmask according to the acm bit values that
		 * are included in wmm ie element of association response
		 */
		if (!sta_wmm_update_wmm_ac_ies(sdev, dev, peer, assoc_rsp_ie, assoc_rsp_ie_len))
			SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM AC ies\n");

		ndev_vif->sta.roam_mlme_procedure_started_ind = NULL;

		if (temporal_keys_required) {
			peer->pairwise_key_set = 0;
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DOING_KEY_CONFIG);
		}

		WARN_ON(assoc_ie_len && !assoc_ie);
		WARN_ON(assoc_rsp_ie_len && !assoc_rsp_ie);

		SLSI_NET_DBG3(dev, SLSI_MLME, "cfg80211_roamed()\n");
		cfg80211_roamed(dev,
				ndev_vif->sta.sta_bss->channel,
				peer->address,
				assoc_ie,
				assoc_ie_len,
				assoc_rsp_ie,
				assoc_rsp_ie_len,
				GFP_KERNEL);
#ifdef CONFIG_SCSC_WLAN_KEY_MGMT_OFFLOAD
		if (slsi_send_roam_vendor_event(sdev, peer->address, assoc_ie, assoc_ie_len,
						assoc_rsp_ie, assoc_rsp_ie_len, !temporal_keys_required) != 0) {
			SLSI_NET_ERR(dev, "Could not send Roam vendor event up");
		}
#endif
		SLSI_NET_DBG3(dev, SLSI_MLME, "cfg80211_roamed() Done\n");

		ndev_vif->sta.roam_in_progress = false;
		ndev_vif->chan = ndev_vif->sta.sta_bss->channel;
#ifndef SLSI_TEST_DEV
		SLSI_NET_DBG1(dev, SLSI_MLME, "Taking a wakelock for dhcp to finish after roaming\n");
		wake_lock_timeout(&sdev->wlan_roam_wl, msecs_to_jiffies(10 * 1000));
#endif

		if (!temporal_keys_required) {
			slsi_mlme_roamed_resp(sdev, dev);
			cac_update_roam_traffic_params(sdev, dev);
		} else {
			ndev_vif->sta.resp_id = MLME_ROAMED_RES;
		}
	}

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_roam_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif         *ndev_vif = netdev_priv(dev);
	enum ieee80211_statuscode status = WLAN_STATUS_SUCCESS;

	SLSI_UNUSED_PARAMETER(sdev);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_roam_ind(vif:%d, aid:0, result:%d )\n",
		      fapi_get_vif(skb),
		      fapi_get_u16(skb, u.mlme_roam_ind.result_code));

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	if (WARN(ndev_vif->vif_type != FAPI_VIFTYPE_STATION, "Not a Station VIF\n"))
		goto exit_with_lock;

	if (fapi_get_u16(skb, u.mlme_roam_ind.result_code) != FAPI_RESULTCODE_HOST_REQUEST_SUCCESS)
		status = WLAN_STATUS_UNSPECIFIED_FAILURE;

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

static void slsi_tdls_event_discovered(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif     *ndev_vif = netdev_priv(dev);
	u16                   tdls_event =  fapi_get_u16(skb, u.mlme_tdls_peer_ind.tdls_event);
	u16                   peer_index = fapi_get_u16(skb, u.mlme_tdls_peer_ind.peer_index);
	struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
	int                   len = fapi_get_mgmtlen(skb);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_tdls_event_discovered\n");

	if (len != 0) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 9))
		cfg80211_rx_mgmt(&ndev_vif->wdev, ndev_vif->chan->center_freq, 0, (const u8 *)mgmt, len, GFP_ATOMIC);
#else
		cfg80211_rx_mgmt(dev, ndev_vif->chan->center_freq, 0, (const u8 *)mgmt, len, GFP_ATOMIC);
#endif
		/* Handling MLME-TDLS-PEER.response */
		slsi_mlme_tdls_peer_resp(sdev, dev, peer_index, tdls_event);
	}

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	slsi_kfree_skb(skb);
}

static void slsi_tdls_event_connected(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct slsi_peer  *peer = NULL;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16               peer_index = fapi_get_u16(skb, u.mlme_tdls_peer_ind.peer_index);
	u16               tdls_event =  fapi_get_u16(skb, u.mlme_tdls_peer_ind.tdls_event);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	ndev_vif->sta.tdls_enabled = true;

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_tdls_event_connected(vif:%d, peer_index:%d mac[%pM])\n",
		      fapi_get_vif(skb), peer_index, fapi_get_buff(skb, u.mlme_tdls_peer_ind.peer_sta_address));

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	if (WARN(ndev_vif->vif_type != FAPI_VIFTYPE_STATION, "STA VIF"))
		goto exit_with_lock;

	/* Check for MAX client */
	if ((ndev_vif->sta.tdls_peer_sta_records) + 1 > SLSI_TDLS_PEER_CONNECTIONS_MAX) {
		SLSI_NET_ERR(dev, "MAX TDLS peer limit reached. Ignore ind for peer_index:%d\n", peer_index);
		goto exit_with_lock;
	}

	if (peer_index < SLSI_TDLS_PEER_INDEX_MIN || peer_index > SLSI_TDLS_PEER_INDEX_MAX) {
		SLSI_NET_ERR(dev, "Received incorrect peer_index: %d\n", peer_index);
		goto exit_with_lock;
	}

	peer = slsi_peer_add(sdev, dev, fapi_get_buff(skb, u.mlme_tdls_peer_ind.peer_sta_address), peer_index);

	if (!peer) {
		SLSI_NET_ERR(dev, "Peer NOT Created\n");
		goto exit_with_lock;
	}

	/* QoS is mandatory for TDLS - enable QoS for TDLS peer by default */
	peer->qos_enabled = true;

	slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);

	/* Move TDLS packets from STA_Q to TDLS_Q */
	slsi_tdls_move_packets(sdev, dev, ndev_vif->peer_sta_record[SLSI_STA_PEER_QUEUESET], peer, true);

	/* Handling MLME-TDLS-PEER.response */
	slsi_mlme_tdls_peer_resp(sdev, dev, peer_index, tdls_event);

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

static void slsi_tdls_event_disconnected(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct slsi_peer  *peer = NULL;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16               pid = fapi_get_u16(skb, u.mlme_tdls_peer_ind.peer_index);
	u16               tdls_event =  fapi_get_u16(skb, u.mlme_tdls_peer_ind.tdls_event);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (WARN_ON(!dev))
		goto exit;

	SLSI_NET_DBG1(dev, SLSI_MLME, "slsi_tdls_event_disconnected(vif:%d, MAC:%pM)\n", ndev_vif->ifnum, fapi_get_buff(skb, u.mlme_tdls_peer_ind.peer_sta_address));

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit;
	}

	peer = slsi_get_peer_from_mac(sdev, dev, fapi_get_buff(skb, u.mlme_tdls_peer_ind.peer_sta_address));

	if (!peer || (peer->aid == 0)) {
		WARN_ON(!peer || (peer->aid == 0));
		SLSI_NET_DBG1(dev, SLSI_MLME, "peer NOT found by MAC address\n");
		goto exit;
	}

	slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DISCONNECTED);

	/* Move TDLS packets from TDLS_Q to STA_Q */
	slsi_tdls_move_packets(sdev, dev, ndev_vif->peer_sta_record[SLSI_STA_PEER_QUEUESET], peer, false);

	slsi_peer_remove(sdev, dev, peer);

	slsi_mlme_tdls_peer_resp(sdev, dev, pid, tdls_event);
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	slsi_kfree_skb(skb);
}

/* Handling for MLME-TDLS-PEER.indication
 */
void slsi_tdls_peer_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 tdls_event =  fapi_get_u16(skb, u.mlme_tdls_peer_ind.tdls_event);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_tdls_peer_ind tdls_event: %d\n", tdls_event);

	switch (tdls_event) {
	case FAPI_TDLSEVENT_CONNECTED:
		slsi_tdls_event_connected(sdev, dev, skb);
		break;
	case FAPI_TDLSEVENT_DISCONNECTED:
		slsi_tdls_event_disconnected(sdev, dev, skb);
		break;
	case FAPI_TDLSEVENT_DISCOVERED:
		slsi_tdls_event_discovered(sdev, dev, skb);
		break;
	default:
		WARN_ON((tdls_event == 0) || (tdls_event > 4));
		slsi_kfree_skb(skb);
		break;
	}
}

/* Retrieve any buffered frame before connected_ind and pass them up. */
void slsi_rx_buffered_frames(struct slsi_dev *sdev, struct net_device *dev, struct slsi_peer *peer)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff    *buff_frame = NULL;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(ndev_vif->vif_mutex));
	if (WARN(!peer, "Peer is NULL"))
		return;
	WARN(peer->connected_state == SLSI_STA_CONN_STATE_CONNECTING, "Wrong state");

	SLSI_NET_DBG2(dev, SLSI_MLME, "Processing buffered RX frames received before mlme_connected_ind for (vif:%d, aid:%d)\n",
		      ndev_vif->ifnum, peer->aid);
	buff_frame = slsi_skb_dequeue(&peer->buffered_frames);
	while (buff_frame) {
		slsi_debug_frame(sdev, dev, buff_frame, "RX_BUFFERED");
		switch (fapi_get_sigid(buff_frame)) {
		case MA_BLOCKACK_IND:
			SLSI_NET_DBG2(dev, SLSI_MLME, "Transferring buffered MA_BLOCKACK_IND frame");
			__slsi_rx_blockack_ind(sdev, dev, buff_frame);
			break;
		default:
			SLSI_NET_WARN(dev, "Unexpected Data: 0x%.4x\n", fapi_get_sigid(buff_frame));
			slsi_kfree_skb(buff_frame);
			break;
		}
		buff_frame = slsi_skb_dequeue(&peer->buffered_frames);
	}
}

void slsi_rx_connected_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_peer  *peer = NULL;
	u16               aid = fapi_get_u16(skb, u.mlme_connected_ind.peer_index);

	/* For AP mode, peer_index value is equivalent to aid(association_index) value */

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_connected_ind(vif:%d, peer_index:%d)\n",
		      fapi_get_vif(skb),
		      aid);

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	if (WARN(ndev_vif->vif_type == FAPI_VIFTYPE_STATION, "STA VIF and Not Roaming"))
		goto exit_with_lock;

	switch (ndev_vif->vif_type) {
	case FAPI_VIFTYPE_AP:
	{
		if (aid < SLSI_PEER_INDEX_MIN || aid > SLSI_PEER_INDEX_MAX) {
			SLSI_NET_ERR(dev, "Received incorrect peer_index: %d\n", aid);
			goto exit_with_lock;
		}

		peer = slsi_get_peer_from_qs(sdev, dev, aid - 1);
		if (!peer) {
			SLSI_NET_ERR(dev, "Peer (aid:%d) Not Found - Disconnect peer\n", aid);
			goto exit_with_lock;
		}

		cfg80211_new_sta(dev, peer->address, &peer->sinfo, GFP_KERNEL);

		if (ndev_vif->ap.privacy) {
			peer->connected_state = SLSI_STA_CONN_STATE_DOING_KEY_CONFIG;
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DOING_KEY_CONFIG);
		} else {
			peer->connected_state = SLSI_STA_CONN_STATE_CONNECTED;
			slsi_mlme_connected_resp(sdev, dev, aid);
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
		}
		slsi_rx_buffered_frames(sdev, dev, peer);
		break;
	}

	default:
		SLSI_NET_WARN(dev, "mlme_connected_ind(vif:%d, unexpected vif type:%d)\n", fapi_get_vif(skb), ndev_vif->vif_type);
		break;
	}
exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_reassoc_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif         *ndev_vif = netdev_priv(dev);
	enum ieee80211_statuscode status = WLAN_STATUS_SUCCESS;
	struct slsi_peer          *peer = NULL;
	u8                        *assoc_ie = NULL;
	int                       assoc_ie_len = 0;
	u8                        *reassoc_rsp_ie = NULL;
	int                       reassoc_rsp_ie_len = 0;

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_reassoc_ind(vif:%d, result:0x%2x)\n",
		      fapi_get_vif(skb),
		      fapi_get_u16(skb, u.mlme_reassociate_ind.result_code));

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	if (WARN(ndev_vif->vif_type != FAPI_VIFTYPE_STATION, "Not a Station VIF\n"))
		goto exit_with_lock;

	peer = slsi_get_peer_from_qs(sdev, dev, 0);
	if (WARN_ON(!peer)) {
		SLSI_NET_ERR(dev, "PEER Not found\n");
		goto exit_with_lock;
	}

	if (fapi_get_u16(skb, u.mlme_reassociate_ind.result_code) != FAPI_RESULTCODE_SUCCESS) {
		status = WLAN_STATUS_UNSPECIFIED_FAILURE;
		slsi_rx_ba_stop_all(dev, peer);
	} else {
		peer->pairwise_key_set = 0;

		if (peer->assoc_ie) {
			assoc_ie = peer->assoc_ie->data;
			assoc_ie_len = peer->assoc_ie->len;
			WARN_ON(assoc_ie_len && !assoc_ie);
		}

		slsi_peer_reset_stats(sdev, dev, peer);

		peer->sinfo.assoc_req_ies = assoc_ie;
		peer->sinfo.assoc_req_ies_len = assoc_ie_len;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0))
		peer->sinfo.filled |= STATION_INFO_ASSOC_REQ_IES;
#endif
		slsi_peer_update_assoc_rsp(sdev, dev, peer, skb);
		/* skb is consumed by slsi_peer_update_assoc_rsp. So do not access this anymore. */
		skb = NULL;
		if (peer->assoc_resp_ie) {
			reassoc_rsp_ie = peer->assoc_resp_ie->data;
			reassoc_rsp_ie_len = peer->assoc_resp_ie->len;
			WARN_ON(reassoc_rsp_ie_len && !reassoc_rsp_ie);
		}

		/* update the uapsd bitmask according to the bit values
		 * in wmm information element of association request
		 */
		if (!sta_wmm_update_uapsd(sdev, dev, peer, assoc_ie, assoc_ie_len))
			SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM uapsd\n");

		/* update the acm bitmask according to the acm bit values that
		 * are included in wmm ie elements of re-association response
		 */
		if (!sta_wmm_update_wmm_ac_ies(sdev, dev, peer, reassoc_rsp_ie, reassoc_rsp_ie_len))
			SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM AC ies\n");
	}

	/* cfg80211_connect_result will take a copy of any ASSOC or (RE)ASSOC RSP IEs passed to it */
	cfg80211_connect_result(dev,
				peer->address,
				assoc_ie, assoc_ie_len,
				reassoc_rsp_ie, reassoc_rsp_ie_len,
				status,
				GFP_KERNEL);

	if (status == WLAN_STATUS_SUCCESS) {
		ndev_vif->sta.vif_status = SLSI_VIF_STATUS_CONNECTED;

		/* For Open & WEP AP,send reassoc response.
		 * For secured AP, all this would be done after handshake
		 */
		if ((peer->capabilities & WLAN_CAPABILITY_PRIVACY) &&
		    (cfg80211_find_ie(WLAN_EID_RSN, assoc_ie, assoc_ie_len) ||
		     cfg80211_find_ie(SLSI_WLAN_EID_WAPI, assoc_ie, assoc_ie_len) ||
		     cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WPA, assoc_ie, assoc_ie_len))) {
			/*secured AP*/
			ndev_vif->sta.resp_id = MLME_REASSOCIATE_RES;
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DOING_KEY_CONFIG);
			peer->connected_state = SLSI_STA_CONN_STATE_DOING_KEY_CONFIG;
		} else {
			/*Open/WEP AP*/
			slsi_mlme_reassociate_resp(sdev, dev);
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
			peer->connected_state = SLSI_STA_CONN_STATE_CONNECTED;
		}
	} else {
		netif_carrier_off(dev);
		slsi_mlme_del_vif(sdev, dev);
		slsi_vif_deactivated(sdev, dev);
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_connect_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif         *ndev_vif = netdev_priv(dev);
	enum ieee80211_statuscode status = WLAN_STATUS_SUCCESS;
	struct slsi_peer          *peer = NULL;
	u8                        *assoc_ie = NULL;
	int                       assoc_ie_len = 0;
	u8                        *assoc_rsp_ie = NULL;
	int                       assoc_rsp_ie_len = 0;
	u8                        bssid[ETH_ALEN];
	u16                       fw_result_code;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	fw_result_code = fapi_get_u16(skb, u.mlme_connect_ind.result_code);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_connect_ind(vif:%d, result:%d)\n",
		      fapi_get_vif(skb), fw_result_code);

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	if (WARN(ndev_vif->vif_type != FAPI_VIFTYPE_STATION, "Not a Station VIF\n"))
		goto exit_with_lock;

	if (ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTING) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not connecting\n");
		goto exit_with_lock;
	}

	peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
	if (peer) {
		SLSI_ETHER_COPY(bssid, peer->address);
	} else {
		SLSI_NET_ERR(dev, "!!NO peer record for AP\n");
		eth_zero_addr(bssid);
	}

	sdev->assoc_result_code = fw_result_code;
	if (fw_result_code != FAPI_RESULTCODE_SUCCESS) {
		SLSI_NET_ERR(dev, "Connect failed. FAPI code:%d\n", fw_result_code);
		status = WLAN_STATUS_UNSPECIFIED_FAILURE;
	} else {
		if (!peer || !peer->assoc_ie) {
			if (peer)
				WARN(!peer->assoc_ie, "proc-started-ind not received before connect-ind");
			status = WLAN_STATUS_UNSPECIFIED_FAILURE;
		} else {
			if (peer->assoc_ie) {
				assoc_ie = peer->assoc_ie->data;
				assoc_ie_len = peer->assoc_ie->len;
			}

			slsi_peer_update_assoc_rsp(sdev, dev, peer, skb);
			/* skb is consumed by slsi_peer_update_assoc_rsp. So do not access this anymore. */
			skb = NULL;

			if (peer->assoc_resp_ie) {
				assoc_rsp_ie = peer->assoc_resp_ie->data;
				assoc_rsp_ie_len = peer->assoc_resp_ie->len;
			}

			/* this is the right place to initialize the bitmasks for
			 * acm bit and tspec establishment
			 */
			peer->wmm_acm = 0;
			peer->tspec_established = 0;
			peer->uapsd = 0;

			/* update the uapsd bitmask according to the bit values
			 * in wmm information element of association request
			 */
			if (!sta_wmm_update_uapsd(sdev, dev, peer, assoc_ie, assoc_ie_len))
				SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM uapsd\n");

			/* update the wmm ac bitmasks according to the bit values that
			 * are included in wmm ie elements of association response
			 */
			if (!sta_wmm_update_wmm_ac_ies(sdev, dev, peer, assoc_rsp_ie, assoc_rsp_ie_len))
				SLSI_NET_DBG1(dev, SLSI_MLME, "Fail to update WMM AC ies\n");

			WARN_ON(!assoc_rsp_ie_len && !assoc_rsp_ie);
		}

		WARN(!ndev_vif->sta.mlme_scan_ind_skb, "mlme_scan.ind not received before connect-ind");

		if (ndev_vif->sta.mlme_scan_ind_skb) {
			SLSI_NET_DBG1(dev, SLSI_MLME, "Sending scan indication to cfg80211, bssid: %pM\n", fapi_get_mgmt(ndev_vif->sta.mlme_scan_ind_skb)->bssid);

			/* saved skb [mlme_scan_ind] freed inside slsi_rx_scan_pass_to_cfg80211 */
			slsi_rx_scan_pass_to_cfg80211(sdev, dev, ndev_vif->sta.mlme_scan_ind_skb);
			ndev_vif->sta.mlme_scan_ind_skb = NULL;
		}

		if (!ndev_vif->sta.sta_bss) {
			if (peer)
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0))
				ndev_vif->sta.sta_bss = cfg80211_get_bss(sdev->wiphy, NULL, peer->address, NULL, 0,  IEEE80211_BSS_TYPE_ANY, IEEE80211_PRIVACY_ANY);
#else
				ndev_vif->sta.sta_bss = cfg80211_get_bss(sdev->wiphy, NULL, peer->address, NULL, 0,  0, 0);
#endif
			if (!ndev_vif->sta.sta_bss) {
				SLSI_NET_ERR(dev, "sta_bss is not available, terminating the connection (peer: %p)\n", peer);
				status = WLAN_STATUS_UNSPECIFIED_FAILURE;
			}
		}
	}

	/* cfg80211_connect_result will take a copy of any ASSOC or ASSOC RSP IEs passed to it */
	cfg80211_connect_result(dev,
				bssid,
				assoc_ie, assoc_ie_len,
				assoc_rsp_ie, assoc_rsp_ie_len,
				status,
				GFP_KERNEL);

	if (status == WLAN_STATUS_SUCCESS) {
		ndev_vif->sta.vif_status = SLSI_VIF_STATUS_CONNECTED;

		/* For Open & WEP AP,set the power mode (static IP scenario) ,send connect response and install the packet filters .
		 * For secured AP, all this would be done after handshake
		 */
		if ((peer->capabilities & WLAN_CAPABILITY_PRIVACY) &&
		    (cfg80211_find_ie(WLAN_EID_RSN, assoc_ie, assoc_ie_len) ||
		     cfg80211_find_ie(SLSI_WLAN_EID_WAPI, assoc_ie, assoc_ie_len) ||
		     cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WPA, assoc_ie, assoc_ie_len))) {
			/*secured AP*/
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_DOING_KEY_CONFIG);
			ndev_vif->sta.resp_id = MLME_CONNECT_RES;
		} else {
			/*Open/WEP AP*/
			slsi_mlme_connect_resp(sdev, dev);
			slsi_set_packet_filters(sdev, dev);

			if (ndev_vif->ipaddress)
				slsi_mlme_powermgt(sdev, dev, ndev_vif->set_power_mode);
			slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
		}

		/* For P2PCLI, set the Connection Timeout (beacon miss) mib to 10 seconds
		 * This MIB set failure does not cause any fatal isuue. It just varies the
		 * detection time of GO's absence from 10 sec to FW default. So Do not disconnect
		 */
		if (ndev_vif->iftype == NL80211_IFTYPE_P2P_CLIENT) {
			if (slsi_set_uint_mib(sdev, dev, SLSI_PSID_UNIFI_MLME_CLI_INACTIVITY_TIMEOUT,
					      SLSI_P2PGC_CONN_TIMEOUT_MSEC) != 0)
				SLSI_NET_ERR(dev, "P2PGC Connection Timeout MIB set failed");
			SLSI_P2P_STATE_CHANGE(sdev, P2P_GROUP_FORMED_CLI);
		}

		/*Update the firmware with cached channels*/
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
		if (!sdev->device_config.roam_scan_mode && ndev_vif->vif_type == FAPI_VIFTYPE_STATION && ndev_vif->activated && ndev_vif->iftype != NL80211_IFTYPE_P2P_CLIENT) {
#else
		if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && ndev_vif->activated && ndev_vif->iftype != NL80211_IFTYPE_P2P_CLIENT) {
#endif
			const u8 *ssid = cfg80211_find_ie(WLAN_EID_SSID, assoc_ie, assoc_ie_len);
			u8       channels[SLSI_ROAMING_CHANNELS_MAX];
			u32      channels_count = slsi_roaming_scan_configure_channels(sdev, dev, ssid, channels);

			if (channels_count)
				if (slsi_mlme_set_cached_channels(sdev, dev, channels_count, channels) != 0)
					SLSI_NET_ERR(dev, "MLME-SET-CACHED-CHANNELS.req failed\n");
		}
	} else {
		/* Firmware reported connection success, but driver reported failure to cfg80211:
		 * send mlme-disconnect.req to firmware
		 */
		if ((fw_result_code == FAPI_RESULTCODE_SUCCESS) && peer)
			slsi_mlme_disconnect(sdev, dev, peer->address, FAPI_REASONCODE_UNSPECIFIED_REASON, false);
		netif_carrier_off(dev);
		slsi_mlme_del_vif(sdev, dev);
		slsi_vif_deactivated(sdev, dev);
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_disconnect_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_disconnect_ind(vif:%d, MAC:%pM)\n",
		      fapi_get_vif(skb),
		      fapi_get_buff(skb, u.mlme_disconnect_ind.peer_sta_address));

#ifdef CONFIG_SCSC_PRINTK
	mx140_log_dump();
#endif
	slsi_handle_disconnect(sdev,
			       dev,
			       fapi_get_buff(skb, u.mlme_disconnect_ind.peer_sta_address),
			       0);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_disconnected_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_disconnected_ind(vif:%d, reason:%d, MAC:%pM)\n",
		      fapi_get_vif(skb),
		      fapi_get_u16(skb, u.mlme_disconnected_ind.reason_code),
		      fapi_get_buff(skb, u.mlme_disconnected_ind.peer_sta_address));

#ifdef CONFIG_SCSC_PRINTK
	mx140_log_dump();
#endif
	if (ndev_vif->vif_type == FAPI_VIFTYPE_AP) {
		if (fapi_get_u16(skb, u.mlme_disconnected_ind.reason_code) ==
		    FAPI_REASONCODE_HOTSPOT_MAX_CLIENT_REACHED) {
			SLSI_NET_DBG1(dev, SLSI_MLME,
				      "Sending max hotspot client reached notification to user space\n");
			cfg80211_conn_failed(dev, fapi_get_buff(skb, u.mlme_disconnected_ind.peer_sta_address),
					     NL80211_CONN_FAIL_MAX_CLIENTS, GFP_KERNEL);
			goto exit;
		}
	}

	slsi_handle_disconnect(sdev,
			       dev,
			       fapi_get_buff(skb, u.mlme_disconnected_ind.peer_sta_address),
			       fapi_get_u16(skb, u.mlme_disconnected_ind.reason_code));

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
	return;
}

/* Handle Procedure Started (Type = Device Discovered) indication for P2P */
static void slsi_rx_p2p_device_discovered_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int               mgmt_len;

	SLSI_UNUSED_PARAMETER(sdev);

	SLSI_NET_DBG2(dev, SLSI_CFG80211, "Freq = %d\n", ndev_vif->chan->center_freq);

	/* Only Probe Request is expected as of now */
	mgmt_len = fapi_get_mgmtlen(skb);
	if (mgmt_len) {
		struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);

		if (ieee80211_is_mgmt(mgmt->frame_control)) {
			if (ieee80211_is_probe_req(mgmt->frame_control)) {
				SLSI_NET_DBG3(dev, SLSI_CFG80211, "Received Probe Request\n");
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 9))
				cfg80211_rx_mgmt(&ndev_vif->wdev, ndev_vif->chan->center_freq, 0, (const u8 *)mgmt, mgmt_len, GFP_ATOMIC);
#else
				cfg80211_rx_mgmt(dev, ndev_vif->chan->center_freq, 0, (const u8 *)mgmt, mgmt_len, GFP_ATOMIC);
#endif
			} else
				SLSI_NET_ERR(dev, "Ignore Indication - Not Probe Request frame\n");
		} else {
			SLSI_NET_ERR(dev, "Ignore Indication - Not Management frame\n");
		}
	}
}

void slsi_rx_procedure_started_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_peer  *peer = NULL;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_procedure_started_ind(vif:%d, type:%d, peer_index:%d)\n",
		      fapi_get_vif(skb),
		      fapi_get_u16(skb, u.mlme_procedure_started_ind.procedure_type),
		      fapi_get_u16(skb, u.mlme_procedure_started_ind.peer_index));

	if (!ndev_vif->activated) {
		SLSI_NET_DBG1(dev, SLSI_MLME, "VIF not activated\n");
		goto exit_with_lock;
	}

	switch (fapi_get_u16(skb, u.mlme_procedure_started_ind.procedure_type)) {
	case FAPI_PROCEDURETYPE_CONNECTION_STARTED:
		switch (ndev_vif->vif_type) {
		case FAPI_VIFTYPE_AP:
		{
			u16 peer_index = fapi_get_u16(skb, u.mlme_procedure_started_ind.peer_index);

			/* Check for MAX client */
			if ((ndev_vif->peer_sta_records + 1) > SLSI_AP_PEER_CONNECTIONS_MAX) {
				SLSI_NET_ERR(dev, "MAX Station limit reached. Ignore ind for peer_index:%d\n", peer_index);
				goto exit_with_lock;
			}

			if (peer_index < SLSI_PEER_INDEX_MIN || peer_index > SLSI_PEER_INDEX_MAX) {
				SLSI_NET_ERR(dev, "Received incorrect peer_index: %d\n", peer_index);
				goto exit_with_lock;
			}

			peer = slsi_peer_add(sdev, dev, (fapi_get_mgmt(skb))->sa, peer_index);
			if (!peer) {
				SLSI_NET_ERR(dev, "Peer NOT Created\n");
				goto exit_with_lock;
			}
			slsi_peer_update_assoc_req(sdev, dev, peer, skb);
			/* skb is consumed by slsi_peer_update_assoc_req. So do not access this anymore. */
			skb = NULL;
			peer->connected_state = SLSI_STA_CONN_STATE_CONNECTING;

			if ((ndev_vif->iftype == NL80211_IFTYPE_P2P_GO) &&
			    (peer->assoc_ie) &&
			    (cfg80211_find_vendor_ie(WLAN_OUI_MICROSOFT, WLAN_OUI_TYPE_MICROSOFT_WPS, peer->assoc_ie->data, peer->assoc_ie->len))) {
				SLSI_NET_DBG2(dev, SLSI_MLME,  "WPS IE is present. Setting peer->is_wps to TRUE\n");
				peer->is_wps = true;
			}

			/* Take a wakelock to avoid platform suspend before EAPOL exchanges (to avoid connection delay) */
			slsi_wakelock_timeout(&sdev->wlan_wl_to, SLSI_WAKELOCK_TIME_MSEC_EAPOL);
			break;
		}
		case FAPI_VIFTYPE_STATION:
		{
			peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
			if (WARN_ON(!peer)) {
				SLSI_NET_ERR(dev, "Peer NOT FOUND\n");
				goto exit_with_lock;
			}
			slsi_peer_update_assoc_req(sdev, dev, peer, skb);
			/* skb is consumed by slsi_peer_update_assoc_rsp. So do not access this anymore. */
			skb = NULL;
			break;
		}
		default:
			SLSI_NET_ERR(dev, "Incorrect vif type for proceduretype_connection_started\n");
			break;
		}
		break;
	case FAPI_PROCEDURETYPE_DEVICE_DISCOVERED:
		/* Expected only in P2P Device and P2P GO role */
		if (WARN_ON(!SLSI_IS_VIF_INDEX_P2P(ndev_vif) && (ndev_vif->iftype != NL80211_IFTYPE_P2P_GO)))
			goto exit_with_lock;

		/* Send probe request to supplicant only if in listening state. Issues were seen earlier if
		 * Probe request was sent to supplicant while waiting for GO Neg Req from peer.
		 * Send Probe request to supplicant if received in GO mode
		 */
		if ((sdev->p2p_state == P2P_LISTENING) || (ndev_vif->iftype == NL80211_IFTYPE_P2P_GO))
			slsi_rx_p2p_device_discovered_ind(sdev, dev, skb);
		break;
	case FAPI_PROCEDURETYPE_ROAMING_STARTED:
	{
		SLSI_NET_DBG1(dev, SLSI_MLME, "Roaming Procedure Starting with %pM\n", (fapi_get_mgmt(skb))->bssid);
		if (WARN_ON(ndev_vif->vif_type != FAPI_VIFTYPE_STATION))
			goto exit_with_lock;
		if (WARN_ON(!ndev_vif->peer_sta_record[SLSI_STA_PEER_QUEUESET] || !ndev_vif->peer_sta_record[SLSI_STA_PEER_QUEUESET]->valid))
			goto exit_with_lock;
		slsi_kfree_skb(ndev_vif->sta.roam_mlme_procedure_started_ind);
		ndev_vif->sta.roam_mlme_procedure_started_ind = skb;
		/* skb is consumed here. So remove reference to this.*/
		skb = NULL;
		break;
	}
	default:
		SLSI_NET_DBG1(dev, SLSI_MLME, "Unknown Procedure: %d\n", fapi_get_u16(skb, u.mlme_procedure_started_ind.procedure_type));
		goto exit_with_lock;
	}

exit_with_lock:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_frame_transmission_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif	*ndev_vif = netdev_priv(dev);
	struct slsi_peer	*peer;
	u16					host_tag = fapi_get_u16(skb, u.mlme_frame_transmission_ind.host_tag);
	u16					tx_status = fapi_get_u16(skb, u.mlme_frame_transmission_ind.transmission_status);
	bool				ack = true;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG2(dev, SLSI_MLME, "mlme_frame_transmission_ind(vif:%d, host_tag:%d, transmission_status:%d)\n", fapi_get_vif(skb),
		      host_tag,
		      tx_status);

	if (ndev_vif->mgmt_tx_data.host_tag == host_tag) {
		struct netdev_vif *ndev_vif_to_cfg = ndev_vif;

		/* If frame tx failed allow del_vif work to take care of vif deletion.
		 * This work would be queued as part of frame_tx with the wait duration
		 */
		if (tx_status != FAPI_TRANSMISSIONSTATUS_SUCCESSFUL) {
			ack = false;
			if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
				if (sdev->hs2_state == HS2_VIF_TX)
					sdev->hs2_state = HS2_VIF_ACTIVE; /*We wouldn't delete VIF*/
			} else {
				if (sdev->p2p_group_exp_frame != SLSI_P2P_PA_INVALID)
					slsi_clear_offchannel_data(sdev, false);
				else if (ndev_vif->mgmt_tx_data.exp_frame != SLSI_P2P_PA_INVALID)
					(void)slsi_mlme_reset_dwell_time(sdev, dev);
				ndev_vif->mgmt_tx_data.exp_frame = SLSI_P2P_PA_INVALID;
			}
		}

		/* Change state if frame tx was in Listen as peer response is not expected */
		if (SLSI_IS_VIF_INDEX_P2P(ndev_vif) && (ndev_vif->mgmt_tx_data.exp_frame == SLSI_P2P_PA_INVALID)) {
			if (delayed_work_pending(&ndev_vif->unsync.roc_expiry_work))
				SLSI_P2P_STATE_CHANGE(sdev, P2P_LISTENING);
			else
				SLSI_P2P_STATE_CHANGE(sdev, P2P_IDLE_VIF_ACTIVE);
		} else if (SLSI_IS_VIF_INDEX_P2P_GROUP(ndev_vif)) {
			const struct ieee80211_mgmt *mgmt = (const struct ieee80211_mgmt *)ndev_vif->mgmt_tx_data.buf;

			/* If frame transmission was initiated on P2P device vif by supplicant, then use the net_dev of that vif (i.e. p2p0) */
			if ((mgmt) && (memcmp(mgmt->sa, dev->dev_addr, ETH_ALEN) != 0)) {
				struct net_device *ndev = slsi_get_netdev(sdev, SLSI_NET_INDEX_P2P);

				SLSI_NET_DBG2(dev, SLSI_MLME, "Frame Tx was requested with device address - Change ndev_vif for tx_status\n");

				ndev_vif_to_cfg = netdev_priv(ndev);
				if (!ndev_vif_to_cfg) {
					SLSI_NET_ERR(dev, "Getting P2P Index netdev failed\n");
					ndev_vif_to_cfg = ndev_vif;
				}
			}
		}
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
		if (!sdev->device_config.wes_mode) {
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0))
			cfg80211_mgmt_tx_status(&ndev_vif_to_cfg->wdev, ndev_vif->mgmt_tx_data.cookie, ndev_vif->mgmt_tx_data.buf, ndev_vif->mgmt_tx_data.buf_len, ack, GFP_KERNEL);
#else
			cfg80211_mgmt_tx_status(ndev_vif_to_cfg->wdev.netdev, ndev_vif->mgmt_tx_data.cookie, ndev_vif->mgmt_tx_data.buf, ndev_vif->mgmt_tx_data.buf_len, ack, GFP_KERNEL);
#endif

#ifdef CONFIG_SCSC_WLAN_WES_NCHO
		}
#endif
		(void)slsi_set_mgmt_tx_data(ndev_vif, 0, 0, NULL, 0);
	}

	if ((tx_status == FAPI_TRANSMISSIONSTATUS_SUCCESSFUL) || (tx_status == FAPI_TRANSMISSIONSTATUS_RETRY_LIMIT)) {
		if ((ndev_vif->vif_type == FAPI_VIFTYPE_STATION) &&
		    (ndev_vif->sta.m4_host_tag == host_tag)) {
			switch (ndev_vif->sta.resp_id) {
			case MLME_ROAMED_RES:
				slsi_mlme_roamed_resp(sdev, dev);
				peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
				if (WARN_ON(!peer))
					break;
				slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
				cac_update_roam_traffic_params(sdev, dev);
				break;
			case MLME_CONNECT_RES:
				slsi_mlme_connect_resp(sdev, dev);
				slsi_set_packet_filters(sdev, dev);
				peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
				if (WARN_ON(!peer))
					break;
				slsi_ps_port_control(sdev, dev, peer, SLSI_STA_CONN_STATE_CONNECTED);
				break;
			case MLME_REASSOCIATE_RES:
				slsi_mlme_reassociate_resp(sdev, dev);
				break;
			default:
				break;
			}
			ndev_vif->sta.m4_host_tag = 0;
			ndev_vif->sta.resp_id = 0;
		}
		if (tx_status == FAPI_TRANSMISSIONSTATUS_RETRY_LIMIT) {
			if ((ndev_vif->iftype == NL80211_IFTYPE_STATION) &&
			    (ndev_vif->sta.eap_hosttag == host_tag)) {
				if (ndev_vif->sta.sta_bss) {
					SLSI_NET_WARN(dev, "Disconnect as EAP frame transmission failed\n");
					slsi_mlme_disconnect(sdev, dev, ndev_vif->sta.sta_bss->bssid, FAPI_REASONCODE_UNSPECIFIED_REASON, false);
				} else {
					SLSI_NET_WARN(dev, "EAP frame transmission failed, sta_bss not available\n");
				}
			}
			ndev_vif->stats.tx_errors++;
		}
	} else {
		ndev_vif->stats.tx_errors++;
	}
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_received_frame_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16 data_unit_descriptor = fapi_get_u16(skb, u.mlme_received_frame_ind.data_unit_descriptor);
	u16 frequency = SLSI_FREQ_FW_TO_HOST(fapi_get_u16(skb, u.mlme_received_frame_ind.channel_frequency));

	SLSI_NET_DBG2(dev, SLSI_MLME, "mlme_received_frame_ind(vif:%d, data descriptor:%d, freq:%d)\n",
		      fapi_get_vif(skb),
		      data_unit_descriptor,
		      frequency);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (data_unit_descriptor == FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME) {
		struct ieee80211_mgmt *mgmt;
		int mgmt_len;

		mgmt_len = fapi_get_mgmtlen(skb);
		if (!mgmt_len)
			goto exit;
		mgmt = fapi_get_mgmt(skb);
		if (WARN_ON(!(ieee80211_is_action(mgmt->frame_control))))
			goto exit;

		if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
			if (slsi_is_wes_action_frame(mgmt)) {
				SLSI_NET_DBG1(dev, SLSI_CFG80211, "Received NCHO WES VS action frame\n");
				if (!sdev->device_config.wes_mode)
					goto exit;
			} else {
#endif
			if (mgmt->u.action.category == WLAN_CATEGORY_WMM) {
				cac_rx_wmm_action(sdev, dev, mgmt, mgmt_len);
			} else {
				slsi_hs2_dump_public_action_subtype(mgmt, false);
				if (sdev->hs2_state == HS2_VIF_TX)
					sdev->hs2_state = HS2_VIF_ACTIVE;
			}
#ifdef CONFIG_SCSC_WLAN_WES_NCHO
			}
#endif
		} else {
			int subtype = slsi_p2p_get_public_action_subtype(mgmt);

			SLSI_NET_DBG2(dev, SLSI_CFG80211, "Received action frame (%s)\n", slsi_p2p_pa_subtype_text(subtype));

			if (SLSI_IS_P2P_UNSYNC_VIF(ndev_vif) && (ndev_vif->mgmt_tx_data.exp_frame != SLSI_P2P_PA_INVALID) && (subtype == ndev_vif->mgmt_tx_data.exp_frame)) {
				if (sdev->p2p_state == P2P_LISTENING)
					SLSI_NET_WARN(dev, "Driver in incorrect P2P state (P2P_LISTENING)");

				cancel_delayed_work(&ndev_vif->unsync.del_vif_work);

				ndev_vif->mgmt_tx_data.exp_frame = SLSI_P2P_PA_INVALID;
				(void)slsi_mlme_reset_dwell_time(sdev, dev);
				if (delayed_work_pending(&ndev_vif->unsync.roc_expiry_work)) {
					SLSI_P2P_STATE_CHANGE(sdev, P2P_LISTENING);
				} else {
					queue_delayed_work(sdev->device_wq, &ndev_vif->unsync.del_vif_work,
							   msecs_to_jiffies(SLSI_P2P_UNSYNC_VIF_EXTRA_MSEC));
					SLSI_P2P_STATE_CHANGE(sdev, P2P_IDLE_VIF_ACTIVE);
				}
			} else if ((sdev->p2p_group_exp_frame != SLSI_P2P_PA_INVALID) && (sdev->p2p_group_exp_frame == subtype)) {
				SLSI_NET_DBG2(dev, SLSI_MLME, "Expected action frame (%s) received on Group VIF\n", slsi_p2p_pa_subtype_text(subtype));
				slsi_clear_offchannel_data(sdev, (ndev_vif->ifnum != SLSI_NET_INDEX_P2PX) ? true : false);
			}
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 9))
		cfg80211_rx_mgmt(&ndev_vif->wdev, frequency, 0, (const u8 *)mgmt, mgmt_len, GFP_ATOMIC);
#else
		cfg80211_rx_mgmt(dev, frequency, 0, (const u8 *)mgmt, mgmt_len, GFP_ATOMIC);
#endif
	} else if (data_unit_descriptor == FAPI_DATAUNITDESCRIPTOR_IEEE802_3_FRAME) {
		struct slsi_peer *peer = NULL;
		struct ethhdr *ehdr = (struct ethhdr *)fapi_get_data(skb);

		peer = slsi_get_peer_from_mac(sdev, dev, ehdr->h_source);
		if (!peer) {
			SLSI_DBG1(sdev, SLSI_RX, "drop packet as No peer found\n");
			goto exit;
		}

		/* strip signal and any signal/bulk roundings/offsets */
		skb_pull(skb, fapi_get_siglen(skb));

		skb->dev = dev;
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		ndev_vif->stats.rx_packets++;
		ndev_vif->stats.rx_bytes += skb->len;
		dev->last_rx = jiffies;

		skb->protocol = eth_type_trans(skb, dev);
		slsi_dbg_untrack_skb(skb);
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
		SLSI_DBG2(sdev, SLSI_MLME, "pass %u bytes up (proto:%d)\n", skb->len, ntohs(skb->protocol));
		netif_rx_ni(skb);
		slsi_wakelock_timeout(&sdev->wlan_wl_to, SLSI_RX_WAKELOCK_TIME);
		return;
	}
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

void slsi_rx_mic_failure_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u8 *mac_addr;
	u16 key_type, key_id;
	enum nl80211_key_type nl_key_type;

	SLSI_UNUSED_PARAMETER(sdev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	mac_addr = fapi_get_buff(skb, u.mlme_disconnected_ind.peer_sta_address);
	key_type = fapi_get_u16(skb, u.mlme_mic_failure_ind.key_type);
	key_id = fapi_get_u16(skb, u.mlme_mic_failure_ind.key_id);

	SLSI_NET_DBG1(dev, SLSI_MLME, "mlme_mic_failure_ind(vif:%d, MAC:%pM, key_type:%d, key_id:%d)\n",
		      fapi_get_vif(skb), mac_addr, key_type, key_id);

	if (WARN_ON((key_type != FAPI_KEYTYPE_GROUP) && (key_type != FAPI_KEYTYPE_PAIRWISE)))
		goto exit;

	nl_key_type = (key_type == FAPI_KEYTYPE_GROUP) ? NL80211_KEYTYPE_GROUP : NL80211_KEYTYPE_PAIRWISE;

	cfg80211_michael_mic_failure(dev, mac_addr, nl_key_type, key_id, NULL, GFP_KERNEL);

exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

/**
 * Handler for mlme_listen_end_ind.
 * The listen_end_ind would be received when the total Listen Offloading time is over.
 * Indicate completion of Listen Offloading to supplicant by sending Cancel-ROC event
 * with cookie 0xffff. Queue delayed work for unsync vif deletion.
 */
void slsi_rx_listen_end_ind(struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

	SLSI_NET_DBG2(dev, SLSI_CFG80211, "Inform completion of P2P Listen Offloading\n");

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 9))
	cfg80211_remain_on_channel_expired(&ndev_vif->wdev, 0xffff, ndev_vif->chan, GFP_KERNEL);
#else
	cfg80211_remain_on_channel_expired(ndev_vif->wdev.netdev, 0xffff, ndev_vif->chan, ndev_vif->channel_type, GFP_KERNEL);
#endif

	ndev_vif->unsync.listen_offload = false;

	slsi_p2p_queue_unsync_vif_del_work(ndev_vif, SLSI_P2P_UNSYNC_VIF_EXTRA_MSEC);

	SLSI_P2P_STATE_CHANGE(ndev_vif->sdev, P2P_IDLE_VIF_ACTIVE);

	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_kfree_skb(skb);
}

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
static int slsi_rx_msdu_napi(struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	slsi_skb_queue_tail(&ndev_vif->napi.rx_data, skb);
	slsi_spinlock_lock(&ndev_vif->napi.lock);
	if (ndev_vif->napi.interrupt_enabled) {
		ndev_vif->napi.interrupt_enabled = false;
		napi_schedule(&ndev_vif->napi.napi);
	}
	slsi_spinlock_unlock(&ndev_vif->napi.lock);
}
#endif

int slsi_rx_blocking_signals(struct slsi_dev *sdev, struct sk_buff *skb)
{
	u16 pid, id;
	struct slsi_sig_send *sig_wait;
	u16 vif = fapi_get_vif(skb);

	sig_wait = &sdev->sig_wait;
	id = fapi_get_sigid(skb);
	pid = fapi_get_u16(skb, receiver_pid);

	/* ALL mlme cfm signals MUST have blocking call waiting for it (Per Vif or Global) */
	if (fapi_is_cfm(skb)) {
		struct net_device *dev;
		struct netdev_vif *ndev_vif;

		rcu_read_lock();
		dev = slsi_get_netdev_rcu(sdev, vif);
		if (dev) {
			ndev_vif = netdev_priv(dev);
			sig_wait = &ndev_vif->sig_wait;
		}
		spin_lock_bh(&sig_wait->send_signal_lock);
		SLSI_DBG3(sdev, SLSI_MLME, "rx  cfm(0x%.4x, pid:0x%.4x)\n", id, pid);
		SLSI_DBG3(sdev, SLSI_MLME, "exp cfm(0x%.4x, pid:0x%.4x)\n", sig_wait->cfm_id, sig_wait->process_id);
		if (id == sig_wait->cfm_id && pid == sig_wait->process_id) {
			if (WARN_ON(sig_wait->cfm))
				slsi_kfree_skb(sig_wait->cfm);
			sig_wait->cfm = skb;
			spin_unlock_bh(&sig_wait->send_signal_lock);
			complete(&sig_wait->completion);
			rcu_read_unlock();
			return 0;
		}
		/**
		 * Important data frames such as EAPOL, ARP, DHCP are send
		 * over MLME. For these frames driver does not block on confirms.
		 * So there can be unexpected confirms here for such data frames.
		 * These confirms are treated as normal and is silently dropped
		 * here
		 */
		if (id == MLME_SEND_FRAME_CFM) {
			SLSI_DBG3(sdev, SLSI_MLME, "cfm(0x%.4x, pid:0x%.4x, vif:%d)\n", id, pid, vif);
			spin_unlock_bh(&sig_wait->send_signal_lock);
			rcu_read_unlock();
			slsi_kfree_skb(skb);
			return 0;
		}

		SLSI_DBG1(sdev, SLSI_MLME, "Unexpected cfm(0x%.4x, pid:0x%.4x, vif:%d)\n", id, pid, vif);
		spin_unlock_bh(&sig_wait->send_signal_lock);
		rcu_read_unlock();
		return -EINVAL;
	}
	/* Some mlme ind signals have a blocking call waiting (Per Vif or Global) */
	if (fapi_is_ind(skb)) {
		struct net_device *dev;
		struct netdev_vif *ndev_vif;

		rcu_read_lock();
		dev = slsi_get_netdev_rcu(sdev, vif);
		if (dev) {
			ndev_vif = netdev_priv(dev);
			sig_wait = &ndev_vif->sig_wait;
		}
		SLSI_DBG3(sdev, SLSI_MLME, "rx  ind(0x%.4x, pid:0x%.4x)\n", id, pid);
		if (sig_wait->ind_id)
			SLSI_DBG3(sdev, SLSI_MLME, "exp ind(0x%.4x, pid:0x%.4x)\n", sig_wait->ind_id, sig_wait->process_id);
		spin_lock_bh(&sig_wait->send_signal_lock);
		if (id == sig_wait->ind_id && pid == sig_wait->process_id) {
			if (WARN_ON(sig_wait->ind))
				slsi_kfree_skb(sig_wait->ind);
			sig_wait->ind = skb;
			spin_unlock_bh(&sig_wait->send_signal_lock);
			complete(&sig_wait->completion);
			rcu_read_unlock();
			return 0;
		}
		spin_unlock_bh(&sig_wait->send_signal_lock);
		rcu_read_unlock();
	}
	return -EINVAL;
}
