/****************************************************************************
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/types.h>
#include "debug.h"
#include "dev.h"
#include "sap.h"
#include "sap_mlme.h"
#include "hip.h"
#include "mgt.h"
#include "scsc_wifilogger_rings.h"
#include "nl80211_vendor.h"
#include "mlme.h"

#define SUPPORTED_OLD_VERSION   0

static int sap_mlme_version_supported(u16 version);
static int sap_mlme_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb);

static int sap_mlme_notifier(struct slsi_dev *sdev, unsigned long event);

static struct sap_api sap_mlme = {
	.sap_class = SAP_MLME,
	.sap_version_supported = sap_mlme_version_supported,
	.sap_handler = sap_mlme_rx_handler,
	.sap_versions = { FAPI_CONTROL_SAP_VERSION, SUPPORTED_OLD_VERSION },
	.sap_notifier = sap_mlme_notifier,
};

static int sap_mlme_notifier(struct slsi_dev *sdev, unsigned long event)
{
	int i;
#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
	struct net_device *dev;
#endif
	struct netdev_vif *ndev_vif;

	SLSI_INFO_NODEV("Notifier event received: %lu\n", event);
	if (event >= SCSC_MAX_NOTIFIER)
		return -EIO;

	switch (event) {
	case SCSC_WIFI_STOP:
		/* Stop sending signals down*/
		sdev->mlme_blocked = true;
		SLSI_INFO_NODEV("MLME BLOCKED\n");

		/* cleanup all the VIFs and scan data */
		SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
		complete_all(&sdev->sig_wait.completion);
		for (i = 1; i <= CONFIG_SCSC_WLAN_MAX_INTERFACES; i++)
			if (sdev->netdev[i]) {
				ndev_vif = netdev_priv(sdev->netdev[i]);
				slsi_scan_cleanup(sdev, sdev->netdev[i]);
				if (cancel_work_sync(&ndev_vif->set_multicast_filter_work))
					slsi_wakeunlock(&sdev->wlan_wl);
				SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
				slsi_vif_cleanup(sdev, sdev->netdev[i], 0);
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			}
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		SLSI_INFO_NODEV("Force cleaned all VIFs\n");
		break;

	case SCSC_WIFI_FAILURE_RESET:
		break;

	case SCSC_WIFI_SUSPEND:
		break;

	case SCSC_WIFI_RESUME:
#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
		dev = slsi_get_netdev(sdev, SLSI_NET_INDEX_WLAN);
		ndev_vif = netdev_priv(dev);
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);

		if ((ndev_vif->is_wips_running) && (ndev_vif->activated) &&
		    (ndev_vif->vif_type == FAPI_VIFTYPE_STATION) &&
		    (ndev_vif->sta.vif_status == SLSI_VIF_STATUS_CONNECTED)) {
			ndev_vif->is_wips_running = false;

			slsi_send_forward_beacon_abort_vendor_event(sdev, SLSI_FORWARD_BEACON_ABORT_REASON_SUSPENDED);
			SLSI_INFO_NODEV("FORWARD_BEACON: SUSPEND_RESUMED!! send abort event\n");
		}

		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
#endif
		break;

	default:
		SLSI_INFO_NODEV("Unknown event code %lu\n", event);
		break;
	}

	return 0;
}

static int sap_mlme_version_supported(u16 version)
{
	unsigned int major = SAP_MAJOR(version);
	unsigned int minor = SAP_MINOR(version);
	u8           i = 0;

	SLSI_INFO_NODEV("Reported version: %d.%d\n", major, minor);

	for (i = 0; i < SAP_MAX_VER; i++)
		if (SAP_MAJOR(sap_mlme.sap_versions[i]) == major)
			return 0;

	SLSI_ERR_NODEV("Version %d.%d Not supported\n", major, minor);

	return -EINVAL;
}

static int slsi_rx_netdev_mlme(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 id = fapi_get_u16(skb, id);

	/* The skb is consumed by the functions called.
	 */
	switch (id) {
	case MLME_SCAN_IND:
		slsi_rx_scan_ind(sdev, dev, skb);
		break;
	case MLME_SCAN_DONE_IND:
		slsi_rx_scan_done_ind(sdev, dev, skb);
		break;
	case MLME_CONNECT_IND:
		slsi_rx_connect_ind(sdev, dev, skb);
		break;
	case MLME_CONNECTED_IND:
		slsi_rx_connected_ind(sdev, dev, skb);
		break;
	case MLME_RECEIVED_FRAME_IND:
		slsi_rx_received_frame_ind(sdev, dev, skb);
		break;
	case MLME_DISCONNECT_IND:
		slsi_rx_disconnect_ind(sdev, dev, skb);
		break;
	case MLME_DISCONNECTED_IND:
		slsi_rx_disconnected_ind(sdev, dev, skb);
		break;
	case MLME_PROCEDURE_STARTED_IND:
		slsi_rx_procedure_started_ind(sdev, dev, skb);
		break;
	case MLME_FRAME_TRANSMISSION_IND:
		slsi_rx_frame_transmission_ind(sdev, dev, skb);
		break;
	case MA_BLOCKACK_IND:
		slsi_rx_blockack_ind(sdev, dev, skb);
		break;
	case MLME_ROAMED_IND:
		slsi_rx_roamed_ind(sdev, dev, skb);
		break;
	case MLME_ROAM_IND:
		slsi_rx_roam_ind(sdev, dev, skb);
		break;
	case MLME_MIC_FAILURE_IND:
		slsi_rx_mic_failure_ind(sdev, dev, skb);
		break;
	case MLME_REASSOCIATE_IND:
		slsi_rx_reassoc_ind(sdev, dev, skb);
		break;
	case MLME_TDLS_PEER_IND:
		slsi_tdls_peer_ind(sdev, dev, skb);
		break;
	case MLME_LISTEN_END_IND:
		slsi_rx_listen_end_ind(dev, skb);
		break;
	case MLME_CHANNEL_SWITCHED_IND:
		slsi_rx_channel_switched_ind(sdev, dev, skb);
		break;
	case MLME_AC_PRIORITY_UPDATE_IND:
		SLSI_DBG1(sdev, SLSI_MLME, "Unexpected MLME_AC_PRIORITY_UPDATE_IND\n");
		slsi_kfree_skb(skb);
		break;
#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
	case MLME_RSSI_REPORT_IND:
		slsi_rx_rssi_report_ind(sdev, dev, skb);
		break;
	case MLME_RANGE_IND:
		slsi_rx_range_ind(sdev, dev, skb);
		break;
	case MLME_RANGE_DONE_IND:
		slsi_rx_range_done_ind(sdev, dev, skb);
		break;
	case MLME_EVENT_LOG_IND:
		slsi_rx_event_log_indication(sdev, dev, skb);
		break;
#endif
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	case MLME_NAN_EVENT_IND:
		slsi_nan_event(sdev, dev, skb);
		break;
	case MLME_NAN_FOLLOWUP_IND:
		slsi_nan_followup_ind(sdev, dev, skb);
		break;
	case MLME_NAN_SERVICE_IND:
		slsi_nan_service_ind(sdev, dev, skb);
		break;
	case MLME_NDP_REQUEST_IND:
		slsi_nan_ndp_setup_ind(sdev, dev, skb, true);
		break;
	case MLME_NDP_REQUESTED_IND:
		slsi_nan_ndp_requested_ind(sdev, dev, skb);
		break;
	case MLME_NDP_RESPONSE_IND:
		slsi_nan_ndp_setup_ind(sdev, dev, skb, false);
		break;
	case MLME_NDP_TERMINATE_IND:
		slsi_nan_ndp_termination_ind(sdev, dev, skb, false);
		break;
	case MLME_NDP_TERMINATED_IND:
		slsi_nan_ndp_termination_ind(sdev, dev, skb, true);
		break;
#endif
#ifdef CONFIG_SCSC_WLAN_SAE_CONFIG
	case MLME_SYNCHRONISED_IND:
		slsi_rx_synchronised_ind(sdev, dev, skb);
		break;
#endif
#ifdef CONFIG_SLSI_WLAN_STA_FWD_BEACON
	case MLME_BEACON_REPORTING_EVENT_IND:
		slsi_rx_beacon_reporting_event_ind(sdev, dev, skb);
		break;
#endif
	default:
		slsi_kfree_skb(skb);
		SLSI_NET_ERR(dev, "Unhandled Ind: 0x%.4x\n", id);
		break;
	}
	return 0;
}

void slsi_rx_netdev_mlme_work(struct work_struct *work)
{
	struct slsi_skb_work *w = container_of(work, struct slsi_skb_work, work);
	struct slsi_dev *sdev = w->sdev;
	struct net_device *dev = w->dev;
	struct sk_buff *skb = slsi_skb_work_dequeue(w);

	if (WARN_ON(!dev))
		return;

	slsi_wakelock(&sdev->wlan_wl);
	while (skb) {
		slsi_debug_frame(sdev, dev, skb, "RX");
		slsi_rx_netdev_mlme(sdev, dev, skb);
		skb = slsi_skb_work_dequeue(w);
	}
	slsi_wakeunlock(&sdev->wlan_wl);
}

int slsi_rx_enqueue_netdev_mlme(struct slsi_dev *sdev, struct sk_buff *skb, u16 vif)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);
	/* in case of del_vif failure, we may get mlme signals for a netdev which is already removed */
	if (!dev) {
		SLSI_WARN(sdev, "dev is NULL");
		kfree_skb(skb);
		rcu_read_unlock();
		return 0;
	}

	ndev_vif = netdev_priv(dev);

	if (unlikely(ndev_vif->is_fw_test) &&
	    likely(fapi_get_sigid(skb) != MA_BLOCKACK_IND)) {
		slsi_kfree_skb(skb);
		rcu_read_unlock();
		return 0;
	}

	slsi_skb_work_enqueue(&ndev_vif->rx_mlme, skb);
	rcu_read_unlock();
	return 0;
}

static int slsi_rx_action_enqueue_netdev_mlme(struct slsi_dev *sdev, struct sk_buff *skb, u16 vif)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);
	if (WARN_ON(!dev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	ndev_vif = netdev_priv(dev);

	if (unlikely(ndev_vif->is_fw_test)) {
		rcu_read_unlock();
		return 0;
	}

	if (ndev_vif->iftype == NL80211_IFTYPE_P2P_GO || ndev_vif->iftype == NL80211_IFTYPE_P2P_CLIENT) {
		struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
		/*  Check the DA of received action frame with the GO interface address */
		if (memcmp(mgmt->da, dev->dev_addr, ETH_ALEN) != 0) {
			/* If not equal, compare DA of received action frame with the P2P DEV address*/
			struct net_device *p2pdev = slsi_get_netdev_rcu(sdev, SLSI_NET_INDEX_P2P);

			if (WARN_ON(!p2pdev)) {
				rcu_read_unlock();
				return -ENODEV;
			}
			if (memcmp(mgmt->da, p2pdev->dev_addr, ETH_ALEN) == 0) {
				/* If destination address is equal to P2P DEV ADDR, then action frame is received on
				 * GO interface. Hence indicate action frames on P2P DEV
				 */
				ndev_vif = netdev_priv(p2pdev);

				if (unlikely(ndev_vif->is_fw_test)) {
					slsi_kfree_skb(skb);
					rcu_read_unlock();
					return 0;
				}
			}
		}
	}

	slsi_skb_work_enqueue(&ndev_vif->rx_mlme, skb);

	rcu_read_unlock();
	return 0;
}

static int sap_mlme_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb)
{
	u16 scan_id;
	u16 vif = fapi_get_vif(skb);

	if (slsi_rx_blocking_signals(sdev, skb) == 0)
		return 0;

	if (fapi_is_ind(skb)) {
		SCSC_WLOG_PKTFATE_LOG_RX_CTRL_FRAME(fapi_get_data(skb), fapi_get_datalen(skb));

		switch (fapi_get_sigid(skb)) {
		case MLME_SCAN_DONE_IND:
			scan_id = fapi_get_u16(skb, u.mlme_scan_done_ind.scan_id);
#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
			if (slsi_is_gscan_id(scan_id))
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, SLSI_NET_INDEX_WLAN);
#endif
			return slsi_rx_enqueue_netdev_mlme(sdev, skb, (scan_id >> 8));
		case MLME_SCAN_IND:
			if (vif)
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			scan_id = fapi_get_u16(skb, u.mlme_scan_ind.scan_id);
#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
			if (slsi_is_gscan_id(scan_id))
				return slsi_rx_enqueue_netdev_mlme(sdev, skb,  SLSI_NET_INDEX_WLAN);
#endif
			return slsi_rx_enqueue_netdev_mlme(sdev, skb,  (scan_id >> 8));
		case MLME_RECEIVED_FRAME_IND:
			if (vif == 0) {
				SLSI_WARN(sdev, "Received MLME_RECEIVED_FRAME_IND on VIF 0\n");
				goto err;
			}
			return slsi_rx_action_enqueue_netdev_mlme(sdev, skb, vif);
#ifdef CONFIG_SCSC_WLAN_GSCAN_ENABLE
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
		case MLME_NAN_EVENT_IND:
		case MLME_NAN_FOLLOWUP_IND:
		case MLME_NAN_SERVICE_IND:
		case MLME_NDP_REQUEST_IND:
		case MLME_NDP_REQUESTED_IND:
		case MLME_NDP_RESPONSE_IND:
		case MLME_NDP_TERMINATE_IND:
		case MLME_NDP_TERMINATED_IND:
			return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
#endif
		case MLME_RANGE_IND:
		case MLME_RANGE_DONE_IND:
			if (vif == 0)
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, SLSI_NET_INDEX_WLAN);
			else
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
#endif
#ifdef CONFIG_SCSC_WLAN_ENHANCED_LOGGING
		case MLME_EVENT_LOG_IND:
			return slsi_rx_enqueue_netdev_mlme(sdev, skb, SLSI_NET_INDEX_WLAN);
#endif
		case MLME_ROAMED_IND:
			if (vif == 0) {
				SLSI_WARN(sdev, "Received MLME_ROAMED_IND on VIF 0, return error\n");
				goto err;
			} else {
				struct net_device *dev;
				struct netdev_vif *ndev_vif;

				rcu_read_lock();
				dev = slsi_get_netdev_rcu(sdev, vif);
				if (WARN_ON(!dev)) {
					rcu_read_unlock();
					return -ENODEV;
				}
				ndev_vif = netdev_priv(dev);
				if (atomic_read(&ndev_vif->sta.drop_roamed_ind)) {
					/* If roam cfm is not received for the req, ignore this roamed indication. */
					slsi_kfree_skb(skb);
					rcu_read_unlock();
					return 0;
				}
				rcu_read_unlock();
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			}
		default:
			if (vif == 0) {
				SLSI_WARN(sdev, "Received signal 0x%04x on VIF 0, return error\n", fapi_get_sigid(skb));
				goto err;
			} else {
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			}
		}
	}
	if (WARN_ON(fapi_is_req(skb)))
		goto err;

	if (slsi_is_test_mode_enabled()) {
		slsi_kfree_skb(skb);
		return 0;
	}

	WARN_ON(1);

err:
	return -EINVAL;
}

int sap_mlme_init(void)
{
	SLSI_INFO_NODEV("Registering SAP\n");
	slsi_hip_sap_register(&sap_mlme);
	return 0;
}

int sap_mlme_deinit(void)
{
	SLSI_INFO_NODEV("Unregistering SAP\n");
	slsi_hip_sap_unregister(&sap_mlme);
	return 0;
}
