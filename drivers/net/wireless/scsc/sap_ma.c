/****************************************************************************
 *
 * Copyright (c) 2014 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/types.h>
#include "debug.h"
#include "dev.h"
#include "sap.h"
#include "sap_ma.h"
#include "hip.h"
#include "ba.h"
#include "mgt.h"
#include "nl80211_vendor.h"

#include "scsc_wifilogger_rings.h"

#define SUPPORTED_OLD_VERSION   0

static int sap_ma_version_supported(u16 version);
static int sap_ma_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb);
static int sap_ma_txdone(struct slsi_dev *sdev, u16 colour);
static int sap_ma_notifier(struct slsi_dev *sdev, unsigned long event);

static struct sap_api sap_ma = {
	.sap_class = SAP_MA,
	.sap_version_supported = sap_ma_version_supported,
	.sap_handler = sap_ma_rx_handler,
	.sap_versions = { FAPI_DATA_SAP_VERSION, SUPPORTED_OLD_VERSION },
	.sap_txdone = sap_ma_txdone,
	.sap_notifier = sap_ma_notifier,
};

static int sap_ma_notifier(struct slsi_dev *sdev, unsigned long event)
{
	uint vif;

	SLSI_INFO_NODEV("Notifier event received: %lu\n", event);
	if (event >= SCSC_MAX_NOTIFIER)
		return -EIO;

	switch (event) {
	case SCSC_WIFI_STOP:
		SLSI_INFO_NODEV("Stop netdev queues\n");
		rcu_read_lock();
		for (vif = SLSI_NET_INDEX_WLAN;
		     vif <= SLSI_NET_INDEX_P2PX_SWLAN; vif++) {
			struct net_device *ndev =
				slsi_get_netdev_rcu(sdev, vif);
			if (ndev && !netif_queue_stopped(ndev))
				netif_tx_stop_all_queues(ndev);
		}
		rcu_read_unlock();
		break;

	case SCSC_WIFI_FAILURE_RESET:
		SLSI_INFO_NODEV("Netdevs queues will not be restarted - recovery will take care of it\n");
		break;

	case SCSC_WIFI_SUSPEND:
		break;

	case SCSC_WIFI_RESUME:
		break;
	default:
		SLSI_INFO_NODEV("Unknown event code %lu\n", event);
		break;
	}

	return 0;
}

static int sap_ma_version_supported(u16 version)
{
	unsigned int major = SAP_MAJOR(version);
	unsigned int minor = SAP_MINOR(version);
	u8           i = 0;

	SLSI_INFO_NODEV("Reported version: %d.%d\n", major, minor);

	for (i = 0; i < SAP_MAX_VER; i++)
		if (SAP_MAJOR(sap_ma.sap_versions[i]) == major)
			return 0;

	SLSI_ERR_NODEV("Version %d.%d Not supported\n", major, minor);

	return -EINVAL;
}

static int slsi_rx_amsdu_deaggregate(struct net_device *dev, struct sk_buff *skb, struct sk_buff_head *msdu_list)
{
	unsigned int msdu_len;
	unsigned int subframe_len;
	int padding;
	struct sk_buff *subframe = NULL;
	const unsigned char mac_0[ETH_ALEN] = { 0 };
	bool skip_frame = false;
	struct ethhdr *mh;

	SLSI_NET_DBG3(dev, SLSI_RX, "A-MSDU received, length = %d\n", skb->len);

	while (skb != subframe) {
		msdu_len = (skb->data[ETH_ALEN * 2] << 8) | skb->data[(ETH_ALEN * 2) + 1];

		/* check if the length of sub-frame is valid */
		if (!msdu_len || msdu_len >= skb->len) {
			SLSI_NET_ERR(dev, "invalid MSDU length %d, SKB length = %d\n", msdu_len, skb->len);
			__skb_queue_purge(msdu_list);
			slsi_kfree_skb(skb);
			return -EINVAL;
		}

		subframe_len = msdu_len + (2 * ETH_ALEN) + 2;

		/* check if the length of sub-frame is valid */
		if (subframe_len > skb->len) {
			SLSI_NET_ERR(dev, "invalid subframe length %d, SKB length = %d\n", subframe_len, skb->len);
			__skb_queue_purge(msdu_list);
			slsi_kfree_skb(skb);
			return -EINVAL;
		}

		/* For the last subframe skb length and subframe length will be same */
		if (skb->len == subframe_len) {
			/* Use the original skb for the last subframe */
			subframe = skb;

			/* There is no padding for last subframe */
			padding = 0;
		} else {
			/* Clone the skb for the subframe */
			subframe = slsi_skb_clone(skb, GFP_ATOMIC);
			if (!subframe) {
				__skb_queue_purge(msdu_list);
				slsi_kfree_skb(skb);
				SLSI_NET_ERR(dev, "Failed to clone the SKB for A-MSDU subframe\n");
				return -ENOMEM;
			}

			padding = (4 - (subframe_len % 4)) & 0x3;
		}

		/* Remove the other subframes by adjusting the tail pointer of the cloned skb */
		skb_trim(subframe, subframe_len);

		/* Overwrite LLC+SNAP header with src & dest addr */
		SLSI_ETHER_COPY(&subframe->data[14], &subframe->data[6]);
		SLSI_ETHER_COPY(&subframe->data[8], &subframe->data[0]);

		/* Remove 8 bytes of LLC+SNAP header */
		skb_pull(subframe, LLC_SNAP_HDR_LEN);

		SLSI_NET_DBG3(dev, SLSI_RX, "msdu_len = %d, subframe_len = %d, padding = %d\n",
			      msdu_len, subframe_len, padding);
		SLSI_NET_DBG_HEX(dev, SLSI_RX, subframe->data,
				 subframe->len < 64 ? subframe->len : 64, "Subframe before giving to OS:\n");

		/* Before preparing the skb, filter out if the Destination Address of the Ethernet frame
		 * or A-MSDU subframe is set to an invalid value, i.e. all zeros
		 */
		skb_set_mac_header(subframe, 0);
		mh = eth_hdr(subframe);
		if (SLSI_ETHER_EQUAL(mh->h_dest, mac_0)) {
			SLSI_NET_DBG3(dev, SLSI_RX, "msdu subframe filtered out: MAC destination address %pM\n",
				      mh->h_dest);
			skip_frame = true;
		}

		/* If this is not the last subframe then move to the next subframe */
		if (skb != subframe) {
			/* If A-MSDU is not formed correctly (e.g when skb->len < subframe_len + padding),
			 * skb_pull() will return NULL without any manipulation in skb.
			 * It can lead to infinite loop.
			 */
			if (!skb_pull(skb, (subframe_len + padding))) {
				SLSI_NET_ERR(dev, "Invalid subframe + padding length=%d, SKB length=%d\n", subframe_len + padding, skb->len);
				__skb_queue_purge(msdu_list);
				slsi_kfree_skb(skb);
				return -EINVAL;
			}
		}

		/* If this frame has been filtered out, free the clone and continue */
		if (skip_frame) {
			skip_frame = false;
			/* Free the the skbuff structure itself but not the data */
			/* skb will be freed if it is the last subframe (i.e. subframe == skb) */
			slsi_kfree_skb(subframe);
			continue;
		}

		__skb_queue_tail(msdu_list, subframe);
	}

	return 0;
}

static inline bool slsi_rx_is_amsdu(struct sk_buff *skb)
{
	return (fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor) == FAPI_DATAUNITDESCRIPTOR_AMSDU);
}

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
int slsi_rx_data_napi(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb, bool from_ba)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	slsi_debug_frame(sdev, dev, skb, "RX");

	if (slsi_rx_data_process_skb(sdev, dev, skb, from_ba) == 0) {
		slsi_skb_queue_tail(&ndev_vif->napi.rx_data, skb);
		slsi_spinlock_lock(&ndev_vif->napi.lock);
		if (ndev_vif->napi.interrupt_enabled) {
			ndev_vif->napi.interrupt_enabled = false;
			napi_schedule(&ndev_vif->napi.napi);
		}
		slsi_spinlock_unlock(&ndev_vif->napi.lock);
	}
	return 0;
}
#endif

void slsi_rx_data_deliver_skb(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff_head msdu_list;
	struct slsi_peer *peer = NULL;
	struct ethhdr *eth_hdr;
	bool is_amsdu = slsi_rx_is_amsdu(skb);
	u8 trafic_q = slsi_frame_priority_to_ac_queue(fapi_get_u16(skb, u.ma_unitdata_ind.priority));

	__skb_queue_head_init(&msdu_list);

	skb_pull(skb, fapi_get_siglen(skb));

	eth_hdr = (struct ethhdr *)skb->data;
	peer = slsi_get_peer_from_mac(sdev, dev, eth_hdr->h_source);
	if (!peer) {
		SLSI_NET_WARN(dev, "Packet dropped (no peer records)\n");
		slsi_kfree_skb(skb);
		return;
	}

	/* A-MSDU deaggregation */
	if (is_amsdu) {
		if (slsi_rx_amsdu_deaggregate(dev, skb, &msdu_list)) {
			ndev_vif->stats.rx_dropped++;
			if (peer)
				peer->sinfo.rx_dropped_misc++;
			return;
		}
	} else {
		__skb_queue_tail(&msdu_list, skb);
	}

	while (!skb_queue_empty(&msdu_list)) {
		struct sk_buff *rx_skb;

		rx_skb = __skb_dequeue(&msdu_list);

		/* In STA mode, the AP relays back our multicast traffic.
		 * Receiving these frames and passing it up confuses some
		 * protocols and applications, notably IPv6 Duplicate
		 * Address Detection.
		 *
		 * So these frames are dropped instead of passing it further.
		 * No need to update the drop statistics as these frames are
		 * locally generated and should not be accounted in reception.
		 */
		if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION) {
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);

			if (is_multicast_ether_addr(ehdr->h_dest) &&
				!compare_ether_addr(ehdr->h_source, dev->dev_addr)) {
				SLSI_NET_DBG2(dev, SLSI_RX, "drop locally generated multicast frame relayed back by AP\n");
				slsi_kfree_skb(rx_skb);
				continue;
			}
		}

		/* Intra BSS */
		if (ndev_vif->vif_type == FAPI_VIFTYPE_AP && ndev_vif->peer_sta_records) {
			struct slsi_peer *peer = NULL;
			struct ethhdr *ehdr = (struct ethhdr *)(rx_skb->data);

			if (is_multicast_ether_addr(ehdr->h_dest)) {
				struct sk_buff *rebroadcast_skb = slsi_skb_copy(rx_skb, GFP_KERNEL);

				if (!rebroadcast_skb) {
					SLSI_WARN(sdev, "Intra BSS: failed to alloc new SKB for broadcast\n");
				} else {
					SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: multicast %pM\n", ehdr->h_dest);
					rebroadcast_skb->dev = dev;
					rebroadcast_skb->protocol = cpu_to_be16(ETH_P_802_3);
					slsi_dbg_untrack_skb(rebroadcast_skb);
					skb_reset_network_header(rebroadcast_skb);
					skb_reset_mac_header(rebroadcast_skb);
					dev_queue_xmit(rebroadcast_skb);
				}
			} else {
				peer = slsi_get_peer_from_mac(sdev, dev, ehdr->h_dest);
				if (peer && peer->authorized) {
					SLSI_DBG3(sdev, SLSI_RX, "Intra BSS: unicast %pM\n", ehdr->h_dest);
					rx_skb->dev = dev;
					rx_skb->protocol = cpu_to_be16(ETH_P_802_3);
					slsi_dbg_untrack_skb(rx_skb);
					skb_reset_network_header(rx_skb);
					skb_reset_mac_header(rx_skb);
					dev_queue_xmit(rx_skb);
					continue;
				}
			}
		}

		if (peer) {
			peer->sinfo.rx_packets++;
			peer->sinfo.rx_bytes += rx_skb->len;
		}
		ndev_vif->stats.rx_packets++;
		ndev_vif->stats.rx_bytes += rx_skb->len;
		ndev_vif->rx_packets[trafic_q]++;

#ifdef CONFIG_SCSC_WLAN_STA_ENHANCED_ARP_DETECT
	if (!ndev_vif->enhanced_arp_stats.is_duplicate_addr_detected) {
		u8 *frame = rx_skb->data + 12; /* frame points to packet type */
		u16 packet_type = frame[0] << 8 | frame[1];

		if (packet_type == ETH_P_ARP) {
			frame = frame + 2; /* ARP packet */
			/*match source IP address in ARP with the DUT Ip address*/
			if ((frame[SLSI_ARP_SRC_IP_ADDR_OFFSET] == (ndev_vif->ipaddress & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 1] == ((ndev_vif->ipaddress >>  8U) & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 2] == ((ndev_vif->ipaddress >> 16U) & 255)) &&
			    (frame[SLSI_ARP_SRC_IP_ADDR_OFFSET + 3] == ((ndev_vif->ipaddress >> 24U) & 255)) &&
			    !SLSI_IS_GRATUITOUS_ARP(frame) &&
			    !SLSI_ETHER_EQUAL(sdev->hw_addr, frame + 8)) /*if src MAC = DUT MAC */
				ndev_vif->enhanced_arp_stats.is_duplicate_addr_detected = 1;
		}
	}

	if (ndev_vif->enhanced_arp_detect_enabled && (ndev_vif->vif_type == FAPI_VIFTYPE_STATION)) {
		u8 *frame = rx_skb->data + 12; /* frame points to packet type */
		u16 packet_type = frame[0] << 8 | frame[1];
		u16 arp_opcode;

		if (packet_type == ETH_P_ARP) {
			frame = frame + 2; /* ARP packet */
			arp_opcode = frame[SLSI_ARP_OPCODE_OFFSET] << 8 | frame[SLSI_ARP_OPCODE_OFFSET + 1];
			/* check if sender ip = gateway ip and it is an ARP response */
			if ((arp_opcode == SLSI_ARP_REPLY_OPCODE) &&
			    !SLSI_IS_GRATUITOUS_ARP(frame) &&
			    !memcmp(&frame[SLSI_ARP_SRC_IP_ADDR_OFFSET], &ndev_vif->target_ip_addr, 4)) {
				ndev_vif->enhanced_arp_stats.arp_rsp_count_to_netdev++;
				ndev_vif->enhanced_arp_stats.arp_rsp_rx_count_by_upper_mac++;
			}
		}
	}
#endif

		rx_skb->dev = dev;
		rx_skb->ip_summed = CHECKSUM_NONE;
		rx_skb->protocol = eth_type_trans(rx_skb, dev);

		slsi_dbg_untrack_skb(rx_skb);

		SLSI_DBG4(sdev, SLSI_RX, "pass %u bytes to local stack\n", rx_skb->len);
		netif_rx_ni(rx_skb);
		slsi_wakelock_timeout(&sdev->wlan_wl_to, SLSI_RX_WAKELOCK_TIME);
	}
}

static void slsi_rx_data_ind(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_peer *peer = NULL;
	struct ethhdr *eth_hdr = (struct ethhdr *)fapi_get_data(skb);
	u16 seq_num;

	if (!((fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor) == FAPI_DATAUNITDESCRIPTOR_IEEE802_3_FRAME) ||
	    (fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor) == FAPI_DATAUNITDESCRIPTOR_IEEE802_11_FRAME) ||
	    (fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor) == FAPI_DATAUNITDESCRIPTOR_AMSDU))) {
		WARN_ON(1);
		slsi_kfree_skb(skb);
		return;
	}

	seq_num = fapi_get_u16(skb, u.ma_unitdata_ind.sequence_number);
	SLSI_NET_DBG4(dev, SLSI_RX, "ma_unitdata_ind(vif:%d, dest:%pM, src:%pM, datatype:%d, priority:%d, s:%d)\n",
		      fapi_get_vif(skb),
		      eth_hdr->h_dest,
		      eth_hdr->h_source,
		      fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor),
		      fapi_get_u16(skb, u.ma_unitdata_ind.priority),
		      (seq_num & SLSI_RX_SEQ_NUM_MASK));

	peer = slsi_get_peer_from_mac(sdev, dev, eth_hdr->h_source);
	if (!peer) {
		SLSI_NET_DBG1(dev, SLSI_RX, "Packet dropped, peer not found\n");
		/* Race in Data plane (Shows up in fw test mode) */
		slsi_kfree_skb(skb);
		return;
	}

	/* discard data frames if received before key negotiations are completed */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_AP && peer->connected_state != SLSI_STA_CONN_STATE_CONNECTED) {
		SLSI_NET_WARN(dev, "Packet dropped (peer connection not complete (state:%u))\n", peer->connected_state);
		slsi_kfree_skb(skb);
		return;
	}

	/* skip BA reorder if the destination address is Multicast */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && (is_multicast_ether_addr(eth_hdr->h_dest))) {
		/* Skip BA reorder and pass the frames Up */
		SLSI_NET_DBG2(dev, SLSI_RX, "Multicast/Broadcast packet received in STA mode(seq: %d) skip BA\n", (seq_num & SLSI_RX_SEQ_NUM_MASK));
		slsi_rx_data_deliver_skb(sdev, dev, skb);
		return;
	}

	/* When TDLS connection has just been closed a few last frame may still arrive from the closed connection.
	 * This frames must not be injected in to the block session with the AP as the sequence numbers are different
	 * that will confuse the BA process. Therefore we have to skip BA for those frames.
	 */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && peer->aid < SLSI_TDLS_PEER_INDEX_MIN && (seq_num & SLSI_RX_VIA_TDLS_LINK)) {
		if (printk_ratelimit())
			SLSI_NET_WARN(dev, "Packet received from TDLS but no TDLS exists (seq: %x) Skip BA\n", seq_num);

		/* Skip BA reorder and pass the frames Up */
		slsi_rx_data_deliver_skb(sdev, dev, skb);
		return;
	}

	/* TDLS is enabled for the PEER but still packet is received through the AP. Process this packet with the AP PEER */
	if (ndev_vif->vif_type == FAPI_VIFTYPE_STATION && peer->aid >= SLSI_TDLS_PEER_INDEX_MIN && (!(seq_num & SLSI_RX_VIA_TDLS_LINK))) {
		SLSI_NET_DBG2(dev, SLSI_TDLS, "Packet received from TDLS peer through the AP(seq: %x)\n", seq_num);
		peer = slsi_get_peer_from_qs(sdev, dev, SLSI_STA_PEER_QUEUESET);
		if (!peer) {
			SLSI_NET_WARN(dev, "Packet dropped (AP peer not found)\n");
			slsi_kfree_skb(skb);
			return;
		}
	}

	/* If frame belongs to a negotiated BA, BA will consume the frame */
	if (slsi_ba_check(peer, fapi_get_u16(skb, u.ma_unitdata_ind.priority)))
		if (!slsi_ba_process_frame(dev, peer, skb, (seq_num & SLSI_RX_SEQ_NUM_MASK),
					   fapi_get_u16(skb, u.ma_unitdata_ind.priority)))
			return;

	/* Pass to next receive process */
	slsi_rx_data_deliver_skb(sdev, dev, skb);
}

static int slsi_rx_data_cfm(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	u16 host_tag = fapi_get_u16(skb, u.ma_unitdata_cfm.host_tag);

	SLSI_NET_DBG1(dev, SLSI_TX, "ma_unitdata_cfm(vif:%d, host_tag:0x%x, status:%d)\n",
		      fapi_get_vif(skb),
		      host_tag,
		      fapi_get_u16(skb, u.ma_unitdata_cfm.transmission_status));
	if (fapi_get_u16(skb, u.ma_unitdata_cfm.transmission_status) == FAPI_TRANSMISSIONSTATUS_RETRY_LIMIT)
		ndev_vif->tx_no_ack[SLSI_HOST_TAG_TRAFFIC_QUEUE(host_tag)]++;

	slsi_kfree_skb(skb);
	return 0;
}

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
static int slsi_rx_napi_process(struct slsi_dev *sdev, struct sk_buff *skb)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;
	u16 vif;

	vif = fapi_get_vif(skb);

	rcu_read_lock();
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if (vif >= SLSI_NAN_DATA_IFINDEX_START && fapi_get_sigid(skb) == MA_UNITDATA_IND) {
		struct ethhdr *eth_hdr = (struct ethhdr *)fapi_get_data(skb);
		u32 data_len = fapi_get_datalen(skb);

		if (!eth_hdr || data_len < sizeof(*eth_hdr)) {
			SLSI_ERR(sdev, "ma_untidata_ind dropped. datalen:%d\n", data_len);
			rcu_read_unlock();
			return 0; /* return success */
		}
		dev = slsi_get_netdev_by_mac_addr(sdev, eth_hdr->h_dest, SLSI_NAN_DATA_IFINDEX_START);
	} else {
		dev = slsi_get_netdev_rcu(sdev, vif);
	}
#else
	dev = slsi_get_netdev_rcu(sdev, vif);
#endif
	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		return -EINVAL;
	}
	rcu_read_unlock();

	ndev_vif = netdev_priv(dev);

	switch (fapi_get_u16(skb, id)) {
	case MA_UNITDATA_IND:
		slsi_rx_data_ind(sdev, dev, skb);

		/* SKBs in a BA session are not passed yet */
		if (atomic_read(&ndev_vif->ba_flush)) {
			atomic_set(&ndev_vif->ba_flush, 0);
			slsi_ba_process_complete(dev, false);
		}
		break;
	case MA_UNITDATA_CFM:
		(void)slsi_rx_data_cfm(sdev, dev, skb);
		break;
	default:
		SLSI_DBG1(sdev, SLSI_RX, "Unexpected Data: 0x%.4x\n", fapi_get_sigid(skb));
		slsi_kfree_skb(skb);
		break;
	}
	return 0;
}
#endif
void slsi_rx_netdev_data_work(struct work_struct *work)
{
	struct slsi_skb_work *w = container_of(work, struct slsi_skb_work, work);
	struct slsi_dev *sdev = w->sdev;
	struct net_device *dev = w->dev;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct sk_buff *skb;

	if (WARN_ON(!dev))
		return;

	slsi_wakelock(&sdev->wlan_wl);

	while (1) {
		SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
		if (!ndev_vif->activated) {
			slsi_skb_queue_purge(&w->queue);
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			break;
		}
		if (atomic_read(&ndev_vif->ba_flush)) {
			atomic_set(&ndev_vif->ba_flush, 0);
			slsi_ba_process_complete(dev);
		}

		skb = slsi_skb_work_dequeue(w);
		if (!skb) {
			SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			break;
		}

		slsi_debug_frame(sdev, dev, skb, "RX");
		switch (fapi_get_u16(skb, id)) {
		case MA_UNITDATA_IND:
			slsi_rx_data_ind(sdev, dev, skb);
			break;
		case MA_UNITDATA_CFM:
			(void)slsi_rx_data_cfm(sdev, dev, skb);
			break;
		default:
			SLSI_DBG1(sdev, SLSI_MLME, "Unexpected Data: 0x%.4x\n", fapi_get_sigid(skb));
			slsi_kfree_skb(skb);
			break;
		}
		SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	}
	slsi_wakeunlock(&sdev->wlan_wl);
}

static int slsi_rx_queue_data(struct slsi_dev *sdev, struct sk_buff *skb)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;
	int vif;

	vif = fapi_get_vif(skb);

	rcu_read_lock();
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if (vif >= SLSI_NAN_DATA_IFINDEX_START && fapi_get_sigid(skb) == MA_UNITDATA_IND) {
		struct ethhdr *eth_hdr = (struct ethhdr *)fapi_get_data(skb);
		u32 data_len = fapi_get_datalen(skb);

		if (!eth_hdr || data_len < sizeof(*eth_hdr)) {
			SLSI_ERR(sdev, "ma_untidata_ind dropped. datalen:%d\n", data_len);
			rcu_read_unlock();
			return 0; /* return success */
		}
		dev = slsi_get_netdev_by_mac_addr(sdev, eth_hdr->h_dest, SLSI_NAN_DATA_IFINDEX_START);
	} else {
		dev = slsi_get_netdev_rcu(sdev, vif);
	}
#else
	dev = slsi_get_netdev_rcu(sdev, vif);
#endif

	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		goto err;
	}
	ndev_vif = netdev_priv(dev);
	SCSC_WLOG_PKTFATE_LOG_RX_DATA_FRAME(fapi_get_u16(skb, u.ma_unitdata_ind.data_unit_descriptor),
					    fapi_get_data(skb), fapi_get_datalen(skb));

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
	slsi_rx_data_napi(sdev, dev, skb, false);
#else
	slsi_skb_work_enqueue(&ndev_vif->rx_data, skb);
#endif
	rcu_read_unlock();
	return 0;
err:
	return -EINVAL;
}

static int sap_ma_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb)
{
	switch (fapi_get_sigid(skb)) {
	case MA_UNITDATA_IND:
	case MA_UNITDATA_CFM:
		return slsi_rx_queue_data(sdev, skb);
	case MA_BLOCKACK_IND:
		/* It is anomolous to handle the MA_BLOCKACK_IND in the
		 * mlme wq.
		 */
		return slsi_rx_enqueue_netdev_mlme(sdev, skb, fapi_get_vif(skb));
	default:
		break;
	}

	SLSI_ERR_NODEV("Shouldn't be getting here!\n");
	return -EINVAL;
}

/* Adjust the scod value and flow control appropriately. */
static int sap_ma_txdone(struct slsi_dev *sdev, u16 colour)
{
	struct net_device *dev;
	struct slsi_peer *peer;
	u16 vif, peer_index, ac;

	/* Extract information from the coloured mbulk */
	/* colour is defined as: */
	/* u16 register bits:
	 * 0      - do not use
	 * [2:1]  - vif
	 * [7:3]  - peer_index
	 * [10:8] - ac queue
	 */
	vif = (colour & 0x6) >> 1;
	peer_index = (colour & 0xf8) >> 3;
	ac = (colour & 0x300) >> 8;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);

	if (!dev) {
		SLSI_ERR(sdev, "netdev(%d) No longer exists\n", vif);
		rcu_read_unlock();
		return -EINVAL;
	}
	rcu_read_unlock();

	if (peer_index <= SLSI_PEER_INDEX_MAX) {
		/* peer_index = 0 for Multicast queues */
		if (peer_index == 0) {
			struct netdev_vif *ndev_vif = netdev_priv(dev);

			return scsc_wifi_fcq_receive_data(dev, &ndev_vif->ap.group_data_qs, ac, sdev, vif, peer_index);
		}
		peer = slsi_get_peer_from_qs(sdev, dev, MAP_AID_TO_QS(peer_index));
		if (peer)
			return scsc_wifi_fcq_receive_data(dev, &peer->data_qs, ac, sdev, vif, peer_index);

		SLSI_DBG3(sdev, SLSI_RX, "peer record NOT found for peer_index=%d\n", peer_index);
		/* We need to handle this case as special. Peer disappeared bug hip4
		 * is sending back the colours to free.
		 */
		return scsc_wifi_fcq_receive_data_no_peer(dev, ac, sdev, vif, peer_index);
	}
	SLSI_ERR(sdev, "illegal peer_index vif=%d peer_index=%d\n", vif, peer_index);
	return -EINVAL;
}

int sap_ma_init(void)
{
	SLSI_INFO_NODEV("Registering SAP\n");
	slsi_hip_sap_register(&sap_ma);
	return 0;
}

int sap_ma_deinit(void)
{
	SLSI_INFO_NODEV("Unregistering SAP\n");
	slsi_hip_sap_unregister(&sap_ma);
	return 0;
}
