/*****************************************************************************
 *
 * Copyright (c) 2012 - 2018 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <net/sch_generic.h>
#include <linux/if_ether.h>

#include "debug.h"
#include "netif.h"
#include "dev.h"
#include "mgt.h"
#include "scsc_wifi_fcq.h"
#include "ioctl.h"
#include "mib.h"

#define IP4_OFFSET_TO_TOS_FIELD		1
#define IP6_OFFSET_TO_TC_FIELD_0	0
#define IP6_OFFSET_TO_TC_FIELD_1	1
#define FIELD_TO_DSCP			2

/* DSCP */
/* (RFC5865) */
#define DSCP_VA		0x2C
/* (RFC3246) */
#define DSCP_EF		0x2E
/* (RFC2597) */
#define DSCP_AF43	0x26
#define DSCP_AF42	0x24
#define DSCP_AF41	0x22
#define DSCP_AF33	0x1E
#define DSCP_AF32	0x1C
#define DSCP_AF31	0x1A
#define DSCP_AF23	0x16
#define DSCP_AF22	0x14
#define DSCP_AF21	0x12
#define DSCP_AF13	0x0E
#define DSCP_AF12	0x0C
#define DSCP_AF11	0x0A
/* (RFC2474) */
#define CS7		0x38
#define CS6		0x30
#define CS5		0x28
#define CS4		0x20
#define CS3		0x18
#define CS2		0x10
#define CS0		0x00
/* (RFC3662) */
#define CS1		0x08

#define SLSI_TX_WAKELOCK_TIME (100)

static bool tcp_ack_suppression_disable;
module_param(tcp_ack_suppression_disable, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_disable, "Disable TCP ack suppression feature");

static bool tcp_ack_suppression_disable_2g;
module_param(tcp_ack_suppression_disable_2g, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_disable_2g, "Disable TCP ack suppression for only 2.4GHz band");

static bool tcp_ack_suppression_monitor = true;
module_param(tcp_ack_suppression_monitor, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_monitor, "TCP ack suppression throughput monitor: Y: enable (default), N: disable");

static uint tcp_ack_suppression_monitor_interval = 500;
module_param(tcp_ack_suppression_monitor_interval, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_monitor_interval, "Sampling interval (in ms) for throughput monitor");

static uint tcp_ack_suppression_timeout = 16;
module_param(tcp_ack_suppression_timeout, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_timeout, "Timeout (in ms) before cached TCP ack is flushed to tx");

static uint tcp_ack_suppression_max = 16;
module_param(tcp_ack_suppression_max, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_max, "Maximum number of TCP acks suppressed before latest flushed to tx");

static uint tcp_ack_suppression_rate_very_high = 100;
module_param(tcp_ack_suppression_rate_very_high, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_very_high, "Rate (in Mbps) to apply very high degree of suppression");

static uint tcp_ack_suppression_rate_very_high_timeout = 4;
module_param(tcp_ack_suppression_rate_very_high_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_very_high_timeout, "Timeout (in ms) before cached TCP ack is flushed in very high rate");

static uint tcp_ack_suppression_rate_very_high_acks = 20;
module_param(tcp_ack_suppression_rate_very_high_acks, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_very_high_acks, "Maximum number of TCP acks suppressed before latest flushed in very high rate");

static uint tcp_ack_suppression_rate_high = 20;
module_param(tcp_ack_suppression_rate_high, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_high, "Rate (in Mbps) to apply high degree of suppression");

static uint tcp_ack_suppression_rate_high_timeout = 4;
module_param(tcp_ack_suppression_rate_high_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_high_timeout, "Timeout (in ms) before cached TCP ack is flushed in high rate");

static uint tcp_ack_suppression_rate_high_acks = 16;
module_param(tcp_ack_suppression_rate_high_acks, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_high_acks, "Maximum number of TCP acks suppressed before latest flushed in high rate");

static uint tcp_ack_suppression_rate_low = 1;
module_param(tcp_ack_suppression_rate_low, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_low, "Rate (in Mbps) to apply low degree of suppression");

static uint tcp_ack_suppression_rate_low_timeout = 4;
module_param(tcp_ack_suppression_rate_low_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_low_timeout, "Timeout (in ms) before cached TCP ack is flushed in low rate");

static uint tcp_ack_suppression_rate_low_acks = 10;
module_param(tcp_ack_suppression_rate_low_acks, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rate_low_acks, "Maximum number of TCP acks suppressed before latest flushed in low rate");

static uint tcp_ack_suppression_slow_start_acks = 512;
module_param(tcp_ack_suppression_slow_start_acks, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_slow_start_acks, "Maximum number of Acks sent in slow start");

static uint tcp_ack_suppression_rcv_window = 128;
module_param(tcp_ack_suppression_rcv_window, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_suppression_rcv_window, "Receive window size (in unit of Kbytes) that triggers Ack suppression");

/* Indicate WLAN firmware to send TCP ACK frames at specific TX rates. */
static bool tcp_ack_robustness = true;
module_param(tcp_ack_robustness, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(tcp_ack_robustness, "TCP robustness. Run-time option - (default: Y)");

static void slsi_netif_tcp_ack_suppression_timeout(unsigned long data);
static int slsi_netif_tcp_ack_suppression_start(struct net_device *dev);
static int slsi_netif_tcp_ack_suppression_stop(struct net_device *dev);
static struct sk_buff *slsi_netif_tcp_ack_suppression_pkt(struct net_device *dev, struct sk_buff *skb);

#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
void slsi_net_randomize_nmi_ndi(struct slsi_dev *sdev)
{
	int               exor_base = 1, exor_byte = 5, i;
	u8                random_mac[ETH_ALEN];

	/* Randomize mac address */
	SLSI_ETHER_COPY(random_mac, sdev->hw_addr);
	/* If random number is same as actual bytes in hw_address
	 * try random again. hope 2nd random will not be same as
	 * bytes in hw_address
	 */
	slsi_get_random_bytes(&random_mac[3], 3);
	if (!memcmp(&random_mac[3], &sdev->hw_addr[3], 3))
		slsi_get_random_bytes(&random_mac[3], 3);
	SLSI_ETHER_COPY(sdev->netdev_addresses[SLSI_NET_INDEX_NAN], random_mac);
	/* Set the local bit */
	sdev->netdev_addresses[SLSI_NET_INDEX_NAN][0] |= 0x02;
	/* EXOR 4th byte with 0x80 */
	sdev->netdev_addresses[SLSI_NET_INDEX_NAN][3] ^= 0x80;
	for (i = SLSI_NAN_DATA_IFINDEX_START; i < CONFIG_SCSC_WLAN_MAX_INTERFACES + 1; i++) {
		SLSI_ETHER_COPY(sdev->netdev_addresses[i], random_mac);
		sdev->netdev_addresses[i][0] |= 0x02;
		sdev->netdev_addresses[i][exor_byte] ^= exor_base;
		exor_base++;
		/* currently supports upto 15 mac address for nan
		 * data interface
		 */
		if (exor_base > 0xf)
			break;
	}
}
#endif

/* Net Device callback operations */
static int slsi_net_open(struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               err;
	unsigned char	  dev_addr_zero_check[ETH_ALEN];

	SLSI_NET_DBG2(dev, SLSI_NETDEV, "iface_num = %d\n", ndev_vif->ifnum);
	SLSI_INFO(sdev, "%s -- Recovery Status:%d\n",
		  netdev_name(dev), sdev->recovery_status);

	if (WARN_ON(ndev_vif->is_available))
		return -EINVAL;

	if (sdev->mlme_blocked) {
		SLSI_NET_DBG2(dev, SLSI_NETDEV, "MLME Blocked. Reject net_open\n");
		return -EIO;
	}

	slsi_wakelock(&sdev->wlan_wl);

	/* check if request to rf test mode. */
	slsi_check_rf_test_mode();

	err = slsi_start(sdev);
	if (WARN_ON(err)) {
		slsi_wakeunlock(&sdev->wlan_wl);
		return err;
	}

	if (!sdev->netdev_up_count) {
		slsi_get_hw_mac_address(sdev, sdev->hw_addr);
		/* Assign Addresses */
		SLSI_ETHER_COPY(sdev->netdev_addresses[SLSI_NET_INDEX_WLAN], sdev->hw_addr);

		SLSI_ETHER_COPY(sdev->netdev_addresses[SLSI_NET_INDEX_P2P],  sdev->hw_addr);
		/* Set the local bit */
		sdev->netdev_addresses[SLSI_NET_INDEX_P2P][0] |= 0x02;

		SLSI_ETHER_COPY(sdev->netdev_addresses[SLSI_NET_INDEX_P2PX_SWLAN], sdev->hw_addr);
		/* Set the local bit */
		sdev->netdev_addresses[SLSI_NET_INDEX_P2PX_SWLAN][0] |= 0x02;
		/* EXOR 5th byte with 0x80 */
		sdev->netdev_addresses[SLSI_NET_INDEX_P2PX_SWLAN][4] ^= 0x80;
#if CONFIG_SCSC_WLAN_MAX_INTERFACES >= 4 && defined(CONFIG_SCSC_WIFI_NAN_ENABLE)
		slsi_net_randomize_nmi_ndi(sdev);
#endif
		sdev->initial_scan = true;
	}

	memset(dev_addr_zero_check, 0, ETH_ALEN);
	if (!memcmp(dev->dev_addr, dev_addr_zero_check, ETH_ALEN)) {
#if defined(CONFIG_SCSC_WLAN_WIFI_SHARING) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
		if (SLSI_IS_VIF_INDEX_MHS(sdev, ndev_vif))
			SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[SLSI_NET_INDEX_P2P]);
		else
			SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[ndev_vif->ifnum]);
#else
		SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[ndev_vif->ifnum]);
#endif
	}
	SLSI_ETHER_COPY(dev->perm_addr, sdev->netdev_addresses[ndev_vif->ifnum]);
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	ndev_vif->is_available = true;
	sdev->netdev_up_count++;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&ndev_vif->sig_wait.completion);
#else
	INIT_COMPLETION(ndev_vif->sig_wait.completion);
#endif
#ifndef CONFIG_ARM
	slsi_netif_tcp_ack_suppression_start(dev);
#endif

#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if (ndev_vif->ifnum >= SLSI_NAN_DATA_IFINDEX_START)
		netif_carrier_on(dev);
#endif
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);

	netif_tx_start_all_queues(dev);
	slsi_wakeunlock(&sdev->wlan_wl);

	return 0;
}

static int slsi_net_stop(struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;

	SLSI_NET_DBG1(dev, SLSI_NETDEV, "\n");
	SLSI_INFO(sdev, "%s -- Recovery Status:%d\n",
		  netdev_name(dev), sdev->recovery_status);

	slsi_wakelock(&sdev->wlan_wl);
	netif_tx_stop_all_queues(dev);
	sdev->initial_scan = false;

	if (!ndev_vif->is_available) {
		/* May have been taken out by the Chip going down */
		SLSI_NET_DBG1(dev, SLSI_NETDEV, "Not available.\n");
		slsi_wakeunlock(&sdev->wlan_wl);
		return 0;
	}
	slsi_netif_tcp_ack_suppression_stop(dev);
	slsi_stop_net_dev(sdev, dev);

	sdev->allow_switch_40_mhz = true;
	sdev->allow_switch_80_mhz = true;
	sdev->acs_channel_switched = false;
	slsi_wakeunlock(&sdev->wlan_wl);
	return 0;
}

/* This is called after the WE handlers */
static int slsi_net_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	if (cmd == SIOCDEVPRIVATE + 2) { /* 0x89f0 + 2 from wpa_supplicant */
		return slsi_ioctl(dev, rq, cmd);
	}

	return -EOPNOTSUPP;
}

static struct net_device_stats *slsi_net_get_stats(struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);

	SLSI_NET_DBG4(dev, SLSI_NETDEV, "\n");
	return &ndev_vif->stats;
}

#ifdef CONFIG_SCSC_USE_WMM_TOS
static u16 slsi_get_priority_from_tos(u8 *frame, u16 proto)
{
	if (WARN_ON(!frame))
		return FAPI_PRIORITY_QOS_UP0;

	switch (proto) {
	case ETH_P_IP:    /* IPv4 */
		return (u16)(((frame[IP4_OFFSET_TO_TOS_FIELD]) & 0xE0) >> 5);

	case ETH_P_IPV6:    /* IPv6 */
		return (u16)((*frame & 0x0E) >> 1);

	default:
		return FAPI_PRIORITY_QOS_UP0;
	}
}

#else
static u16 slsi_get_priority_from_tos_dscp(u8 *frame, u16 proto)
{
	u8 dscp;

	if (WARN_ON(!frame))
		return FAPI_PRIORITY_QOS_UP0;

	switch (proto) {
	case ETH_P_IP:    /* IPv4 */
		dscp = frame[IP4_OFFSET_TO_TOS_FIELD] >> FIELD_TO_DSCP;
		break;

	case ETH_P_IPV6:    /* IPv6 */
		/* Get traffic class */
		dscp = (((frame[IP6_OFFSET_TO_TC_FIELD_0] & 0x0F) << 4) |
			((frame[IP6_OFFSET_TO_TC_FIELD_1] & 0xF0) >> 4)) >> FIELD_TO_DSCP;
		break;

	default:
		return FAPI_PRIORITY_QOS_UP0;
	}
/* DSCP table based in RFC8325 from Android 10 */
#if (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 10)
	switch (dscp) {
	case CS7:
		return FAPI_PRIORITY_QOS_UP7;
	case CS6:
	case DSCP_EF:
	case DSCP_VA:
		return FAPI_PRIORITY_QOS_UP6;
	case CS5:
		return FAPI_PRIORITY_QOS_UP5;
	case DSCP_AF41:
	case DSCP_AF42:
	case DSCP_AF43:
	case CS4:
	case DSCP_AF31:
	case DSCP_AF32:
	case DSCP_AF33:
	case CS3:
		return FAPI_PRIORITY_QOS_UP4;
	case DSCP_AF21:
	case DSCP_AF22:
	case DSCP_AF23:
		return FAPI_PRIORITY_QOS_UP3;
	case CS2:
	case DSCP_AF11:
	case DSCP_AF12:
	case DSCP_AF13:
	case CS0:
		return FAPI_PRIORITY_QOS_UP0;
	case CS1:
		return FAPI_PRIORITY_QOS_UP1;
	default:
		return FAPI_PRIORITY_QOS_UP0;
	}
#else
	switch (dscp) {
	case DSCP_EF:
	case DSCP_VA:
		return FAPI_PRIORITY_QOS_UP6;
	case DSCP_AF43:
	case DSCP_AF42:
	case DSCP_AF41:
		return FAPI_PRIORITY_QOS_UP5;
	case DSCP_AF33:
	case DSCP_AF32:
	case DSCP_AF31:
	case DSCP_AF23:
	case DSCP_AF22:
	case DSCP_AF21:
	case DSCP_AF13:
	case DSCP_AF12:
	case DSCP_AF11:
		return FAPI_PRIORITY_QOS_UP0;
	case CS7:
		return FAPI_PRIORITY_QOS_UP7;
	case CS6:
		return FAPI_PRIORITY_QOS_UP6;
	case CS5:
		return FAPI_PRIORITY_QOS_UP5;
	case CS4:
		return FAPI_PRIORITY_QOS_UP4;
	case CS3:
		return FAPI_PRIORITY_QOS_UP3;
	case CS2:
		return FAPI_PRIORITY_QOS_UP2;
	case CS1:
		return FAPI_PRIORITY_QOS_UP1;
	case CS0:
		return FAPI_PRIORITY_QOS_UP0;
	default:
		return FAPI_PRIORITY_QOS_UP0;
	}
#endif
}

#endif

static bool slsi_net_downgrade_ac(struct net_device *dev, struct sk_buff *skb)
{
	SLSI_UNUSED_PARAMETER(dev);

	switch (skb->priority) {
	case 6:
	case 7:
		skb->priority = FAPI_PRIORITY_QOS_UP5; /* VO -> VI */
		return true;
	case 4:
	case 5:
		skb->priority = FAPI_PRIORITY_QOS_UP3; /* VI -> BE */
		return true;
	case 0:
	case 3:
		skb->priority = FAPI_PRIORITY_QOS_UP2; /* BE -> BK */
		return true;
	default:
		return false;
	}
}

static u8 slsi_net_up_to_ac_mapping(u8 priority)
{
	switch (priority) {
	case FAPI_PRIORITY_QOS_UP6:
	case FAPI_PRIORITY_QOS_UP7:
		return BIT(FAPI_PRIORITY_QOS_UP6) | BIT(FAPI_PRIORITY_QOS_UP7);
	case FAPI_PRIORITY_QOS_UP4:
	case FAPI_PRIORITY_QOS_UP5:
		return BIT(FAPI_PRIORITY_QOS_UP4) | BIT(FAPI_PRIORITY_QOS_UP5);
	case FAPI_PRIORITY_QOS_UP0:
	case FAPI_PRIORITY_QOS_UP3:
		return BIT(FAPI_PRIORITY_QOS_UP0) | BIT(FAPI_PRIORITY_QOS_UP3);
	default:
		return BIT(FAPI_PRIORITY_QOS_UP1) | BIT(FAPI_PRIORITY_QOS_UP2);
	}
}

enum slsi_traffic_q slsi_frame_priority_to_ac_queue(u16 priority)
{
	switch (priority) {
	case FAPI_PRIORITY_QOS_UP0:
	case FAPI_PRIORITY_QOS_UP3:
		return SLSI_TRAFFIC_Q_BE;
	case FAPI_PRIORITY_QOS_UP1:
	case FAPI_PRIORITY_QOS_UP2:
		return SLSI_TRAFFIC_Q_BK;
	case FAPI_PRIORITY_QOS_UP4:
	case FAPI_PRIORITY_QOS_UP5:
		return SLSI_TRAFFIC_Q_VI;
	case FAPI_PRIORITY_QOS_UP6:
	case FAPI_PRIORITY_QOS_UP7:
		return SLSI_TRAFFIC_Q_VO;
	default:
		return SLSI_TRAFFIC_Q_BE;
	}
}

int slsi_ac_to_tids(enum slsi_traffic_q ac, int *tids)
{
	switch (ac) {
	case SLSI_TRAFFIC_Q_BE:
		tids[0] = FAPI_PRIORITY_QOS_UP0;
		tids[1] = FAPI_PRIORITY_QOS_UP3;
		break;

	case SLSI_TRAFFIC_Q_BK:
		tids[0] = FAPI_PRIORITY_QOS_UP1;
		tids[1] = FAPI_PRIORITY_QOS_UP2;
		break;

	case SLSI_TRAFFIC_Q_VI:
		tids[0] = FAPI_PRIORITY_QOS_UP4;
		tids[1] = FAPI_PRIORITY_QOS_UP5;
		break;

	case SLSI_TRAFFIC_Q_VO:
		tids[0] = FAPI_PRIORITY_QOS_UP6;
		tids[1] = FAPI_PRIORITY_QOS_UP7;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void slsi_net_downgrade_pri(struct net_device *dev, struct slsi_peer *peer,
				   struct sk_buff *skb)
{
	/* in case we are a client downgrade the ac if acm is
	 * set and tspec is not established
	 */
	while (unlikely(peer->wmm_acm & BIT(skb->priority)) &&
	       !(peer->tspec_established & slsi_net_up_to_ac_mapping(skb->priority))) {
		SLSI_NET_DBG3(dev, SLSI_NETDEV, "Downgrading from UP:%d\n", skb->priority);
		if (!slsi_net_downgrade_ac(dev, skb))
			break;
	}
	SLSI_NET_DBG4(dev, SLSI_NETDEV, "To UP:%d\n", skb->priority);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
static u16 slsi_net_select_queue(struct net_device *dev, struct sk_buff *skb, void *accel_priv, select_queue_fallback_t fallback)
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
static u16 slsi_net_select_queue(struct net_device *dev, struct sk_buff *skb, void *accel_priv)
#else
static u16 slsi_net_select_queue(struct net_device *dev, struct sk_buff *skb)
#endif
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	u16               netif_q = 0;
	struct ethhdr     *ehdr = (struct ethhdr *)skb->data;
	int               proto = 0;
	struct slsi_peer  *peer;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	(void)accel_priv;
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
	(void)fallback;
#endif
	SLSI_NET_DBG4(dev, SLSI_NETDEV, "\n");

	/* Defensive check for uninitialized mac header */
	if (!skb_mac_header_was_set(skb))
		skb_reset_mac_header(skb);

	proto = be16_to_cpu(eth_hdr(skb)->h_proto);

	switch (proto) {
	default:
		/* SLSI_NETIF_Q_PRIORITY is used only for EAP, ARP and IP frames with DHCP */
		break;
	case ETH_P_PAE:
	case ETH_P_WAI:
		SLSI_NET_DBG3(dev, SLSI_TX, "EAP packet. Priority Queue Selected\n");
		return SLSI_NETIF_Q_PRIORITY;
	case ETH_P_ARP:
		SLSI_NET_DBG3(dev, SLSI_TX, "ARP frame. Priority Queue Selected\n");
		return SLSI_NETIF_Q_PRIORITY;
	case ETH_P_IP:
		if (slsi_is_dhcp_packet(skb->data) == SLSI_TX_IS_NOT_DHCP)
			break;
		SLSI_NET_DBG3(dev, SLSI_TX, "DHCP packet. Priority Queue Selected\n");
		return SLSI_NETIF_Q_PRIORITY;
	}

	if (ndev_vif->vif_type == FAPI_VIFTYPE_AP)
		/* MULTICAST/BROADCAST Queue is only used for AP */
		if (is_multicast_ether_addr(ehdr->h_dest)) {
			SLSI_NET_DBG3(dev, SLSI_TX, "Multicast AC queue will be selected\n");
#ifdef CONFIG_SCSC_USE_WMM_TOS
			skb->priority = slsi_get_priority_from_tos(skb->data + ETH_HLEN, proto);
#else
			skb->priority = slsi_get_priority_from_tos_dscp(skb->data + ETH_HLEN, proto);
#endif
			return slsi_netif_get_multicast_queue(slsi_frame_priority_to_ac_queue(skb->priority));
		}

	slsi_spinlock_lock(&ndev_vif->peer_lock);
	peer = slsi_get_peer_from_mac(sdev, dev, ehdr->h_dest);
	if (!peer) {
		SLSI_NET_DBG1(dev, SLSI_TX, "Peer NOT found : %pM\n", ehdr->h_dest);
		slsi_spinlock_unlock(&ndev_vif->peer_lock);
		return SLSI_NETIF_Q_DISCARD;
	}

	if (peer->qos_enabled) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 14, 0))
		if (peer->qos_map_set) {			/*802.11 QoS for interworking*/
			skb->priority = cfg80211_classify8021d(skb, &peer->qos_map);
		} else
#endif
		{
#ifdef CONFIG_SCSC_USE_WMM_TOS
			skb->priority = slsi_get_priority_from_tos(skb->data + ETH_HLEN, proto);
#else
			skb->priority = slsi_get_priority_from_tos_dscp(skb->data + ETH_HLEN, proto);
#endif
		}
	} else{
		skb->priority = FAPI_PRIORITY_QOS_UP0;
	}

	/* Downgrade the priority if acm bit is set and tspec is not established */
	slsi_net_downgrade_pri(dev, peer, skb);

	netif_q = slsi_netif_get_peer_queue(peer->queueset, slsi_frame_priority_to_ac_queue(skb->priority));
	SLSI_NET_DBG3(dev, SLSI_TX, "%u Queue Selected\n", netif_q);
	slsi_spinlock_unlock(&ndev_vif->peer_lock);
	return netif_q;
}

#define UNUSED(x) \
	\
	((void)(x))

void slsi_tdls_move_packets(struct slsi_dev *sdev, struct net_device *dev,
			    struct slsi_peer *sta_peer, struct slsi_peer *tdls_peer, bool connection)
{
	struct netdev_vif *netdev_vif = netdev_priv(dev);
	struct sk_buff                *skb = NULL;
	struct ethhdr                 *ehdr;
	struct Qdisc                  *qd;
	u32                           num_pkts;
	u16                           staq;
	u16                           tdlsq;
	u16                           netq;
	u16                           i;
	u16                           j;
	int index;
	struct slsi_tcp_ack_s *tcp_ack;

	/* Get the netdev queue number from queueset */
	staq = slsi_netif_get_peer_queue(sta_peer->queueset, 0);
	tdlsq = slsi_netif_get_peer_queue(tdls_peer->queueset, 0);

	SLSI_NET_DBG1(dev, SLSI_TDLS, "Connection: %d, sta_qset: %d, tdls_qset: %d, sta_netq = %d, tdls_netq = %d\n",
		      connection, sta_peer->queueset, tdls_peer->queueset, staq, tdlsq);

	/* Pause the TDLS queues and STA netdev queues */
	slsi_tx_pause_queues(sdev);

	/* walk through frames in TCP Ack suppression queue and change mapping to TDLS queue */
	for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
		tcp_ack = &netdev_vif->ack_suppression[index];
		if (!tcp_ack->state)
			continue;

		skb_queue_walk(&tcp_ack->list, skb) {
			SLSI_NET_DBG2(dev, SLSI_TDLS, "frame in TCP Ack list (peer:%pM)\n", eth_hdr(skb)->h_dest);
			/* is it destined to TDLS peer? */
			if (compare_ether_addr(tdls_peer->address, eth_hdr(skb)->h_dest) == 0) {
				if (connection) {
					/* TDLS setup: change the queue mapping to TDLS queue */
					skb->queue_mapping += (tdls_peer->queueset * SLSI_NETIF_Q_PER_PEER);
				} else {
					/* TDLS teardown: change the queue to STA queue */
					skb->queue_mapping -= (tdls_peer->queueset * SLSI_NETIF_Q_PER_PEER);
				}
			}
		}
	}

	/**
	 * For TDLS connection set PEER valid to true. After this ndo_select_queue() will select TDLSQ instead of STAQ
	 * For TDLS teardown set PEER valid to false. After this ndo_select_queue() will select STAQ instead of TDLSQ
	 */
	if (connection)
		tdls_peer->valid = true;
	else
		tdls_peer->valid = false;

	/* Move packets from netdev queues */
	for (i = 0; i < SLSI_NETIF_Q_PER_PEER; i++) {
		SLSI_NET_DBG2(dev, SLSI_TDLS, "NETQ%d: Before: tdlsq_len = %d, staq_len = %d\n",
			      i, skb_queue_len(&dev->_tx[tdlsq + i].qdisc->q), skb_queue_len(&dev->_tx[staq + i].qdisc->q));

		if (connection) {
			/* Check if any packet is already avilable in TDLS queue (most likely from last session) */
			if (skb_queue_len(&dev->_tx[tdlsq + i].qdisc->q))
				SLSI_NET_ERR(dev, "tdls_connection: Packet present in queue %d\n", tdlsq + i);

			qd = dev->_tx[staq + i].qdisc;
			/* Get the total number of packets in STAQ */
			num_pkts = skb_queue_len(&qd->q);

			/* Check all the pkt in STAQ and move the TDLS pkts to TDSLQ */
			for (j = 0; j < num_pkts; j++) {
				qd = dev->_tx[staq + i].qdisc;
				/* Dequeue the pkt form STAQ. This logic is similar to kernel API dequeue_skb() */
				skb = qd->gso_skb;
				if (skb) {
					qd->gso_skb = NULL;
					qd->q.qlen--;
				} else {
					skb = qd->dequeue(qd);
				}

				if (!skb) {
					SLSI_NET_ERR(dev, "tdls_connection: STA NETQ skb is NULL\n");
					break;
				}

				/* Change the queue mapping for the TDLS packets */
				netq = skb->queue_mapping;
				ehdr = (struct ethhdr *)skb->data;
				if (compare_ether_addr(tdls_peer->address, ehdr->h_dest) == 0) {
					netq += (tdls_peer->queueset * SLSI_NETIF_Q_PER_PEER);
					SLSI_NET_DBG3(dev, SLSI_TDLS, "NETQ%d: Queue mapping changed from %d to %d\n",
						      i, skb->queue_mapping, netq);
					skb_set_queue_mapping(skb, netq);
				}

				qd = dev->_tx[netq].qdisc;

				/* If the netdev queue is already full then enqueue() will drop the skb */
				qd->enqueue(skb, qd);
			}
		} else {
			num_pkts = skb_queue_len(&dev->_tx[tdlsq + i].qdisc->q);
			/* Move the packets from TDLS to STA queue */
			for (j = 0; j < num_pkts; j++) {
				/* Dequeue the pkt form TDLS_Q. This logic is similar to kernel API dequeue_skb() */
				qd = dev->_tx[tdlsq + i].qdisc;
				skb = qd->gso_skb;
				if (skb) {
					qd->gso_skb = NULL;
					qd->q.qlen--;
				} else {
					skb = qd->dequeue(qd);
				}

				if (!skb) {
					SLSI_NET_ERR(dev, "tdls_teardown: TDLS NETQ skb is NULL\n");
					break;
				}

				/* Update the queue mapping */
				skb_set_queue_mapping(skb, staq + i);

				/* Enqueue the packet in STA queue */
				qd = dev->_tx[staq + i].qdisc;

				/* If the netdev queue is already full then enqueue() will drop the skb */
				qd->enqueue(skb, qd);
			}
		}
		SLSI_NET_DBG2(dev, SLSI_TDLS, "NETQ%d: After : tdlsq_len = %d, staq_len = %d\n",
			      i, skb_queue_len(&dev->_tx[tdlsq + i].qdisc->q), skb_queue_len(&dev->_tx[staq + i].qdisc->q));
	}

	/* Teardown - after teardown there should not be any packet in TDLS queues */
	if (!connection)
		for (i = 0; i < SLSI_NETIF_Q_PER_PEER; i++) {
			if (skb_queue_len(&dev->_tx[tdlsq + i].qdisc->q))
				SLSI_NET_ERR(dev, "tdls_teardown: Packet present in NET queue %d\n", tdlsq + i);
		}

	/* Resume the STA and TDLS netdev queues */
	slsi_tx_unpause_queues(sdev);
}

/**
 * This is the main TX entry point for the driver.
 *
 * Ownership of the skb is transferred to another function ONLY IF such
 * function was able to deal with that skb and ended with a SUCCESS ret code.
 * Owner HAS the RESPONSIBILITY to handle the life cycle of the skb.
 *
 * In the context of this function:
 * - ownership is passed DOWN to the LOWER layers HIP-functions when skbs were
 *   SUCCESSFULLY transmitted, and there they will be FREED. As a consequence
 *   kernel netstack will receive back NETDEV_TX_OK too.
 * - ownership is KEPT HERE by this function when lower layers fails somehow
 *   to deal with the transmission of the skb. In this case the skb WOULD HAVE
 *   NOT BEEN FREED by lower layers that instead returns a proper ERRCODE.
 * - intermediate lower layer functions (NOT directly involved in failure or
 *   success) will relay any retcode up to this layer for evaluation.
 *
 *   WHAT HAPPENS THEN, is ERRCODE-dependent, and at the moment:
 *    - ENOSPC: something related to queueing happened...this should be
 *    retried....NETDEV_TX_BUSY is returned to NetStack ...packet will be
 *    requeued by the Kernel NetStack itself, using the proper queue.
 *    As a  consequence SKB is NOT FREED HERE !.
 *    - ANY OTHER ERR: all other errors are considered at the moment NOT
 *    recoverable and SO skbs are droppped(FREED) HERE...Kernel will receive
 *    the proper ERRCODE and stops dealing with the packet considering it
 *    consumed by lower layer. (same behavior as NETDEV_TX_OK)
 *
 *    BIG NOTE:
 *    As detailed in Documentation/networking/drivers.txt the above behavior
 *    of returning NETDEV_TX_BUSY to trigger requeueinng by the Kernel is
 *    discouraged and should be used ONLY in case of a real HARD error(?);
 *    the advised solution is to actively STOP the queues before finishing
 *    the available space and WAKING them up again when more free buffers
 *    would have arrived.
 */
static netdev_tx_t slsi_net_hw_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	int               r = NETDEV_TX_OK;
	struct sk_buff    *original_skb = NULL;
#ifdef CONFIG_SCSC_WLAN_DEBUG
	int               known_users = 0;
#endif
	/* Keep the packet length. The packet length will be used to increment
	 * stats for the netdev if the packet was successfully transmitted.
	 * The ownership of the SKB is passed to lower layers, so we should
	 * not refer the SKB after this point
	 */
	unsigned int packet_len = skb->len;
	enum slsi_traffic_q traffic_q = slsi_frame_priority_to_ac_queue(skb->priority);

	slsi_wakelock(&sdev->wlan_wl);
	slsi_wakelock_timeout(&sdev->wlan_wl_to, SLSI_TX_WAKELOCK_TIME);

	/* Check for misaligned (oddly aligned) data.
	 * The f/w requires 16 bit aligned.
	 * This is a corner case - for example, the kernel can generate BPDU
	 * that are oddly aligned. Therefore it is acceptable to copy these
	 * frames to a 16 bit alignment.
	 */
	if ((uintptr_t)skb->data & 0x1) {
		struct sk_buff *skb2 = NULL;
		/* Received a socket buffer aligned on an odd address.
		 * Re-align by asking for headroom.
		 */
		skb2 = skb_copy_expand(skb, SLSI_NETIF_SKB_HEADROOM, skb_tailroom(skb), GFP_ATOMIC);
		if (skb2 && (!(((uintptr_t)skb2->data) & 0x1))) {
			/* We should account for this duplication */
			original_skb = skb;
			skb = skb2;
			SLSI_NET_DBG3(dev, SLSI_TX, "Oddly aligned skb realigned\n");
		} else {
			/* Drop the packet if we can't re-align. */
			SLSI_NET_ERR(dev, "Oddly aligned skb failed realignment, dropping\n");
			if (skb2) {
				SLSI_NET_DBG3(dev, SLSI_TX, "skb_copy_expand didn't align for us\n");
				slsi_kfree_skb(skb2);
			} else {
				SLSI_NET_DBG3(dev, SLSI_TX, "skb_copy_expand failed when trying to align\n");
			}
			r = -EFAULT;
			goto evaluate;
		}
	}
	slsi_dbg_track_skb(skb, GFP_ATOMIC);

	/* Be defensive about the mac_header - some kernels have a bug where a
	 * frame can be delivered to the driver with mac_header initialised
	 * to ~0U and this causes a crash when the pointer is dereferenced to
	 * access part of the Ethernet header.
	 */
	if (!skb_mac_header_was_set(skb))
		skb_reset_mac_header(skb);

	SLSI_NET_DBG3(dev, SLSI_TX, "Proto 0x%.4X\n", be16_to_cpu(eth_hdr(skb)->h_proto));

#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	if (ndev_vif->ifnum < SLSI_NAN_DATA_IFINDEX_START) {
#endif
		if (!ndev_vif->is_available) {
			SLSI_NET_WARN(dev, "vif NOT available\n");
			r = -EFAULT;
			goto evaluate;
		}
#ifdef CONFIG_SCSC_WIFI_NAN_ENABLE
	}
#endif
	if (skb->queue_mapping == SLSI_NETIF_Q_DISCARD) {
		SLSI_NET_DBG1(dev, SLSI_TX, "Discard Queue :: Packet Dropped\n");
		r = -EIO;
		goto evaluate;
	}

#ifdef CONFIG_SCSC_WLAN_DEBUG
	known_users = atomic_read(&skb->users);
#endif

	skb = slsi_netif_tcp_ack_suppression_pkt(dev, skb);
	if (!skb) {
		slsi_wakeunlock(&sdev->wlan_wl);
		if (original_skb)
			slsi_kfree_skb(original_skb);
		return NETDEV_TX_OK;
	}

	/* SKB is owned by slsi_tx_data() ONLY IF ret value is success (0) */
	r = slsi_tx_data(sdev, dev, skb);
evaluate:
	if (r == 0) {
		/**
		 * A copy has been passed down and successfully transmitted
		 * and freed....here we free the original coming from the
		 * upper network layers....if a copy was passed down.
		 */
		if (original_skb)
			slsi_kfree_skb(original_skb);
		/* skb freed by lower layers on success...enjoy */
		dev->trans_start = jiffies;

		ndev_vif->tx_packets[traffic_q]++;
		ndev_vif->stats.tx_packets++;
		ndev_vif->stats.tx_bytes += packet_len;
		r = NETDEV_TX_OK;
	} else {
		/**
		 * Failed to send:
		 *  - if QueueFull/OutOfMBulk (-ENOSPC returned) the skb was
		 *  NOT discarded by lower layers and NETDEV_TX_BUSY should
		 *  be returned to upper layers: this will cause the skb
		 *  (THAT MUST NOT HAVE BEEN FREED BY LOWER LAYERS !)
		 *  to be requeued ...
		 *  NOTE THAT it's the original skb that will be retried
		 *  by upper netstack.
		 *  THIS CONDITION SHOULD NOT BE REACHED...NEVER...see in
		 *  the following.
		 *
		 *  - with any other -ERR instead return the error: this
		 *  anyway let the kernel think that the SKB has
		 *  been consumed, and we drop the frame and free it.
		 *
		 *  - a WARN_ON() takes care to ensure the SKB has NOT been
		 *  freed by someone despite this was NOT supposed to happen,
		 *  just before the actual freeing.
		 *
		 */
		if (r == -ENOSPC) {
			/* SLSI_NET_DBG1(dev, SLSI_TEST, "Packet Requeued...should NOT get here !\n"); */
			ndev_vif->stats.tx_fifo_errors++;
			/* Free the local copy if any ... */
			if (original_skb)
				slsi_kfree_skb(skb);
			r = NETDEV_TX_BUSY;
		} else {
#ifdef CONFIG_SCSC_WLAN_DEBUG
			WARN_ON(known_users &&
				atomic_read(&skb->users) != known_users);
#endif
			if (original_skb)
				slsi_kfree_skb(original_skb);
			slsi_kfree_skb(skb);
			ndev_vif->stats.tx_dropped++;
			/* We return the ORIGINAL Error 'r' anyway
			 * BUT Kernel treats them as TX complete anyway
			 * and assumes the SKB has been consumed.
			 */
			/* SLSI_NET_DBG1(dev, SLSI_TEST, "Packet Dropped\n"); */
		}
	}
	/* SKBs are always considered consumed if the driver
	 * returns NETDEV_TX_OK.
	 */
	slsi_wakeunlock(&sdev->wlan_wl);
	return r;
}

static netdev_features_t slsi_net_fix_features(struct net_device *dev, netdev_features_t features)
{
	SLSI_UNUSED_PARAMETER(dev);

#ifdef CONFIG_SCSC_WLAN_SG
	SLSI_NET_DBG1(dev, SLSI_RX, "Scatter-gather and GSO enabled\n");
	features |= NETIF_F_SG;
	features |= NETIF_F_GSO;
#endif

#ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO
	SLSI_NET_DBG1(dev, SLSI_RX, "napi rx gro enabled\n");
	features |= NETIF_F_GRO;
#else
	SLSI_NET_DBG1(dev, SLSI_RX, "napi rx gro enabled\n");
	features &= ~NETIF_F_GRO;
#endif
	return features;
}

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
int slsi_net_rx_poll(struct napi_struct *napi, int budget)
{
	struct netdev_vif *ndev_vif = netdev_priv(napi->dev);
	struct sk_buff    *skb = slsi_skb_dequeue(&ndev_vif->napi.rx_data);
	int               npackets = 0;

	while (skb) {
		npackets++;
		slsi_dbg_untrack_skb(skb);
#ifdef CONFIG_SCSC_WLAN_RX_NAPI_GRO
		napi_gro_receive(napi, skb);
#else
		netif_receive_skb(skb);
#endif
		if (npackets == budget)
			break;
		skb = slsi_skb_dequeue(&ndev_vif->napi.rx_data);
	}

	if (npackets < budget) {
		ndev_vif->napi.interrupt_enabled = true;
		napi_complete(napi);
	}

	return npackets;
}
#endif

static void  slsi_set_multicast_list(struct net_device *dev)
{
	struct netdev_vif     *ndev_vif = netdev_priv(dev);
	u8                    count, i = 0;
	u8                    mdns_addr[ETH_ALEN] = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFB };

#ifdef CONFIG_SCSC_WLAN_BLOCK_IPV6
	u8                    mc_addr_prefix[3] = { 0x01, 0x00, 0x5e };
#else
	u8                    mdns6_addr[ETH_ALEN] = { 0x33, 0x33, 0x00, 0x00, 0x00, 0xFB };
	const u8              solicited_node_addr[ETH_ALEN] = { 0x33, 0x33, 0xff, 0x00, 0x00, 0x01 };
	u8                    ipv6addr_suffix[3];
#endif
	struct netdev_hw_addr *ha;

	if (ndev_vif->vif_type != FAPI_VIFTYPE_STATION)
		return;

	if (!ndev_vif->is_available) {
		SLSI_NET_DBG1(dev, SLSI_NETDEV, "Not available\n");
		return;
	}

	count = netdev_mc_count(dev);
	if (!count)
		goto exit;

#ifndef CONFIG_SCSC_WLAN_BLOCK_IPV6
	slsi_spinlock_lock(&ndev_vif->ipv6addr_lock);
	memcpy(ipv6addr_suffix, &ndev_vif->ipv6address.s6_addr[13], 3);
	slsi_spinlock_unlock(&ndev_vif->ipv6addr_lock);
#endif

	netdev_for_each_mc_addr(ha, dev) {
#ifdef CONFIG_SCSC_WLAN_BLOCK_IPV6
		if ((!memcmp(ha->addr, mdns_addr, ETH_ALEN)) ||                                                   /*mDns is handled separately*/
		    (memcmp(ha->addr, mc_addr_prefix, 3))) {                                                   /*only consider IPv4 multicast addresses*/
#else
		if ((!memcmp(ha->addr, mdns_addr, ETH_ALEN)) ||
		    (!memcmp(ha->addr, mdns6_addr, ETH_ALEN)) ||        /*mDns is handled separately*/
		    (!memcmp(ha->addr, solicited_node_addr, 3) &&
		     !memcmp(&ha->addr[3], ipv6addr_suffix, 3))) { /* local multicast addr handled separately*/
#endif

			SLSI_NET_DBG3(dev, SLSI_NETDEV, "Drop mac address = %pM\n", ha->addr);
			continue;
		}
		if (i == SLSI_MC_ADDR_ENTRY_MAX) {
			SLSI_NET_WARN(dev, "WARNING :mac list has reached max limit(%d), actual count= %d\n", SLSI_MC_ADDR_ENTRY_MAX, count);
			break;
		}

		SLSI_NET_DBG3(dev, SLSI_NETDEV, "mac address %d = %pM\n", i, ha->addr);
		SLSI_ETHER_COPY(ndev_vif->sta.regd_mc_addr[i++], ha->addr);
	}

exit:
	ndev_vif->sta.regd_mc_addr_count = i;
	slsi_wakelock(&ndev_vif->sdev->wlan_wl);
	if (!schedule_work(&ndev_vif->set_multicast_filter_work))
		slsi_wakeunlock(&ndev_vif->sdev->wlan_wl);
}

static int  slsi_set_mac_address(struct net_device *dev, void *addr)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_dev   *sdev = ndev_vif->sdev;
	struct sockaddr *sa = (struct sockaddr *)addr;

	SLSI_NET_DBG1(dev, SLSI_NETDEV, "slsi_set_mac_address %pM\n", sa->sa_data);
	SLSI_ETHER_COPY(dev->dev_addr, sa->sa_data);
	sdev->mac_changed = true;

	/* Interface is pulled down before mac address is changed.
	 * First scan initiated after interface is brought up again, should be treated as initial scan, for faster reconnection.
	 */
	if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
		sdev->initial_scan = true;
	}
	return 0;
}

static const struct net_device_ops slsi_netdev_ops = {
	.ndo_open         = slsi_net_open,
	.ndo_stop         = slsi_net_stop,
	.ndo_start_xmit   = slsi_net_hw_xmit,
	.ndo_do_ioctl     = slsi_net_ioctl,
	.ndo_get_stats    = slsi_net_get_stats,
	.ndo_select_queue = slsi_net_select_queue,
	.ndo_fix_features = slsi_net_fix_features,
	.ndo_set_rx_mode = slsi_set_multicast_list,
	.ndo_set_mac_address = slsi_set_mac_address,
};

static void slsi_if_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->netdev_ops = &slsi_netdev_ops;
	dev->destructor = free_netdev;
}

static void slsi_set_multicast_filter_work(struct work_struct *data)
{
	struct netdev_vif   *ndev_vif = container_of(data, struct netdev_vif, set_multicast_filter_work);
	struct slsi_dev *sdev = ndev_vif->sdev;

	if (!sdev) {
		SLSI_WARN_NODEV("sdev is NULL\n");
		return;
	}
	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	if (!ndev_vif->is_available ||
	    !ndev_vif->activated ||
	    ndev_vif->vif_type != FAPI_VIFTYPE_STATION)
		goto exit;

	if (ndev_vif->sta.vif_status != SLSI_VIF_STATUS_CONNECTED) {
		SLSI_INFO_NODEV("STA is not connected!\n");
		goto exit;
	}
	if (ndev_vif->ifnum > CONFIG_SCSC_WLAN_MAX_INTERFACES) {
		SLSI_INFO_NODEV("Improper ifidx: %d\n", ndev_vif->ifnum);
		goto exit;
	}
	SLSI_MUTEX_LOCK(sdev->device_config_mutex);
	SLSI_INFO_NODEV("user_suspend_mode = %d\n", ndev_vif->sdev->device_config.user_suspend_mode);
	/* if LCD is off, update the mcast filter */
	if (ndev_vif->sdev->device_config.user_suspend_mode == 1) {
		struct net_device *dev = NULL;
		int ret = 0;

		dev = sdev->netdev[ndev_vif->ifnum];
		if (!dev) {
			SLSI_ERR_NODEV("Dev is NULL ifnum:%d\n", ndev_vif->ifnum);
			SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
			goto exit;
		}
		ret = slsi_set_multicast_packet_filters(ndev_vif->sdev, dev);
		if (ret)
			SLSI_NET_ERR(dev, "Failed to update mcast filter\n");
	}
	SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
exit:
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	slsi_wakeunlock(&sdev->wlan_wl);
}

int slsi_netif_add_locked(struct slsi_dev *sdev, const char *name, int ifnum)
{
	struct net_device   *dev = NULL;
	struct netdev_vif   *ndev_vif;
	struct wireless_dev *wdev;
	int                 alloc_size, txq_count = 0, ret;

	WARN_ON(!SLSI_MUTEX_IS_LOCKED(sdev->netdev_add_remove_mutex));

	if (WARN_ON(!sdev || ifnum > CONFIG_SCSC_WLAN_MAX_INTERFACES || sdev->netdev[ifnum]))
		return -EINVAL;

	alloc_size = sizeof(struct netdev_vif);

	txq_count = SLSI_NETIF_Q_PEER_START + (SLSI_NETIF_Q_PER_PEER * (SLSI_ADHOC_PEER_CONNECTIONS_MAX));

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 16, 0))
	dev = alloc_netdev_mqs(alloc_size, name, NET_NAME_PREDICTABLE, slsi_if_setup, txq_count, 1);
#else
	dev = alloc_netdev_mqs(alloc_size, name, slsi_if_setup, txq_count, 1);
#endif
	if (!dev) {
		SLSI_ERR(sdev, "Failed to allocate private data for netdev\n");
		return -ENOMEM;
	}

	/* Reserve space in skb for later use */
	dev->needed_headroom = SLSI_NETIF_SKB_HEADROOM;
	dev->needed_tailroom = SLSI_NETIF_SKB_TAILROOM;

	ret = dev_alloc_name(dev, dev->name);
	if (ret < 0)
		goto exit_with_error;

	ndev_vif = netdev_priv(dev);
	memset(ndev_vif, 0x00, sizeof(*ndev_vif));
	SLSI_MUTEX_INIT(ndev_vif->vif_mutex);
	SLSI_MUTEX_INIT(ndev_vif->scan_mutex);
	SLSI_MUTEX_INIT(ndev_vif->scan_result_mutex);
	INIT_WORK(&ndev_vif->sched_scan_stop_wk, slsi_sched_scan_stopped);
	INIT_WORK(&ndev_vif->set_multicast_filter_work, slsi_set_multicast_filter_work);
	skb_queue_head_init(&ndev_vif->ba_complete);
	slsi_sig_send_init(&ndev_vif->sig_wait);
	ndev_vif->sdev = sdev;
	ndev_vif->ifnum = ifnum;
	ndev_vif->vif_type = SLSI_VIFTYPE_UNSPECIFIED;
#ifndef CONFIG_SCSC_WLAN_BLOCK_IPV6
	slsi_spinlock_create(&ndev_vif->ipv6addr_lock);
#endif
	slsi_spinlock_create(&ndev_vif->peer_lock);
	atomic_set(&ndev_vif->ba_flush, 0);

	/* Reserve memory for the peer database - Not required for p2p0/nan interface */
	if (!(SLSI_IS_VIF_INDEX_P2P(ndev_vif) || SLSI_IS_VIF_INDEX_NAN(ndev_vif))) {
		int queueset;

		for (queueset = 0; queueset < SLSI_ADHOC_PEER_CONNECTIONS_MAX; queueset++) {
			ndev_vif->peer_sta_record[queueset] = kzalloc(sizeof(*ndev_vif->peer_sta_record[queueset]), GFP_KERNEL);

			if (!ndev_vif->peer_sta_record[queueset]) {
				int j;

				SLSI_NET_ERR(dev, "Could not allocate memory for peer entry (queueset:%d)\n", queueset);

				/* Free previously allocated peer database memory till current queueset */
				for (j = 0; j < queueset; j++) {
					kfree(ndev_vif->peer_sta_record[j]);
					ndev_vif->peer_sta_record[j] = NULL;
				}

				ret = -ENOMEM;
				goto exit_with_error;
			}
		}
	}

	/* The default power mode in host*/
	if (slsi_is_rf_test_mode_enabled()) {
		SLSI_NET_ERR(dev, "*#rf# rf test mode set is enabled.\n");
		ndev_vif->set_power_mode = FAPI_POWERMANAGEMENTMODE_ACTIVE_MODE;
	} else {
		ndev_vif->set_power_mode = FAPI_POWERMANAGEMENTMODE_POWER_SAVE;
	}

	INIT_LIST_HEAD(&ndev_vif->sta.network_map);
#ifdef CONFIG_SCSC_WLAN_BSS_SELECTION
	INIT_LIST_HEAD(&ndev_vif->sta.ssid_info);
	INIT_LIST_HEAD(&ndev_vif->sta.blacklist_head);
#endif
	SLSI_DBG1(sdev, SLSI_NETDEV, "ifnum=%d\n", ndev_vif->ifnum);

	/* For HS2 interface */
	if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif))
		sdev->wlan_unsync_vif_state = WLAN_UNSYNC_NO_VIF;

	/* For p2p0 interface */
	else if (SLSI_IS_VIF_INDEX_P2P(ndev_vif)) {
		ret = slsi_p2p_init(sdev, ndev_vif);
		if (ret)
			goto exit_with_error;
	}

	INIT_DELAYED_WORK(&ndev_vif->scan_timeout_work, slsi_scan_ind_timeout_handle);

	ret = slsi_skb_work_init(sdev, dev, &ndev_vif->rx_data, "slsi_wlan_rx_data", slsi_rx_netdev_data_work);
	if (ret)
		goto exit_with_error;

	ret = slsi_skb_work_init(sdev, dev, &ndev_vif->rx_mlme, "slsi_wlan_rx_mlme", slsi_rx_netdev_mlme_work);
	if (ret) {
		slsi_skb_work_deinit(&ndev_vif->rx_data);
		goto exit_with_error;
	}

	wdev = &ndev_vif->wdev;

	dev->ieee80211_ptr = wdev;
	wdev->wiphy = sdev->wiphy;
	wdev->netdev = dev;
	wdev->iftype = NL80211_IFTYPE_STATION;
	SET_NETDEV_DEV(dev, sdev->dev);

	/* We are not ready to send data yet. */
	netif_carrier_off(dev);

#if defined(CONFIG_SCSC_WLAN_WIFI_SHARING) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
	if (strcmp(name, CONFIG_SCSC_AP_INTERFACE_NAME) == 0)
		SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[SLSI_NET_INDEX_P2P]);
	else
		SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[ifnum]);
#else
	SLSI_ETHER_COPY(dev->dev_addr, sdev->netdev_addresses[ifnum]);
#endif
	SLSI_DBG1(sdev, SLSI_NETDEV, "Add:%pM\n", dev->dev_addr);
	rcu_assign_pointer(sdev->netdev[ifnum], dev);

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
	SLSI_NET_DBG1(dev, SLSI_RX, "napi rx enabled\n");
	skb_queue_head_init(&ndev_vif->napi.rx_data);
	slsi_spinlock_create(&ndev_vif->napi.lock);
	ndev_vif->napi.interrupt_enabled = true;
	/* TODO_HARDMAC: What weight should we use? 32 is just a Guess */
	netif_napi_add(dev, &ndev_vif->napi.napi, slsi_net_rx_poll, 32);
	napi_enable(&ndev_vif->napi.napi);
#endif
	ndev_vif->delete_probe_req_ies = false;
	ndev_vif->probe_req_ies = NULL;
	ndev_vif->probe_req_ie_len = 0;
	sdev->require_vif_delete[ndev_vif->ifnum] = false;

	return 0;

exit_with_error:
	SLSI_MUTEX_LOCK(sdev->netdev_remove_mutex);
	free_netdev(dev);
	SLSI_MUTEX_UNLOCK(sdev->netdev_remove_mutex);
	return ret;
}

int slsi_netif_dynamic_iface_add(struct slsi_dev *sdev, const char *name)
{
	int index = -EINVAL;
	int err;

	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);

#if defined(CONFIG_SCSC_WLAN_MHS_STATIC_INTERFACE) || (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 9)
	if (sdev->netdev[SLSI_NET_INDEX_P2PX_SWLAN] == sdev->netdev_ap) {
		rcu_assign_pointer(sdev->netdev[SLSI_NET_INDEX_P2PX_SWLAN], NULL);
		err = slsi_netif_add_locked(sdev, name, SLSI_NET_INDEX_P2PX_SWLAN);
		index = err ? err : SLSI_NET_INDEX_P2PX_SWLAN;
	}
#else
	err = slsi_netif_add_locked(sdev, name, SLSI_NET_INDEX_P2PX_SWLAN);
	index = err ? err : SLSI_NET_INDEX_P2PX_SWLAN;
#endif

	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	return index;
}

void slsi_netif_remove_locked(struct slsi_dev *sdev, struct net_device *dev);

int slsi_netif_init(struct slsi_dev *sdev)
{
	int i;

	SLSI_DBG3(sdev, SLSI_NETDEV, "Init\n");

	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);

	/* Initialize all other netdev interfaces to NULL */
	for (i = 1; i <= CONFIG_SCSC_WLAN_MAX_INTERFACES; i++)
		RCU_INIT_POINTER(sdev->netdev[i], NULL);

	if (slsi_netif_add_locked(sdev, "wlan%d", SLSI_NET_INDEX_WLAN) != 0) {
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return -EINVAL;
	}

	if (slsi_netif_add_locked(sdev, "p2p%d", SLSI_NET_INDEX_P2P) != 0) {
		rtnl_lock();
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_WLAN]);
		rtnl_unlock();
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return -EINVAL;
	}
#if defined(CONFIG_SCSC_WLAN_WIFI_SHARING) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
#if defined(CONFIG_SCSC_WLAN_MHS_STATIC_INTERFACE) || (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 9) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
	if (slsi_netif_add_locked(sdev, CONFIG_SCSC_AP_INTERFACE_NAME, SLSI_NET_INDEX_P2PX_SWLAN) != 0) {
		rtnl_lock();
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_WLAN]);
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_P2P]);
		rtnl_unlock();
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return -EINVAL;
	}
#endif
#endif
#if CONFIG_SCSC_WLAN_MAX_INTERFACES >= 4
	if (slsi_netif_add_locked(sdev, "nan%d", SLSI_NET_INDEX_NAN) != 0) {
		rtnl_lock();
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_WLAN]);
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_P2P]);
#if defined(CONFIG_SCSC_WLAN_WIFI_SHARING) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
#if defined(CONFIG_SCSC_WLAN_MHS_STATIC_INTERFACE) || (defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 9) || defined(CONFIG_SCSC_WLAN_DUAL_STATION)
		slsi_netif_remove_locked(sdev, sdev->netdev[SLSI_NET_INDEX_P2PX_SWLAN]);
#endif
#endif
		rtnl_unlock();
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		return -EINVAL;
	}
#endif
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	return 0;
}

int slsi_netif_register_locked(struct slsi_dev *sdev, struct net_device *dev)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int               err;

	WARN_ON(!rtnl_is_locked());
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(sdev->netdev_add_remove_mutex));
	SLSI_NET_DBG1(dev, SLSI_NETDEV, "Register:%pM\n", dev->dev_addr);
	if (atomic_read(&ndev_vif->is_registered)) {
		SLSI_NET_ERR(dev, "Register:%pM Failed: Already registered\n", dev->dev_addr);
		return 0;
	}

	err = register_netdevice(dev);
	if (err)
		SLSI_NET_ERR(dev, "Register:%pM Failed\n", dev->dev_addr);
	else
		atomic_set(&ndev_vif->is_registered, 1);
	return err;
}

int slsi_netif_register_rtlnl_locked(struct slsi_dev *sdev, struct net_device *dev)
{
	int err;

	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	err = slsi_netif_register_locked(sdev, dev);
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	return err;
}

int slsi_netif_register(struct slsi_dev *sdev, struct net_device *dev)
{
	int err;

	rtnl_lock();
	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	err = slsi_netif_register_locked(sdev, dev);
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	rtnl_unlock();
	return err;
}

void slsi_netif_remove_locked(struct slsi_dev *sdev, struct net_device *dev)
{
	int               i;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
#ifdef CONFIG_SCSC_WLAN_BSS_SELECTION
	struct list_head    *pos, *q, *blacklist_pos, *blacklist_q;
#endif

	SLSI_NET_DBG1(dev, SLSI_NETDEV, "Unregister:%pM\n", dev->dev_addr);

	WARN_ON(!rtnl_is_locked());
	WARN_ON(!SLSI_MUTEX_IS_LOCKED(sdev->netdev_add_remove_mutex));

	if (atomic_read(&ndev_vif->is_registered)) {
		netif_tx_disable(dev);
		netif_carrier_off(dev);

		slsi_stop_net_dev(sdev, dev);
	}

	rcu_assign_pointer(sdev->netdev[ndev_vif->ifnum], NULL);
	synchronize_rcu();

	/* Free memory of the peer database - Not required for p2p0 interface */
	if (!SLSI_IS_VIF_INDEX_P2P(ndev_vif)) {
		int queueset;

		for (queueset = 0; queueset < SLSI_ADHOC_PEER_CONNECTIONS_MAX; queueset++) {
			kfree(ndev_vif->peer_sta_record[queueset]);
			ndev_vif->peer_sta_record[queueset] = NULL;
		}
	}

	if (SLSI_IS_VIF_INDEX_P2P(ndev_vif)) {
		slsi_p2p_deinit(sdev, ndev_vif);
	} else if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
		sdev->wlan_unsync_vif_state = WLAN_UNSYNC_NO_VIF;
		ndev_vif->vif_type = SLSI_VIFTYPE_UNSPECIFIED;
	}

	cancel_delayed_work(&ndev_vif->scan_timeout_work);
	ndev_vif->scan[SLSI_SCAN_HW_ID].requeue_timeout_work = false;

	slsi_skb_work_deinit(&ndev_vif->rx_data);
	slsi_skb_work_deinit(&ndev_vif->rx_mlme);
	for (i = 0; i < SLSI_SCAN_MAX; i++)
		slsi_purge_scan_results(ndev_vif, i);

	slsi_kfree_skb(ndev_vif->sta.mlme_scan_ind_skb);
	slsi_roam_channel_cache_prune(dev, 0);

#ifdef CONFIG_SCSC_WLAN_RX_NAPI
	slsi_skb_queue_purge(&ndev_vif->napi.rx_data);
#endif

#ifdef CONFIG_SCSC_WLAN_BSS_SELECTION
	if (SLSI_IS_VIF_INDEX_WLAN(ndev_vif)) {
		SLSI_NET_DBG1(dev, SLSI_NETDEV, "Cleaning up scan list!\n");
		list_for_each_safe(pos, q, &ndev_vif->sta.ssid_info) {
			struct slsi_ssid_info *ssid_info = list_entry(pos, struct slsi_ssid_info, list);
			struct list_head *bssid_pos, *p;

			list_for_each_safe(bssid_pos, p, &ssid_info->bssid_list) {
				struct slsi_bssid_info *bssid_info = list_entry(bssid_pos, struct slsi_bssid_info, list);

				list_del(bssid_pos);
				kfree(bssid_info);
			}
			list_del(pos);
			kfree(ssid_info);
		}
		list_for_each_safe(blacklist_pos, blacklist_q, &ndev_vif->sta.blacklist_head) {
			struct slsi_bssid_blacklist_info *blacklist_info = list_entry(blacklist_pos,
				struct slsi_bssid_blacklist_info, list);

			list_del(blacklist_pos);
			kfree(blacklist_info);
		}
	}
#endif
	kfree(ndev_vif->probe_req_ies);
	ndev_vif->probe_req_ies = NULL;
        ndev_vif->probe_req_ie_len = 0;

	if (atomic_read(&ndev_vif->is_registered)) {
		atomic_set(&ndev_vif->is_registered, 0);
		unregister_netdevice(dev);
	} else {
		SLSI_MUTEX_LOCK(sdev->netdev_remove_mutex);
		free_netdev(dev);
		SLSI_MUTEX_UNLOCK(sdev->netdev_remove_mutex);
	}
}

void slsi_netif_remove_rtlnl_locked(struct slsi_dev *sdev, struct net_device *dev)
{
	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	slsi_netif_remove_locked(sdev, dev);
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
}

void slsi_netif_remove(struct slsi_dev *sdev, struct net_device *dev)
{
	rtnl_lock();
	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	slsi_netif_remove_locked(sdev, dev);
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	rtnl_unlock();
}

void slsi_netif_remove_all(struct slsi_dev *sdev)
{
	int i;

	SLSI_DBG1(sdev, SLSI_NETDEV, "\n");
	rtnl_lock();
	SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
	for (i = 1; i <= CONFIG_SCSC_WLAN_MAX_INTERFACES; i++)
		if (sdev->netdev[i])
			slsi_netif_remove_locked(sdev, sdev->netdev[i]);
	rcu_assign_pointer(sdev->netdev_ap, NULL);
	SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
	rtnl_unlock();
}

void slsi_netif_deinit(struct slsi_dev *sdev)
{
	SLSI_DBG1(sdev, SLSI_NETDEV, "\n");
	slsi_netif_remove_all(sdev);
}

int slsi_netif_pending_queues(int vif_type, struct net_device *dev)
{
	int len = 0, tid = 0, i = 0;

	/*Get the network level queue length */
	if (vif_type == FAPI_VIFTYPE_STATION)
		for (i = SLSI_NETIF_Q_PEER_START; i < (SLSI_NETIF_Q_PEER_START + SLSI_NETIF_Q_PER_PEER); i++)
			len += skb_queue_len(&dev->_tx[i].qdisc->q);

	if (vif_type == FAPI_VIFTYPE_AP)
		for (i = 0; i < SLSI_AP_PEER_CONNECTIONS_MAX; i++)
			for (tid = SLSI_NETIF_Q_PEER_START; tid < (SLSI_NETIF_Q_PEER_START + SLSI_NETIF_Q_PER_PEER); tid++)
				len += skb_queue_len(&dev->_tx[tid].qdisc->q);

	return len;
}

static int slsi_netif_tcp_ack_suppression_start(struct net_device *dev)
{
	int index;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_tcp_ack_s *tcp_ack;

	ndev_vif->last_tcp_ack = NULL;
	for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
		tcp_ack = &ndev_vif->ack_suppression[index];
		tcp_ack->dport = 0;
		tcp_ack->daddr = 0;
		tcp_ack->sport = 0;
		tcp_ack->saddr = 0;
		tcp_ack->ack_seq = 0;
		tcp_ack->count = 0;
		tcp_ack->max = 0;
		tcp_ack->age = 0;
		skb_queue_head_init(&tcp_ack->list);
		tcp_ack->timer.function = slsi_netif_tcp_ack_suppression_timeout;
		tcp_ack->timer.data = (unsigned long)tcp_ack;
		init_timer(&tcp_ack->timer);
		tcp_ack->ndev_vif = ndev_vif;
		tcp_ack->state = 1;
	}
	slsi_spinlock_create(&ndev_vif->tcp_ack_lock);
	memset(&ndev_vif->tcp_ack_stats, 0, sizeof(struct slsi_tcp_ack_stats));
	return 0;
}

static int slsi_netif_tcp_ack_suppression_stop(struct net_device *dev)
{
	int index;
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_tcp_ack_s *tcp_ack;

	SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
	slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);
	for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
		tcp_ack = &ndev_vif->ack_suppression[index];
		slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
		del_timer_sync(&tcp_ack->timer);
		slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);
		tcp_ack->state = 0;
		skb_queue_purge(&tcp_ack->list);
	}
	ndev_vif->last_tcp_ack = NULL;
	slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
	SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
	return 0;
}

static void slsi_netif_tcp_ack_suppression_timeout(unsigned long data)
{
	struct slsi_tcp_ack_s *tcp_ack = (struct slsi_tcp_ack_s *)data;
	struct sk_buff *skb;
	struct netdev_vif *ndev_vif;
	struct slsi_dev   *sdev;
	int r;

	if (!tcp_ack)
		return;

	ndev_vif = tcp_ack->ndev_vif;

	if (!ndev_vif)
		return;

	sdev = ndev_vif->sdev;

	slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);

	if (!tcp_ack->state) {
		slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
		return;
	}

	while ((skb = skb_dequeue(&tcp_ack->list)) != 0) {
		tcp_ack->count = 0;

		if (!skb->dev) {
			kfree_skb(skb);
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return;
		}

		if (tcp_ack_robustness)
			slsi_skb_cb_get(skb)->frame_format = SLSI_NETIF_FRAME_TCP_ACK;

		ndev_vif->tcp_ack_stats.tack_timeout++;
		r = slsi_tx_data(sdev, skb->dev, skb);
		if (r == 0) {
			ndev_vif->tcp_ack_stats.tack_sent++;
			tcp_ack->last_sent = ktime_get();
		} else if (r == -ENOSPC) {
			ndev_vif->tcp_ack_stats.tack_dropped++;
			slsi_kfree_skb(skb);
		} else {
			ndev_vif->tcp_ack_stats.tack_dropped++;
		}
	}
	slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
}

static int slsi_netif_tcp_ack_suppression_option(struct sk_buff *skb, u32 option)
{
	unsigned char *options;
	u32 optlen = 0, len = 0;

	if (tcp_hdr(skb)->doff > 5)
		optlen = (tcp_hdr(skb)->doff - 5) * 4;

	options = ((u8 *)tcp_hdr(skb)) + TCP_ACK_SUPPRESSION_OPTIONS_OFFSET;

	while (optlen > 0) {
		switch (options[0]) {
		case TCP_ACK_SUPPRESSION_OPTION_EOL:
			return 0;
		case TCP_ACK_SUPPRESSION_OPTION_NOP:
			len = 1;
			break;
		case TCP_ACK_SUPPRESSION_OPTION_MSS:
			if (option == TCP_ACK_SUPPRESSION_OPTION_MSS)
				return ((options[2] << 8) | options[3]);
			len = options[1];
			break;
		case TCP_ACK_SUPPRESSION_OPTION_WINDOW:
			if (option == TCP_ACK_SUPPRESSION_OPTION_WINDOW)
				return options[2];
			len = 1;
			break;
		case TCP_ACK_SUPPRESSION_OPTION_SACK:
			if (option == TCP_ACK_SUPPRESSION_OPTION_SACK)
				return 1;
			len = options[1];
			break;
		default:
			len = options[1];
			break;
		}
		/* if length field in TCP options is 0, or greater than
		 * total options length, then options are incorrect; return here
		 */
		if ((len == 0) || (len > optlen)) {
			SLSI_DBG_HEX_NODEV(SLSI_TX, skb->data, skb->len < 128 ? skb->len : 128, "SKB:\n");
			return 0;
		}
		optlen -= len;
		options += len;
	}
	return 0;
}

static void slsi_netif_tcp_ack_suppression_syn(struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_tcp_ack_s *tcp_ack;
	int index;

	SLSI_NET_DBG2(dev, SLSI_TX, "\n");
	slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);
	for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
		tcp_ack = &ndev_vif->ack_suppression[index];

		if (!tcp_ack->state) {
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return;
		}
		/* Recover old/hung/unused record. */
		if (tcp_ack->daddr) {
			if (ktime_to_ms(ktime_sub(ktime_get(), tcp_ack->last_sent)) >= TCP_ACK_SUPPRESSION_RECORD_UNUSED_TIMEOUT * 1000) {
				SLSI_NET_DBG2(dev, SLSI_TX, "delete at %d (%pI4.%d > %pI4.%d)\n", index, &tcp_ack->saddr, ntohs(tcp_ack->sport), &tcp_ack->daddr, ntohs(tcp_ack->dport));
				skb_queue_purge(&tcp_ack->list);
				tcp_ack->dport = 0;
				tcp_ack->sport = 0;
				tcp_ack->daddr = 0;
				tcp_ack->saddr = 0;
				tcp_ack->count = 0;
				tcp_ack->ack_seq = 0;
				del_timer(&tcp_ack->timer);
			}
		}

		if (tcp_ack->daddr == 0) {
			SLSI_NET_DBG2(dev, SLSI_TX, "add at %d (%pI4.%d > %pI4.%d)\n", index, &ip_hdr(skb)->saddr, ntohs(tcp_hdr(skb)->source), &ip_hdr(skb)->daddr, ntohs(tcp_hdr(skb)->dest));
			tcp_ack->daddr = ip_hdr(skb)->daddr;
			tcp_ack->saddr = ip_hdr(skb)->saddr;
			tcp_ack->dport = tcp_hdr(skb)->dest;
			tcp_ack->sport = tcp_hdr(skb)->source;
			tcp_ack->count = 0;
			tcp_ack->ack_seq = 0;
			tcp_ack->slow_start_count = 0;
			tcp_ack->tcp_slow_start = true;
			if (tcp_ack_suppression_monitor) {
				tcp_ack->max = 0;
				tcp_ack->age = 0;
			} else {
				tcp_ack->max = tcp_ack_suppression_max;
				tcp_ack->age = tcp_ack_suppression_timeout;
			}
			tcp_ack->last_sent = ktime_get();

			if (tcp_ack_suppression_monitor) {
				tcp_ack->last_sample_time = ktime_get();
				tcp_ack->last_ack_seq = 0;
				tcp_ack->last_tcp_rate = 0;
				tcp_ack->num_bytes = 0;
				tcp_ack->hysteresis = 0;
			}

			/* read and validate the window scaling multiplier */
			tcp_ack->window_multiplier = slsi_netif_tcp_ack_suppression_option(skb, TCP_ACK_SUPPRESSION_OPTION_WINDOW);
			if (tcp_ack->window_multiplier > 14)
				tcp_ack->window_multiplier = 0;
			tcp_ack->mss = slsi_netif_tcp_ack_suppression_option(skb, TCP_ACK_SUPPRESSION_OPTION_MSS);
			SLSI_NET_DBG2(dev, SLSI_TX, "options: mss:%u, window:%u\n", tcp_ack->mss, tcp_ack->window_multiplier);
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return;
		}
	}
	slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
}

static void slsi_netif_tcp_ack_suppression_fin(struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	struct slsi_tcp_ack_s *tcp_ack;
	int index;

	SLSI_NET_DBG2(dev, SLSI_TX, "\n");
	slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);
	for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
		tcp_ack = &ndev_vif->ack_suppression[index];

		if ((tcp_ack->dport == tcp_hdr(skb)->dest) &&
		    (tcp_ack->daddr == ip_hdr(skb)->daddr)) {
			SLSI_NET_DBG2(dev, SLSI_TX, "delete at %d (%pI4.%d > %pI4.%d)\n", index, &tcp_ack->saddr, ntohs(tcp_ack->sport), &tcp_ack->daddr, ntohs(tcp_ack->dport));
			skb_queue_purge(&tcp_ack->list);
			tcp_ack->dport = 0;
			tcp_ack->sport = 0;
			tcp_ack->daddr = 0;
			tcp_ack->saddr = 0;
			tcp_ack->count = 0;
			tcp_ack->ack_seq = 0;

			if (tcp_ack_suppression_monitor) {
				tcp_ack->last_ack_seq = 0;
				tcp_ack->last_tcp_rate = 0;
				tcp_ack->num_bytes = 0;
				tcp_ack->hysteresis = 0;
			}

			del_timer(&tcp_ack->timer);
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return;
		}
	}
	slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
}

static struct sk_buff *slsi_netif_tcp_ack_suppression_pkt(struct net_device *dev, struct sk_buff *skb)
{
	struct netdev_vif *ndev_vif = netdev_priv(dev);
	int index, found;
	struct slsi_tcp_ack_s *tcp_ack;
	int forward_now = 0, flush = 0;
	struct sk_buff *cskb = 0;
	u32 tcp_recv_window_size = 0;

	if (tcp_ack_suppression_disable)
		return skb;

	if (tcp_ack_suppression_disable_2g && !SLSI_IS_VIF_CHANNEL_5G(ndev_vif))
		return skb;

	/* for AP type (AP or P2P Go) check if the packet is local or intra BSS. If intra BSS then
	 * the IP header and TCP header are not set; so return the SKB
	 */
	if ((ndev_vif->vif_type == FAPI_VIFTYPE_AP) && (compare_ether_addr(eth_hdr(skb)->h_source, dev->dev_addr) != 0))
		return skb;

	/* Return SKB that doesn't match. */
	if (be16_to_cpu(eth_hdr(skb)->h_proto) != ETH_P_IP)
		return skb;
	if (ip_hdr(skb)->protocol != IPPROTO_TCP)
		return skb;
	if (!skb_transport_header_was_set(skb))
		return skb;
	if (tcp_hdr(skb)->syn) {
		slsi_netif_tcp_ack_suppression_syn(dev, skb);
		return skb;
	}
	if (tcp_hdr(skb)->fin) {
		slsi_netif_tcp_ack_suppression_fin(dev, skb);
		return skb;
	}
	if (!tcp_hdr(skb)->ack)
		return skb;
	if (tcp_hdr(skb)->rst)
		return skb;
	if (tcp_hdr(skb)->urg)
		return skb;

	slsi_spinlock_lock(&ndev_vif->tcp_ack_lock);
	ndev_vif->tcp_ack_stats.tack_acks++;

	/* If we find a record, leave the spinlock taken until the end of the function. */
	found = 0;
	if (ndev_vif->last_tcp_ack) {
		tcp_ack = ndev_vif->last_tcp_ack;
		if (!tcp_ack->state) {
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			ndev_vif->tcp_ack_stats.tack_sent++;
			SLSI_ERR_NODEV("last_tcp_ack record not enabled\n");
			return skb;
		}
		if ((tcp_ack->dport == tcp_hdr(skb)->dest) &&
		    (tcp_ack->daddr == ip_hdr(skb)->daddr)) {
			found = 1;
			ndev_vif->tcp_ack_stats.tack_lastrecord++;
		}
	}
	if (found == 0) {
		/* Search for an existing record on this connection. */
		for (index = 0; index < TCP_ACK_SUPPRESSION_RECORDS_MAX; index++) {
			tcp_ack = &ndev_vif->ack_suppression[index];

			if (!tcp_ack->state) {
				slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
				ndev_vif->tcp_ack_stats.tack_sent++;
				SLSI_ERR_NODEV("tcp_ack record %d not enabled\n", index);
				return skb;
			}
			if ((tcp_ack->dport == tcp_hdr(skb)->dest) &&
			    (tcp_ack->daddr == ip_hdr(skb)->daddr)) {
				found = 1;
				ndev_vif->tcp_ack_stats.tack_searchrecord++;
				break;
			}
		}
		if (found == 0) {
			/* No record found, so We cannot suppress the ack, return. */
			ndev_vif->tcp_ack_stats.tack_norecord++;
			ndev_vif->tcp_ack_stats.tack_sent++;
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return skb;
		}
		ndev_vif->last_tcp_ack = tcp_ack;
	}

	/* If it is a DUP Ack, send straight away without flushing the cache. */
	if (be32_to_cpu(tcp_hdr(skb)->ack_seq) < tcp_ack->ack_seq) {
		/* check for wrap-around */
		if (((s32)((u32)be32_to_cpu(tcp_hdr(skb)->ack_seq) - (u32)tcp_ack->ack_seq)) < 0) {
			ndev_vif->tcp_ack_stats.tack_dacks++;
			ndev_vif->tcp_ack_stats.tack_sent++;
			slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
			return skb;
		}
	}

	/* Has data, forward straight away. */
	if (be16_to_cpu(ip_hdr(skb)->tot_len) > ((ip_hdr(skb)->ihl * 4) + (tcp_hdr(skb)->doff * 4))) {
		ndev_vif->tcp_ack_stats.tack_hasdata++;
		forward_now = 1;
		goto _forward_now;
	}

	/* PSH flag set, forward straight away. */
	if (tcp_hdr(skb)->psh) {
		ndev_vif->tcp_ack_stats.tack_psh++;
		forward_now = 1;
		goto _forward_now;
	}

	/* The ECE flag is set for Explicit Congestion Notification supporting connections when the ECT flag
	 * is set in the segment packet. We must forward ECE marked acks immediately for ECN to work.
	 */
	if (tcp_hdr(skb)->ece) {
		ndev_vif->tcp_ack_stats.tack_ece++;
		forward_now = 1;
		goto _forward_now;
	}

	if (tcp_ack_suppression_monitor) {
		/* Measure the throughput of TCP stream by monitoring the bytes Acked by each Ack over a
		 * sampling period. Based on throughput apply different degree of Ack suppression
		 */
		if (tcp_ack->last_ack_seq)
			tcp_ack->num_bytes += ((u32)be32_to_cpu(tcp_hdr(skb)->ack_seq) - tcp_ack->last_ack_seq);

		tcp_ack->last_ack_seq = be32_to_cpu(tcp_hdr(skb)->ack_seq);
		if (ktime_to_ms(ktime_sub(ktime_get(), tcp_ack->last_sample_time)) > tcp_ack_suppression_monitor_interval) {
			u16 acks_max;
			u32 tcp_rate = ((tcp_ack->num_bytes * 8) / (tcp_ack_suppression_monitor_interval * 1000));

			SLSI_NET_DBG2(dev, SLSI_TX, "hysteresis:%u total_bytes:%llu rate:%u Mbps\n",
				      tcp_ack->hysteresis, tcp_ack->num_bytes, tcp_rate);

			/* hysterisis - change only if the variation from last value is more than threshold */
			if ((abs(tcp_rate - tcp_ack->last_tcp_rate)) > tcp_ack->hysteresis) {
				if (tcp_rate >= tcp_ack_suppression_rate_very_high) {
					tcp_ack->max = tcp_ack_suppression_rate_very_high_acks;
					tcp_ack->age = tcp_ack_suppression_rate_very_high_timeout;
				} else if (tcp_rate >= tcp_ack_suppression_rate_high) {
					tcp_ack->max = tcp_ack_suppression_rate_high_acks;
					tcp_ack->age = tcp_ack_suppression_rate_high_timeout;
				} else if (tcp_rate >= tcp_ack_suppression_rate_low) {
					tcp_ack->max = tcp_ack_suppression_rate_low_acks;
					tcp_ack->age = tcp_ack_suppression_rate_low_timeout;
				} else {
					tcp_ack->max = 0;
					tcp_ack->age = 0;
				}

				/* Should not be suppressing Acks more than 20% of receiver window size
				 * doing so can lead to increased RTT and low transmission rate at the
				 * TCP sender
				 */
				if (tcp_ack->window_multiplier)
					tcp_recv_window_size = be16_to_cpu(tcp_hdr(skb)->window) * (2 << tcp_ack->window_multiplier);
				else
					tcp_recv_window_size = be16_to_cpu(tcp_hdr(skb)->window);

				acks_max = (tcp_recv_window_size / 5) / (2 * tcp_ack->mss);
				if (tcp_ack->max > acks_max)
					tcp_ack->max = acks_max;
			}
			tcp_ack->hysteresis = tcp_rate / 5; /* 20% hysteresis */
			tcp_ack->last_tcp_rate = tcp_rate;
			tcp_ack->num_bytes = 0;
			tcp_ack->last_sample_time = ktime_get();
		}
	}

	/* Do not suppress Selective Acks. */
	if (slsi_netif_tcp_ack_suppression_option(skb, TCP_ACK_SUPPRESSION_OPTION_SACK)) {
		ndev_vif->tcp_ack_stats.tack_sacks++;

		/* A TCP selective Ack suggests TCP segment loss. The TCP sender
		 * may reduce congestion window and limit the number of segments
		 * it sends before waiting for Ack.
		 * It is ideal to switch off TCP ack suppression for certain time
		 * (being replicated here by tcp_ack_suppression_slow_start_acks
		 * count) and send as many Acks as possible to allow the cwnd to
		 * grow at the TCP sender
		 */
		tcp_ack->slow_start_count = 0;
		tcp_ack->tcp_slow_start = true;
		forward_now = 1;
		goto _forward_now;
	}

	if (be32_to_cpu(tcp_hdr(skb)->ack_seq) == tcp_ack->ack_seq) {
		ndev_vif->tcp_ack_stats.tack_dacks++;
		forward_now = 1;
		goto _forward_now;
	}

	/* When the TCP connection is made, wait until a number of Acks
	 * are sent before applying the suppression rules. It is to
	 * allow the cwnd to grow at a normal rate at the TCP sender
	 */
	if (tcp_ack->tcp_slow_start) {
		tcp_ack->slow_start_count++;
		if (tcp_ack->slow_start_count >= tcp_ack_suppression_slow_start_acks) {
			tcp_ack->slow_start_count = 0;
			tcp_ack->tcp_slow_start = false;
		}
		forward_now = 1;
		goto _forward_now;
	}

	/* do not suppress if so decided by the TCP monitor */
	if (tcp_ack_suppression_monitor && (!tcp_ack->max || !tcp_ack->age)) {
		forward_now = 1;
		goto _forward_now;
	}

	/* do not suppress delayed Acks that acknowledges for more than 2 TCP
	 * maximum size segments
	 */
	if (((u32)be32_to_cpu(tcp_hdr(skb)->ack_seq)) - (tcp_ack->ack_seq) > (2 * tcp_ack->mss)) {
		ndev_vif->tcp_ack_stats.tack_delay_acks++;
		forward_now = 1;
		goto _forward_now;
	}

	/* Do not suppress unless the receive window is large
	 * enough.
	 * With low receive window size the cwnd can't grow much.
	 * So suppressing Acks has a negative impact on sender
	 * rate as it increases the Round trip time measured at
	 * sender
	 */
	if (!tcp_ack_suppression_monitor) {
		if (tcp_ack->window_multiplier)
			tcp_recv_window_size = be16_to_cpu(tcp_hdr(skb)->window) * (2 << tcp_ack->window_multiplier);
		else
			tcp_recv_window_size = be16_to_cpu(tcp_hdr(skb)->window);
		if (tcp_recv_window_size < tcp_ack_suppression_rcv_window * 1024) {
			ndev_vif->tcp_ack_stats.tack_low_window++;
			forward_now = 1;
			goto _forward_now;
		}
	}

	if (!tcp_ack_suppression_monitor && ktime_to_ms(ktime_sub(ktime_get(), tcp_ack->last_sent)) >= tcp_ack->age) {
		ndev_vif->tcp_ack_stats.tack_ktime++;
		forward_now = 1;
		if (tcp_ack_robustness)
			slsi_skb_cb_get(skb)->frame_format = SLSI_NETIF_FRAME_TCP_ACK;
		goto _forward_now;
	}

	/* Test for a new cache */
	if (!skb_queue_len(&tcp_ack->list)) {
		skb_queue_tail(&tcp_ack->list, skb);
		tcp_ack->count = 1;
		tcp_ack->ack_seq = be32_to_cpu(tcp_hdr(skb)->ack_seq);
		if (tcp_ack->age)
			mod_timer(&tcp_ack->timer, jiffies + msecs_to_jiffies(tcp_ack->age));
		slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
		return 0;
	}
_forward_now:
	cskb = skb_dequeue(&tcp_ack->list);
	if (cskb) {
		if (tcp_ack_suppression_monitor && tcp_ack->age)
			mod_timer(&tcp_ack->timer, jiffies + msecs_to_jiffies(tcp_ack->age));
		ndev_vif->tcp_ack_stats.tack_suppressed++;
		slsi_kfree_skb(cskb);
	}
	skb_queue_tail(&tcp_ack->list, skb);
	tcp_ack->ack_seq = be32_to_cpu(tcp_hdr(skb)->ack_seq);
	tcp_ack->count++;
	if (forward_now) {
		flush = 1;
	} else {
		if (tcp_ack->count >= tcp_ack->max) {
			flush = 1;
			ndev_vif->tcp_ack_stats.tack_max++;
			if (tcp_ack_robustness)
				slsi_skb_cb_get(skb)->frame_format = SLSI_NETIF_FRAME_TCP_ACK;
		}
	}
	if (!flush) {
		slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
		return 0;
	}
	/* Flush the cache. */
	cskb = skb_dequeue(&tcp_ack->list);
	tcp_ack->count = 0;

	if (tcp_ack->age)
		del_timer(&tcp_ack->timer);

	tcp_ack->last_sent = ktime_get();
	ndev_vif->tcp_ack_stats.tack_sent++;
	slsi_spinlock_unlock(&ndev_vif->tcp_ack_lock);
	return cskb;
}
