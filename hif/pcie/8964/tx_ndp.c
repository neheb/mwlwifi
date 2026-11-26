/*
 * Copyright (C) 2006-2018, Marvell International Ltd.
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

/* Description:  This file implements transmit related functions for new data
 * path.
 */

#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "sysadpt.h"
#include "core.h"
#include "compat.h"
#include "utils.h"
#include "hif/fwcmd.h"
#include "hif/pcie/dev.h"
#include "hif/pcie/8964/tx_ndp.h"

#define MAX_NUM_TX_RING_BYTES   (MAX_NUM_TX_DESC * \
				sizeof(struct pcie_tx_desc_ndp))
#define MAX_NUM_TX_RING_DONE_BYTES (MAX_NUM_TX_DESC * \
				sizeof(struct tx_ring_done))
#define QUEUE_STAOFFSET         ((SYSADPT_NUM_OF_AP - 1) + \
				SYSADPT_NUM_OF_CLIENT)
#define PROBE_RESPONSE_TXQNUM   ((SYSADPT_MAX_STA_SC4 + SYSADPT_NUM_OF_AP + \
				SYSADPT_NUM_OF_CLIENT) * SYSADPT_MAX_TID)
#define MGMT_TXQNUM             ((PROBE_RESPONSE_TXQNUM + 1))
#define TXDONE_THRESHOLD        4

#define TX_CTRL_TYPE_DATA       BIT(0)
#define TX_CTRL_EAPOL           BIT(1)
#define TX_CTRL_TCP_ACK         BIT(2)
#define TX_CTRL_STATUS_SENT     BIT(4)

/* Transmission information to transmit a socket buffer.
 */
struct pcie_tx_ctrl_ndp {
	u16 tx_que_priority;
	u8 hdrlen;
	u8 flags;
	u32 rate;
	u32 tcp_dst_src;
	u32 tcp_sn;
	u16 qos;
	u8 mic_room;
} __packed;

static int pcie_tx_ring_alloc_ndp(struct mwl_priv *priv)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	void *mem;

	mem = dmam_alloc_coherent(priv->dev,
				  MAX_NUM_TX_RING_BYTES,
				  &desc->pphys_tx_ring,
				  GFP_KERNEL);
	if (!mem)
		goto err_no_mem;
	desc->ptx_ring = mem;

	mem = dmam_alloc_coherent(priv->dev,
				  MAX_NUM_TX_RING_DONE_BYTES,
				  &desc->pphys_tx_ring_done,
				  GFP_KERNEL);
	if (!mem)
		goto err_no_mem;
	desc->ptx_ring_done = mem;

	mem = dmam_alloc_coherent(priv->dev,
				  DEFAULT_ACNT_RING_SIZE,
				  &desc->pphys_acnt_ring,
				  GFP_KERNEL);
	if (!mem)
		goto err_no_mem;
	desc->pacnt_ring = mem;

	desc->pacnt_buf = devm_kzalloc(priv->dev, DEFAULT_ACNT_RING_SIZE,
				       GFP_KERNEL);
	if (!desc->pacnt_buf)
		goto err_no_mem;
	desc->acnt_ring_size = DEFAULT_ACNT_RING_SIZE;

	return 0;

err_no_mem:

	wiphy_err(priv->hw->wiphy, "cannot alloc mem\n");

	return -ENOMEM;
}

static int pcie_tx_ring_init_ndp(struct mwl_priv *priv)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	int i;

	for (i = 0; i < PCIE_NUM_OF_DESC_DATA; i++)
		skb_queue_head_init(&pcie_priv->txq[i]);

	if (desc->ptx_ring) {
		for (i = 0; i < MAX_NUM_TX_DESC; i++)
			desc->ptx_ring[i].user = cpu_to_le32(i);
		desc->tx_desc_busy_cnt = 0;
	}

	return 0;
}

/* Unmap a TX buffer that the firmware has returned, or that it will never
 * return, and hand the skb back to mac80211.
 */
static void pcie_tx_return_skb_ndp(struct ieee80211_hw *hw, u32 index,
				   bool dropped)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	struct sk_buff *skb = desc->tx_vbuflist[index];
	struct ieee80211_tx_info *tx_info;
	struct pcie_tx_ctrl_ndp *tx_ctrl;
	struct pcie_dma_data *dma_data;
	u16 hdrlen;

	dma_unmap_single(&(pcie_priv->pdev)->dev,
			 desc->pphys_tx_buflist[index],
			 desc->tx_buflen[index],
			 DMA_TO_DEVICE);
	desc->pphys_tx_buflist[index] = 0;
	desc->tx_vbuflist[index] = NULL;

	/* The status of this frame (or of a copy of it) has already been
	 * reported; what is left is only the driver's reference.
	 */
	if (__test_and_clear_bit(index, desc->tx_status_sent)) {
		dev_kfree_skb_any(skb);
		return;
	}

	tx_info = IEEE80211_SKB_CB(skb);
	tx_ctrl = (struct pcie_tx_ctrl_ndp *)tx_info->driver_data;

	if (tx_ctrl->flags & TX_CTRL_TYPE_DATA) {
		skb_push(skb, tx_ctrl->hdrlen);
	} else {
		skb_trim(skb, skb->len - tx_ctrl->mic_room);

		/* Remove H/W dma header */
		dma_data = (struct pcie_dma_data *)skb->data;
		hdrlen = ieee80211_hdrlen(dma_data->wh.frame_control);
		if (ieee80211_is_qos_nullfunc(dma_data->wh.frame_control) ||
		   ieee80211_is_data_qos(dma_data->wh.frame_control)) {
			memmove(dma_data->data - hdrlen, &dma_data->wh, hdrlen - IEEE80211_QOS_CTL_LEN);
			*((__le16 *)(dma_data->data - IEEE80211_QOS_CTL_LEN)) =
				cpu_to_le16(tx_ctrl->qos);
		} else
			memmove(dma_data->data - hdrlen, &dma_data->wh, hdrlen);
		skb_pull(skb, sizeof(*dma_data) - hdrlen);
	}

	if (dropped) {
		ieee80211_free_txskb(hw, skb);
		return;
	}

	pcie_tx_prepare_info(priv, 0, tx_info);
	ieee80211_tx_status_skb(hw, skb);
}

static void pcie_tx_ring_cleanup_ndp(struct mwl_priv *priv)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	int i;

	for (i = 0; i < PCIE_NUM_OF_DESC_DATA; i++)
		ieee80211_purge_tx_queue(priv->hw, &pcie_priv->txq[i]);

	/* Frames the firmware still holds would otherwise keep their
	 * mac80211 ACK status entries alive.
	 */
	for (i = 0; i < MAX_TX_RING_SEND_SIZE; i++)
		if (desc->tx_vbuflist[i])
			pcie_tx_return_skb_ndp(priv->hw, i, true);
	desc->tx_sent_tail = 0;
	desc->tx_sent_head = 0;
	desc->tx_done_tail = 0;
	desc->tx_vbuflist_idx = 0;
	desc->tx_desc_busy_cnt = 0;
}

static void pcie_tx_ring_free_ndp(struct mwl_priv *priv)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;

	/* The memory is freed when the device detaches. */
	desc->ptx_ring = NULL;
	desc->ptx_ring_done = NULL;
	desc->pacnt_ring = NULL;
	desc->pacnt_buf = NULL;
}

static inline u32 pcie_tx_set_skb(struct mwl_priv *priv, struct sk_buff *skb,
				  dma_addr_t dma)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	u32 index = desc->tx_vbuflist_idx;
	u32 i;

	for (i = 0; desc->tx_vbuflist[index]; i++) {
		/* Slots of bad TX done entries are never freed. */
		if (i == MAX_TX_RING_SEND_SIZE)
			return MAX_TX_RING_SEND_SIZE;
		index = (index + 1) % MAX_TX_RING_SEND_SIZE;
	}

	desc->tx_vbuflist_idx = (index + 1) % MAX_TX_RING_SEND_SIZE;
	desc->pphys_tx_buflist[index] = dma;
	desc->tx_vbuflist[index] = skb;

	return index;
}

static inline int pcie_tx_skb_ndp(struct mwl_priv *priv,
				  struct sk_buff *tx_skb)
{
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	u32 tx_send_tail;
	u32 tx_send_head_new;
	struct ieee80211_tx_info *tx_info;
	struct pcie_tx_ctrl_ndp *tx_ctrl;
	struct pcie_tx_desc_ndp *pnext_tx_desc;
	struct ieee80211_hdr *wh;
	u32 ctrl = 0;
	dma_addr_t dma;
	u32 index;

	guard(spinlock_bh)(&pcie_priv->tx_desc_lock);

	tx_send_tail = desc->tx_sent_tail;
	tx_send_head_new = desc->tx_sent_head;

	if (((tx_send_head_new + 1) & (MAX_NUM_TX_DESC-1)) == tx_send_tail) {
		/* Mask it like the other ring indices read from the
		 * device.
		 */
		tx_send_tail = readl(pcie_priv->iobase1 +
				     MACREG_REG_TXSEDNTAIL) &
			       (MAX_NUM_TX_DESC - 1);
		desc->tx_sent_tail = tx_send_tail;

		if (((tx_send_head_new + 1) & (MAX_NUM_TX_DESC-1)) ==
		    tx_send_tail) {
			return -EAGAIN;
		}
	}

	tx_info = IEEE80211_SKB_CB(tx_skb);
	tx_ctrl = (struct pcie_tx_ctrl_ndp *)tx_info->driver_data;

	/* The firmware reports completions 64 at a time, and not at all
	 * while fewer than 64 are outstanding, so a sending socket would
	 * wait on frames that are long gone and a UDP sender with the
	 * default buffer stalls. Release the socket's memory now, unless
	 * the TX status is wanted (mac80211 also sets REQ_TX_STATUS for a
	 * socket that asked for wifi status). TCP is left alone: TSQ
	 * allows far more than the firmware holds back, and without it a
	 * local sender fills the driver's queue and adds latency.
	 */
	if (tx_skb->sk && !sk_is_tcp(tx_skb->sk) &&
	    !(tx_info->flags & IEEE80211_TX_CTL_REQ_TX_STATUS))
		skb_orphan(tx_skb);

	pnext_tx_desc = &desc->ptx_ring[tx_send_head_new];
	memset(pnext_tx_desc, 0, sizeof(*pnext_tx_desc));

	if (tx_ctrl->flags & TX_CTRL_TYPE_DATA) {
		wh = (struct ieee80211_hdr *)tx_skb->data;

		skb_pull(tx_skb, tx_ctrl->hdrlen);
		ether_addr_copy(pnext_tx_desc->u.sa,
				ieee80211_get_SA(wh));
		ether_addr_copy(pnext_tx_desc->u.da,
				ieee80211_get_DA(wh));

		if (tx_ctrl->flags & TX_CTRL_EAPOL)
			ctrl = FIELD_PREP(TXRING_CTRL_TAG_MASK,
					  TXRING_CTRL_TAG_EAP);
		if (tx_ctrl->flags & TX_CTRL_TCP_ACK) {
			pnext_tx_desc->tcp_dst_src =
				cpu_to_le32(tx_ctrl->tcp_dst_src);
			pnext_tx_desc->tcp_sn = cpu_to_le32(tx_ctrl->tcp_sn);
			ctrl = FIELD_PREP(TXRING_CTRL_TAG_MASK,
					  TXRING_CTRL_TAG_TCP_ACK);
		}
		ctrl |= FIELD_PREP(TXRING_CTRL_QID_MASK,
				   tx_ctrl->tx_que_priority) |
			FIELD_PREP(TXRING_CTRL_LEN_MASK, tx_skb->len);
	} else {
		/* Assigning rate code; use legacy 6mbps rate. */
		pnext_tx_desc->u.rate_code = cpu_to_le16(RATECODE_TYPE_LEGACY |
			FIELD_PREP(RATECODE_MCS_MASK, 0) | RATECODE_BW_20MHZ);
		pnext_tx_desc->u.max_retry = 5;

		ctrl = FIELD_PREP(TXRING_CTRL_QID_MASK,
				  tx_ctrl->tx_que_priority) |
			FIELD_PREP(TXRING_CTRL_LEN_MASK,
				   tx_skb->len - sizeof(struct pcie_dma_data)) |
			FIELD_PREP(TXRING_CTRL_TAG_MASK, TXRING_CTRL_TAG_MGMT);
	}

	dma = dma_map_single(&(pcie_priv->pdev)->dev, tx_skb->data,
			     tx_skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(&(pcie_priv->pdev)->dev, dma)) {
		ieee80211_free_txskb(priv->hw, tx_skb);
		wiphy_err_ratelimited(priv->hw->wiphy,
				      "failed to map pci memory!\n");
		return -EIO;
	}

	index = pcie_tx_set_skb(priv, tx_skb, dma);
	if (index == MAX_TX_RING_SEND_SIZE) {
		dma_unmap_single(&(pcie_priv->pdev)->dev, dma, tx_skb->len,
				 DMA_TO_DEVICE);
		ieee80211_free_txskb(priv->hw, tx_skb);
		wiphy_err_ratelimited(priv->hw->wiphy, "no free TX buffer slot\n");
		return -ENOSPC;
	}
	pnext_tx_desc->data = cpu_to_le32(dma);
	pnext_tx_desc->ctrl = cpu_to_le32(ctrl);
	desc->tx_buflen[index] = tx_skb->len;
	pnext_tx_desc->user = cpu_to_le32(index);

	if (tx_ctrl->flags & TX_CTRL_STATUS_SENT) {
		__set_bit(index, desc->tx_status_sent);
	} else if ((tx_ctrl->flags & TX_CTRL_TYPE_DATA) &&
		   (tx_ctrl->rate != 0)) {
		__set_bit(index, desc->tx_status_sent);
		skb_push(tx_skb, tx_ctrl->hdrlen);
		skb_get(tx_skb);
		pcie_tx_prepare_info(priv, tx_ctrl->rate, tx_info);
		ieee80211_tx_status_skb(priv->hw, tx_skb);
	}

	if (++tx_send_head_new >= MAX_NUM_TX_DESC)
		tx_send_head_new = 0;
	desc->tx_sent_head = tx_send_head_new;
	wmb(); /*Data Memory Barrier*/
	writel(tx_send_head_new, pcie_priv->iobase1 + MACREG_REG_TXSENDHEAD);
	desc->tx_desc_busy_cnt++;

	return 0;
}

static inline void pcie_tx_check_tcp_ack(struct sk_buff *tx_skb,
					 struct pcie_tx_ctrl_ndp *tx_ctrl)
{
	struct iphdr *iph;
	struct tcphdr *tcph;
	unsigned int off;

	if (!(tx_ctrl->flags & TX_CTRL_TYPE_DATA) ||
	    tx_skb->protocol != htons(ETH_P_IP))
		return;

	/* Skip the 802.11 header and the LLC/SNAP header. */
	off = tx_ctrl->hdrlen + 8;
	if (skb_headlen(tx_skb) < off + sizeof(*iph))
		return;
	iph = (struct iphdr *)(tx_skb->data + off);
	if (iph->protocol != IPPROTO_TCP || iph->ihl < 5 ||
	    skb_headlen(tx_skb) < off + iph->ihl * 4 + sizeof(*tcph))
		return;
	tcph = (struct tcphdr *)((u8 *)iph + (iph->ihl * 4));

	if ((tcph->ack == 1) && (ntohs(iph->tot_len) ==
	    (iph->ihl * 4 + tcph->doff * 4))) {
		if (tcph->syn || tcph->fin)
			return;

		tx_ctrl->flags |= TX_CTRL_TCP_ACK;
		tx_ctrl->tcp_dst_src = ntohs(tcph->source) |
			(ntohs(tcph->dest) << 16);
		tx_ctrl->tcp_sn = ntohl(tcph->ack_seq);
	}
}

int pcie_tx_init_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct sk_buff skb;
	struct ieee80211_tx_info *tx_info = IEEE80211_SKB_CB(&skb);
	int rc;

	BUILD_BUG_ON(sizeof(struct pcie_tx_ctrl_ndp) >
		     sizeof(tx_info->driver_data));

	rc = pcie_tx_ring_alloc_ndp(priv);
	if (rc) {
		pcie_tx_ring_free_ndp(priv);
		wiphy_err(hw->wiphy, "allocating TX ring failed\n");
		return rc;
	}

	rc = pcie_tx_ring_init_ndp(priv);
	if (rc) {
		pcie_tx_ring_free_ndp(priv);
		wiphy_err(hw->wiphy, "initializing TX ring failed\n");
		return rc;
	}

	return 0;
}

void pcie_tx_deinit_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	pcie_tx_ring_cleanup_ndp(priv);
	pcie_tx_ring_free_ndp(priv);
}

static int pcie_tx_prepare_ndp(struct ieee80211_hw *hw,
			       struct ieee80211_sta *sta, struct sk_buff *skb);

/* Whether the firmware may take another frame, after reaping the TX done
 * ring if it has as many as it may.
 */
static bool pcie_tx_room_ndp(struct ieee80211_hw *hw, u32 inflight_max)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	if (pcie_priv->desc_data_ndp.tx_desc_busy_cnt < inflight_max)
		return true;
	pcie_tx_done_ndp(hw);
	return pcie_priv->desc_data_ndp.tx_desc_busy_cnt < inflight_max;
}

/* Hand a prepared frame to the firmware. Returns false if the ring took
 * no more: the frame is then back at the head of the driver queue q, or
 * dropped.
 */
static bool pcie_tx_send_ndp(struct ieee80211_hw *hw,
			     struct sk_buff_head *q, struct sk_buff *skb)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int rc;

	rc = pcie_tx_skb_ndp(priv, skb);
	if (rc) {
		pcie_tx_done_ndp(hw);
		if (rc == -EAGAIN)
			skb_queue_head(q, skb);
		return false;
	}

	if (++pcie_priv->tx_done_cnt > TXDONE_THRESHOLD) {
		pcie_tx_done_ndp(hw);
		pcie_priv->tx_done_cnt = 0;
	}
	return true;
}

/* Pull frames from mac80211's TXQs of an AC, one station after the other.
 * Returns false if the firmware took no more.
 */
static bool pcie_tx_pull_ndp(struct ieee80211_hw *hw, int num,
			     u32 inflight_max)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int ac = SYSADPT_TX_WMM_QUEUES - num - 1;
	struct ieee80211_txq *txq;
	struct sk_buff *skb;
	bool room = true;
	int qnum;

	ieee80211_txq_schedule_start(hw, ac);
	while (room && (txq = ieee80211_next_txq(hw, ac))) {
		/* mac80211 requires RCU to be held from dequeueing a frame
		 * until the driver is done with it: tx_info->control.hw_key
		 * may be freed once it is dropped.
		 */
		rcu_read_lock();
		while ((room = pcie_tx_room_ndp(hw, inflight_max)) &&
		       (skb = ieee80211_tx_dequeue(hw, txq))) {
			qnum = pcie_tx_prepare_ndp(hw, txq->sta, skb);
			if (qnum < 0)
				continue;
			room = pcie_tx_send_ndp(hw, &pcie_priv->txq[qnum], skb);
			if (!room)
				break;
		}
		rcu_read_unlock();
		ieee80211_return_txq(hw, txq, false);
	}
	ieee80211_txq_schedule_end(hw, ac);

	return room;
}

/* Hand frames to the firmware while fewer than tx_inflight_max are in
 * flight, for each AC from VO to BK: first those in the driver queue,
 * which mac80211 handed over directly, then those in its TXQs.
 */
void pcie_tx_skbs_ndp(struct work_struct *work)
{
	struct pcie_priv *pcie_priv = from_work(pcie_priv, work, tx_work);
	struct mwl_priv *priv = pcie_mwl_priv(pcie_priv);
	struct ieee80211_hw *hw = priv->hw;
	int num = SYSADPT_TX_WMM_QUEUES;
	struct sk_buff *skb;
	u32 inflight_max;
	bool room = true;

	inflight_max = clamp_t(u32, pcie_priv->tx_inflight_max, 1,
			       MAX_TX_RING_SEND_SIZE - 1);

	while (room && num--) {
		while ((room = pcie_tx_room_ndp(hw, inflight_max)) &&
		       (skb = skb_dequeue(&pcie_priv->txq[num]))) {
			room = pcie_tx_send_ndp(hw, &pcie_priv->txq[num], skb);
			if (!room)
				break;
		}

		if (room && priv->radio_on)
			room = pcie_tx_pull_ndp(hw, num, inflight_max);
	}

	/* Look again once the firmware has room: mac80211 doesn't call
	 * wake_tx_queue for frames it already told the driver about.
	 */
	WRITE_ONCE(pcie_priv->tx_txq_waiting, !room);

	if (pcie_priv->desc_data_ndp.tx_desc_busy_cnt)
		pcie_arm_tx_poll(pcie_priv);
}

bool pcie_tx_pending_ndp(struct pcie_priv *pcie_priv)
{
	int num;

	if (READ_ONCE(pcie_priv->tx_txq_waiting))
		return true;
	for (num = 0; num < SYSADPT_TX_WMM_QUEUES; num++)
		if (skb_queue_len(&pcie_priv->txq[num]))
			return true;
	return false;
}

void pcie_tx_done_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	u32 tx_done_head, tx_done_tail;
	struct tx_ring_done *ptx_ring_done;
	u32 index;

	guard(spinlock_bh)(&pcie_priv->tx_desc_lock);

	tx_done_head = readl(pcie_priv->iobase1 +
			     MACREG_REG_TXDONEHEAD);
	tx_done_tail = desc->tx_done_tail & (MAX_TX_RING_DONE_SIZE - 1);
	tx_done_head &= (MAX_TX_RING_DONE_SIZE - 1);

	while (tx_done_head != tx_done_tail) {
		ptx_ring_done = &desc->ptx_ring_done[tx_done_tail];

		index = le32_to_cpu(ptx_ring_done->user);
		ptx_ring_done->user = 0;
		/* Skip a bad entry rather than stall the ring on it. The
		 * firmware has still finished with one descriptor, so count
		 * it as done; otherwise the busy count only grows and TX
		 * eventually stops for good.
		 */
		if (index >= MAX_TX_RING_SEND_SIZE) {
			wiphy_err_ratelimited(hw->wiphy,
					      "corruption for index of buffer\n");
		} else if (!desc->tx_vbuflist[index]) {
			wiphy_err_ratelimited(hw->wiphy,
					      "buffer is NULL for tx done ring\n");
		} else {
			pcie_tx_return_skb_ndp(hw, index, false);
		}
		if (desc->tx_desc_busy_cnt)
			desc->tx_desc_busy_cnt--;

		if (++tx_done_tail >= MAX_TX_RING_DONE_SIZE)
			tx_done_tail = 0;
	}

	writel(tx_done_tail, pcie_priv->iobase1 +
	       MACREG_REG_TXDONETAIL);
	desc->tx_done_tail = tx_done_tail;
}

/* Fill in the driver data of a frame for pcie_tx_skb_ndp(), and return
 * the driver queue it belongs to, or -1 if the frame was dropped.
 */
static int pcie_tx_prepare_ndp(struct ieee80211_hw *hw,
			       struct ieee80211_sta *sta, struct sk_buff *skb)
{
	struct mwl_priv *priv = hw->priv;
	struct ieee80211_tx_info *tx_info;
	struct ieee80211_key_conf *k_conf;
	struct mwl_vif *mwl_vif;
	int index;
	struct mwl_sta *sta_info;
	struct ieee80211_hdr *wh;
	u8 *da;
	u16 qos;
	u8 tid = 0;
	struct mwl_ampdu_stream *stream = NULL;
	u16 tx_que_priority;
	bool mgmtframe = false;
	struct ieee80211_mgmt *mgmt;
	bool eapol_frame = false;
	bool start_ba_session = false;
	struct pcie_tx_ctrl_ndp *tx_ctrl;
	int mic_room = 0;
	bool status_sent = false;

	tx_info = IEEE80211_SKB_CB(skb);
	k_conf = tx_info->control.hw_key;
	mwl_vif = tx_info->control.vif ?
		  mwl_dev_get_vif(tx_info->control.vif) : NULL;
	index = skb_get_queue_mapping(skb);
	sta_info = sta ? mwl_dev_get_sta(sta) : NULL;

	wh = (struct ieee80211_hdr *)skb->data;

	if (ieee80211_is_data_qos(wh->frame_control))
		qos = le16_to_cpu(*((__le16 *)ieee80211_get_qos_ctl(wh)));
	else
		qos = 0xFFFF;

	if (skb->protocol == cpu_to_be16(ETH_P_PAE)) {
		index = IEEE80211_AC_VO;
		eapol_frame = true;
	}

	if (ieee80211_is_mgmt(wh->frame_control)) {
		mgmtframe = true;
		mgmt = (struct ieee80211_mgmt *)skb->data;
	}

	/* The firmware builds the header of data frames itself, from the
	 * queue they're sent on, so an injected data frame without a vif
	 * can't be sent as it is.
	 */
	if (!mwl_vif && !mgmtframe) {
		ieee80211_free_txskb(hw, skb);
		return -1;
	}

	if (mgmtframe) {
		u16 capab;

		/* The body of a protected frame starts with its CCMP
		 * header (or is encrypted). mac80211 queues ADDBA requests
		 * on the right AC anyway.
		 */
		if (unlikely(ieee80211_is_action(wh->frame_control) &&
			     !ieee80211_has_protected(wh->frame_control) &&
			     mgmt->u.action.category == WLAN_CATEGORY_BACK &&
			     MWL_ACTION_CODE(mgmt) ==
			     WLAN_ACTION_ADDBA_REQ)) {
			capab = le16_to_cpu(MWL_ADDBA_REQ(mgmt).capab);
			tid = FIELD_GET(IEEE80211_ADDBA_PARAM_TID_MASK, capab);
			if (tid < IEEE80211_FIRST_TSPEC_TSID)
				index = utils_tid_to_ac(tid);
		}

		if (ieee80211_is_probe_req(wh->frame_control) ||
		    ieee80211_is_probe_resp(wh->frame_control))
			tx_que_priority = PROBE_RESPONSE_TXQNUM;
		else {
			if ((
			    mwl_vif && (mwl_vif->macid == SYSADPT_NUM_OF_AP) &&
			    (!ieee80211_has_protected(wh->frame_control) ||
			    (ieee80211_has_protected(wh->frame_control) &&
			    ieee80211_is_auth(wh->frame_control)))
			    ) ||
			    !sta ||
			    ieee80211_is_auth(wh->frame_control) ||
			    ieee80211_is_assoc_req(wh->frame_control) ||
			    ieee80211_is_assoc_resp(wh->frame_control))
				tx_que_priority = MGMT_TXQNUM;
			else {
				if (is_multicast_ether_addr(wh->addr1) &&
				    (mwl_vif->macid != SYSADPT_NUM_OF_AP))
					tx_que_priority = mwl_vif->macid *
						SYSADPT_MAX_TID;
				else
					tx_que_priority = SYSADPT_MAX_TID *
						(sta_info->stnid +
						QUEUE_STAOFFSET) + 6;
			}
		}

		/* The firmware reports no ACK status, and can hold on to
		 * probe responses for minutes, so report the status of a
		 * copy right away.
		 */
		if (ieee80211_is_assoc_resp(wh->frame_control) ||
		    ieee80211_is_reassoc_resp(wh->frame_control) ||
		    (ieee80211_is_probe_resp(wh->frame_control) &&
		     (tx_info->flags & IEEE80211_TX_CTL_REQ_TX_STATUS))) {
			struct sk_buff *ack_skb;
			struct ieee80211_tx_info *ack_info;

			ack_skb = skb_copy(skb, GFP_ATOMIC);
			if (ack_skb) {
				ack_info = IEEE80211_SKB_CB(ack_skb);
				pcie_tx_prepare_info(priv, 0, ack_info);
				ieee80211_tx_status_skb(hw, ack_skb);
				status_sent = true;
			}
		}

		if (pcie_tx_encapsulate_frame(priv, skb, k_conf)) {
			ieee80211_free_txskb(hw, skb);
			return -1;
		}
		if (k_conf &&
		    (k_conf->cipher == WLAN_CIPHER_SUITE_CCMP ||
		     k_conf->cipher == WLAN_CIPHER_SUITE_CCMP_256 ||
		     k_conf->cipher == WLAN_CIPHER_SUITE_GCMP ||
		     k_conf->cipher == WLAN_CIPHER_SUITE_GCMP_256))
			mic_room = k_conf->icv_len;
	} else {
		tid = qos & 0x7;
		if (sta && sta->deflink.ht_cap.ht_supported && !eapol_frame &&
		    qos != 0xFFFF) {
			pcie_tx_count_packet(sta, tid);
			spin_lock_bh(&priv->stream_lock);
			stream = mwl_fwcmd_lookup_stream(hw, sta, tid);
			if (!stream ||
			    stream->state == AMPDU_STREAM_IN_PROGRESS) {
				wiphy_warn_ratelimited(hw->wiphy,
						       "can't send packet during ADDBA\n");
				spin_unlock_bh(&priv->stream_lock);
				ieee80211_free_txskb(hw, skb);
				return -1;
			}
			if ((stream->state == AMPDU_NO_STREAM) &&
			    mwl_fwcmd_ampdu_allowed(sta, tid)) {
				stream = mwl_fwcmd_add_stream(hw, sta, tid);
				if (stream)
					start_ba_session = true;
			}
			spin_unlock_bh(&priv->stream_lock);
		}

		da = ieee80211_get_DA(wh);

		if (is_multicast_ether_addr(da)
		    && (mwl_vif->macid != SYSADPT_NUM_OF_AP)) {

			tx_que_priority = mwl_vif->macid * SYSADPT_MAX_TID;

			if (da[ETH_ALEN - 1] == 0xff)
				tx_que_priority += 7;

			if (ieee80211_has_a4(wh->frame_control)) {
				if (sta && sta_info->wds)
					tx_que_priority = SYSADPT_MAX_TID *
						(sta_info->stnid +
						QUEUE_STAOFFSET) + 6;
			}
		} else {
			if (sta) {
				if (!eapol_frame)
					tx_que_priority = SYSADPT_MAX_TID *
						(sta_info->stnid +
						QUEUE_STAOFFSET) +
						((qos == 0xFFFF) ? 0 : tid);
				else
					tx_que_priority = SYSADPT_MAX_TID *
						(sta_info->stnid +
						QUEUE_STAOFFSET) +
						((qos == 0xFFFF) ? 0 : 6);
			} else
				tx_que_priority = 0;
		}
	}

	if (mic_room) {
		/* The firmware expects the length of a protected
		 * management frame to include room for the MIC.
		 */
		if (skb_tailroom(skb) < mic_room &&
		    pskb_expand_head(skb, 0, mic_room, GFP_ATOMIC)) {
			ieee80211_free_txskb(hw, skb);
			return -1;
		}
		skb_put_zero(skb, mic_room);
		wh = &((struct pcie_dma_data *)skb->data)->wh;
	}

	index = SYSADPT_TX_WMM_QUEUES - index - 1;

	tx_ctrl = (struct pcie_tx_ctrl_ndp *)tx_info->driver_data;
	tx_ctrl->tx_que_priority = tx_que_priority;
	tx_ctrl->hdrlen = ieee80211_hdrlen(wh->frame_control);
	tx_ctrl->flags = 0;
	tx_ctrl->mic_room = mic_room;
	if (status_sent)
		tx_ctrl->flags |= TX_CTRL_STATUS_SENT;
	tx_ctrl->qos = qos;
	if (!mgmtframe)
		tx_ctrl->flags |= TX_CTRL_TYPE_DATA;
	if (eapol_frame)
		tx_ctrl->flags |= TX_CTRL_EAPOL;
	/* With a rate, the status is reported as soon as the frame goes to
	 * the firmware, which only reports completions 64 at a time. Do
	 * that for null-data frames too: hostapd polls idle stations with
	 * them and disconnects a station whose poll gets no status.
	 */
	tx_ctrl->rate = sta ? sta_info->tx_rate_info : 0;
	pcie_tx_check_tcp_ack(skb, tx_ctrl);

	/* Initiate the ampdu session here */
	if (start_ba_session) {
		scoped_guard(spinlock_bh, &priv->stream_lock) {
			if (mwl_fwcmd_start_stream(hw, stream))
				mwl_fwcmd_remove_stream(hw, stream);
			stream->jiffies = jiffies;
			stream->start_time = stream->jiffies;
			stream->desc_num = index;
		}
	}

	return index;
}

/* Frames mac80211 hands over directly instead of through a TXQ, such as
 * management frames. They go out before the TXQs of their AC.
 */
void pcie_tx_xmit_ndp(struct ieee80211_hw *hw,
		      struct ieee80211_tx_control *control,
		      struct sk_buff *skb)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int num;

	num = pcie_tx_prepare_ndp(hw, control->sta, skb);
	if (num < 0)
		return;

	skb_queue_tail(&pcie_priv->txq[num], skb);
	pcie_schedule_tx(pcie_priv);
}

void pcie_wake_tx_queue_ndp(struct ieee80211_hw *hw,
			    struct ieee80211_txq *txq)
{
	struct mwl_priv *priv = hw->priv;

	pcie_schedule_tx(priv->hif.priv);
}
