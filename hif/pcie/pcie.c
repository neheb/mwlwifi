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

/* Description:  This file implements functions needed for PCIe module. */

#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/seq_file.h>
#include <linux/iopoll.h>

#include "sysadpt.h"
#include "compat.h"
#include "core.h"
#include "utils.h"
#include "hif/fwcmd.h"
#include "hif/pcie/dev.h"
#include "hif/pcie/fwdl.h"
#include "hif/pcie/8864/tx.h"
#include "hif/pcie/8997/tx.h"
#include "hif/pcie/rx.h"
#include "hif/pcie/8964/tx_ndp.h"
#include "hif/pcie/8964/rx_ndp.h"

#define PCIE_DRV_DESC "Marvell Mac80211 Wireless PCIE Network Driver"

#define FW_CMD_POLL_US                  50
#define FW_CMD_TIMEOUT_US               (10 * USEC_PER_SEC)
#define CHECK_BA_TRAFFIC_TIME           HZ  /* 1 sec */
#define CHECK_TX_DONE_TIME              50  /* msec */

static struct pci_device_id pcie_id_tbl[] = {
	{ PCI_VDEVICE(MARVELL, 0x2a55),     .driver_data = MWL8864, },
	{ PCI_VDEVICE(MARVELL, 0x2b38),     .driver_data = MWL8897, },
	{ PCI_VDEVICE(MARVELL, 0x2b40),     .driver_data = MWL8964, },
	{ PCI_VDEVICE(MARVELL_EXT, 0x2b42), .driver_data = MWL8997, },
	{ },
};

static struct mwl_chip_info pcie_chip_tbl[] = {
	[MWL8864] = {
		.part_name	= "88W8864",
		.fw_image	= "mwlwifi/88W8864.bin",
		.cal_file	= NULL,
		.txpwrlmt_file  = NULL,
		.antenna_tx	= ANTENNA_TX_4_AUTO,
		.antenna_rx	= ANTENNA_RX_4_AUTO,
	},
	[MWL8897] = {
		.part_name	= "88W8897",
		.fw_image	= "mwlwifi/88W8897.bin",
		.cal_file	= NULL,
		.txpwrlmt_file  = NULL,
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
	[MWL8964] = {
		.part_name	= "88W8964",
		.fw_image	= "mwlwifi/88W8964.bin",
		.cal_file	= NULL,
		.txpwrlmt_file  = NULL,
		.antenna_tx	= ANTENNA_TX_4_AUTO,
		.antenna_rx	= ANTENNA_RX_4_AUTO,
	},
	[MWL8997] = {
		.part_name	= "88W8997",
		.fw_image	= "mwlwifi/88W8997.bin",
		.cal_file	= "mwlwifi/WlanCalData_ext.conf",
		.txpwrlmt_file  = "mwlwifi/txpwrlmt_cfg.conf",
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
};

static int pcie_alloc_resource(struct pcie_priv *pcie_priv)
{
	struct pci_dev *pdev = pcie_priv->pdev;
	int bar1 = 1;

	/* A 64-bit BAR 0 takes two BAR slots. */
	if (pci_resource_flags(pdev, 0) & IORESOURCE_MEM_64)
		bar1 = 2;

	pcie_priv->iobase0 = pcim_iomap_region(pdev, 0, PCIE_DRV_NAME);
	if (IS_ERR(pcie_priv->iobase0))
		return dev_err_probe(&pdev->dev, PTR_ERR(pcie_priv->iobase0),
				     "cannot map BAR 0\n");

	pcie_priv->iobase1 = pcim_iomap_region(pdev, bar1, PCIE_DRV_NAME);
	if (IS_ERR(pcie_priv->iobase1))
		return dev_err_probe(&pdev->dev, PTR_ERR(pcie_priv->iobase1),
				     "cannot map BAR %d\n", bar1);

	return 0;
}

static u32 pcie_read_mac_reg(struct pcie_priv *pcie_priv, u32 offset)
{
	struct mwl_priv *priv = pcie_mwl_priv(pcie_priv);

	if (priv->chip_type == MWL8964) {
		/* The firmware returns as many words as the length asks for. */
		u32 addr_val[4] = {};

		if (mwl_fwcmd_get_addr_value(priv->hw, 0x8000a000 + offset,
					     ARRAY_SIZE(addr_val), addr_val, 0))
			return 0;
		return addr_val[0];
	}

	return readl(MAC_REG_ADDR_PCI(offset));
}

static bool pcie_chk_adapter(struct pcie_priv *pcie_priv)
{
	struct mwl_priv *priv = pcie_mwl_priv(pcie_priv);
	u32 regval;

	regval = readl(pcie_priv->iobase1 + MACREG_REG_INT_CODE);

	if (regval == 0xffffffff) {
		wiphy_err(priv->hw->wiphy, "adapter does not exist\n");
		return false;
	}

	if (priv->cmd_timeout)
		wiphy_debug(priv->hw->wiphy, "MACREG_REG_INT_CODE: 0x%04x\n",
			    regval);

	return true;
}

static void pcie_send_cmd(struct pcie_priv *pcie_priv)
{
	writel(pcie_mwl_priv(pcie_priv)->pphys_cmd_buf,
	       pcie_priv->iobase1 + MACREG_REG_GEN_PTR);
	writel(MACREG_H2ARIC_BIT_DOOR_BELL,
	       pcie_priv->iobase1 + MACREG_REG_H2A_INTERRUPT_EVENTS);
}

static int pcie_wait_complete(struct mwl_priv *priv, unsigned short cmd)
{
	__le16 *pcmd = priv->pcmd_buf;
	u16 int_code;

	/* The firmware writes the response code over the command code.
	 * Most commands take well under a millisecond, so poll finely.
	 */
	read_poll_timeout(le16_to_cpu, int_code,
			  int_code == cmd || READ_ONCE(priv->rmmod),
			  FW_CMD_POLL_US, FW_CMD_TIMEOUT_US, false,
			  READ_ONCE(pcmd[0]));
	/* Read the response only after its code. */
	dma_rmb();

	if (int_code != cmd) {
		if (priv->rmmod)
			return -ENODEV;
		wiphy_err(priv->hw->wiphy, "cmd 0x%04x=%s timed out\n",
			  cmd, mwl_fwcmd_get_cmd_string(cmd));
		wiphy_err(priv->hw->wiphy, "return code: 0x%04x\n", int_code);
		return -EIO;
	}

	/* The firmware can post the response before it is ready for the
	 * next command: without this, the 88W8964 misses a SET_SPECTRUM_MGMT
	 * doorbell right after AP_BEACON.
	 */
	if (priv->chip_type != MWL8997)
		usleep_range(3000, 5000);

	return 0;
}

static int pcie_init_8997(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct hostcmd_get_hw_spec hw_spec;
	const struct hostcmd_get_hw_spec *get_hw_spec = &hw_spec;
	struct hostcmd_set_hw_spec set_hw_spec;
	int rc, i;

	spin_lock_init(&pcie_priv->int_mask_lock);
	/* The TX work stays disabled until the radio is started. */
	INIT_WORK(&pcie_priv->tx_work, pcie_8997_tx_skbs);
	disable_work(&pcie_priv->tx_work);
	INIT_WORK(&pcie_priv->tx_done_work, pcie_8997_tx_done_task);
	disable_work(&pcie_priv->tx_done_work);
	spin_lock_init(&pcie_priv->tx_desc_lock);
	pcie_priv->napi_dev = alloc_netdev_dummy(0);
	if (!pcie_priv->napi_dev)
		return -ENOMEM;
	netif_napi_add(pcie_priv->napi_dev, &pcie_priv->napi, pcie_rx_poll_napi);
	pcie_priv->txq_limit = PCIE_TX_QUEUE_LIMIT;
	pcie_priv->txq_wake_threshold = PCIE_TX_WAKE_Q_THRESHOLD;
	pcie_priv->recv_limit = PCIE_RECEIVE_LIMIT;

	rc = pcie_8997_tx_init(hw);
	if (rc)
		goto err_mwl_tx_init;

	rc = pcie_rx_init(hw);
	if (rc)
		goto err_mwl_rx_init;

	/* get and prepare HW specifications */
	if (mwl_fwcmd_get_hw_specs(hw, &hw_spec)) {
		rc = -EIO;
		goto err_get_hw_specs;
	}
	ether_addr_copy(&priv->hw_data.mac_addr[0],
			get_hw_spec->permanent_addr);
	pcie_priv->desc_data[0].wcb_base =
		le32_to_cpu(get_hw_spec->wcb_base0) & 0x0000ffff;
	for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
		pcie_priv->desc_data[i].wcb_base =
			le32_to_cpu(get_hw_spec->wcb_base[i - 1]) & 0x0000ffff;
	pcie_priv->desc_data[0].rx_desc_read =
		le32_to_cpu(get_hw_spec->rxpd_rd_ptr) & 0x0000ffff;
	pcie_priv->desc_data[0].rx_desc_write =
		le32_to_cpu(get_hw_spec->rxpd_wr_ptr) & 0x0000ffff;
	priv->hw_data.fw_release_num = le32_to_cpu(get_hw_spec->fw_release_num);
	priv->hw_data.hw_version = get_hw_spec->version;
	writel(pcie_priv->desc_data[0].pphys_rx_ring,
	       pcie_priv->iobase0 + pcie_priv->desc_data[0].rx_desc_read);
	writel(pcie_priv->desc_data[0].pphys_rx_ring,
	       pcie_priv->iobase0 + pcie_priv->desc_data[0].rx_desc_write);

	/* prepare and set HW specifications */
	memset(&set_hw_spec, 0, sizeof(set_hw_spec));
	set_hw_spec.wcb_base[0] =
			cpu_to_le32(pcie_priv->txbd_ring_pbase);
	set_hw_spec.tx_wcb_num_per_queue =
			cpu_to_le32(PCIE_MAX_TXRX_BD);
	set_hw_spec.num_tx_queues = cpu_to_le32(1);
	set_hw_spec.features |= cpu_to_le32(HW_SET_PARMS_FEATURES_HOST_PROBE_RESP);
	set_hw_spec.total_rx_wcb = cpu_to_le32(PCIE_MAX_NUM_RX_DESC);
	set_hw_spec.rxpd_wr_ptr = cpu_to_le32(pcie_priv->desc_data[0].pphys_rx_ring);
	rc = mwl_fwcmd_set_hw_specs(hw, &set_hw_spec);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to set HW specifications\n", PCIE_DRV_NAME);
		goto err_set_hw_specs;
	}

	return rc;

err_set_hw_specs:
err_get_hw_specs:

	pcie_rx_deinit(hw);

err_mwl_rx_init:

	pcie_8997_tx_deinit(hw);

err_mwl_tx_init:
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);

	return rc;
}

static int pcie_init_8864(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct hostcmd_get_hw_spec hw_spec;
	const struct hostcmd_get_hw_spec *get_hw_spec = &hw_spec;
	struct hostcmd_set_hw_spec set_hw_spec;
	int rc, i;

	spin_lock_init(&pcie_priv->int_mask_lock);
	/* The TX work stays disabled until the radio is started. */
	INIT_WORK(&pcie_priv->tx_work, pcie_8864_tx_skbs);
	disable_work(&pcie_priv->tx_work);
	INIT_WORK(&pcie_priv->tx_done_work, pcie_8864_tx_done_task);
	disable_work(&pcie_priv->tx_done_work);
	spin_lock_init(&pcie_priv->tx_desc_lock);
	pcie_priv->napi_dev = alloc_netdev_dummy(0);
	if (!pcie_priv->napi_dev)
		return -ENOMEM;
	netif_napi_add(pcie_priv->napi_dev, &pcie_priv->napi, pcie_rx_poll_napi);
	pcie_priv->txq_limit = PCIE_TX_QUEUE_LIMIT;
	pcie_priv->txq_wake_threshold = PCIE_TX_WAKE_Q_THRESHOLD;
	pcie_priv->recv_limit = PCIE_RECEIVE_LIMIT;

	rc = pcie_8864_tx_init(hw);
	if (rc)
		goto err_mwl_tx_init;

	rc = pcie_rx_init(hw);
	if (rc)
		goto err_mwl_rx_init;

	/* get and prepare HW specifications */
	if (mwl_fwcmd_get_hw_specs(hw, &hw_spec)) {
		rc = -EIO;
		goto err_get_hw_specs;
	}
	ether_addr_copy(&priv->hw_data.mac_addr[0], get_hw_spec->permanent_addr);
	pcie_priv->desc_data[0].wcb_base = le32_to_cpu(get_hw_spec->wcb_base0) & 0x0000ffff;

	for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
		pcie_priv->desc_data[i].wcb_base = le32_to_cpu(get_hw_spec->wcb_base[i - 1]) & 0x0000ffff;

	pcie_priv->desc_data[0].rx_desc_read = le32_to_cpu(get_hw_spec->rxpd_rd_ptr) & 0x0000ffff;
	pcie_priv->desc_data[0].rx_desc_write = le32_to_cpu(get_hw_spec->rxpd_wr_ptr) & 0x0000ffff;
	priv->hw_data.fw_release_num = le32_to_cpu(get_hw_spec->fw_release_num);
	priv->hw_data.hw_version = get_hw_spec->version;
	writel(pcie_priv->desc_data[0].pphys_tx_ring, pcie_priv->iobase0 + pcie_priv->desc_data[0].wcb_base);
	for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
		writel(pcie_priv->desc_data[i].pphys_tx_ring, pcie_priv->iobase0 + pcie_priv->desc_data[i].wcb_base);
	writel(pcie_priv->desc_data[0].pphys_rx_ring, pcie_priv->iobase0 + pcie_priv->desc_data[0].rx_desc_read);
	writel(pcie_priv->desc_data[0].pphys_rx_ring, pcie_priv->iobase0 + pcie_priv->desc_data[0].rx_desc_write);

	/* prepare and set HW specifications */
	memset(&set_hw_spec, 0, sizeof(set_hw_spec));
	set_hw_spec.wcb_base[0] =
		cpu_to_le32(pcie_priv->desc_data[0].pphys_tx_ring);
	for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
		set_hw_spec.wcb_base[i] = cpu_to_le32(
			pcie_priv->desc_data[i].pphys_tx_ring);
	set_hw_spec.features |= cpu_to_le32(HW_SET_PARMS_FEATURES_HOST_PROBE_RESP);
	set_hw_spec.tx_wcb_num_per_queue =
			cpu_to_le32(PCIE_MAX_NUM_TX_DESC);
	set_hw_spec.num_tx_queues = cpu_to_le32(PCIE_NUM_OF_DESC_DATA);
	set_hw_spec.total_rx_wcb = cpu_to_le32(PCIE_MAX_NUM_RX_DESC);
	set_hw_spec.rxpd_wr_ptr =
		cpu_to_le32(pcie_priv->desc_data[0].pphys_rx_ring);
	rc = mwl_fwcmd_set_hw_specs(hw, &set_hw_spec);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to set HW specifications\n",
			  PCIE_DRV_NAME);
		goto err_set_hw_specs;
	}

	return rc;

err_set_hw_specs:
err_get_hw_specs:

	pcie_rx_deinit(hw);

err_mwl_rx_init:

	pcie_8864_tx_deinit(hw);

err_mwl_tx_init:
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);

	return rc;
}

static void pcie_deinit_8997(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	/* Stop the firmware before freeing the buffers it may still use. */
	pcie_reset(hw);

	disable_work_sync(&pcie_priv->tx_done_work);
	disable_work_sync(&pcie_priv->tx_work);

	pcie_rx_deinit(hw);
	pcie_8997_tx_deinit(hw);
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);
}

static void pcie_deinit_8864(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	/* Stop the firmware before freeing the buffers it may still use. */
	pcie_reset(hw);

	disable_work_sync(&pcie_priv->tx_done_work);
	disable_work_sync(&pcie_priv->tx_work);

	pcie_rx_deinit(hw);
	pcie_8864_tx_deinit(hw);
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);
}

static void pcie_get_info(struct ieee80211_hw *hw, struct seq_file *s)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	seq_printf(s, "iobase0: %p\n",
		   pcie_priv->iobase0);
	seq_printf(s, "iobase1: %p\n",
		   pcie_priv->iobase1);
	seq_printf(s, "tx limit: %u\n", pcie_priv->txq_limit);
	seq_printf(s, "rx limit: %d\n", pcie_priv->recv_limit);
}

static void pcie_enable_data_tasks(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	enable_work(&pcie_priv->tx_done_work);
	enable_work(&pcie_priv->tx_work);
	napi_enable(&pcie_priv->napi);
	/* A disabled work item is not queued, unlike a disabled tasklet
	 * that runs once enabled, so look at the queues once now.
	 */
	pcie_schedule_tx(pcie_priv);
}

static void pcie_disable_data_tasks(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	disable_work_sync(&pcie_priv->tx_work);
	disable_work_sync(&pcie_priv->tx_done_work);
	napi_synchronize(&pcie_priv->napi);
	napi_disable(&pcie_priv->napi);
}

static int pcie_exec_cmd(struct ieee80211_hw *hw, unsigned short cmd)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int rc;

	might_sleep();

	if (!pcie_chk_adapter(pcie_priv)) {
		wiphy_err(priv->hw->wiphy, "adapter does not exist\n");
		priv->in_send_cmd = false;
		return -EIO;
	}

	if (priv->rmmod)
		return -ENODEV;

	if (priv->in_send_cmd) {
		wiphy_warn(priv->hw->wiphy, "previous command is running\n");
		return -EBUSY;
	}

	priv->in_send_cmd = true;
	if (priv->dump_hostcmd)
		wiphy_debug(priv->hw->wiphy, "send cmd 0x%04x=%s\n",
			    cmd, mwl_fwcmd_get_cmd_string(cmd));
	pcie_send_cmd(pcie_priv);
	rc = pcie_wait_complete(priv, 0x8000 | cmd);
	priv->in_send_cmd = false;
	if (rc == -EIO) {
		wiphy_err(priv->hw->wiphy, "timeout: 0x%04x\n", cmd);
		priv->cmd_timeout = true;
	}

	return rc;
}

static int pcie_get_irq_num(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	return pcie_priv->pdev->irq;
}

/* 88W8864 and 88W8997 */
static irqreturn_t pcie_isr(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	u32 int_status;

	int_status = readl(pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);

	if (int_status == 0x00000000)
		return IRQ_NONE;

	if (int_status == 0xffffffff) {
		wiphy_warn(hw->wiphy, "card unplugged?\n");
	} else {
		writel(~int_status,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);

		if (int_status & MACREG_A2HRIC_BIT_TX_DONE) {
			priv->hif.ops->irq_disable(hw);
			queue_work(system_bh_wq, &pcie_priv->tx_done_work);
		}

		if (int_status & MACREG_A2HRIC_BIT_RX_RDY) {
			priv->hif.ops->irq_disable(hw);
			napi_schedule(&pcie_priv->napi);
		}

		if (int_status & MACREG_A2HRIC_BIT_RADAR_DETECT) {
			wiphy_info(hw->wiphy, "radar detected by firmware\n");
			ieee80211_radar_detected(hw, NULL);
		}

		if (int_status & MACREG_A2HRIC_BIT_CHAN_SWITCH)
			ieee80211_queue_work(hw, &priv->chnl_switch_handle);

		if (int_status & MACREG_A2HRIC_BA_WATCHDOG)
			ieee80211_queue_work(hw, &priv->watchdog_ba_handle);
	}

	return IRQ_HANDLED;
}

static void pcie_irq_enable(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	if (pcie_chk_adapter(pcie_priv)) {
		writel(0x00,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
		writel(MACREG_A2HRIC_BIT_MASK,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
	}
}

static void pcie_irq_disable(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	if (pcie_chk_adapter(pcie_priv))
		writel(0x00,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
}

static void pcie_timer_routine(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_ampdu_stream *stream;
	struct mwl_sta *sta_info;
	struct mwl_tx_info *tx_stats;
	u8 i;

	if (jiffies - priv->jiffies_ampdu < CHECK_BA_TRAFFIC_TIME)
		return;
	priv->jiffies_ampdu = jiffies;

	guard(spinlock_bh)(&priv->stream_lock);
	for (i = 0; i < priv->ampdu_num; i++) {
		stream = &priv->ampdu[i];

		if (stream->state == AMPDU_STREAM_ACTIVE) {
			sta_info = mwl_dev_get_sta(stream->sta);
			tx_stats = &sta_info->tx_stats[stream->tid];

			if (jiffies - stream->jiffies > SYSADPT_TIMER_AMPDU_KEEPALIVE &&
			    tx_stats->pkts < SYSADPT_AMPDU_PACKET_THRESHOLD) {
				ieee80211_stop_tx_ba_session(stream->sta, stream->tid);
				if (priv->debug_ampdu)
					wiphy_debug(hw->wiphy, "Stop BA %pM\n", stream->sta->addr);
			}

			if (tx_stats->pkts > SYSADPT_AMPDU_PACKET_THRESHOLD)
				stream->jiffies = jiffies;

			if (jiffies - tx_stats->start_time > HZ) {
				tx_stats->pkts = 0;
				tx_stats->start_time = jiffies;
			}
		}
	}
}

static void pcie_8997_tx_return_pkts(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int num;

	pcie_8997_tx_done(hw);

	for (num = 0; num < PCIE_NUM_OF_DESC_DATA; num++)
		ieee80211_purge_tx_queue(hw, &pcie_priv->txq[num]);
}

static void pcie_8864_tx_return_pkts(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int num;

	pcie_8864_tx_done(hw);

	for (num = 0; num < PCIE_NUM_OF_DESC_DATA; num++)
		ieee80211_purge_tx_queue(hw, &pcie_priv->txq[num]);
}

static void pcie_get_survey(struct ieee80211_hw *hw,
			    struct mwl_survey_info *survey_info)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	survey_info->filled = SURVEY_INFO_TIME |
			      SURVEY_INFO_TIME_BUSY |
			      SURVEY_INFO_TIME_TX |
			      SURVEY_INFO_NOISE_DBM;
	survey_info->time_period += pcie_read_mac_reg(pcie_priv, MCU_LAST_READ);
	survey_info->time_busy += pcie_read_mac_reg(pcie_priv, MCU_CCA_CNT);
	survey_info->time_tx += pcie_read_mac_reg(pcie_priv, MCU_TXPE_CNT);
	survey_info->noise = priv->noise;
}

static struct mwl_hif_ops pcie_hif_ops_8997 = {
	.driver_name           = PCIE_DRV_NAME,
	.driver_version        = PCIE_DRV_VERSION,
	.tx_head_room          = PCIE_MIN_BYTES_HEADROOM,
	.ampdu_num             = PCIE_AMPDU_QUEUES,
	.reset                 = pcie_reset,
	.init                  = pcie_init_8997,
	.deinit                = pcie_deinit_8997,
	.get_info              = pcie_get_info,
	.enable_data_tasks     = pcie_enable_data_tasks,
	.disable_data_tasks    = pcie_disable_data_tasks,
	.exec_cmd              = pcie_exec_cmd,
	.get_irq_num           = pcie_get_irq_num,
	.irq_handler           = pcie_isr,
	.irq_enable            = pcie_irq_enable,
	.irq_disable           = pcie_irq_disable,
	.download_firmware     = pcie_download_firmware,
	.timer_routine         = pcie_timer_routine,
	.tx_xmit               = pcie_8997_tx_xmit,
	.tx_del_pkts_via_vif   = pcie_8997_tx_del_pkts_via_vif,
	.tx_del_pkts_via_sta   = pcie_8997_tx_del_pkts_via_sta,
	.tx_del_ampdu_pkts     = pcie_8997_tx_del_ampdu_pkts,
	.tx_del_sta_amsdu_pkts = pcie_8997_tx_del_sta_amsdu_pkts,
	.tx_return_pkts        = pcie_8997_tx_return_pkts,
	.get_survey            = pcie_get_survey,
};

static struct mwl_hif_ops pcie_hif_ops_8864 = {
	.driver_name           = PCIE_DRV_NAME,
	.driver_version        = PCIE_DRV_VERSION,
	.tx_head_room          = PCIE_MIN_BYTES_HEADROOM,
	.ampdu_num             = PCIE_AMPDU_QUEUES,
	.reset                 = pcie_reset,
	.init                  = pcie_init_8864,
	.deinit                = pcie_deinit_8864,
	.get_info              = pcie_get_info,
	.enable_data_tasks     = pcie_enable_data_tasks,
	.disable_data_tasks    = pcie_disable_data_tasks,
	.exec_cmd              = pcie_exec_cmd,
	.get_irq_num           = pcie_get_irq_num,
	.irq_handler           = pcie_isr,
	.irq_enable            = pcie_irq_enable,
	.irq_disable           = pcie_irq_disable,
	.download_firmware     = pcie_download_firmware,
	.timer_routine         = pcie_timer_routine,
	.tx_xmit               = pcie_8864_tx_xmit,
	.tx_del_pkts_via_vif   = pcie_8864_tx_del_pkts_via_vif,
	.tx_del_pkts_via_sta   = pcie_8864_tx_del_pkts_via_sta,
	.tx_del_ampdu_pkts     = pcie_8864_tx_del_ampdu_pkts,
	.tx_del_sta_amsdu_pkts = pcie_8864_tx_del_sta_amsdu_pkts,
	.tx_return_pkts        = pcie_8864_tx_return_pkts,
	.get_survey            = pcie_get_survey,
};

static enum hrtimer_restart pcie_tx_poll_timer_ndp(struct hrtimer *timer)
{
	struct pcie_priv *pcie_priv = container_of(timer, struct pcie_priv,
						   tx_poll_timer);

	/* The NAPI poll reaps the TX done ring. */
	napi_schedule(&pcie_priv->napi);
	return HRTIMER_NORESTART;
}

static int pcie_init_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct hostcmd_get_hw_spec hw_spec;
	const struct hostcmd_get_hw_spec *get_hw_spec = &hw_spec;
	struct hostcmd_set_hw_spec set_hw_spec;
	int rc;

	spin_lock_init(&pcie_priv->int_mask_lock);
	/* The TX work stays disabled until the radio is started. */
	INIT_WORK(&pcie_priv->tx_work, pcie_tx_skbs_ndp);
	disable_work(&pcie_priv->tx_work);
	mwl_hrtimer_setup(&pcie_priv->tx_poll_timer, pcie_tx_poll_timer_ndp,
			  CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pcie_priv->tx_poll_us = TX_POLL_US;
	spin_lock_init(&pcie_priv->tx_desc_lock);
	pcie_priv->napi_dev = alloc_netdev_dummy(0);
	if (!pcie_priv->napi_dev)
		return -ENOMEM;
	netif_napi_add(pcie_priv->napi_dev, &pcie_priv->napi, pcie_rx_poll_ndp);
	pcie_priv->tx_inflight_max = TX_INFLIGHT_MAX;

	rc = pcie_tx_init_ndp(hw);
	if (rc)
		goto err_mwl_tx_init;

	rc = pcie_rx_init_ndp(hw);
	if (rc)
		goto err_mwl_rx_init;

	/* get and prepare HW specifications */
	if (mwl_fwcmd_get_hw_specs(hw, &hw_spec)) {
		rc = -EIO;
		goto err_get_hw_specs;
	}
	ether_addr_copy(&priv->hw_data.mac_addr[0],
			get_hw_spec->permanent_addr);
	priv->hw_data.fw_release_num = le32_to_cpu(get_hw_spec->fw_release_num);
	priv->hw_data.hw_version = get_hw_spec->version;

	/* prepare and set HW specifications */
	memset(&set_hw_spec, 0, sizeof(set_hw_spec));
	set_hw_spec.wcb_base[0] =
		cpu_to_le32(pcie_priv->desc_data_ndp.pphys_tx_ring);
	set_hw_spec.wcb_base[1] =
		cpu_to_le32(pcie_priv->desc_data_ndp.pphys_tx_ring_done);
	set_hw_spec.wcb_base[2] =
		cpu_to_le32(pcie_priv->desc_data_ndp.pphys_rx_ring);
	set_hw_spec.wcb_base[3] =
		cpu_to_le32(pcie_priv->desc_data_ndp.pphys_rx_ring_done);
	set_hw_spec.acnt_base_addr =
		cpu_to_le32(pcie_priv->desc_data_ndp.pphys_acnt_ring);
	set_hw_spec.acnt_buf_size =
		cpu_to_le32(pcie_priv->desc_data_ndp.acnt_ring_size);

	rc = mwl_fwcmd_set_hw_specs(hw, &set_hw_spec);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to set HW specifications\n",
			  PCIE_DRV_NAME);
		goto err_set_hw_specs;
	}

	return rc;

err_set_hw_specs:
err_get_hw_specs:

	pcie_rx_deinit_ndp(hw);

err_mwl_rx_init:

	pcie_tx_deinit_ndp(hw);

err_mwl_tx_init:
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);

	return rc;
}

static void pcie_deinit_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	/* Stop the firmware before freeing the buffers it may still use. */
	pcie_reset(hw);

	disable_work_sync(&pcie_priv->tx_work);
	hrtimer_cancel(&pcie_priv->tx_poll_timer);

	pcie_rx_deinit_ndp(hw);
	pcie_tx_deinit_ndp(hw);
	netif_napi_del(&pcie_priv->napi);
	free_netdev(pcie_priv->napi_dev);
}

static void pcie_get_info_ndp(struct ieee80211_hw *hw, struct seq_file *s)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	seq_printf(s, "iobase0: %p\n",
		   pcie_priv->iobase0);
	seq_printf(s, "iobase1: %p\n",
		   pcie_priv->iobase1);
	seq_printf(s, "tx inflight max: %u\n", pcie_priv->tx_inflight_max);
}

static void pcie_get_tx_status_ndp(struct ieee80211_hw *hw, struct seq_file *s)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	seq_printf(s, "tx_done_cnt: %d\n",
		   pcie_priv->tx_done_cnt);
	seq_printf(s, "tx_desc_busy_cnt: %d\n",
		   pcie_priv->desc_data_ndp.tx_desc_busy_cnt);
	seq_printf(s, "txq_len: %u %u %u %u\n",
		   skb_queue_len(&pcie_priv->txq[0]),
		   skb_queue_len(&pcie_priv->txq[1]),
		   skb_queue_len(&pcie_priv->txq[2]),
		   skb_queue_len(&pcie_priv->txq[3]));
	seq_printf(s, "tx_sent_head: %d\n",
		   pcie_priv->desc_data_ndp.tx_sent_head);
	seq_printf(s, "tx_sent_tail: %d\n",
		   pcie_priv->desc_data_ndp.tx_sent_tail);
	seq_printf(s, "tx_done_head: %d\n",
		   readl(pcie_priv->iobase1 + MACREG_REG_TXDONEHEAD));
	seq_printf(s, "tx_done_tail: %d\n",
		   pcie_priv->desc_data_ndp.tx_done_tail);
	seq_printf(s, "tx_vbuflist_idx: %d\n",
		   pcie_priv->desc_data_ndp.tx_vbuflist_idx);
}

static void pcie_get_rx_status_ndp(struct ieee80211_hw *hw, struct seq_file *s)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	seq_printf(s, "rx_done_head: %d\n",
		   readl(pcie_priv->iobase1 + MACREG_REG_RXDONEHEAD));
	seq_printf(s, "rx_done_tail: %d\n",
		   readl(pcie_priv->iobase1 + MACREG_REG_RXDONETAIL));
	seq_printf(s, "rx_desc_head: %d\n",
		   readl(pcie_priv->iobase1 + MACREG_REG_RXDESCHEAD));
	seq_printf(s, "recheck_rxringdone: %d\n",
		   pcie_priv->recheck_rxringdone);
	seq_printf(s, "fast_data_cnt: %d\n",
		   pcie_priv->rx_cnts.fast_data_cnt);
	seq_printf(s, "fast_bad_amsdu_cnt: %d\n",
		   pcie_priv->rx_cnts.fast_bad_amsdu_cnt);
	seq_printf(s, "slow_noqueue_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_noqueue_cnt);
	seq_printf(s, "slow_norun_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_norun_cnt);
	seq_printf(s, "slow_mcast_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_mcast_cnt);
	seq_printf(s, "slow_bad_sta_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_bad_sta_cnt);
	seq_printf(s, "slow_bad_mic_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_bad_mic_cnt);
	seq_printf(s, "slow_bad_pn_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_bad_pn_cnt);
	seq_printf(s, "slow_mgmt_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_mgmt_cnt);
	seq_printf(s, "slow_promisc_cnt: %d\n",
		   pcie_priv->rx_cnts.slow_promisc_cnt);
	seq_printf(s, "drop_cnt: %d\n",
		   pcie_priv->rx_cnts.drop_cnt);
	seq_printf(s, "offch_promisc_cnt: %d\n",
		   pcie_priv->rx_cnts.offch_promisc_cnt);
	seq_printf(s, "mu_pkt_cnt: %d\n",
		   pcie_priv->rx_cnts.mu_pkt_cnt);
}

static void pcie_enable_data_tasks_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	enable_work(&pcie_priv->tx_work);
	napi_enable(&pcie_priv->napi);
	/* A disabled work item is not queued, unlike a disabled tasklet
	 * that runs once enabled, so look at the queues once now.
	 */
	pcie_schedule_tx(pcie_priv);
}

static void pcie_disable_data_tasks_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	disable_work_sync(&pcie_priv->tx_work);
	hrtimer_cancel(&pcie_priv->tx_poll_timer);
	napi_synchronize(&pcie_priv->napi);
	napi_disable(&pcie_priv->napi);
}

static irqreturn_t pcie_isr_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	u32 int_status;

	int_status = readl(pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);

	if (int_status == 0x00000000)
		return IRQ_NONE;

	if (int_status == 0xffffffff) {
		wiphy_warn(hw->wiphy, "card unplugged?\n");
	} else {
		writel(~int_status,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);

		if (int_status & MACREG_A2HRIC_ACNT_HEAD_RDY)
			ieee80211_queue_work(hw, &priv->account_handle);

		if (int_status & MACREG_A2HRIC_RX_DONE_HEAD_RDY) {
			if (napi_schedule_prep(&pcie_priv->napi)) {
				pcie_mask_int(pcie_priv,
					      MACREG_A2HRIC_RX_DONE_HEAD_RDY,
					      false);
				__napi_schedule(&pcie_priv->napi);
			}
		}

		if (int_status & MACREG_A2HRIC_NEWDP_DFS) {
			wiphy_info(hw->wiphy, "radar detected by firmware\n");
			ieee80211_radar_detected(hw, NULL);
		}

		if (int_status & MACREG_A2HRIC_NEWDP_CHANNEL_SWITCH)
			ieee80211_queue_work(hw, &priv->chnl_switch_handle);
	}

	return IRQ_HANDLED;
}

static void pcie_irq_enable_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	if (pcie_chk_adapter(pcie_priv)) {
		writel(0x00,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
		writel(MACREG_A2HRIC_BIT_MASK_NDP,
		       pcie_priv->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
	}
}

static void pcie_timer_routine_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;

	if (pcie_tx_pending_ndp(pcie_priv))
		pcie_schedule_tx(pcie_priv);

	if ((++pcie_priv->tx_done_timer_cnt * SYSADPT_TIMER_WAKEUP_TIME) >=
	    CHECK_TX_DONE_TIME) {
		pcie_tx_done_ndp(hw);
		pcie_priv->tx_done_timer_cnt = 0;
	}
}

static void pcie_tx_return_pkts_ndp(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	int i;

	pcie_tx_done_ndp(hw);

	/* Frames the firmware still holds are returned as it completes
	 * them, or on removal; only the queued ones can be dropped here.
	 */
	for (i = 0; i < SYSADPT_TX_WMM_QUEUES; i++)
		ieee80211_purge_tx_queue(hw, &pcie_priv->txq[i]);
}

static void pcie_set_sta_id(struct ieee80211_hw *hw,
			    struct ieee80211_sta *sta,
			    bool sta_mode, bool set)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct mwl_sta *sta_info;
	u16 stnid;

	sta_info = mwl_dev_get_sta(sta);
	stnid = sta_mode ? 0 : sta_info->stnid;
	rcu_assign_pointer(pcie_priv->sta_link[stnid], set ? sta : NULL);
}

static void pcie_tx_account(struct mwl_priv *priv,
			    struct mwl_sta *sta_info,
			    struct acnt_tx_s *acnt_tx)
{
	u32 rate_info, tx_cnt;
	u8 index, type, rate_ac, format, bw, gi, mcs, nss;
	u16 ratemask;
	u8 i, found;
	struct mwl_tx_hist *tx_hist;
	struct mwl_tx_hist_data *tx_hist_data;

	rate_info = le32_to_cpu(acnt_tx->tx_info.rate_info);
	tx_cnt = le32_to_cpu(acnt_tx->tx_cnt);
	index = acnt_tx->rate_tbl_index;
	type = acnt_tx->type;

	if (!rate_info)
		return;
	sta_info->tx_rate_info = rate_info;

	tx_hist = &sta_info->tx_hist;
	if (!tx_hist || (type >= SU_MU_TYPE_CNT))
		return;

	if ((type == SU_MIMO) && (index >= MAX_SUPPORTED_RATES) &&
	    (index != 0xFF))
		return;

	format = FIELD_GET(MWL_TX_RATE_FORMAT_MASK, rate_info);
	bw = FIELD_GET(MWL_TX_RATE_BANDWIDTH_MASK, rate_info);
	gi = FIELD_GET(MWL_TX_RATE_SHORTGI_MASK, rate_info);
	mcs = FIELD_GET(MWL_TX_RATE_RATEIDMCS_MASK, rate_info);

	tx_hist->cur_rate_info[type] = rate_info;

	/* Rate table index is valid */
	if (index != 0xff) {
		if (type == MU_MIMO) {
			rate_ac = mcs & 0xf;
			nss = mcs >> 4;
			if (nss < (QS_NUM_SUPPORTED_11AC_NSS - 1) &&
			    rate_ac < QS_NUM_SUPPORTED_11AC_MCS) {
				tx_hist_data =
					&tx_hist->mu_rate[nss][bw][gi][rate_ac];
				tx_hist_data->rateinfo = rate_info;
				tx_hist_data->cnt++;
				tx_hist->total_tx_cnt[type] += tx_cnt;
			}
		} else {
			/* If legacy, skip legacy preamble bit 15 */
			if (format == TX_RATE_FORMAT_LEGACY)
				ratemask = 0xfff;
			else
				ratemask = 0xffff;
			tx_hist_data = &tx_hist->su_rate[0];
			if ((tx_hist_data[index].rateinfo & ratemask) ==
			    (rate_info & ratemask)) {
				tx_hist_data[index].cnt++;
				tx_hist->total_tx_cnt[type] += tx_cnt;
			}
		}
	} else {
		if (type == MU_MIMO) {
			rate_ac = mcs & 0xf;
			nss = mcs >> 4;
			if (nss < (QS_NUM_SUPPORTED_11AC_NSS - 1) &&
			    rate_ac < QS_NUM_SUPPORTED_11AC_MCS) {
				tx_hist_data =
					&tx_hist->mu_rate[nss][bw][gi][rate_ac];
				tx_hist_data->rateinfo = rate_info;
				tx_hist_data->cnt++;
				tx_hist->total_tx_cnt[type] += tx_cnt;
			}
		} else {
			/* If legacy, skip legacy preamble bit 15 */
			if (format == TX_RATE_FORMAT_LEGACY)
				ratemask = 0xfff;
			else
				ratemask = 0xffff;
			tx_hist_data = &tx_hist->custom_rate[0];
			/* Go through non rate table buffer to see if any has
			 * been used. If all used up, recycle by using index 0
			 */
			found = 0;
			for (i = 0; i < TX_RATE_HISTO_CUSTOM_CNT; i++) {
				if (!tx_hist_data[i].rateinfo ||
				    ((tx_hist_data[i].rateinfo & ratemask) ==
				    (rate_info & ratemask))) {
					found = 1;
					break;
				}
			}
			if (found)
				index = i;
			else
				index = 0; /* reuse index 0 buffer */
			tx_hist_data[index].rateinfo = rate_info;
			tx_hist_data[index].cnt++;
			tx_hist->total_tx_cnt[type] += tx_cnt;
		}
	}
}

static void pcie_rx_account(struct mwl_priv *priv,
			    struct mwl_sta *sta_info,
			    struct acnt_rx_s *acnt_rx)
{
	u32 sig1, sig2, rate, param;
	u16 format, nss, bw, gi, rate_mcs;

	sig1 = FIELD_GET(RXINFO_HT_SIG1, le32_to_cpu(acnt_rx->rx_info.ht_sig1));
	sig2 = FIELD_GET(RXINFO_HT_SIG2,
			 le32_to_cpu(acnt_rx->rx_info.ht_sig2_rate));
	rate = FIELD_GET(RXINFO_RATE,
			 le32_to_cpu(acnt_rx->rx_info.ht_sig2_rate));
	param = FIELD_GET(RXINFO_PARAM, le32_to_cpu(acnt_rx->rx_info.param));

	format = FIELD_GET(RXINFO_PARAM_FORMAT, param);
	nss = 0;
	bw = RX_RATE_INFO_HT20;
	switch (format) {
	case RX_RATE_INFO_FORMAT_11A:
		rate_mcs = rate & 0xF;
		if (rate_mcs == 10)
			rate_mcs = 7; /* 12 Mbps */
		else
			rate_mcs = utils_get_rate_id(rate_mcs);
		gi = RX_RATE_INFO_SHORT_INTERVAL;
		if ((rate_mcs == 5) || (rate_mcs == 7) || (rate_mcs == 9))
			return;
		break;
	case RX_RATE_INFO_FORMAT_11B:
		/* The rate is the PLCP signal value in units of 100 kb/s
		 * (10, 20, 55, 110), not an OFDM rate code, so it must not
		 * be masked. 1 and 2 Mb/s are left out like the basic OFDM
		 * rates above.
		 */
		rate_mcs = utils_get_rate_id(rate);
		gi = RX_RATE_INFO_LONG_INTERVAL;
		if ((rate_mcs != 2) && (rate_mcs != 3))
			return;
		break;
	case RX_RATE_INFO_FORMAT_11N:
		if (FIELD_GET(HT_SIG1_MCS, sig1) >= 16)
			return;
		bw = FIELD_GET(HT_SIG1_CBW, sig1);
		gi = FIELD_GET(HT_SIG2_SGI, sig2);
		rate_mcs = FIELD_GET(HT_SIG1_MCS, sig1);
		if (rate_mcs > 76)
			return;
		break;
	case RX_RATE_INFO_FORMAT_11AC:
		if (FIELD_GET(VHT_SIG2_MCS, sig2) >= 10)
			return;
		nss = FIELD_GET(VHT_SIG1_NSS, sig1);
		if (!nss)
			return;
		bw = FIELD_GET(VHT_SIG1_BW, sig1);
		gi = FIELD_GET(VHT_SIG2_SGI, sig2);
		rate_mcs = FIELD_GET(VHT_SIG2_MCS, sig2);
		if (rate_mcs > 9)
			return;
		break;
	default:
		return;
	}

	sta_info->rx_format = format;
	sta_info->rx_nss = nss;
	sta_info->rx_bw = bw;
	sta_info->rx_gi = gi;
	sta_info->rx_rate_mcs = rate_mcs;
	sta_info->rx_signal = FIELD_GET(RXINFO_RSSI_X,
					le32_to_cpu(acnt_rx->rx_info.rssi_x));
}

static void pcie_tx_per(struct mwl_priv *priv, struct mwl_sta *sta_info,
			struct acnt_ra_s *acnt_ra)
{
	u32 rate_info;
	u8 index, per, type, rate_ac, per_index, format, bw, gi, mcs, nss;
	u16 ratemask;
	u8 i, found;
	struct mwl_tx_hist *tx_hist;
	struct mwl_tx_hist_data *tx_hist_data;

	rate_info = le32_to_cpu(acnt_ra->rate_info);
	index = acnt_ra->rate_tbl_index;
	per = acnt_ra->per;
	type = acnt_ra->type;

	tx_hist = &sta_info->tx_hist;

	if (!tx_hist || !rate_info || (type >= SU_MU_TYPE_CNT))
		return;

	if ((type == SU_MIMO) && (index >= MAX_SUPPORTED_RATES) &&
	    (index != 0xFF))
		return;

	if (per >= TX_HISTO_PER_THRES[3])
		per_index = 4;
	else if (per >= TX_HISTO_PER_THRES[2])
		per_index = 3;
	else if (per >= TX_HISTO_PER_THRES[1])
		per_index = 2;
	else if (per >= TX_HISTO_PER_THRES[0])
		per_index = 1;
	else
		per_index = 0;

	format = FIELD_GET(MWL_TX_RATE_FORMAT_MASK, rate_info);
	bw = FIELD_GET(MWL_TX_RATE_BANDWIDTH_MASK, rate_info);
	gi = FIELD_GET(MWL_TX_RATE_SHORTGI_MASK, rate_info);
	mcs = FIELD_GET(MWL_TX_RATE_RATEIDMCS_MASK, rate_info);

	/* Rate table index is valid */
	if (index != 0xff) {
		if (type == MU_MIMO) {
			rate_ac = mcs & 0xf;
			nss = mcs >> 4;
			if (nss < (QS_NUM_SUPPORTED_11AC_NSS - 1) &&
			    rate_ac < QS_NUM_SUPPORTED_11AC_MCS) {
				tx_hist_data =
					&tx_hist->mu_rate[nss][bw][gi][rate_ac];
				tx_hist_data->rateinfo = rate_info;
				tx_hist_data->per[per_index]++;
			}
		} else {
			/* If legacy, skip legacy preamble bit 15 */
			if (format == TX_RATE_FORMAT_LEGACY)
				ratemask = 0xfff;
			else
				ratemask = 0xffff;
			tx_hist_data = &tx_hist->su_rate[0];
			if ((tx_hist_data[index].rateinfo & ratemask) ==
			    (rate_info & ratemask))
				tx_hist_data[index].per[per_index]++;
		}
	} else {
		if (type == MU_MIMO) {
			rate_ac = mcs & 0xf;
			nss = mcs >> 4;
			if (nss < (QS_NUM_SUPPORTED_11AC_NSS - 1) &&
			    rate_ac < QS_NUM_SUPPORTED_11AC_MCS) {
				tx_hist_data =
					&tx_hist->mu_rate[nss][bw][gi][rate_ac];
				tx_hist_data->rateinfo = rate_info;
				tx_hist_data->per[per_index]++;
			}
		} else {
			/* If legacy, skip legacy preamble bit 15 */
			if (format == TX_RATE_FORMAT_LEGACY)
				ratemask = 0xfff;
			else
				ratemask = 0xffff;
			tx_hist_data = &tx_hist->custom_rate[0];
			/* Go through non rate table buffer to see if any has
			 * been used. If all used up, recycle by using index 0
			 */
			found = 0;
			for (i = 0; i < TX_RATE_HISTO_CUSTOM_CNT; i++) {
				if (!tx_hist_data[i].rateinfo ||
				    ((tx_hist_data[i].rateinfo & ratemask) ==
				    (rate_info & ratemask))) {
					found = 1;
					break;
				}
			}
			if (found)
				index = i;
			else
				index = 0; /* reuse index 0 buffer */
			tx_hist_data[index].rateinfo = rate_info;
			tx_hist_data[index].per[per_index]++;
		}
	}
}

static void pcie_ba_account(struct mwl_priv *priv,
			    struct mwl_sta *sta_info,
			    struct acnt_ba_s *acnt_ba)
{
	struct mwl_tx_ba_hist *ba_hist = &sta_info->ba_hist;

	if (sta_info->stnid != le16_to_cpu(acnt_ba->stnid))
		return;

	if (ba_hist->enable && ba_hist->ba_stats &&
	    (ba_hist->index < ACNT_BA_SIZE)) {
		ba_hist->type = acnt_ba->type;
		ba_hist->ba_stats[ba_hist->index].ba_hole = acnt_ba->ba_hole;
		ba_hist->ba_stats[ba_hist->index].ba_expected =
			acnt_ba->ba_expected;
		ba_hist->ba_stats[ba_hist->index].no_ba = acnt_ba->no_ba;
		ba_hist->index++;
		if (ba_hist->index == ACNT_BA_SIZE)
			wiphy_info(priv->hw->wiphy,
				   "Aid:%d BA histo collection done\n",
				   priv->ba_aid);
	}
}

static void pcie_bf_mimo_ctrl_decode(struct mwl_priv *priv,
				     struct acnt_bf_mimo_ctrl_s *bf_mimo_ctrl)
{
	wiphy_dbg(priv->hw->wiphy,
		  "BF MIMO ctrl: MAC %pM %s MIMO_Ctrl_Field 0x%x Comp_BF_Report_First_8Bytes 0x%llx\n",
		  bf_mimo_ctrl->rec_mac, bf_mimo_ctrl->type ? "MU" : "SU",
		  le32_to_cpu(bf_mimo_ctrl->mimo_ctrl),
		  le64_to_cpu(bf_mimo_ctrl->comp_bf_rep));
}

/* Smallest record length the decoder below reads for each record type. */
static size_t pcie_acnt_min_len(u16 code)
{
	switch (code) {
	case ACNT_CODE_TX_ENQUEUE:
		return sizeof(struct acnt_tx_s);
	case ACNT_CODE_RX_PPDU:
		return sizeof(struct acnt_rx_s) + sizeof(struct pcie_dma_data);
	case ACNT_CODE_RA_STATS:
		return sizeof(struct acnt_ra_s);
	case ACNT_CODE_BA_STATS:
		return sizeof(struct acnt_ba_s);
	case ACNT_CODE_BF_MIMO_CTRL:
		return sizeof(struct acnt_bf_mimo_ctrl_s);
	default:
		return sizeof(struct acnt_s);
	}
}

static void pcie_process_account(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	struct pcie_desc_data_ndp *desc = &pcie_priv->desc_data_ndp;
	u32 acnt_head, acnt_tail;
	u32 read_size;
	size_t rec_len;
	u16 code;
	u8 *acnt_recds;
	u8 *pstart, *pend;
	struct acnt_s *acnt;
	struct acnt_tx_s *acnt_tx;
	struct acnt_rx_s *acnt_rx;
	struct acnt_ra_s *acnt_ra;
	struct acnt_ba_s *acnt_ba;
	struct acnt_bf_mimo_ctrl_s *acnt_bf_mimo_ctrl;
	struct pcie_dma_data *dma_data;
	struct mwl_sta *sta_info;
	u16 nf_a, nf_b, nf_c, nf_d;
	u16 stnid;
	u32 attempts;
	u8 type;

	acnt_head = readl(pcie_priv->iobase1 + MACREG_REG_ACNTHEAD);
	acnt_tail = readl(pcie_priv->iobase1 + MACREG_REG_ACNTTAIL);

	if (acnt_tail == acnt_head)
		return;

	if (acnt_head > desc->acnt_ring_size ||
	    acnt_tail > desc->acnt_ring_size) {
		wiphy_err_ratelimited(hw->wiphy,
				      "account ring index out of range (%u %u)\n",
				      acnt_head, acnt_tail);
		return;
	}

	if (acnt_tail > acnt_head) {
		read_size = desc->acnt_ring_size - acnt_tail + acnt_head;
		if (read_size > desc->acnt_ring_size) {
			wiphy_err(hw->wiphy,
				  "account size overflow (%d %d %d)\n",
				  acnt_head, acnt_tail, read_size);
			goto process_next;
		}
		memcpy(desc->pacnt_buf, desc->pacnt_ring + acnt_tail,
		       desc->acnt_ring_size - acnt_tail);
		memcpy(desc->pacnt_buf + desc->acnt_ring_size - acnt_tail,
		       desc->pacnt_ring, acnt_head);
		acnt_recds = desc->pacnt_buf;
	} else {
		read_size = acnt_head - acnt_tail;
		if (read_size > desc->acnt_ring_size) {
			wiphy_err(hw->wiphy,
				  "account size overflow (%d %d %d)\n",
				  acnt_head, acnt_tail, read_size);
			goto process_next;
		}
		acnt_recds = desc->pacnt_ring + acnt_tail;
	}

	pstart = acnt_recds;
	pend = pstart + read_size;
	while (pend - pstart >= sizeof(struct acnt_s)) {
		acnt = (struct acnt_s *)pstart;
		code = le16_to_cpu(acnt->code);
		rec_len = acnt->len * 4;

		if (!rec_len)
			break;

		if (rec_len > pend - pstart || rec_len < pcie_acnt_min_len(code)) {
			wiphy_warn_ratelimited(hw->wiphy,
					       "corrupt accounting record (code 0x%x len %u)\n",
					       code, acnt->len);
			break;
		}

		switch (code) {
		case ACNT_CODE_BUSY:
			pcie_priv->acnt_busy++;
			break;
		case ACNT_CODE_WRAP:
			pcie_priv->acnt_wrap++;
			break;
		case ACNT_CODE_DROP:
			pcie_priv->acnt_drop++;
			break;
		case ACNT_CODE_TX_ENQUEUE:
			acnt_tx = (struct acnt_tx_s *)pstart;
			scoped_guard(spinlock_bh, &priv->sta_lock) {
				sta_info = utils_find_sta(priv,
							  acnt_tx->hdr.wh.addr1);
				if (sta_info)
					pcie_tx_account(priv, sta_info, acnt_tx);
			}
			break;
		case ACNT_CODE_RX_PPDU:
			acnt_rx = (struct acnt_rx_s *)pstart;
			nf_a = FIELD_GET(RXINFO_NF_A,
					 le32_to_cpu(acnt_rx->rx_info.nf_a_b));
			nf_b = FIELD_GET(RXINFO_NF_B,
					 le32_to_cpu(acnt_rx->rx_info.nf_a_b));
			nf_c = FIELD_GET(RXINFO_NF_C,
					 le32_to_cpu(acnt_rx->rx_info.nf_c_d));
			nf_d = FIELD_GET(RXINFO_NF_D,
					 le32_to_cpu(acnt_rx->rx_info.nf_c_d));
			if ((nf_a >= 2048) && (nf_b >= 2048) &&
			    (nf_c >= 2048) && (nf_d >= 2048)) {
				nf_a = ((4096 - nf_a) >> 4);
				nf_b = ((4096 - nf_b) >> 4);
				nf_c = ((4096 - nf_c) >> 4);
				nf_d = ((4096 - nf_d) >> 4);
				priv->noise =
					-((nf_a + nf_b + nf_c + nf_d) / 4);
			}
			dma_data = (struct pcie_dma_data *)
				&acnt_rx->rx_info.hdr[0];
			scoped_guard(spinlock_bh, &priv->sta_lock) {
				sta_info = utils_find_sta(priv,
							  dma_data->wh.addr2);
				if (sta_info)
					pcie_rx_account(priv, sta_info, acnt_rx);
			}
			break;
		case ACNT_CODE_RA_STATS:
			acnt_ra = (struct acnt_ra_s *)pstart;
			stnid = le16_to_cpu(acnt_ra->stn_id);
			if ((stnid > 0) && (stnid <= priv->stnid_num)) {
				type = acnt_ra->type;
				attempts = le32_to_cpu(acnt_ra->tx_attempt_cnt);
				if (type < 2) {
					if (attempts >= 250)
						priv->ra_tx_attempt[type][5]++;
					else if (attempts >= 100)
						priv->ra_tx_attempt[type][4]++;
					else if (attempts >= 50)
						priv->ra_tx_attempt[type][3]++;
					else if (attempts >= 15)
						priv->ra_tx_attempt[type][2]++;
					else if (attempts >= 4)
						priv->ra_tx_attempt[type][1]++;
					else
						priv->ra_tx_attempt[type][0]++;
				}
				scoped_guard(spinlock_bh, &priv->sta_lock) {
					sta_info = utils_find_sta_by_id(priv,
									stnid);
					if (sta_info)
						pcie_tx_per(priv, sta_info,
							    acnt_ra);
				}
			}
			break;
		case ACNT_CODE_BA_STATS:
			acnt_ba = (struct acnt_ba_s *)pstart;
			if (priv->ba_aid) {
				scoped_guard(spinlock_bh, &priv->sta_lock) {
					sta_info = utils_find_sta_by_aid(priv,
									 priv->ba_aid);
					if (sta_info)
						pcie_ba_account(priv, sta_info,
								acnt_ba);
				}
			}
			break;
		case ACNT_CODE_BF_MIMO_CTRL:
			acnt_bf_mimo_ctrl =
				(struct acnt_bf_mimo_ctrl_s *)pstart;
			pcie_bf_mimo_ctrl_decode(priv, acnt_bf_mimo_ctrl);
			break;
		default:
			break;
		}

		pstart += rec_len;
	}
process_next:
	acnt_tail = acnt_head;
	writel(acnt_tail, pcie_priv->iobase1 + MACREG_REG_ACNTTAIL);
}

static int pcie_mcast_cts(struct ieee80211_hw *hw, bool enable)
{
	return mwl_fwcmd_mcast_cts(hw, enable ? 1 : 0);
}

static const struct mwl_hif_ops pcie_hif_ops_ndp = {
	.driver_name           = PCIE_DRV_NAME,
	.driver_version        = PCIE_DRV_VERSION,
	.tx_head_room          = PCIE_MIN_BYTES_HEADROOM,
	.ampdu_num             = AMPDU_QUEUES_NDP,
	.reset                 = pcie_reset,
	.init                  = pcie_init_ndp,
	.deinit                = pcie_deinit_ndp,
	.get_info              = pcie_get_info_ndp,
	.get_tx_status         = pcie_get_tx_status_ndp,
	.get_rx_status         = pcie_get_rx_status_ndp,
	.enable_data_tasks     = pcie_enable_data_tasks_ndp,
	.disable_data_tasks    = pcie_disable_data_tasks_ndp,
	.exec_cmd              = pcie_exec_cmd,
	.get_irq_num           = pcie_get_irq_num,
	.irq_handler           = pcie_isr_ndp,
	.irq_enable            = pcie_irq_enable_ndp,
	.irq_disable           = pcie_irq_disable,
	.download_firmware     = pcie_download_firmware,
	.timer_routine         = pcie_timer_routine_ndp,
	.tx_xmit               = pcie_tx_xmit_ndp,
	.tx_return_pkts        = pcie_tx_return_pkts_ndp,
	.wake_tx_queue         = pcie_wake_tx_queue_ndp,
	.get_survey            = pcie_get_survey,
	.set_sta_id            = pcie_set_sta_id,
	.process_account       = pcie_process_account,
	.mcast_cts             = pcie_mcast_cts,
};

static int pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ieee80211_hw *hw;
	struct mwl_priv *priv;
	struct pcie_priv *pcie_priv;
	const struct mwl_hif_ops *hif_ops;
	int rc = 0;

	if (id->driver_data >= MWLUNKNOWN)
		return -ENODEV;

	rc = pcim_enable_device(pdev);
	if (rc)
		return dev_err_probe(&pdev->dev, rc, "cannot enable device\n");

	rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (rc)
		return dev_err_probe(&pdev->dev, rc,
				     "32-bit DMA not supported\n");

	pci_set_master(pdev);

	if (id->driver_data == MWL8964)
		hif_ops = &pcie_hif_ops_ndp;
	else if (id->driver_data == MWL8997)
		hif_ops = &pcie_hif_ops_8997;
	else
		hif_ops = &pcie_hif_ops_8864;

	hw = mwl_alloc_hw(MWL_BUS_PCIE, id->driver_data, &pdev->dev,
			  hif_ops, sizeof(*pcie_priv));
	if (!hw)
		return -ENOMEM;

	pci_set_drvdata(pdev, hw);

	priv = hw->priv;
	priv->antenna_tx = pcie_chip_tbl[priv->chip_type].antenna_tx;
	priv->antenna_rx = pcie_chip_tbl[priv->chip_type].antenna_rx;
	pcie_priv = priv->hif.priv;

	pcie_priv->pdev = pdev;
	if (id->driver_data != MWL8964) {
		pcie_priv->tx_head_room = PCIE_MIN_BYTES_HEADROOM;
		if (id->driver_data == MWL8997) {
			if (NET_SKB_PAD < PCIE_MIN_TX_HEADROOM_KF2) {
				pcie_priv->tx_head_room =
					PCIE_MIN_TX_HEADROOM_KF2;
				pcie_hif_ops_8997.tx_head_room =
					PCIE_MIN_TX_HEADROOM_KF2;
			}
		}
	}

	rc = pcie_alloc_resource(pcie_priv);
	if (rc)
		goto err_alloc_pci_resource;

	rc = mwl_init_hw(hw, pcie_chip_tbl[priv->chip_type].fw_image,
			 pcie_chip_tbl[priv->chip_type].cal_file,
			 pcie_chip_tbl[priv->chip_type].txpwrlmt_file);
	if (rc)
		goto err_wl_init;

	return rc;

err_wl_init:

	pcie_reset(hw);

err_alloc_pci_resource:

	pci_set_drvdata(pdev, NULL);
	mwl_free_hw(hw);

	return rc;
}

static void pcie_remove(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct mwl_priv *priv = hw->priv;

	/* mac80211 still removes keys and stations while it unregisters,
	 * so only cut commands short now if the firmware has stopped
	 * answering; mwl_wl_deinit() does it once mac80211 is done.
	 */
	if (priv->cmd_timeout) {
		priv->rmmod = true;
		while (priv->in_send_cmd)
			usleep_range(1000, 2000);
	}
	mwl_deinit_hw(hw);
	pci_set_drvdata(pdev, NULL);
	mwl_free_hw(hw);
}

static struct pci_driver mwl_pcie_driver = {
	.name     = PCIE_DRV_NAME,
	.id_table = pcie_id_tbl,
	.probe    = pcie_probe,
	.remove   = pcie_remove
};


module_pci_driver(mwl_pcie_driver);

MODULE_DESCRIPTION(PCIE_DRV_DESC);
MODULE_VERSION(PCIE_DRV_VERSION);
MODULE_AUTHOR("Marvell Semiconductor, Inc.");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(pci, pcie_id_tbl);
