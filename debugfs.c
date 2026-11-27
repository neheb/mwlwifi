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

/* Description:  This file implements debug fs related functions. */

#include <linux/debugfs.h>
#include <linux/devcoredump.h>
#include <linux/etherdevice.h>
#include <linux/seq_file.h>

#include "sysadpt.h"
#include "compat.h"
#include "core.h"
#include "utils.h"
#include "thermal.h"
#include "hif/fwcmd.h"
#include "hif/pcie/dev.h"
#include "hif/hif-ops.h"
#include "debugfs.h"

#define MWLWIFI_DEBUGFS_ADD_FILE(name) \
	debugfs_create_file(#name, 0644, priv->debugfs_phy, priv, \
			    &mwl_debugfs_##name##_fops)

/* Write-only files */
#define MWLWIFI_DEBUGFS_FILE_WRITE_OPS(name) \
static const struct file_operations mwl_debugfs_##name##_fops = { \
	.write = mwl_debugfs_##name##_write, \
	.open = simple_open, \
}

/* Valid for seq_file and simple_open files alike */
static struct mwl_priv *mwl_debugfs_priv(struct file *file)
{
	return file_inode(file)->i_private;
}

static const char chipname[MWLUNKNOWN][8] = {
	"88W8864",
	"88W8897",
	"88W8964",
	"88W8997"
};

static void dump_data(struct seq_file *s, u8 *data, int data_len,
		      char *title)
{
	int i;

	seq_printf(s, "%s\n", title);
	for (i = 0; i < data_len; i++)
		seq_printf(s, "0x%02x%c", data[i],
			   (i % 8 == 7 || i == data_len - 1) ? '\n' : ' ');
}

static void _dump_tx_hist_mu(struct seq_file *s, bool *printed,
			     u32 *total, u8 nss, u8 bw, u8 mcs, u8 sgi,
			     struct mwl_sta *sta_info)
{
	char *bw_str[4] = {"ht20", "ht40", "ht80", "ht160"};
	char *sgi_str[2] = {"lgi", "sgi"};
	struct mwl_tx_hist_data *tx_hist_data;
	u32 cnt, rateinfo, per0, per1, per2, per3, per4, ratemask;

	tx_hist_data = &sta_info->tx_hist.mu_rate[nss][bw][sgi][mcs];
	cnt = tx_hist_data->cnt;
	rateinfo = tx_hist_data->rateinfo;
	if (cnt && (rateinfo > 0)) {
		*total += cnt;
		per4 = tx_hist_data->per[4];
		per3 = tx_hist_data->per[3];
		per2 = tx_hist_data->per[2];
		per1 = tx_hist_data->per[1];
		per0 = tx_hist_data->per[0];
		if (!*printed) {
			seq_printf(s, "%s %26s  <%2d %8s%2d%8s%2d%8s%2d%8s%2d\n",
				   "MU_MIMO rate", " PER%", TX_HISTO_PER_THRES[0],
				   ">=", TX_HISTO_PER_THRES[0],
				   ">=", TX_HISTO_PER_THRES[1],
				   ">=", TX_HISTO_PER_THRES[2],
				   ">=", TX_HISTO_PER_THRES[3]);
			seq_printf(s, "TOTAL MPDU tx pkt: %d\n",
				   sta_info->tx_hist.total_tx_cnt[MU_MIMO]);
				*printed = true;
		}
		if ((rateinfo & 0x3) == 0)
			ratemask = 0xfff;
		else
			ratemask = 0xffff;
		if ((sta_info->tx_hist.cur_rate_info[MU_MIMO] & ratemask) ==
		    (rateinfo & ratemask))
			 /* mark as current rate */
			seq_puts(s, "*");
		else
			seq_puts(s, " ");
		seq_printf(s, "%5s_%3s_%1dSS_MCS%2d: %10u, %9d, %9d, %9d, %9d, %9d\n",
			   bw_str[bw], sgi_str[sgi], (nss + 1), mcs, cnt, per0,
			   per1, per2, per3, per4);
	}
}

static void dump_tx_hist_mu(struct seq_file *s, bool *printed,
			    u32 *total, struct mwl_sta *sta_info)
{
	u8 nss, bw, mcs, sgi;

	for (nss = 0; nss < (QS_NUM_SUPPORTED_11AC_NSS - 1); nss++) {
		for (bw = 0; bw < QS_NUM_SUPPORTED_11AC_BW; bw++) {
			for (mcs = 0; mcs < QS_NUM_SUPPORTED_11AC_MCS; mcs++) {
				for (sgi = 0; sgi < QS_NUM_SUPPORTED_GI;
				     sgi++) {
					_dump_tx_hist_mu(s, printed,
							 total, nss, bw, mcs,
							 sgi, sta_info);
				}
			}
		}
	}
}


static void dump_tx_hist_su(struct seq_file *s, bool su, bool *printed,
			    u32 *total, struct mwl_sta *sta_info)
{
	int g_rate[14] = {1, 2, 5, 11, 22, 6, 9, 12, 18, 24, 36, 48, 54, 72};
	char *bw_str[4] = {"ht20", "ht40", "ht80", "ht160"};
	char *sgi_str[2] = {"lgi", "sgi"};
	char title_str[32];
	struct mwl_tx_hist *tx_hist;
	struct mwl_tx_hist_data *tx_hist_data;
	u32 j, loopcnt;
	u32 cnt, rateinfo, per0, per1, per2, per3, per4, ratemask;
	u8 format, bw, sgi, mcs, nss;

	tx_hist = &sta_info->tx_hist;
	if (su) {
		loopcnt = MAX_SUPPORTED_RATES;
		tx_hist_data = &tx_hist->su_rate[0];
	} else {
		loopcnt = TX_RATE_HISTO_CUSTOM_CNT;
		tx_hist_data = &tx_hist->custom_rate[0];
	}

	for (j = 0; j < loopcnt; j++) {
		cnt = tx_hist_data[j].cnt;
		rateinfo = tx_hist_data[j].rateinfo;
		if (cnt && (rateinfo > 0)) {
			*total += cnt;
			per4 = tx_hist_data[j].per[4];
			per3 = tx_hist_data[j].per[3];
			per2 = tx_hist_data[j].per[2];
			per1 = tx_hist_data[j].per[1];
			per0 = tx_hist_data[j].per[0];
			if (!*printed) {
				seq_printf(s, "%s %26s  <%2d %8s%2d%8s%2d%8s%2d%8s%2d\n",
					   su ? "SU_MIMO rate" : " Custom rate",
					   " PER%", TX_HISTO_PER_THRES[0],
					   ">=", TX_HISTO_PER_THRES[0],
					   ">=", TX_HISTO_PER_THRES[1],
					   ">=", TX_HISTO_PER_THRES[2],
					   ">=", TX_HISTO_PER_THRES[3]);
			seq_printf(s, "TOTAL MPDU tx pkt: %d\n",
				   tx_hist->total_tx_cnt[SU_MIMO]);
				*printed = true;
			}
			format = FIELD_GET(MWL_TX_RATE_FORMAT_MASK, rateinfo);
			bw = FIELD_GET(MWL_TX_RATE_BANDWIDTH_MASK, rateinfo);
			sgi = FIELD_GET(MWL_TX_RATE_SHORTGI_MASK, rateinfo);
			mcs = FIELD_GET(MWL_TX_RATE_RATEIDMCS_MASK, rateinfo);
			if (format == TX_RATE_FORMAT_LEGACY)
				ratemask = 0xfff;
			else
				ratemask = 0xffff;
			if ((tx_hist->cur_rate_info[SU_MIMO] & ratemask) ==
			    (rateinfo & ratemask))
				/* mark as current rate */
				seq_puts(s, "*");
			else
				seq_puts(s, " ");
			if (format == TX_RATE_FORMAT_LEGACY) {
				if (mcs == 2) {
					seq_printf(s, "%s %10u, %9d, %9d, %9d, %9d, %9d\n",
						   "5.5Mbps             :", cnt, per0,
						   per1, per2, per3, per4);
				} else if (mcs < ARRAY_SIZE(g_rate)) {
					scnprintf(title_str, sizeof(title_str),
						  "%-3dMbps             :",
						  g_rate[mcs]);
					seq_printf(s, "%s %10u, %9d, %9d, %9d, %9d, %9d\n",
						   title_str, cnt, per0, per1, per2, per3,
						   per4);
				} else {
					seq_printf(s, "rate id %-3u         : %10u, %9d, %9d, %9d, %9d, %9d\n",
						   mcs, cnt, per0, per1, per2, per3, per4);
				}
			} else if (format ==  TX_RATE_FORMAT_11N) {
				scnprintf(title_str, sizeof(title_str),
					  "%4s_%3s_MCS%2d	    :",
					  bw_str[bw], sgi_str[sgi], mcs);
				seq_printf(s, "%s %10u, %9d, %9d, %9d, %9d, %9d\n",
					   title_str, cnt, per0, per1, per2, per3,
					   per4);
			} else {
				nss = (mcs >> 4);
				scnprintf(title_str, sizeof(title_str),
					  "%5s_%3s_%1dSS_MCS%2d :",
					  bw_str[bw], sgi_str[sgi], (nss + 1),
					  (mcs & 0xf));
				seq_printf(s, "%s %10u, %9d, %9d, %9d, %9d, %9d\n",
					   title_str, cnt, per0, per1, per2, per3,
					   per4);
			}
		}
	}
}

static void dump_tx_hist(struct seq_file *s, struct mwl_sta *sta_info)
{
	int type;
	bool printed, su;
	u32 total;

	for (type = 0; type <= SU_MU_TYPE_CNT; type++) {
		printed = false;
		total = 0;
		if (type == MU_MIMO) {
			dump_tx_hist_mu(s, &printed,
					&total, sta_info);
		} else {
			su = (type == SU_MIMO) ? true : false;
			dump_tx_hist_su(s, su, &printed,
					&total, sta_info);
		}
		if (printed)
			seq_printf(s, "  TOTAL              : %10u\n\n",
				   total);
	}
}

static int mwl_debugfs_info_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct pcie_priv *pcie_priv = priv->hif.priv;
	unsigned long flags;
	int tx_num = 4, rx_num = 4;
	/* Asking the firmware again stops the 88W8964 beaconing, so print
	 * what it reported at probe time.
	 */
	const struct hostcmd_get_hw_spec *get_hw_spec = &priv->hw_spec;
	int i;

	seq_printf(s, "host_if: %d\n", get_hw_spec->host_if);
	seq_printf(s, "num_antenna: %d\n", le16_to_cpu(get_hw_spec->num_antenna));
	seq_printf(s, "region_code: %d\n", le16_to_cpu(get_hw_spec->region_code));
	seq_printf(s, "num_wcb: %d\n", le16_to_cpu(get_hw_spec->num_wcb));

	if (priv->chip_type == MWL8864) {
		seq_puts(s, "-------------  address|qlen|fw_desc_cnt\n");
		spin_lock_irqsave(&pcie_priv->tx_desc_lock, flags);
		seq_printf(s, "wcb_base0   : %8x|%4d|%d\n",
			   le32_to_cpu(get_hw_spec->wcb_base0),
			   skb_queue_len(&pcie_priv->txq[0]),
			   pcie_priv->fw_desc_cnt[0]);
		for(i = 0; i < SYSADPT_TOTAL_TX_QUEUES - 1; i++)
			seq_printf(s, "wcb_base[%2d]: %8x|%4d|%d\n", i,
				   le32_to_cpu(get_hw_spec->wcb_base[i]),
				   skb_queue_len(&pcie_priv->txq[i + 1]),
				   pcie_priv->fw_desc_cnt[i + 1]);
		spin_unlock_irqrestore(&pcie_priv->tx_desc_lock, flags);
	}

	seq_printf(s, "num_mcast_addr: %X\n", le16_to_cpu(get_hw_spec->num_mcast_addr));
	seq_printf(s, "permanent mac address: %pM\n", get_hw_spec->permanent_addr);
	seq_printf(s, "fw_awake_cookie: %d\n", le32_to_cpu(get_hw_spec->fw_awake_cookie));
	seq_printf(s, "driver name: %s\n",
		   mwl_hif_get_driver_name(priv->hw));
	seq_printf(s, "chip type: %s\n",
		   chipname[priv->chip_type]);
	seq_printf(s, "hw version: %X\n", priv->hw_data.hw_version);
	seq_printf(s, "driver version: %s\n",
		   mwl_hif_get_driver_version(priv->hw));
	seq_printf(s, "firmware version: 0x%08x\n",
		   priv->hw_data.fw_release_num);
	seq_printf(s, "power table loaded from dts: %s\n",
		   priv->forbidden_setting ? "no" : "yes");
	seq_printf(s, "firmware region code: 0x%x\n",
		   priv->fw_region_code);
	seq_printf(s, "mac address: %pM\n", priv->hw_data.mac_addr);
	seq_printf(s, "2g: %s\n", priv->disable_2g ? "disable" : "enable");
	seq_printf(s, "5g: %s\n", priv->disable_5g ? "disable" : "enable");
	if (priv->antenna_tx == ANTENNA_TX_2)
		tx_num = 2;
	else if (priv->antenna_tx == ANTENNA_TX_3)
		tx_num = 3;
	if (priv->antenna_rx == ANTENNA_RX_2)
		rx_num = 2;
	else if (priv->antenna_rx == ANTENNA_RX_3)
		rx_num = 3;
	seq_printf(s, "antenna: %d %d\n",
		   tx_num, rx_num);
	seq_printf(s, "irq number: %d\n", priv->irq);
	seq_printf(s, "ap macid support: %08x\n",
		   priv->ap_macids_supported);
	seq_printf(s, "sta macid support: %08x\n",
		   priv->sta_macids_supported);
	seq_printf(s, "macid used: %08x\n", priv->macids_used);
	seq_printf(s, "radio: %s\n", priv->radio_on ? "enable" : "disable");
	seq_printf(s, "radio_short_preamble: %s\n", priv->radio_short_preamble ? "enable" : "disable");
	seq_printf(s, "use_short_preamble: %s\n", priv->use_short_preamble ? "enable" : "disable");
	mwl_hif_get_info(priv->hw, s);

	return 0;
}

static int mwl_debugfs_rx_decrypt_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	seq_printf(s, "%5s\n", priv->rx_decrypt ? "true" : "false");


	return 0;

}

static ssize_t mwl_debugfs_rx_decrypt_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int value;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &value);
	if (ret)
		return ret;

	priv->rx_decrypt = value ? true : false;

	return count;
}

static int mwl_debugfs_tx_status_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	mwl_hif_get_tx_status(priv->hw, s);


	return 0;
}

static int mwl_debugfs_rx_status_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	mwl_hif_get_rx_status(priv->hw, s);


	return 0;
}

static int mwl_debugfs_vif_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_vif *mwl_vif;
	struct ieee80211_vif *vif;
	char ssid[IEEE80211_MAX_SSID_LEN + 1];
	struct cfg80211_chan_def *chan_def;
	struct beacon_info *beacon_info;

	guard(spinlock_bh)(&priv->vif_lock);
	list_for_each_entry(mwl_vif, &priv->vif_list, list) {
		vif = container_of((void *)mwl_vif, struct ieee80211_vif,
				   drv_priv);
		seq_printf(s, "macid: %d\n", mwl_vif->macid);
		switch (vif->type) {
		case NL80211_IFTYPE_AP:
			seq_puts(s, "type: ap\n");
			memcpy(ssid, vif->cfg.ssid,
			       vif->cfg.ssid_len);
			ssid[vif->cfg.ssid_len] = 0;
			seq_printf(s, "ssid: %s\n", ssid);
			seq_printf(s, "mac address: %pM\n", mwl_vif->bssid);
			break;
		case NL80211_IFTYPE_MESH_POINT:
			seq_puts(s, "type: mesh\n");
			seq_printf(s, "mac address: %pM\n", mwl_vif->bssid);
			break;
		case NL80211_IFTYPE_STATION:
			seq_puts(s, "type: sta\n");
			seq_printf(s, "mac address: %pM\n",
				   mwl_vif->sta_mac);
			break;
		default:
			seq_puts(s, "type: unknown\n");
			break;
		}
		if (vif->bss_conf.chanreq.oper.chan) {
			chan_def = &vif->bss_conf.chanreq.oper;
			seq_printf(s, "channel: %d: width: %d\n",
				   chan_def->chan->hw_value,
				   chan_def->width);
			seq_printf(s, "freq: %d freq1: %d freq2: %d\n",
				   chan_def->chan->center_freq,
				   chan_def->center_freq1,
				   chan_def->center_freq2);
		}
		seq_printf(s, "hw_crypto_enabled: %s\n",
			   mwl_vif->is_hw_crypto_enabled ?
			   "true" : "false");
		beacon_info = &mwl_vif->beacon_info;
		dump_data(s, beacon_info->ie_wmm_ptr,
			  beacon_info->ie_wmm_len, "WMM:");
		dump_data(s, beacon_info->ie_wpa_ptr,
			  beacon_info->ie_wpa_len, "WPA:");
		dump_data(s, beacon_info->ie_rsn48_ptr,
			  beacon_info->ie_rsn48_len, "RSN48:");
		dump_data(s, beacon_info->ie_mde_ptr,
			  beacon_info->ie_mde_len, "MDE:");
		/* ie_ht_ptr and ie_vht_ptr are write cursors while a beacon
		 * is parsed, so dump the lists themselves.
		 */
		dump_data(s, beacon_info->ie_list_ht,
			  min_t(int, beacon_info->ie_ht_len,
				sizeof(beacon_info->ie_list_ht)), "HT:");
		dump_data(s, beacon_info->ie_list_vht,
			  min_t(int, beacon_info->ie_vht_len,
				sizeof(beacon_info->ie_list_vht)), "VHT:");
		if (vif->type == NL80211_IFTYPE_MESH_POINT) {
			dump_data(s, beacon_info->ie_meshid_ptr,
				  beacon_info->ie_meshid_len, "MESHID:");
			dump_data(s, beacon_info->ie_meshcfg_ptr,
				  beacon_info->ie_meshcfg_len, "MESHCFG:");
			dump_data(s, beacon_info->ie_meshchsw_ptr,
				  beacon_info->ie_meshchsw_len, "MESHCHSW:");
		}
	}


	return 0;
}

static int mwl_debugfs_sta_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_sta *sta_info;
	struct ieee80211_sta *sta;

	seq_puts(s, "       macaddress|aid|ampdu|amsdu|  wds|ba_hist|amsdu_cap|  ht cap, ampdu,         rx_mask|   vht_cap|           mcs|rx bw,nss|tdls|init|wme|mfp\n");

	guard(spinlock_bh)(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		sta = container_of((void *)sta_info, struct ieee80211_sta,
				   drv_priv);

		seq_printf(s, "%pM|%3d|%5s|%5s|%5s|%7s|     0x%02x|0x%06x, %02x-%02x,0x%02x, %02x, %02x, %02x|0x%08x|0x%04x, 0x%04x|rx %2d, %2d|%4d|%4d|%3d|%3d\n",
			   sta->addr,
			   sta->aid,
			   sta_info->is_ampdu_allowed ? "true" : "false",
			   sta_info->is_amsdu_allowed ? "true" : "false",
			   sta_info->wds ? "true" : "false",
			   sta_info->ba_hist.enable ? "enable" : "disable",
			   sta_info->is_amsdu_allowed ? sta_info->amsdu_ctrl.cap : 0 ,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.cap : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.ampdu_factor : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.ampdu_density : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.mcs.rx_mask[0] : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.mcs.rx_mask[1] : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.mcs.rx_mask[2] : 0,
			   sta->deflink.ht_cap.ht_supported ? sta->deflink.ht_cap.mcs.rx_mask[3] : 0,
			   sta->deflink.vht_cap.vht_supported ? sta->deflink.vht_cap.cap : 0,
			   sta->deflink.vht_cap.vht_supported ? sta->deflink.vht_cap.vht_mcs.rx_mcs_map : 0,
			   sta->deflink.vht_cap.vht_supported ? sta->deflink.vht_cap.vht_mcs.tx_mcs_map : 0,
			   sta->deflink.bandwidth,
			   sta->deflink.rx_nss,
			   sta->tdls,
			   sta->tdls_initiator,
			   sta->wme,
			   sta->mfp
			   );
	}


	return 0;
}

static int mwl_debugfs_ampdu_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_ampdu_stream *stream;
	int i;
	struct mwl_sta *sta_info;
	struct ieee80211_sta *sta;
	struct mwl_tx_info *tx_stats;

	if (priv->chip_type == MWL8964)
		return -EPERM;

	seq_printf(s, "debug_ampdu: %d\n", priv->debug_ampdu);
	seq_puts(s, "=================\n");

	spin_lock_bh(&priv->stream_lock);
	seq_puts(s, "stream|idx|state|       macaddress|tid|pkts|remain|start time\n");
	for (i = 0; i < priv->ampdu_num; i++) {
		stream = &priv->ampdu[i];
		if (!stream->state)
			continue;

		if (stream->sta) {
			sta_info = mwl_dev_get_sta(stream->sta);
			tx_stats = &sta_info->tx_stats[stream->tid];
			seq_printf(s, "%6d|%3u|%5u|%pM|%3u|%4u|%6lu|%lu\n", i,
				   stream->idx,
				   stream->state,
				   stream->sta->addr,
				   stream->tid,
				   tx_stats->pkts,
				   (SYSADPT_TIMER_AMPDU_KEEPALIVE - (jiffies - stream->jiffies))/ HZ,
				   abs(jiffies - stream->start_time) / HZ
				   );

		}
	}
	spin_unlock_bh(&priv->stream_lock);
	guard(spinlock_bh)(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		for (i = 0; i < MWL_MAX_TID; i++) {
			if (sta_info->check_ba_failed[i]) {
				sta = container_of((void *)sta_info,
						   struct ieee80211_sta,
						   drv_priv);
				seq_printf(s, "%pM(%d): %d\n",
					   sta->addr, i,
					   sta_info->check_ba_failed[i]);
			}
		}
	}


	return 0;
}

static ssize_t mwl_debugfs_ampdu_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int value;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &value);
	if (ret)
		return ret;

	priv->debug_ampdu = value ? true : false;

	return count;
}

static int mwl_debugfs_stnid_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_stnid *stnid;
	int i;

	guard(spinlock_bh)(&priv->stnid_lock);
	for (i = 0; i < priv->stnid_num; i++) {
		stnid = &priv->stnid[i];
		if (!stnid->aid)
			continue;
		seq_printf(s, "stnid: %d macid: %d aid: %d\n",
			   i + 1, stnid->macid, stnid->aid);
	}


	return 0;
}

static int mwl_debugfs_device_pwrtbl_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	int i, j;

	seq_printf(s, "power table loaded from dts: %s\n",
		   priv->forbidden_setting ? "no" : "yes");
	seq_printf(s, "firmware region code: 0x%x\n",
		   priv->fw_region_code);
	seq_printf(s, "number of channel: %d\n",
		   priv->number_of_channels);
	for (i = 0; i < priv->number_of_channels; i++) {
		seq_printf(s, "%3d ",
			   priv->device_pwr_tbl[i].channel);
		for (j = 0; j < SYSADPT_TX_POWER_LEVEL_TOTAL; j++)
			seq_printf(s, "%3d ",
				   priv->device_pwr_tbl[i].tx_pwr[j]);
		seq_printf(s, "%3d ",
			   priv->device_pwr_tbl[i].dfs_capable);
		seq_printf(s, "%3d ",
			   priv->device_pwr_tbl[i].ax_ant);
		seq_printf(s, "%3d\n",
			   priv->device_pwr_tbl[i].cdd);
	}


	return 0;
}

static ssize_t mwl_debugfs_txpwrlmt_read(struct file *file,
					 char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);

	return simple_read_from_buffer(ubuf, count, ppos,
				       priv->txpwrlmt_data.buf,
				       priv->txpwrlmt_data.len);
}

static int mwl_debugfs_tx_amsdu_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	seq_printf(s, "tx amsdu: %s\n",
		   priv->tx_amsdu ? "enable" : "disable");


	return 0;
}

static ssize_t mwl_debugfs_tx_amsdu_write(struct file *file,
					  const char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int value;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &value);
	if (ret)
		return ret;

	priv->tx_amsdu = value ? true : false;

	return count;
}

static int mwl_debugfs_dump_hostcmd_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	seq_printf(s, "dump_hostcmd: %s\n",
		   priv->dump_hostcmd ? "enable" : "disable");


	return 0;
}

static ssize_t mwl_debugfs_dump_hostcmd_write(struct file *file,
					      const char __user *ubuf,
					      size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int value;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &value);
	if (ret)
		return ret;

	priv->dump_hostcmd = value ? true : false;

	return count;
}

static int mwl_debugfs_heartbeat_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	seq_printf(s, "heartbeat: %d\n",
		   priv->heartbeat);


	return 0;
}

static ssize_t mwl_debugfs_heartbeat_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &priv->heartbeat);
	if (ret)
		return ret;
	priv->pre_jiffies = jiffies;

	return count;
}

static int mwl_debugfs_thermal_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;

	if (mwl_fwcmd_get_temp(priv->hw, &priv->temperature)) {
			return -EIO;
	}

	seq_printf(s, "quiet period: %d\n",
		   priv->quiet_period);
	seq_printf(s, "throttle state: %d\n",
		   priv->throttle_state);
	seq_printf(s, "temperature: %d\n",
		   priv->temperature);


	return 0;
}

static ssize_t mwl_debugfs_thermal_write(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int throttle_state;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &throttle_state);
	if (ret)
		return ret;

	if (throttle_state < 0 ||
	    throttle_state > SYSADPT_THERMAL_THROTTLE_MAX) {
		wiphy_warn(priv->hw->wiphy,
			   "throttle state %d is exceeding the limit %d\n",
			   throttle_state, SYSADPT_THERMAL_THROTTLE_MAX);
		return -EINVAL;
	}

	priv->throttle_state = throttle_state;
	mwl_thermal_set_throttling(priv);
	return count;
}

static int mwl_debugfs_ratetable_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_sta *sta_info;
	struct ieee80211_sta *sta;
	u8 addr[ETH_ALEN];
	/* two words per rate: the rate info and a reserved word */
	int table_size = sizeof(__le32) * 2 * SYSADPT_MAX_RATE_ADAPT_RATES;
	__le32 *rate_table;
	u32 rate_info;
	u8 fmt, stbc, bw, sgi, mcs, preamble_gf, power_id, ldpc, bf, ant;
	int idx, rate, nss;
	bool found;
	int ret;

	if (!priv->ra_aid)
		return -EINVAL;

	found = false;
	spin_lock_bh(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		sta = container_of((void *)sta_info, struct ieee80211_sta,
				   drv_priv);
		if (priv->ra_aid == sta->aid) {
			ether_addr_copy(addr, sta->addr);
			found = true;
			break;
		}
	}
	spin_unlock_bh(&priv->sta_lock);

	if (!found)
		return -EINVAL;

	rate_table = kmalloc(table_size, GFP_KERNEL);
	if (!rate_table)
		return -ENOMEM;

	ret = mwl_fwcmd_get_ratetable(priv->hw, addr, rate_table,
				      table_size, 0);
	if (ret) {
		kfree(rate_table);
		return ret;
	}

	seq_printf(s, "%3s %6s %5s %5s %5s %5s %5s %4s %2s %5s %4s %5s %5s\n",
		   "Num", "Fmt", "STBC", "BW", "SGI", "Nss", "RateId",
		   "GF/Pre", "PId", "LDPC", "BF", "TxAnt", "Rate");
	for (idx = 0; idx < SYSADPT_MAX_RATE_ADAPT_RATES; idx++) {
		rate_info = le32_to_cpu(rate_table[idx * 2]);
		if (!rate_info)
			break;
		fmt = FIELD_GET(MWL_TX_RATE_FORMAT_MASK, rate_info);
		stbc = FIELD_GET(MWL_TX_RATE_STBC_MASK, rate_info);
		bw = FIELD_GET(MWL_TX_RATE_BANDWIDTH_MASK, rate_info);
		sgi = FIELD_GET(MWL_TX_RATE_SHORTGI_MASK, rate_info);
		mcs = FIELD_GET(MWL_TX_RATE_RATEIDMCS_MASK, rate_info);
		preamble_gf = FIELD_GET(MWL_TX_RATE_PREAMBLE_MASK, rate_info);
		power_id = FIELD_GET(MWL_TX_RATE_POWERID_MASK, rate_info);
		ldpc = FIELD_GET(MWL_TX_RATE_ADVCODING_MASK, rate_info);
		bf = FIELD_GET(MWL_TX_RATE_BF_MASK, rate_info);
		ant = FIELD_GET(MWL_TX_RATE_ANTSELECT_MASK, rate_info);

		if (fmt == TX_RATE_FORMAT_11AC) {
			rate = mcs & 0xf; /* 11ac, mcs[3:0]: rate      */
			nss = mcs >> 4;   /* 11ac, mcs[6:4] = nss code */
			nss++;  /* ddd 1 to correct Nss representation */
		} else {
			rate = mcs;
			nss = 0;
			if (fmt == TX_RATE_FORMAT_11N) {
				if ((mcs >= 0) && (mcs < 8))
					nss = 1;
				else if ((mcs >= 8) && (mcs < 16))
					nss = 2;
				else if ((mcs >= 16) && (mcs < 24))
					nss = 3;
			}
		}

		seq_printf(s, "%3d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d %5d\n",
			   idx, fmt, stbc, bw, sgi, nss, rate,
			   preamble_gf, power_id, ldpc, bf,
			   ant,
			   utils_get_phy_rate(fmt, bw, sgi, mcs));
	}

	kfree(rate_table);

	return 0;
}

static ssize_t mwl_debugfs_ratetable_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int sta_aid;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &sta_aid);
	if (ret)
		return ret;

	if ((sta_aid <= 0) || (sta_aid > SYSADPT_MAX_STA_SC4)) {
		wiphy_warn(priv->hw->wiphy,
			   "station aid is exceeding the limit %d\n", sta_aid);
		return -EINVAL;
	}

	priv->ra_aid = sta_aid;
	return count;
}

static int mwl_debugfs_tx_hist_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct ieee80211_sta *sta;
	struct mwl_sta *sta_info;

	if (priv->chip_type != MWL8964)
		return -EPERM;

	seq_printf(s, "SU: <4:%d >=4:%d >=15:%d >=50:%d >=100:%d >=250:%d\n",
		   priv->ra_tx_attempt[SU_MIMO][0],
		   priv->ra_tx_attempt[SU_MIMO][1],
		   priv->ra_tx_attempt[SU_MIMO][2],
		   priv->ra_tx_attempt[SU_MIMO][3],
		   priv->ra_tx_attempt[SU_MIMO][4],
		   priv->ra_tx_attempt[SU_MIMO][5]);
	seq_printf(s, "MU: <4:%d >=4:%d >=15:%d >=50:%d >=100:%d >=250:%d\n",
		   priv->ra_tx_attempt[MU_MIMO][0],
		   priv->ra_tx_attempt[MU_MIMO][1],
		   priv->ra_tx_attempt[MU_MIMO][2],
		   priv->ra_tx_attempt[MU_MIMO][3],
		   priv->ra_tx_attempt[MU_MIMO][4],
		   priv->ra_tx_attempt[MU_MIMO][5]);
	guard(spinlock_bh)(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		sta = container_of((void *)sta_info, struct ieee80211_sta,
				   drv_priv);
		seq_printf(s, "\nSTA %pM\n", sta->addr);
		seq_puts(s, "============================\n");
		dump_tx_hist(s, sta_info);
		seq_puts(s, "============================\n");
	}

	return 0;
}

static ssize_t mwl_debugfs_tx_hist_write(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int reset;
	struct mwl_sta *sta_info;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &reset);
	if (ret)
		return ret;

	if (!reset) {
		memset(priv->ra_tx_attempt, 0, sizeof(priv->ra_tx_attempt));
		scoped_guard(spinlock_bh, &priv->sta_lock) {
			list_for_each_entry(sta_info, &priv->sta_list, list) {
				memset(&sta_info->tx_hist, 0,
				       sizeof(sta_info->tx_hist));
			}
		}
	}

	return count;
}

static int mwl_debugfs_ba_hist_show(struct seq_file *s, void *data)
{
	struct mwl_priv *priv = s->private;
	struct mwl_sta *sta_info;
	struct mwl_tx_ba_stats *ba_stats;
	u32 i;
	u32 baholecnt, baexpcnt, bmap0cnt, nobacnt;
	u8 bmap0flag, nobaflag;

	if (!priv->ba_aid)
		return -EINVAL;

	/* The accounting path writes the histogram, and a write to this
	 * file frees it, under sta_lock.
	 */
	guard(spinlock_bh)(&priv->sta_lock);
	sta_info = utils_find_sta_by_aid(priv, priv->ba_aid);
	if (sta_info && sta_info->ba_hist.enable &&
	    sta_info->ba_hist.ba_stats) {
		ba_stats = sta_info->ba_hist.ba_stats;
		seq_printf(s, "BA histogram aid: %d, stnid: %d type: %s\n",
			   priv->ba_aid, sta_info->stnid,
			   sta_info->ba_hist.type ? "MU" : "SU");
		seq_printf(s, "%8s,%8s,%8s,%8s\n",
			   "BAhole", "Expect", "Bmap0", "NoBA");
		baholecnt = 0;
		baexpcnt = 0;
		bmap0cnt = 0;
		nobacnt = 0;
		for (i = 0; i < ACNT_BA_SIZE; i++) {
			if (!*(u32 *)&ba_stats[i])
				break;

			/* If no BA event does not happen, check BA hole and BA
			 * expected to mark BA bitmap all 0 event
			 */
			if (!ba_stats[i].no_ba)
				bmap0flag = (ba_stats[i].ba_hole ==
					     ba_stats[i].ba_expected) ? 1 : 0;
			else
				bmap0flag = 0;
			nobaflag = ba_stats[i].no_ba;

			seq_printf(s, "%8d,%8d,",
				   ba_stats[i].ba_hole,
				   ba_stats[i].ba_expected);

			baholecnt += ba_stats[i].ba_hole;
			baexpcnt += ba_stats[i].ba_expected;
			if (bmap0flag) {
				seq_puts(s, "       #,");
				bmap0cnt++;
			} else {
				seq_printf(s, "%8d,",
					   bmap0flag);
			}
			if (nobaflag) {
				seq_puts(s, "       *\n");
				nobacnt++;
			} else {
				seq_printf(s, "%8d\n",
					   nobaflag);
			}
		}

		seq_printf(s, "hole: %d, expect: %d, bmap0: %d, noba: %d\n",
			   baholecnt, baexpcnt, bmap0cnt, nobacnt);
	} else {
		seq_printf(s, "No BA histogram for sta aid: %d\n",
			   priv->ba_aid);
	}

	return 0;
}

static ssize_t mwl_debugfs_ba_hist_write(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int sta_aid;
	struct mwl_sta *sta_info;
	struct mwl_tx_ba_stats *ba_stats, *old_stats = NULL;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &sta_aid);
	if (ret)
		return ret;

	if ((sta_aid <= 0) || (sta_aid > SYSADPT_MAX_STA_SC4)) {
		wiphy_warn(priv->hw->wiphy,
			   "station aid is exceeding the limit %d\n", sta_aid);
		return -EINVAL;
	}

	ba_stats = kcalloc(ACNT_BA_SIZE, sizeof(*ba_stats), GFP_KERNEL);
	if (!ba_stats) {
		ret = -ENOMEM;
		return ret;
	}

	scoped_guard(spinlock_bh, &priv->sta_lock) {
		if (priv->ba_aid) {
			sta_info = utils_find_sta_by_aid(priv, priv->ba_aid);
			if (sta_info) {
				sta_info->ba_hist.enable = false;
				old_stats = sta_info->ba_hist.ba_stats;
				sta_info->ba_hist.ba_stats = NULL;
			}
		}
		priv->ba_aid = 0;
		sta_info = utils_find_sta_by_aid(priv, sta_aid);
		if (sta_info) {
			/* The same station may have been selected before. */
			kfree(old_stats);
			old_stats = sta_info->ba_hist.ba_stats;
			sta_info->ba_hist.ba_stats = ba_stats;
			sta_info->ba_hist.index = 0;
			sta_info->ba_hist.enable = true;
			priv->ba_aid = sta_aid;
			ba_stats = NULL;
			ret = count;
		} else {
			ret = -EINVAL;
		}
	}

	kfree(old_stats);
	kfree(ba_stats);

	return ret;
}

/* Upper bound on the dump size, in case the firmware reports nonsense */
#define MWL_COREDUMP_MAX_SIZE (16 * 1024 * 1024)

/* Collect the firmware memory regions and hand them to devcoredump. The
 * dump is the struct coredump header followed by each region's contents
 * in order. The read returns a summary of the regions.
 */
static ssize_t mwl_debugfs_core_dump_read(struct file *file, char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	unsigned long page;
	char *p;
	int len = 0, size = PAGE_SIZE;
	struct coredump_cmd *core_dump = NULL;
	struct coredump *cd = NULL;
	char *buff = NULL;
	u8 *dump = NULL;
	size_t dump_size, pos;
	u32 i, offset, chunk;
	u32 address, length;
	ssize_t ret;

	if (priv->chip_type != MWL8964)
		return -EPERM;

	if (!IS_ENABLED(CONFIG_DEV_COREDUMP))
		return -EOPNOTSUPP;

	if (*ppos)
		return 0;

	page = get_zeroed_page(GFP_KERNEL);
	if (!page)
		return -ENOMEM;
	p = (char *)page;

	core_dump = kzalloc_obj(*core_dump);
	buff = kzalloc(MAX_CORE_DUMP_BUFFER, GFP_KERNEL);
	cd = kmalloc_obj(*cd);
	if (!core_dump || !buff || !cd) {
		ret = -ENOMEM;
		goto err;
	}

	if (mwl_fwcmd_get_fw_core_dump(priv->hw, core_dump, buff)) {
		ret = -EIO;
		goto err;
	}
	memcpy(cd, buff, sizeof(*cd));

	/* The chunk request carries the region index in the top nibble of
	 * its context word, so only 16 regions can be addressed.
	 */
	if (cd->num_regions > min(MAX_CORE_REGIONS, 16)) {
		wiphy_err(priv->hw->wiphy, "core dump: %d regions\n",
			  cd->num_regions);
		ret = -EIO;
		goto err;
	}

	/* check each region before adding it so the sum can't wrap */
	dump_size = sizeof(*cd);
	for (i = 0; i < cd->num_regions; i++) {
		length = le32_to_cpu(cd->region[i].length);
		if (length > MWL_COREDUMP_MAX_SIZE - dump_size) {
			wiphy_err(priv->hw->wiphy,
				  "core dump: region %u length %x too large\n",
				  i, length);
			ret = -EIO;
			goto err;
		}
		dump_size += length;
	}

	dump = vzalloc(dump_size);
	if (!dump) {
		ret = -ENOMEM;
		goto err;
	}
	memcpy(dump, cd, sizeof(*cd));
	pos = sizeof(*cd);

	len += scnprintf(p + len, size - len, "Major Version : %d\n",
			 cd->version_major);
	len += scnprintf(p + len, size - len, "Minor Version : %d\n",
			 cd->version_minor);
	len += scnprintf(p + len, size - len, "Patch Version : %d\n",
			 cd->version_patch);
	len += scnprintf(p + len, size - len, "Num of Regions: %d\n",
			 cd->num_regions);
	len += scnprintf(p + len, size - len, "Num of Symbols: %d\n",
			 cd->num_symbols);

	for (i = 0; i < cd->num_regions; i++) {
		address = le32_to_cpu(cd->region[i].address);
		length = le32_to_cpu(cd->region[i].length);
		len += scnprintf(p + len, size - len, "\ncd.region[%d]: address=%x, length=%x\n",
				 i, address, length);

		for (offset = 0; offset < length;
		     offset += MAX_CORE_DUMP_BUFFER) {
			core_dump->context = cpu_to_le32((i << 28) | offset);
			core_dump->flags = 0;
			core_dump->size_kb = 0;
			if (mwl_fwcmd_get_fw_core_dump(priv->hw,
			    core_dump, buff)) {
				wiphy_err(priv->hw->wiphy,
					  "core dump: region:%d offset:%x failed\n",
					  i, offset);
				vfree(dump);
				ret = -EIO;
				goto err;
			}
			chunk = min_t(u32, length - offset,
				      MAX_CORE_DUMP_BUFFER);
			memcpy(dump + pos + offset, buff, chunk);
		}
		pos += length;
	}

	/* devcoredump takes ownership of the buffer */
	dev_coredumpv(wiphy_dev(priv->hw->wiphy), dump, dump_size,
		      GFP_KERNEL);
	len += scnprintf(p + len, size - len, "\nDump of %zu bytes available through devcoredump\n",
			 dump_size);

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);

err:
	kfree(core_dump);
	kfree(buff);
	kfree(cd);
	free_page(page);
	return ret;
}

static ssize_t mwl_debugfs_core_dump_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int text_mode;
	ssize_t ret;

	if (priv->chip_type != MWL8964)
		return -EPERM;

	/* The dump used to be written to /dev/shm in text or binary form;
	 * it now goes through devcoredump, so the value is only validated.
	 */
	ret = kstrtoint_from_user(ubuf, count, 0, &text_mode);
	if (ret)
		return ret;
	if (text_mode < 0 || text_mode > 1)
		return -EINVAL;

	mwl_fwcmd_core_dump_diag_mode(priv->hw, 1);
	return count;
}

static ssize_t mwl_debugfs_mcast_cts_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	int cts_enable = 0;
	ssize_t ret;

	ret = kstrtoint_from_user(ubuf, count, 0, &cts_enable);
	if (ret)
		return ret;

	ret = mwl_hif_mcast_cts(priv->hw, cts_enable ? true : false);
	if (ret)
		return ret;

	return count;
}

static ssize_t mwl_debugfs_wmmedcaap_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = mwl_debugfs_priv(file);
	u32 index = 0, cw_min = 0, cw_max = 0, aifsn = 0, txop = 0;
	char buf[64] = {};
	ssize_t ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;

	if (sscanf(buf, "%u %x %x %u %x", &index, &cw_min,
		   &cw_max, &aifsn, &txop) != 5)
		return -EINVAL;
	wiphy_info(priv->hw->wiphy, "set TCQ%d wmm edca with:\n", index);
	wiphy_info(priv->hw->wiphy,
		   "cw_min=0x%x, cw_max=0x%x, aifs_num=%d, txop=0x%x\n",
		   cw_min, cw_max, aifsn, txop);

	ret = mwl_fwcmd_set_edca_params(priv->hw, index,
			cw_min, cw_max, aifsn, txop);
	if (ret)
		return ret;

	return count;
}

DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_info);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_tx_status);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_rx_status);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_vif);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_sta);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_stnid);
DEFINE_SHOW_ATTRIBUTE(mwl_debugfs_device_pwrtbl);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_ampdu);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_tx_amsdu);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_rx_decrypt);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_dump_hostcmd);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_heartbeat);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_thermal);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_ratetable);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_tx_hist);
DEFINE_SHOW_STORE_ATTRIBUTE(mwl_debugfs_ba_hist);

static const struct file_operations mwl_debugfs_txpwrlmt_fops = {
	.read = mwl_debugfs_txpwrlmt_read,
	.open = simple_open,
};

/* Not a seq_file: a read collects a firmware dump, which must not be
 * repeated when seq_file retries with a larger buffer.
 */
static const struct file_operations mwl_debugfs_core_dump_fops = {
	.read = mwl_debugfs_core_dump_read,
	.write = mwl_debugfs_core_dump_write,
	.open = simple_open,
};

MWLWIFI_DEBUGFS_FILE_WRITE_OPS(mcast_cts);
MWLWIFI_DEBUGFS_FILE_WRITE_OPS(wmmedcaap);

void mwl_debugfs_init(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	if (!priv->debugfs_phy)
		priv->debugfs_phy = debugfs_create_dir("mwlwifi",
						       hw->wiphy->debugfsdir);

	MWLWIFI_DEBUGFS_ADD_FILE(info);
	if (priv->chip_type == MWL8964) {
		struct pcie_priv *pcie_priv = priv->hif.priv;

		debugfs_create_u32("tx_inflight_max", 0600, priv->debugfs_phy,
				   &pcie_priv->tx_inflight_max);
		debugfs_create_u32("tx_poll_us", 0600, priv->debugfs_phy,
				   &pcie_priv->tx_poll_us);
	}
	MWLWIFI_DEBUGFS_ADD_FILE(tx_status);
	MWLWIFI_DEBUGFS_ADD_FILE(rx_decrypt);
	MWLWIFI_DEBUGFS_ADD_FILE(rx_status);
	MWLWIFI_DEBUGFS_ADD_FILE(vif);
	MWLWIFI_DEBUGFS_ADD_FILE(sta);
	MWLWIFI_DEBUGFS_ADD_FILE(ampdu);
	MWLWIFI_DEBUGFS_ADD_FILE(stnid);
	MWLWIFI_DEBUGFS_ADD_FILE(device_pwrtbl);
	MWLWIFI_DEBUGFS_ADD_FILE(txpwrlmt);
	MWLWIFI_DEBUGFS_ADD_FILE(tx_amsdu);
	MWLWIFI_DEBUGFS_ADD_FILE(dump_hostcmd);
	MWLWIFI_DEBUGFS_ADD_FILE(heartbeat);
	MWLWIFI_DEBUGFS_ADD_FILE(thermal);
	MWLWIFI_DEBUGFS_ADD_FILE(ratetable);
	MWLWIFI_DEBUGFS_ADD_FILE(tx_hist);
	MWLWIFI_DEBUGFS_ADD_FILE(ba_hist);
	MWLWIFI_DEBUGFS_ADD_FILE(core_dump);
	MWLWIFI_DEBUGFS_ADD_FILE(mcast_cts);
	MWLWIFI_DEBUGFS_ADD_FILE(wmmedcaap);
}

void mwl_debugfs_remove(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	debugfs_remove(priv->debugfs_phy);
	priv->debugfs_phy = NULL;
}
