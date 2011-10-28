/*
 * This file is part of wl1271
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "debugfs.h"

#include <linux/skbuff.h>
#include <linux/slab.h>

#include "wl12xx.h"
#include "acx.h"
#include "ps.h"
#include "io.h"
#include "tx.h"

/* ms */
#define WL1271_DEBUGFS_STATS_LIFETIME 1000

/* debugfs macros idea from mac80211 */
#define DEBUGFS_FORMAT_BUFFER_SIZE 100
static int wl1271_format_buffer(char __user *userbuf, size_t count,
				    loff_t *ppos, char *fmt, ...)
{
	va_list args;
	char buf[DEBUGFS_FORMAT_BUFFER_SIZE];
	int res;

	va_start(args, fmt);
	res = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

#define DEBUGFS_READONLY_FILE(name, fmt, value...)			\
static ssize_t name## _read(struct file *file, char __user *userbuf,	\
			    size_t count, loff_t *ppos)			\
{									\
	struct wl1271 *wl = file->private_data;				\
	return wl1271_format_buffer(userbuf, count, ppos,		\
				    fmt "\n", ##value);			\
}									\
									\
static const struct file_operations name## _ops = {			\
	.read = name## _read,						\
	.open = wl1271_open_file_generic,				\
	.llseek	= generic_file_llseek,					\
};

#define DEBUGFS_ADD(name, parent)					\
	entry = debugfs_create_file(#name, 0400, parent,		\
				    wl, &name## _ops);			\
	if (!entry || IS_ERR(entry))					\
		goto err;						\

#define DEBUGFS_ADD_PREFIX(prefix, name, parent)			\
	do {								\
		entry = debugfs_create_file(#name, 0400, parent,	\
				    wl, &prefix## _## name## _ops);	\
		if (!entry || IS_ERR(entry))				\
			goto err;					\
	} while (0);

#define DEBUGFS_WL12XX_FWSTATS_ADD(sub, name)				\
	DEBUGFS_ADD(wl12xx_ ##sub## _ ##name, stats)

#define DEBUGFS_WL12XX_FWSTATS_FILE(sub, name, fmt)			\
static ssize_t wl12xx_ ##sub## _ ##name## _read(struct file *file,	\
				      char __user *userbuf,		\
				      size_t count, loff_t *ppos)	\
{									\
	struct wl1271 *wl = file->private_data;				\
									\
	wl1271_debugfs_update_stats(wl);				\
									\
	return wl1271_format_buffer(userbuf, count, ppos, fmt "\n",	\
				    wl->stats.wl12xx->sub.name);	\
}									\
									\
static const struct file_operations wl12xx_ ##sub## _ ##name## _ops = {	\
	.read = wl12xx_ ##sub## _ ##name## _read,			\
	.open = wl1271_open_file_generic,				\
	.llseek	= generic_file_llseek,					\
};

#define DEBUGFS_WL18XX_FWSTATS_ADD(sub, name)				\
	DEBUGFS_ADD(wl18xx_ ##sub## _ ##name, stats)

#define DEBUGFS_WL18XX_FWSTATS_FILE(sub, name, fmt)			\
static ssize_t wl18xx_ ##sub## _ ##name## _read(struct file *file,	\
				      char __user *userbuf,		\
				      size_t count, loff_t *ppos)	\
{									\
	struct wl1271 *wl = file->private_data;				\
									\
	wl1271_debugfs_update_stats(wl);				\
									\
	return wl1271_format_buffer(userbuf, count, ppos, fmt "\n",	\
				    wl->stats.wl18xx->sub.name);	\
}									\
									\
static const struct file_operations wl18xx_ ##sub## _ ##name## _ops = {	\
	.read = wl18xx_ ##sub## _ ##name## _read,			\
	.open = wl1271_open_file_generic,				\
	.llseek	= generic_file_llseek,					\
};

static void wl1271_debugfs_update_stats(struct wl1271 *wl)
{
	int ret;

	mutex_lock(&wl->mutex);

	ret = wl1271_ps_elp_wakeup(wl);
	if (ret < 0)
		goto out;

	if (wl->state == WL1271_STATE_ON &&
	    time_after(jiffies, wl->stats.fw_stats_update +
		       msecs_to_jiffies(WL1271_DEBUGFS_STATS_LIFETIME))) {
		if (wl->conf.platform_type == 1)
			wl12xx_acx_statistics(wl, wl->stats.wl12xx);
		else
			wl18xx_acx_statistics(wl, wl->stats.wl18xx);
		wl->stats.fw_stats_update = jiffies;
	}

	wl1271_ps_elp_sleep(wl);

out:
	mutex_unlock(&wl->mutex);
}

static int wl1271_open_file_generic(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

DEBUGFS_WL12XX_FWSTATS_FILE(tx, internal_desc_overflow, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(rx, out_of_mem, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, hdr_overflow, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, hw_stuck, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, dropped, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, fcs_err, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, xfr_hint_trig, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, path_reset, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rx, reset_counter, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(dma, rx_requested, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(dma, rx_errors, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(dma, tx_requested, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(dma, tx_errors, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(isr, cmd_cmplt, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, fiqs, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, rx_headers, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, rx_mem_overflow, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, rx_rdys, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, irqs, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, tx_procs, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, decrypt_done, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, dma0_done, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, dma1_done, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, tx_exch_complete, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, commands, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, rx_procs, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, hw_pm_mode_changes, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, host_acknowledges, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, pci_pm, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, wakeups, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(isr, low_rssi, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(wep, addr_key_count, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(wep, default_key_count, "%u");
/* skipping wep.reserved */
DEBUGFS_WL12XX_FWSTATS_FILE(wep, key_not_found, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(wep, decrypt_fail, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(wep, packets, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(wep, interrupt, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(pwr, ps_enter, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, elp_enter, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, missing_bcns, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, wake_on_host, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, wake_on_timer_exp, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, tx_with_ps, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, tx_without_ps, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, rcvd_beacons, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, power_save_off, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, enable_ps, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, disable_ps, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, fix_tsf_ps, "%u");
/* skipping cont_miss_bcns_spread for now */
DEBUGFS_WL12XX_FWSTATS_FILE(pwr, rcvd_awake_beacons, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(mic, rx_pkts, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(mic, calc_failure, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(aes, encrypt_fail, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(aes, decrypt_fail, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(aes, encrypt_packets, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(aes, decrypt_packets, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(aes, encrypt_interrupt, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(aes, decrypt_interrupt, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(event, heart_beat, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, calibration, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, rx_mismatch, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, rx_mem_empty, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, rx_pool, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, oom_late, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, phy_transmit_error, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(event, tx_stuck, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(ps, pspoll_timeouts, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, upsd_timeouts, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, upsd_max_sptime, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, upsd_max_apturn, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, pspoll_max_apturn, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, pspoll_utilization, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(ps, upsd_utilization, "%u");

DEBUGFS_WL12XX_FWSTATS_FILE(rxpipe, rx_prep_beacon_drop, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rxpipe, descr_host_int_trig_rx_data, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rxpipe,
				beacon_buffer_thres_host_int_trig_rx_data, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rxpipe,
				missed_beacon_host_int_trig_rx_data, "%u");
DEBUGFS_WL12XX_FWSTATS_FILE(rxpipe, tx_xfr_host_int_trig_rx_data, "%u");

/* ring */
DEBUGFS_WL18XX_FWSTATS_FILE(ring, tx_procs, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, prepared_descs, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, tx_xfr, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, tx_dma, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, tx_cmplt, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, rx_procs, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ring, rx_data, "%u");

/* debug */
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug1, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug2, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug3, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug4, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug5, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dbg, debug6, "%u");

/* tx */
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_mpdu_alloc_failed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_init_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_in_process_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_tkip_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_key_not_found, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_need_fragmentation, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_bad_mem_blk_num, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_failed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_cache_hit, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_cache_miss, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_1, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_2, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_3, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_4, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_5, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_6, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_7, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, frag_8, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, template_prepared, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, data_prepared, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, template_programmed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, data_programmed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, burst_programmed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, starts, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, imm_resp, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, start_tempaltes, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, start_int_template, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, start_fw_gen, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, start_data, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, start_null_frame, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, exch, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, retry_template, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, retry_data, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, exch_pending, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, exch_mismatch, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, done_template, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, done_data, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, done_intTemplate, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, pre_xfr, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, xfr, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, xfr_out_of_mem, "%u")
DEBUGFS_WL18XX_FWSTATS_FILE(tx, dma_programmed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, dma_done, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, checksum_req, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(tx, checksum_calc, "%u");

/* rx */
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_out_of_mem, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_hdr_overflow, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_hw_stuck, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_dropped_frame, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_complete_dropped_frame, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_Alloc_frame, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_done_queue, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, rx_done, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_init_Called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_in_Process_Called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_tkip_called, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_need_defrag, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_decrypt_failed, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, decrypt_Key_not_found, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_need_decr, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag1, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag2, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag3, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag4, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag5, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag6, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag7, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag8, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, defrag_end, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, xfr, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, xfr_end, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, cmplt, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, pre_cmplt, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, cmplt_task, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, phy_hdr, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, timeout, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, checksum_req, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx, checksum_calc, "%u");


/* dma */
DEBUGFS_WL18XX_FWSTATS_FILE(dma, rx_errors, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(dma, tx_errors, "%u");

/* irq */
DEBUGFS_WL18XX_FWSTATS_FILE(isr, irqs, "%u");


/* pwr */
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, missing_bcns, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, rcvd_beacons, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, conn_out_of_sync, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_1, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_2, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_3, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_4, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_5, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_6, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_7, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_8, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_9, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, cont_missbcns_spread_10_plus, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(pwr, rcvd_awake_beacons_cnt, "%u");


/* event */
DEBUGFS_WL18XX_FWSTATS_FILE(event, calibration, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(event, rx_mismatch, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(event, rx_mem_empty, "%u");


/* ps_poll_upsd */
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, ps_poll_timeouts, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, upsd_timeouts, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, upsd_max_ap_turn, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, ps_poll_max_ap_turn, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, ps_poll_utilization, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(ps_poll_upsd, upsd_utilization, "%u");


/* rx_filter */
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, beacon_filter, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, arp_filter, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, mc_filter, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, dup_filter, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, data_filter, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(rx_filter, ibss_filter, "%u");


/* calibration_fail */
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, init_cal_total, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, init_radio_bands_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, init_set_params, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, init_tx_clpc_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, init_rx_iq_mm_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_cal_total, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_rtrim_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_pd_buf_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_tx_mix_freq_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_ta_cal, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_rxIf2Gain, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_rx_dac, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_chan_tune, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_rx_tx_lpf, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_drpw_lna_tank, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_tx_lo_leak_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_tx_iq_mm_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_tx_pdet_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_tx_ppa_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_tx_clpc_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_rx_ana_dc_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_rx_dig_dc_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, tune_rx_iq_mm_fail, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(calibration_fail, cal_state_fail, "%u");


/* agg_size */
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_1, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_2, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_3, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_4, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_5, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_6, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_7, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(agg, size_8, "%u");

/* new_pipe_line */
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, hs_tx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, hs_rx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, tcp_tx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, tcp_rx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, enc_tx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, enc_rx_stat_fifo_int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, rx_complete_stat_fifo_Int, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, pre_proc_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, post_proc_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, sec_frag_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, pre_to_defrag_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, defrag_to_csum_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, csum_to_rx_xfer_swi, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, dec_packet_in_fifo_Full, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, dec_packet_out, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, cs_rx_packet_in, "%u");
DEBUGFS_WL18XX_FWSTATS_FILE(new_pipe_line, cs_rx_packet_out, "%u");


DEBUGFS_READONLY_FILE(retry_count, "%u", wl->stats.retry_count);
DEBUGFS_READONLY_FILE(excessive_retries, "%u",
		      wl->stats.excessive_retries);

static ssize_t tx_queue_len_read(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	u32 queue_len;
	char buf[20];
	int res;

	queue_len = wl1271_tx_total_queue_count(wl);

	res = scnprintf(buf, sizeof(buf), "%u\n", queue_len);
	return simple_read_from_buffer(userbuf, count, ppos, buf, res);
}

static const struct file_operations tx_queue_len_ops = {
	.read = tx_queue_len_read,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t gpio_power_read(struct file *file, char __user *user_buf,
			  size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	bool state = test_bit(WL1271_FLAG_GPIO_POWER, &wl->flags);

	int res;
	char buf[10];

	res = scnprintf(buf, sizeof(buf), "%d\n", state);

	return simple_read_from_buffer(user_buf, count, ppos, buf, res);
}

static ssize_t gpio_power_write(struct file *file,
			   const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value in gpio_power");
		return -EINVAL;
	}

	mutex_lock(&wl->mutex);

	if (value)
		wl1271_power_on(wl);
	else
		wl1271_power_off(wl);

	mutex_unlock(&wl->mutex);
	return count;
}

static const struct file_operations gpio_power_ops = {
	.read = gpio_power_read,
	.write = gpio_power_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t start_recovery_write(struct file *file,
				    const char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;

	mutex_lock(&wl->mutex);
	wl12xx_queue_recovery_work(wl);
	mutex_unlock(&wl->mutex);

	return count;
}

static const struct file_operations start_recovery_ops = {
	.write = start_recovery_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t driver_state_read(struct file *file, char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	int res = 0;
	char buf[1024];

	mutex_lock(&wl->mutex);

#define DRIVER_STATE_PRINT(x, fmt)   \
	(res += scnprintf(buf + res, sizeof(buf) - res,\
			  #x " = " fmt "\n", wl->x))

#define DRIVER_STATE_PRINT_LONG(x) DRIVER_STATE_PRINT(x, "%ld")
#define DRIVER_STATE_PRINT_INT(x)  DRIVER_STATE_PRINT(x, "%d")
#define DRIVER_STATE_PRINT_STR(x)  DRIVER_STATE_PRINT(x, "%s")
#define DRIVER_STATE_PRINT_LHEX(x) DRIVER_STATE_PRINT(x, "0x%lx")
#define DRIVER_STATE_PRINT_HEX(x)  DRIVER_STATE_PRINT(x, "0x%x")

	DRIVER_STATE_PRINT_INT(tx_blocks_available);
	DRIVER_STATE_PRINT_INT(tx_allocated_blocks);
	DRIVER_STATE_PRINT_INT(tx_allocated_pkts[0]);
	DRIVER_STATE_PRINT_INT(tx_allocated_pkts[1]);
	DRIVER_STATE_PRINT_INT(tx_allocated_pkts[2]);
	DRIVER_STATE_PRINT_INT(tx_allocated_pkts[3]);
	DRIVER_STATE_PRINT_INT(tx_frames_cnt);
	DRIVER_STATE_PRINT_LHEX(tx_frames_map[0]);
	DRIVER_STATE_PRINT_INT(tx_queue_count[0]);
	DRIVER_STATE_PRINT_INT(tx_queue_count[1]);
	DRIVER_STATE_PRINT_INT(tx_queue_count[2]);
	DRIVER_STATE_PRINT_INT(tx_queue_count[3]);
	DRIVER_STATE_PRINT_INT(tx_packets_count);
	DRIVER_STATE_PRINT_INT(tx_results_count);
	DRIVER_STATE_PRINT_LHEX(flags);
	DRIVER_STATE_PRINT_INT(tx_blocks_freed);
	DRIVER_STATE_PRINT_INT(rx_counter);
	DRIVER_STATE_PRINT_INT(state);
	DRIVER_STATE_PRINT_INT(channel);
	DRIVER_STATE_PRINT_INT(band);
	DRIVER_STATE_PRINT_INT(power_level);
	DRIVER_STATE_PRINT_INT(sg_enabled);
	DRIVER_STATE_PRINT_INT(enable_11a);
	DRIVER_STATE_PRINT_INT(noise);
	DRIVER_STATE_PRINT_HEX(ap_fw_ps_map);
	DRIVER_STATE_PRINT_LHEX(ap_ps_map);
	DRIVER_STATE_PRINT_HEX(quirks);
	DRIVER_STATE_PRINT_HEX(irq);
	DRIVER_STATE_PRINT_HEX(ref_clock);
	DRIVER_STATE_PRINT_HEX(tcxo_clock);
	DRIVER_STATE_PRINT_HEX(hw_pg_ver);
	DRIVER_STATE_PRINT_HEX(platform_quirks);
	DRIVER_STATE_PRINT_HEX(chip.id);
	DRIVER_STATE_PRINT_STR(chip.fw_ver_str);
	DRIVER_STATE_PRINT_INT(sched_scanning);

#undef DRIVER_STATE_PRINT_INT
#undef DRIVER_STATE_PRINT_LONG
#undef DRIVER_STATE_PRINT_HEX
#undef DRIVER_STATE_PRINT_LHEX
#undef DRIVER_STATE_PRINT_STR
#undef DRIVER_STATE_PRINT

	mutex_unlock(&wl->mutex);

	return simple_read_from_buffer(user_buf, count, ppos, buf, res);
}

static const struct file_operations driver_state_ops = {
	.read = driver_state_read,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t dtim_interval_read(struct file *file, char __user *user_buf,
				  size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	u8 value;

	if (wl->conf.conn.wake_up_event == CONF_WAKE_UP_EVENT_DTIM ||
	    wl->conf.conn.wake_up_event == CONF_WAKE_UP_EVENT_N_DTIM)
		value = wl->conf.conn.listen_interval;
	else
		value = 0;

	return wl1271_format_buffer(user_buf, count, ppos, "%d\n", value);
}

static ssize_t dtim_interval_write(struct file *file,
				   const char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value for dtim_interval");
		return -EINVAL;
	}

	if (value < 1 || value > 10) {
		wl1271_warning("dtim value is not in valid range");
		return -ERANGE;
	}

	mutex_lock(&wl->mutex);

	wl->conf.conn.listen_interval = value;
	/* for some reason there are different event types for 1 and >1 */
	if (value == 1)
		wl->conf.conn.wake_up_event = CONF_WAKE_UP_EVENT_DTIM;
	else
		wl->conf.conn.wake_up_event = CONF_WAKE_UP_EVENT_N_DTIM;

	/*
	 * we don't reconfigure ACX_WAKE_UP_CONDITIONS now, so it will only
	 * take effect on the next time we enter psm.
	 */
	mutex_unlock(&wl->mutex);
	return count;
}

static const struct file_operations dtim_interval_ops = {
	.read = dtim_interval_read,
	.write = dtim_interval_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t beacon_interval_read(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	u8 value;

	if (wl->conf.conn.wake_up_event == CONF_WAKE_UP_EVENT_BEACON ||
	    wl->conf.conn.wake_up_event == CONF_WAKE_UP_EVENT_N_BEACONS)
		value = wl->conf.conn.listen_interval;
	else
		value = 0;

	return wl1271_format_buffer(user_buf, count, ppos, "%d\n", value);
}

static ssize_t beacon_interval_write(struct file *file,
				     const char __user *user_buf,
				     size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value for beacon_interval");
		return -EINVAL;
	}

	if (value < 1 || value > 255) {
		wl1271_warning("beacon interval value is not in valid range");
		return -ERANGE;
	}

	mutex_lock(&wl->mutex);

	wl->conf.conn.listen_interval = value;
	/* for some reason there are different event types for 1 and >1 */
	if (value == 1)
		wl->conf.conn.wake_up_event = CONF_WAKE_UP_EVENT_BEACON;
	else
		wl->conf.conn.wake_up_event = CONF_WAKE_UP_EVENT_N_BEACONS;

	/*
	 * we don't reconfigure ACX_WAKE_UP_CONDITIONS now, so it will only
	 * take effect on the next time we enter psm.
	 */
	mutex_unlock(&wl->mutex);
	return count;
}

static const struct file_operations beacon_interval_ops = {
	.read = beacon_interval_read,
	.write = beacon_interval_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t rx_streaming_interval_write(struct file *file,
			   const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	struct wl12xx_vif *wlvif;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value in rx_streaming_interval!");
		return -EINVAL;
	}

	/* valid values: 0, 10-100 */
	if (value && (value < 10 || value > 100)) {
		wl1271_warning("value is not in range!");
		return -ERANGE;
	}

	mutex_lock(&wl->mutex);

	wl->conf.rx_streaming.interval = value;

	ret = wl1271_ps_elp_wakeup(wl);
	if (ret < 0)
		goto out;

	wl12xx_for_each_wlvif_sta(wl, wlvif) {
		wl1271_recalc_rx_streaming(wl, wlvif);
	}

	wl1271_ps_elp_sleep(wl);
out:
	mutex_unlock(&wl->mutex);
	return count;
}

static ssize_t rx_streaming_interval_read(struct file *file,
			    char __user *userbuf,
			    size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	return wl1271_format_buffer(userbuf, count, ppos,
				    "%d\n", wl->conf.rx_streaming.interval);
}

static const struct file_operations rx_streaming_interval_ops = {
	.read = rx_streaming_interval_read,
	.write = rx_streaming_interval_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t rx_streaming_always_write(struct file *file,
			   const char __user *user_buf,
			   size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	struct wl12xx_vif *wlvif;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value in rx_streaming_write!");
		return -EINVAL;
	}

	/* valid values: 0, 10-100 */
	if (!(value == 0 || value == 1)) {
		wl1271_warning("value is not in valid!");
		return -EINVAL;
	}

	mutex_lock(&wl->mutex);

	wl->conf.rx_streaming.always = value;

	ret = wl1271_ps_elp_wakeup(wl);
	if (ret < 0)
		goto out;

	wl12xx_for_each_wlvif_sta(wl, wlvif) {
		wl1271_recalc_rx_streaming(wl, wlvif);
	}

	wl1271_ps_elp_sleep(wl);
out:
	mutex_unlock(&wl->mutex);
	return count;
}

static ssize_t rx_streaming_always_read(struct file *file,
			    char __user *userbuf,
			    size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	return wl1271_format_buffer(userbuf, count, ppos,
				    "%d\n", wl->conf.rx_streaming.always);
}

static const struct file_operations rx_streaming_always_ops = {
	.read = rx_streaming_always_read,
	.write = rx_streaming_always_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t beacon_filtering_write(struct file *file,
				      const char __user *user_buf,
				      size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	struct wl12xx_vif *wlvif;
	char buf[10];
	size_t len;
	unsigned long value;
	int ret;

	len = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = '\0';

	ret = kstrtoul(buf, 0, &value);
	if (ret < 0) {
		wl1271_warning("illegal value for beacon_filtering!");
		return -EINVAL;
	}

	mutex_lock(&wl->mutex);

	ret = wl1271_ps_elp_wakeup(wl);
	if (ret < 0)
		goto out;

	wl12xx_for_each_wlvif(wl, wlvif) {
		ret = wl1271_acx_beacon_filter_opt(wl, wlvif, !!value);
	}

	wl1271_ps_elp_sleep(wl);
out:
	mutex_unlock(&wl->mutex);
	return count;
}

static const struct file_operations beacon_filtering_ops = {
	.write = beacon_filtering_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static ssize_t platform_type_write(struct file *file,
				   const char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	unsigned long value;
	int ret;

	ret = kstrtoul_from_user(user_buf, count, 10, &value);
	if (ret < 0) {
		wl1271_warning("illegal value in platform_type_write!");
		return -EINVAL;
	}

	/* valid values: 1, 2 */
	if (!(value == 1 || value == 2)) {
		wl1271_warning("value is not in valid!");
		return -EINVAL;
	}

	wl->conf.platform_type = value;

	return count;
}

static ssize_t platform_type_read(struct file *file,
				  char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	struct wl1271 *wl = file->private_data;
	return wl1271_format_buffer(userbuf, count, ppos,
				    "%d\n", wl->conf.platform_type);
}

static const struct file_operations platform_type_ops = {
	.read = platform_type_read,
	.write = platform_type_write,
	.open = wl1271_open_file_generic,
	.llseek = default_llseek,
};

static int wl12xx_debugfs_add_fwstats(struct wl1271 *wl,
				      struct dentry *rootdir)
{
	int ret = 0;
	struct dentry *entry, *stats;

	wl->stats.wl12xx = kzalloc(sizeof(*wl->stats.wl12xx), GFP_KERNEL);
	if (!wl->stats.wl12xx)
		return -ENOMEM;

	stats = debugfs_create_dir("fw-statistics", rootdir);
	if (!stats || IS_ERR(stats)) {
		entry = stats;
		goto err;
	}

	wl->stats.fw_stats_update = jiffies;

	DEBUGFS_WL12XX_FWSTATS_ADD(tx, internal_desc_overflow);

	DEBUGFS_WL12XX_FWSTATS_ADD(rx, out_of_mem);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, hdr_overflow);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, hw_stuck);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, dropped);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, fcs_err);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, xfr_hint_trig);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, path_reset);
	DEBUGFS_WL12XX_FWSTATS_ADD(rx, reset_counter);

	DEBUGFS_WL12XX_FWSTATS_ADD(dma, rx_requested);
	DEBUGFS_WL12XX_FWSTATS_ADD(dma, rx_errors);
	DEBUGFS_WL12XX_FWSTATS_ADD(dma, tx_requested);
	DEBUGFS_WL12XX_FWSTATS_ADD(dma, tx_errors);

	DEBUGFS_WL12XX_FWSTATS_ADD(isr, cmd_cmplt);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, fiqs);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, rx_headers);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, rx_mem_overflow);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, rx_rdys);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, irqs);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, tx_procs);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, decrypt_done);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, dma0_done);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, dma1_done);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, tx_exch_complete);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, commands);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, rx_procs);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, hw_pm_mode_changes);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, host_acknowledges);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, pci_pm);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, wakeups);
	DEBUGFS_WL12XX_FWSTATS_ADD(isr, low_rssi);

	DEBUGFS_WL12XX_FWSTATS_ADD(wep, addr_key_count);
	DEBUGFS_WL12XX_FWSTATS_ADD(wep, default_key_count);
	/* skipping wep.reserved */
	DEBUGFS_WL12XX_FWSTATS_ADD(wep, key_not_found);
	DEBUGFS_WL12XX_FWSTATS_ADD(wep, decrypt_fail);
	DEBUGFS_WL12XX_FWSTATS_ADD(wep, packets);
	DEBUGFS_WL12XX_FWSTATS_ADD(wep, interrupt);

	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, ps_enter);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, elp_enter);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, missing_bcns);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, wake_on_host);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, wake_on_timer_exp);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, tx_with_ps);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, tx_without_ps);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, rcvd_beacons);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, power_save_off);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, enable_ps);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, disable_ps);
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, fix_tsf_ps);
	/* skipping cont_miss_bcns_spread for now */
	DEBUGFS_WL12XX_FWSTATS_ADD(pwr, rcvd_awake_beacons);

	DEBUGFS_WL12XX_FWSTATS_ADD(mic, rx_pkts);
	DEBUGFS_WL12XX_FWSTATS_ADD(mic, calc_failure);

	DEBUGFS_WL12XX_FWSTATS_ADD(aes, encrypt_fail);
	DEBUGFS_WL12XX_FWSTATS_ADD(aes, decrypt_fail);
	DEBUGFS_WL12XX_FWSTATS_ADD(aes, encrypt_packets);
	DEBUGFS_WL12XX_FWSTATS_ADD(aes, decrypt_packets);
	DEBUGFS_WL12XX_FWSTATS_ADD(aes, encrypt_interrupt);
	DEBUGFS_WL12XX_FWSTATS_ADD(aes, decrypt_interrupt);

	DEBUGFS_WL12XX_FWSTATS_ADD(event, heart_beat);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, calibration);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, rx_mismatch);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, rx_mem_empty);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, rx_pool);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, oom_late);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, phy_transmit_error);
	DEBUGFS_WL12XX_FWSTATS_ADD(event, tx_stuck);

	DEBUGFS_WL12XX_FWSTATS_ADD(ps, pspoll_timeouts);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, upsd_timeouts);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, upsd_max_sptime);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, upsd_max_apturn);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, pspoll_max_apturn);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, pspoll_utilization);
	DEBUGFS_WL12XX_FWSTATS_ADD(ps, upsd_utilization);

	DEBUGFS_WL12XX_FWSTATS_ADD(rxpipe, rx_prep_beacon_drop);
	DEBUGFS_WL12XX_FWSTATS_ADD(rxpipe, descr_host_int_trig_rx_data);
	DEBUGFS_WL12XX_FWSTATS_ADD(rxpipe, beacon_buffer_thres_host_int_trig_rx_data);
	DEBUGFS_WL12XX_FWSTATS_ADD(rxpipe, missed_beacon_host_int_trig_rx_data);
	DEBUGFS_WL12XX_FWSTATS_ADD(rxpipe, tx_xfr_host_int_trig_rx_data);

	return 0;

err:
	if (IS_ERR(entry))
		ret = PTR_ERR(entry);
	else
		ret = -ENOMEM;

	return ret;
}

static int wl18xx_debugfs_add_fwstats(struct wl1271 *wl,
				      struct dentry *rootdir)
{
	int ret = 0;
	struct dentry *entry, *stats;

	wl->stats.wl18xx = kzalloc(sizeof(*wl->stats.wl18xx), GFP_KERNEL);
	if (!wl->stats.wl12xx)
		return -ENOMEM;

	stats = debugfs_create_dir("fw-statistics", rootdir);
	if (!stats || IS_ERR(stats)) {
		entry = stats;
		goto err;
	}

	wl->stats.fw_stats_update = jiffies;

	/* ring */
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, prepared_descs);
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, tx_xfr);
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, tx_dma);
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, tx_cmplt);
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, rx_procs);
	DEBUGFS_WL18XX_FWSTATS_ADD(ring, rx_data);

	/* debug */
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug1);
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug2);
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug3);
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug4);
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug5);
	DEBUGFS_WL18XX_FWSTATS_ADD(dbg, debug6);

	/* tx */
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_mpdu_alloc_failed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_init_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_in_process_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_tkip_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_key_not_found);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_need_fragmentation);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_bad_mem_blk_num);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_failed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_cache_hit);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_cache_miss);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_1);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_2);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_3);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_4);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_5);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_6);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_7);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, frag_8);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, template_prepared);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, data_prepared);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, template_programmed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, data_programmed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, burst_programmed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, starts);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, imm_resp);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, start_tempaltes);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, start_int_template);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, start_fw_gen);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, start_data);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, start_null_frame);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, exch);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, retry_template);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, retry_data);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, exch_pending)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, exch_mismatch)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, done_template)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, done_data)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, done_intTemplate)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, pre_xfr)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, xfr)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, xfr_out_of_mem)
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, dma_programmed);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, dma_done);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, checksum_req);
	DEBUGFS_WL18XX_FWSTATS_ADD(tx, checksum_calc);

	/* rx */
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_out_of_mem);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_hdr_overflow);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_hw_stuck);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_dropped_frame);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_complete_dropped_frame);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_Alloc_frame);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_done_queue);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, rx_done);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_init_Called);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_in_Process_Called);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_tkip_called);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_need_defrag);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_decrypt_failed);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, decrypt_Key_not_found);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_need_decr);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag1);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag2);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag3);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag4);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag5);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag6);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag7);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag8);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, defrag_end);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, xfr);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, xfr_end);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, cmplt);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, pre_cmplt);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, cmplt_task);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, phy_hdr);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, timeout);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, checksum_req);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx, checksum_calc);


	/* dma */
	DEBUGFS_WL18XX_FWSTATS_ADD(dma, rx_errors);
	DEBUGFS_WL18XX_FWSTATS_ADD(dma, tx_errors);

	/* isr */
	DEBUGFS_WL18XX_FWSTATS_ADD(isr, irqs);


	/* pwr */
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, missing_bcns);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, rcvd_beacons);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, conn_out_of_sync);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_1);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_2);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_3);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_4);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_5);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_6);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_7);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_8);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_9);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, cont_missbcns_spread_10_plus);
	DEBUGFS_WL18XX_FWSTATS_ADD(pwr, rcvd_awake_beacons_cnt);


	/* event */
	DEBUGFS_WL18XX_FWSTATS_ADD(event, calibration);
	DEBUGFS_WL18XX_FWSTATS_ADD(event, rx_mismatch);
	DEBUGFS_WL18XX_FWSTATS_ADD(event, rx_mem_empty);


	/* ps_poll_upsd */
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, ps_poll_timeouts);
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, upsd_timeouts);
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, upsd_max_ap_turn);
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, ps_poll_max_ap_turn);
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, ps_poll_utilization);
	DEBUGFS_WL18XX_FWSTATS_ADD(ps_poll_upsd, upsd_utilization);


	/* rx_filter */
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, beacon_filter);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, arp_filter);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, mc_filter);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, dup_filter);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, data_filter);
	DEBUGFS_WL18XX_FWSTATS_ADD(rx_filter, ibss_filter);


	/* calibration_fail */
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, init_cal_total);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, init_radio_bands_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, init_set_params);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, init_tx_clpc_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, init_rx_iq_mm_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_cal_total);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_rtrim_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_pd_buf_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_tx_mix_freq_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_ta_cal);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_rxIf2Gain);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_rx_dac);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_chan_tune);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_rx_tx_lpf);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_drpw_lna_tank);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_tx_lo_leak_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_tx_iq_mm_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_tx_pdet_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_tx_ppa_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_tx_clpc_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_rx_ana_dc_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_rx_dig_dc_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, tune_rx_iq_mm_fail);
	DEBUGFS_WL18XX_FWSTATS_ADD(calibration_fail, cal_state_fail);


	/* agg_size */
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_1);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_2);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_3);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_4);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_5);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_6);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_7);
	DEBUGFS_WL18XX_FWSTATS_ADD(agg, size_8);

	/* new_pipe_line */
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, hs_tx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, hs_rx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, tcp_tx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, tcp_rx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, enc_tx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, enc_rx_stat_fifo_int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, rx_complete_stat_fifo_Int);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, pre_proc_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, post_proc_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, sec_frag_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, pre_to_defrag_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, defrag_to_csum_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, csum_to_rx_xfer_swi);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, dec_packet_in_fifo_Full);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, dec_packet_out);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, cs_rx_packet_in);
	DEBUGFS_WL18XX_FWSTATS_ADD(new_pipe_line, cs_rx_packet_out);

	return 0;

err:
	if (IS_ERR(entry))
		ret = PTR_ERR(entry);
	else
		ret = -ENOMEM;

	return ret;
}

static int wlcore_debugfs_add_family_specific(struct wl1271 *wl,
					      struct dentry *rootdir)
{
	if (wl->conf.platform_type == 1)
		return wl12xx_debugfs_add_fwstats(wl, rootdir);
	else
		return wl18xx_debugfs_add_fwstats(wl, rootdir);
}

/* add debugfs files that depend on a running FW */
int wlcore_debugfs_add_fw_files(struct wl1271 *wl)
{
	int ret = 0;
	struct dentry *entry, *streaming, *rootdir;

	if (!wl->debugfs_rootdir)
		return -EINVAL;

	rootdir = wl->debugfs_rootdir;

	DEBUGFS_ADD(start_recovery, rootdir);
	DEBUGFS_ADD(beacon_filtering, rootdir);

	streaming = debugfs_create_dir("rx_streaming", rootdir);
	if (!streaming || IS_ERR(streaming))
		goto err;

	DEBUGFS_ADD_PREFIX(rx_streaming, interval, streaming);
	DEBUGFS_ADD_PREFIX(rx_streaming, always, streaming);

	return wlcore_debugfs_add_family_specific(wl, rootdir);

err:
	if (IS_ERR(entry))
		ret = PTR_ERR(entry);
	else
		ret = -ENOMEM;

	return ret;
}

/*
 * Common debugfs files for all chip families. Added before the FW is up,
 * when the device is first registered.
 */
static int wlcore_debugfs_add_device_files(struct wl1271 *wl,
					   struct dentry *rootdir)
{
	int ret = 0;
	struct dentry *entry;

	DEBUGFS_ADD(tx_queue_len, rootdir);
	DEBUGFS_ADD(retry_count, rootdir);
	DEBUGFS_ADD(excessive_retries, rootdir);

	DEBUGFS_ADD(gpio_power, rootdir);
	DEBUGFS_ADD(driver_state, rootdir);
	DEBUGFS_ADD(dtim_interval, rootdir);
	DEBUGFS_ADD(beacon_interval, rootdir);

	DEBUGFS_ADD(platform_type, rootdir);

	return 0;

err:
	if (IS_ERR(entry))
		ret = PTR_ERR(entry);
	else
		ret = -ENOMEM;

	return ret;
}

void wlcore_debugfs_reset(struct wl1271 *wl)
{
	/* remove all files and add back just the ones not related to FW */
	if (wl->debugfs_rootdir)
		wlcore_debugfs_exit(wl);

	wlcore_debugfs_init(wl);

	wl->stats.retry_count = 0;
	wl->stats.excessive_retries = 0;
}

int wlcore_debugfs_init(struct wl1271 *wl)
{
	int ret;

	if (!wl->debugfs_rootdir) {
		struct dentry *rootdir;

		rootdir = debugfs_create_dir(KBUILD_MODNAME,
					     wl->hw->wiphy->debugfsdir);

		if (IS_ERR(rootdir)) {
			ret = PTR_ERR(rootdir);
			return ret;
		}

		wl->debugfs_rootdir = rootdir;
	}

	ret = wlcore_debugfs_add_device_files(wl, wl->debugfs_rootdir);
	if (ret < 0)
		goto err;

	return 0;

err:
	debugfs_remove_recursive(wl->debugfs_rootdir);
	wl->debugfs_rootdir = NULL;
	return ret;
}

void wlcore_debugfs_exit(struct wl1271 *wl)
{
	if (wl->conf.platform_type == 1) {
		if (wl->debugfs_rootdir)
			debugfs_remove_recursive(wl->debugfs_rootdir);
		wl->debugfs_rootdir = NULL;
		kfree(wl->stats.wl12xx);
		wl->stats.wl12xx = NULL;
	} else {
		if (wl->debugfs_rootdir)
			debugfs_remove_recursive(wl->debugfs_rootdir);
		wl->debugfs_rootdir = NULL;
		kfree(wl->stats.wl18xx);
		wl->stats.wl18xx = NULL;
	}
}
