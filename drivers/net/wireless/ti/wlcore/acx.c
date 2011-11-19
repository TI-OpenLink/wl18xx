/*
 * This file is part of wlcore
 *
 * Copyright (C) 2011 Texas Instruments Inc.
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

#include "wlcore.h"
#include "acx.h"
#include "debug.h"

int wlcore_acx_event_mbox_mask(struct wlcore *wl, u32 event_mask)
{
	struct acx_event_mask *mask;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx event mbox mask");

	mask = kzalloc(sizeof(*mask), GFP_KERNEL);
	if (!mask) {
		ret = -ENOMEM;
		goto out;
	}

	/* high event mask is unused */
	mask->high_event_mask = cpu_to_le32(0xFFFFFFFF);
	mask->event_mask = cpu_to_le32(event_mask);

	ret = wlcore_cmd_configure(wl, ACX_EVENT_MBOX_MASK,
				   mask, sizeof(*mask));
	if (ret < 0) {
		wlcore_warning("failed to set acx_event_mbox_mask: %d", ret);
		goto out;
	}

out:
	kfree(mask);
	return ret;
}

int wlcore_acx_mem_cfg(struct wlcore *wl)
{
	struct acx_config_memory *mem_conf;
	struct wlcore_conf_hw_mem *mem = &wl->conf->hw_mem;
	int ret;

	wlcore_debug(DEBUG_ACX, "wlcore mem cfg");

	mem_conf = kzalloc(sizeof(*mem_conf), GFP_KERNEL);
	if (!mem_conf) {
		ret = -ENOMEM;
		goto out;
	}

	/* memory config */
	mem_conf->num_stations = mem->num_stations;
	mem_conf->rx_mem_block_num = mem->rx_block_num;
	mem_conf->tx_min_mem_block_num = mem->tx_min_block_num;
	mem_conf->num_ssid_profiles = mem->ssid_profiles;
	mem_conf->total_tx_descriptors =
		cpu_to_le32(mem->num_tx_descriptors);
	mem_conf->dyn_mem_enable = mem->dynamic_memory;
	mem_conf->tx_free_req = mem->min_req_tx_blocks;
	mem_conf->rx_free_req = mem->min_req_rx_blocks;
	mem_conf->tx_min = mem->tx_min;
	/* TODO: add correct number of fwlog mem_blocks */
	mem_conf->fwlog_blocks = 0;

	ret = wlcore_cmd_configure(wl, ACX_MEM_CFG, mem_conf,
				   sizeof(*mem_conf));
	if (ret < 0) {
		wlcore_warning("wlcore mem config failed: %d", ret);
		goto out;
	}
out:
	kfree(mem_conf);
	return ret;
}

int wlcore_acx_get_mem_map(struct wlcore *wl,
			   struct wlcore_acx_mem_map *map)
{
	int ret;

	wlcore_debug(DEBUG_ACX, "acx get mem map");

	ret = wlcore_cmd_interrogate(wl, ACX_MEM_MAP, wl->mem_map,
				     sizeof(*wl->mem_map));
	if (ret < 0) {
		wlcore_error("couldn't retrieve firmware memory map");
		goto out;
	}

	/* initialize TX block book keeping */
	wl->tx_blocks_available = le32_to_cpu(wl->mem_map->num_tx_mem_blocks);
	wlcore_debug(DEBUG_TX, "available tx blocks: %d",
		     wl->tx_blocks_available);

out:
	return ret;
}

int wlcore_acx_tx_config(struct wlcore *wl)
{
	struct acx_tx_config_options *acx;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx tx config");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->complete_timeout = cpu_to_le16(wl->conf->tx.complete_timeout);
	acx->complete_threshold = cpu_to_le16(wl->conf->tx.complete_threshold);
	ret = wlcore_cmd_configure(wl, ACX_TX_CONFIG_OPT, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set tx config: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_cca_threshold(struct wlcore *wl)
{
	struct acx_cca_threshold *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx cca threshold");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->rx_cca_threshold = 0; /* deprecated? */
	acx->tx_energy_detection = wl->conf->tx.energy_detection;

	ret = wlcore_cmd_configure(wl, ACX_CCA_THRESHOLD,
				   acx, sizeof(*acx));
	if (ret < 0)
		wlcore_warning("failed to set cca threshold: %d", ret);

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_frag_threshold(struct wlcore *wl, u32 frag_threshold)
{
	struct acx_frag_threshold *acx;
	int ret = 0;

	/* if fragmentation is not configured or out of range, use the max */
	if (frag_threshold > IEEE80211_MAX_FRAG_THRESHOLD)
		frag_threshold = IEEE80211_MAX_FRAG_THRESHOLD;

	wlcore_debug(DEBUG_ACX, "acx frag threshold: %d", frag_threshold);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->frag_threshold = cpu_to_le16((u16)frag_threshold);
	ret = wlcore_cmd_configure(wl, ACX_FRAG_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("Setting of frag threshold failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_rx_msdu_lifetime(struct wlcore *wl)
{
	struct acx_rx_msdu_lifetime *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx rx msdu lifetime");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->lifetime = cpu_to_le32(wl->conf->rx.msdu_lifetime);
	ret = wlcore_cmd_configure(wl, ACX_DOT11_RX_MSDU_LIFE_TIME,
				   acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set rx msdu lifetime: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_rx_irq_config(struct wlcore *wl)
{
	struct acx_rx_irq_config *rx_conf;
	int ret;

	wlcore_debug(DEBUG_ACX, "wlcore rx irq config");

	rx_conf = kzalloc(sizeof(*rx_conf), GFP_KERNEL);
	if (!rx_conf) {
		ret = -ENOMEM;
		goto out;
	}

	rx_conf->threshold = cpu_to_le16(wl->conf->rx.irq_pkt_threshold);
	rx_conf->timeout = cpu_to_le16(wl->conf->rx.irq_timeout);
	rx_conf->mblk_threshold = cpu_to_le16(wl->conf->rx.irq_blk_threshold);
	rx_conf->queue_type = wl->conf->rx.queue_type;

	ret = wlcore_cmd_configure(wl, ACX_RX_CONFIG_OPT, rx_conf,
				   sizeof(*rx_conf));
	if (ret < 0) {
		wlcore_warning("wlcore rx irq config failed: %d", ret);
		goto out;
	}

out:
	kfree(rx_conf);
	return ret;
}

int wlcore_acx_sleep_auth(struct wlcore *wl, u8 sleep_auth)
{
	struct acx_sleep_auth *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx sleep auth");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->sleep_auth = sleep_auth;

	ret = wlcore_cmd_configure(wl, ACX_SLEEP_AUTH, acx, sizeof(*acx));

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_sta_rate_policies(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	struct acx_rate_policy *acx;
	struct wlcore_conf_tx_rate_class *c = &wl->conf->tx.sta_rc;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx rate policies");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	wlcore_debug(DEBUG_ACX, "basic_rate: 0x%x, full_rate: 0x%x",
		wlvif->basic_rate, wlvif->rate_set);

	/* configure one basic rate class */
	acx->rate_policy_idx = cpu_to_le32(wlvif->sta.basic_rate_idx);
	acx->rate_policy.enabled_rates = cpu_to_le32(wlvif->basic_rate);
	acx->rate_policy.short_retry_limit = c->short_retry_limit;
	acx->rate_policy.long_retry_limit = c->long_retry_limit;
	acx->rate_policy.aflags = c->aflags;

	ret = wlcore_cmd_configure(wl, ACX_RATE_POLICY, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("Setting of rate policies failed: %d", ret);
		goto out;
	}

	/* configure one AP supported rate class */
	acx->rate_policy_idx = cpu_to_le32(wlvif->sta.ap_rate_idx);
	acx->rate_policy.enabled_rates = cpu_to_le32(wlvif->rate_set);
	acx->rate_policy.short_retry_limit = c->short_retry_limit;
	acx->rate_policy.long_retry_limit = c->long_retry_limit;
	acx->rate_policy.aflags = c->aflags;

	ret = wlcore_cmd_configure(wl, ACX_RATE_POLICY, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("Setting of rate policies failed: %d", ret);
		goto out;
	}

	/*
	 * configure one rate class for basic p2p operations.
	 * (p2p packets should always go out with OFDM rates, even
	 * if we are currently connected to 11b AP)
	 */
	acx->rate_policy_idx = cpu_to_le32(wlvif->sta.p2p_rate_idx);
	acx->rate_policy.enabled_rates =
				cpu_to_le32(WLCORE_TX_RATE_MASK_BASIC_P2P);
	acx->rate_policy.short_retry_limit = c->short_retry_limit;
	acx->rate_policy.long_retry_limit = c->long_retry_limit;
	acx->rate_policy.aflags = c->aflags;

	ret = wlcore_cmd_configure(wl, ACX_RATE_POLICY, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("Setting of rate policies failed: %d", ret);
		goto out;
	}

	/* TODO: add op to configure HW specific rate policies */

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_config_ps(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	struct acx_config_ps *config_ps;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx config ps");

	config_ps = kzalloc(sizeof(*config_ps), GFP_KERNEL);
	if (!config_ps) {
		ret = -ENOMEM;
		goto out;
	}

	config_ps->exit_retries = wl->conf->conn.psm_exit_retries;
	config_ps->enter_retries = wl->conf->conn.psm_entry_retries;
	config_ps->null_data_rate = cpu_to_le32(wlvif->basic_rate);

	ret = wlcore_cmd_configure(wl, ACX_CONFIG_PS, config_ps,
				   sizeof(*config_ps));

	if (ret < 0) {
		wlcore_warning("acx config ps failed: %d", ret);
		goto out;
	}

out:
	kfree(config_ps);
	return ret;
}

int wlcore_acx_group_address_tbl(struct wlcore *wl, struct wlcore_vif *wlvif,
				 bool enable, void *mc_list, u32 mc_list_len)
{
	struct acx_dot11_grp_addr_tbl *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx group address tbl");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	/* MAC filtering */
	acx->role_id = wlvif->role_id;
	acx->enabled = enable;
	acx->num_groups = mc_list_len;
	memcpy(acx->mac_table, mc_list, mc_list_len * ETH_ALEN);

	ret = wlcore_cmd_configure(wl, ACX_DOT11_GROUP_ADDRESS_TBL,
				   acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set group addr table: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_conn_monit_params(struct wlcore *wl, struct wlcore_vif *wlvif,
				 bool enable)
{
	struct acx_conn_monit_params *acx;
	u32 threshold = ACX_CONN_MONIT_DISABLE_VALUE;
	u32 timeout = ACX_CONN_MONIT_DISABLE_VALUE;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx connection monitor parameters: %s",
		     enable ? "enabled" : "disabled");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	if (enable) {
		threshold = wl->conf->conn.sync_fail_threshold;
		timeout = wl->conf->conn.bss_lose_timeout;
	}

	acx->role_id = wlvif->role_id;
	acx->sync_fail_threshold = cpu_to_le32(threshold);
	acx->bss_lose_timeout = cpu_to_le32(timeout);

	ret = wlcore_cmd_configure(wl, ACX_CONN_MONIT_PARAMS,
				   acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set connection monitor "
			       "parameters: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_beacon_filter_table(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	struct acx_beacon_filter_ie_table *ie_table;
	int i, idx = 0;
	int ret;
	bool vendor_spec = false;

	wlcore_debug(DEBUG_ACX, "acx beacon filter table %d rules",
		     wl->conf->conn.bcn_filter_count);

	ie_table = kzalloc(sizeof(*ie_table), GFP_KERNEL);
	if (!ie_table) {
		ret = -ENOMEM;
		goto out;
	}

	/* configure default beacon pass-through rules */
	ie_table->role_id = wlvif->role_id;
	ie_table->num_ie = 0;
	for (i = 0; i < wl->conf->conn.bcn_filter_count; i++) {
		struct wlcore_conf_bcn_filter *r =
			&(wl->conf->conn.bcn_filter[i]);
		ie_table->table[idx++] = r->ie;
		ie_table->table[idx++] = r->rule;

		if (r->ie == WLAN_EID_VENDOR_SPECIFIC) {
			/* TODO: check this, seems that 6 vendor IEs are ok */
			/* only one vendor specific ie allowed */
			if (vendor_spec)
				continue;

			/* for vendor specific rules configure the
			   additional fields */
			memcpy(&(ie_table->table[idx]), r->oui,
			       CONF_BCN_IE_OUI_LEN);
			idx += CONF_BCN_IE_OUI_LEN;
			ie_table->table[idx++] = r->type;
			memcpy(&(ie_table->table[idx]), r->version,
			       CONF_BCN_IE_VER_LEN);
			idx += CONF_BCN_IE_VER_LEN;
			vendor_spec = true;
		}
		/* TODO: check max number of non-vendor IEs too? */

		ie_table->num_ie++;
	}

	ret = wlcore_cmd_configure(wl, ACX_BEACON_FILTER_TABLE,
				   ie_table, sizeof(*ie_table));
	if (ret < 0) {
		wlcore_warning("failed to set beacon filter table: %d", ret);
		goto out;
	}

out:
	kfree(ie_table);
	return ret;
}

int wlcore_acx_beacon_filter_opt(struct wlcore *wl, struct wlcore_vif *wlvif,
				 bool enable_filter)
{
	struct acx_beacon_filter_option *beacon_filter = NULL;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx beacon filter opt %s",
		     enable_filter ? "enable" : "disable");

	if (enable_filter && !wl->conf->conn.enable_bcn_filter)
		goto out;

	beacon_filter = kzalloc(sizeof(*beacon_filter), GFP_KERNEL);
	if (!beacon_filter) {
		ret = -ENOMEM;
		goto out;
	}

	beacon_filter->role_id = wlvif->role_id;
	beacon_filter->enable = enable_filter;

	/*
	 * When set to zero, and the filter is enabled, beacons
	 * without the unicast TIM bit set are dropped.
	 */
	beacon_filter->max_num_beacons = 0;

	ret = wlcore_cmd_configure(wl, ACX_BEACON_FILTER_OPT,
				   beacon_filter, sizeof(*beacon_filter));
	if (ret < 0) {
		wlcore_warning("failed to set beacon filter opt: %d", ret);
		goto out;
	}

out:
	kfree(beacon_filter);
	return ret;
}

int wlcore_acx_pm_config(struct wlcore *wl)
{
	struct acx_pm_config *acx;
	struct wlcore_conf_pm *c = &wl->conf->pm_config;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx pm config");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->host_clk_settling_time = cpu_to_le32(c->host_clk_settling_time);
	acx->host_fast_wakeup_support = c->host_fast_wakeup_support;

	ret = wlcore_cmd_configure(wl, ACX_PM_CONFIG, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("acx pm config failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_set_rate_mgmt_params(struct wlcore *wl)
{
	struct acx_set_rate_mgmt_params *acx;
	struct wlcore_conf_rate_policy *conf = &wl->conf->rate;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx set rate mgmt params");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->index = ACX_RATE_MGMT_ALL_PARAMS;
	acx->rate_retry_score = cpu_to_le16(conf->rate_retry_score);
	acx->per_add = cpu_to_le16(conf->per_add);
	acx->per_th1 = cpu_to_le16(conf->per_th1);
	acx->per_th2 = cpu_to_le16(conf->per_th2);
	acx->max_per = cpu_to_le16(conf->max_per);
	acx->inverse_curiosity_factor = conf->inverse_curiosity_factor;
	acx->tx_fail_low_th = conf->tx_fail_low_th;
	acx->tx_fail_high_th = conf->tx_fail_high_th;
	acx->per_alpha_shift = conf->per_alpha_shift;
	acx->per_add_shift = conf->per_add_shift;
	acx->per_beta1_shift = conf->per_beta1_shift;
	acx->per_beta2_shift = conf->per_beta2_shift;
	acx->rate_check_up = conf->rate_check_up;
	acx->rate_check_down = conf->rate_check_down;
	memcpy(acx->rate_retry_policy, conf->rate_retry_policy,
	       sizeof(acx->rate_retry_policy));

	ret = wlcore_cmd_configure(wl, ACX_SET_RATE_MGMT_PARAMS,
				   acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("acx set rate mgmt params failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_config_hangover(struct wlcore *wl)
{
	struct acx_config_hangover *acx;
	struct wlcore_conf_hangover *conf = &wl->conf->hangover;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx config hangover");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->recover_time = cpu_to_le32(conf->recover_time);
	acx->hangover_period = conf->hangover_period;
	acx->dynamic_mode = conf->dynamic_mode;
	acx->early_termination_mode = conf->early_termination_mode;
	acx->max_period = conf->max_period;
	acx->min_period = conf->min_period;
	acx->increase_delta = conf->increase_delta;
	acx->decrease_delta = conf->decrease_delta;
	acx->quiet_time = conf->quiet_time;
	acx->increase_time = conf->increase_time;
	acx->window_size = acx->window_size;

	ret = wlcore_cmd_configure(wl, ACX_CONFIG_HANGOVER, acx,
				   sizeof(*acx));

	if (ret < 0) {
		wlcore_warning("acx config hangover failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_bcn_dtim_options(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	struct acx_beacon_broadcast *bb;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx bcn dtim options");

	bb = kzalloc(sizeof(*bb), GFP_KERNEL);
	if (!bb) {
		ret = -ENOMEM;
		goto out;
	}

	bb->role_id = wlvif->role_id;
	bb->beacon_rx_timeout = cpu_to_le16(wl->conf->conn.beacon_rx_timeout);
	bb->broadcast_timeout = cpu_to_le16(wl->conf->conn.broadcast_timeout);
	bb->rx_broadcast_in_ps = wl->conf->conn.rx_broadcast_in_ps;
	bb->ps_poll_threshold = wl->conf->conn.ps_poll_threshold;

	ret = wlcore_cmd_configure(wl, ACX_BCN_DTIM_OPTIONS, bb, sizeof(*bb));
	if (ret < 0) {
		wlcore_warning("failed to set rx config: %d", ret);
		goto out;
	}

out:
	kfree(bb);
	return ret;
}

int wlcore_acx_rssi_snr_avg_weights(struct wlcore *wl,
				    struct wlcore_vif *wlvif)
{
	struct acx_rssi_snr_avg_weights *acx = NULL;
	struct wlcore_conf_roam_trigger *c = &wl->conf->roam_trigger;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx rssi snr avg weights");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->rssi_beacon = c->avg_weight_rssi_beacon;
	acx->rssi_data = c->avg_weight_rssi_data;
	acx->snr_beacon = c->avg_weight_snr_beacon;
	acx->snr_data = c->avg_weight_snr_data;

	ret = wlcore_cmd_configure(wl, ACX_RSSI_SNR_WEIGHTS, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("acx rssi snr trigger weights failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}
