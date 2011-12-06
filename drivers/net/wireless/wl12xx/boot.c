/*
 * This file is part of wl1271
 *
 * Copyright (C) 2008-2010 Nokia Corporation
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

#include <linux/slab.h>
#include <linux/wl12xx.h>

#include "acx.h"
#include "reg.h"
#include "boot.h"
#include "io.h"
#include "event.h"
#include "rx.h"


static struct wl1271_partition_set part_table[PART_TABLE_LEN] = {
	[PART_TOP_PRCM_ELP_SOC] = {
		.mem = {
			.start = 0x00A02000,
			.size  = 0x00010000
		},
		.reg = {
			.start = 0x00807000,
			.size  = 0x00005000
		},
		.mem2 = {
			.start = 0x00800000,
			.size  = 0x0000B000
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_DOWN] = {
		.mem = {
			.start = 0x00000000,
			.size  = 0x00014000
		},
		.reg = {
			.start = 0x00810000,
			.size  = 0x0000BFFF
		},
		.mem2 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_WORK] = {
		.mem = {
			.start = REGISTERS_BASE,
			.size  = 0x000050FC
		},
		.reg = {
			.start = 0x00B00404,
			.size  = 0x00001000
		},
		.mem2 = {
			.start = 0x00C00000,
			.size  = 0x00000400
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_BOOT] = {
		.mem = {
			.start = 0x00700000,
			.size  = 0x0000030c
		},
		.reg = {
			.start = 0x00802000,
			.size  = 0x00014578
		},
		.mem2 = {
			.start = 0x00B00404,
			.size  = 0x00001000
		},
		.mem3 = {
			.start = 0x00C00000,
			.size  = 0x00000400
		}
	},

	[PART_PHY_INIT] = {
			.mem = {
				.start = PHY_INIT_MEM_ADDR,
				.size  = sizeof(struct conf_mac_and_phy_params)
			},
			.reg = {
				.start = 0x00000000,
				.size  = 0x00000000
			},
			.mem2 = {
				.start = 0x00000000,
				.size  = 0x00000000
			},
			.mem3 = {
				.start = 0x00000000,
				.size  = 0x00000000
					}
	},
	[PART_PHY_PDSP_WA] = {
			.mem = {
				.start = 0x00940100,
				.size  = 0x00000200
			},
			.reg = {
				.start = 0x00953000,
				.size  = 0x00012000
			},
			.mem2 = {
				.start = 0x00000000,
				.size  = 0x00000000
			},
			.mem3 = {
				.start = 0x00000000,
				.size  = 0x00000000
					}
	}
};

static void wl1271_boot_set_ecpu_ctrl(struct wl1271 *wl, u32 flag)
{
	u32 cpu_ctrl;

	/* 10.5.0 run the firmware (I) */
	cpu_ctrl = wl1271_read32(wl, ACX_REG_ECPU_CONTROL);

	/* 10.5.1 run the firmware (II) */
	cpu_ctrl |= flag;
	wl1271_write32(wl, ACX_REG_ECPU_CONTROL, cpu_ctrl);
}

static void wl1271_parse_fw_ver(struct wl1271 *wl)
{
	int ret;

	/* parse MAC FW version */
	ret = sscanf(wl->chip.fw_ver_str + 4, "%u.%u.%u.%u.%u",
		     &wl->chip.fw_ver[0], &wl->chip.fw_ver[1],
		     &wl->chip.fw_ver[2], &wl->chip.fw_ver[3],
		     &wl->chip.fw_ver[4]);

	if (ret != 5) {
		wl1271_warning("fw version incorrect value");
		memset(wl->chip.fw_ver, 0, sizeof(wl->chip.fw_ver));
		return;
	}

	/* parse PHY FW version */
    ret = sscanf(wl->chip.phy_fw_ver_str + 4, "%u.%u.%u.%u.%u",
		     &wl->chip.phy_fw_ver[0], &wl->chip.phy_fw_ver[1],
		     &wl->chip.phy_fw_ver[2], &wl->chip.phy_fw_ver[3],
		     &wl->chip.phy_fw_ver[4]);

	if (ret != 5) {
		wl1271_warning("PHY fw version incorrect value");
		memset(wl->chip.phy_fw_ver, 0, sizeof(wl->chip.phy_fw_ver));
		return;
	}
}

static void wl1271_boot_fw_version(struct wl1271 *wl)
{
	struct wl1271_static_data static_data;

	wl1271_read(wl, wl->cmd_box_addr, &static_data, sizeof(static_data),
		    false);

	strncpy(wl->chip.fw_ver_str, static_data.fw_version,
		sizeof(wl->chip.fw_ver_str));
	strncpy(wl->chip.phy_fw_ver_str, static_data.phy_fw_version,
		sizeof(wl->chip.phy_fw_ver_str));

	/* make sure the string is NULL-terminated */
	wl->chip.fw_ver_str[sizeof(wl->chip.fw_ver_str) - 1] = '\0';
	wl->chip.phy_fw_ver_str[sizeof(wl->chip.phy_fw_ver_str) - 1] = '\0';

	wl1271_parse_fw_ver(wl);
}

static int wl1271_boot_upload_firmware_chunk(struct wl1271 *wl, void *buf,
					     size_t fw_data_len, u32 dest)
{
	struct wl1271_partition_set partition;
	int addr, chunk_num, partition_limit;
	u8 *p, *chunk;

	/* whal_FwCtrl_LoadFwImageSm() */

	wl1271_debug(DEBUG_BOOT, "starting firmware upload");

	wl1271_debug(DEBUG_BOOT, "fw_data_len %zd chunk_size %d",
		     fw_data_len, CHUNK_SIZE);

	if ((fw_data_len % 4) != 0) {
		wl1271_error("firmware length not multiple of four");
		return -EIO;
	}

	chunk = kmalloc(CHUNK_SIZE, GFP_KERNEL);
	if (!chunk) {
		wl1271_error("allocation for firmware upload chunk failed");
		return -ENOMEM;
	}

	memcpy(&partition, &part_table[PART_DOWN], sizeof(partition));
	partition.mem.start = dest;
	wl1271_set_partition(wl, &partition);

	/* 10.1 set partition limit and chunk num */
	chunk_num = 0;
	partition_limit = part_table[PART_DOWN].mem.size;

	while (chunk_num < fw_data_len / CHUNK_SIZE) {
		/* 10.2 update partition, if needed */
		addr = dest + (chunk_num + 2) * CHUNK_SIZE;
		if (addr > partition_limit) {
			addr = dest + chunk_num * CHUNK_SIZE;
			partition_limit = chunk_num * CHUNK_SIZE +
				part_table[PART_DOWN].mem.size;
			partition.mem.start = addr;
			wl1271_set_partition(wl, &partition);
		}

		/* 10.3 upload the chunk */
		addr = dest + chunk_num * CHUNK_SIZE;
		p = buf + chunk_num * CHUNK_SIZE;
		memcpy(chunk, p, CHUNK_SIZE);
		wl1271_debug(DEBUG_BOOT, "uploading fw chunk 0x%p to 0x%x",
			     p, addr);
		wl1271_write(wl, addr, chunk, CHUNK_SIZE, false);

		chunk_num++;
	}

	/* 10.4 upload the last chunk */
	addr = dest + chunk_num * CHUNK_SIZE;
	p = buf + chunk_num * CHUNK_SIZE;
	memcpy(chunk, p, fw_data_len % CHUNK_SIZE);
	wl1271_debug(DEBUG_BOOT, "uploading fw last chunk (%zd B) 0x%p to 0x%x",
		     fw_data_len % CHUNK_SIZE, p, addr);
	wl1271_write(wl, addr, chunk, fw_data_len % CHUNK_SIZE, false);

	kfree(chunk);
	return 0;
}

static int wl1271_boot_upload_firmware(struct wl1271 *wl)
{
	u32 chunks, addr, len;
	int ret = 0;
	u8 *fw;

	fw = wl->fw;
	chunks = be32_to_cpup((__be32 *) fw);
	fw += sizeof(u32);

	wl1271_debug(DEBUG_BOOT, "firmware chunks to be uploaded: %u", chunks);

	while (chunks--) {
		addr = be32_to_cpup((__be32 *) fw);
		fw += sizeof(u32);
		len = be32_to_cpup((__be32 *) fw);
		fw += sizeof(u32);

		if (len > 300000) {
			wl1271_info("firmware chunk too long: %u", len);
			return -EINVAL;
		}
		wl1271_debug(DEBUG_BOOT, "chunk %d addr 0x%x len %u",
			     chunks, addr, len);
		ret = wl1271_boot_upload_firmware_chunk(wl, fw, len, addr);
		if (ret != 0)
			break;
		fw += len;
	}

	return ret;
}

static int wl1271_boot_upload_nvs(struct wl1271 *wl)
{
	size_t nvs_len, burst_len;
	int i;
	u32 dest_addr, val;
	u8 *nvs_ptr, *nvs_aligned;

	if (wl->nvs == NULL)
		return -ENODEV;

	if ((wl->chip.id == CHIP_ID_1283_PG20) ||
		(wl->chip.id == CHIP_ID_185x_PG10) ||
		(wl->chip.id == CHIP_ID_185x_PG20))  {
		struct wl128x_nvs_file *nvs = (struct wl128x_nvs_file *)wl->nvs;

		if (wl->nvs_len == sizeof(struct wl128x_nvs_file)) {
			if (nvs->general_params.dual_mode_select)
				wl->enable_11a = true;
		} else {
			wl1271_error("nvs size is not as expected: %zu != %zu",
				     wl->nvs_len,
				     sizeof(struct wl128x_nvs_file));
			kfree(wl->nvs);
			wl->nvs = NULL;
			wl->nvs_len = 0;
			return -EILSEQ;
		}

		/* only the first part of the NVS needs to be uploaded */
		nvs_len = sizeof(nvs->nvs);
		nvs_ptr = (u8 *)nvs->nvs;

	} else {
		struct wl1271_nvs_file *nvs =
			(struct wl1271_nvs_file *)wl->nvs;
		/*
		 * FIXME: the LEGACY NVS image support (NVS's missing the 5GHz
		 * band configurations) can be removed when those NVS files stop
		 * floating around.
		 */
		if (wl->nvs_len == sizeof(struct wl1271_nvs_file) ||
		    wl->nvs_len == WL1271_INI_LEGACY_NVS_FILE_SIZE) {
			if (nvs->general_params.dual_mode_select)
				wl->enable_11a = true;
		}

		if (wl->nvs_len != sizeof(struct wl1271_nvs_file) &&
		    (wl->nvs_len != WL1271_INI_LEGACY_NVS_FILE_SIZE ||
		     wl->enable_11a)) {
			wl1271_error("nvs size is not as expected: %zu != %zu",
				wl->nvs_len, sizeof(struct wl1271_nvs_file));
			kfree(wl->nvs);
			wl->nvs = NULL;
			wl->nvs_len = 0;
			return -EILSEQ;
		}

		/* only the first part of the NVS needs to be uploaded */
		nvs_len = sizeof(nvs->nvs);
		nvs_ptr = (u8 *) nvs->nvs;
	}

	/* update current MAC address to NVS */
	nvs_ptr[11] = wl->mac_addr[0];
	nvs_ptr[10] = wl->mac_addr[1];
	nvs_ptr[6] = wl->mac_addr[2];
	nvs_ptr[5] = wl->mac_addr[3];
	nvs_ptr[4] = wl->mac_addr[4];
	nvs_ptr[3] = wl->mac_addr[5];

	/*
	 * Layout before the actual NVS tables:
	 * 1 byte : burst length.
	 * 2 bytes: destination address.
	 * n bytes: data to burst copy.
	 *
	 * This is ended by a 0 length, then the NVS tables.
	 */

	/* FIXME: Do we need to check here whether the LSB is 1? */
	while (nvs_ptr[0]) {
		burst_len = nvs_ptr[0];
		dest_addr = (nvs_ptr[1] & 0xfe) | ((u32)(nvs_ptr[2] << 8));

		/*
		 * Due to our new wl1271_translate_reg_addr function,
		 * we need to add the REGISTER_BASE to the destination
		 */
		dest_addr += REGISTERS_BASE;

		/* We move our pointer to the data */
		nvs_ptr += 3;

		for (i = 0; i < burst_len; i++) {
			val = (nvs_ptr[0] | (nvs_ptr[1] << 8)
			       | (nvs_ptr[2] << 16) | (nvs_ptr[3] << 24));

			wl1271_debug(DEBUG_BOOT,
				     "nvs burst write 0x%x: 0x%x",
				     dest_addr, val);
			wl1271_write32(wl, dest_addr, val);

			nvs_ptr += 4;
			dest_addr += 4;
		}
	}

	/*
	 * We've reached the first zero length, the first NVS table
	 * is located at an aligned offset which is at least 7 bytes further.
	 * NOTE: The wl->nvs->nvs element must be first, in order to
	 * simplify the casting, we assume it is at the beginning of
	 * the wl->nvs structure.
	 */
	nvs_ptr = (u8 *)wl->nvs +
			ALIGN(nvs_ptr - (u8 *)wl->nvs + 7, 4);
	nvs_len -= nvs_ptr - (u8 *)wl->nvs;

	/* Now we must set the partition correctly */
	wl1271_set_partition(wl, &part_table[PART_WORK]);

	/* Copy the NVS tables to a new block to ensure alignment */
	nvs_aligned = kmemdup(nvs_ptr, nvs_len, GFP_KERNEL);
	if (!nvs_aligned)
		return -ENOMEM;

	/* And finally we upload the NVS tables */
	wl1271_write(wl, CMD_MBOX_ADDRESS, nvs_aligned, nvs_len, false);

	kfree(nvs_aligned);
	return 0;
}

static void wl1271_boot_enable_interrupts(struct wl1271 *wl)
{
	wl1271_enable_interrupts(wl);
	wl1271_write32(wl, ACX_REG_INTERRUPT_MASK,
		       WL1271_ACX_INTR_ALL & ~(WL1271_INTR_MASK));
#if 0
	wl1271_write32(wl, HI_CFG, HI_CFG_DEF_VAL);
#endif
}

static int wl1271_boot_soft_reset(struct wl1271 *wl)
{
#if 0
	unsigned long timeout;
	u32 boot_data;

	/* perform soft reset */
	wl1271_write32(wl, ACX_REG_SLV_SOFT_RESET, ACX_SLV_SOFT_RESET_BIT);

	/* SOFT_RESET is self clearing */
	timeout = jiffies + usecs_to_jiffies(SOFT_RESET_MAX_TIME);
	while (1) {
		boot_data = wl1271_read32(wl, ACX_REG_SLV_SOFT_RESET);
		wl1271_debug(DEBUG_BOOT, "soft reset bootdata 0x%x", boot_data);
		if ((boot_data & ACX_SLV_SOFT_RESET_BIT) == 0)
			break;

		if (time_after(jiffies, timeout)) {
			/* 1.2 check pWhalBus->uSelfClearTime if the
			 * timeout was reached */
			wl1271_error("soft reset timeout");
			return -1;
		}

		udelay(SOFT_RESET_STALL_TIME);
	}
#endif
	/* disable Rx/Tx */
	wl1271_write32(wl, ENABLE, 0x0);

	/* disable auto calibration on start*/
	wl1271_write32(wl, SPARE_A2, 0xffff);

	return 0;
}

static int wl1271_boot_run_firmware(struct wl1271 *wl)
{
	int loop, ret;
	u32 chip_id, intr;

	wl1271_set_partition(wl, &part_table[PART_BOOT]);

	wl1271_boot_set_ecpu_ctrl(wl, ECPU_CONTROL_HALT);

	chip_id = wl1271_read32(wl, CHIP_ID_B);

	wl1271_debug(DEBUG_BOOT, "chip id after firmware boot: 0x%x", chip_id);

	if (chip_id != wl->chip.id) {
		wl1271_error("chip id doesn't match after firmware boot");
		return -EIO;
	}

	/* wait for init to complete */
	loop = 0;
	while (loop++ < INIT_LOOP) {
		udelay(INIT_LOOP_DELAY);
		intr = wl1271_read32(wl, ACX_REG_INTERRUPT_NO_CLEAR);

		if (intr == 0xffffffff) {
			wl1271_error("error reading hardware complete "
				     "init indication");
			return -EIO;
		}
		/* check that ACX_INTR_INIT_COMPLETE is enabled */
		else if (intr & WL1271_ACX_INTR_INIT_COMPLETE) {
			wl1271_write32(wl, ACX_REG_INTERRUPT_ACK,
				       WL1271_ACX_INTR_INIT_COMPLETE);
			break;
		}
	}

	if (loop > INIT_LOOP) {
		wl1271_error("timeout waiting for the hardware to "
			     "complete initialization");
		return -EIO;
	}

	wl1271_info("Init Completed");

	/* get hardware config command mail box */
	wl->cmd_box_addr = wl1271_read32(wl, REG_COMMAND_MAILBOX_PTR);

	/* get hardware config event mail box */
	wl->event_box_addr = wl1271_read32(wl, REG_EVENT_MAILBOX_PTR);

	wl1271_debug(DEBUG_MAILBOX, "cmd_box_addr 0x%x event_box_addr 0x%x",
				 wl->cmd_box_addr, wl->event_box_addr);

	wl1271_boot_fw_version(wl);

	/*
	 * in case of full asynchronous mode the firmware event must be
	 * ready to receive event from the command mailbox
	 */

	/* TODO: when removing a role, mask the appropriate events */

	/* unmask required mbox events  */
	wl->event_mask = BSS_LOSE_EVENT_ID |
		SCAN_COMPLETE_EVENT_ID |
		PS_REPORT_EVENT_ID |
		DISCONNECT_EVENT_COMPLETE_ID |
		RSSI_SNR_TRIGGER_0_EVENT_ID |
		PSPOLL_DELIVERY_FAILURE_EVENT_ID |
		SOFT_GEMINI_SENSE_EVENT_ID |
		CHANGE_AUTO_MODE_TIMEOUT_EVENT_ID |
		PERIODIC_SCAN_REPORT_EVENT_ID |
		PERIODIC_SCAN_COMPLETE_EVENT_ID |
		DUMMY_PACKET_EVENT_ID |
		REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID |
		BA_SESSION_RX_CONSTRAINT_EVENT_ID;

	/* TODO: mode can change dynamically. make it more sane */
	wl->event_mask |= PEER_REMOVE_COMPLETE_EVENT_ID;

	/* TODO: there's a bug in MAX_TX_RETRY_EVENT_ID in STA mode */
	if (wl->bss_type == BSS_TYPE_AP_BSS)
		wl->event_mask |= MAX_TX_RETRY_EVENT_ID |
				  INACTIVE_STA_EVENT_ID;
	else
		wl->event_mask |= CHANNEL_SWITCH_COMPLETE_EVENT_ID;

	ret = wl1271_event_unmask(wl);
	if (ret < 0) {
		wl1271_error("EVENT mask setting failed");
		return ret;
	}

	wl1271_event_mbox_config(wl);

	/* firmware startup completed */
	return 0;
}

#if 0
static int wl1271_boot_write_irq_polarity(struct wl1271 *wl)
{
	u32 polarity;

	polarity = wl1271_top_reg_read(wl, OCP_REG_POLARITY);

	/* We use HIGH polarity, so unset the LOW bit */
	polarity &= ~POLARITY_LOW;
	wl1271_top_reg_write(wl, OCP_REG_POLARITY, polarity);

	return 0;
}
#endif

static void wl1271_boot_hw_version(struct wl1271 *wl)
{
	u32 fuse;

	if ((wl->chip.id == CHIP_ID_185x_PG10) ||
		(wl->chip.id == CHIP_ID_185x_PG20)) {
        wl1271_set_partition(wl, &part_table[PART_TOP_PRCM_ELP_SOC]);
        fuse = wl1271_read32(wl, WL18XX_REG_FUSE_DATA_1_3);
		wl1271_set_partition(wl, &part_table[PART_BOOT]);
        fuse = (fuse & WL18XX_PG_VER_MASK) >> WL18XX_PG_VER_OFFSET;
	}
	else if (wl->chip.id == CHIP_ID_1283_PG20) {
        fuse = wl1271_top_reg_read(wl, WL128X_REG_FUSE_DATA_2_1);
		fuse = (fuse & WL12XX_PG_VER_MASK) >> WL12XX_PG_VER_OFFSET;
	}
	else {
		fuse = wl1271_top_reg_read(wl, WL127X_REG_FUSE_DATA_2_1);
		fuse = (fuse & WL12XX_PG_VER_MASK) >> WL12XX_PG_VER_OFFSET;
	}

	wl->hw_pg_ver = (s8)fuse;
}

static int wl128x_switch_tcxo_to_fref(struct wl1271 *wl)
{
	u16 spare_reg;

	/* Mask bits [2] & [8:4] in the sys_clk_cfg register */
	spare_reg = wl1271_top_reg_read(wl, WL_SPARE_REG);
	if (spare_reg == 0xFFFF)
		return -EFAULT;
	spare_reg |= (BIT(3) | BIT(5) | BIT(6));
	wl1271_top_reg_write(wl, WL_SPARE_REG, spare_reg);

	/* Enable FREF_CLK_REQ & mux MCS and coex PLLs to FREF */
	wl1271_top_reg_write(wl, SYS_CLK_CFG_REG,
			     WL_CLK_REQ_TYPE_PG2 | MCS_PLL_CLK_SEL_FREF);

	/* Delay execution for 15msec, to let the HW settle */
	mdelay(15);

	return 0;
}

static bool wl128x_is_tcxo_valid(struct wl1271 *wl)
{
	u16 tcxo_detection;

	tcxo_detection = wl1271_top_reg_read(wl, TCXO_CLK_DETECT_REG);
	if (tcxo_detection & TCXO_DET_FAILED)
		return false;

	return true;
}

static bool wl128x_is_fref_valid(struct wl1271 *wl)
{
	u16 fref_detection;

	fref_detection = wl1271_top_reg_read(wl, FREF_CLK_DETECT_REG);
	if (fref_detection & FREF_CLK_DETECT_FAIL)
		return false;

	return true;
}

static int wl128x_manually_configure_mcs_pll(struct wl1271 *wl)
{
	wl1271_top_reg_write(wl, MCS_PLL_M_REG, MCS_PLL_M_REG_VAL);
	wl1271_top_reg_write(wl, MCS_PLL_N_REG, MCS_PLL_N_REG_VAL);
	wl1271_top_reg_write(wl, MCS_PLL_CONFIG_REG, MCS_PLL_CONFIG_REG_VAL);

	return 0;
}

static int wl128x_configure_mcs_pll(struct wl1271 *wl, int clk)
{
	u16 spare_reg;
	u16 pll_config;
	u8 input_freq;

	/* Mask bits [3:1] in the sys_clk_cfg register */
	spare_reg = wl1271_top_reg_read(wl, WL_SPARE_REG);
	if (spare_reg == 0xFFFF)
		return -EFAULT;
	spare_reg |= BIT(2);
	wl1271_top_reg_write(wl, WL_SPARE_REG, spare_reg);

	/* Handle special cases of the TCXO clock */
	if (wl->tcxo_clock == WL12XX_TCXOCLOCK_16_8 ||
	    wl->tcxo_clock == WL12XX_TCXOCLOCK_33_6)
		return wl128x_manually_configure_mcs_pll(wl);

	/* Set the input frequency according to the selected clock source */
	input_freq = (clk & 1) + 1;

	pll_config = wl1271_top_reg_read(wl, MCS_PLL_CONFIG_REG);
	if (pll_config == 0xFFFF)
		return -EFAULT;
	pll_config |= (input_freq << MCS_SEL_IN_FREQ_SHIFT);
	pll_config |= MCS_PLL_ENABLE_HP;
	wl1271_top_reg_write(wl, MCS_PLL_CONFIG_REG, pll_config);

	return 0;
}

static int wl18xx_boot_clk(struct wl1271 *wl)
{
	u32 platform_type;
	u32 osc_en;
    u32 clk_type;
	u32 clk_freq;
	u32 pllsh_wcs_pll_N = 0;
	u32 pllsh_wcs_pll_M = 0;
	u32 pllsh_wcs_pll_Q = 0;
	u32 pllsh_wcs_pll_P = 0;

	switch (wl->conf.mac_and_phy_params.hw_board_type)
	{
	case BOARD_TYPE_FPGA_18XX:
		{
			wl1271_info("FPGA board configured (skipping top init)");
			wl1271_write32(wl, SCR_PAD2, 0xB1);
			break;
		}
	case BOARD_TYPE_HDK_18XX:
		{
			wl1271_info("HDK board configured");
			wl1271_write32(wl, SCR_PAD2,0xB2);
			break;
		}
	case BOARD_TYPE_DVP_EVB_18XX:
		{
			wl1271_info("DVP/EVB board configured");
			wl1271_write32(wl, SCR_PAD2, 0xB3);
			break;
		}
	}

    if (wl->conf.mac_and_phy_params.hw_board_type != BOARD_TYPE_FPGA_18XX)
    {
        wl1271_info("Starting Init");
        wl1271_set_partition(wl, &part_table[PART_TOP_PRCM_ELP_SOC]);

		/* 1. Platform detection */

		/* Read platform type */
		platform_type = wl1271_top_reg_read(wl, PLATFORM_DETECTION);

		/* Platform detection */
		switch (platform_type) {
		case 0:
			wl1271_info("ORCA (6450) chipset Detected");
			break;
		case 1:
			wl1271_info("TRIO (1273) chipset Detected");
			break;
		case 2:
			wl1271_info("NL5500 chipset Detected");
			break;
		case 3:
			wl1271_info("NAPOLEON chipset Detected");
			break;
		case 4:
			wl1271_info("QUATTRO (1283) chipset Detected");
			break;
		case 5:
			wl1271_info("185X chipset Detected");
			break;
		case 6:
			wl1271_info("189X chipset Detected");
			break;
		default:
			wl1271_error("Unknown chipset Detected");
		}

		/* 2. CLK detection */
		wl1271_debug(DEBUG_BOOT, "CLK detection");
		/* Read CLK type from detection (for PG2) - TCXO/FREF or XTAL */
		osc_en = wl1271_top_reg_read(wl, OSC_EN);
		wl1271_debug(DEBUG_BOOT, "osc_en = 0x%x", osc_en);

		/* Check the clock source in bit 3 from OSC_EN */
		if (!(osc_en & PRCM_WLAN_CLK_DETECTION_MASK)) {
			wl1271_debug(DEBUG_BOOT, "CLK detection bit 3 is clear (OSC_EN=0x%x), working with TCXO/FREF clock", osc_en);
			clk_type = FREF;

		} else {
			if (platform_type == 5) /* platform_type = 185x */ {
				wl1271_debug(DEBUG_BOOT, "185x Platform Detected, CLK Type Is XTAL");
				clk_type = XTAL;
			} else {               /* platform_type = 189x */
				wl1271_error("189x Platform Detected, ERROR");
				return -EINVAL;
			}
		}

		/* Read freq from detection */
		clk_freq = wl1271_top_reg_read(wl, PRIMARY_CLK_DETECT);
		wl1271_debug(DEBUG_BOOT, "Read from addr 0x%x clock freq %d", PRIMARY_CLK_DETECT, clk_freq);

		/* 3. WCS PLL Config */
		wl1271_debug(DEBUG_BOOT, "WCS PLL Config");

		switch (clk_freq) {
		case CLOCK_CONFIG_16_2_M:
			pllsh_wcs_pll_N = 7;
			pllsh_wcs_pll_M = 104;
			pllsh_wcs_pll_Q = 801;
			pllsh_wcs_pll_P = 4;
			break;
		case CLOCK_CONFIG_16_368_M:
			pllsh_wcs_pll_N = 9;
			pllsh_wcs_pll_M = 132;
			pllsh_wcs_pll_Q = 3751;
			pllsh_wcs_pll_P = 4;
			break;
		case CLOCK_CONFIG_16_8_M:
			pllsh_wcs_pll_N = 7;
			pllsh_wcs_pll_M = 100;
			break;
		case CLOCK_CONFIG_19_2_M:
			pllsh_wcs_pll_N = 8;
			pllsh_wcs_pll_M = 100;
			break;
		case CLOCK_CONFIG_26_M:
			pllsh_wcs_pll_N = 13;
			pllsh_wcs_pll_M = 120;
			break;
		case CLOCK_CONFIG_32_736_M:
			pllsh_wcs_pll_N = 9;
			pllsh_wcs_pll_M = 132;
			pllsh_wcs_pll_Q = 3751;
			pllsh_wcs_pll_P = 4;
			break;
		case CLOCK_CONFIG_33_6_M:
			pllsh_wcs_pll_N = 7;
			pllsh_wcs_pll_M = 100;
			break;
		case CLOCK_CONFIG_38_468_M:
			pllsh_wcs_pll_N = 8;
			pllsh_wcs_pll_M = 100;
			break;
		case CLOCK_CONFIG_52_M:
			pllsh_wcs_pll_N = 13;
			pllsh_wcs_pll_M = 120;
			break;
		default:
			wl1271_error("Unknown clock frequency 0x%x", clk_freq);
		}

		/* Config N (pre divider) parameters according to the input frequency */
		wl1271_debug(DEBUG_BOOT, "write to addr 0x%x Config N %d", PLLSH_WCS_PLL_N, pllsh_wcs_pll_N);
		wl1271_top_reg_write(wl, PLLSH_WCS_PLL_N, pllsh_wcs_pll_N);
	
		/* Config M (divider) parameters according to the input frequency */
		wl1271_debug(DEBUG_BOOT, "write to addr 0x%x Config M %d", PLLSH_WCS_PLL_M, pllsh_wcs_pll_M);
		wl1271_top_reg_write(wl, PLLSH_WCS_PLL_M, pllsh_wcs_pll_M);
	
		/* Swallowing is only needed for the following CLK frequencies:
			16.2MHz, 16.368MHz, 32.736MHz */
		if ((clk_freq == CLOCK_CONFIG_16_2_M) ||
			(clk_freq == CLOCK_CONFIG_16_368_M) ||
			(clk_freq == CLOCK_CONFIG_32_736_M)) {
	
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_Q_FACTOR_CFG_1,
					(pllsh_wcs_pll_Q & PLLSH_WCS_PLL_Q_FACTOR_CFG_1_MASK));
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_Q_FACTOR_CFG_2,
					((pllsh_wcs_pll_Q >> REG_16_SHIFT) & PLLSH_WCS_PLL_Q_FACTOR_CFG_2_MASK));
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_P_FACTOR_CFG_1,
					(pllsh_wcs_pll_P & PLLSH_WCS_PLL_P_FACTOR_CFG_1_MASK));
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_P_FACTOR_CFG_2,
					((pllsh_wcs_pll_P >> REG_16_SHIFT) & PLLSH_WCS_PLL_P_FACTOR_CFG_2_MASK));
	
			/* Activate swallowing mechanism if needed */
			wl1271_debug(DEBUG_BOOT, "Activate swallowing mechanism %d", PLLSH_WCS_PLL_SWALLOW_EN_VAL1);
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_SWALLOW_EN, PLLSH_WCS_PLL_SWALLOW_EN_VAL1);
		}
		else {
			/* Activate swallowing mechanism if needed */
			wl1271_debug(DEBUG_BOOT, "Activate swallowing mechanism %d in addr 0x%x",
					PLLSH_WCS_PLL_SWALLOW_EN_VAL2, PLLSH_WCS_PLL_SWALLOW_EN);
			wl1271_top_reg_write(wl, PLLSH_WCS_PLL_SWALLOW_EN, PLLSH_WCS_PLL_SWALLOW_EN_VAL2);
		}
	
		wl1271_debug(DEBUG_BOOT, "HW TOP init is done!!!");
	}

	return 0;
}

/*
 * WL128x has two clocks input - TCXO and FREF.
 * TCXO is the main clock of the device, while FREF is used to sync
 * between the GPS and the cellular modem.
 * In cases where TCXO is 32.736MHz or 16.368MHz, the FREF will be used
 * as the WLAN/BT main clock.
 */
static int wl128x_boot_clk(struct wl1271 *wl, int *selected_clock)
{
	u16 sys_clk_cfg;

	/* For XTAL-only modes, FREF will be used after switching from TCXO */
	if (wl->ref_clock == WL12XX_REFCLOCK_26_XTAL ||
	    wl->ref_clock == WL12XX_REFCLOCK_38_XTAL) {
		if (!wl128x_switch_tcxo_to_fref(wl))
			return -EINVAL;
		goto fref_clk;
	}

	/* Query the HW, to determine which clock source we should use */
	sys_clk_cfg = wl1271_top_reg_read(wl, SYS_CLK_CFG_REG);
	if (sys_clk_cfg == 0xFFFF)
		return -EINVAL;
	if (sys_clk_cfg & PRCM_CM_EN_MUX_WLAN_FREF)
		goto fref_clk;

	/* If TCXO is either 32.736MHz or 16.368MHz, switch to FREF */
	if (wl->tcxo_clock == WL12XX_TCXOCLOCK_16_368 ||
	    wl->tcxo_clock == WL12XX_TCXOCLOCK_32_736) {
		if (!wl128x_switch_tcxo_to_fref(wl))
			return -EINVAL;
		goto fref_clk;
	}

	/* TCXO clock is selected */
	if (!wl128x_is_tcxo_valid(wl))
		return -EINVAL;
	*selected_clock = wl->tcxo_clock;
	goto config_mcs_pll;

fref_clk:
	/* FREF clock is selected */
	if (!wl128x_is_fref_valid(wl))
		return -EINVAL;
	*selected_clock = wl->ref_clock;

config_mcs_pll:
	return wl128x_configure_mcs_pll(wl, *selected_clock);
}

static int wl127x_boot_clk(struct wl1271 *wl)
{
	u32 pause;
	u32 clk;

	if (((wl->hw_pg_ver & PG_MAJOR_VER_MASK) >> PG_MAJOR_VER_OFFSET) < 3)
		wl->quirks |= WL12XX_QUIRK_END_OF_TRANSACTION;

	if (wl->ref_clock == CONF_REF_CLK_19_2_E ||
	    wl->ref_clock == CONF_REF_CLK_38_4_E ||
	    wl->ref_clock == CONF_REF_CLK_38_4_M_XTAL)
		/* ref clk: 19.2/38.4/38.4-XTAL */
		clk = 0x3;
	else if (wl->ref_clock == CONF_REF_CLK_26_E ||
		 wl->ref_clock == CONF_REF_CLK_52_E)
		/* ref clk: 26/52 */
		clk = 0x5;
	else
		return -EINVAL;

	if (wl->ref_clock != CONF_REF_CLK_19_2_E) {
		u16 val;
		/* Set clock type (open drain) */
		val = wl1271_top_reg_read(wl, OCP_REG_CLK_TYPE);
		val &= FREF_CLK_TYPE_BITS;
		wl1271_top_reg_write(wl, OCP_REG_CLK_TYPE, val);

		/* Set clock pull mode (no pull) */
		val = wl1271_top_reg_read(wl, OCP_REG_CLK_PULL);
		val |= NO_PULL;
		wl1271_top_reg_write(wl, OCP_REG_CLK_PULL, val);
	} else {
		u16 val;
		/* Set clock polarity */
		val = wl1271_top_reg_read(wl, OCP_REG_CLK_POLARITY);
		val &= FREF_CLK_POLARITY_BITS;
		val |= CLK_REQ_OUTN_SEL;
		wl1271_top_reg_write(wl, OCP_REG_CLK_POLARITY, val);
	}

	wl1271_write32(wl, PLL_PARAMETERS, clk);

	pause = wl1271_read32(wl, PLL_PARAMETERS);

	wl1271_debug(DEBUG_BOOT, "pause1 0x%x", pause);

	pause &= ~(WU_COUNTER_PAUSE_VAL);
	pause |= WU_COUNTER_PAUSE_VAL;
	wl1271_write32(wl, WU_COUNTER_PAUSE, pause);

	return 0;
}

/* Summarizing yield issue:
Bug description: Some PDSP Regs wake up value is such that OCP bridge is stuck, this happened randomly
WA: toggle BT and PHY reset till OCP is free, PDSP registers value is valid */
static int wl18xx_pdsp_wa_sequence(struct wl1271 *wl)
{
	int i;
	u32 ocp_state;
	u32 val;

	/* PHY Reset */
	for (i = 0; i < 10; i++) {

        wl1271_set_partition(wl, &part_table[PART_PHY_PDSP_WA]);

		ocp_state = wl1271_read32(wl, PHY_HRAM_RD_EN_PER_RAM);

		/* Check OCP state */
		if (ocp_state == 0x3F) {
			goto toggle_pdsp_reset;
		}

		wl1271_set_partition(wl, &part_table[PART_TOP_PRCM_ELP_SOC]);
		wl1271_top_reg_write(wl, 0xA021FE, 0x400);  /* WL PHY off*/
		wl1271_top_reg_write(wl, 0xA021FC, 0x400);  /* PHY on */
		wl1271_top_reg_write(wl, 0xA021FE, 0);      /* remove override - turn off BT */
		wl1271_top_reg_write(wl, 0xA021FC, 0);      /* remove override - turn off BT */
	}

	/* BT Reset */
	for (i = 10; i < 20; i++) {

		wl1271_set_partition(wl, &part_table[PART_PHY_PDSP_WA]);

		ocp_state = wl1271_read32(wl, PHY_HRAM_RD_EN_PER_RAM);

		/* Check OCP state */
		if (ocp_state == 0x3F) {
			goto toggle_pdsp_reset;
		}

		wl1271_set_partition(wl, &part_table[PART_TOP_PRCM_ELP_SOC]);
		val = wl1271_top_reg_read(wl, PRCM_BT_PWR_RST);

		/* check if BT is disabled */
		if (val == 0) {
			wl1271_top_reg_write(wl, 0xA021FE, 0x400);  /* WL PHY off*/
			wl1271_top_reg_write(wl, 0xA021FC, 0x7800); /* BT enable */
			wl1271_top_reg_write(wl, 0xA021FE, 0x7C00); /* BT enable */
			wl1271_top_reg_write(wl, 0xA021FC, 0x7C00); /* PHY on */
			wl1271_top_reg_write(wl, 0xA021FE, 0);      /* remove override - turn off BT */
			wl1271_top_reg_write(wl, 0xA021FC, 0);      /* remove override - turn off BT */
		}
	}

	wl1271_set_partition(wl, &part_table[PART_PHY_PDSP_WA]);
	ocp_state = wl1271_read32(wl, PHY_HRAM_RD_EN_PER_RAM);

	/* Check OCP state */
	if (ocp_state != 0x3F) {
		if (val == 0) {
			wl1271_error("Init error #2 - Please reset device");
		}
		else
		{
			wl1271_error("Init error #1 - Please reset device");
		}
		return -1;
	}

toggle_pdsp_reset:
	/* toggle PDSP reset*/
	wl1271_write32(wl, PDSP_CONTROL_REG, 0x00000007);
	wl1271_write32(wl, PDSP_CONTROL_REG, 0x00000000);
	wl1271_write32(wl, PDSP_CONTROL_REG, 0x00000007);
	wl1271_write32(wl, PDSP_CONTROL_REG, 0x00000000);

	/* Validate PDSP Reset */
	wl1271_write32(wl, FDSP_RAM, 0xA5A5A5A5);
	val = wl1271_read32(wl, FDSP_RAM);
	if (val == 0xA5A5A5A5) {
		goto out;
	}

	wl1271_error("Init error #3 - Please reset device");
	return -1;

out:
	wl1271_set_partition(wl, &part_table[PART_BOOT]);

	return 0;
}

/* uploads NVS and firmware */
int wl1271_load_firmware(struct wl1271 *wl)
{
	int ret = 0;
	u32 tmp;
	int selected_clock = -1;

	wl1271_boot_hw_version(wl);

	if ((wl->chip.id == CHIP_ID_185x_PG10) ||
		(wl->chip.id == CHIP_ID_185x_PG20)) {
		ret = wl18xx_boot_clk(wl);
		if (ret < 0)
			goto out;

		/* PDSP work around sequence for Yield issue */
		if (wl->chip.id == CHIP_ID_185x_PG10) {
			ret = wl18xx_pdsp_wa_sequence(wl);
			if (ret < 0)
				goto out;
		}
	}
	else if (wl->chip.id == CHIP_ID_1283_PG20) {
		ret = wl128x_boot_clk(wl, &selected_clock);
		if (ret < 0)
			goto out;
	} else {
		ret = wl127x_boot_clk(wl);
		if (ret < 0)
			goto out;
	}

	/* Continue the ELP wake up sequence */
	wl1271_write32(wl, WELP_ARM_COMMAND, WELP_ARM_COMMAND_VAL);
	udelay(500);
	wl1271_set_partition(wl, &part_table[PART_BOOT]);

	/* Disable interrupts */
	wl1271_write32(wl, ACX_REG_INTERRUPT_MASK, WL1271_ACX_INTR_ALL);

	ret = wl1271_boot_soft_reset(wl);
	if (ret < 0)
		goto out;

	/* 2. start processing NVS file */
	ret = wl1271_boot_upload_nvs(wl);
	if (ret < 0)
		goto out;

	/* write firmware's last address (ie. it's length) to
	 * ACX_EEPROMLESS_IND_REG */
	wl1271_set_partition(wl, &part_table[PART_BOOT]);
	wl1271_debug(DEBUG_BOOT, "ACX_EEPROMLESS_IND_REG");

	wl1271_write32(wl, ACX_EEPROMLESS_IND_REG, ACX_EEPROMLESS_IND_REG);

	tmp = wl1271_read32(wl, CHIP_ID_B);

	wl1271_debug(DEBUG_BOOT, "chip id 0x%x", tmp);

	/* 6. read the EEPROM parameters */
	tmp = wl1271_read32(wl, SCR_PAD2);

	/* WL1271: The reference driver skips steps 7 to 10 (jumps directly
	 * to upload_fw) */

	if (wl->chip.id == CHIP_ID_1283_PG20)
		wl1271_top_reg_write(wl, SDIO_IO_DS, wl->conf.hci_io_ds);

	ret = wl1271_boot_upload_firmware(wl);
	if (ret < 0)
		goto out;

out:
	return ret;
}
EXPORT_SYMBOL_GPL(wl1271_load_firmware);

int wl1271_boot(struct wl1271 *wl)
{
	int ret;

	/* upload NVS and firmware */
	ret = wl1271_load_firmware(wl);
	if (ret)
		return ret;

	wl1271_set_partition(wl, &part_table[PART_PHY_INIT]);
	wl1271_write(wl, PHY_INIT_MEM_ADDR ,(u8*)&wl->conf.mac_and_phy_params, sizeof (wl->conf.mac_and_phy_params), false);

	/* 10.5 start firmware */
	ret = wl1271_boot_run_firmware(wl);
	if (ret < 0)
		goto out;

#if 0
	ret = wl1271_boot_write_irq_polarity(wl);
	if (ret < 0)
		goto out;
#endif

	wl1271_write32(wl, ACX_REG_INTERRUPT_MASK,
		       WL1271_ACX_ALL_EVENTS_VECTOR);

	/* Enable firmware interrupts now */
	wl1271_boot_enable_interrupts(wl);

	/* set the wl1271 default filters */
	wl1271_set_default_filters(wl);

	wl1271_event_mbox_config(wl);

	/* set the working partition to its "running" mode offset */
	wl1271_set_partition(wl, &part_table[PART_WORK]);

out:
	return ret;
}
