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

static struct wl1271_partition_set wl12xx_part_table[PART_TABLE_LEN] = {
	[PART_DOWN] = {
		.mem = {
			.start = 0x00000000,
			.size  = 0x000177c0
		},
		.reg = {
			.start = WL12XX_REGISTERS_BASE,
			.size  = 0x00008800
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
			.start = 0x00040000,
			.size  = 0x00014fc0
		},
		.reg = {
			.start = WL12XX_REGISTERS_BASE,
			.size  = 0x0000a000
		},
		.mem2 = {
			.start = 0x003004f8,
			.size  = 0x00000004
		},
		.mem3 = {
			.start = 0x00040404,
			.size  = 0x00000000
		},
	},

	[PART_DRPW] = {
		.mem = {
			.start = 0x00040000,
			.size  = 0x00014fc0
		},
		.reg = {
			.start = WL12XX_DRPW_BASE,
			.size  = 0x00006000
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

static struct wl1271_partition_set wl18xx_part_table[PART_TABLE_LEN] = {
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
			.start = WL18XX_REGISTERS_BASE,
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
			.start = WL18XX_REGISTERS_BASE,
			.size  = 0x000050FC
		},
		.reg = {
			.start = 0x00B00404,
			.size  = 0x00001000
		},
		.mem2 = {
			.start = WL18XX_HOST_BASE,
			.size  = 0x00000400
		},
		.mem3 = {
			.start = 0x00000000,
			.size  = 0x00000000
		},
	},

	[PART_BOOT] = {
		.mem = {
			.start = WL18XX_MCU_KEY_SEARCH_BASE,
			.size  = 0x0000030c
		},
		.reg = {
			.start = WL18XX_REG_BOOT_PART_START,
			.size  = WL18XX_REG_BOOT_PART_SIZE
		},
		.mem2 = {
			.start = 0x00B00404,
			.size  = 0x00001000
		},
		.mem3 = {
			.start = WL18XX_HOST_BASE,
			.size  = 0x00000400
		}
	},

	[PART_PHY_INIT] = {
		.mem = {
			.start = WL18XX_PHY_INIT_MEM_ADDR,
			.size  = sizeof(struct wl18xx_mac_and_phy_params)
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
	}
};


static void wl1271_boot_set_ecpu_ctrl(struct wl1271 *wl, u32 flag)
{
	u32 cpu_ctrl;

	/* 10.5.0 run the firmware (I) */
	if (wl->conf.platform_type == 1) {
		cpu_ctrl = wl1271_read32(wl, WL12XX_ACX_REG_ECPU_CONTROL);

		/* 10.5.1 run the firmware (II) */
		cpu_ctrl |= flag;
		wl1271_write32(wl, WL12XX_ACX_REG_ECPU_CONTROL, cpu_ctrl);
	} else {
		cpu_ctrl = wl1271_read32(wl, WL18XX_ACX_REG_ECPU_CONTROL);

		/* 10.5.1 run the firmware (II) */
		cpu_ctrl |= flag;
		wl1271_write32(wl, WL18XX_ACX_REG_ECPU_CONTROL, cpu_ctrl);
	}
}

static unsigned int wl12xx_get_fw_ver_quirks(struct wl1271 *wl)
{
	unsigned int quirks = 0;
	unsigned int *fw_ver = wl->chip.fw_ver;

	if (wl->conf.platform_type == 1) {
		/* Only new station firmwares support routing fw logs to the host */
		if ((fw_ver[FW_VER_IF_TYPE] == FW_VER_IF_TYPE_STA) &&
		    (fw_ver[FW_VER_MINOR] < FW_VER_MINOR_FWLOG_STA_MIN))
			quirks |= WL12XX_QUIRK_FWLOG_NOT_IMPLEMENTED;

		/* This feature is not yet supported for AP mode */
		if (fw_ver[FW_VER_IF_TYPE] == FW_VER_IF_TYPE_AP)
			quirks |= WL12XX_QUIRK_FWLOG_NOT_IMPLEMENTED;
	}

	return quirks;
}

static void wl1271_parse_fw_ver(struct wl1271 *wl)
{
	int ret;

	ret = sscanf(wl->chip.fw_ver_str + 4, "%u.%u.%u.%u.%u",
		     &wl->chip.fw_ver[0], &wl->chip.fw_ver[1],
		     &wl->chip.fw_ver[2], &wl->chip.fw_ver[3],
		     &wl->chip.fw_ver[4]);

	if (ret != 5) {
		wl1271_warning("fw version incorrect value");
		memset(wl->chip.fw_ver, 0, sizeof(wl->chip.fw_ver));
		return;
	}

	/* Check if any quirks are needed with older fw versions */
	wl->quirks |= wl12xx_get_fw_ver_quirks(wl);
}

static void wl1271_boot_fw_version(struct wl1271 *wl)
{
	struct wl1271_static_data static_data;

	wl1271_read(wl, wl->cmd_box_addr, &static_data, sizeof(static_data),
		    false);

	strncpy(wl->chip.fw_ver_str, static_data.fw_version,
		sizeof(wl->chip.fw_ver_str));

	/* make sure the string is NULL-terminated */
	wl->chip.fw_ver_str[sizeof(wl->chip.fw_ver_str) - 1] = '\0';

	wl1271_parse_fw_ver(wl);
}

static int wl1271_boot_upload_firmware_chunk(struct wl1271 *wl, void *buf,
					     size_t fw_data_len, u32 dest)
{
	struct wl1271_partition_set partition;
	int addr, chunk_num, partition_limit;
	u8 *p, *chunk;
	struct wl1271_partition_set *part_table;

	if (wl->conf.platform_type == 1)
		part_table = wl12xx_part_table;
	else
		part_table = wl18xx_part_table;

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

	if (wl->chip.id == CHIP_ID_1283_PG20 || wl->conf.platform_type == 2) {
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
		if (wl->conf.platform_type == 1)
			dest_addr += WL12XX_REGISTERS_BASE;
		else
			dest_addr += WL18XX_REGISTERS_BASE;

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

	/* 18xxTODO: make an op for set_partition */
	/* Now we must set the partition correctly */
	if (wl->conf.platform_type == 1)
		wl1271_set_partition(wl, &wl12xx_part_table[PART_WORK]);
	else
		wl1271_set_partition(wl, &wl18xx_part_table[PART_WORK]);

	/* Copy the NVS tables to a new block to ensure alignment */
	nvs_aligned = kmemdup(nvs_ptr, nvs_len, GFP_KERNEL);
	if (!nvs_aligned)
		return -ENOMEM;

	/* And finally we upload the NVS tables */
	if (wl->conf.platform_type == 1)
		wl1271_write(wl, WL12XX_CMD_MBOX_ADDRESS, nvs_aligned, nvs_len, false);
	else
		wl1271_write(wl, WL18XX_CMD_MBOX_ADDRESS, nvs_aligned, nvs_len, false);

	kfree(nvs_aligned);
	return 0;
}

static void wl1271_boot_enable_interrupts(struct wl1271 *wl)
{
	wl1271_enable_interrupts(wl);

	if (wl->conf.platform_type == 1) {
		wl1271_write32(wl, WL12XX_ACX_REG_INTERRUPT_MASK,
			       WL1271_ACX_INTR_ALL & ~(WL1271_INTR_MASK));
		wl1271_write32(wl, WL12XX_HI_CFG, HI_CFG_DEF_VAL);
	} else {
		wl1271_write32(wl, WL18XX_ACX_REG_INTERRUPT_MASK,
			       WL1271_ACX_INTR_ALL & ~(WL1271_INTR_MASK));
	}
}

static int wl1271_boot_soft_reset(struct wl1271 *wl)
{
	unsigned long timeout;
	u32 boot_data;

	if (wl->conf.platform_type == 1) {
		/* perform soft reset */
		wl1271_write32(wl, WL12XX_ACX_REG_SLV_SOFT_RESET, ACX_SLV_SOFT_RESET_BIT);

		/* SOFT_RESET is self clearing */
		timeout = jiffies + usecs_to_jiffies(SOFT_RESET_MAX_TIME);
		while (1) {
			boot_data = wl1271_read32(wl, WL12XX_ACX_REG_SLV_SOFT_RESET);
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
		/* disable Rx/Tx */
		wl1271_write32(wl, WL12XX_ENABLE, 0x0);

		/* disable auto calibration on start*/
		wl1271_write32(wl, WL12XX_SPARE_A2, 0xffff);
	} else {
		/* disable Rx/Tx */
		wl1271_write32(wl, WL18XX_ENABLE, 0x0);

		/* disable auto calibration on start*/
		wl1271_write32(wl, WL18XX_SPARE_A2, 0xffff);
	}

	return 0;
}

static int wl1271_boot_run_firmware(struct wl1271 *wl)
{
	int loop, ret;
	u32 chip_id, intr;

	/* 18xxTODO: probably not needed */
	if (wl->conf.platform_type == 2)
		wl1271_set_partition(wl, &wl18xx_part_table[PART_BOOT]);

	wl1271_boot_set_ecpu_ctrl(wl, ECPU_CONTROL_HALT);

	/* 18xxTODO: should put read_chip_id as separate op */
	if (wl->conf.platform_type == 1)
		chip_id = wl1271_read32(wl, WL12XX_CHIP_ID_B);
	else
		chip_id = wl1271_read32(wl, WL18XX_CHIP_ID_B);

	wl1271_debug(DEBUG_BOOT, "chip id after firmware boot: 0x%x", chip_id);

	if (chip_id != wl->chip.id) {
		wl1271_error("chip id doesn't match after firmware boot");
		return -EIO;
	}

	/* wait for init to complete */
	loop = 0;
	while (loop++ < INIT_LOOP) {
		udelay(INIT_LOOP_DELAY);
		intr = PLAT_READ_REG32(wl, ACX_REG_INTERRUPT_NO_CLEAR);

		if (intr == 0xffffffff) {
			wl1271_error("error reading hardware complete "
				     "init indication");
			return -EIO;
		}
		/* check that ACX_INTR_INIT_COMPLETE is enabled */
		else if (intr & WL1271_ACX_INTR_INIT_COMPLETE) {
			PLAT_WRITE_REG32(wl, ACX_REG_INTERRUPT_ACK,
					 WL1271_ACX_INTR_INIT_COMPLETE);
			break;
		}
	}

	if (loop > INIT_LOOP) {
		wl1271_error("timeout waiting for the hardware to "
			     "complete initialization");
		return -EIO;
	}

	if (wl->conf.platform_type == 1) {
		/* get hardware config command mail box */
		wl->cmd_box_addr = wl1271_read32(wl, WL12XX_REG_COMMAND_MAILBOX_PTR);

		/* get hardware config event mail box */
		wl->event_box_addr = wl1271_read32(wl, WL12XX_REG_EVENT_MAILBOX_PTR);
	} else {
		/* get hardware config command mail box */
		wl->cmd_box_addr = wl1271_read32(wl, WL18XX_REG_COMMAND_MAILBOX_PTR);

		/* get hardware config event mail box */
		wl->event_box_addr = wl1271_read32(wl, WL18XX_REG_EVENT_MAILBOX_PTR);
	}

	/* set the working partition to its "running" mode offset */
	if (wl->conf.platform_type == 1)
		wl1271_set_partition(wl, &wl12xx_part_table[PART_WORK]);

	wl1271_debug(DEBUG_MAILBOX, "cmd_box_addr 0x%x event_box_addr 0x%x",
		     wl->cmd_box_addr, wl->event_box_addr);

	wl1271_boot_fw_version(wl);

	/*
	 * in case of full asynchronous mode the firmware event must be
	 * ready to receive event from the command mailbox
	 */

	/* unmask required mbox events  */
	wl->event_mask = BSS_LOSE_EVENT_ID |
		SCAN_COMPLETE_EVENT_ID |
		PS_REPORT_EVENT_ID |
		DISCONNECT_EVENT_COMPLETE_ID |
		RSSI_SNR_TRIGGER_0_EVENT_ID |
		PSPOLL_DELIVERY_FAILURE_EVENT_ID |
		SOFT_GEMINI_SENSE_EVENT_ID |
		PERIODIC_SCAN_REPORT_EVENT_ID |
		PERIODIC_SCAN_COMPLETE_EVENT_ID |
		DUMMY_PACKET_EVENT_ID |
		PEER_REMOVE_COMPLETE_EVENT_ID |
		BA_SESSION_RX_CONSTRAINT_EVENT_ID |
		REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID |
		INACTIVE_STA_EVENT_ID |
		MAX_TX_RETRY_EVENT_ID |
		CHANNEL_SWITCH_COMPLETE_EVENT_ID;

	ret = wl1271_event_unmask(wl);
	if (ret < 0) {
		wl1271_error("EVENT mask setting failed");
		return ret;
	}

	wl1271_event_mbox_config(wl);

	/* firmware startup completed */
	return 0;
}

static int wl1271_boot_write_irq_polarity(struct wl1271 *wl)
{
	u32 polarity;

	polarity = wl12xx_top_reg_read(wl, OCP_REG_POLARITY);

	/* We use HIGH polarity, so unset the LOW bit */
	polarity &= ~POLARITY_LOW;
	wl12xx_top_reg_write(wl, OCP_REG_POLARITY, polarity);

	return 0;
}

static void wl1271_boot_hw_version(struct wl1271 *wl)
{
	u32 fuse;

	if (wl->chip.id == CHIP_ID_1283_PG20)
		fuse = wl12xx_top_reg_read(wl, WL128X_REG_FUSE_DATA_2_1);
	else
		fuse = wl12xx_top_reg_read(wl, WL127X_REG_FUSE_DATA_2_1);
	fuse = (fuse & PG_VER_MASK) >> PG_VER_OFFSET;

	wl->hw_pg_ver = (s8)fuse;
}

static int wl128x_switch_tcxo_to_fref(struct wl1271 *wl)
{
	u16 spare_reg;

	/* Mask bits [2] & [8:4] in the sys_clk_cfg register */
	spare_reg = wl12xx_top_reg_read(wl, WL_SPARE_REG);
	if (spare_reg == 0xFFFF)
		return -EFAULT;
	spare_reg |= (BIT(3) | BIT(5) | BIT(6));
	wl12xx_top_reg_write(wl, WL_SPARE_REG, spare_reg);

	/* Enable FREF_CLK_REQ & mux MCS and coex PLLs to FREF */
	wl12xx_top_reg_write(wl, SYS_CLK_CFG_REG,
			     WL_CLK_REQ_TYPE_PG2 | MCS_PLL_CLK_SEL_FREF);

	/* Delay execution for 15msec, to let the HW settle */
	mdelay(15);

	return 0;
}

static bool wl128x_is_tcxo_valid(struct wl1271 *wl)
{
	u16 tcxo_detection;

	tcxo_detection = wl12xx_top_reg_read(wl, TCXO_CLK_DETECT_REG);
	if (tcxo_detection & TCXO_DET_FAILED)
		return false;

	return true;
}

static bool wl128x_is_fref_valid(struct wl1271 *wl)
{
	u16 fref_detection;

	fref_detection = wl12xx_top_reg_read(wl, FREF_CLK_DETECT_REG);
	if (fref_detection & FREF_CLK_DETECT_FAIL)
		return false;

	return true;
}

static int wl128x_manually_configure_mcs_pll(struct wl1271 *wl)
{
	wl12xx_top_reg_write(wl, MCS_PLL_M_REG, MCS_PLL_M_REG_VAL);
	wl12xx_top_reg_write(wl, MCS_PLL_N_REG, MCS_PLL_N_REG_VAL);
	wl12xx_top_reg_write(wl, MCS_PLL_CONFIG_REG, MCS_PLL_CONFIG_REG_VAL);

	return 0;
}

static int wl128x_configure_mcs_pll(struct wl1271 *wl, int clk)
{
	u16 spare_reg;
	u16 pll_config;
	u8 input_freq;

	/* Mask bits [3:1] in the sys_clk_cfg register */
	spare_reg = wl12xx_top_reg_read(wl, WL_SPARE_REG);
	if (spare_reg == 0xFFFF)
		return -EFAULT;
	spare_reg |= BIT(2);
	wl12xx_top_reg_write(wl, WL_SPARE_REG, spare_reg);

	/* Handle special cases of the TCXO clock */
	if (wl->tcxo_clock == WL12XX_TCXOCLOCK_16_8 ||
	    wl->tcxo_clock == WL12XX_TCXOCLOCK_33_6)
		return wl128x_manually_configure_mcs_pll(wl);

	/* Set the input frequency according to the selected clock source */
	input_freq = (clk & 1) + 1;

	pll_config = wl12xx_top_reg_read(wl, MCS_PLL_CONFIG_REG);
	if (pll_config == 0xFFFF)
		return -EFAULT;
	pll_config |= (input_freq << MCS_SEL_IN_FREQ_SHIFT);
	pll_config |= MCS_PLL_ENABLE_HP;
	wl12xx_top_reg_write(wl, MCS_PLL_CONFIG_REG, pll_config);

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
	sys_clk_cfg = wl12xx_top_reg_read(wl, SYS_CLK_CFG_REG);
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
		val = wl12xx_top_reg_read(wl, OCP_REG_CLK_TYPE);
		val &= FREF_CLK_TYPE_BITS;
		wl12xx_top_reg_write(wl, OCP_REG_CLK_TYPE, val);

		/* Set clock pull mode (no pull) */
		val = wl12xx_top_reg_read(wl, OCP_REG_CLK_PULL);
		val |= NO_PULL;
		wl12xx_top_reg_write(wl, OCP_REG_CLK_PULL, val);
	} else {
		u16 val;
		/* Set clock polarity */
		val = wl12xx_top_reg_read(wl, OCP_REG_CLK_POLARITY);
		val &= FREF_CLK_POLARITY_BITS;
		val |= CLK_REQ_OUTN_SEL;
		wl12xx_top_reg_write(wl, OCP_REG_CLK_POLARITY, val);
	}

	wl1271_write32(wl, WL12XX_PLL_PARAMETERS, clk);

	pause = wl1271_read32(wl, WL12XX_PLL_PARAMETERS);

	wl1271_debug(DEBUG_BOOT, "pause1 0x%x", pause);

	pause &= ~(WU_COUNTER_PAUSE_VAL);
	pause |= WU_COUNTER_PAUSE_VAL;
	wl1271_write32(wl, WL12XX_WU_COUNTER_PAUSE, pause);

	return 0;
}

static int wl18xx_boot_clk(struct wl1271 *wl, int *selected_clock)
{
	switch (wl->conf.subtype_18xx) {
	case BOARD_TYPE_FPGA_18XX:
		wl1271_info("wl18XX FPGA board");
		wl1271_write32(wl, WL18XX_SCR_PAD2, 0xB1);
		break;
	case BOARD_TYPE_HDK_18XX:
		wl1271_info("wl18XX HDK board");
		wl1271_write32(wl, WL18XX_SCR_PAD2,0xB2);
		break;
	case BOARD_TYPE_DVP_EVB_18XX:
		wl1271_info("wl18xx DVP/EVB board");
		wl1271_write32(wl, WL18XX_SCR_PAD2, 0xB3);
		break;
	default:
		wl1271_error("invalid board type");
		return -EINVAL;
	}

	if (wl->conf.subtype_18xx != BOARD_TYPE_FPGA_18XX) {
		wl1271_set_partition(wl,
				     &wl18xx_part_table[PART_TOP_PRCM_ELP_SOC]);
		wl1271_write32(wl, 0x00A02360, 0xD0078);
		wl1271_write32(wl, 0x00A0236c, 0x12);
		wl1271_write32(wl, 0x00A02390, 0x20118);
	}

	return 0;
}

/* 18xxTODO: probably need a separate function for 18xx here */
/* uploads NVS and firmware */
int wl1271_load_firmware(struct wl1271 *wl)
{
	int ret = 0;
	u32 tmp, clk;
	int selected_clock = -1;

	if (wl->conf.platform_type == 1) {
		wl1271_boot_hw_version(wl);

		if (wl->chip.id == CHIP_ID_1283_PG20) {
			ret = wl128x_boot_clk(wl, &selected_clock);
			if (ret < 0)
				goto out;
		} else {
			ret = wl127x_boot_clk(wl);
			if (ret < 0)
				goto out;
		}
	} else {
		ret = wl18xx_boot_clk(wl, &selected_clock);
		if (ret < 0)
			goto out;
	}

	/* Continue the ELP wake up sequence */
	if (wl->conf.platform_type == 1)
		wl1271_write32(wl, WL12XX_WELP_ARM_COMMAND, WELP_ARM_COMMAND_VAL);
	else
		wl1271_write32(wl, WL18XX_WELP_ARM_COMMAND, WELP_ARM_COMMAND_VAL);

	udelay(500);

	if (wl->conf.platform_type == 1) {
		wl1271_set_partition(wl, &wl12xx_part_table[PART_DRPW]);

		/* Read-modify-write DRPW_SCRATCH_START register (see next state)
		   to be used by DRPw FW. The RTRIM value will be added by the FW
		   before taking DRPw out of reset */

		wl1271_debug(DEBUG_BOOT, "DRPW_SCRATCH_START %08x", WL12XX_DRPW_SCRATCH_START);
		clk = wl1271_read32(wl, WL12XX_DRPW_SCRATCH_START);

		wl1271_debug(DEBUG_BOOT, "clk2 0x%x", clk);

		if (wl->chip.id == CHIP_ID_1283_PG20) {
			clk |= ((selected_clock & 0x3) << 1) << 4;
		} else {
			clk |= (wl->ref_clock << 1) << 4;
		}

		wl1271_write32(wl, WL12XX_DRPW_SCRATCH_START, clk);

		wl1271_set_partition(wl, &wl12xx_part_table[PART_WORK]);

		/* Disable interrupts */
		wl1271_write32(wl, WL12XX_ACX_REG_INTERRUPT_MASK, WL1271_ACX_INTR_ALL);
	} else {
		wl1271_set_partition(wl, &wl18xx_part_table[PART_BOOT]);

		/* Disable interrupts */
		wl1271_write32(wl, WL18XX_ACX_REG_INTERRUPT_MASK, WL1271_ACX_INTR_ALL);
	}

	ret = wl1271_boot_soft_reset(wl);
	if (ret < 0)
		goto out;

	/* 2. start processing NVS file */
	ret = wl1271_boot_upload_nvs(wl);
	if (ret < 0)
		goto out;

	/* write firmware's last address (ie. it's length) to
	 * ACX_EEPROMLESS_IND_REG */
	wl1271_debug(DEBUG_BOOT, "ACX_EEPROMLESS_IND_REG");

	if (wl->conf.platform_type == 1) {
		wl1271_write32(wl, WL12XX_ACX_EEPROMLESS_IND_REG, WL12XX_ACX_EEPROMLESS_IND_REG);
	} else {
		wl1271_set_partition(wl, &wl18xx_part_table[PART_BOOT]);
		wl1271_write32(wl, WL18XX_ACX_EEPROMLESS_IND_REG, WL18XX_ACX_EEPROMLESS_IND_REG);
	}

	/* 18xxTODO: probably not required */
	if (wl->conf.platform_type == 1)
		tmp = wl1271_read32(wl, WL12XX_CHIP_ID_B);
	else
		tmp = wl1271_read32(wl, WL18XX_CHIP_ID_B);

	wl1271_debug(DEBUG_BOOT, "chip id 0x%x", tmp);

	/* 6. read the EEPROM parameters */
	if (wl->conf.platform_type == 1)
		tmp = wl1271_read32(wl, WL12XX_SCR_PAD2);
	else
		tmp = wl1271_read32(wl, WL18XX_SCR_PAD2);

	/* WL1271: The reference driver skips steps 7 to 10 (jumps directly
	 * to upload_fw) */

	if (wl->chip.id == CHIP_ID_1283_PG20)
		wl12xx_top_reg_write(wl, SDIO_IO_DS, wl->conf.hci_io_ds);

	ret = wl1271_boot_upload_firmware(wl);
	if (ret < 0)
		goto out;

out:
	return ret;
}
/* 18xxTODO: no need for symbol export without sdio_test.c */
EXPORT_SYMBOL_GPL(wl1271_load_firmware);

void wl18xx_init_mac_and_phy_params(struct wl1271 *wl)
{
	struct wl18xx_mac_and_phy_params params;

	memset(&params, 0, sizeof(params));

	params.phy_standalone = wl->conf.phy_params.phy_standalone;
	params.rdl = wl->conf.phy_params.rdl;
	params.enable_clpc = wl->conf.phy_params.enable_clpc;
	params.enable_tx_low_pwr_on_siso_rdl =
		wl->conf.phy_params.enable_tx_low_pwr_on_siso_rdl;
	params.auto_detect = wl->conf.phy_params.auto_detect;
	params.dedicated_fem = wl->conf.phy_params.dedicated_fem;
	params.low_band_component = wl->conf.phy_params.low_band_component;
	params.low_band_component_type =
		wl->conf.phy_params.low_band_component_type;
	params.high_band_component = wl->conf.phy_params.high_band_component;
	params.high_band_component_type =
		wl->conf.phy_params.high_band_component_type;
	params.number_of_assembled_ant2_4 =
		wl->conf.phy_params.number_of_assembled_ant2_4;
	params.number_of_assembled_ant5 =
		wl->conf.phy_params.number_of_assembled_ant5;
	params.external_pa_dc2dc = wl->conf.phy_params.external_pa_dc2dc;
	params.tcxo_ldo_voltage = wl->conf.phy_params.tcxo_ldo_voltage;;
	params.xtal_itrim_val = wl->conf.phy_params.xtal_itrim_val;
	params.srf_state = wl->conf.phy_params.srf_state;
	params.io_configuration = wl->conf.phy_params.io_configuration;
	params.sdio_configuration = wl->conf.phy_params.sdio_configuration;
	params.settings = wl->conf.phy_params.settings;
	params.rx_profile = wl->conf.phy_params.rx_profile;
	params.primary_clock_setting_time =
		wl->conf.phy_params.primary_clock_setting_time;
	params.clock_valid_on_wake_up =
		wl->conf.phy_params.clock_valid_on_wake_up;
	params.secondary_clock_setting_time =
		wl->conf.phy_params.secondary_clock_setting_time;

	params.board_type = wl->conf.subtype_18xx;

	wl1271_set_partition(wl, &wl18xx_part_table[PART_PHY_INIT]);
	wl1271_write(wl, WL18XX_PHY_INIT_MEM_ADDR, (u8*)&params,
		     sizeof(params), false);

	/*
	 * 18xxTODO: should probably revert the set_partition thing, or
	 * do all set_partition in a single function. otherwise confusing.
	 */
}

int wl1271_boot(struct wl1271 *wl)
{
	int ret;

	/* upload NVS and firmware */
	ret = wl1271_load_firmware(wl);
	if (ret)
		return ret;

	if (wl->conf.platform_type == 2)
		wl18xx_init_mac_and_phy_params(wl);

	/* 10.5 start firmware */
	ret = wl1271_boot_run_firmware(wl);
	if (ret < 0)
		goto out;

	if (wl->conf.platform_type == 1) {
		ret = wl1271_boot_write_irq_polarity(wl);
		if (ret < 0)
			goto out;
	}

	/* 18xxTODO: should make a macro for writing a u32 to a WL12XX/WL18XX named reg */
	if (wl->conf.platform_type == 1)
		wl1271_write32(wl, WL12XX_ACX_REG_INTERRUPT_MASK, WL1271_ACX_ALL_EVENTS_VECTOR);
	else
		wl1271_write32(wl, WL18XX_ACX_REG_INTERRUPT_MASK, WL1271_ACX_ALL_EVENTS_VECTOR);

	/* Enable firmware interrupts now */
	wl1271_boot_enable_interrupts(wl);

	wl1271_event_mbox_config(wl);

	if (wl->conf.platform_type == 2)
		wl1271_set_partition(wl, &wl18xx_part_table[PART_WORK]);

out:
	return ret;
}
