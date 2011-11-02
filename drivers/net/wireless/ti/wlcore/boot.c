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

#include <linux/firmware.h>
#include <linux/vmalloc.h>

#include "wlcore.h"
#include "io.h"
#include "boot.h"
#include "debug.h"

static int wlcore_request_firmware(struct wlcore *wl)
{
	const struct firmware *fw;
	int ret = 0;

	wlcore_debug(DEBUG_BOOT, "requesting firmware %s", wl->fw_name);

	ret = request_firmware(&fw, wl->fw_name, wl->dev);
	if (ret < 0) {
		wlcore_error("could not get firmware: %d", ret);
		goto out;
	}

	if (fw->size % 4) {
		wlcore_error("firmware size is not multiple of 32 bits: %zu",
			     fw->size);
		ret = -EILSEQ;
		goto out_release;
	}

	vfree(wl->fw);
	wl->fw_len = 0;

	wl->fw = vmalloc(fw->size);
	if (!wl->fw) {
		wlcore_error("could not allocate memory for the firmware");
		ret = -ENOMEM;
		goto out_release;
	}

	memcpy(wl->fw, fw->data, fw->size);
	wl->fw_len = fw->size;

out_release:
	release_firmware(fw);
out:
	return ret;
}

static void wlcore_release_fw(struct wlcore *wl)
{
	vfree(wl->fw);
	wl->fw = NULL;
}

/* TODO: should this be combined with wlcore_request_firmware()? */
static int wlcore_request_nvs(struct wlcore *wl)
{
	const struct firmware *nvs;
	int ret = 0;

	if (wl->nvs)
		goto out;

	wlcore_debug(DEBUG_BOOT, "requesting NVS %s", wl->nvs_name);

	ret = request_firmware(&nvs, wl->nvs_name, wl->dev);
	if (ret < 0) {
		wlcore_error("could not get nvs file: %d", ret);
		goto out;
	}

	/* TODO: need to check NVS size alignment as for the FW? */

	wl->nvs = kmemdup(nvs->data, nvs->size, GFP_KERNEL);
	if (!wl->nvs) {
		wlcore_error("could not allocate memory for the nvs file");
		ret = -ENOMEM;
		goto out_release;
	}

	wl->nvs_len = nvs->size;

out_release:
	release_firmware(nvs);
out:
	return ret;
}

static void wlcore_release_nvs(struct wlcore *wl)
{
	kfree(wl->nvs);
	wl->nvs = NULL;
}

/* TODO: refactor this whole thing, we probably don't need 512-byte subchunks */
#define CHUNK_SIZE 512
static int wlcore_boot_upload_firmware_chunk(struct wlcore *wl, void *buf,
					     size_t fw_data_len, u32 dest)
{
	struct wlcore_partition_set partition;
	int addr, chunk_num, partition_limit;
	u8 *p, *chunk;

	memcpy(&partition, &wl->ptable[PART_DOWN], sizeof(partition));

	wlcore_debug(DEBUG_BOOT, "starting firmware upload");

	wlcore_debug(DEBUG_BOOT, "fw_data_len %zd chunk_size %d",
		     fw_data_len, CHUNK_SIZE);

	if ((fw_data_len % 4) != 0) {
		wlcore_error("firmware length not multiple of four");
		return -EIO;
	}

	chunk = kmalloc(CHUNK_SIZE, GFP_KERNEL);
	if (!chunk) {
		wlcore_error("allocation for firmware upload chunk failed");
		return -ENOMEM;
	}

	partition.mem.start = dest;
	wlcore_set_partition(wl, &partition);

	chunk_num = 0;
	partition_limit = partition.mem.size;

	while (chunk_num < fw_data_len / CHUNK_SIZE) {
		addr = dest + (chunk_num + 2) * CHUNK_SIZE;
		if (addr > partition_limit) {
			addr = dest + chunk_num * CHUNK_SIZE;
			partition_limit = chunk_num * CHUNK_SIZE +
				partition.mem.size;
			partition.mem.start = addr;
			wlcore_set_partition(wl, &partition);
		}

		addr = dest + chunk_num * CHUNK_SIZE;
		p = buf + chunk_num * CHUNK_SIZE;
		memcpy(chunk, p, CHUNK_SIZE);
		wlcore_debug(DEBUG_BOOT, "uploading fw chunk 0x%p to 0x%x",
			     p, addr);
		wlcore_write(wl, addr, chunk, CHUNK_SIZE, false);

		chunk_num++;
	}

	addr = dest + chunk_num * CHUNK_SIZE;
	p = buf + chunk_num * CHUNK_SIZE;
	memcpy(chunk, p, fw_data_len % CHUNK_SIZE);
	wlcore_debug(DEBUG_BOOT, "uploading fw last chunk (%zd B) 0x%p to 0x%x",
		     fw_data_len % CHUNK_SIZE, p, addr);
	wlcore_write(wl, addr, chunk, fw_data_len % CHUNK_SIZE, false);

	kfree(chunk);
	return 0;
}

static int wlcore_boot_upload_firmware(struct wlcore *wl)
{
	u32 chunks, addr, len;
	int ret = 0;
	u8 *fw;

	fw = wl->fw;
	chunks = be32_to_cpup((__be32 *) fw);
	fw += sizeof(u32);

	wlcore_debug(DEBUG_BOOT, "firmware chunks to be uploaded: %u", chunks);

	while (chunks--) {
		addr = be32_to_cpup((__be32 *) fw);
		fw += sizeof(u32);
		len = be32_to_cpup((__be32 *) fw);
		fw += sizeof(u32);

		if (len > 300000) {
			wlcore_info("firmware chunk too long: %u", len);
			return -EINVAL;
		}
		wlcore_debug(DEBUG_BOOT, "chunk %d addr 0x%x len %u",
			     chunks, addr, len);
		ret = wlcore_boot_upload_firmware_chunk(wl, fw, len, addr);
		if (ret != 0)
			break;
		fw += len;
	}

	return ret;
}

int wlcore_boot(struct wlcore *wl)
{
	int ret;
	wlcore_debug(DEBUG_BOOT, "wlcore_boot");

	msleep(20); /* WL1271_PRE_POWER_ON_SLEEP */
	ret = wlcore_io_power_on(wl);
	if (ret < 0)
		goto out;
	msleep(200); /* WL1271_POWER_ON_SLEEP */
	wlcore_io_reset(wl);
	wlcore_io_init(wl);

	/*
	 * TODO: check if this works with wl12xx too and that it is
	 * okay to do it before set_partition
	 */
	wlcore_raw_write32(wl, HW_ACCESS_ELP_CTRL_REG_ADDR, ELPCTRL_WAKE_UP);

	/* TODO: we don't need this here, we only need to set the partition */
	ret = wl->ops->get_chip_id(wl);
	if (ret < 0)
		goto out_power;
	wl->chip_id = ret;

	ret = wl->ops->config_pll(wl);
	if (ret < 0)
		goto out_power;

	/*
	 * TODO: we probably want to check if the firmware is not
	 * allocated yet or whether the name has changed
	 */
	ret = wlcore_request_firmware(wl);
	if (ret < 0)
		goto out_power;

	ret = wlcore_request_nvs(wl);
	if (ret < 0)
		goto out_fw;

	ret = wlcore_boot_upload_firmware(wl);
	if (ret < 0)
		goto out_nvs;

	goto out;

out_nvs:
	wlcore_release_nvs(wl);
out_fw:
	wlcore_release_fw(wl);
out_power:
	wlcore_io_power_off(wl);
out:
	return ret;
}

void wlcore_shutdown(struct wlcore *wl)
{
	wlcore_debug(DEBUG_BOOT, "wlcore_shutdown");

	wlcore_io_power_off(wl);

	/* TODO: we probably don't want to release the firmware here */
	wlcore_release_fw(wl);
	wlcore_release_nvs(wl);
}
