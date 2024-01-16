// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Boundary Devices
 */
#include <env.h>
#include <errno.h>
#include <tee.h>
#include <asm/arch-mediatek/sys_proto.h>

#define EFUSE_TEST_UUID {0xa2567d51, 0x0144, 0x4543, \
	{ 0xb4, 0x0a, 0xca, 0xba,0x40, 0x27, 0x97, 0x03} }
#define EFUSE_REG_LEN		4
#define EFUSE_MAC_HIGH_IDX	243
#define EFUSE_MAC_LOW_IDX	244
#define TZCMD_MAX_SIZE		3
#define TZCMD_EFUSE_READ	0

static struct udevice *tee;
static u32 session;

static int optee_mtk_efuse_ta_open(void)
{
	const struct tee_optee_ta_uuid uuid = EFUSE_TEST_UUID;
	struct tee_open_session_arg arg = { 0 };
	int ret;

	tee = tee_find_device(tee, NULL, NULL, NULL);
	if (!tee)
		return -ENODEV;

	tee_optee_ta_uuid_to_octets(arg.uuid, &uuid);
	ret = tee_open_session(tee, &arg, 0, NULL);
	if (ret)
		return ret;
	session = arg.session;

	return 0;
}

static void optee_mtk_efuse_ta_close(void)
{
	if (tee)
		tee_close_session(tee, session);

	tee = NULL;
}

static int invoke_func(u32 func, ulong num, struct tee_param *param)
{
	struct tee_invoke_arg arg;

	if (!tee) {
		if (optee_mtk_efuse_ta_open())
			return -ENODEV;
	}

	memset(&arg, 0, sizeof(arg));
	arg.func = func;
	arg.session = session;

	if (tee_invoke_func(tee, &arg, num, param))
		return -EFAULT;
	switch (arg.ret) {
		case TEE_SUCCESS:
			return 0;
		case TEE_ERROR_OUT_OF_MEMORY:
		case TEE_ERROR_STORAGE_NO_SPACE:
			return -ENOSPC;
		case TEE_ERROR_ITEM_NOT_FOUND:
			return -EIO;
		case TEE_ERROR_TARGET_DEAD:
			optee_mtk_efuse_ta_close();
			return -EIO;
		default:
			return -EIO;
	}
}

static int optee_mtk_efuse_value(u8 idx, u8 *out, size_t size, size_t *num)
{
	struct tee_shm *buf;
	struct tee_param param[TZCMD_MAX_SIZE];
	int rc = 0;

	if (!tee) {
		if (optee_mtk_efuse_ta_open())
			return -ENODEV;
	}

	rc = tee_shm_alloc(tee, size, TEE_SHM_ALLOC, &buf);
	if (rc) {
		rc = -ENOMEM;
		goto out;
	}

	memset(param, 0, sizeof(param));
	param[0].attr = 1;
	param[0].u.value.a = idx;
	param[1].attr = 6;
	param[1].u.memref.shm = buf;
	param[1].u.memref.size = size;
	param[2].attr = 1;
	param[2].u.value.a = *num;

	rc = invoke_func(TZCMD_EFUSE_READ, TZCMD_MAX_SIZE, param);
	if (rc)
		goto out;

	if (param[1].u.memref.size > size) {
		rc = -EINVAL;
		goto out;
	}

	*num = param[1].u.memref.size;
	memcpy(out, buf->addr, *num);

	out:
	if (buf)
		tee_shm_free(buf);

	optee_mtk_efuse_ta_close();
	return rc;
}

int mtk_get_mac_from_fuse(unsigned char *mac)
{
	size_t len = sizeof(u32);
	u8 data[EFUSE_REG_LEN];
	int ret;

	ret = optee_mtk_efuse_value(EFUSE_MAC_HIGH_IDX, data,
				    EFUSE_REG_LEN, &len);
	if (ret) {
		pr_err("Failed to read fuse TA hi %d\n", ret);
		return ret;
	}
	mac[0] = data[2];
	mac[1] = data[3];

	ret = optee_mtk_efuse_value(EFUSE_MAC_LOW_IDX, data,
				    EFUSE_REG_LEN, &len);
	if (ret) {
		pr_err("Failed to read fuse TA lo %d\n", ret);
		return ret;
	}
	mac[2] = data[0];
	mac[3] = data[1];
	mac[4] = data[2];
	mac[5] = data[3];

	return 0;
}
