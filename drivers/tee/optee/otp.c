// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020 NXP
 * Copyright 2022 BayLibre
 *
 * Based on cmd/optee_rpmb.c
 */

#include <env.h>
#include <errno.h>
#include <mmc.h>
#include <net.h>
#include <tee.h>
#include <fdt_support.h>

#include <tee/optee_ta_otp.h>

#define MAC_ADDRESS_LEN 6
#define MAX_SN_LEN 64

static struct udevice *tee;
static u32 session;

static int optee_otp_ta_open_session(void)
{
	const struct tee_optee_ta_uuid uuid = TA_OTP_UUID;
	struct tee_open_session_arg arg;
	int rc;

	tee = tee_find_device(tee, NULL, NULL, NULL);
	if (!tee)
		return -ENODEV;

	memset(&arg, 0, sizeof(arg));
	tee_optee_ta_uuid_to_octets(arg.uuid, &uuid);
	rc = tee_open_session(tee, &arg, 0, NULL);
	if (!rc)
		session = arg.session;

	return 0;
}

void optee_otp_ta_close_session(void)
{
	if (tee)
		tee_close_session(tee, session);
	tee = NULL;
}

static int invoke_func(u32 func, ulong num_param, struct tee_param *param)
{
	struct tee_invoke_arg arg;

	if (!tee)
		if (optee_otp_ta_open_session())
			return -ENODEV;

	memset(&arg, 0, sizeof(arg));
	arg.func = func;
	arg.session = session;

	if (tee_invoke_func(tee, &arg, num_param, param))
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
		/*
		 * The TA has paniced, close the session to reload the TA
		 * for the next request.
		 */
		tee_close_session(tee, session);
		tee = NULL;
		return -EIO;
	default:
		return -EIO;
	}
}

int optee_otp_readp_value(const char *name,
			  size_t buffer_size,
			  u8 *out_buffer,
			  size_t *out_num_bytes_read)
{
	int rc = 0;
	struct tee_shm *shm_name;
	struct tee_shm *shm_buf;
	struct tee_param param[2];
	size_t name_size = strlen(name) + 1;

	if (!tee)
		if (optee_otp_ta_open_session())
			return -ENODEV;

	rc = tee_shm_alloc(tee, name_size,
			   TEE_SHM_ALLOC, &shm_name);
	if (rc)
		return -ENOMEM;

	rc = tee_shm_alloc(tee, buffer_size,
			   TEE_SHM_ALLOC, &shm_buf);
	if (rc) {
		rc = -ENOMEM;
		goto free_name;
	}

	memcpy(shm_name->addr, name, name_size);

	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = shm_name;
	param[0].u.memref.size = name_size;
	param[1].attr = TEE_PARAM_ATTR_TYPE_MEMREF_OUTPUT;
	param[1].u.memref.shm = shm_buf;
	param[1].u.memref.size = buffer_size;

	rc = invoke_func(TA_OTP_CMD_READ_RAW,
			 2, param);
	if (rc)
		goto out;

	if (param[1].u.memref.size > buffer_size) {
		rc = -EINVAL;
		goto out;
	}

	*out_num_bytes_read = param[1].u.memref.size;

	memcpy(out_buffer, shm_buf->addr, *out_num_bytes_read);

out:
	tee_shm_free(shm_buf);
free_name:
	tee_shm_free(shm_name);

	return rc;
}

int optee_otp_read_serial(void)
{
	u8 serial_data[MAX_SN_LEN];
	size_t len;
	int ret;

	if (env_get("serial#")) {
		printf("Using serial from u-boot env\n");
		return 0;
	}

	memset(serial_data, 0, MAX_SN_LEN);
	ret = optee_otp_readp_value("serial", MAX_SN_LEN, serial_data, &len);
	if (!ret)
		env_set("serial#", serial_data);

	return ret;
}

int optee_otp_read_mac(const char *name)
{
	u8 ethaddr[MAC_ADDRESS_LEN];
	size_t len;
	int ret;

	if (env_get(name)) {
		printf("%s: Using mac address from u-boot env\n", name);
		return 0;
	}

	ret = optee_otp_readp_value("mac", MAC_ADDRESS_LEN, ethaddr, &len);
	if (!ret && is_valid_ethaddr(ethaddr))
		eth_env_set_enetaddr(name, ethaddr);

	return ret;
}

int optee_otp_read_mac_fdt(void *blob, const char *node,
			   const char *attribute,
			   const char *otp_name)
{
	u8 ethaddr[MAC_ADDRESS_LEN];
	size_t len;
	int ret;
	int nodeoffset;

	ret = optee_otp_readp_value(otp_name, MAC_ADDRESS_LEN, ethaddr, &len);
	if (!ret && is_valid_ethaddr(ethaddr)) {
		nodeoffset = fdt_path_offset(blob, node);
		if (nodeoffset < 0)
			return nodeoffset;

		ret = fdt_setprop(blob, nodeoffset, attribute, ethaddr,
				  sizeof(ethaddr));
		if (ret < 0) {
			printf("WARNING: could not set %s %s.\n", attribute,
					fdt_strerror(ret));
			return ret;
		}
	} else {
		printf("Failed to read mac address\n");
		ret = 0;
	}

	optee_otp_ta_close_session();

	return ret;
}
