// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2018, Linaro Limited
 */

#include <avb_verify.h>
#include <blk.h>
#include <cpu_func.h>
#include <image.h>
#include <malloc.h>
#include <part.h>
#include <tee.h>
#include <tee/optee_ta_avb.h>

/** Location of the root public key. These symbols are declared at the start
 * and end of region where the content of the file CONFIG_AVB_PUBKEY_FILE is
 * imported into. */
extern const unsigned char _binary_common_avb_pubkey_start;
extern const unsigned char _binary_common_avb_pubkey_end;

const char *str_avb_io_error(AvbIOResult res)
{
	switch (res) {
	case AVB_IO_RESULT_OK:
		return "Requested operation was successful";
	case AVB_IO_RESULT_ERROR_IO:
		return "Underlying hardware encountered an I/O error";
	case AVB_IO_RESULT_ERROR_OOM:
		return "Unable to allocate memory";
	case AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION:
		return "Requested partition does not exist";
	case AVB_IO_RESULT_ERROR_RANGE_OUTSIDE_PARTITION:
		return "Bytes requested is outside the range of partition";
	case AVB_IO_RESULT_ERROR_NO_SUCH_VALUE:
		return "Named persistent value does not exist";
	case AVB_IO_RESULT_ERROR_INVALID_VALUE_SIZE:
		return "Named persistent value size is not supported";
	case AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE:
		return "Buffer is too small for the requested operation";
	default:
		return "Unknown AVB error";
	}
}

const char *str_avb_slot_error(AvbSlotVerifyResult res)
{
	switch (res) {
	case AVB_SLOT_VERIFY_RESULT_OK:
		return "Verification passed successfully";
	case AVB_SLOT_VERIFY_RESULT_ERROR_OOM:
		return "Allocation of memory failed";
	case AVB_SLOT_VERIFY_RESULT_ERROR_IO:
		return "I/O error occurred while trying to load data";
	case AVB_SLOT_VERIFY_RESULT_ERROR_VERIFICATION:
		return "Digest didn't match or signature checks failed";
	case AVB_SLOT_VERIFY_RESULT_ERROR_ROLLBACK_INDEX:
		return "Rollback index is less than its stored value";
	case AVB_SLOT_VERIFY_RESULT_ERROR_PUBLIC_KEY_REJECTED:
		return "Public keys are not accepted";
	case AVB_SLOT_VERIFY_RESULT_ERROR_INVALID_METADATA:
		return "Metadata is invalid or inconsistent";
	case AVB_SLOT_VERIFY_RESULT_ERROR_UNSUPPORTED_VERSION:
		return "Metadata requires a newer version of libavb";
	case AVB_SLOT_VERIFY_RESULT_ERROR_INVALID_ARGUMENT:
		return "Invalid arguments are used";
	default:
		return "Unknown AVB slot verification error";
	}
}
/**
 * ============================================================================
 * Boot states support (GREEN, YELLOW, ORANGE, RED) and dm_verity
 * ============================================================================
 */
char *avb_set_state(AvbOps *ops, enum avb_boot_state boot_state)
{
	struct AvbOpsData *data;
	char *cmdline = NULL;

	if (!ops)
		return NULL;

	data = (struct AvbOpsData *)ops->user_data;
	if (!data)
		return NULL;

	data->boot_state = boot_state;
	switch (boot_state) {
	case AVB_GREEN:
		cmdline = "androidboot.verifiedbootstate=green";
		break;
	case AVB_YELLOW:
		cmdline = "androidboot.verifiedbootstate=yellow";
		break;
	case AVB_ORANGE:
		cmdline = "androidboot.verifiedbootstate=orange";
	case AVB_RED:
		break;
	}

	return cmdline;
}

char *append_cmd_line(char *cmdline_orig, char *cmdline_new)
{
	char *cmd_line;

	if (!cmdline_new)
		return cmdline_orig;

	if (cmdline_orig)
		cmd_line = cmdline_orig;
	else
		cmd_line = " ";

	cmd_line = avb_strdupv(cmd_line, " ", cmdline_new, NULL);

	return cmd_line;
}

static int avb_find_dm_args(char **args, char *str)
{
	int i;

	if (!str)
		return -1;

	for (i = 0; i < AVB_MAX_ARGS && args[i]; ++i) {
		if (strstr(args[i], str))
			return i;
	}

	return -1;
}

static char *avb_set_enforce_option(const char *cmdline, const char *option)
{
	char *cmdarg[AVB_MAX_ARGS];
	char *newargs = NULL;
	int i = 0;
	int total_args;

	memset(cmdarg, 0, sizeof(cmdarg));
	cmdarg[i++] = strtok((char *)cmdline, " ");

	do {
		cmdarg[i] = strtok(NULL, " ");
		if (!cmdarg[i])
			break;

		if (++i >= AVB_MAX_ARGS) {
			printf("%s: Can't handle more then %d args\n",
			       __func__, i);
			return NULL;
		}
	} while (true);

	total_args = i;
	i = avb_find_dm_args(&cmdarg[0], VERITY_TABLE_OPT_LOGGING);
	if (i >= 0) {
		cmdarg[i] = (char *)option;
	} else {
		i = avb_find_dm_args(&cmdarg[0], VERITY_TABLE_OPT_RESTART);
		if (i < 0) {
			printf("%s: No verity options found\n", __func__);
			return NULL;
		}

		cmdarg[i] = (char *)option;
	}

	for (i = 0; i <= total_args; i++)
		newargs = append_cmd_line(newargs, cmdarg[i]);

	return newargs;
}

char *avb_set_ignore_corruption(const char *cmdline)
{
	char *newargs = NULL;

	newargs = avb_set_enforce_option(cmdline, VERITY_TABLE_OPT_LOGGING);
	if (newargs)
		newargs = append_cmd_line(newargs,
					  "androidboot.veritymode=eio");

	return newargs;
}

char *avb_set_enforce_verity(const char *cmdline)
{
	char *newargs;

	newargs = avb_set_enforce_option(cmdline, VERITY_TABLE_OPT_RESTART);
	if (newargs)
		newargs = append_cmd_line(newargs,
					  "androidboot.veritymode=enforcing");
	return newargs;
}

/**
 * ============================================================================
 * IO(mmc) auxiliary functions
 * ============================================================================
 */
static unsigned long mmc_read_and_flush(struct mmc_part *part,
					lbaint_t start,
					lbaint_t sectors,
					void *buffer)
{
	unsigned long blks;
	void *tmp_buf;
	size_t buf_size;
	bool unaligned = is_buf_unaligned(buffer);

	if (start < part->info.start) {
		printf("%s: partition start out of bounds\n", __func__);
		return 0;
	}
	if ((start + sectors) > (part->info.start + part->info.size)) {
		sectors = part->info.start + part->info.size - start;
		printf("%s: read sector aligned to partition bounds (%ld)\n",
		       __func__, sectors);
	}

	/*
	 * Reading fails on unaligned buffers, so we have to
	 * use aligned temporary buffer and then copy to destination
	 */
	if (unaligned) {
		debug("%s: handling unaligned read buffer, addr = 0x%p\n",
		      __func__, buffer);
		tmp_buf = get_sector_buf();
		buf_size = get_sector_buf_size();
		if (sectors > buf_size / part->info.blksz)
			sectors = buf_size / part->info.blksz;
	} else {
		tmp_buf = buffer;
	}

	blks = blk_dread(part->mmc_blk,
			 start, sectors, tmp_buf);
	/* flush cache after read */
	flush_cache((ulong)tmp_buf, sectors * part->info.blksz);

	if (unaligned)
		memcpy(buffer, tmp_buf, sectors * part->info.blksz);

	return blks;
}

static unsigned long mmc_write(struct mmc_part *part, lbaint_t start,
			       lbaint_t sectors, void *buffer)
{
	void *tmp_buf;
	size_t buf_size;
	bool unaligned = is_buf_unaligned(buffer);

	if (start < part->info.start) {
		printf("%s: partition start out of bounds\n", __func__);
		return 0;
	}
	if ((start + sectors) > (part->info.start + part->info.size)) {
		sectors = part->info.start + part->info.size - start;
		printf("%s: sector aligned to partition bounds (%ld)\n",
		       __func__, sectors);
	}
	if (unaligned) {
		tmp_buf = get_sector_buf();
		buf_size = get_sector_buf_size();
		debug("%s: handling unaligned read buffer, addr = 0x%p\n",
		      __func__, buffer);
		if (sectors > buf_size / part->info.blksz)
			sectors = buf_size / part->info.blksz;

		memcpy(tmp_buf, buffer, sectors * part->info.blksz);
	} else {
		tmp_buf = buffer;
	}

	return blk_dwrite(part->mmc_blk,
			  start, sectors, tmp_buf);
}

static struct mmc_part *get_partition(AvbOps *ops, const char *partition)
{
	int ret;
	u8 dev_num;
	int part_num = 0;
	struct mmc_part *part;
	struct blk_desc *mmc_blk;

	part = malloc(sizeof(struct mmc_part));
	if (!part)
		return NULL;

	dev_num = get_boot_device(ops);
	part->mmc = find_mmc_device(dev_num);
	if (!part->mmc) {
		printf("%s: no MMC device at slot %x\n", __func__, dev_num);
		goto err;
	}

	ret = mmc_init(part->mmc);
	if (ret) {
		printf("%s: MMC initialization failed, err = %d\n",
		       __func__, ret);
		goto err;
	}

	if (IS_MMC(part->mmc)) {
		ret = mmc_switch_part(part->mmc, part_num);
		if (ret) {
			printf("%s: MMC part switch failed, err = %d\n",
			       __func__, ret);
			goto err;
		}
	}

	mmc_blk = mmc_get_blk_desc(part->mmc);
	if (!mmc_blk) {
		printf("%s: failed to obtain block descriptor\n", __func__);
		goto err;
	}

	ret = part_get_info_by_name(mmc_blk, partition, &part->info);
	if (ret < 0) {
		printf("%s: can't find partition '%s'\n", __func__, partition);
		goto err;
	}

	part->dev_num = dev_num;
	part->mmc_blk = mmc_blk;

	return part;
err:
	free(part);
	return NULL;
}

static AvbIOResult mmc_byte_io(AvbOps *ops,
			       const char *partition,
			       s64 offset,
			       size_t num_bytes,
			       void *buffer,
			       size_t *out_num_read,
			       enum mmc_io_type io_type)
{
	ulong ret;
	struct mmc_part *part;
	u64 start_offset, start_sector, sectors, residue;
	u8 *tmp_buf;
	size_t io_cnt = 0;

	if (!partition || !buffer || io_type > IO_WRITE)
		return AVB_IO_RESULT_ERROR_IO;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	if (!part->info.blksz)
		return AVB_IO_RESULT_ERROR_IO;

	start_offset = calc_offset(part, offset);
	while (num_bytes) {
		start_sector = start_offset / part->info.blksz;
		sectors = num_bytes / part->info.blksz;
		/* handle non block-aligned reads */
		if (start_offset % part->info.blksz ||
		    num_bytes < part->info.blksz) {
			tmp_buf = get_sector_buf();
			if (start_offset % part->info.blksz) {
				residue = part->info.blksz -
					(start_offset % part->info.blksz);
				if (residue > num_bytes)
					residue = num_bytes;
			} else {
				residue = num_bytes;
			}

			if (io_type == IO_READ) {
				ret = mmc_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 1, tmp_buf);

				if (ret != 1) {
					printf("%s: read error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
				/*
				 * if this is not aligned at sector start,
				 * we have to adjust the tmp buffer
				 */
				tmp_buf += (start_offset % part->info.blksz);
				memcpy(buffer, (void *)tmp_buf, residue);
			} else {
				ret = mmc_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 1, tmp_buf);

				if (ret != 1) {
					printf("%s: read error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
				memcpy((void *)tmp_buf +
					start_offset % part->info.blksz,
					buffer, residue);

				ret = mmc_write(part, part->info.start +
						start_sector, 1, tmp_buf);
				if (ret != 1) {
					printf("%s: write error (%ld, %lld)\n",
					       __func__, ret, start_sector);
					return AVB_IO_RESULT_ERROR_IO;
				}
			}

			io_cnt += residue;
			buffer += residue;
			start_offset += residue;
			num_bytes -= residue;
			continue;
		}

		if (sectors) {
			if (io_type == IO_READ) {
				ret = mmc_read_and_flush(part,
							 part->info.start +
							 start_sector,
							 sectors, buffer);
			} else {
				ret = mmc_write(part,
						part->info.start +
						start_sector,
						sectors, buffer);
			}

			if (!ret) {
				printf("%s: sector read error\n", __func__);
				return AVB_IO_RESULT_ERROR_IO;
			}

			io_cnt += ret * part->info.blksz;
			buffer += ret * part->info.blksz;
			start_offset += ret * part->info.blksz;
			num_bytes -= ret * part->info.blksz;
		}
	}

	/* Set counter for read operation */
	if (io_type == IO_READ && out_num_read)
		*out_num_read = io_cnt;

	return AVB_IO_RESULT_OK;
}

/**
 * ============================================================================
 * AVB 2.0 operations
 * ============================================================================
 */

/**
 * read_from_partition() - reads @num_bytes from  @offset from partition
 * identified by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition_name: partition name, NUL-terminated UTF-8 string
 * @offset: offset from the beginning of partition
 * @num_bytes: amount of bytes to read
 * @buffer: destination buffer to store data
 * @out_num_read:
 *
 * @return:
 *      AVB_IO_RESULT_OK, if partition was found and read operation succeed
 *      AVB_IO_RESULT_ERROR_IO, if i/o error occurred from the underlying i/o
 *            subsystem
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if there is no partition with
 *      the given name
 */
static AvbIOResult read_from_partition(AvbOps *ops,
				       const char *partition_name,
				       s64 offset_from_partition,
				       size_t num_bytes,
				       void *buffer,
				       size_t *out_num_read)
{
	return mmc_byte_io(ops, partition_name, offset_from_partition,
			   num_bytes, buffer, out_num_read, IO_READ);
}

/**
 * write_to_partition() - writes N bytes to a partition identified by a string
 * name
 *
 * @ops: AvbOps, contains AVB ops handlers
 * @partition_name: partition name
 * @offset_from_partition: offset from the beginning of partition
 * @num_bytes: amount of bytes to write
 * @buf: data to write
 * @out_num_read:
 *
 * @return:
 *      AVB_IO_RESULT_OK, if partition was found and read operation succeed
 *      AVB_IO_RESULT_ERROR_IO, if input/output error occurred
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition, specified in
 *            @partition_name was not found
 */
static AvbIOResult write_to_partition(AvbOps *ops,
				      const char *partition_name,
				      s64 offset_from_partition,
				      size_t num_bytes,
				      const void *buffer)
{
	return mmc_byte_io(ops, partition_name, offset_from_partition,
			   num_bytes, (void *)buffer, NULL, IO_WRITE);
}

/**
 * validate_vmbeta_public_key() - checks if the given public key used to sign
 * the vbmeta partition is trusted
 *
 * @ops: AvbOps, contains AVB ops handlers
 * @public_key_data: public key for verifying vbmeta partition signature
 * @public_key_length: length of public key
 * @public_key_metadata:
 * @public_key_metadata_length:
 * @out_key_is_trusted:
 *
 * @return:
 *      AVB_IO_RESULT_OK, if partition was found and read operation succeed
 */
static AvbIOResult validate_vbmeta_public_key(AvbOps *ops,
					      const u8 *public_key_data,
					      size_t public_key_length,
					      const u8
					      *public_key_metadata,
					      size_t
					      public_key_metadata_length,
					      bool *out_key_is_trusted)
{
	size_t root_pubkey_size = (size_t)&_binary_common_avb_pubkey_end -
			(size_t)&_binary_common_avb_pubkey_start;
	if (!public_key_length || !public_key_data || !out_key_is_trusted)
		return AVB_IO_RESULT_ERROR_IO;

	*out_key_is_trusted = false;
	if (public_key_length != root_pubkey_size)
		return AVB_IO_RESULT_ERROR_IO;

	if (memcmp(&_binary_common_avb_pubkey_start, public_key_data, public_key_length) == 0)
		*out_key_is_trusted = true;

	return AVB_IO_RESULT_OK;
}

#ifdef CONFIG_OPTEE_TA_AVB
static int get_open_session(struct AvbOpsData *ops_data)
{
	struct udevice *tee = NULL;

	while (!ops_data->tee) {
		const struct tee_optee_ta_uuid uuid = TA_AVB_UUID;
		struct tee_open_session_arg arg;
		int rc;

		tee = tee_find_device(tee, NULL, NULL, NULL);
		if (!tee)
			return -ENODEV;

		memset(&arg, 0, sizeof(arg));
		tee_optee_ta_uuid_to_octets(arg.uuid, &uuid);
		rc = tee_open_session(tee, &arg, 0, NULL);
		if (rc || arg.ret)
			continue;

		ops_data->tee = tee;
		ops_data->session = arg.session;
	}

	return 0;
}

static AvbIOResult invoke_func(struct AvbOpsData *ops_data, u32 func,
			       ulong num_param, struct tee_param *param)
{
	struct tee_invoke_arg arg;

	if (get_open_session(ops_data))
		return AVB_IO_RESULT_ERROR_IO;

	memset(&arg, 0, sizeof(arg));
	arg.func = func;
	arg.session = ops_data->session;

	if (tee_invoke_func(ops_data->tee, &arg, num_param, param))
		return AVB_IO_RESULT_ERROR_IO;
	switch (arg.ret) {
	case TEE_SUCCESS:
		return AVB_IO_RESULT_OK;
	case TEE_ERROR_OUT_OF_MEMORY:
		return AVB_IO_RESULT_ERROR_OOM;
	case TEE_ERROR_STORAGE_NO_SPACE:
		return AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE;
	case TEE_ERROR_ITEM_NOT_FOUND:
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;
	case TEE_ERROR_TARGET_DEAD:
		/*
		 * The TA has paniced, close the session to reload the TA
		 * for the next request.
		 */
		tee_close_session(ops_data->tee, ops_data->session);
		ops_data->tee = NULL;
		return AVB_IO_RESULT_ERROR_IO;
	default:
		return AVB_IO_RESULT_ERROR_IO;
	}
}
#endif

/**
 * read_rollback_index() - gets the rollback index corresponding to the
 * location of given by @out_rollback_index.
 *
 * @ops: contains AvbOps handlers
 * @rollback_index_slot:
 * @out_rollback_index: used to write a retrieved rollback index.
 *
 * @return
 *       AVB_IO_RESULT_OK, if the roolback index was retrieved
 */
static AvbIOResult read_rollback_index(AvbOps *ops,
				       size_t rollback_index_slot,
				       u64 *out_rollback_index)
{
#ifndef CONFIG_OPTEE_TA_AVB
	/* For now we always return 0 as the stored rollback index. */
	debug("%s: rollback protection is not implemented\n", __func__);

	if (out_rollback_index)
		*out_rollback_index = 0;

	return AVB_IO_RESULT_OK;
#else
	AvbIOResult rc;
	struct tee_param param[2];

	if (rollback_index_slot >= TA_AVB_MAX_ROLLBACK_LOCATIONS)
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;

	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = rollback_index_slot;
	param[1].attr = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT;

	rc = invoke_func(ops->user_data, TA_AVB_CMD_READ_ROLLBACK_INDEX,
			 ARRAY_SIZE(param), param);
	if (rc)
		return rc;

	*out_rollback_index = (u64)param[1].u.value.a << 32 |
			      (u32)param[1].u.value.b;
	return AVB_IO_RESULT_OK;
#endif
}

/**
 * write_rollback_index() - sets the rollback index corresponding to the
 * location of given by @out_rollback_index.
 *
 * @ops: contains AvbOps handlers
 * @rollback_index_slot:
 * @rollback_index: rollback index to write.
 *
 * @return
 *       AVB_IO_RESULT_OK, if the roolback index was retrieved
 */
static AvbIOResult write_rollback_index(AvbOps *ops,
					size_t rollback_index_slot,
					u64 rollback_index)
{
#ifndef CONFIG_OPTEE_TA_AVB
	/* For now this is a no-op. */
	debug("%s: rollback protection is not implemented\n", __func__);

	return AVB_IO_RESULT_OK;
#else
	struct tee_param param[2];

	if (rollback_index_slot >= TA_AVB_MAX_ROLLBACK_LOCATIONS)
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;

	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[0].u.value.a = rollback_index_slot;
	param[1].attr = TEE_PARAM_ATTR_TYPE_VALUE_INPUT;
	param[1].u.value.a = (u32)(rollback_index >> 32);
	param[1].u.value.b = (u32)rollback_index;

	return invoke_func(ops->user_data, TA_AVB_CMD_WRITE_ROLLBACK_INDEX,
			   ARRAY_SIZE(param), param);
#endif
}

/**
 * read_is_device_unlocked() - gets whether the device is unlocked
 *
 * @ops: contains AVB ops handlers
 * @out_is_unlocked: device unlock state is stored here, true if unlocked,
 *       false otherwise
 *
 * @return:
 *       AVB_IO_RESULT_OK: state is retrieved successfully
 *       AVB_IO_RESULT_ERROR_IO: an error occurred
 */
static AvbIOResult read_is_device_unlocked(AvbOps *ops, bool *out_is_unlocked)
{
#ifndef CONFIG_OPTEE_TA_AVB
	/* For now we always return that the device is unlocked. */
	debug("%s: device locking is not implemented\n", __func__);

	*out_is_unlocked = true;

	return AVB_IO_RESULT_OK;
#else
	AvbIOResult rc;
	struct tee_param param = { .attr = TEE_PARAM_ATTR_TYPE_VALUE_OUTPUT };

	rc = invoke_func(ops->user_data, TA_AVB_CMD_READ_LOCK_STATE, 1, &param);
	if (rc)
		return rc;
	*out_is_unlocked = !param.u.value.a;
	return AVB_IO_RESULT_OK;
#endif
}

/**
 * get_unique_guid_for_partition() - gets the GUID for a partition identified
 * by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition: partition name (NUL-terminated UTF-8 string)
 * @guid_buf: buf, used to copy in GUID string. Example of value:
 *      527c1c6d-6361-4593-8842-3c78fcd39219
 * @guid_buf_size: @guid_buf buffer size
 *
 * @return:
 *      AVB_IO_RESULT_OK, on success (GUID found)
 *      AVB_IO_RESULT_ERROR_IO, if incorrect buffer size (@guid_buf_size) was
 *             provided
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition was not found
 */
static AvbIOResult get_unique_guid_for_partition(AvbOps *ops,
						 const char *partition,
						 char *guid_buf,
						 size_t guid_buf_size)
{
	struct mmc_part *part;
	size_t uuid_size;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	uuid_size = sizeof(part->info.uuid);
	if (uuid_size > guid_buf_size)
		return AVB_IO_RESULT_ERROR_IO;

	memcpy(guid_buf, part->info.uuid, uuid_size);
	guid_buf[uuid_size - 1] = 0;

	return AVB_IO_RESULT_OK;
}

/**
 * get_size_of_partition() - gets the size of a partition identified
 * by a string name
 *
 * @ops: contains AVB ops handlers
 * @partition: partition name (NUL-terminated UTF-8 string)
 * @out_size_num_bytes: returns the value of a partition size
 *
 * @return:
 *      AVB_IO_RESULT_OK, on success (GUID found)
 *      AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE, out_size_num_bytes is NULL
 *      AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION, if partition was not found
 */
static AvbIOResult get_size_of_partition(AvbOps *ops,
					 const char *partition,
					 u64 *out_size_num_bytes)
{
	struct mmc_part *part;

	if (!out_size_num_bytes)
		return AVB_IO_RESULT_ERROR_INSUFFICIENT_SPACE;

	part = get_partition(ops, partition);
	if (!part)
		return AVB_IO_RESULT_ERROR_NO_SUCH_PARTITION;

	*out_size_num_bytes = part->info.blksz * part->info.size;

	return AVB_IO_RESULT_OK;
}

#ifdef CONFIG_OPTEE_TA_AVB
static AvbIOResult read_persistent_value(AvbOps *ops,
					 const char *name,
					 size_t buffer_size,
					 u8 *out_buffer,
					 size_t *out_num_bytes_read)
{
	AvbIOResult rc;
	struct tee_shm *shm_name;
	struct tee_shm *shm_buf;
	struct tee_param param[2];
	struct udevice *tee;
	size_t name_size = strlen(name) + 1;

	if (get_open_session(ops->user_data))
		return AVB_IO_RESULT_ERROR_IO;

	tee = ((struct AvbOpsData *)ops->user_data)->tee;

	rc = tee_shm_alloc(tee, name_size,
			   TEE_SHM_ALLOC, &shm_name);
	if (rc)
		return AVB_IO_RESULT_ERROR_OOM;

	rc = tee_shm_alloc(tee, buffer_size,
			   TEE_SHM_ALLOC, &shm_buf);
	if (rc) {
		rc = AVB_IO_RESULT_ERROR_OOM;
		goto free_name;
	}

	memcpy(shm_name->addr, name, name_size);

	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = shm_name;
	param[0].u.memref.size = name_size;
	param[1].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INOUT;
	param[1].u.memref.shm = shm_buf;
	param[1].u.memref.size = buffer_size;

	rc = invoke_func(ops->user_data, TA_AVB_CMD_READ_PERSIST_VALUE,
			 2, param);
	if (rc)
		goto out;

	if (param[1].u.memref.size > buffer_size) {
		rc = AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;
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

static AvbIOResult write_persistent_value(AvbOps *ops,
					  const char *name,
					  size_t value_size,
					  const u8 *value)
{
	AvbIOResult rc;
	struct tee_shm *shm_name;
	struct tee_shm *shm_buf;
	struct tee_param param[2];
	struct udevice *tee;
	size_t name_size = strlen(name) + 1;

	if (get_open_session(ops->user_data))
		return AVB_IO_RESULT_ERROR_IO;

	tee = ((struct AvbOpsData *)ops->user_data)->tee;

	if (!value_size)
		return AVB_IO_RESULT_ERROR_NO_SUCH_VALUE;

	rc = tee_shm_alloc(tee, name_size,
			   TEE_SHM_ALLOC, &shm_name);
	if (rc)
		return AVB_IO_RESULT_ERROR_OOM;

	rc = tee_shm_alloc(tee, value_size,
			   TEE_SHM_ALLOC, &shm_buf);
	if (rc) {
		rc = AVB_IO_RESULT_ERROR_OOM;
		goto free_name;
	}

	memcpy(shm_name->addr, name, name_size);
	memcpy(shm_buf->addr, value, value_size);

	memset(param, 0, sizeof(param));
	param[0].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[0].u.memref.shm = shm_name;
	param[0].u.memref.size = name_size;
	param[1].attr = TEE_PARAM_ATTR_TYPE_MEMREF_INPUT;
	param[1].u.memref.shm = shm_buf;
	param[1].u.memref.size = value_size;

	rc = invoke_func(ops->user_data, TA_AVB_CMD_WRITE_PERSIST_VALUE,
			 2, param);
	if (rc)
		goto out;

out:
	tee_shm_free(shm_buf);
free_name:
	tee_shm_free(shm_name);

	return rc;
}
#endif

/**
 * ============================================================================
 * AVB2.0 AvbOps alloc/initialisation/free
 * ============================================================================
 */
AvbOps *avb_ops_alloc(int boot_device)
{
	struct AvbOpsData *ops_data;

	ops_data = avb_calloc(sizeof(struct AvbOpsData));
	if (!ops_data)
		return NULL;

	ops_data->ops.user_data = ops_data;

	ops_data->ops.read_from_partition = read_from_partition;
	ops_data->ops.write_to_partition = write_to_partition;
	ops_data->ops.validate_vbmeta_public_key = validate_vbmeta_public_key;
	ops_data->ops.read_rollback_index = read_rollback_index;
	ops_data->ops.write_rollback_index = write_rollback_index;
	ops_data->ops.read_is_device_unlocked = read_is_device_unlocked;
	ops_data->ops.get_unique_guid_for_partition =
		get_unique_guid_for_partition;
#ifdef CONFIG_OPTEE_TA_AVB
	ops_data->ops.write_persistent_value = write_persistent_value;
	ops_data->ops.read_persistent_value = read_persistent_value;
#endif
	ops_data->ops.get_size_of_partition = get_size_of_partition;
	ops_data->mmc_dev = boot_device;

	return &ops_data->ops;
}

void avb_ops_free(AvbOps *ops)
{
	struct AvbOpsData *ops_data;

	if (!ops)
		return;

	ops_data = ops->user_data;

	if (ops_data) {
#ifdef CONFIG_OPTEE_TA_AVB
		if (ops_data->tee)
			tee_close_session(ops_data->tee, ops_data->session);
#endif
		avb_free(ops_data);
	}
}
