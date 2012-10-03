/*
 * (C) Copyright 2009
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * (C) Copyright 2008
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "mkimage.h"
#include <image.h>
#include "imximage.h"

/*
 * Supported commands for configuration file
 */
static table_entry_t imximage_cmds[] = {
	{CMD_BOOT_FROM,         "BOOT_FROM",            "boot command",	  },
	{CMD_DATA,              "DATA",                 "Reg Write Data", },
	{CMD_IMAGE_VERSION,     "IMAGE_VERSION",        "image version",  },
	{-1,                    "",                     "",	          },
};

/*
 * Supported Boot options for configuration file
 * this is needed to set the correct flash offset
 */
static table_entry_t imximage_bootops[] = {
	{FLASH_OFFSET_ONENAND,	"onenand",	"OneNAND Flash",},
	{FLASH_OFFSET_NAND,	"nand",		"NAND Flash",	},
	{FLASH_OFFSET_NOR,	"nor",		"NOR Flash",	},
	{FLASH_OFFSET_SATA,	"sata",		"SATA Disk",	},
	{FLASH_OFFSET_SD,	"sd",		"SD Card",	},
	{FLASH_OFFSET_SPI,	"spi",		"SPI Flash",	},
	{-1,			"",		"Invalid",	},
};

/*
 * IMXIMAGE version definition for i.MX chips
 */
static table_entry_t imximage_versions[] = {
	{IMXIMAGE_V1,	"",	" (i.MX25/35/51 compatible)", },
	{IMXIMAGE_V2,	"",	" (i.MX53/6 compatible)",     },
	{-1,            "",     " (Invalid)",                 },
};

static uint32_t *header_size_ptr;
static uint32_t g_flash_offset;

static struct image_type_params imximage_params;

static uint32_t detect_imximage_version(struct imx_header *imx_hdr)
{
	imx_header_v1_t *hdr_v1 = &imx_hdr->header.hdr_v1;
	imx_header_v2_t *hdr_v2 = &imx_hdr->header.hdr_v2;
	flash_header_v1_t *fhdr_v1 = &hdr_v1->fhdr;
	flash_header_v2_t *fhdr_v2 = &hdr_v2->fhdr;

	/* Try to detect V1 */
	if ((fhdr_v1->app_code_barker == APP_CODE_BARKER) &&
		(hdr_v1->dcd_table.preamble.barker == DCD_BARKER))
		return IMXIMAGE_V1;

	/* Try to detect V2 */
	if (fhdr_v2->header.tag == IVT_HEADER_TAG)
		return IMXIMAGE_V2;

	return IMXIMAGE_VER_INVALID;
}

static void err_imximage_version(int version)
{
	fprintf(stderr,
		"Error: Unsupported imximage version:%d\n", version);

	exit(EXIT_FAILURE);
}

static int set_dcd_val_v1(struct data_src *ds, uint32_t *data)
{
	dcd_v1_t *dcd_v1 = &ds->imxhdr->header.hdr_v1.dcd_table;
	uint32_t val = *data++;

	/* Byte, halfword, word */
	if ((val != 1) && (val != 2) && (val != 4)) {
		fprintf(stderr, "Error: Invalid register size (%d)\n", val);
		return -1;
	}
	*ds->p_entry++ = val;
	*ds->p_entry++ = *data++;
	*ds->p_entry++ = *data++;
	dcd_v1->preamble.length = (char *)ds->p_entry - (char *)&dcd_v1->
			addr_data[0].type;
	return 0;
}

static int set_dcd_val_v2(struct data_src *ds, uint32_t *data)
{
	uint32_t len;
	dcd_v2_t *dcd_v2 = &ds->imxhdr->header.hdr_v2.dcd_table;
	uint32_t val = *data++;

	/* Byte, halfword, word */
	if ((val != 1) && (val != 2) && (val != 4)) {
		fprintf(stderr, "Error: Invalid register size (%d)\n", val);
		return -1;
	}
	if (!(ds->p_dcd && (ds->p_dcd->param == val))) {
		if (!ds->p_dcd) {
			dcd_v2->header.tag = DCD_HEADER_TAG;
			dcd_v2->header.version = DCD_VERSION;
			ds->p_dcd = &dcd_v2->write_dcd_command;
		} else {
			ds->p_dcd = (write_dcd_command_t *)ds->p_entry;
		}
		ds->p_dcd->param = val;
		ds->p_dcd->tag = DCD_COMMAND_TAG;
		ds->p_entry = (uint32_t *)(ds->p_dcd + 1);
	}
	val = *data++;
	*ds->p_entry++ = cpu_to_be32(val);
	val = *data++;
	*ds->p_entry++ = cpu_to_be32(val);
	len = (char *)ds->p_entry - (char *)&dcd_v2->header;
	dcd_v2->header.length = cpu_to_be16(len);
	len = (char *)ds->p_entry - (char *)ds->p_dcd;
	ds->p_dcd->length = cpu_to_be16(len);
	return 0;
}

static int set_imx_hdr_v1(struct data_src *ds,
		uint32_t entry_point, uint32_t flash_offset)
{
	imx_header_v1_t *hdr_v1 = &ds->imxhdr->header.hdr_v1;
	flash_header_v1_t *fhdr_v1 = &hdr_v1->fhdr;
	uint32_t hdr_base;
	uint32_t header_length = ((char *)ds->p_entry) + 4
			- ((char *)ds->imxhdr);

	/* Set magic number */
	fhdr_v1->app_code_barker = APP_CODE_BARKER;

	hdr_base = entry_point - header_length;
	fhdr_v1->app_dest_ptr = hdr_base - flash_offset;
	fhdr_v1->app_code_jump_vector = entry_point;

	fhdr_v1->dcd_ptr_ptr = hdr_base + offsetof(flash_header_v1_t, dcd_ptr);
	fhdr_v1->dcd_ptr = hdr_base + offsetof(imx_header_v1_t, dcd_table);

	/* Security feature are not supported */
	fhdr_v1->app_code_csf = 0;
	fhdr_v1->super_root_key = 0;
	header_size_ptr = (uint32_t *)(((char *)ds->imxhdr) +
			header_length - 4);
	return header_length;
}

static int set_imx_hdr_v2(struct data_src *ds,
		uint32_t entry_point, uint32_t flash_offset)
{
	imx_header_v2_t *hdr_v2 = &ds->imxhdr->header.hdr_v2;
	flash_header_v2_t *fhdr_v2 = &hdr_v2->fhdr;
	uint32_t hdr_base;
	uint32_t header_length = ((char *)ds->p_entry) - ((char *)ds->imxhdr);

	/* Set magic number */
	fhdr_v2->header.tag = IVT_HEADER_TAG; /* 0xD1 */
	fhdr_v2->header.length = cpu_to_be16(sizeof(flash_header_v2_t));
	fhdr_v2->header.version = IVT_VERSION; /* 0x40 */

	fhdr_v2->entry = entry_point;
	fhdr_v2->reserved1 = fhdr_v2->reserved2 = 0;
	fhdr_v2->self = hdr_base = entry_point - header_length;

	fhdr_v2->dcd_ptr = (ds->p_dcd) ? hdr_base
			+ offsetof(imx_header_v2_t, dcd_table) : 0;
	fhdr_v2->boot_data_ptr = hdr_base
			+ offsetof(imx_header_v2_t, boot_data);
	hdr_v2->boot_data.start = hdr_base - flash_offset;

	/* Security feature are not supported */
	fhdr_v2->csf = 0;
	header_size_ptr = &hdr_v2->boot_data.size;
	return header_length;
}

static void set_hdr_func(struct data_src *ds, uint32_t imximage_version)
{
	switch (imximage_version) {
	case IMXIMAGE_V1:
		ds->set_dcd_val = set_dcd_val_v1;
		ds->set_imx_hdr = set_imx_hdr_v1;
		ds->p_entry = &ds->imxhdr->header.hdr_v1.dcd_table
				.addr_data[0].type;
		ds->p_max_dcd = &ds->imxhdr->header.hdr_v1.dcd_table
				.addr_data[MAX_HW_CFG_SIZE_V1].type;
		ds->imxhdr->header.hdr_v1.dcd_table.preamble.barker =
				DCD_BARKER;
		break;
	case IMXIMAGE_V2:
		ds->set_dcd_val = set_dcd_val_v2;
		ds->set_imx_hdr = set_imx_hdr_v2;
		ds->p_entry = (uint32_t *)&ds->imxhdr->header.hdr_v2.dcd_table;
		ds->p_max_dcd = (uint32_t *)
				((char *)ds->imxhdr + MAX_HEADER_SIZE);
		break;
	default:
		err_imximage_version(imximage_version);
		break;
	}
}

static void print_hdr_v1(struct imx_header *imx_hdr)
{
	imx_header_v1_t *hdr_v1 = &imx_hdr->header.hdr_v1;
	flash_header_v1_t *fhdr_v1 = &hdr_v1->fhdr;
	dcd_v1_t *dcd_v1 = &hdr_v1->dcd_table;
	uint32_t size, length, ver;

	size = dcd_v1->preamble.length;
	if (size > (MAX_HW_CFG_SIZE_V1 * sizeof(dcd_type_addr_data_t))) {
		fprintf(stderr,
			"Error: Image corrupt DCD size %d exceed maximum %d\n",
			(uint32_t)(size / sizeof(dcd_type_addr_data_t)),
			MAX_HW_CFG_SIZE_V1);
		exit(EXIT_FAILURE);
	}

	length = dcd_v1->preamble.length / sizeof(dcd_type_addr_data_t);
	ver = detect_imximage_version(imx_hdr);

	printf("Image Type:   Freescale IMX Boot Image\n");
	printf("Image Ver:    %x", ver);
	printf("%s\n", get_table_entry_name(imximage_versions, NULL, ver));
	printf("Data Size:    ");
	genimg_print_size(dcd_v1->addr_data[length].type);
	printf("Load Address: %08x\n", (uint32_t)fhdr_v1->app_dest_ptr);
	printf("Entry Point:  %08x\n", (uint32_t)fhdr_v1->app_code_jump_vector);
}

static void print_hdr_v2(struct imx_header *imx_hdr)
{
	imx_header_v2_t *hdr_v2 = &imx_hdr->header.hdr_v2;
	flash_header_v2_t *fhdr_v2 = &hdr_v2->fhdr;
	dcd_v2_t *dcd_v2 = &hdr_v2->dcd_table;
	uint32_t size, version;

	size = be16_to_cpu(dcd_v2->header.length) - 8;
	if (size > (MAX_HW_CFG_SIZE_V2 * sizeof(dcd_addr_data_t))) {
		fprintf(stderr,
			"Error: Image corrupt DCD size %d exceed maximum %d\n",
			(uint32_t)(size / sizeof(dcd_addr_data_t)),
			MAX_HW_CFG_SIZE_V2);
		exit(EXIT_FAILURE);
	}

	version = detect_imximage_version(imx_hdr);

	printf("Image Type:   Freescale IMX Boot Image\n");
	printf("Image Ver:    %x", version);
	printf("%s\n", get_table_entry_name(imximage_versions, NULL, version));
	printf("Data Size:    ");
	genimg_print_size(hdr_v2->boot_data.size);
	printf("Load Address: %08x\n", (uint32_t)fhdr_v2->boot_data_ptr);
	printf("Entry Point:  %08x\n", (uint32_t)fhdr_v2->entry);
}

static int parse_cmd_data(struct data_src *ds)
{
	uint32_t data[3];
	int ret = ph_get_array(&ds->ph, data, 3);

	if (ret < 0)
		return ret;
	ret = (*ds->set_dcd_val)(ds, data);
	if (ret)
		return ret;
	if (ds->p_entry > ds->p_max_dcd) {
		uint32_t size = (char *)ds->p_max_dcd - (char *)ds->imxhdr;
		fprintf(stderr, "Error: header exceeds maximum size(%d)\n",
				size);
		return -1;
	}
	return 0;
}

static int parse_image_version(struct data_src *ds)
{
	int ret;
	uint32_t imximage_version;

	ret = ph_get_value(&ds->ph, &imximage_version);
	if (ret)
		return ret;
	if (ds->cmd_cnt) {
		fprintf(stderr, "Error: IMAGE_VERSION command needs be "
				"before other valid commands in the file\n");
		return -1;
	}
	set_hdr_func(ds, imximage_version);
	return 0;
}

static int parse_boot_from(struct data_src *ds)
{
	g_flash_offset = ph_get_table_entry_id(&ds->ph, imximage_bootops,
			"imximage boot option");
	if (g_flash_offset == -1) {
		fprintf(stderr, "Error: Invalid boot device\n");
		return -1;
	}
	return 0;
}

static const parse_fld_t cmd_table[] = {
	parse_image_version, parse_boot_from, parse_cmd_data
};

static int parse_command(struct data_src *ds)
{
	int cmd = ph_get_table_entry_id(&ds->ph, imximage_cmds,
			"imximage commands");
	if (cmd < 0)
		return cmd;
	return cmd_table[cmd](ds);
}

static int parse_cfg_file(struct imx_header *imxhdr, char *name,
		uint32_t entry_point)
{
	struct data_src ds;
	struct parse_helper *ph = &ds.ph;

	/* Be able to detect if the cfg file has no BOOT_FROM tag */
	g_flash_offset = FLASH_OFFSET_UNDEFINED;
	memset(&ds, 0, sizeof(struct data_src));
	ds.imxhdr = imxhdr;
	/*
	 * In order to not change the old imx cfg file
	 * by adding VERSION command into it, here need
	 * set up function ptr group to V1 by default.
	 */
	set_hdr_func(&ds, IMXIMAGE_V1);
	if (ph_open(ph, name)) {
		fprintf(stderr, "Error: %s - Can't open DCD file\n", name);
		exit(EXIT_FAILURE);
	}

	/* Very simple parsing, line starting with # are comments
	 * and are dropped
	 */
	for (;;) {
		ph->cmd_started = 0;
		if (ph_skip_separators(ph))
			break;
		ph->cmd_started = 1;
		if (parse_command(&ds)) {
			fprintf(stderr, "Error: invalid token %s[%d](%s)\n",
					name, ph->lineno, ph->p);
			exit(EXIT_FAILURE);
		}
		ds.cmd_cnt++;
	}
	ph_close(ph);
	/* Exit if there is no BOOT_FROM field specifying the flash_offset */
	if (g_flash_offset == FLASH_OFFSET_UNDEFINED) {
		fprintf(stderr, "Error: No BOOT_FROM tag in %s\n", name);
		exit(EXIT_FAILURE);
	}
	/* Set the imx header */
	return (*ds.set_imx_hdr)(&ds, entry_point, g_flash_offset);
}

static int imximage_check_image_types(uint8_t type)
{
	if (type == IH_TYPE_IMXIMAGE)
		return EXIT_SUCCESS;
	else
		return EXIT_FAILURE;
}

static int imximage_verify_header(unsigned char *ptr, int image_size,
			struct mkimage_params *params)
{
	struct imx_header *imx_hdr = (struct imx_header *) ptr;

	if (detect_imximage_version(imx_hdr) == IMXIMAGE_VER_INVALID)
		return -FDT_ERR_BADSTRUCTURE;

	return 0;
}

static void imximage_print_header(const void *ptr)
{
	struct imx_header *imx_hdr = (struct imx_header *) ptr;
	uint32_t version = detect_imximage_version(imx_hdr);

	switch (version) {
	case IMXIMAGE_V1:
		print_hdr_v1(imx_hdr);
		break;
	case IMXIMAGE_V2:
		print_hdr_v2(imx_hdr);
		break;
	default:
		err_imximage_version(version);
		break;
	}
}

int imximage_vrec_header(struct mkimage_params *params,
		struct image_type_params *tparams)
{
	struct imx_header *imxhdr;

	/*
	 * A little extra space to avoid access violation on dcd table overflow.
	 * Overflow is checked after entry is added.
	 */
	imxhdr = calloc(1, MAX_HEADER_SIZE + 32);
	if (!imxhdr) {
		fprintf(stderr, "Error: out of memory\n");
		exit(EXIT_FAILURE);
	}

	/* Parse dcd configuration file */
	imximage_params.header_size = parse_cfg_file(imxhdr, params->imagename,
			params->ep);
	imximage_params.hdr = imxhdr;
	return 0;
}

static void imximage_set_header(void *ptr, struct stat *sbuf, int ifd,
				struct mkimage_params *params)
{
	/* Set the size in header */
	uint32_t offset = (char *)header_size_ptr - (char *)imximage_params.hdr;
	uint32_t *p = (uint32_t *)((char *)ptr + offset);

	*p = sbuf->st_size + g_flash_offset;
}

int imximage_check_params(struct mkimage_params *params)
{
	if (!params)
		return -1;
	if (!strlen(params->imagename)) {
		fprintf(stderr, "Error: %s - Configuration file not specified, "
			"it is needed for imximage generation\n",
			params->cmdname);
		return -1;
	}
	/*
	 * Check parameters:
	 * XIP is not allowed and verify that incompatible
	 * parameters are not sent at the same time
	 * For example, if list is required a data image must not be provided
	 */
	return	(params->dflag && (params->fflag || params->lflag)) ||
		(params->fflag && (params->dflag || params->lflag)) ||
		(params->lflag && (params->dflag || params->fflag)) ||
		(params->xflag) || !(strlen(params->imagename));
}

/*
 * imximage parameters
 */
static struct image_type_params imximage_params = {
	.name		= "Freescale i.MX 5x Boot Image support",
	.vrec_header	= imximage_vrec_header,
	.check_image_type = imximage_check_image_types,
	.verify_header	= imximage_verify_header,
	.print_header	= imximage_print_header,
	.set_header	= imximage_set_header,
	.check_params	= imximage_check_params,
};

void init_imx_image_type(void)
{
	mkimage_register(&imximage_params);
}
