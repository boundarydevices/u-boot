// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <ufs.h>

#include "ufs.h"

static int ufs_mtk_link_startup_notify(struct ufs_hba *hba,
				       enum ufs_notify_change_status status)
{
	return 0;
}

static int ufs_mtk_hce_enable_notify(struct ufs_hba *hba,
				     enum ufs_notify_change_status status)
{
	return 0;
}

static int ufs_mtk_init(struct ufs_hba *hba)
{
	return 0;
}

static struct ufs_hba_ops ufs_mtk_hba_ops = {
	.init = ufs_mtk_init,
	.hce_enable_notify = ufs_mtk_hce_enable_notify,
	.link_startup_notify = ufs_mtk_link_startup_notify,
};

static int ufs_mtk_probe(struct udevice *dev)
{
	int err = ufshcd_probe(dev, &ufs_mtk_hba_ops);
	if (err)
		dev_err(dev, "ufshcd_probe() failed %d\n", err);

	return err;
}

static int ufs_mtk_bind(struct udevice *dev)
{
	struct udevice *scsi_dev;
	return ufs_scsi_bind(dev, &scsi_dev);
}

static const struct udevice_id ufs_mtk_of_match[] = {
	{
		.compatible = "mediatek,mt8195-ufshci",
	},
	{},
};

U_BOOT_DRIVER(ti_j721e_ufs) = {
	.name			= "ufshcd-mtk",
	.id			= UCLASS_UFS,
	.of_match		= ufs_mtk_of_match,
	.probe			= ufs_mtk_probe,
	.bind			= ufs_mtk_bind,
};
