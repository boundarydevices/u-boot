bool is_usb_boot(void);
#if CONFIG_IS_ENABLED(MTK_MAC_FUSE)
int mtk_get_mac_from_fuse(unsigned char*);
#endif
