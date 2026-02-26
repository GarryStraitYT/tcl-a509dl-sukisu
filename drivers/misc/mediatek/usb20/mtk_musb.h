/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __MUSB_MTK_MUSB_H__
#define __MUSB_MTK_MUSB_H__

/* Begin mod by jin.wang for otg issue on 2022-4-12 */
#if IS_ENABLED(CONFIG_MTK_MUSB_PHY) || IS_ENABLED(CONFIG_MUSB_OLD_ARCH)
#ifdef CONFIG_OF
extern struct musb *mtk_musb;

#ifdef USB2_PHY_V2
#define USB_PHY_OFFSET 0x300
#else
#define USB_PHY_OFFSET 0x800
#endif

#define USBPHY_READ8(offset) \
	readb((void __iomem *)\
		(((unsigned long)\
		mtk_musb->xceiv->io_priv)+USB_PHY_OFFSET+offset))
#define USBPHY_WRITE8(offset, value)  writeb(value, (void __iomem *)\
		(((unsigned long)mtk_musb->xceiv->io_priv)+USB_PHY_OFFSET+offset))
#define USBPHY_SET8(offset, mask) \
	USBPHY_WRITE8(offset, (USBPHY_READ8(offset)) | (mask))
#define USBPHY_CLR8(offset, mask) \
	USBPHY_WRITE8(offset, (USBPHY_READ8(offset)) & (~(mask)))

#define USBPHY_READ32(offset) \
	readl((void __iomem *)(((unsigned long)\
		mtk_musb->xceiv->io_priv)+USB_PHY_OFFSET+offset))
#define USBPHY_WRITE32(offset, value) \
	writel(value, (void __iomem *)\
		(((unsigned long)mtk_musb->xceiv->io_priv)+USB_PHY_OFFSET+offset))
#define USBPHY_SET32(offset, mask) \
	USBPHY_WRITE32(offset, (USBPHY_READ32(offset)) | (mask))
#define USBPHY_CLR32(offset, mask) \
	USBPHY_WRITE32(offset, (USBPHY_READ32(offset)) & (~(mask)))

#endif /* End of CONFIG_OF define */
#endif /* End of CONFIG_MTK_MUSB_PHY */
/* End mod by jin.wang */

struct musb;

enum usb_state_enum {
	USB_SUSPEND = 0,
	USB_UNCONFIGURED,
	USB_CONFIGURED
};

/* USB phy and clock */
extern bool usb_pre_clock(bool enable);
#ifdef CONFIG_MTK_UART_USB_SWITCH
extern void usb_phy_context_restore(void);
extern void usb_phy_context_save(void);
#endif

/* general USB */
extern bool mt_usb_is_device(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
extern void mt_usb_reconnect(void);
extern bool usb_cable_connected(struct musb *musb);
extern void musb_sync_with_bat(struct musb *musb, int usb_state);

bool is_saving_mode(void);

/* host and otg */
extern void mt_usb_init_drvvbus(void);

/* Begin add by jin.wang for task 11356632 on 2021-07-22 */
#if !defined(CONFIG_MTK_MUSB_DUAL_ROLE)
extern void mt_usb_set_vbus(struct musb *musb, int is_on);
#endif
/* End add by jin.wang for task 11356632 on 2021-07-22 */

extern void mt_usb_iddig_int(struct musb *musb);
extern void switch_int_to_device(struct musb *musb);
extern void switch_int_to_host(struct musb *musb);
extern void switch_int_to_host_and_mask(struct musb *musb);
extern void musb_session_restart(struct musb *musb);
#ifdef CONFIG_DUAL_ROLE_USB_INTF
extern int mt_usb_dual_role_init(struct musb *musb);
extern int mt_usb_dual_role_changed(struct musb *musb);
#endif /* CONFIG_DUAL_ROLE_USB_INTF */
extern bool is_usb_rdy(void);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
#if IS_ENABLED(CONFIG_TCT_CHARGER)
/* Begin added by bitao.xiong for task-11599163(BYD charger) on 2021-11-08 */
extern void Charger_Detect_Release_Pulldown(void);
/* End added by bitao.xiong for task-11599163(BYD charger) on 2021-11-08 */
#endif
#endif
