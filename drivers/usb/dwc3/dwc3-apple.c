// SPDX-License-Identifier: GPL-2.0
/*
 * Apple Silicon DWC3 Glue driver
 * Copyright (C) The Asahi Linux Contributors
 *
 * Based on:
 *  - dwc3-qcom.c Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *  - dwc3-of-simple.c Copyright (c) 2015 Texas Instruments Incorporated - https://www.ti.com
 */

#include <linux/of.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/workqueue.h>

#include "glue.h"
#include "io.h"

/*
 * This platform requires a very specific sequence of operations to bring up dwc3 and its USB3 PHY:
 *
 * 1) The PHY itself has to be brought up; for this we need to know the mode (USB3,
 *    USB3+DisplayPort, USB4, etc) and the lane orientation. This happens through typec_mux_set.
 * 2) DWC3 has to be brought up but we must not touch the gadget area or start xhci yet.
 * 3) The PHY bring-up has to be finalized and dwc3's PIPE interface has to be switched to the
 *    USB3 PHY, this is done inside phy_set_mode.
 * 4) We can now initialize xhci or gadget mode.
 *
 * We can switch 1 and 2 but 3 has to happen after (1 and 2) and 4 has to happen after 3.
 *
 * And then to bring this all down again:
 *
 * 1) DWC3 has to exit host or gadget mode and must no longer touch those registers
 * 2) The PHY has to switch dwc3's PIPE interface back to the dummy backend
 * 3) The PHY itself can be shut down, this happens from typec_mux_set
 *
 * We also can't transition the PHY from one mode to another while dwc3 is up and running (this is
 * slightly wrong, some transitions are possible, others aren't but because we have no documentation
 * for this I'd rather play it safe).
 *
 * After both the PHY and dwc3 are initialized we will only ever see a single "new device connected"
 * event. If we just keep them running only the first device plugged in will ever work. XHCI's port
 * status register actually does show the correct state but no interrupt ever comes in. In gadget
 * mode we don't even get a USBDisconnected event and everything looks like there's still something
 * connected on the other end.
 * This can be partially explained because the USB2 D+/D- lines are connected through a stateful
 * eUSB2 repeater which in turn is controlled by a variant of the TI TPS6598x USB PD chip which
 * resets the repeater out-of-band everytime the CC lines are (dis)connected. This then requires a
 * PHY reset to make sure the PHY and the eUSB2 repeater state are synchronized again.
 *
 * And to make this all extra fun: If we get the order of some of this wrong either the port is just
 * broken until a phy+dwc3 reset, or it's broken until a full SoC reset (likely because we can't
 * reset some parts of the PHY), or some watchdog kicks in after a few seconds and forces a full SoC
 * reset (mostly seen this with USB4/Thunderbolt but there's clearly some watchdog that hates
 * invalid states).
 *
 * Hence there's really no good way to keep dwc3 fully up and running after we disconnect a cable
 * because then we can't shut down the PHY anymore. And if we kept the PHY running in whatever mode
 * it was until the next cable is connected we'd need to tear it all down and bring it back up again
 * anyway to detect and use the next device.
 *
 * Instead, we just shut down everything when a cable is disconnected and transition to
 * DWC3_APPLE_NO_CABLE.
 * During initial probe we don't have any information about the connected cable and can't bring up
 * the PHY properly and thus also can't fully bring up dwc3. Instead, we just keep everything off
 * and defer the first dwc3 probe until we get the first cable connected event. Until then we stay
 * in DWC3_APPLE_PROBE_PENDING.
 * Once a cable is connected we then keep track of the controller mode here by transitioning to
 * DWC3_APPLE_HOST or DWC3_APPLE_DEVICE.
 */
enum dwc3_apple_state {
	DWC3_APPLE_PROBE_PENDING, /* Before first cable connection, dwc3_core_probe not called */
	DWC3_APPLE_NO_CABLE, /* No cable connected, dwc3 suspended after dwc3_core_exit */
	DWC3_APPLE_HOST, /* Cable connected, dwc3 in host mode */
	DWC3_APPLE_DEVICE, /* Cable connected, dwc3 in device mode */
};

#define DWC3_APPLE_MAX_ROLE_INPUTS 2

struct dwc3_apple;

/*
 * A fixed USB2 hub can fan one DWC3 controller out to several Type-C port
 * managers.  Give each HPM its own role-switch endpoint so a request from one
 * connector cannot overwrite the state of the other connector.
 */
struct dwc3_apple_role_input {
	struct dwc3_apple *appledwc;
	struct usb_role_switch *role_sw;
	enum usb_role role;
	u32 index;
};

/**
 * struct dwc3_apple - Apple-specific DWC3 USB controller
 * @dwc: Core DWC3 structure
 * @dev: Pointer to the device structure
 * @mmio_resource: Resource to be passed to dwc3_core_probe
 * @apple_regs: Apple-specific DWC3 registers
 * @reset: Reset control
 * @role_sw: USB role switch
 * @lock: Mutex for synchronizing access
 * @state: Current state of the controller, see documentation for the enum for details
 */
struct dwc3_apple {
	struct dwc3 dwc;

	struct device *dev;
	struct resource *mmio_resource;
	void __iomem *apple_regs;
	void __iomem *dwc3_reset_regs;
	void __iomem *xhci_regs;

	struct reset_control *reset;
	struct usb_role_switch *role_sw;
	struct gpio_desc *hub_reset_gpio;
	bool force_usb2_host;
	bool usb2_hub_always_on;
	bool usb2_retry_reset;
	bool usb2_retry_done;
	struct delayed_work usb2_retry_work;
	struct dwc3_apple_role_input role_inputs[DWC3_APPLE_MAX_ROLE_INPUTS];
	unsigned int num_role_inputs;

	struct mutex lock;

	enum dwc3_apple_state state;
};

#define to_dwc3_apple(d) container_of((d), struct dwc3_apple, dwc)

/*
 * Apple Silicon dwc3 vendor-specific registers
 *
 * These registers were identified by tracing XNU's memory access patterns and correlating them with
 * debug output over serial to determine their names. We don't exactly know what these do but
 * without these USB3 devices sometimes don't work.
 */
#define APPLE_DWC3_REGS_START 0xcd00
#define APPLE_DWC3_REGS_END 0xcdff

#define APPLE_DWC3_CIO_LFPS_OFFSET 0xcd38
#define APPLE_DWC3_CIO_LFPS_OFFSET_VALUE 0xf800f80

#define APPLE_DWC3_CIO_BW_NGT_OFFSET 0xcd3c
#define APPLE_DWC3_CIO_BW_NGT_OFFSET_VALUE 0xfc00fc0

#define APPLE_DWC3_CIO_LINK_TIMER 0xcd40
#define APPLE_DWC3_CIO_PENDING_HP_TIMER GENMASK(23, 16)
#define APPLE_DWC3_CIO_PENDING_HP_TIMER_VALUE 0x14
#define APPLE_DWC3_CIO_PM_LC_TIMER GENMASK(15, 8)
#define APPLE_DWC3_CIO_PM_LC_TIMER_VALUE 0xa
#define APPLE_DWC3_CIO_PM_ENTRY_TIMER GENMASK(7, 0)
#define APPLE_DWC3_CIO_PM_ENTRY_TIMER_VALUE 0x10

/* PIPE handler reset bits used by the ATC reset controller. */
#define APPLE_DWC3_PIPEHANDLER_AON_GEN 0x1c
#define APPLE_DWC3_FORCE_CLAMP_EN BIT(4)
#define APPLE_DWC3_RESET_N BIT(0)

#define XHCI_CAPLENGTH_MASK GENMASK(7, 0)
#define XHCI_HCSPARAMS1 0x4
#define XHCI_USBCMD 0x0
#define XHCI_USBSTS 0x4
#define XHCI_PORTSC_BASE 0x400
#define XHCI_PORTSC_STRIDE 0x10

static void dwc3_apple_dump_usb2_state(struct dwc3_apple *appledwc,
				      const char *phase)
{
	u32 cap, hcs1, op_base, portsc;
	u32 gusb2, gusb3, gctl;
	unsigned int max_ports, port;

	if (!appledwc->xhci_regs || !appledwc->dwc.regs)
		return;

	cap = readl(appledwc->xhci_regs);
	hcs1 = readl(appledwc->xhci_regs + XHCI_HCSPARAMS1);
	op_base = cap & XHCI_CAPLENGTH_MASK;
	max_ports = (hcs1 >> 24) & 0xff;

	gusb2 = dwc3_readl(&appledwc->dwc, DWC3_GUSB2PHYCFG(0));
	gusb3 = dwc3_readl(&appledwc->dwc, DWC3_GUSB3PIPECTL(0));
	gctl = dwc3_readl(&appledwc->dwc, DWC3_GCTL);

	dev_info(appledwc->dev,
		 "J700_USB2_CORE_STATE: phase=%s GUSB2PHYCFG=%08x SUSPHY2=%u GUSB3PIPECTL=%08x SUSPHY3=%u GCTL=%08x USBCMD=%08x USBSTS=%08x ports=%u\n",
		 phase, gusb2, !!(gusb2 & DWC3_GUSB2PHYCFG_SUSPHY),
		 gusb3, !!(gusb3 & DWC3_GUSB3PIPECTL_SUSPHY), gctl,
		 readl(appledwc->xhci_regs + op_base + XHCI_USBCMD),
		 readl(appledwc->xhci_regs + op_base + XHCI_USBSTS),
		 max_ports);

	for (port = 0; port < min(max_ports, 4U); port++) {
		portsc = readl(appledwc->xhci_regs + op_base +
			       XHCI_PORTSC_BASE + port * XHCI_PORTSC_STRIDE);
		dev_info(appledwc->dev,
			 "J700_USB2_ROOT_PORT: phase=%s port=%u PORTSC=%08x CCS=%u PED=%u PLS=%u PP=%u SPEED=%u CSC=%u PEC=%u PRC=%u PLC=%u\n",
			 phase, port + 1, portsc, !!(portsc & BIT(0)),
			 !!(portsc & BIT(1)), (portsc >> 5) & 0xf,
			 !!(portsc & BIT(9)), (portsc >> 10) & 0xf,
			 !!(portsc & BIT(17)), !!(portsc & BIT(18)),
			 !!(portsc & BIT(21)), !!(portsc & BIT(22)));
	}
}

static int dwc3_apple_reset_assert(struct dwc3_apple *appledwc)
{
	u32 value;

	if (appledwc->reset)
		return reset_control_assert(appledwc->reset);

	value = readl(appledwc->dwc3_reset_regs + APPLE_DWC3_PIPEHANDLER_AON_GEN);
	value &= ~APPLE_DWC3_RESET_N;
	value |= APPLE_DWC3_FORCE_CLAMP_EN;
	writel(value, appledwc->dwc3_reset_regs + APPLE_DWC3_PIPEHANDLER_AON_GEN);
	return 0;
}

static int dwc3_apple_reset_deassert(struct dwc3_apple *appledwc)
{
	u32 value;

	if (appledwc->reset)
		return reset_control_deassert(appledwc->reset);

	value = readl(appledwc->dwc3_reset_regs + APPLE_DWC3_PIPEHANDLER_AON_GEN);
	value &= ~APPLE_DWC3_FORCE_CLAMP_EN;
	value |= APPLE_DWC3_RESET_N;
	writel(value, appledwc->dwc3_reset_regs + APPLE_DWC3_PIPEHANDLER_AON_GEN);
	return 0;
}

static inline void dwc3_apple_writel(struct dwc3_apple *appledwc, u32 offset, u32 value)
{
	writel(value, appledwc->apple_regs + offset - APPLE_DWC3_REGS_START);
}

static inline u32 dwc3_apple_readl(struct dwc3_apple *appledwc, u32 offset)
{
	return readl(appledwc->apple_regs + offset - APPLE_DWC3_REGS_START);
}

static inline void dwc3_apple_mask(struct dwc3_apple *appledwc, u32 offset, u32 mask, u32 value)
{
	u32 reg;

	reg = dwc3_apple_readl(appledwc, offset);
	reg &= ~mask;
	reg |= value;
	dwc3_apple_writel(appledwc, offset, reg);
}

static void dwc3_apple_setup_cio(struct dwc3_apple *appledwc)
{
	dwc3_apple_writel(appledwc, APPLE_DWC3_CIO_LFPS_OFFSET, APPLE_DWC3_CIO_LFPS_OFFSET_VALUE);
	dwc3_apple_writel(appledwc, APPLE_DWC3_CIO_BW_NGT_OFFSET,
			  APPLE_DWC3_CIO_BW_NGT_OFFSET_VALUE);
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PENDING_HP_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PENDING_HP_TIMER,
				   APPLE_DWC3_CIO_PENDING_HP_TIMER_VALUE));
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PM_LC_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PM_LC_TIMER, APPLE_DWC3_CIO_PM_LC_TIMER_VALUE));
	dwc3_apple_mask(appledwc, APPLE_DWC3_CIO_LINK_TIMER, APPLE_DWC3_CIO_PM_ENTRY_TIMER,
			FIELD_PREP(APPLE_DWC3_CIO_PM_ENTRY_TIMER,
				   APPLE_DWC3_CIO_PM_ENTRY_TIMER_VALUE));
}

static void dwc3_apple_set_ptrcap(struct dwc3_apple *appledwc, u32 mode)
{
	guard(spinlock_irqsave)(&appledwc->dwc.lock);
	dwc3_set_prtcap(&appledwc->dwc, mode, false);
}

static int dwc3_apple_core_probe(struct dwc3_apple *appledwc)
{
	struct dwc3_probe_data probe_data = {};
	int ret;

	lockdep_assert_held(&appledwc->lock);
	WARN_ON_ONCE(appledwc->state != DWC3_APPLE_PROBE_PENDING);

	appledwc->dwc.dev = appledwc->dev;
	probe_data.dwc = &appledwc->dwc;
	probe_data.res = appledwc->mmio_resource;
	probe_data.ignore_clocks_and_resets = true;
	probe_data.skip_core_init_mode = true;
	probe_data.properties = DWC3_DEFAULT_PROPERTIES;

	ret = dwc3_core_probe(&probe_data);
	if (ret)
		return ret;

	appledwc->state = DWC3_APPLE_NO_CABLE;
	return 0;
}

static int dwc3_apple_core_init(struct dwc3_apple *appledwc)
{
	int ret;

	lockdep_assert_held(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_PROBE_PENDING:
		ret = dwc3_apple_core_probe(appledwc);
		if (ret)
			dev_err(appledwc->dev, "Failed to probe DWC3 Core, err=%d\n", ret);
		break;
	case DWC3_APPLE_NO_CABLE:
		ret = dwc3_core_init(&appledwc->dwc);
		if (ret)
			dev_err(appledwc->dev, "Failed to initialize DWC3 Core, err=%d\n", ret);
		break;
	default:
		/* Unreachable unless there's a bug in this driver */
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int dwc3_apple_init(struct dwc3_apple *appledwc, enum dwc3_apple_state state)
{
	int ret, ret_reset;

	lockdep_assert_held(&appledwc->lock);

	/*
	 * The USB2 PHY on this platform must be configured for host or device mode while it is
	 * still powered off and before dwc3 tries to access it. Otherwise, the new configuration
	 * will sometimes only take affect after the *next* time dwc3 is brought up which causes
	 * the connected device to just not work.
	 * The USB3 PHY must be configured later after dwc3 has already been initialized.
	 */
	switch (state) {
	case DWC3_APPLE_HOST:
		phy_set_mode(appledwc->dwc.usb2_generic_phy[0], PHY_MODE_USB_HOST);
		break;
	case DWC3_APPLE_DEVICE:
		phy_set_mode(appledwc->dwc.usb2_generic_phy[0], PHY_MODE_USB_DEVICE);
		break;
	default:
		/* Unreachable unless there's a bug in this driver */
		return -EINVAL;
	}

	ret = dwc3_apple_reset_deassert(appledwc);
	if (ret) {
		dev_err(appledwc->dev, "Failed to deassert reset, err=%d\n", ret);
		return ret;
	}

	ret = dwc3_apple_core_init(appledwc);
	if (ret)
		goto reset_assert;

	/*
	 * Now that the core is initialized and already went through dwc3_core_soft_reset we can
	 * configure some unknown Apple-specific settings and then bring up xhci or gadget mode.
	 */
	dwc3_apple_setup_cio(appledwc);

	switch (state) {
	case DWC3_APPLE_HOST:
		appledwc->dwc.dr_mode = USB_DR_MODE_HOST;
		dwc3_apple_set_ptrcap(appledwc, DWC3_GCTL_PRTCAP_HOST);
		/*
		 * This platform requires SUSPHY to be enabled here already in order to properly
		 * configure the PHY and switch dwc3's PIPE interface to USB3 PHY. The USB2 PHY
		 * has already been configured to the correct mode earlier.
		 */
		dwc3_enable_susphy(&appledwc->dwc, true);
		phy_set_mode(appledwc->dwc.usb3_generic_phy[0], PHY_MODE_USB_HOST);
		ret = dwc3_host_init(&appledwc->dwc);
		if (ret) {
			dev_err(appledwc->dev, "Failed to initialize host, ret=%d\n", ret);
			goto core_exit;
		}
		dwc3_apple_dump_usb2_state(appledwc, "host-init");

		break;
	case DWC3_APPLE_DEVICE:
		appledwc->dwc.dr_mode = USB_DR_MODE_PERIPHERAL;
		dwc3_apple_set_ptrcap(appledwc, DWC3_GCTL_PRTCAP_DEVICE);
		/*
		 * This platform requires SUSPHY to be enabled here already in order to properly
		 * configure the PHY and switch dwc3's PIPE interface to USB3 PHY. The USB2 PHY
		 * has already been configured to the correct mode earlier.
		 */
		dwc3_enable_susphy(&appledwc->dwc, true);
		phy_set_mode(appledwc->dwc.usb3_generic_phy[0], PHY_MODE_USB_DEVICE);
		ret = dwc3_gadget_init(&appledwc->dwc);
		if (ret) {
			dev_err(appledwc->dev, "Failed to initialize gadget, ret=%d\n", ret);
			goto core_exit;
		}
		break;
	default:
		/* Unreachable unless there's a bug in this driver */
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		goto core_exit;
	}

	appledwc->state = state;
	if (state == DWC3_APPLE_HOST && appledwc->usb2_retry_reset &&
	    !appledwc->usb2_retry_done)
		mod_delayed_work(system_wq, &appledwc->usb2_retry_work,
				 msecs_to_jiffies(5000));
	return 0;

core_exit:
	dwc3_core_exit(&appledwc->dwc);
reset_assert:
	ret_reset = dwc3_apple_reset_assert(appledwc);
	if (ret_reset)
		dev_warn(appledwc->dev, "Failed to assert reset, err=%d\n", ret_reset);

	return ret;
}

static int dwc3_apple_exit(struct dwc3_apple *appledwc)
{
	int ret = 0;

	lockdep_assert_held(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_PROBE_PENDING:
	case DWC3_APPLE_NO_CABLE:
		/* Nothing to do if we're already off */
		return 0;
	case DWC3_APPLE_DEVICE:
		dwc3_gadget_exit(&appledwc->dwc);
		break;
	case DWC3_APPLE_HOST:
		dwc3_host_exit(&appledwc->dwc);
		break;
	}

	/*
	 * This platform requires SUSPHY to be enabled in order to properly power down the PHY
	 * and switch dwc3's PIPE interface back to a dummy PHY (i.e. no USB3 support and USB2 via
	 * a different PHY connected through ULPI).
	 */
	dwc3_enable_susphy(&appledwc->dwc, true);
	dwc3_core_exit(&appledwc->dwc);
	appledwc->state = DWC3_APPLE_NO_CABLE;

	ret = dwc3_apple_reset_assert(appledwc);
	if (ret) {
		dev_err(appledwc->dev, "Failed to assert reset, err=%d\n", ret);
		return ret;
	}

	return 0;
}

static void dwc3_apple_usb2_retry_work(struct work_struct *work)
{
	struct dwc3_apple *appledwc =
		container_of(to_delayed_work(work), struct dwc3_apple,
			     usb2_retry_work);
	int ret;

	guard(mutex)(&appledwc->lock);

	if (appledwc->usb2_retry_done ||
	    appledwc->state != DWC3_APPLE_HOST)
		return;

	appledwc->usb2_retry_done = true;
	dev_info(appledwc->dev,
		 "J700 USB2 diagnostic: retrying DWC3 after shipping eUSB2 USBCTL settle\n");
	dwc3_apple_dump_usb2_state(appledwc, "pre-retry");

	/*
	 * The first hub reset is followed by a five-second settle in probe.  The
	 * late I2C worker then applies the ADT VLF0 and repeater tables and opens
	 * the TICD2E22.  Reinitialize the controller once so the post-open PHY
	 * power-on includes Apple's recovered 5-ms settle immediately before
	 * USBCTL is enabled.  This is the cycle-35 full retry with only that
	 * shipping delay added.
	 */
	dev_info(appledwc->dev,
		 "J700_USB2_DWC_RETRY: post-repeater-open USBCTL settle enabled\n");

	ret = dwc3_apple_exit(appledwc);
	if (!ret)
		ret = dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	dwc3_apple_dump_usb2_state(appledwc, "post-retry");

	if (ret)
		dev_err(appledwc->dev,
			"J700_USB2_DWC_RETRY_FAIL: err=%d\n", ret);
	else
		dev_info(appledwc->dev, "J700_USB2_DWC_RETRY_PASS\n");
}

static int dwc3_usb_role_switch_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);
	enum usb_role requested_role = role;
	int ret;

	guard(mutex)(&appledwc->lock);

	/*
	 * Some machines put a fixed USB2 hub between DWC3 and multiple Type-C
	 * connectors.  The hub is DWC3's permanently connected USB2 peer; an HPM
	 * disconnect only describes one downstream connector and must not shut the
	 * shared host down.  Device mode also needs a separate repeater bypass mux
	 * which is not described yet, so retain host mode until that mux exists.
	 */
	if (appledwc->usb2_hub_always_on && role != USB_ROLE_HOST) {
		role = USB_ROLE_HOST;
		dev_info_ratelimited(appledwc->dev,
			"J700_USB2_FAKE_MUX: requested=%d effective=%d state=%d\n",
			requested_role, role, appledwc->state);
	}

	/*
	 * Skip role switches if appledwc is already in the desired state. The
	 * USB-C port controller on M2 and M1/M2 Pro/Max/Ultra devices issues
	 * additional interrupts which results in usb_role_switch_set_role()
	 * calls with the current role.
	 * Ignore those calls here to ensure the USB-C port controller and
	 * appledwc are in a consistent state.
	 * This matches the behaviour in __dwc3_set_mode().
	 * Do no handle USB_ROLE_NONE for DWC3_APPLE_NO_CABLE and
	 * DWC3_APPLE_PROBE_PENDING since that is no-op anyway.
	 */
	if (appledwc->state == DWC3_APPLE_HOST && role == USB_ROLE_HOST)
		return 0;
	if (appledwc->state == DWC3_APPLE_DEVICE && role == USB_ROLE_DEVICE)
		return 0;

	/*
	 * We need to tear all of dwc3 down and re-initialize it every time a cable is
	 * connected or disconnected or when the mode changes. See the documentation for enum
	 * dwc3_apple_state for details.
	 */
	ret = dwc3_apple_exit(appledwc);
	if (ret)
		return ret;

	switch (role) {
	case USB_ROLE_NONE:
		/* Nothing to do if no cable is connected */
		return 0;
	case USB_ROLE_HOST:
		return dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	case USB_ROLE_DEVICE:
		return dwc3_apple_init(appledwc, DWC3_APPLE_DEVICE);
	default:
		dev_err(appledwc->dev, "Invalid target role: %d\n", role);
		return -EINVAL;
	}
}

static enum usb_role dwc3_usb_role_switch_get(struct usb_role_switch *sw)
{
	struct dwc3_apple *appledwc = usb_role_switch_get_drvdata(sw);

	guard(mutex)(&appledwc->lock);

	switch (appledwc->state) {
	case DWC3_APPLE_HOST:
		return USB_ROLE_HOST;
	case DWC3_APPLE_DEVICE:
		return USB_ROLE_DEVICE;
	case DWC3_APPLE_NO_CABLE:
	case DWC3_APPLE_PROBE_PENDING:
		return USB_ROLE_NONE;
	default:
		/* Unreachable unless there's a bug in this driver */
		dev_err(appledwc->dev, "Invalid internal state: %d\n", appledwc->state);
		return USB_ROLE_NONE;
	}
}

static int dwc3_apple_role_input_set(struct usb_role_switch *sw,
				      enum usb_role role)
{
	struct dwc3_apple_role_input *input =
		usb_role_switch_get_drvdata(sw);
	struct dwc3_apple *appledwc = input->appledwc;
	enum usb_role old_role;
	int ret = 0;

	guard(mutex)(&appledwc->lock);

	old_role = input->role;
	input->role = role;

	/*
	 * The VL122 is the controller's permanent USB2 peer.  HPM roles describe
	 * downstream connectors, so NONE or DEVICE on either input must never
	 * tear down the shared host.  Preserve the per-input state for hotplug
	 * diagnostics while presenting the only valid aggregate: HOST.
	 */
	if (appledwc->state != DWC3_APPLE_HOST) {
		ret = dwc3_apple_exit(appledwc);
		if (!ret)
			ret = dwc3_apple_init(appledwc, DWC3_APPLE_HOST);
	}

	/*
	 * Only the rear/DFU-side connector (input 0) owns the direct ATC lanes.
	 * CD321x calls its Type-C mux before this role-switch endpoint, including
	 * for same-role mode changes. Re-issue the aggregate host mode here so
	 * the live DWC PIPE follows USB3 <-> dummy transitions without tearing
	 * down the fixed USB2 hub and its downstream devices.
	 */
	if (!ret && input->index == 0 &&
	    appledwc->dwc.usb3_generic_phy[0]) {
		int pipe_ret;

		pipe_ret = phy_set_mode(appledwc->dwc.usb3_generic_phy[0],
					PHY_MODE_USB_HOST);
		if (pipe_ret)
			dev_warn(appledwc->dev,
				 "J700_USB3_PIPE_SYNC_FAIL: requested=%d err=%d\n",
				 role, pipe_ret);
		else
			dev_info(appledwc->dev,
				 "J700_USB3_PIPE_SYNC_PASS: requested=%d aggregate=host\n",
				 role);
	}

	dev_info(appledwc->dev,
		 "J700_USB2_FAKE_MUX_INPUT: input=%u old=%d requested=%d aggregate=%d roles=%d/%d state=%d ret=%d\n",
		 input->index, old_role, role, USB_ROLE_HOST,
		 appledwc->role_inputs[0].role,
		 appledwc->role_inputs[1].role, appledwc->state, ret);

	/*
	 * A connected-at-boot HPM event is the best available indication that
	 * the slow fixed hub can now be observed.  Keep the one-time delayed
	 * DWC3 retry armed, but never reset an already enumerated hub for later
	 * downstream hotplug events.
	 */
	if (!ret && old_role != role && appledwc->usb2_retry_reset &&
	    !appledwc->usb2_retry_done)
		mod_delayed_work(system_wq, &appledwc->usb2_retry_work,
				 msecs_to_jiffies(5000));

	return ret;
}

static enum usb_role
dwc3_apple_role_input_get(struct usb_role_switch *sw)
{
	struct dwc3_apple_role_input *input =
		usb_role_switch_get_drvdata(sw);
	struct dwc3_apple *appledwc = input->appledwc;
	enum usb_role role;

	guard(mutex)(&appledwc->lock);
	role = input->role;

	return role;
}

static void dwc3_apple_remove_role_inputs(struct dwc3_apple *appledwc)
{
	while (appledwc->num_role_inputs) {
		struct dwc3_apple_role_input *input =
			&appledwc->role_inputs[--appledwc->num_role_inputs];

		usb_role_switch_unregister(input->role_sw);
		input->role_sw = NULL;
	}
}

static int dwc3_apple_setup_role_inputs(struct dwc3_apple *appledwc)
{
	struct fwnode_handle *inputs, *child = NULL;
	struct usb_role_switch_desc desc = { };
	int ret = 0;

	if (!appledwc->usb2_hub_always_on)
		return 0;

	inputs = device_get_named_child_node(appledwc->dev,
					     "usb2-role-inputs");
	if (!inputs)
		return dev_err_probe(appledwc->dev, -EINVAL,
				     "Missing fixed-hub role inputs\n");

	while ((child = fwnode_get_next_child_node(inputs, child))) {
		struct dwc3_apple_role_input *input;
		const char *name;
		u32 index;

		if (!fwnode_property_present(child, "usb-role-switch"))
			continue;
		if (appledwc->num_role_inputs >= DWC3_APPLE_MAX_ROLE_INPUTS) {
			ret = -E2BIG;
			break;
		}

		input = &appledwc->role_inputs[appledwc->num_role_inputs];
		index = appledwc->num_role_inputs;
		fwnode_property_read_u32(child, "reg", &index);
		input->appledwc = appledwc;
		input->index = index;
		input->role = USB_ROLE_NONE;

		name = devm_kasprintf(appledwc->dev, GFP_KERNEL,
				       "%s-usb2-input%u",
				       dev_name(appledwc->dev), index);
		if (!name) {
			ret = -ENOMEM;
			break;
		}

		desc.fwnode = child;
		desc.set = dwc3_apple_role_input_set;
		desc.get = dwc3_apple_role_input_get;
		desc.driver_data = input;
		desc.name = name;
		input->role_sw = usb_role_switch_register(appledwc->dev,
							 &desc);
		if (IS_ERR(input->role_sw)) {
			ret = PTR_ERR(input->role_sw);
			input->role_sw = NULL;
			break;
		}

		appledwc->num_role_inputs++;
	}
	fwnode_handle_put(child);
	fwnode_handle_put(inputs);

	if (!ret && appledwc->num_role_inputs != DWC3_APPLE_MAX_ROLE_INPUTS)
		ret = -EINVAL;
	if (ret) {
		dwc3_apple_remove_role_inputs(appledwc);
		return dev_err_probe(appledwc->dev, ret,
				     "Failed to register fixed-hub role inputs\n");
	}

	dev_info(appledwc->dev,
		 "J700_USB2_FAKE_MUX_READY: inputs=%u aggregate=host hub=always-on\n",
		 appledwc->num_role_inputs);
	return 0;
}

static int dwc3_apple_setup_role_switch(struct dwc3_apple *appledwc)
{
	struct usb_role_switch_desc dwc3_role_switch = { NULL };

	dwc3_role_switch.fwnode = dev_fwnode(appledwc->dev);
	dwc3_role_switch.set = dwc3_usb_role_switch_set;
	dwc3_role_switch.get = dwc3_usb_role_switch_get;
	dwc3_role_switch.driver_data = appledwc;
	appledwc->role_sw = usb_role_switch_register(appledwc->dev, &dwc3_role_switch);
	if (IS_ERR(appledwc->role_sw))
		return PTR_ERR(appledwc->role_sw);

	return 0;
}

static int dwc3_apple_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_apple *appledwc;
	int ret;

	appledwc = devm_kzalloc(&pdev->dev, sizeof(*appledwc), GFP_KERNEL);
	if (!appledwc)
		return -ENOMEM;

	appledwc->dev = &pdev->dev;
	mutex_init(&appledwc->lock);
	INIT_DELAYED_WORK(&appledwc->usb2_retry_work,
			  dwc3_apple_usb2_retry_work);
	appledwc->force_usb2_host = device_property_read_bool(dev, "apple,force-usb2-host");
	appledwc->usb2_hub_always_on = device_property_read_bool(
		dev, "apple,usb2-hub-always-on");
	appledwc->usb2_retry_reset =
		device_property_read_bool(dev, "apple,j700-usb2-retry-reset");

	appledwc->reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(appledwc->reset))
		return dev_err_probe(&pdev->dev, PTR_ERR(appledwc->reset),
				     "Failed to get reset control\n");
	if (!appledwc->reset) {
		if (!appledwc->force_usb2_host && !appledwc->usb2_hub_always_on)
			return dev_err_probe(dev, -ENODEV, "Missing reset control\n");

		appledwc->dwc3_reset_regs =
			devm_platform_ioremap_resource_byname(pdev, "dwc3-reset");
		if (IS_ERR(appledwc->dwc3_reset_regs))
			return dev_err_probe(dev, PTR_ERR(appledwc->dwc3_reset_regs),
					     "Failed to map DWC3 reset registers\n");
	}

	ret = dwc3_apple_reset_assert(appledwc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to assert reset, err=%d\n", ret);
		return ret;
	}

	/*
	 * J700 routes the external USB-C connectors through an onboard VIA hub.
	 * iBoot can leave the hub in reset across handoff, so pulse its dedicated
	 * active-low reset before the host controller starts enumerating devices.
	 */
	appledwc->hub_reset_gpio = devm_gpiod_get_optional(dev, "hub-reset",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(appledwc->hub_reset_gpio))
		return dev_err_probe(dev, PTR_ERR(appledwc->hub_reset_gpio),
				     "Failed to acquire onboard hub reset GPIO\n");
	if (appledwc->hub_reset_gpio) {
		usleep_range(100, 200);
		gpiod_set_value_cansleep(appledwc->hub_reset_gpio, 0);
		/* The VL122 is not ready in time for an immediate DWC3 reset. */
		msleep(5000);
		dev_info(dev, "pulsed onboard USB hub reset; 5s settle complete\n");
	}

	appledwc->mmio_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dwc3-core");
	if (!appledwc->mmio_resource) {
		dev_err(dev, "Failed to get DWC3 MMIO\n");
		return -EINVAL;
	}
	appledwc->xhci_regs = devm_ioremap(dev, appledwc->mmio_resource->start,
					   min_t(resource_size_t,
						 resource_size(appledwc->mmio_resource),
						 0x1000));
	if (!appledwc->xhci_regs)
		return dev_err_probe(dev, -ENOMEM, "Failed to map xHCI diagnostics\n");

	appledwc->apple_regs = devm_platform_ioremap_resource_byname(pdev, "dwc3-apple");
	if (IS_ERR(appledwc->apple_regs))
		return dev_err_probe(dev, PTR_ERR(appledwc->apple_regs),
				     "Failed to map Apple-specific MMIO\n");

	/*
	 * On this platform, DWC3 can only be brought up after parts of the PHY have been
	 * initialized with knowledge of the target mode and cable orientation from typec_set_mux.
	 * Since this has not happened here we cannot setup DWC3 yet and instead defer this until
	 * the first cable is connected. See the documentation for enum dwc3_apple_state for
	 * details.
	 */
	appledwc->state = DWC3_APPLE_PROBE_PENDING;
	ret = dwc3_apple_setup_role_switch(appledwc);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to setup role switch\n");
	ret = dwc3_apple_setup_role_inputs(appledwc);
	if (ret) {
		usb_role_switch_unregister(appledwc->role_sw);
		return ret;
	}

	if (appledwc->force_usb2_host || appledwc->usb2_hub_always_on) {
		ret = dwc3_usb_role_switch_set(appledwc->role_sw, USB_ROLE_HOST);
		if (ret) {
			dwc3_apple_remove_role_inputs(appledwc);
			usb_role_switch_unregister(appledwc->role_sw);
			return dev_err_probe(dev, ret, "Failed to force USB2 host mode\n");
		}
		if (appledwc->usb2_hub_always_on)
			dev_info(dev,
				 "J700_USB2_HUB_ALWAYS_ON: shared USB2 host initialized independently of HPM cable state\n");
		else
			dev_info(dev,
				 "USB2-only host mode active; ATC/SuperSpeed disabled\n");
	}

	return 0;
}

static void dwc3_apple_remove(struct platform_device *pdev)
{
	struct dwc3 *dwc = platform_get_drvdata(pdev);
	struct dwc3_apple *appledwc = to_dwc3_apple(dwc);

	cancel_delayed_work_sync(&appledwc->usb2_retry_work);
	guard(mutex)(&appledwc->lock);

	dwc3_apple_remove_role_inputs(appledwc);
	usb_role_switch_unregister(appledwc->role_sw);

	/*
	 * If we're still in DWC3_APPLE_PROBE_PENDING we never got any cable connected event and
	 * dwc3_core_probe was never called and there's hence no need to call dwc3_core_remove.
	 * dwc3_apple_exit can be called unconditionally because it checks the state itself.
	 */
	dwc3_apple_exit(appledwc);
	if (appledwc->state != DWC3_APPLE_PROBE_PENDING)
		dwc3_core_remove(&appledwc->dwc);
}

static const struct of_device_id dwc3_apple_of_match[] = {
	{ .compatible = "apple,t8103-dwc3" },
	{}
};
MODULE_DEVICE_TABLE(of, dwc3_apple_of_match);

static struct platform_driver dwc3_apple_driver = {
	.probe		= dwc3_apple_probe,
	.remove		= dwc3_apple_remove,
	.driver		= {
		.name	= "dwc3-apple",
		.of_match_table	= dwc3_apple_of_match,
	},
};

module_platform_driver(dwc3_apple_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_DESCRIPTION("DesignWare DWC3 Apple Silicon Glue Driver");
