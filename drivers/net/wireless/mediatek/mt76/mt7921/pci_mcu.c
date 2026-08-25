// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2021 MediaTek Inc. */

#include "mt7921.h"
#include "mcu.h"

int mt7921e_driver_own(struct mt792x_dev *dev)
{
	u32 reg;
	int ret;

	if (is_mt7932(&dev->mt76)) {
		ret = __mt792xe_mcu_drv_pmctrl(dev);
		if (!ret)
			dev_info(dev->mt76.dev,
				 "J700_MT7932_DRIVER_OWN_PASS: register=0x%08x request=0x%08x sync_mask=0x%08x source=apple-connac2\n",
				 MT_CONN_ON_LPCTL,
				 (u32)PCIE_LPCR_HOST_CLR_OWN,
				 (u32)PCIE_LPCR_HOST_OWN_SYNC);
		return ret;
	}

	reg = mt7921_reg_map_l1(dev, MT_TOP_LPCR_HOST_BAND0);
	mt76_wr(dev, reg, MT_TOP_LPCR_HOST_DRV_OWN);
	if (!mt76_poll_msec(dev, reg, MT_TOP_LPCR_HOST_FW_OWN,
			    0, 500)) {
		dev_err(dev->mt76.dev, "Timeout for driver own\n");
		return -EIO;
	}

	return 0;
}

static int
mt7921_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
			int cmd, int *seq)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	enum mt76_mcuq_id txq = MT_MCUQ_WM;
	struct mt76_queue *q;
	int ret;

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	mdev->mcu.timeout = is_mt7932(mdev) ? 5 * HZ : 3 * HZ;

	if (cmd == MCU_CMD(FW_SCATTER))
		txq = MT_MCUQ_FWDL;
	q = mdev->q_mcu[txq];

	ret = mt76_tx_queue_skb_raw(dev, q, skb, 0);

	return ret;
}

int mt7921e_mcu_init(struct mt792x_dev *dev)
{
	static const struct mt76_mcu_ops mt7921_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.mcu_skb_send_msg = mt7921_mcu_send_message,
		.mcu_parse_response = mt7921_mcu_parse_response,
	};
	struct mt76_queue *cleanup_q;
	int err;

	dev->mt76.mcu_ops = &mt7921_mcu_ops;

	/* MT7932 ownership was already claimed before WFDMA setup in PCI probe,
	 * matching AppleSunriseWLAN's single pre-initialization transition.
	 */
	if (!is_mt7932(&dev->mt76)) {
		err = mt7921e_driver_own(dev);
		if (err)
			return err;
	}

	/* MT_PCIE_MAC_PM is BAR 0x10194, which __mt7921_reg_addr() maps from
	 * chip address 0x74030000 (PCIE_MAC_IREG) - the device's own PCIe MAC
	 * control block.  mt76's window assumptions have already been shown
	 * wrong for MT7932 once, at 0x7c050000 (Cycles 196-198), and the
	 * endpoint leaves the link cleanly with no uncorrectable AER errors.
	 * Skip the write on MT7932 until the layout is confirmed.
	 */
	if (!is_mt7932(&dev->mt76))
		mt76_rmw_field(dev, MT_PCIE_MAC_PM, MT_PCIE_MAC_PM_L0S_DIS, 1);
	else
		dev_info(dev->mt76.dev,
			 "J700_MT7932_PCIE_MAC_PM_SKIP: pm=0x%08x left untouched\n",
			 mt76_rr(dev, MT_PCIE_MAC_PM));

	err = mt7921_run_firmware(dev);
	cleanup_q = is_mt7932(&dev->mt76) ? dev->mphy.q_tx[MT_TXQ_BE] :
						  dev->mt76.q_mcu[MT_MCUQ_FWDL];

	if (!is_mt7932(&dev->mt76) || !err)
		mt76_queue_tx_cleanup(dev, cleanup_q, false);
	else
		dev_warn(dev->mt76.dev,
			 "J700_MT7932_FWDL_CLEANUP_SKIPPED: preserving diagnostic state after err=%d\n",
			 err);

	return err;
}
