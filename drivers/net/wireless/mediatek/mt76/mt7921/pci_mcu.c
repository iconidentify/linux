// SPDX-License-Identifier: BSD-3-Clause-Clear
/* Copyright (C) 2021 MediaTek Inc. */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iommu.h>

#include "mt7921.h"
#include "../dma.h"
#include "mcu.h"

static void
mt7932_dma_path_trace(struct mt792x_dev *dev, struct mt76_queue *q,
		      struct mt76_desc *desc)
{
	struct iommu_domain *domain;
	void __iomem *dart, *port;
	phys_addr_t desc_phys = 0, buf_phys = 0;
	dma_addr_t buf_iova;
	u32 info = le32_to_cpu(READ_ONCE(desc->info));

	buf_iova = le32_to_cpu(READ_ONCE(desc->buf0));
	buf_iova |= (dma_addr_t)(info & GENMASK(3, 0)) << 32;
	domain = iommu_get_domain_for_dev(dev->mt76.dev);
	if (domain) {
		desc_phys = iommu_iova_to_phys(domain, q->desc_dma);
		buf_phys = iommu_iova_to_phys(domain, buf_iova);
	}
	dev_info(dev->mt76.dev,
		 "J700_MT7932_IOMMU_PATH: domain=%u desc_iova=%pad desc_phys=%pa buf_iova=%pad buf_phys=%pa\n",
		 !!domain, &q->desc_dma, &desc_phys, &buf_iova, &buf_phys);

	port = ioremap(0x390028000ULL, SZ_32K);
	if (port) {
		dev_info(dev->mt76.dev,
			 "J700_MT7932_RID2SID: map0=%08x map1=%08x map2=%08x map3=%08x\n",
			 readl(port + 0x3000), readl(port + 0x3004),
			 readl(port + 0x3008), readl(port + 0x300c));
		iounmap(port);
	} else {
		dev_info(dev->mt76.dev, "J700_MT7932_RID2SID: unavailable\n");
	}

	dart = ioremap(0x390000000ULL, SZ_8K);
	if (dart) {
		dev_info(dev->mt76.dev,
			 "J700_MT7932_DART_STATE: error=%08x mask=%08x addr=%08x/%08x streams=%08x enable=%08x tcr0=%08x ttbr0=%08x tcr16=%08x ttbr16=%08x tcr17=%08x ttbr17=%08x tcr18=%08x ttbr18=%08x\n",
			 readl(dart + 0x100), readl(dart + 0x104),
			 readl(dart + 0x174), readl(dart + 0x170),
			 readl(dart + 0x1c0), readl(dart + 0xc00),
			 readl(dart + 0x1000), readl(dart + 0x1400),
			 readl(dart + 0x1040), readl(dart + 0x1440),
			 readl(dart + 0x1044), readl(dart + 0x1444),
			 readl(dart + 0x1048), readl(dart + 0x1448));
		iounmap(dart);
	} else {
		dev_info(dev->mt76.dev, "J700_MT7932_DART_STATE: unavailable\n");
	}
}

static void
mt7932_mcu_ring_trace(struct mt792x_dev *dev, struct mt76_queue *q,
		      const char *phase)
{
	struct mt76_desc *desc;
	u16 desc_idx;
	u32 tx = MT_TX_RING_BASE + q->hw_idx * 0x10;
	u32 rx = MT_RX_EVENT_RING_BASE;
	u32 own = mt7921_reg_map_l1(dev, MT_TOP_LPCR_HOST_BAND0);

	desc_idx = q->head ? q->head - 1 : q->ndesc - 1;
	desc = &q->desc[desc_idx];
	dma_rmb();

	dev_info(dev->mt76.dev,
		 "J700_MT7932_MCU_RING: phase=%s hw=%u tx=%08x/%08x/%08x/%08x rx=%08x/%08x/%08x/%08x int=%08x/%08x\n",
		 phase, q->hw_idx,
		 mt76_rr(dev, tx), mt76_rr(dev, tx + 0x4),
		 mt76_rr(dev, tx + 0x8), mt76_rr(dev, tx + 0xc),
		 mt76_rr(dev, rx), mt76_rr(dev, rx + 0x4),
		 mt76_rr(dev, rx + 0x8), mt76_rr(dev, rx + 0xc),
		 mt76_rr(dev, MT_WFDMA0_HOST_INT_STA),
		 mt76_rr(dev, MT_WFDMA0_HOST_INT_ENA));
	dev_info(dev->mt76.dev,
		 "J700_MT7932_WFDMA_STATE: phase=%s glo=%08x rst=%08x busy=%08x ext0=%08x hif=%08x mcu=%08x dummy=%08x own=%08x pm=%08x pciint=%08x shdl=%08x\n",
		 phase, mt76_rr(dev, MT_WFDMA0_GLO_CFG),
		 mt76_rr(dev, MT_WFDMA0_RST),
		 mt76_rr(dev, MT_WFDMA0_BUSY_ENA),
		 mt76_rr(dev, MT_WFDMA0_GLO_CFG_EXT0),
		 mt76_rr(dev, MT_WFDMA_EXT_CSR_HIF_MISC),
		 mt76_rr(dev, MT_MCU_CMD),
		 mt76_rr(dev, MT_WFDMA_DUMMY_CR), mt76_rr(dev, own),
		 mt76_rr(dev, MT_PCIE_MAC_PM),
		 mt76_rr(dev, MT_PCIE_MAC_INT_ENABLE),
		 mt76_rr(dev, MT_DMASHDL_SW_CONTROL));
	dev_info(dev->mt76.dev,
		 "J700_MT7932_DMA_DESC: phase=%s q=%u/%u/%d idx=%u dma=%pad words=%08x/%08x/%08x/%08x\n",
		 phase, q->head, q->tail, q->queued, desc_idx, &q->desc_dma,
		 le32_to_cpu(READ_ONCE(desc->buf0)),
		 le32_to_cpu(READ_ONCE(desc->ctrl)),
		 le32_to_cpu(READ_ONCE(desc->buf1)),
		 le32_to_cpu(READ_ONCE(desc->info)));
	/* The direct platform snapshot is safe only during first contact.  Once
	 * patch-finish changes firmware ownership, the same DART window can gate.
	 */
	if (!strcmp(phase, "after-10ms") && q->head == 2)
		mt7932_dma_path_trace(dev, q, desc);
}

static void
mt7932_fw_start_memory_trace(struct mt792x_dev *dev,
			     struct mt76_queue *tx, const char *phase)
{
	struct mt76_queue *rx = &dev->mt76.q_rx[MT_RXQ_MCU];
	u16 tx_idx = tx->head ? tx->head - 1 : tx->ndesc - 1;
	u16 rx_idx = rx->tail;
	struct mt76_desc *txd = &tx->desc[tx_idx];
	struct mt76_desc *rxd = &rx->desc[rx_idx];

	dma_rmb();
	dev_info(dev->mt76.dev,
		 "J700_MT7932_FW_START_MEMORY: phase=%s hw=%u tx=%u/%u/%d idx=%u desc=%08x/%08x/%08x/%08x rx=%u/%u/%d idx=%u desc=%08x/%08x/%08x/%08x\n",
		 phase, tx->hw_idx, tx->head, tx->tail, tx->queued, tx_idx,
		 le32_to_cpu(READ_ONCE(txd->buf0)),
		 le32_to_cpu(READ_ONCE(txd->buf1)),
		 le32_to_cpu(READ_ONCE(txd->ctrl)),
		 le32_to_cpu(READ_ONCE(txd->info)),
		 rx->head, rx->tail, rx->queued, rx_idx,
		 le32_to_cpu(READ_ONCE(rxd->buf0)),
		 le32_to_cpu(READ_ONCE(rxd->buf1)),
		 le32_to_cpu(READ_ONCE(rxd->ctrl)),
		 le32_to_cpu(READ_ONCE(rxd->info)));
}

int mt7921e_driver_own(struct mt792x_dev *dev)
{
	u32 reg = mt7921_reg_map_l1(dev, MT_TOP_LPCR_HOST_BAND0);

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
	bool trace_first_contact = false;
	int ret;

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	mdev->mcu.timeout = is_mt7932(mdev) ? 5 * HZ : 3 * HZ;

	if (cmd == MCU_CMD(FW_SCATTER))
		txq = MT_MCUQ_FWDL;
	q = mdev->q_mcu[txq];

	/* AppleSunriseWLAN 25G83 has both init_cmd_port (+0x8a) and
	 * fw_dl_port (+0x8c) set to zero.  nicTxInitCmd passes either field to
	 * kalDevPortWrite, which directly indexes the WFDMA TX-ring table.
	 */
	if (is_mt7932(mdev) &&
	    (cmd == MCU_CMD(PATCH_SEM_CONTROL) ||
	     cmd == MCU_CMD(TARGET_ADDRESS_LEN_REQ) ||
	     cmd == MCU_CMD(PATCH_START_REQ) ||
	     cmd == MCU_CMD(PATCH_FINISH_REQ) ||
	     cmd == MCU_CMD(FW_SCATTER) ||
	     cmd == MCU_CMD(FW_START_REQ))) {
		q = dev->mphy.q_tx[MT_TXQ_BE];
		if (cmd != MCU_CMD(FW_SCATTER) || q->head < 16)
			dev_info(mdev->dev,
				 "J700_MT7932_APPLE_INIT_ROUTE: cmd=0x%08x hw=%u\n",
				 cmd, q->hw_idx);
	}

	if (is_mt7932(mdev) && cmd == MCU_CMD(PATCH_SEM_CONTROL) &&
	    q->head < 3)
		trace_first_contact = true;

	if (trace_first_contact)
		mt7932_mcu_ring_trace(dev, q, "before-kick");

	ret = mt76_tx_queue_skb_raw(dev, q, skb, 0);
	if (trace_first_contact) {
		mt7932_mcu_ring_trace(dev, q, "after-kick");
		usleep_range(10000, 11000);
		mt7932_mcu_ring_trace(dev, q, "after-10ms");
	}
	if (is_mt7932(mdev) && cmd == MCU_CMD(FW_START_REQ)) {
		mt7932_fw_start_memory_trace(dev, q, "after-kick");
		usleep_range(10000, 11000);
		mt7932_fw_start_memory_trace(dev, q, "after-10ms");
	}

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

	err = mt7921e_driver_own(dev);
	if (err)
		return err;

	mt76_rmw_field(dev, MT_PCIE_MAC_PM, MT_PCIE_MAC_PM_L0S_DIS, 1);

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
