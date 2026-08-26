// SPDX-License-Identifier: ISC
/*
 * MediaTek MT79xx Bluetooth PCIe transport
 *
 * Copyright (C) 2026 Aurora Silicon
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define MTK_PCIE_VENDOR_ID		0x14c3
#define MTK_PCIE_DEVICE_MT7922_BT	0x792a
#define MTK_PCIE_DEVICE_MT7923_BT	0x792b
#define MTK_PCIE_DEVICE_MT7932_BT	0x793b

/* AppleSunriseBluetooth 25G83 _btmtk_read_mmio_addr() first programs the
 * connection-to-PCIe selector for 0x70010000, then its generic fallback remap
 * selector for logical window 0x1890, before reading 0x70010200 through BAR0.
 */
#define MTK_PCIE_BAR0_REMAP_SEL		0x23014
#define MTK_PCIE_BAR0_DYNAMIC_WINDOW	0xc0000
#define MTK_PCIE_BAR0_FALLBACK_SEL	0xfe3b8
#define MTK_PCIE_DYNAMIC_REMAP_1890	0x188d1890
#define MTK_PCIE_CONN_CHIP_ID		0x70010200
#define MTK_PCIE_CHIP_ID_WINDOW		(MTK_PCIE_BAR0_DYNAMIC_WINDOW + \
					 (MTK_PCIE_CONN_CHIP_ID & 0xffff))

/* AppleSunriseBluetooth 25G83 setRomVersionProperty() reads config dword
 * 0x48c and extracts bits [6:4].  Values 0 and 1 select the dedicated
 * MT7932B0 and MT7932B1 IPC images respectively; all others fail closed.
 */
#define MTK_PCIE_ROM_VERSION_CONFIG	0x48c

#define MTK_PCIE_FW_B0 \
	"mediatek/MT7932B0_OS_TypeB_0.1.44.0_241001003711.bin"
#define MTK_PCIE_FW_B1 \
	"mediatek/MT7932B1_OS_TypeB_0.1.133.0_260128190103.bin"
#define MTK_PCIE_PTX \
	"mediatek/MT7932_PTB_IzubaA_0.1.0.0_20251021141303.ptx"
#define MTK_PCIE_BT_CALIBRATION	"mediatek/j700-mt7932-btcal.bin"
#define MTK_PCIE_BT_ADDRESS	"mediatek/j700-mt7932-bdaddr.bin"
#define MTK_PCIE_FW_TRAILER_SIZE		32
#define MTK_PCIE_FW_ALPS_OFFSET		16
#define MTK_PCIE_SRS_OPCODE		0xfdd0
#define MTK_PCIE_SRS_PREFIX_SIZE		4
#define MTK_PCIE_SRS_CHUNK_SIZE		0xf0
#define MTK_PCIE_SET_BDADDR_OPCODE	0xfc1a

#define MTK_PCIE_BAR0_DOORBELL		0x19484
#define MTK_PCIE_BAR0_IPC_STATUS		0x33804
#define MTK_PCIE_BAR0_IMAGE_DMA_LO	0x33810
#define MTK_PCIE_BAR0_IMAGE_DMA_HI	0x33814
#define MTK_PCIE_BAR0_IMAGE_LENGTH	0x33818
#define MTK_PCIE_BAR0_IMAGE_RESPONSE	0x3381c
#define MTK_PCIE_BAR0_HOST_CONTROL	0x33828
#define MTK_PCIE_BAR0_CONTEXT_DMA_LO	0x33830
#define MTK_PCIE_BAR0_CONTEXT_DMA_HI	0x33834
#define MTK_PCIE_BAR0_WINDOW_BASE_LO	0x3383c
#define MTK_PCIE_BAR0_WINDOW_BASE_HI	0x33840
#define MTK_PCIE_BAR0_WINDOW_SPAN	0x33844
#define MTK_PCIE_BAR0_FWDL_LTR		0x33c28
#define MTK_PCIE_BAR0_FWDL_SEMAPHORE	0xc1060
#define MTK_PCIE_BAR0_FWDL_RELEASE	0xc1260
#define MTK_PCIE_BAR0_TRANSPORT_REMAP	0xfe3b4
#define MTK_PCIE_FWDL_REMAP		0x18070000
#define MTK_PCIE_TRANSPORT_REMAP		0x7000188a

#define MTK_PCIE_IPC_CONTEXT_SIZE	0x68
#define MTK_PCIE_IPC_PERI_INFO_SIZE	0x10
#define MTK_PCIE_IPC_TR_COUNTER_SIZE	0x18
#define MTK_PCIE_IPC_CR_COUNTER_SIZE	0x4
#define MTK_PCIE_IPC_CONTROL_POOL_SIZE	0x2200
#define MTK_PCIE_IPC_CR_RING_SIZE	0xc00
#define MTK_PCIE_IPC_SNAPSHOT_SIZE	0x1400
#define MTK_PCIE_IPC_RING_DEPTH		128
#define MTK_PCIE_IPC_RING_COUNT		11
#define MTK_PCIE_IPC_CONTROL_RECORD_SIZE	0x44
#define MTK_PCIE_IPC_CONTROL_DOORBELL_BIT	20
#define MTK_PCIE_IPC_DESCRIPTOR_TAG	0x0c
#define MTK_PCIE_IPC_DESCRIPTOR_STATE	0x0f
#define MTK_PCIE_IPC_TOTAL_BYTES		0xea6b0
#define MTK_PCIE_IPC_STATUS_TIMEOUT_MS	1000

struct btmtk_pcie_ipc_context {
	u8 reserved_00[0x08];
	__le64 peri_info_dma;
	__le64 cr_hia_dma;
	__le64 tr_tia_dma;
	__le64 cr_tia_dma;
	__le64 tr_hia_dma;
	__le32 config;
	__le64 cr_ring_dma;
	__le64 control_pool_dma;
	__le16 cr_depth;
	__le16 tr_depth;
	u8 reserved_48[0x09];
	u8 constant_51;
	u8 reserved_52;
	u8 constant_53;
	u8 reserved_54[0x14];
} __packed;

static_assert(sizeof(struct btmtk_pcie_ipc_context) ==
	      MTK_PCIE_IPC_CONTEXT_SIZE);
static_assert(offsetof(struct btmtk_pcie_ipc_context, cr_ring_dma) == 0x34);
static_assert(offsetof(struct btmtk_pcie_ipc_context, control_pool_dma) ==
	      0x3c);
static_assert(offsetof(struct btmtk_pcie_ipc_context, constant_51) == 0x51);
static_assert(offsetof(struct btmtk_pcie_ipc_context, constant_53) == 0x53);

struct btmtk_pcie_dma_region {
	void *vaddr;
	dma_addr_t dma;
	size_t size;
};

struct btmtk_pcie_ring_layout {
	u32 bytes;
	u16 stride;
	u16 max_payload;
	u8 doorbell_bit;
	bool tx;
	bool valid;
};

static const struct btmtk_pcie_ring_layout
btmtk_pcie_ring_layout[MTK_PCIE_IPC_RING_COUNT] = {
	[0]  = { 0x00c00, 0x018,    0, 19, false, true  },
	[1]  = { 0x08e00, 0x11c,  268, 21, true,  true  },
	[2]  = { 0x08e00, 0x11c,  268, 22, false, true  },
	[3]  = { 0x20600, 0x40c, 1020, 23, true,  true  },
	[4]  = { 0x20600, 0x40c, 1020, 24, false, true  },
	[5]  = { 0x20c00, 0x418, 1032, 25, true,  true  },
	[6]  = { 0x20c00, 0x418, 1032, 26, false, true  },
	[7]  = {       0,     0,    0,  0, false, false },
	[8]  = { 0x20c00, 0x418, 1032, 27, false, true  },
	[9]  = { 0x18c00, 0x318,  776, 28, true,  true  },
	[10] = { 0x18c00, 0x318,  776, 29, false, true  },
};

struct btmtk_pcie_ipc {
	struct btmtk_pcie_dma_region context;
	struct btmtk_pcie_dma_region peri_info;
	struct btmtk_pcie_dma_region tr_hia;
	struct btmtk_pcie_dma_region tr_tia;
	struct btmtk_pcie_dma_region cr_hia;
	struct btmtk_pcie_dma_region cr_tia;
	struct btmtk_pcie_dma_region control_pool;
	struct btmtk_pcie_dma_region cr_ring;
	struct btmtk_pcie_dma_region snapshot;
	struct btmtk_pcie_dma_region image;
	struct btmtk_pcie_dma_region ring[MTK_PCIE_IPC_RING_COUNT];
	u16 tr_tia_previous[MTK_PCIE_IPC_RING_COUNT];
	u64 dma_min;
	u64 dma_end;
	size_t total_bytes;
	bool dma_window_valid;
};

struct btmtk_pcie_dev {
	struct pci_dev *pdev;
	void __iomem *bar0;
	void __iomem *bar2;
	u16 fallback_chip_id;
	struct btmtk_pcie_ipc *ipc;
	u8 rom_version;
	u8 firmware_trailer[MTK_PCIE_FW_TRAILER_SIZE];
	spinlock_t ipc_lock;
	spinlock_t ring_lock;
	wait_queue_head_t ipc_wait;
	struct timer_list ipc_timer;
	struct hci_dev *hdev;
	u8 ipc_state;
	u8 remote_status;
	int ipc_error;
	bool rom_version_valid;
	bool firmware_active;
	bool bus_master_enabled;
	bool context_published;
	bool irq_registered;
	bool rings_prepared;
	bool hci_registered;
	bool radio_config_loaded;
	bool stopping;
};

static bool probe_remap;
module_param(probe_remap, bool, 0444);

static bool probe_rom_version;
module_param(probe_rom_version, bool, 0444);

static bool prepare_ipc;
module_param(prepare_ipc, bool, 0444);

static bool activate_firmware;
module_param(activate_firmware, bool, 0444);

static bool start_ipc;
module_param(start_ipc, bool, 0444);

static bool prepare_rings;
module_param(prepare_rings, bool, 0444);

static bool enable_hci;
module_param(enable_hci, bool, 0444);

static bool load_radio_config;
module_param(load_radio_config, bool, 0444);

/* Keep the hardware gate selectable within J700's measured U-Boot bootargs
 * limit.  The explicit Boolean parameters remain available for offline
 * contracts, while gate=N enables exactly the prerequisite prefix through N.
 */
static uint gate;
module_param(gate, uint, 0444);

static int btmtk_pcie_apply_gate_selector(void)
{
	if (!gate)
		return 0;
	if (gate < 2 || gate > 8)
		return -EINVAL;

	probe_rom_version = gate >= 2;
	probe_remap = gate >= 2;
	prepare_ipc = gate >= 3;
	activate_firmware = gate >= 4;
	start_ipc = gate >= 5;
	prepare_rings = gate >= 6;
	enable_hci = gate >= 7;
	load_radio_config = gate >= 8;

	return 0;
}

static int btmtk_pcie_alloc_region(struct btmtk_pcie_dev *bdev,
				   struct btmtk_pcie_dma_region *region,
				   size_t size)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	size_t total;
	u64 end;

	region->vaddr = dmam_alloc_coherent(&bdev->pdev->dev, size,
					    &region->dma, GFP_KERNEL);
	if (!region->vaddr)
		return -ENOMEM;

	memset(region->vaddr, 0, size);
	region->size = size;
	if (check_add_overflow((u64)region->dma, (u64)size, &end) ||
	    check_add_overflow(ipc->total_bytes, size, &total))
		return -EOVERFLOW;

	ipc->total_bytes = total;
	if (!ipc->dma_window_valid) {
		ipc->dma_min = region->dma;
		ipc->dma_end = end;
		ipc->dma_window_valid = true;
	} else {
		ipc->dma_min = min_t(u64, ipc->dma_min, region->dma);
		ipc->dma_end = max_t(u64, ipc->dma_end, end);
	}

	return 0;
}

static void btmtk_pcie_fill_ipc_context(struct btmtk_pcie_ipc *ipc)
{
	struct btmtk_pcie_ipc_context *context = ipc->context.vaddr;

	put_unaligned_le64(ipc->peri_info.dma, &context->peri_info_dma);
	put_unaligned_le64(ipc->cr_hia.dma, &context->cr_hia_dma);
	put_unaligned_le64(ipc->tr_tia.dma, &context->tr_tia_dma);
	put_unaligned_le64(ipc->cr_tia.dma, &context->cr_tia_dma);
	put_unaligned_le64(ipc->tr_hia.dma, &context->tr_hia_dma);
	put_unaligned_le32(0x000b0001, &context->config);
	put_unaligned_le64(ipc->cr_ring.dma, &context->cr_ring_dma);
	put_unaligned_le64(ipc->control_pool.dma, &context->control_pool_dma);
	put_unaligned_le16(MTK_PCIE_IPC_RING_DEPTH, &context->cr_depth);
	put_unaligned_le16(MTK_PCIE_IPC_RING_DEPTH, &context->tr_depth);
	context->constant_51 = 13;
	context->constant_53 = 2;
}

static int btmtk_pcie_prepare_ipc(struct btmtk_pcie_dev *bdev)
{
	struct btmtk_pcie_ipc *ipc;
	u64 span;
	u16 command;
	int ring, err;

	err = dma_set_mask_and_coherent(&bdev->pdev->dev, DMA_BIT_MASK(64));
	if (err)
		return err;

	ipc = devm_kzalloc(&bdev->pdev->dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;
	bdev->ipc = ipc;

	err = btmtk_pcie_alloc_region(bdev, &ipc->context,
				      MTK_PCIE_IPC_CONTEXT_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->tr_hia,
				      MTK_PCIE_IPC_TR_COUNTER_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->tr_tia,
				      MTK_PCIE_IPC_TR_COUNTER_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->cr_hia,
				      MTK_PCIE_IPC_CR_COUNTER_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->cr_tia,
				      MTK_PCIE_IPC_CR_COUNTER_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->peri_info,
				      MTK_PCIE_IPC_PERI_INFO_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->control_pool,
				      MTK_PCIE_IPC_CONTROL_POOL_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->cr_ring,
				      MTK_PCIE_IPC_CR_RING_SIZE);
	if (err)
		return err;
	err = btmtk_pcie_alloc_region(bdev, &ipc->snapshot,
				      MTK_PCIE_IPC_SNAPSHOT_SIZE);
	if (err)
		return err;

	for (ring = 1; ring < MTK_PCIE_IPC_RING_COUNT; ring++) {
		if (!btmtk_pcie_ring_layout[ring].valid)
			continue;
		err = btmtk_pcie_alloc_region(bdev, &ipc->ring[ring],
					      btmtk_pcie_ring_layout[ring].bytes);
		if (err)
			return err;
	}

	if (ipc->total_bytes != MTK_PCIE_IPC_TOTAL_BYTES)
		return -EINVAL;
	span = ipc->dma_end - ipc->dma_min;
	if (span > U32_MAX)
		return -ERANGE;

	btmtk_pcie_fill_ipc_context(ipc);
	pci_read_config_word(bdev->pdev, PCI_COMMAND, &command);
	if (command & PCI_COMMAND_MASTER)
		return -EBUSY;

	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_IPC_LAYOUT_PASS: total=%zu span=%llu context=104 tr=24/24 cr=4/4 peri=16 rings=10 depth=128 published=0 pci_master=0 irq=0 firmware=0 hci=0\n",
		 ipc->total_bytes, span);

	return 0;
}

static bool btmtk_pcie_bar0_has(struct btmtk_pcie_dev *bdev, u32 offset)
{
	resource_size_t length = pci_resource_len(bdev->pdev, 0);

	return length >= sizeof(u32) && offset <= length - sizeof(u32);
}

static int btmtk_pcie_select_firmware(struct btmtk_pcie_dev *bdev,
				      const char **name)
{
	if (!bdev->rom_version_valid)
		return -EINVAL;
	if (bdev->rom_version == 0) {
		*name = MTK_PCIE_FW_B0;
		return 0;
	}
	if (bdev->rom_version == 1) {
		*name = MTK_PCIE_FW_B1;
		return 0;
	}

	return -ENODEV;
}

static int btmtk_pcie_load_firmware_image(struct btmtk_pcie_dev *bdev,
					  const char *name,
					  size_t *body_length,
					  size_t *aligned_length)
{
	struct btmtk_pcie_dma_region *image = &bdev->ipc->image;
	const struct firmware *firmware;
	const u8 *trailer;
	size_t body, aligned;
	int err;

	err = request_firmware(&firmware, name, &bdev->pdev->dev);
	if (err)
		return err;
	if (firmware->size <= MTK_PCIE_FW_TRAILER_SIZE) {
		err = -EINVAL;
		goto out_release;
	}

	body = firmware->size - MTK_PCIE_FW_TRAILER_SIZE;
	if (body > U32_MAX || check_add_overflow(body, (size_t)3, &aligned)) {
		err = -EOVERFLOW;
		goto out_release;
	}
	aligned &= ~(size_t)3;
	trailer = firmware->data + body;
	if (memcmp(trailer + MTK_PCIE_FW_ALPS_OFFSET, "ALPS", 4) ||
	    memcmp(trailer + MTK_PCIE_FW_ALPS_OFFSET + 4,
		   "\x8a\x10\x8a\x10", 4)) {
		err = -EINVAL;
		goto out_release;
	}
	if (image->vaddr) {
		err = -EBUSY;
		goto out_release;
	}

	/*
	 * The ROM-patch image is temporary.  Apple releases it after response 1
	 * and boot stage 2, before publishing the persistent IPC window.  Keeping
	 * it in btmtk_pcie_alloc_region() would leave the freed image covered by
	 * dma_min/dma_end and expose stale DMA space to the running controller.
	 */
	image->vaddr = dma_alloc_coherent(&bdev->pdev->dev, aligned,
					  &image->dma, GFP_KERNEL);
	if (!image->vaddr) {
		err = -ENOMEM;
		goto out_release;
	}
	image->size = aligned;
	memcpy(image->vaddr, firmware->data, body);
	if (aligned != body)
		memset(image->vaddr + body, 0, aligned - body);
	memcpy(bdev->firmware_trailer, trailer, MTK_PCIE_FW_TRAILER_SIZE);
	*body_length = body;
	*aligned_length = aligned;

out_release:
	release_firmware(firmware);
	return err;
}

static void btmtk_pcie_free_firmware_image(struct btmtk_pcie_dev *bdev)
{
	struct btmtk_pcie_dma_region *image;

	if (!bdev->ipc)
		return;
	image = &bdev->ipc->image;
	if (!image->vaddr)
		return;

	dma_free_coherent(&bdev->pdev->dev, image->size, image->vaddr,
			  image->dma);
	memset(image, 0, sizeof(*image));
}

static void btmtk_pcie_select_fwdl_window(struct btmtk_pcie_dev *bdev)
{
	writel(MTK_PCIE_FWDL_REMAP,
	       bdev->bar0 + MTK_PCIE_BAR0_REMAP_SEL);
	writel(MTK_PCIE_DYNAMIC_REMAP_1890,
	       bdev->bar0 + MTK_PCIE_BAR0_FALLBACK_SEL);
}

static int btmtk_pcie_activate_firmware(struct btmtk_pcie_dev *bdev)
{
	static const u32 offsets[] = {
		MTK_PCIE_BAR0_DOORBELL,
		MTK_PCIE_BAR0_IMAGE_RESPONSE,
		MTK_PCIE_BAR0_IMAGE_DMA_LO,
		MTK_PCIE_BAR0_IMAGE_DMA_HI,
		MTK_PCIE_BAR0_IMAGE_LENGTH,
		MTK_PCIE_BAR0_FWDL_LTR,
		MTK_PCIE_BAR0_FWDL_SEMAPHORE,
		MTK_PCIE_BAR0_FWDL_RELEASE,
		MTK_PCIE_BAR0_TRANSPORT_REMAP,
		MTK_PCIE_BAR0_FALLBACK_SEL,
	};
	const char *firmware_name;
	size_t body_length, aligned_length;
	u32 boot_config = 0, response = 0, semaphore = 0;
	u16 command;
	int attempt, err;

	for (attempt = 0; attempt < ARRAY_SIZE(offsets); attempt++) {
		if (!btmtk_pcie_bar0_has(bdev, offsets[attempt]))
			return -ENXIO;
	}

	err = btmtk_pcie_select_firmware(bdev, &firmware_name);
	if (err)
		return err;
	err = btmtk_pcie_load_firmware_image(bdev, firmware_name,
					     &body_length, &aligned_length);
	if (err)
		return err;

	pci_set_master(bdev->pdev);
	err = pci_read_config_word(bdev->pdev, PCI_COMMAND, &command);
	if (err != PCIBIOS_SUCCESSFUL || !(command & PCI_COMMAND_MASTER)) {
		pci_clear_master(bdev->pdev);
		err = -EIO;
		goto out_free_image;
	}
	bdev->bus_master_enabled = true;

	dma_wmb();
	writel(MTK_PCIE_TRANSPORT_REMAP,
	       bdev->bar0 + MTK_PCIE_BAR0_TRANSPORT_REMAP);
	writel(1, bdev->bar0 + MTK_PCIE_BAR0_FWDL_LTR);
	writel(lower_32_bits(bdev->ipc->image.dma),
	       bdev->bar0 + MTK_PCIE_BAR0_IMAGE_DMA_LO);
	writel(upper_32_bits(bdev->ipc->image.dma),
	       bdev->bar0 + MTK_PCIE_BAR0_IMAGE_DMA_HI);
	writel((u32)body_length, bdev->bar0 + MTK_PCIE_BAR0_IMAGE_LENGTH);

	btmtk_pcie_select_fwdl_window(bdev);
	for (attempt = 0; attempt < 5000; attempt++) {
		semaphore = readl(bdev->bar0 + MTK_PCIE_BAR0_FWDL_SEMAPHORE);
		if (semaphore & 1)
			break;
		udelay(1000);
	}
	if (!(semaphore & 1)) {
		err = -ETIMEDOUT;
		goto out_clear_master;
	}

	dma_wmb();
	writel(BIT(11), bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
	for (attempt = 0; attempt < 1000; attempt++) {
		response = readl(bdev->bar0 + MTK_PCIE_BAR0_IMAGE_RESPONSE);
		err = pci_read_config_dword(bdev->pdev,
					   MTK_PCIE_ROM_VERSION_CONFIG,
					   &boot_config);
		if (err != PCIBIOS_SUCCESSFUL) {
			err = -EIO;
			goto out_clear_master;
		}
		if (response == 1 && (boot_config & 0xf) == 2)
			break;
		udelay(1000);
	}
	if (response != 1 || (boot_config & 0xf) != 2) {
		err = -ETIMEDOUT;
		goto out_clear_master;
	}

	btmtk_pcie_select_fwdl_window(bdev);
	writel(1, bdev->bar0 + MTK_PCIE_BAR0_FWDL_RELEASE);
	btmtk_pcie_free_firmware_image(bdev);
	bdev->firmware_active = true;
	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_FWDL_PASS: firmware=MT7932B%u body=%zu aligned=%zu semaphore=1 doorbell=11 response=1 boot_stage=2 release=1 image_dma_released=1 context_published=0 ipc_running=0 irq=0 hci=0\n",
		 bdev->rom_version, body_length, aligned_length);

	return 0;

out_clear_master:
	pci_clear_master(bdev->pdev);
	bdev->bus_master_enabled = false;
out_free_image:
	btmtk_pcie_free_firmware_image(bdev);
	return err;
}

static void btmtk_pcie_set_host_control(struct btmtk_pcie_dev *bdev,
					u32 state)
{
	writel(state, bdev->bar0 + MTK_PCIE_BAR0_HOST_CONTROL);
	/* The firmware must observe the new control state before its doorbell. */
	wmb();
	writel(BIT(13), bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
}

static void btmtk_pcie_ipc_timer(struct timer_list *timer)
{
	struct btmtk_pcie_dev *bdev =
		timer_container_of(bdev, timer, ipc_timer);
	unsigned long flags;

	spin_lock_irqsave(&bdev->ipc_lock, flags);
	if (!bdev->stopping && bdev->ipc_state != 2) {
		bdev->ipc_state = 4;
		bdev->ipc_error = -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&bdev->ipc_lock, flags);
	wake_up_all(&bdev->ipc_wait);
}

static void btmtk_pcie_arm_ipc_timer(struct btmtk_pcie_dev *bdev)
{
	if (!timer_pending(&bdev->ipc_timer))
		mod_timer(&bdev->ipc_timer,
			  jiffies +
			  msecs_to_jiffies(MTK_PCIE_IPC_STATUS_TIMEOUT_MS));
}

static int btmtk_pcie_publish_context(struct btmtk_pcie_dev *bdev)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	u64 span;

	if (!ipc || !ipc->dma_window_valid)
		return -EINVAL;
	if (ipc->image.vaddr)
		return -EBUSY;
	span = ipc->dma_end - ipc->dma_min;
	if (span > U32_MAX)
		return -ERANGE;

	dma_wmb();
	writel(lower_32_bits(ipc->context.dma),
	       bdev->bar0 + MTK_PCIE_BAR0_CONTEXT_DMA_LO);
	writel(upper_32_bits(ipc->context.dma),
	       bdev->bar0 + MTK_PCIE_BAR0_CONTEXT_DMA_HI);
	writel(lower_32_bits(ipc->dma_min),
	       bdev->bar0 + MTK_PCIE_BAR0_WINDOW_BASE_LO);
	writel(upper_32_bits(ipc->dma_min),
	       bdev->bar0 + MTK_PCIE_BAR0_WINDOW_BASE_HI);
	writel((u32)span, bdev->bar0 + MTK_PCIE_BAR0_WINDOW_SPAN);
	bdev->context_published = true;

	return 0;
}

static int btmtk_pcie_rx_packet_type(unsigned int ring)
{
	switch (ring) {
	case 2:
		return HCI_EVENT_PKT;
	case 4:
		return HCI_SCODATA_PKT;
	case 6:
		return HCI_ACLDATA_PKT;
	case 8:
		return 0;
	case 10:
		return HCI_ISODATA_PKT;
	default:
		return -EINVAL;
	}
}

static int btmtk_pcie_validate_rx_descriptor(unsigned int ring,
					     const u8 *descriptor,
					     u16 expected_slot,
					     u32 *length)
{
	const struct btmtk_pcie_ring_layout *layout =
		&btmtk_pcie_ring_layout[ring];
	u32 actual_length;

	actual_length = descriptor[1] | descriptor[2] << 8 |
			descriptor[3] << 16;
	if (descriptor[0] != 2 || get_unaligned_le64(descriptor + 4) ||
	    get_unaligned_le16(descriptor + MTK_PCIE_IPC_DESCRIPTOR_TAG) !=
	    expected_slot || descriptor[0x0e] ||
	    descriptor[MTK_PCIE_IPC_DESCRIPTOR_STATE] != 4 ||
	    actual_length > layout->max_payload)
		return -EPROTO;

	*length = actual_length;
	return 0;
}

static void btmtk_pcie_account_rx(struct hci_dev *hdev, unsigned int ring,
				  unsigned int length)
{
	hdev->stat.byte_rx += length;
	switch (ring) {
	case 2:
		hdev->stat.evt_rx++;
		break;
	case 4:
		hdev->stat.sco_rx++;
		break;
	case 6:
		hdev->stat.acl_rx++;
		break;
	default:
		break;
	}
}

static int btmtk_pcie_process_rx_ring(struct btmtk_pcie_dev *bdev,
				      unsigned int ring)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	const struct btmtk_pcie_ring_layout *layout =
		&btmtk_pcie_ring_layout[ring];
	struct sk_buff *frames[MTK_PCIE_IPC_RING_DEPTH - 1] = { };
	__le16 *tr_hia = ipc->tr_hia.vaddr;
	__le16 *tr_tia = ipc->tr_tia.vaddr;
	struct hci_dev *hdev = READ_ONCE(bdev->hdev);
	unsigned int count, frame_count = 0, i;
	unsigned long flags;
	u16 new_tia, previous, slot, hia;
	int packet_type, err = 0;
	u32 length;

	packet_type = btmtk_pcie_rx_packet_type(ring);
	if (packet_type < 0 || !layout->valid || layout->tx)
		return -EINVAL;

	dma_rmb();
	spin_lock_irqsave(&bdev->ring_lock, flags);
	new_tia = le16_to_cpu(READ_ONCE(tr_tia[ring])) & 0x7f;
	previous = ipc->tr_tia_previous[ring] & 0x7f;
	if (new_tia == previous)
		goto out_unlock;

	/* Apple caches the new TIA before walking the completed descriptors. */
	ipc->tr_tia_previous[ring] = new_tia;
	count = (new_tia - previous) & 0x7f;
	slot = (new_tia - count) & 0x7f;
	for (i = 0; i < count; i++) {
		u8 *descriptor = ipc->ring[ring].vaddr +
				 slot * layout->stride;

		err = btmtk_pcie_validate_rx_descriptor(ring, descriptor, slot,
							&length);
		if (err)
			goto out_free;
		if (packet_type && hdev) {
			struct sk_buff *skb = bt_skb_alloc(length, GFP_ATOMIC);

			if (!skb) {
				err = -ENOMEM;
				goto out_free;
			}
			memcpy(skb_put(skb, length), descriptor + 0x10, length);
			hci_skb_pkt_type(skb) = packet_type;
			frames[frame_count++] = skb;
		}
		slot = (slot + 1) & 0x7f;
	}

	hia = le16_to_cpu(READ_ONCE(tr_hia[ring])) & 0x7f;
	for (i = 0; i < count; i++) {
		u8 *descriptor = ipc->ring[ring].vaddr +
				 hia * layout->stride;

		memset(descriptor, 0, layout->stride);
		put_unaligned_le16(hia,
				   descriptor + MTK_PCIE_IPC_DESCRIPTOR_TAG);
		descriptor[MTK_PCIE_IPC_DESCRIPTOR_STATE] = 0;
		hia = (hia + 1) & 0x7f;
	}
	WRITE_ONCE(tr_hia[ring], cpu_to_le16(hia));
	dma_wmb();
	writel(BIT(layout->doorbell_bit),
	       bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
	spin_unlock_irqrestore(&bdev->ring_lock, flags);

	for (i = 0; i < frame_count; i++) {
		unsigned int frame_length = frames[i]->len;

		btmtk_pcie_account_rx(hdev, ring, frame_length);
		if (hci_recv_frame(hdev, frames[i]))
			hdev->stat.err_rx++;
	}
	return 0;

out_free:
	while (frame_count)
		kfree_skb(frames[--frame_count]);
out_unlock:
	spin_unlock_irqrestore(&bdev->ring_lock, flags);
	return err;
}

static void btmtk_pcie_process_cr_ring(struct btmtk_pcie_dev *bdev)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	__le16 *cr_hia = ipc->cr_hia.vaddr;
	__le16 *cr_tia = ipc->cr_tia.vaddr;
	unsigned long flags;
	u16 producer, consumer;

	dma_rmb();
	spin_lock_irqsave(&bdev->ring_lock, flags);
	producer = le16_to_cpu(READ_ONCE(cr_hia[0])) & 0x7f;
	consumer = le16_to_cpu(READ_ONCE(cr_tia[0])) & 0x7f;
	if (producer != consumer) {
		WRITE_ONCE(cr_tia[0], cpu_to_le16(producer));
		dma_wmb();
		writel(BIT(btmtk_pcie_ring_layout[0].doorbell_bit),
		       bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
	}
	spin_unlock_irqrestore(&bdev->ring_lock, flags);
}

static int btmtk_pcie_process_rings(struct btmtk_pcie_dev *bdev)
{
	static const u8 rx_rings[] = { 2, 4, 6, 8, 10 };
	unsigned int i;
	int err;

	btmtk_pcie_process_cr_ring(bdev);
	for (i = 0; i < ARRAY_SIZE(rx_rings); i++) {
		err = btmtk_pcie_process_rx_ring(bdev, rx_rings[i]);
		if (err)
			return err;
	}

	return 0;
}

static irqreturn_t btmtk_pcie_irq(int irq, void *data)
{
	struct btmtk_pcie_dev *bdev = data;
	struct device *dev = &bdev->pdev->dev;
	unsigned long flags;
	u32 control = 0;
	u8 remote, state;
	bool publish = false, wake = false;
	int err = 0;

	if (READ_ONCE(bdev->stopping))
		return IRQ_NONE;
	dma_rmb();
	remote = READ_ONCE(((u8 *)bdev->ipc->peri_info.vaddr)[4]);
	if (remote <= 1)
		remote = readl(bdev->bar0 + MTK_PCIE_BAR0_IPC_STATUS) & 0xff;
	if (remote > 4)
		return IRQ_NONE;
	timer_delete(&bdev->ipc_timer);

	spin_lock_irqsave(&bdev->ipc_lock, flags);
	state = bdev->ipc_state;
	bdev->remote_status = remote;
	if (state == 0 && remote == 0 && bdev->firmware_active) {
		control = 1;
	} else if (state == 0 && remote == 1) {
		bdev->ipc_state = 1;
		publish = true;
		control = 2;
	} else if (state == 1 && remote == 2) {
		bdev->ipc_state = 2;
		wake = true;
	} else if (state == 2 && remote == 4) {
		bdev->ipc_state = 4;
		bdev->ipc_error = -EIO;
		wake = true;
	} else if (state == 4 && remote == 3) {
		bdev->ipc_state = 3;
	} else if ((state == 3 || state == 4) && remote == 1) {
		bdev->ipc_state = 1;
		control = 2;
	}
	spin_unlock_irqrestore(&bdev->ipc_lock, flags);

	if (publish) {
		err = btmtk_pcie_publish_context(bdev);
		if (err) {
			spin_lock_irqsave(&bdev->ipc_lock, flags);
			bdev->ipc_state = 4;
			bdev->ipc_error = err;
			spin_unlock_irqrestore(&bdev->ipc_lock, flags);
			wake = true;
			control = 0;
		}
	}
	if (control) {
		btmtk_pcie_set_host_control(bdev, control);
		/* 25G83 arms its 1 s IPC-status watchdog only after the
		 * remote INIT has caused context publication and host RUNNING.
		 * Starting it at the earlier host-INIT notification shortens the
		 * firmware's cold-start allowance and is not Apple-equivalent.
		 */
		if (control == 2)
			btmtk_pcie_arm_ipc_timer(bdev);
	}
	if (wake)
		wake_up_all(&bdev->ipc_wait);
	if (READ_ONCE(bdev->rings_prepared)) {
		err = btmtk_pcie_process_rings(bdev);
		if (err) {
			if (READ_ONCE(bdev->hdev))
				bdev->hdev->stat.err_rx++;
			dev_err_ratelimited(dev,
					    "MT793b invalid RX ring state: %d\n",
					    err);
		}
	}

	return IRQ_HANDLED;
}

static void btmtk_pcie_quiesce(struct btmtk_pcie_dev *bdev)
{
	WRITE_ONCE(bdev->stopping, true);
	timer_shutdown_sync(&bdev->ipc_timer);
	if (bdev->irq_registered)
		devm_free_irq(&bdev->pdev->dev, bdev->pdev->irq, bdev);
	bdev->irq_registered = false;
	if (bdev->bus_master_enabled) {
		pci_clear_master(bdev->pdev);
		bdev->bus_master_enabled = false;
	}
	btmtk_pcie_free_firmware_image(bdev);
}

static int btmtk_pcie_start_ipc(struct btmtk_pcie_dev *bdev)
{
	long timeout;
	int err;

	if (!bdev->firmware_active || !bdev->ipc)
		return -EINVAL;
	if (!btmtk_pcie_bar0_has(bdev, MTK_PCIE_BAR0_WINDOW_SPAN))
		return -ENXIO;

	bdev->ipc_state = 0;
	bdev->remote_status = 0xff;
	bdev->ipc_error = 0;
	bdev->context_published = false;
	bdev->stopping = false;
	err = devm_request_irq(&bdev->pdev->dev, bdev->pdev->irq,
			       btmtk_pcie_irq, IRQF_SHARED, KBUILD_MODNAME,
			       bdev);
	if (err)
		return err;
	bdev->irq_registered = true;

	btmtk_pcie_irq(bdev->pdev->irq, bdev);
	timeout = wait_event_timeout(bdev->ipc_wait,
				     READ_ONCE(bdev->ipc_state) == 2 ||
				     READ_ONCE(bdev->ipc_error),
				     msecs_to_jiffies(10000));
	if (!timeout || READ_ONCE(bdev->ipc_state) != 2) {
		err = READ_ONCE(bdev->ipc_error);
		if (!err)
			err = -ETIMEDOUT;
		btmtk_pcie_quiesce(bdev);
		return err;
	}

	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_IPC_STATE2_PASS: context_published=1 ipc_state=2 irq=1 irq_ack_register=none rings_advertised=0 rx_primed=0 upper_ready=0 hci=0\n");

	return 0;
}

static u8 btmtk_pcie_ring_channel(unsigned int ring)
{
	switch (ring) {
	case 1:
	case 2:
		return 0x43;
	case 3:
	case 4:
		return 0xff;
	case 5:
	case 6:
	case 8:
		return 0x81;
	case 9:
	case 10:
		return 0xc2;
	default:
		return 0;
	}
}

static void btmtk_pcie_fill_ring_record(struct btmtk_pcie_dev *bdev,
					u8 *record, unsigned int ring,
					u16 sequence)
{
	const struct btmtk_pcie_ring_layout *layout =
		&btmtk_pcie_ring_layout[ring];

	memset(record, 0, MTK_PCIE_IPC_CONTROL_RECORD_SIZE);
	/* Apple submits ring 7 as an entirely zero reserved record. */
	if (!layout->valid)
		return;

	put_unaligned_le16(0x34, record + 0x01);
	put_unaligned_le16(sequence, record + 0x0c);
	record[0x10] = 1;
	record[0x12] = btmtk_pcie_ring_channel(ring);
	if (ring == 5 || ring == 6 || ring == 8)
		record[0x13] = 1;
	put_unaligned_le32((ring << 16) | ring, record + 0x14);
	put_unaligned_le64(bdev->ipc->ring[ring].dma, record + 0x18);
	put_unaligned_le16(MTK_PCIE_IPC_RING_DEPTH, record + 0x28);
	put_unaligned_le16(layout->doorbell_bit, record + 0x2c);
	put_unaligned_le16(0x59, record + 0x2e);
	put_unaligned_le32(4, record + 0x30);
	put_unaligned_le16(layout->doorbell_bit, record + 0x34);
	put_unaligned_le16(1000, record + 0x3c);
}

static int btmtk_pcie_advertise_rings(struct btmtk_pcie_dev *bdev)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	u8 *pool = ipc->control_pool.vaddr;
	__le16 *tr_hia = ipc->tr_hia.vaddr;
	__le16 *tr_tia = ipc->tr_tia.vaddr;
	unsigned int ring;
	u16 sequence;
	u8 *record;

	for (ring = 1; ring < MTK_PCIE_IPC_RING_COUNT; ring++) {
		sequence = le16_to_cpu(READ_ONCE(tr_hia[0])) & 0x7f;
		record = pool + sequence * MTK_PCIE_IPC_CONTROL_RECORD_SIZE;
		btmtk_pcie_fill_ring_record(bdev, record, ring, sequence);

		/* Type 1 ring advertisement resets all three transport indices. */
		WRITE_ONCE(tr_hia[ring], cpu_to_le16(0));
		WRITE_ONCE(tr_tia[ring], cpu_to_le16(0));
		ipc->tr_tia_previous[ring] = 0;
		WRITE_ONCE(tr_hia[0], cpu_to_le16((sequence + 1) & 0x7f));

		dma_wmb();
		writel(BIT(MTK_PCIE_IPC_CONTROL_DOORBELL_BIT),
		       bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
	}

	return 0;
}

static int btmtk_pcie_prime_rx_ring(struct btmtk_pcie_dev *bdev,
				    unsigned int ring)
{
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	const struct btmtk_pcie_ring_layout *layout =
		&btmtk_pcie_ring_layout[ring];
	__le16 *tr_hia = ipc->tr_hia.vaddr;
	u8 *ring_memory = ipc->ring[ring].vaddr;
	u16 hia = le16_to_cpu(READ_ONCE(tr_hia[ring])) & 0x7f;
	unsigned int offered;

	if (!layout->valid || layout->tx || ring == 0 || !layout->stride)
		return -EINVAL;

	for (offered = 0; offered < MTK_PCIE_IPC_RING_DEPTH - 1; offered++) {
		u8 *descriptor = ring_memory + hia * layout->stride;

		memset(descriptor, 0, layout->stride);
		put_unaligned_le16(hia, descriptor + MTK_PCIE_IPC_DESCRIPTOR_TAG);
		descriptor[MTK_PCIE_IPC_DESCRIPTOR_STATE] = 0;
		hia = (hia + 1) & 0x7f;
		WRITE_ONCE(tr_hia[ring], cpu_to_le16(hia));
	}

	dma_wmb();
	writel(BIT(layout->doorbell_bit),
	       bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);

	return 0;
}

static int btmtk_pcie_prepare_rings(struct btmtk_pcie_dev *bdev)
{
	static const u8 rx_rings[] = { 2, 4, 6, 8, 10 };
	unsigned int i;
	int err;

	if (!bdev->ipc || !bdev->context_published || !bdev->irq_registered ||
	    READ_ONCE(bdev->ipc_state) != 2)
		return -EINVAL;

	err = btmtk_pcie_advertise_rings(bdev);
	if (err)
		return err;
	for (i = 0; i < ARRAY_SIZE(rx_rings); i++) {
		err = btmtk_pcie_prime_rx_ring(bdev, rx_rings[i]);
		if (err)
			return err;
	}

	bdev->rings_prepared = true;
	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_IPC_RINGS_PASS: records=10 reserved_ring7=1 control_doorbell=20 rx_rings=2/4/6/8/10 rx_offers_each=127 rings_advertised=1 rx_primed=1 init_commands=0 upper_ready=0 hci=0\n");

	return 0;
}

static int btmtk_pcie_hci_tx_layout(struct sk_buff *skb,
				    unsigned int *ring,
				    unsigned int *length)
{
	unsigned int expected;

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		if (skb->len < HCI_COMMAND_HDR_SIZE)
			return -EMSGSIZE;
		*ring = 1;
		expected = HCI_COMMAND_HDR_SIZE + skb->data[2];
		break;
	case HCI_ACLDATA_PKT:
		if (skb->len < HCI_ACL_HDR_SIZE)
			return -EMSGSIZE;
		*ring = 5;
		expected = HCI_ACL_HDR_SIZE + get_unaligned_le16(skb->data + 2);
		break;
	case HCI_SCODATA_PKT:
		if (skb->len < HCI_SCO_HDR_SIZE)
			return -EMSGSIZE;
		*ring = 3;
		expected = HCI_SCO_HDR_SIZE + skb->data[2];
		break;
	case HCI_ISODATA_PKT:
		if (skb->len < HCI_ISO_HDR_SIZE)
			return -EMSGSIZE;
		*ring = 9;
		expected = HCI_ISO_HDR_SIZE +
			   (get_unaligned_le16(skb->data + 2) & 0x3fff);
		break;
	default:
		return -EILSEQ;
	}
	if (expected != skb->len ||
	    expected > btmtk_pcie_ring_layout[*ring].max_payload)
		return -EMSGSIZE;

	*length = expected;
	return 0;
}

static int btmtk_pcie_send_srs_payload(struct hci_dev *hdev, u8 type,
				       const u8 *data, size_t size)
{
	u8 params[MTK_PCIE_SRS_PREFIX_SIZE + MTK_PCIE_SRS_CHUNK_SIZE];
	struct sk_buff *skb;
	size_t chunk;

	if (!data || !size || size > 0xffff || type > 1)
		return -EINVAL;

	while (size) {
		chunk = min_t(size_t, size, MTK_PCIE_SRS_CHUNK_SIZE);
		params[0] = 0x01;
		params[1] = 0x04;
		params[2] = type ? 0x30 : 0x20;
		params[3] = size > chunk;
		memcpy(params + MTK_PCIE_SRS_PREFIX_SIZE, data, chunk);

		skb = __hci_cmd_sync(hdev, MTK_PCIE_SRS_OPCODE,
				     MTK_PCIE_SRS_PREFIX_SIZE + chunk, params,
				     HCI_INIT_TIMEOUT);
		if (IS_ERR(skb))
			return PTR_ERR(skb);
		if (!skb->len || skb->data[0]) {
			kfree_skb(skb);
			return -EIO;
		}
		kfree_skb(skb);
		data += chunk;
		size -= chunk;
	}

	return 0;
}

static int btmtk_pcie_validate_ptx(const struct firmware *ptx)
{
	static const u8 directory[0x60] = {
		0x42, 0x4c, 0x4f, 0x42, 0x60, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
		0x0e, 0x00, 0x00, 0x00, 0x3d, 0x01, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00,
		0x6e, 0x00, 0x00, 0x00, 0x2c, 0x00, 0x00, 0x00,
		0x48, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x03, 0x00, 0x00, 0x9a, 0x00, 0x00, 0x00,
		0x2c, 0x00, 0x00, 0x00, 0x01, 0x09, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x00,
		0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	if (ptx->size != 198 ||
	    memcmp(ptx->data, directory, sizeof(directory)))
		return -EINVAL;

	return 0;
}

static int btmtk_pcie_send_bdaddr(struct hci_dev *hdev,
				  const struct firmware *address)
{
	struct sk_buff *skb;
	u8 params[6];
	unsigned int i;

	if (address->size != sizeof(params) ||
	    !memcmp(address->data, BDADDR_ANY, sizeof(params)) ||
	    !memcmp(address->data, BDADDR_NONE, sizeof(params)))
		return -EINVAL;

	/* 25G83 _SRS_WriteBDAddr reverses the IODeviceTree byte order. */
	for (i = 0; i < sizeof(params); i++)
		params[i] = address->data[sizeof(params) - i - 1];

	skb = __hci_cmd_sync(hdev, MTK_PCIE_SET_BDADDR_OPCODE,
			     sizeof(params), params, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);
	if (!skb->len || skb->data[0]) {
		kfree_skb(skb);
		return -EIO;
	}
	kfree_skb(skb);
	return 0;
}

static int btmtk_pcie_hci_setup(struct hci_dev *hdev)
{
	struct btmtk_pcie_dev *bdev = hci_get_drvdata(hdev);
	const struct firmware *address;
	const struct firmware *calibration;
	const struct firmware *ptx;
	int err;

	if (!load_radio_config)
		return 0;
	if (bdev->pdev->device != MTK_PCIE_DEVICE_MT7932_BT)
		return -ENODEV;

	err = request_firmware(&address, MTK_PCIE_BT_ADDRESS, &bdev->pdev->dev);
	if (err)
		return err;
	if (address->size != 6) {
		err = -EINVAL;
		goto release_address;
	}

	err = request_firmware(&calibration, MTK_PCIE_BT_CALIBRATION,
			       &bdev->pdev->dev);
	if (err)
		goto release_address;
	if (!calibration->size || calibration->size > 0xffff) {
		err = -EINVAL;
		goto release_calibration;
	}

	err = request_firmware(&ptx, MTK_PCIE_PTX, &bdev->pdev->dev);
	if (err)
		goto release_calibration;
	err = btmtk_pcie_validate_ptx(ptx);
	if (err)
		goto release_ptx;

	/* Apple 25G83 loads type-1 board calibration before type-0 PTX. */
	err = btmtk_pcie_send_srs_payload(hdev, 1, calibration->data,
					  calibration->size);
	if (err)
		goto release_ptx;
	err = btmtk_pcie_send_srs_payload(hdev, 0, ptx->data, ptx->size);
	if (err)
		goto release_ptx;
	err = btmtk_pcie_send_bdaddr(hdev, address);
	if (err)
		goto release_ptx;

	bdev->radio_config_loaded = true;
	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_RADIO_CONFIG_PASS: srs_opcode=fdd0 bdaddr_opcode=fc1a order=btcal-then-ptx-then-bdaddr btcal_bytes=%zu ptx_bytes=%zu bdaddr_bytes=6 max_chunk=240 ptx_prefix=01042000 hci=1 radio_unvalidated=1\n",
		 calibration->size, ptx->size);

release_ptx:
	release_firmware(ptx);
release_calibration:
	release_firmware(calibration);
release_address:
	release_firmware(address);
	return err;
}

static int btmtk_pcie_hci_open(struct hci_dev *hdev)
{
	return 0;
}

static int btmtk_pcie_hci_close(struct hci_dev *hdev)
{
	return 0;
}

static int btmtk_pcie_hci_send_frame(struct hci_dev *hdev,
				     struct sk_buff *skb)
{
	struct btmtk_pcie_dev *bdev = hci_get_drvdata(hdev);
	struct btmtk_pcie_ipc *ipc = bdev->ipc;
	const struct btmtk_pcie_ring_layout *layout;
	__le16 *tr_hia = ipc->tr_hia.vaddr;
	__le16 *tr_tia = ipc->tr_tia.vaddr;
	unsigned long flags;
	unsigned int ring, length;
	u16 hia, tia, next;
	u8 *descriptor;
	int err;

	if (READ_ONCE(bdev->stopping) || READ_ONCE(bdev->ipc_state) != 2 ||
	    !READ_ONCE(bdev->rings_prepared))
		return -ENODEV;
	err = btmtk_pcie_hci_tx_layout(skb, &ring, &length);
	if (err)
		return err;
	layout = &btmtk_pcie_ring_layout[ring];

	dma_rmb();
	spin_lock_irqsave(&bdev->ring_lock, flags);
	hia = le16_to_cpu(READ_ONCE(tr_hia[ring])) & 0x7f;
	tia = le16_to_cpu(READ_ONCE(tr_tia[ring])) & 0x7f;
	next = (hia + 1) & 0x7f;
	if (next == tia) {
		err = -ENOSPC;
		goto out_unlock;
	}

	descriptor = ipc->ring[ring].vaddr + hia * layout->stride;
	memset(descriptor, 0, layout->stride);
	descriptor[0] = 2;
	descriptor[1] = length;
	descriptor[2] = length >> 8;
	descriptor[3] = length >> 16;
	put_unaligned_le16(hia, descriptor + MTK_PCIE_IPC_DESCRIPTOR_TAG);
	descriptor[MTK_PCIE_IPC_DESCRIPTOR_STATE] = 2;
	memcpy(descriptor + 0x10, skb->data, length);
	WRITE_ONCE(tr_hia[ring], cpu_to_le16(next));
	dma_wmb();
	writel(BIT(layout->doorbell_bit),
	       bdev->bar0 + MTK_PCIE_BAR0_DOORBELL);
	err = 0;

out_unlock:
	spin_unlock_irqrestore(&bdev->ring_lock, flags);
	if (err) {
		hdev->stat.err_tx++;
		return err;
	}

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	default:
		break;
	}
	hdev->stat.byte_tx += length;
	kfree_skb(skb);
	return 0;
}

static void btmtk_pcie_unregister_hci(struct btmtk_pcie_dev *bdev)
{
	struct hci_dev *hdev = bdev->hdev;

	WRITE_ONCE(bdev->hdev, NULL);
	if (!hdev)
		return;
	if (bdev->hci_registered)
		hci_unregister_dev(hdev);
	bdev->hci_registered = false;
	hci_free_dev(hdev);
}

static int btmtk_pcie_register_hci(struct btmtk_pcie_dev *bdev)
{
	struct hci_dev *hdev;
	int err;

	if (!bdev->rings_prepared || READ_ONCE(bdev->ipc_state) != 2)
		return -EINVAL;
	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_PCI;
	hdev->open = btmtk_pcie_hci_open;
	hdev->close = btmtk_pcie_hci_close;
	hdev->send = btmtk_pcie_hci_send_frame;
	hdev->setup = btmtk_pcie_hci_setup;
	hci_set_drvdata(hdev, bdev);
	SET_HCIDEV_DEV(hdev, &bdev->pdev->dev);
	WRITE_ONCE(bdev->hdev, hdev);

	err = hci_register_dev(hdev);
	if (err) {
		WRITE_ONCE(bdev->hdev, NULL);
		hci_free_dev(hdev);
		return err;
	}
	bdev->hci_registered = true;

	err = btmtk_pcie_process_rings(bdev);
	if (err) {
		btmtk_pcie_unregister_hci(bdev);
		return err;
	}
	dev_info(&bdev->pdev->dev,
		 "J700_MT793B_HCI_REGISTER_PASS: bus=PCI tx_rings=cmd1/sco3/acl5/iso9 rx_rings=event2/sco4/acl6/iso10 fwlog8 cr0=ack-only rings=1 rx_refill=1 hci=1 radio_unvalidated=1\n");

	return 0;
}

static int btmtk_pcie_read_chip_id(struct btmtk_pcie_dev *bdev, u32 *chip_id)
{
	resource_size_t bar0_len = pci_resource_len(bdev->pdev, 0);
	u32 remap_base = MTK_PCIE_CONN_CHIP_ID & 0xffff0000;

	if (bar0_len < MTK_PCIE_CHIP_ID_WINDOW + sizeof(*chip_id) ||
	    bar0_len < MTK_PCIE_BAR0_FALLBACK_SEL + sizeof(u32))
		return -ENXIO;

	writel(remap_base, bdev->bar0 + MTK_PCIE_BAR0_REMAP_SEL);
	writel(MTK_PCIE_DYNAMIC_REMAP_1890,
	       bdev->bar0 + MTK_PCIE_BAR0_FALLBACK_SEL);
	*chip_id = readl(bdev->bar0 + MTK_PCIE_CHIP_ID_WINDOW);

	return 0;
}

static int btmtk_pcie_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct btmtk_pcie_dev *bdev;
	void __iomem * const *iomap;
	u16 command, subsystem_vendor, subsystem_device;
	u8 revision;
	u32 chip_id, rom_config;
	u8 rom_version;
	int err;

	err = btmtk_pcie_apply_gate_selector();
	if (err) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=gate-selector gate=%u ret=%d\n",
			gate, err);
		return err;
	}

	err = pcim_enable_device(pdev);
	if (err)
		return err;

	err = pcim_iomap_regions(pdev, BIT(0) | BIT(2), KBUILD_MODNAME);
	if (err)
		return err;

	iomap = pcim_iomap_table(pdev);
	if (!iomap || !iomap[0] || !iomap[2])
		return -ENXIO;

	bdev = devm_kzalloc(&pdev->dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;

	bdev->pdev = pdev;
	bdev->bar0 = iomap[0];
	bdev->bar2 = iomap[2];
	bdev->fallback_chip_id = id->driver_data;
	spin_lock_init(&bdev->ipc_lock);
	spin_lock_init(&bdev->ring_lock);
	init_waitqueue_head(&bdev->ipc_wait);
	timer_setup(&bdev->ipc_timer, btmtk_pcie_ipc_timer, 0);
	pci_set_drvdata(pdev, bdev);

	pci_read_config_word(pdev, PCI_COMMAND, &command);
	pci_read_config_byte(pdev, PCI_REVISION_ID, &revision);
	pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &subsystem_vendor);
	pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &subsystem_device);

	dev_info(&pdev->dev,
		 "J700_MT793B_PCIE_INVENTORY_PASS: pci=%04x:%04x rev=%02x sub=%04x:%04x irq=%u command=%04x bar0=%pa/%pa bar2=%pa/%pa fallback_chip=%04x dma=off hci=off\n",
		 pdev->vendor, pdev->device, revision, subsystem_vendor,
		 subsystem_device, pdev->irq, command, &pdev->resource[0].start,
		 &pdev->resource[0].end, &pdev->resource[2].start,
		 &pdev->resource[2].end, bdev->fallback_chip_id);

	if (prepare_ipc && (!probe_rom_version || !probe_remap)) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=ipc-prerequisite require_rom=1 require_remap=1\n");
		return -EINVAL;
	}
	if (activate_firmware && !prepare_ipc) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=fwdl-prerequisite require_ipc=1\n");
		return -EINVAL;
	}
	if (start_ipc && !activate_firmware) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=ipc-start-prerequisite require_fwdl=1\n");
		return -EINVAL;
	}
	if (prepare_rings && !start_ipc) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=ring-prerequisite require_ipc_state2=1\n");
		return -EINVAL;
	}
	if (enable_hci && !prepare_rings) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=hci-prerequisite require_rings=1\n");
		return -EINVAL;
	}
	if (load_radio_config && !enable_hci) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=radio-config-prerequisite require_hci=1\n");
		return -EINVAL;
	}

	if (probe_rom_version) {
		err = pci_read_config_dword(pdev, MTK_PCIE_ROM_VERSION_CONFIG,
					   &rom_config);
		if (err != PCIBIOS_SUCCESSFUL) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=rom-config-48c ret=%d\n",
				err);
			return -EIO;
		}

		rom_version = (rom_config >> 4) & 0x7;
		if (rom_version > 1) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=rom-version raw=%08x bits6to4=%u\n",
				rom_config, rom_version);
			return -ENODEV;
		}
		bdev->rom_version = rom_version;
		bdev->rom_version_valid = true;

		dev_info(&pdev->dev,
			 "J700_MT793B_ROM_VERSION_PASS: config_offset=48c raw=%08x bits6to4=%u firmware=MT7932B%u dma=0 hci=0\n",
			 rom_config, rom_version, rom_version);
	}

	if (!probe_remap) {
		dev_info(&pdev->dev,
			 "J700_MT793B_PCIE_GATE_READY: remap_write=0 mmio_reads=0 rom_config_read=%u dma=0 hci=0\n",
			 probe_rom_version);
		return 0;
	}

	err = btmtk_pcie_read_chip_id(bdev, &chip_id);
	if (err) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=chip-id ret=%d\n",
			err);
		return err;
	}

	if (!chip_id) {
		dev_info(&pdev->dev,
			 "J700_MT793B_CHIP_ID_DEFERRED: register=70010200 value=00000000 source=pre-conninfra exact_remap=23014/fe3b8 dma=0 hci=0\n");
	} else if ((chip_id & 0xffff) != 0x7922 &&
	    (chip_id & 0xffff) != 0x7923 &&
	    (chip_id & 0xffff) != 0x7932) {
		dev_err(&pdev->dev,
			"J700_MT793B_PCIE_GATE_FAIL: stage=chip-id value=%08x\n",
			chip_id);
		return -ENODEV;
	} else {
		dev_info(&pdev->dev,
			 "J700_MT793B_CHIP_ID_PASS: register=70010200 value=%08x core=%04x remap=70010000 bar0_selector=23014 fallback_selector=fe3b8 fallback=188d1890 bar0_window=c0200 dma=0 hci=0\n",
			 chip_id, chip_id & 0xffff);
	}

	if (prepare_ipc) {
		err = btmtk_pcie_prepare_ipc(bdev);
		if (err) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=ipc-layout ret=%d\n",
				err);
			return err;
		}
	}
	if (activate_firmware) {
		err = btmtk_pcie_activate_firmware(bdev);
		if (err) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=firmware-activation ret=%d\n",
				err);
			return err;
		}
	}
	if (start_ipc) {
		err = btmtk_pcie_start_ipc(bdev);
		if (err) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=ipc-start ret=%d\n",
				err);
			return err;
		}
	}
	if (prepare_rings) {
		err = btmtk_pcie_prepare_rings(bdev);
		if (err) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=ring-prepare ret=%d\n",
				err);
			btmtk_pcie_quiesce(bdev);
			return err;
		}
	}
	if (enable_hci) {
		err = btmtk_pcie_register_hci(bdev);
		if (err) {
			dev_err(&pdev->dev,
				"J700_MT793B_PCIE_GATE_FAIL: stage=hci-register ret=%d\n",
				err);
			btmtk_pcie_quiesce(bdev);
			return err;
		}
	}

	return 0;
}

static void btmtk_pcie_remove(struct pci_dev *pdev)
{
	struct btmtk_pcie_dev *bdev = pci_get_drvdata(pdev);

	if (bdev) {
		btmtk_pcie_quiesce(bdev);
		btmtk_pcie_unregister_hci(bdev);
		bdev->firmware_active = false;
	}
}

static void btmtk_pcie_shutdown(struct pci_dev *pdev)
{
	btmtk_pcie_remove(pdev);
}

static const struct pci_device_id btmtk_pcie_table[] = {
	{ PCI_DEVICE(MTK_PCIE_VENDOR_ID, MTK_PCIE_DEVICE_MT7922_BT),
	  .driver_data = 0x7922 },
	{ PCI_DEVICE(MTK_PCIE_VENDOR_ID, MTK_PCIE_DEVICE_MT7923_BT),
	  .driver_data = 0x7923 },
	{ PCI_DEVICE(MTK_PCIE_VENDOR_ID, MTK_PCIE_DEVICE_MT7932_BT),
	  .driver_data = 0x7932 },
	{ }
};
MODULE_DEVICE_TABLE(pci, btmtk_pcie_table);

static struct pci_driver btmtk_pcie_driver = {
	.name = KBUILD_MODNAME,
	.id_table = btmtk_pcie_table,
	.probe = btmtk_pcie_probe,
	.remove = btmtk_pcie_remove,
	.shutdown = btmtk_pcie_shutdown,
};
module_pci_driver(btmtk_pcie_driver);

MODULE_AUTHOR("Aurora Silicon");
MODULE_DESCRIPTION("MediaTek MT79xx Bluetooth PCIe transport");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_FIRMWARE(MTK_PCIE_FW_B0);
MODULE_FIRMWARE(MTK_PCIE_FW_B1);
MODULE_FIRMWARE(MTK_PCIE_PTX);
MODULE_FIRMWARE(MTK_PCIE_BT_CALIBRATION);
MODULE_FIRMWARE(MTK_PCIE_BT_ADDRESS);
