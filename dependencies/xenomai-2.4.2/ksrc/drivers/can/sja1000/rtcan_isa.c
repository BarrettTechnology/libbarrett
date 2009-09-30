/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Copyright (C) 2005, 2006 Sebastian Smolorz
 *                          <Sebastian.Smolorz@stud.uni-hannover.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; eitherer version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#include <rtdm/rtdm_driver.h>

#include <rtdm/rtcan.h>
#include <rtcan_dev.h>
#include <rtcan_raw.h>
#include <rtcan_internal.h>
#include <rtcan_sja1000.h>
#include <rtcan_sja1000_regs.h>

#define RTCAN_DEV_NAME    "rtcan%d"
#define RTCAN_DRV_NAME    "sja1000-isa"

#define RTCAN_ISA_MAX_DEV 4

static char *isa_board_name = "ISA-Board";

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("RTCAN board driver for standard ISA boards");
MODULE_SUPPORTED_DEVICE("ISA board");
MODULE_LICENSE("GPL");

static u16 io[RTCAN_ISA_MAX_DEV];
static int irq[RTCAN_ISA_MAX_DEV];
static u32 clock[RTCAN_ISA_MAX_DEV];
static u8 ocr[RTCAN_ISA_MAX_DEV];
static u8 cdr[RTCAN_ISA_MAX_DEV];

compat_module_param_array(io, ushort, RTCAN_ISA_MAX_DEV, 0444);
compat_module_param_array(irq, int, RTCAN_ISA_MAX_DEV, 0444);
compat_module_param_array(clock, uint, RTCAN_ISA_MAX_DEV, 0444);
compat_module_param_array(ocr, byte, RTCAN_ISA_MAX_DEV, 0444);
compat_module_param_array(cdr, byte, RTCAN_ISA_MAX_DEV, 0444);

MODULE_PARM_DESC(io, "The io-port address");
MODULE_PARM_DESC(irq, "The interrupt number");
MODULE_PARM_DESC(clock, "External clock frequency (default 16 MHz)");
MODULE_PARM_DESC(ocr, "Value of output control register (default 0x1a)");
MODULE_PARM_DESC(cdr, "Value of clock divider register (default 0xc8");

#define RTCAN_ISA_PORT_SIZE 32

struct rtcan_isa
{
	u16 io;
};

static struct rtcan_device *rtcan_isa_devs[RTCAN_ISA_MAX_DEV];

static u8 rtcan_isa_readreg(struct rtcan_device *dev, int port)
{
	struct rtcan_isa *board = (struct rtcan_isa *)dev->board_priv;
	return inb(board->io + port);
}

static void rtcan_isa_writereg(struct rtcan_device *dev, int port, u8 val)
{
	struct rtcan_isa *board = (struct rtcan_isa *)dev->board_priv;
	outb(val, board->io + port);
}


int __init rtcan_isa_init_one(int idx)
{
	struct rtcan_device *dev;
	struct rtcan_sja1000 *chip;
	struct rtcan_isa *board;
	int ret;

	if ((dev = rtcan_dev_alloc(sizeof(struct rtcan_sja1000),
				   sizeof(struct rtcan_isa))) == NULL)
		return -ENOMEM;

	chip = (struct rtcan_sja1000 *)dev->priv;
	board = (struct rtcan_isa *)dev->board_priv;

	dev->board_name = isa_board_name;

	board->io = io[idx];

	chip->irq_num = irq[idx];
	chip->irq_flags = RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE;

	chip->read_reg = rtcan_isa_readreg;
	chip->write_reg = rtcan_isa_writereg;

	/* Check and request I/O ports */
	if (!request_region(board->io, RTCAN_ISA_PORT_SIZE, RTCAN_DRV_NAME)) {
		ret = -EBUSY;
		goto out_dev_free;
	}

	/* Clock frequency in Hz */
	if (clock[idx])
		dev->can_sys_clock = clock[idx] / 2;
	else
		dev->can_sys_clock = 8000000; /* 16/2 MHz */

	/* Output control register */
	if (ocr[idx])
		chip->ocr = ocr[idx];
	else
		chip->ocr = SJA_OCR_MODE_NORMAL | SJA_OCR_TX0_PUSHPULL;

	if (cdr[idx])
		chip->cdr = cdr[idx];
	else
		chip->cdr = SJA_CDR_CAN_MODE | SJA_CDR_CLK_OFF | SJA_CDR_CBP;

	strncpy(dev->name, RTCAN_DEV_NAME, IFNAMSIZ);

	ret = rtcan_sja1000_register(dev);
	if (ret) {
		printk(KERN_ERR "ERROR %d while trying to register SJA1000 "
		       "device!\n", ret);
		goto out_free_region;
	}

	rtcan_isa_devs[idx] = dev;
	return 0;

 out_free_region:
	release_region(board->io, RTCAN_ISA_PORT_SIZE);

 out_dev_free:
	rtcan_dev_free(dev);

	return ret;
}

static void rtcan_isa_exit(void);

/** Init module */
static int __init rtcan_isa_init(void)
{
	int i, err;
	int devices = 0;

	for (i = 0; i < RTCAN_ISA_MAX_DEV && io[i] != 0; i++) {
		err = rtcan_isa_init_one(i);
		if (err) {
			rtcan_isa_exit();
			return err;
		}
		devices++;
	}
	if (devices)
		return 0;

	printk(KERN_ERR "ERROR! No devices specified! "
	       "Use io=<port1>[,...] irq=<irq1>[,...]\n");
	return -EINVAL;
}


/** Cleanup module */
static void rtcan_isa_exit(void)
{
	int i;
	struct rtcan_device *dev;
	
	for (i = 0; i < RTCAN_ISA_MAX_DEV; i++) {
		dev = rtcan_isa_devs[i];
		if (!dev)
			continue;
		rtcan_sja1000_unregister(dev);
		rtcan_dev_free(dev);
	}
}

module_init(rtcan_isa_init);
module_exit(rtcan_isa_exit);
