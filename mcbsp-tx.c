/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (C) 2011 Tamas Bondar <dev@tamasbondar.info>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <asm/uaccess.h>
#include <plat/dma.h>
#include <plat/mcbsp.h>
#include <linux/dma-mapping.h>

//#include "mcbsp-tx.h"

#define DEVICE_NAME "mcbsp3-tx"
#define OMAP_MCBSP3 2

#define DEFAULT_DMA_QUEUE_THRESHOLD 10

/* Read a McBSP register
   (input type: struct omap_mcbsp *mcbsp) */
#define MCBSP_READ(mcbsp, reg) __raw_readl(mcbsp->io_base + OMAP_MCBSP_REG_##reg)

struct mcbsptx {
    dev_t devt;
    struct device *dev;
    struct cdev cdev;
    struct class *class;

    short dma_channel;
    short dma_current_idx;
    int dma_queue_threshold;

    int element_count;
    int frame_count;
};

static struct mcbsptx mcbsptx;

#define DEFAULT_CLKDIV 83

#define CLKGDV_MASK (0xff)
#define WDLEN_MASK  (0x07)
#define FRLEN_MASK  (0x7f)
#define FWID_MASK   (0xff)
#define FPER_MASK   (0x0fff)

static struct omap_mcbsp_reg_cfg mcbsp_config = {
    .spcr2 = XINTM(3),
    .spcr1 = 0,
    .xcr2  = 0,
    .xcr1  = XFRLEN1(0) | XWDLEN1(OMAP_MCBSP_WORD_32),
    .srgr1 = FWID(31) | CLKGDV(DEFAULT_CLKDIV),
    .srgr2 = CLKSM | FPER(33),
    .mcr2 = 0,
    .mcr1 = 0,
    .pcr0  = FSXM | CLKXM | FSXP,
    .xccr = XDMAEN | XDISABLE,
    .rccr = 0,
};

#define NUMBER_OF_BLOCKS 4
#define MCBSPTX_BLOCK_SIZE (128*4)
#define MCBSPTX_BUFFER_SIZE (MCBSPTX_BLOCK_SIZE * NUMBER_OF_BLOCKS)


typedef struct {
    struct list_head lh;
    u8 *buffer_addr;
    dma_addr_t dma_addr;
} mcbsptx_block_t;

/* Buffer is a contiguous area to store data to be written off 
   to the device */
static u8 *buffer;

/* Buffer is split to consecutive blocks that serves as the unit
   of DMA transfer */
static mcbsptx_block_t block[NUMBER_OF_BLOCKS];

/* Blocks are arranged in various lists */
struct list_head free_blocks;   /* free blocks */
struct list_head dma_blocks;    /* awaiting DMA transfer */

/* Block currently under DMA transaction */
static mcbsptx_block_t *dma_block;

/* Device Control */
static ssize_t attr_word_length_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int val;

    switch(mcbsp_config.xcr1 & XWDLEN1(WDLEN_MASK))
    {
    case XWDLEN1(OMAP_MCBSP_WORD_8):
        val = 8;
        break;
    case XWDLEN1(OMAP_MCBSP_WORD_12):
        val = 12;
        break;
    case XWDLEN1(OMAP_MCBSP_WORD_16):
        val = 16;
        break;
    case XWDLEN1(OMAP_MCBSP_WORD_20):
        val = 20;
        break;
    case XWDLEN1(OMAP_MCBSP_WORD_24):
        val = 24;
        break;
    case XWDLEN1(OMAP_MCBSP_WORD_32):
        val = 32;
        break;
    default:
        val = 0;
    }

    return sprintf(buf, "%u\n", val);
}

static ssize_t attr_word_length_store(struct device *dev, struct device_attribute *attr,
                                      const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *endp;
    unsigned int val2;
    unsigned long val = simple_strtoul(buf, &endp, 10);
    size_t count = endp - buf;

    if (isspace(*endp))
            count++;

    if (count != size)
        goto exit;

    switch(val) {
    case 8:
        val2 = OMAP_MCBSP_WORD_8;
        break;
    case 12:
        val2 = OMAP_MCBSP_WORD_12;
        break;
    case 16:
        val2 = OMAP_MCBSP_WORD_16;
        break;
    case 20:
        val2 = OMAP_MCBSP_WORD_20;
        break;
    case 24:
        val2 = OMAP_MCBSP_WORD_24;
        break;
    case 32:
        val2 = OMAP_MCBSP_WORD_32;
        break;
    default:
        goto exit;
    }
    mcbsp_config.xcr1  &= ~XWDLEN1(WDLEN_MASK);
    mcbsp_config.xcr1  |= XWDLEN1(val2);
    ret = count;

 exit:
    return ret;
}

static ssize_t attr_frame_length_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int val;

    val = mcbsp_config.xcr1 & XFRLEN1(FRLEN_MASK); /* mask bitfield */
    val /= XFRLEN1(1); /* shift right to bit 0 */
    val += 1;

    return sprintf(buf, "%u\n", val);
}

static ssize_t attr_frame_length_store(struct device *dev, struct device_attribute *attr,
                                       const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *endp;
    unsigned long val = simple_strtoul(buf, &endp, 10);
    size_t count = endp - buf;

    if (isspace(*endp))
            count++;

    if (count == size && val > 0 && val <= FRLEN_MASK + 1) {
        mcbsp_config.xcr1  &= ~XFRLEN1(FRLEN_MASK);
        mcbsp_config.xcr1  |= XFRLEN1(val-1);
        ret = count;
    }
    return ret;
}

static ssize_t attr_frame_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int val;

    val = mcbsp_config.srgr1 & FWID(FWID_MASK); /* mask bitfield */
    val /= FWID(1); /* shift right to bit 0 */
    val += 1;

    return sprintf(buf, "%u\n", val);
}

static ssize_t attr_frame_width_store(struct device *dev, struct device_attribute *attr,
                                      const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *endp;
    unsigned long val = simple_strtoul(buf, &endp, 10);
    size_t count = endp - buf;

    if (isspace(*endp))
            count++;

    if (count == size && val > 0 && val <= FWID_MASK + 1) {
        mcbsp_config.srgr1  &= ~FWID(FWID_MASK);
        mcbsp_config.srgr1  |= FWID(val-1);
        ret = count;
    }
    return ret;
}

static ssize_t attr_frame_period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int val;

    val = mcbsp_config.srgr2 & FPER(FPER_MASK); /* mask bitfield */
    val /= FPER(1); /* shift right to bit 0 */
    val += 1;

    return sprintf(buf, "%u\n", val);
}

static ssize_t attr_frame_period_store(struct device *dev, struct device_attribute *attr,
                                       const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *endp;
    unsigned long val = simple_strtoul(buf, &endp, 10);
    size_t count = endp - buf;

    if (isspace(*endp))
            count++;

    if (count == size && val > 0 && val <= FPER_MASK + 1) {
        mcbsp_config.srgr2  &= ~FPER(FPER_MASK);
        mcbsp_config.srgr2  |= FPER(val-1);
        ret = count;
    }
    return ret;
}

static ssize_t attr_clock_divider_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int val;

    val = mcbsp_config.srgr1 & CLKGDV(CLKGDV_MASK); /* mask bitfield */
    val /= CLKGDV(1); /* shift right to bit 0 */

    return sprintf(buf, "%u\n", val);
}

static ssize_t attr_clock_divider_store(struct device *dev, struct device_attribute *attr,
                                        const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;
    char *endp;
    unsigned long val = simple_strtoul(buf, &endp, 10);
    size_t count = endp - buf;

    if (isspace(*endp))
            count++;

    if (count == size && val > 0 && val <= CLKGDV_MASK) {
        mcbsp_config.srgr1  &= ~CLKGDV(CLKGDV_MASK);
        mcbsp_config.srgr1  |= CLKGDV(val);
        ret = count;
    }
    return ret;
}

static ssize_t attr_framesync_polarity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    if (mcbsp_config.pcr0 & FSXP)
        return sprintf(buf, "low\n");
    else
        return sprintf(buf, "high\n");
}

static ssize_t attr_framesync_polarity_store(struct device *dev, struct device_attribute *attr,
                                             const char *buf, size_t size)
{
    ssize_t ret = -EINVAL;

    if (!strncmp(buf, "low", 3))
        mcbsp_config.pcr0  |= FSXP;
    else if (!strncmp(buf, "high", 4))
        mcbsp_config.pcr0  &= ~FSXP;

    return ret;
}

/* Control attributes */
static struct device_attribute control_attrs[] = {
    __ATTR(word_length, 0664, attr_word_length_show, attr_word_length_store),
    __ATTR(frame_length, 0664, attr_frame_length_show, attr_frame_length_store),
    __ATTR(frame_width, 0664, attr_frame_width_show, attr_frame_width_store),
    __ATTR(frame_period, 0664, attr_frame_period_show, attr_frame_period_store),
    __ATTR(clock_divider, 0664, attr_clock_divider_show, attr_clock_divider_store),
    __ATTR(framesync_polarity, 0664, attr_framesync_polarity_show, attr_framesync_polarity_store),
    __ATTR_NULL
};


static void mcbsptx_set_mcbsp_config(void)
{
    omap_mcbsp_set_tx_threshold(OMAP_MCBSP3, 3);
    omap_mcbsp_config(OMAP_MCBSP3, &mcbsp_config);
}


static void mcbsptx_kickstart_dma(void)
{
    struct list_head *head;

    if (dma_block)
    {
        printk(KERN_INFO "Kicking DMA failed: running\n");
        return;
    }

    if (list_empty(&dma_blocks))
    {
        printk(KERN_INFO "Kicking DMA failed: empty\n");
        return;
    }

    /* Get DMA block */
    head = dma_blocks.next;
    list_del(head);
    dma_block = list_entry(head, mcbsptx_block_t, lh);

    printk(KERN_INFO "Kicking DMA for block %p\n", dma_block->buffer_addr);

    omap_set_dma_src_params(mcbsptx.dma_channel,
                            0,
                            OMAP_DMA_AMODE_POST_INC,
                            dma_block->dma_addr,
                            0, 0);

    omap_start_dma(mcbsptx.dma_channel);
}


static void mcbsptx_dma_callback(int lch, u16 ch_status, void *data)
{
    printk(KERN_INFO "DMA callback\n");

    if (ch_status != 0x0020)
    {
        printk(KERN_ERR "ch_status = 0x%04X\n", ch_status);
        return;
    }

    if (!dma_block)
    {
        printk(KERN_ERR "Error\n");
        return;
    }

    /* Unmap block for DMA */
    dma_unmap_single(mcbsptx.dev, dma_block->dma_addr,
                     MCBSPTX_BLOCK_SIZE, DMA_TO_DEVICE);

    /* Insert block to free queue */
    list_add_tail(&dma_block->lh, &free_blocks);
    printk(KERN_INFO "Added block %p to free list\n", dma_block->buffer_addr);

    dma_block = NULL;

    /* Next DMA */
    mcbsptx_kickstart_dma();
}


static int mcbsptx_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    int dma_channel;
    int i;

    printk(KERN_INFO "McBSP-TX open\n");

    if (buffer)
    {
        printk(KERN_ERR "Buffer already allocated\n");
        ret = -ENOMEM;
        goto fail1;
    }

    /* Allocate data buffer */
    buffer = kmalloc(MCBSPTX_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
    if (!buffer) {
        printk(KERN_ERR "Buffer allocation failed\n");
        ret = -ENOMEM;
        goto fail1;
    }

    /* Initialise block lists */
    INIT_LIST_HEAD(&free_blocks);
    INIT_LIST_HEAD(&dma_blocks);

    /* Split buffer to blocks */
    for (i = 0; i < NUMBER_OF_BLOCKS; i++)
    {
        block[i].buffer_addr = buffer + i * MCBSPTX_BLOCK_SIZE;
        /* Add block to the free block list */
        list_add_tail(&block[i].lh, &free_blocks);
        printk(KERN_INFO "Added block %p to free list\n", block[i].buffer_addr);
    }

    /* Request DMA channel */
    if (mcbsptx.dma_channel == -1)
    {
        ret = omap_request_dma(OMAP24XX_DMA_MCBSP3_TX, "McBSP-TX",
                               mcbsptx_dma_callback, 0,
                               &dma_channel);
        if (ret) {
            printk(KERN_ERR "DMA channel request failed\n");
            goto fail2;
        }

        omap_set_dma_transfer_params(dma_channel,
                                     OMAP_DMA_DATA_TYPE_S32,
                                     mcbsptx.element_count, mcbsptx.frame_count,
                                     OMAP_DMA_SYNC_BLOCK, OMAP24XX_DMA_MCBSP3_TX, OMAP_DMA_DST_SYNC);

        omap_set_dma_dest_params(dma_channel, 0,
                                 OMAP_DMA_AMODE_CONSTANT,
                                 omap_mcbsp_dma_reg_params(OMAP_MCBSP3, 0),
                                 0, 0);

        omap_disable_dma_irq(dma_channel, OMAP_DMA_DROP_IRQ);
        mcbsptx.dma_channel = dma_channel;
    }

    mcbsptx_set_mcbsp_config();

    /* missing ecbsp_map_dma_block() */

    omap_mcbsp_start(OMAP_MCBSP3, 1 /* tx */, 0 /* rx */);

    /* missing ecbsp_kickstart_dma() */

    return 0;

 fail2:
    kfree(buffer);
    buffer = NULL;
 fail1:
    return ret;
}

static int mcbsptx_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "McBSP-TX release\n");

    omap_mcbsp_stop(OMAP_MCBSP3, 1 /* tx */, 0 /* rx */);

    if (mcbsptx.dma_channel != -1)
    {
        omap_stop_dma(mcbsptx.dma_channel);
        omap_free_dma(mcbsptx.dma_channel);
        mcbsptx.dma_channel = -1;
    }

    /* FIXME: make sure all DMA has finished */
    kfree(buffer);
    buffer = NULL;

    return 0;
}


static ssize_t mcbsptx_write(struct file *file,
                             const char __user *buff,
                             size_t count, loff_t *f_pos)
{
    ssize_t ret = 0;
    mcbsptx_block_t *block;
    size_t length;

    printk(KERN_INFO "McBSP-TX write (%d bytes)\n", count);

    while (count > 0 && !list_empty(&free_blocks))
    {
        /* Get free block */
        list_move_tail(free_blocks.next, &dma_blocks);
        block = list_entry(dma_blocks.prev, mcbsptx_block_t, lh);

        /* Number of bytes to copy */
        length = count < MCBSPTX_BLOCK_SIZE ? count : MCBSPTX_BLOCK_SIZE;

        printk(KERN_INFO "Filling block %p with %d bytes\n", block->buffer_addr, length);

        /* Fill block with user data */
        if (copy_from_user(block->buffer_addr, buff, length))
        {
            ret = -EFAULT;
            goto exit;
        }

        /* Map block for DMA */
        block->dma_addr = dma_map_single(mcbsptx.dev, block->buffer_addr,
                                         MCBSPTX_BLOCK_SIZE, DMA_TO_DEVICE);
        if (!block->dma_addr)
        {
            printk(KERN_ERR "DMA mapping failed\n");
            ret = -ENOMEM;
            goto exit;
        }

        mcbsptx_kickstart_dma();

        *f_pos += length;
        ret += length;
        count -= length;
    }

 exit:
    return ret;
}

static const struct file_operations mcbsptx_fops = {
    .owner   = THIS_MODULE,
    .open    = mcbsptx_open,	
    .release = mcbsptx_release,
    .write   = mcbsptx_write,
};

static int mcbsptx_mcbsp_request(void)
{
    int ret = 0;

    ret = omap_mcbsp_request(OMAP_MCBSP3);
    if (ret < 0) {
        printk(KERN_ERR "omap_mcbsp_request() failed\n");
        goto fail;
    }

    mcbsptx.element_count = 16;
    mcbsptx.frame_count = 1;
    //mcbsptx.state |= MCBSP_REQUESTED;
    //mcbsptx.state &= ~MCBSP_CONFIG_SET;

 fail:
    return ret;
}

static int __init mcbsptx_init(void)
{
    int err;

    printk(KERN_INFO "McBSP-TX init\n");

    memset(&mcbsptx, 0, sizeof(struct mcbsptx));

    mcbsptx.dma_channel = -1;
    mcbsptx.dma_current_idx = -1;
    mcbsptx.dma_queue_threshold = DEFAULT_DMA_QUEUE_THRESHOLD;

    /* Get device number */
    mcbsptx.devt = MKDEV(0, 0);
    err = alloc_chrdev_region(&mcbsptx.devt, 0, 1, DEVICE_NAME);
    if (err)
    {
        printk(KERN_ALERT "alloc_chrdev_region() failed (%d)\n", err);
        goto fail1;
    }

    /* Initialise character device */
    cdev_init(&mcbsptx.cdev, &mcbsptx_fops);
    mcbsptx.cdev.owner = THIS_MODULE;

    err = cdev_add(&mcbsptx.cdev, mcbsptx.devt, 1);
    if (err)
    {
        printk(KERN_ALERT "cdev_add() failed (%d)\n", err);
        goto fail2;
    }

    /* Populate sysfs entries */
    mcbsptx.class = class_create(THIS_MODULE, DEVICE_NAME);
    if (!mcbsptx.class)
    {
        printk(KERN_ALERT "class_create() failed\n");
        goto fail3;
    }
    mcbsptx.class->dev_attrs = control_attrs;

    /* Send uevents to udev to create device nodes */
    mcbsptx.dev = device_create(mcbsptx.class, NULL, mcbsptx.devt, NULL, DEVICE_NAME);
    if (!mcbsptx.dev)
    {
        goto fail4;
    }

    /* Init McBSP subsystem */
    if (mcbsptx_mcbsp_request())
    {
        printk(KERN_ALERT "mcbsptx_mcbsp_request() failed\n");
        goto fail5;
    }

    return 0;

 fail5:
    device_destroy(mcbsptx.class, mcbsptx.devt);
 fail4:
    class_destroy(mcbsptx.class);
 fail3:
    cdev_del(&mcbsptx.cdev);
 fail2:
    unregister_chrdev_region(mcbsptx.devt, 1);
 fail1:
    return -1;
}
module_init(mcbsptx_init);

static void __exit mcbsptx_exit(void)
{
    printk(KERN_INFO "McBSP-TX exit\n");

    omap_mcbsp_free(OMAP_MCBSP3);

    device_destroy(mcbsptx.class, mcbsptx.devt);
    class_destroy(mcbsptx.class);
    cdev_del(&mcbsptx.cdev);
    unregister_chrdev_region(mcbsptx.devt, 1);
}
module_exit(mcbsptx_exit);

MODULE_AUTHOR("Tamas Bondar");
MODULE_DESCRIPTION("OMAP3530 McBSP-TX driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.3");
