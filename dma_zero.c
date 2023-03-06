/**
 * Copyright (C) 2021 Xilinx, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

/* DMA Proxy
 *
 * This module is designed to be a small example of a DMA device driver that is
 * a client to the DMA Engine using the AXI DMA / MCDMA driver. It serves as a proxy
 * for kernel space DMA control to a user space application.
 *
 * A zero copy scheme is provided by allowing user space to mmap a kernel allocated
 * memory region into user space, referred to as a set of channel buffers. Ioctl functions 
 * are provided to start a DMA transfer (non-blocking), finish a DMA transfer (blocking) 
 * previously started, or start and finish a DMA transfer blocking until it is complete.
 * An input argument which specifies a channel buffer number (0 - N) to be used for the
 * transfer is required.
 *
 * By default the kernel memory allocated for user space mapping is going to be 
 * non-cached at this time. Non-cached memory is pretty slow for the application.
 * A h/w coherent system for MPSOC has been tested and is recommended for higher
 * performance applications. 
 *
 * Hardware coherency requires the following items in the system as documented on the 
 * Xilinx wiki and summarized below::
 *   The AXI DMA read and write channels AXI signals must be tied to the correct state to
 *    generate coherent transactions.
 *   An HPC slave port on MPSOC is required
 *   The CCI of MPSOC must be initialized prior to the APU booting Linux
 *   A dma-coherent property is added in the device tree for the proxy driver.
 *
 * There is an associated user space application, dma_proxy_test.c, and dma_proxy.h
 * that works with this device driver.
 *
 * The hardware design was tested with an AXI DMA / MCDMA  with scatter gather and
 * with the transmit channel looped back to the receive channel. It should
 * work with or without scatter gather as the scatter gather mentioned in the 
 * driver is only at the s/w framework level rather than in the hw.
 *
 * This driver is character driver which creates devices that user space can
 * access for each DMA channel, such as /dev/dma_proxy_rx and /dev/dma_proxy_tx.
 * The number and names of channels are taken from the device tree.
 * Multiple instances of the driver (with multiple IPs) are also supported.

 * An internal test mode is provided to allow it to be self testing without the 
 * need for a user space application and this mode is good for making bigger
 * changes to this driver.
 *
 * This driver is designed to be simple to help users get familiar with how to 
 * use the DMA driver provided by Xilinx which uses the Linux DMA Engine. 
 *
 * To use this driver a node must be added into the device tree.  Add a 
 * node similar to the examples below adjusting the dmas property to match the
 * name of the AXI DMA / MCDMA node.
 * 
 * The dmas property contains pairs with the first of each pair being a reference
 * to the DMA IP in the device tree and the second of each pair being the
 * channel of the DMA IP. For the AXI DMA IP the transmit channel is always 0 and
 * the receive is always 1. For the AXI MCDMA IP the 1st transmit channel is
 * always 0 and receive channels start at 16 since there can be a maximum of 16
 * transmit channels. Each name in the dma-names corresponds to a pair in the dmas
 * property and is only a logical name that allows user space access to the channel
 * such that the name can be any name as long as it is unique.
 *
 *	For h/w coherent systems with MPSoC, the property dma-coherent can be added
 * to the node in the device tree. 
 * 
 * Example device tree nodes: 
 *
 * For AXI DMA with transmit and receive channels with a loopback in hardware
 * 
 * dma_proxy {
 *   compatible ="xlnx,dma_proxy";
 *   dmas = <&axi_dma_1_loopback 0  &axi_dma_1_loopback 1>;
 *   dma-names = "dma_proxy_tx", "dma_proxy_rx";
 * };
 *
 * For AXI DMA with only the receive channel
 * 
 * dma_proxy2 {
 *   compatible ="xlnx,dma_proxy";
 *   dmas = <&axi_dma_0_noloopback 1>;
 *   dma-names = "dma_proxy_rx_only";
 * };
 *
 * For AXI MCDMA with two channels 
 *
 * dma_proxy3 {
 *   compatible ="xlnx,dma_proxy";
 *   dmas = <&axi_mcdma_0 0  &axi_mcdma_0 16 &axi_mcdma_0 1 &axi_mcdma_0 17> ;
 *   dma-names = "dma_proxy_tx_0", "dma_proxy_rx_0", "dma_proxy_tx_1", "dma_proxy_rx_1";
 * };
 */

// Override the fmt string preprocessor, so that every
// log message contains the module name.
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt


#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/of_dma.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>

#include <zcdma.h>

MODULE_LICENSE("GPL");

#define DRIVER_NAME 			"dma_proxy"
#define MAX_NAME_LENG			32
#define TX_CHANNEL				0
#define RX_CHANNEL				1
#define ERROR 					-1
#define TEST_SIZE 				1024
#define TIMEOUT_DEFAULT_MSECS	3000

/* The following module parameter controls if the internal test runs when the module is inserted.
 * Note that this test requires a transmit and receive channel to function and uses the first
 * transmit and receive channnels when multiple channels exist.
 */
static unsigned internal_test = 0;
module_param(internal_test, int, S_IRUGO);


static int dma_zero_probe(struct platform_device *pdev);
static int dma_zero_remove(struct platform_device *pdev);
static int __init dma_zero_init(void);
static void __exit dma_zero_exit(void);

struct dma_zero_channel {
    struct channel_buffer *buffer_table_p;	/* user to kernel space interface */
    dma_addr_t buffer_phys_addr;

    struct device *proxy_device_p;			/* character device support */
    struct device *dma_device_p;

    struct dma_chan *channel_p;			/* dma support */
    unsigned int timeout;				/* DMA transfer timeout */
    int bdindex;

    /* Char device API */
    dev_t dev_node;
    struct cdev cdev;
    struct class *class_p;

    /* Zerocopy */
    char name[MAX_NAME_LENG];			/* channel name */
    u32 direction;						/* DMA_MEM_TO_DEV or DMA_DEV_TO_MEM */
    struct zcdma* zcdma;
};


struct dma_zero {
    struct platform_device*     platform_device;
    int                         dma_channel_count;
    const char**                dma_channel_names;
    struct dma_zero_channel*    dma_channels;
};

static int total_count;


/* Open the device file and set up the data pointer to the proxy channel data for the
 * proxy channel such that the ioctl function can access the data structure later.
 */
static int local_open(struct inode *ino, struct file *file)
{
    file->private_data = container_of(ino->i_cdev, struct dma_proxy_channel, cdev);

    return 0;
}

/* Close the file and there's nothing to do for it
 */
static int release(struct inode *ino, struct file *file)
{
#if 0
    struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)file->private_data;
    struct dma_device *dma_device = pchannel_p->channel_p->device;

    /* Stop all the activity when the channel is closed assuming this
     * may help if the application is aborted without normal closure
     * This is not working and causes an issue that may need investigation in the 
     * DMA driver at the lower level.
     */
    dma_device->device_terminate_all(pchannel_p->channel_p);
#endif
    return 0;
}

static ssize_t read(struct file *file, char __user *userbuf, size_t count, loff_t *f_pos)
{
    ssize_t rc = 0;
    int read_size = 0;
    struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)file->private_data;

    if (DMA_DEV_TO_MEM != pchannel_p->direction)
    {
        printk(KERN_ERR ": %s: can't read, is a TX device\n", pchannel_p->name);
        return -EINVAL;
    }

    read_size = zcdma_read(pchannel_p->zcdma, userbuf, count);	
    if (read_size <= 0)
    {
        printk(KERN_ERR "%s: can't read(), no data and timeout or error occurred\n", pchannel_p->name);
        return -EPERM;
    }
    
    rc = read_size;
    return rc;
}

static ssize_t write(struct file *file, const char __user *userbuf, size_t count, loff_t *f_pos)
{
    ssize_t rc = 0;
    int write_size = 0;
    struct dma_proxy_channel *pchannel_p = (struct dma_proxy_channel *)file->private_data;

    if (DMA_MEM_TO_DEV != pchannel_p->direction)
    {
        printk(KERN_ERR "%s: can't write, is an RX device\n", pchannel_p->name);
        return -EINVAL;
    }

    rc = zcdma_write(pchannel_p->zcdma, userbuf, count);
    if (write_size <= 0)
    {
        printk(KERN_ERR "%s: can't write(), no data and timeout or error occurred\n", pchannel_p->name);
        return -EPERM;
    }
    
    rc = write_size;
    return rc;
}

static struct file_operations dma_zero_file_ops = {
    .owner    = THIS_MODULE,
    .open     = local_open,
    .read     = read,
    .write    = write,
    .release  = release,
    .unlocked_ioctl = ioctl
};


/* Initialize the driver to be a character device such that is responds to
 * file operations.
 */
static int cdevice_init(    struct dma_zero*            dma_zero, 
                            struct dma_zero_channel*    channel_desc, 
                            const char*                 name            )
{
    int error_code;
    char device_name[2 * MAX_NAME_LENG] = "dma_proxy";
    static struct class *local_class_p = NULL;

    /* Allocate a character device from the kernel for this driver.
     */
    error_code = alloc_chrdev_region(&channel_desc->dev_node, 0, 1, "dma_proxy");
    if (error_code)
    {
        pr_err("Unable to get a char device number.\n");
        return error_code;
    }

    /* Initialize the device data structure before registering the character 
     * device with the kernel.
     */
    cdev_init(&channel_desc->cdev, &dma_zero_file_ops);
    channel_desc->cdev.owner = THIS_MODULE;
    error_code = cdev_add(&channel_desc->cdev, channel_desc->dev_node, 1);
    if (error_code)
    {
        pr_err("Unable to add char device\n");
        goto init_error1;
    }

    /* Only one class in sysfs is to be created for multiple channels,
     * create the device in sysfs which will allow the device node
     * in /dev to be created
     */
    if (!local_class_p) {
        local_class_p = class_create(THIS_MODULE, DRIVER_NAME);

        if (IS_ERR(pchannel_p->dma_device_p->class)) {
            dev_err(pchannel_p->dma_device_p, "unable to create class\n");
            error_code = ERROR;
            goto init_error2;
        }
    }
    pchannel_p->class_p = local_class_p;

    /* Create the device node in /dev so the device is accessible
     * as a character device
     */
    strcat(device_name, name);
    pchannel_p->proxy_device_p = device_create(pchannel_p->class_p, NULL,
                           pchannel_p->dev_node, NULL, name);

    if (IS_ERR(pchannel_p->proxy_device_p)) {
        dev_err(pchannel_p->dma_device_p, "unable to create the device\n");
        goto init_error3;
    }

    return 0;

init_error3:
    class_destroy(pchannel_p->class_p);

init_error2:
    cdev_del(&pchannel_p->cdev);

init_error1:
    unregister_chrdev_region(channel_desc->dev_node, 1);
    return error_code;
}

/* Exit the character device by freeing up the resources that it created and
 * disconnecting itself from the kernel.
 */
static void cdevice_exit(struct dma_proxy_channel *pchannel_p)
{
    /* Take everything down in the reverse order
     * from how it was created for the char device
     */
    if (pchannel_p->proxy_device_p) {
        device_destroy(pchannel_p->class_p, pchannel_p->dev_node);

        /* If this is the last channel then get rid of the /sys/class/dma_proxy
         */
        if (total_count == 1)
            class_destroy(pchannel_p->class_p);

        cdev_del(&pchannel_p->cdev);
        unregister_chrdev_region(pchannel_p->dev_node, 1);
    }
}

/* Create a DMA channel by getting a DMA channel from the DMA Engine and then setting
 * up the channel as a character device to allow user space control.
 */
static bool create_channel( struct dma_zero*            dma_zero,
                            struct dma_zero_channel*    channel_desc, 
                            const char*                 name, 
                            u32                         direction   )
{
    int rc, bd;
    struct dma_chan*    dma;
    struct zcdma_hw_info    dma_channel_hw_info;

    // Request the DMA channel from the DMA engine and then use the device from
    // the channel for the proxy channel also.
    dma = dma_request_slave_channel(&dma_zero->platform_device->dev, name);
    if (NULL == dma)
    {
        pr_err("Could not get the dma chanel with name '%s' from the dmaengine.\n", name);
        return false;
    }

    // Initialize a zerocopy engine for the given channel.
    dma_channel_hw_info.direction = direction;
    dma_channel_hw_info.dma_chan = dma;
    channel_desc->zcdma = zcdma_alloc(&dma_channel_hw_info);
    if(NULL == channel_desc->zcdma)
    {
        pr_error("Zerocopy engine initialization failed for dma channel '%s'.\n", name);
        return false;
    }

    // Initialize the character device for the dma channel
    rc = cdevice_init(dma_zero, channel_desc, name);
    if (rc)
    {
        return rc;
    }

    // Save the dma channel relevant data
    strncpy(channel_desc->name, name, MAX_NAME_LENG);
    channel_desc->direction = direction;
    channel_desc->timeout = TIMEOUT_DEFAULT_MSECS;

    return true;
}

/* Initialize the dma proxy device driver module.
 */
static int dma_zero_probe(struct platform_device *pdev)
{
    int error_code, i;
    struct dma_zero* drv_data;
    struct device *dev = &pdev->dev;

    pr_info("Driver is initializing...\n");
    
    // Allocate memory for the driver data and register it into the driver.
    drv_data = (struct dma_zero *) devm_kmalloc(&pdev->dev, sizeof(struct dma_zero), GFP_KERNEL);
    if (!drv_data) {		
        dev_err(dev, "Cound not allocate proxy device\n");
        return -ENOMEM;
    }
    dev_set_drvdata(dev, drv_data);
    drv_data->platform_device = pdev;

    // Figure out how many channels there are from the device tree based
    // on the number of strings in the dma-names property-
    drv_data->dma_channel_count = device_property_read_string_array(&pdev->dev,
                         "dma-names", NULL, 0);
    if (drv_data->dma_channel_count <= 0)
    {
        return 0;
    }
    pr_info("Device Tree Channel Count: %d\n", drv_data->dma_channel_count);

    // Allocate the memory for channel names and then get the names
    // from the device tree.
    drv_data->dma_channel_names = devm_kmalloc_array(&pdev->dev, drv_data->dma_channel_count, 
            sizeof(char *), GFP_KERNEL);
    if (!drv_data->dma_channel_names)
    {
        return -ENOMEM;
    }
    error_code = device_property_read_string_array(
                    &pdev->dev, 
                    "dma-names", 
                    (const char **)drv_data->dma_channel_names, 
                    drv_data->dma_channel_count
                );
    if (error_code < 0)
    {
        pr_err("Could not get the dma channel names from the device tree.");
        return error_code;
    }

    //  Allocate the memory for the channel structures.
    drv_data->dma_channels = devm_kmalloc(&pdev->dev,
            sizeof(struct dma_zero_channel) * drv_data->dma_channel_count, GFP_KERNEL);
    if (NULL == drv_data->dma_channels)
    {
        return -ENOMEM;
    }

    // Create the channels in the proxy. The direction does not matter
    // as the DMA channel has it inside it and uses it, other than this will not work 
    // for cyclic mode.
    for (i = 0; i < drv_data->dma_channel_count; i++)
    {
        drv_data->dma_channels = NULL;

        pr_info("Creating channel: '%s'\n", drv_data->dma_channel_names[i]);
        error_code = create_channel(
                        pdev, 
                        &drv_data->dma_channels[i], 
                        drv_data->dma_channel_names[i], 
                        DMA_MEM_TO_DEV);

        if (0 != error_code)
        {
            pr_error("Error ('%d') while creating channel object for dma channel with name: '%s'\n",
                error_code,
                drv_data->dma_channel_names[i]
            );
            return error_code;
        } 
    }

    // TODO internal test
    return 0;
}
 
/* Exit the dma proxy device driver module.
 */
static int dma_zero_remove(struct platform_device *pdev)
{
    int i;
    struct device *dev = &pdev->dev;
    struct dma_proxy *lp = dev_get_drvdata(dev);

    pr_info("Driver is unloading...\n");

    /* Take care of the char device infrastructure for each
     * channel except for the last channel. Handle the last
     * channel seperately.
     */
    for (i = 0; i < lp->channel_count; i++) { 
        if (lp->channels[i].proxy_device_p)
            cdevice_exit(&lp->channels[i]);
        total_count--;
    }
    /* Take care of the DMA channels and any buffers allocated
     * for the DMA transfers. The DMA buffers are using managed
     * memory such that it's automatically done.
     */
    for (i = 0; i < lp->channel_count; i++)
        if (lp->channels[i].channel_p) {
            lp->channels[i].channel_p->device->device_terminate_all(lp->channels[i].channel_p);
            dma_release_channel(lp->channels[i].channel_p);
        }

    return 0;
}

static const struct of_device_id dma_zero_of_ids[] = {
    { .compatible = "xlnx,dma_zero",},
    {}
};

static struct platform_driver dma_zero_driver = {
    .driver = {
        .name = "dma_zero_driver",
        .owner = THIS_MODULE,
        .of_match_table = dma_zero_of_ids,
    },
    .probe = dma_zero_probe,
    .remove = dma_zero_remove,
};

static int __init dma_zero_init(void)
{
    return platform_driver_register(&dma_zero_driver);

}

static void __exit dma_zero_exit(void)
{
    platform_driver_unregister(&dma_zero_driver);
}

module_init(dma_zero_init)
module_exit(dma_zero_exit)

MODULE_AUTHOR("Tusori Tibor");
MODULE_DESCRIPTION("Zerocopy DMS driver");
MODULE_LICENSE("GPL v2");
