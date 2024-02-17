/**
 * Zero-copy DMA driver
 * Copyright (C) 2023-2024 Tibor Tusori
 * 
 * SPDX-License-Identifier: GPL-2.0
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

// Zero copy DMA backend
#include <zcdma.h>


// ------------------ Local constants ------------------
#define DRIVER_NAME 			"dma_manager"
#define MAX_NAME_LENG			32
#define ERROR 					-1
#define TIMEOUT_DEFAULT_MSECS	3000


// ------------------ Local types ------------------
struct dma_frontend {
    // frontend parameters
    char name[MAX_NAME_LENG];			    /* channel name */

    // Used DMA channel
    enum dma_transfer_direction direction;	/* DMA_MEM_TO_DEV or DMA_DEV_TO_MEM */
    struct dma_chan* dma_channel;           /* dma support */

    // Zerocopy backend
    struct zcdma* zcdma;                    /* zerocopy dma support */

    // Char device API
    struct cdev     cdev;
    dev_t           dev_node;
    struct device*  char_device;    
};


// This struct is used to store the DMA manager state
// and is passed as a pointer to the platform device
// driver data.
// It contains a list of frontends and the number of
// frontends.
struct dma_manager {
    struct platform_device*         platform_device;
    int                             frontend_count;
    const char**                    frontend_names;
    struct dma_frontend*            frontends;
};



// ------------------ Local functions ------------------
static int dma_manager_probe(struct platform_device *pdev);
static int dma_manager_remove(struct platform_device *pdev);
static int __init dma_manager_init(void);
static void __exit dma_manager_exit(void);



// ------------------ Global variables ------------------
/**
 * @brief Global sysfs level device class.
 * This class will be used for all the devices created
 * for the character device interface.
 * This class is created upon module loading.
 * 
 * 
 */
static struct class* dma_char_device_class = NULL;


// TODO
/* The following module parameter controls if the internal test runs when the module is inserted.
 * Note that this test requires a transmit and receive channel to function and uses the first
 * transmit and receive channnels when multiple channels exist.
 */
static unsigned internal_test = 0;
module_param(internal_test, int, S_IRUGO);



// ------------------ Function definitions ------------------

/* Open the file descriptor of the character device device
 * and get the driver specific DMA frontend structure.
 */
static int local_open(struct inode *inode, struct file *f)
{
    // Extract our driver specific structure from the character device data.
    f->private_data = container_of(inode->i_cdev, struct dma_frontend, cdev);

    return 0;
}

/* Release thefile descriptor of the character device.
 */
static int release(struct inode *inode, struct file *f)
{
    // TODO: Any cleanup operation???
    return 0;
}

/* Read data from the DMA channel. The data is copied directly into the user buffer.
 */
static ssize_t read(struct file *f, char __user *userbuf, size_t count, loff_t *f_pos)
{
    int read_size = 0;
    struct dma_frontend* frontend = (struct dma_frontend *)f->private_data;

    pr_debug("DMA read API is called with parameters: userbuf=0x%p, count=%lu, offset=%lld.\n", 
                                                                        userbuf,
                                                                        count,
                                                                        *f_pos);

    if (DMA_DEV_TO_MEM != frontend->direction)
    {
        pr_err("Can't read, '%s' is a TX device\n", frontend->name);
        return -EINVAL;
    }

    read_size = zcdma_read(frontend->zcdma, userbuf, count);	
    pr_debug("zcdma_read return: %d.\n", read_size);
    if (read_size <= 0)
    {
        pr_err("Can't read() on channel '%s', no data and timeout or error occurred.\n", frontend->name);
        return -EPERM;
    }
    
    return (ssize_t)read_size;
}


/* Write data to the DMA channel. The data is copied from the user buffer to the DMA buffer
 * and the DMA buffer is then submitted to the DMA channel.
 */
static ssize_t write(struct file *f, const char __user *userbuf, size_t count, loff_t *f_pos)
{
    int write_size = 0;
    struct dma_frontend *frontend = (struct dma_frontend *)f->private_data;

    pr_debug("DMA write API is called with parameters: userbuf=0x%p, count=%lu, offset=%lld.\n", 
                                                                        userbuf,
                                                                        count,
                                                                        *f_pos);

    if (DMA_MEM_TO_DEV != frontend->direction)
    {
        pr_err("Can't write, '%s' is an RX device.\n", frontend->name);
        return -EINVAL;
    }

    write_size = zcdma_write(frontend->zcdma, userbuf, count);
    pr_debug("zcdma_write return: %d.\n", write_size);
    if (write_size <= 0)
    {
        pr_err("Can't write() on channel '%s', no data and timeout or error occurred.\n", frontend->name);
        return -EPERM;
    }
    
    return (ssize_t)write_size;
}


// Character device operations
static struct file_operations dma_zero_file_ops = {
    .owner    = THIS_MODULE,
    .open     = local_open,
    .read     = read,
    .write    = write,
    .release  = release
};


/**
 * @brief Create a character device interface for the given dma zero channel.
 * 
 * @param frontend Channel to create a char device interface for.
 * @return int 0 when no error occurs, error code otherwise 
 */
static int cdevice_init(struct dma_frontend* frontend)
{
    int error_code;

    pr_debug("Creating char device for channel '%s'.\n", frontend->name);

    /* Allocate a character device number for the device.
     * The major number will be dynamically allocated.
     */
    error_code = alloc_chrdev_region(&frontend->dev_node, 0, 1, DRIVER_NAME);
    if (error_code)
    {
        pr_err("Unable to get a char device number.\n");
        return error_code;
    }

    /* Create the device class for the character device interface.
     * This will create a /sys/class/ directory.
     */
    cdev_init(&frontend->cdev, &dma_zero_file_ops);
    frontend->cdev.owner = THIS_MODULE;

    // Add the character device to the system.
    error_code = cdev_add(&frontend->cdev, frontend->dev_node, 1);
    if (error_code)
    {
        pr_err("Unable to add char device.\n");
        goto init_error1;
    }

    /* Create the device node in /dev so the device is accessible
     * as a character device
     */
    frontend->char_device = device_create(
            dma_char_device_class,  // class
            NULL,                   // parent device
            frontend->dev_node,     // dev_t 
            (void*)frontend,        // drv_data
            "%s", frontend->name);

    if ((NULL==frontend->char_device) || IS_ERR(frontend->char_device))
    {
        pr_err("Unable to create the device for the char device interface with name: '%s'\n",frontend->name);
        goto init_error2;
    }

    return 0;


init_error2:
    // Remove the character device from the kernel.
    cdev_del(&frontend->cdev);

init_error1:
    // Free the major/minor number region.
    unregister_chrdev_region(frontend->dev_node, 1);
    return error_code;
}


/**
 * @brief Remove the character device interface for the given frontend.
 * 
 * @param frontend Frontend object to remove the char device interface for.
 */
static void cdevice_deinit(struct dma_frontend* frontend)
{
    // Deinit the character device interface.
    if(frontend->char_device)
    {
        pr_debug("Removing char device interface for '%s'.\n", frontend->name);

        // Destroy the device node in /dev.
        device_destroy(dma_char_device_class, frontend->dev_node);
        frontend->char_device = NULL;

        // Remove the character device from the kernel.
        cdev_del(&frontend->cdev);
        memset(&frontend->cdev, 0, sizeof(frontend->cdev));

        // Free the major/minor number region.
        unregister_chrdev_region(frontend->dev_node, 1);
        frontend->dev_node = 0;
    }

    return;
}


/**
 * @brief                   Create a frontend by getting a DMA channel from the DMA Engine and then setting
 *                          up a character device to allow user space control.
 *                          After this function the object is either fully initialized,
 *                          or filled completely with 0.
 * 
 * @param frontend          Frontend to initialize.
 * @param frontend_name     Name of the frontend.
 * @param dma_channel       Dma cannel to be used by the frontend.
 * @param direction         Direction of the dma channel.
 * @return true             In case the initialization succeeds.
 * @return false            In case of an error. In this case the frontend does not need to be uninitialized.
 */
static bool frontend_init(  struct dma_frontend*    frontend, 
                            const char*             frontend_name,
                            struct dma_chan*        dma_channel,
                            u32                     direction   )
{
    bool init_error = false;
    int rc;
    struct dma_hw_channel_info  dma_channel_hw_info;

    // Init the frontend parameters
    strncpy(frontend->name, frontend_name, MAX_NAME_LENG);

    // Init the dma channel parameters
    frontend->dma_channel = dma_channel;
    frontend->direction = direction;    

    // Initialize a zerocopy engine for the given channel.
    dma_channel_hw_info.direction = direction;
    dma_channel_hw_info.dma_chan = dma_channel;
    frontend->zcdma = zcdma_alloc(&dma_channel_hw_info);
    if(NULL == frontend->zcdma)
    {
        pr_err("Zerocopy engine initialization failed for frontend '%s'.\n", frontend_name);
        init_error = true;
        memset(frontend, 0, sizeof(*frontend));
    }

    // Initialize the character device for the dma channel
    if(!init_error)
    {
        rc = cdevice_init(frontend);
        if (0 != rc)
        {
            init_error = true;
            // revert the zcdma initialization
            zcdma_free(frontend->zcdma);
            memset(frontend, 0, sizeof(*frontend));
        }
    }

    return init_error;
}


/**
 * @brief Deinitialize a frontend object.
 *        It accepts both initialized and full 0 frontend objects.
 * 
 * @param frontend Object to deinitialize.
 */
static void frontend_deinit(struct dma_frontend* frontend)
{
    pr_info("Deinitializing frontend '%s'.\n", frontend->name);

    // clean up the char device interface
    cdevice_deinit(frontend);

    // clean up the zcdma data
    if(NULL != frontend->zcdma)
    {
        zcdma_free(frontend->zcdma);
    }

    // terminate all the dma transfers, and release the dma channel
    if(NULL != frontend->dma_channel)
    {
        dmaengine_terminate_sync(frontend->dma_channel);
        dma_release_channel(frontend->dma_channel);
    }

    // clear all the data from the frontend object
    memset(frontend, 0, sizeof(*frontend));

    return;
}


/**
 * @brief Try to initialize the driver for the given platform device.
 * 
 * @param pdev Platform device to initialize the driver for.
 * @return int Error code.
 */
static int dma_manager_probe(struct platform_device *pdev)
{
    int ret = 0;
    int error_code, i;
    bool frontend_init_success                  =   false;
    struct dma_manager*         manager         =   NULL;
    struct dma_frontend*        frontend        =   NULL;
    const char*                 frontend_name   =   NULL;
    struct dma_chan*            dma_channel     =   NULL;

    if(NULL != pdev->name)
    {
        dev_info(&pdev->dev, "Probing the driver with the device '%s'...\n", pdev->name);
    }
    
    // Allocate memory for the driver data and register it into the driver.
    manager = (struct dma_manager *) devm_kmalloc(&pdev->dev, sizeof(struct dma_manager), GFP_KERNEL);
    if (!manager) {		
        dev_err(&pdev->dev, "Could not allocate DMA manager.\n");
        return -ENOMEM;
    }
    dev_set_drvdata(&pdev->dev, (void*)manager);
    manager->platform_device = pdev;

    // Figure out how many channels are there from the device tree
    // based on the number of strings in the dma-names property.
    manager->frontend_count = device_property_read_string_array(&pdev->dev,
                                                                "dma-names", 
                                                                NULL, 
                                                                0);
    if (manager->frontend_count <= 0)
    {
        dev_warn(&pdev->dev, "No DMA channels are defined for the manager.\n");
        return 0;
    }
    dev_info(&pdev->dev, "Device Tree Channel count: %d\n", manager->frontend_count);

    // Allocate the memory for channel names and then get the names
    // from the device tree.
    manager->frontend_names = devm_kmalloc_array(&pdev->dev, manager->frontend_count, 
                                                    sizeof(char*), GFP_KERNEL);
    if (NULL == manager->frontend_names)
    {
        dev_err(&pdev->dev, "Cannot allocate memory for the frontend names.\n");
        return -ENOMEM;
    }

    error_code = device_property_read_string_array(
                    &pdev->dev, 
                    "dma-names", 
                    (const char **)manager->frontend_names, 
                    manager->frontend_count
                );
    if (error_code < 0)
    {
        dev_err(&pdev->dev, "Could not get the dma channel names from the device tree.\n");
        return error_code;
    }

    //  Allocate the memory for the channel structures.
    manager->frontends = devm_kmalloc(&pdev->dev, sizeof(struct dma_frontend) * manager->frontend_count, GFP_KERNEL);
    if (NULL == manager->frontends)
    {
        dev_err(&pdev->dev, "Cannot allocate memory for the frontends.\n");
        return -ENOMEM;
    }

    // clear the frontends, full 0 is a valid uninitialized state
    memset(manager->frontends, 0, sizeof(struct dma_frontend) * manager->frontend_count);

    // Initialize a frontend for every specified dma channel.
    for (i = 0; i < manager->frontend_count; i++)
    {
        frontend_name = manager->frontend_names[i];        
        frontend = &manager->frontends[i];
        pr_debug("Requesting dma channel with name '%s' from the dmaengine.\n", frontend_name);
        // Request the DMA channel from the DMA engine.
        dma_channel = dma_request_chan(&manager->platform_device->dev, frontend_name);
        if(IS_ERR(dma_channel))
        {
            ret = PTR_ERR(dma_channel);
            dev_err(&pdev->dev, "Could not get the dma chanel with name '%s' from the dmaengine, error:%d!\n",
                        frontend_name,
                        ret);
            // terminate the init cycle
            break;
        }
        else
        {
            dev_info(&pdev->dev, "Initializing frontend '%s'.\n", frontend_name);
            // Initialize the frontend object.
            // The dma_channel object is now owned by the frontend object, 
            // and will be released by the frontend_deinit function using the dma_release_channel function.
            frontend_init_success = frontend_init(  frontend,
                                                    frontend_name,
                                                    dma_channel,
                                                    DMA_MEM_TO_DEV);
            if (false == frontend_init_success)
            {
                dev_err(&pdev->dev, "Error while initializing frontend object for dma channel with name: '%s'\n",
                    frontend_name
                );
                ret = -ENODEV;
            } 
        }
    }

    // error handling
    if(0 != ret)
    {
        // We have encountered error during the frontend initializations,
        // so clean up the half initialized frontend array.
        for (i = 0; i < manager->frontend_count; i++)
        {
            frontend_deinit(&manager->frontends[i]);
        }
    }

    return ret;
}
 

/**
 * @brief Unload the driver for the given platform device.
 * 
 * @param pdev Platform device to deinit
 */
static int dma_manager_remove(struct platform_device *pdev)
{
    int i;
    struct device*      dev = &pdev->dev;
    struct dma_manager* manager = dev_get_drvdata(dev);

    dev_info(&pdev->dev, "Driver is unloading...\n");

    // Deinitialize all the frontends.
    for (i = 0; i < manager->frontend_count; i++)
    {
        frontend_deinit(&manager->frontends[i]);
    }
    manager->frontend_count = 0;
    manager->frontend_names = NULL;
    manager->frontends = NULL;
    manager->platform_device = NULL;

    return 0;
}


/**
 * @brief Device tree match table for the dma_manager driver.
 */
static const struct of_device_id dma_manager_of_ids[] = {
    { .compatible = "xlnx,dma_manager",},
    // ensure backward comptability with the dma proxy driver
    // of xilinx
    { .compatible = "xlnx,dma_proxy",},
    {}
};


/**
 * @brief Platform driver structure for the dma_manager driver.
 */
static struct platform_driver dma_manager_driver = {
    .driver = {
        .name = "dma_manager_driver",
        .owner = THIS_MODULE,
        .of_match_table = dma_manager_of_ids,
    },
    .probe = dma_manager_probe,
    .remove = dma_manager_remove,
};


/**
 * @brief Kernel module initializer, allocated a class for the
 *          character device interface and registers the dma_manager
 *          platform driver.
 * 
 * @return int 0 on success, ERROR otherwise.
 */
static int __init dma_manager_init(void)
{
    int errorCode = 0;

    pr_debug("Initializing the DMA manager module.");

    // Create a device class in the sysfs.
    // This will be used to create the individual devices for every dma channels.
    dma_char_device_class = class_create(THIS_MODULE, DRIVER_NAME);
    if ((NULL==dma_char_device_class) || IS_ERR(dma_char_device_class)) {
        pr_err("Unable to create class\n");
        errorCode = ERROR;
    }
    else
    {
        pr_debug("Registering the dma manager platform driver.");
        errorCode = platform_driver_register(&dma_manager_driver);
    }

    return errorCode;    
}


/**
 * @brief Kernel module exit function, unregisters the dma_manager
 *          platform driver and destroys the class for the character
 *          device interface.
 */
static void __exit dma_manager_exit(void)
{
    platform_driver_unregister(&dma_manager_driver);

    if((NULL!= dma_char_device_class) && !IS_ERR(dma_char_device_class))
    {
        class_destroy(dma_char_device_class);
        dma_char_device_class = NULL;
    }
}

module_init(dma_manager_init)
module_exit(dma_manager_exit)

MODULE_AUTHOR("Tusori Tibor");
MODULE_DESCRIPTION("DMA manager driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
