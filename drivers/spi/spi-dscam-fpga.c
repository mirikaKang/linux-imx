// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2019 NXP
 * Copyright 2020-2021 Variscite Ltd.
 * Copyright 2024-2025 Abyz inc.
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/completion.h>

#define DEVICE_NAME "abyz_fpga_control_spi"

struct spi_config {
    uint32_t frequency;
    uint8_t bits_per_word;
    uint8_t mode;
};

struct abyz_spi_dev {
    struct cdev c_dev;
    struct spi_device *spi;
    struct mutex buf_lock;
    struct completion spi_done;

    struct spi_config config;
    u8 *tx_buffer;
    u8 *rx_buffer;
    int cs_gpio;
};

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static dev_t dev_num;
static struct class *abyz_spi_class = NULL;

#define SPI_IOC_MAGIC 'k'
#define SPI_SET_MODE _IOW(SPI_IOC_MAGIC, 1, uint8_t)
#define SPI_SET_FREQUENCY _IOW(SPI_IOC_MAGIC, 2, uint32_t)
#define SPI_SET_BITS_PER_WORD _IOW(SPI_IOC_MAGIC, 3, uint8_t)
#define SPI_GET_CONFIG _IOR(SPI_IOC_MAGIC, 4, struct spi_config)
#define SPI_SET_CS_GPIO _IOW(SPI_IOC_MAGIC, 5, int)

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int device_open(struct inode *inode, struct file *file);
static int device_release(struct inode *inode, struct file *file);
static ssize_t device_read(struct file *file, char __user *buffer, size_t length, loff_t *offset);
static ssize_t device_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset);
static int abyz_spi_probe(struct spi_device *spi);
static void  abyz_spi_remove(struct spi_device *spi);

static ssize_t abyz_spi_sync(struct abyz_spi_dev *abyz_spi, struct spi_message *message, int timeout);
static ssize_t abyz_spi_sync_write(struct abyz_spi_dev *abyz_spi, size_t len, int timeout);
static ssize_t abyz_spi_sync_read(struct abyz_spi_dev *abyz_spi, size_t len, int timeout);

static ssize_t abyz_spi_sync(struct abyz_spi_dev *abyz_spi, struct spi_message *message, int timeout) {
    int status;
    struct spi_device *spi = abyz_spi->spi;

    reinit_completion(&abyz_spi->spi_done);
    message->complete = (void (*)(void *))complete;
    message->context = &abyz_spi->spi_done;

    status = spi_async(spi, message);
    if (status == 0) {
        if (!wait_for_completion_timeout(&abyz_spi->spi_done, msecs_to_jiffies(timeout))) {
            dev_err(&spi->dev, "SPI transfer timed out\n");
            return -ETIMEDOUT;
        }
        status = message->status;
        dev_dbg(&spi->dev,"[%s:%d] status=%d\n",__func__,__LINE__,status);
        if (status == 0)
            status = message->actual_length;
    }

    dev_dbg(&spi->dev,"[%s:%d] status=%d\n",__func__,__LINE__,status);
    return status;
}

static inline ssize_t abyz_spi_sync_write(struct abyz_spi_dev *abyz_spi, size_t len, int timeout) {
    struct spi_transfer t = {
        .tx_buf = abyz_spi->tx_buffer,
        .len = len,
        .speed_hz = abyz_spi->spi->max_speed_hz,
    };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return abyz_spi_sync(abyz_spi, &m, timeout);
}

static inline ssize_t abyz_spi_sync_read(struct abyz_spi_dev *abyz_spi, size_t len, int timeout) {
    struct spi_transfer t = {
        .rx_buf = abyz_spi->rx_buffer,
        .len = len,
        .speed_hz = abyz_spi->spi->max_speed_hz,
    };
    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    return abyz_spi_sync(abyz_spi, &m, timeout);
}

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    struct abyz_spi_dev *abyz_spi_device = file->private_data;
    struct spi_device *spi = abyz_spi_device->spi;
    if (!abyz_spi_device) {
        dev_err(&spi->dev,"abyz_spi_device is NULL in ioctl\n");
        return -ENODEV;
    }
    dev_info(&spi->dev,"device_ioctl: cmd=%u\n", cmd);

    switch (cmd) {
        case SPI_SET_MODE:
            if (copy_from_user(&abyz_spi_device->config.mode, (uint8_t __user *)arg, sizeof(uint8_t)))
                return -EFAULT;
            abyz_spi_device->spi->mode = abyz_spi_device->config.mode;
            spi_setup(abyz_spi_device->spi);
            dev_info(&spi->dev,"set mode=%u\n", abyz_spi_device->config.mode);
            break;

        case SPI_SET_FREQUENCY:
            if (copy_from_user(&abyz_spi_device->config.frequency, (uint32_t __user *)arg, sizeof(uint32_t)))
                return -EFAULT;
            abyz_spi_device->spi->max_speed_hz = abyz_spi_device->config.frequency;
            spi_setup(abyz_spi_device->spi);
            dev_info(&spi->dev,"set frequency=%u\n", abyz_spi_device->config.frequency);
            break;

        case SPI_SET_BITS_PER_WORD:
            if (copy_from_user(&abyz_spi_device->config.bits_per_word, (uint8_t __user *)arg, sizeof(uint8_t)))
                return -EFAULT;
            abyz_spi_device->spi->bits_per_word = abyz_spi_device->config.bits_per_word;
            spi_setup(abyz_spi_device->spi);
            dev_info(&spi->dev,"set bits per word=%u\n", abyz_spi_device->config.bits_per_word);
            break;

        case SPI_GET_CONFIG:
            if (copy_to_user((void __user *)arg, &abyz_spi_device->config, sizeof(struct spi_config)))
                return -EFAULT;
            dev_info(&spi->dev,"get config\n");
            break;

        case SPI_SET_CS_GPIO:
            if (copy_from_user(&abyz_spi_device->cs_gpio, (int __user *)arg, sizeof(int)))
                return -EFAULT;
            if (!gpio_is_valid(abyz_spi_device->cs_gpio)) {
                dev_err(&spi->dev,"Invalid GPIO: %d\n", abyz_spi_device->cs_gpio);
                return -EINVAL;
            }
            if (gpio_request(abyz_spi_device->cs_gpio, "cs_gpio")) {
                dev_err(&spi->dev,"Failed to request GPIO: %d\n", abyz_spi_device->cs_gpio);
                return -EBUSY;
            }
            gpio_direction_output(abyz_spi_device->cs_gpio, 1);
            dev_info(&spi->dev,"set CS GPIO=%d\n", abyz_spi_device->cs_gpio);
            break;

        default:
            return -ENOTTY;
    }

    return 0;
}

static int device_open(struct inode *inode, struct file *file) {
    int status = 0;
    struct abyz_spi_dev *abyz_spi_device =
        container_of(inode->i_cdev, struct abyz_spi_dev, c_dev);

    pr_info("device_open: device=%p\n", abyz_spi_device);

    if (!mutex_trylock(&abyz_spi_device->buf_lock)) {
        dev_info(&abyz_spi_device->spi->dev,
                 "Abyz SPI Device is in use by another process\n");
        return -EBUSY;
    }

    if (!abyz_spi_device->tx_buffer) {
        abyz_spi_device->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
        if (!abyz_spi_device->tx_buffer) {
            dev_dbg(&abyz_spi_device->spi->dev, "open/ENOMEM\n");
            status = -ENOMEM;
            goto exit_err;
        }
    }

    if (!abyz_spi_device->rx_buffer) {
        abyz_spi_device->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
        if (!abyz_spi_device->rx_buffer) {
            dev_dbg(&abyz_spi_device->spi->dev, "open/ENOMEM");
            kfree(abyz_spi_device->tx_buffer);
            abyz_spi_device->tx_buffer = NULL;
            status = -ENOMEM;
            goto exit_err;
        }
    }
    file->private_data = abyz_spi_device;
    mutex_unlock(&abyz_spi_device->buf_lock);
    return 0;

exit_err:
    mutex_unlock(&abyz_spi_device->buf_lock);
    return status;
}

static int device_release(struct inode *inode, struct file *file) {
    struct abyz_spi_dev *abyz_spi_device = file->private_data;
    pr_info("device_release: device=%p\n", abyz_spi_device);
    mutex_lock(&abyz_spi_device->buf_lock);
    file->private_data = NULL;
    kfree(abyz_spi_device->tx_buffer);
    abyz_spi_device->tx_buffer = NULL;
    kfree(abyz_spi_device->rx_buffer);
    abyz_spi_device->rx_buffer = NULL;
    if (gpio_is_valid(abyz_spi_device->cs_gpio))
        gpio_free(abyz_spi_device->cs_gpio);
    mutex_unlock(&abyz_spi_device->buf_lock);
    return 0;
}

static ssize_t device_read(struct file *file, char __user *buffer, size_t length, loff_t *offset) {
    struct abyz_spi_dev *abyz_spi_device;
    size_t status;
    struct spi_device *spi;

    abyz_spi_device = file->private_data;
    spi = abyz_spi_device->spi;

    if (length > bufsiz)
        return -EMSGSIZE;

    if (!abyz_spi_device) {
        dev_err(&spi->dev,"abyz_spi_device is NULL in read\n");
        return -ENODEV;
    }

    mutex_lock(&abyz_spi_device->buf_lock);
    if (abyz_spi_device->cs_gpio != -1) {
	    if (gpio_is_valid(abyz_spi_device->cs_gpio))
		    gpio_set_value(abyz_spi_device->cs_gpio, 0); // Set CS low
    }

    status = abyz_spi_sync_read(abyz_spi_device, length, 1000);

    if (abyz_spi_device->cs_gpio != -1) {
	    if (gpio_is_valid(abyz_spi_device->cs_gpio))
		    gpio_set_value(abyz_spi_device->cs_gpio, 1); // Set CS high
    }

    if (status > 0) {
        unsigned long missing;
        missing = copy_to_user(buffer, abyz_spi_device->rx_buffer, status);
        if (missing == status)
            status = -EFAULT;
        else
            status = status - missing;
    }
    mutex_unlock(&abyz_spi_device->buf_lock);
    return status;
}

static ssize_t device_write(struct file *file, const char __user *buffer, size_t length, loff_t *offset) {
    struct abyz_spi_dev *abyz_spi_device;
    struct spi_device *spi;
    unsigned long missing;
    size_t status;

    abyz_spi_device = file->private_data;
    spi = abyz_spi_device->spi;

    if (length > bufsiz)
        return -EMSGSIZE;

    if (!abyz_spi_device) {
        dev_err(&spi->dev,"abyz_spi_device is NULL in write\n");
        return -ENODEV;
    }

    if (!abyz_spi_device->tx_buffer)
        return -EFAULT;

    dev_dbg(&spi->dev,"[device_write] length=%zu, buffer_size=%u\n", length, bufsiz);

    mutex_lock(&abyz_spi_device->buf_lock);
    if (abyz_spi_device->cs_gpio != -1) {
	    if (gpio_is_valid(abyz_spi_device->cs_gpio))
		    gpio_set_value(abyz_spi_device->cs_gpio, 0); // Set CS low
    }
    memset(abyz_spi_device->tx_buffer,0,bufsiz);
    missing = copy_from_user(abyz_spi_device->tx_buffer, buffer, length);
    if (missing == 0)
        status = abyz_spi_sync_write(abyz_spi_device, length, 1000);
    else
        status = -EFAULT;

    if (abyz_spi_device->cs_gpio != -1) {
	    if (gpio_is_valid(abyz_spi_device->cs_gpio))
		    gpio_set_value(abyz_spi_device->cs_gpio, 1); // Set CS high
    }
    mutex_unlock(&abyz_spi_device->buf_lock);

    return status;
}

static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = device_ioctl,
    .open = device_open,
    .release = device_release,
    .read = device_read,
    .write = device_write,
};

static const struct of_device_id abyz_spi_of_match[] = {
    { .compatible = "abyz,spi-dscam-fpga" },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, abyz_spi_of_match);

static struct spi_driver abyz_spi_driver = {
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = abyz_spi_of_match,
        .owner = THIS_MODULE,
    },
    .probe = abyz_spi_probe,
    .remove = abyz_spi_remove,
};

static int abyz_spi_probe(struct spi_device *spi) {
    struct abyz_spi_dev *abyz_spi_device;
    struct device *dev;
    int err;

    abyz_spi_device = kzalloc(sizeof(*abyz_spi_device), GFP_KERNEL);
    if (!abyz_spi_device) {
        dev_err(&spi->dev, "Failed to allocate memory for Abyz SPI device\n");
        return -ENOMEM;
    }

    abyz_spi_device->spi = spi;
    abyz_spi_device->cs_gpio=-1;
    mutex_init(&abyz_spi_device->buf_lock);
    init_completion(&abyz_spi_device->spi_done);

    cdev_init(&abyz_spi_device->c_dev, &fops);
    abyz_spi_device->c_dev.owner = THIS_MODULE;

    err = cdev_add(&abyz_spi_device->c_dev, dev_num, 1);
    if (err) {
        kfree(abyz_spi_device);
        return err;
    }

    dev = device_create(abyz_spi_class, &spi->dev, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(dev)) {
        cdev_del(&abyz_spi_device->c_dev);
        kfree(abyz_spi_device);
        return PTR_ERR(dev);
    }

    spi_set_drvdata(spi, abyz_spi_device);
    dev_info(&spi->dev, "Abyz SPI Device Probed and ready with major %d and minor %d\n",
             MAJOR(dev_num), MINOR(dev_num));
    return 0;
}

static void abyz_spi_remove(struct spi_device *spi) {
    struct abyz_spi_dev *abyz_spi_device = spi_get_drvdata(spi);

    mutex_lock(&abyz_spi_device->buf_lock);
    abyz_spi_device->spi = NULL;
    mutex_unlock(&abyz_spi_device->buf_lock);

    device_destroy(abyz_spi_class, dev_num);
    cdev_del(&abyz_spi_device->c_dev);
    kfree(abyz_spi_device);

    dev_info(&spi->dev, "Abyz SPI Device Removed\n");

}

static int __init abyz_spi_init(void) {
    int ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        printk(KERN_ERR "Abyz SPI: Failed to allocate character device region\n");
        return ret;
    }

    abyz_spi_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(abyz_spi_class)) {
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(abyz_spi_class);
    }

    return spi_register_driver(&abyz_spi_driver);
}

static void __exit abyz_spi_exit(void) {
    spi_unregister_driver(&abyz_spi_driver);
    class_destroy(abyz_spi_class);
    unregister_chrdev_region(dev_num, 1);
}

module_init(abyz_spi_init);
module_exit(abyz_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mirika74@gmail.com");
MODULE_DESCRIPTION("Abyz DSCAM X-ray Detector FPGA Control SPI  Driver with IOCTL, Mutex, Read, and Write, with Dynamic Debug");
