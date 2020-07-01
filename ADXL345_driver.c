#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "../address_map_arm.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dang Nguyen");
MODULE_DESCRIPTION("DE1SoC ADXL345 Accelerometer");
MODULE_VERSION("0.01");

#define SUCCESS 0
#define DEVICE_NAME "accel"

/* Kernel Character Device Driver /dev/accel */
static int device_open (struct inode * inode, struct file * file);
static int device_release (struct inode * inode, struct file * filp);
static ssize_t accel_read (struct file * filp, char * buffer, size_t length, loff_t *offset);
static int __init init_accel(void);
static void __exit stop_accel(void);

//static ssize_t accel_write (struct file * filp, const char * buffer, size_t length, loff_t *offset);

/* Module Functions Prototype */
static void mux_init(void);
static int I2C0_Init(void);
static void ADXL345_Init(void);
static int I2C0_OnOff(unsigned int onoff);
static void ADXL345_IdRead(u8 *pId);
static void ADXL345_REG_READ(u8 address, u8 * value);
static void ADXL345_REG_WRITE(u8 address, u8 value);
static void ADXL345_REG_MULTI_READ(u8 address, u8 values[], u8 len);
static int ADXL345_IsDataReady(void);
static void ADXL345_XYZ_Read(s16 szData16[3]);

/* Character Kernel Variables */
static dev_t accel_no = 0;
static struct cdev * accel_cdev = NULL;
static struct class * accel_class = NULL;

static struct file_operations accel_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.release = device_release,
	.read = accel_read,
	//.write = accel_write
};

/* Module Variables */
static volatile int * I2C0_ptr, * SYSMGR_ptr;
static u8 mg_per_lsb = 3;
static u16 XYZ[3];
static char reg_read[32];
static s16 new_data, xmg, ymg, zmg, ind_read = 0;

static int __init init_accel(void) {

	int err = 0;
	u8 devid;

	//Register char device in a range of no
	if ((err = alloc_chrdev_region (&accel_no, 0, 1, DEVICE_NAME)) < 0) {
		printk(KERN_ERR "chardev: alloc_chrdev_region() error %d\n", err);
		return err;
	}

	accel_class = class_create(THIS_MODULE, DEVICE_NAME);
	accel_cdev = cdev_alloc();
	accel_cdev->ops = &accel_fops;
	accel_cdev->owner = THIS_MODULE;

	if ((err = cdev_add(accel_cdev, accel_no, 1)) < 0) {
		printk(KERN_ERR "chardev: cdev_add() error %d\n", err);
		return err;
	}

	device_create(accel_class, NULL, accel_no, NULL, DEVICE_NAME);

	//Accelerometer Initialization
	printk("Initializing Accelerometer\n");
	I2C0_ptr = ioremap_nocache(I2C0_BASE, I2C0_SPAN);
	SYSMGR_ptr = ioremap_nocache(SYSMGR_BASE, SYSMGR_SPAN);

	if ((I2C0_ptr == 0) || (SYSMGR_ptr == NULL)) 
		printk (KERN_ERR "Error: ioremap_nocache returned NULL\n");

	mux_init();
	I2C0_Init();
	ADXL345_Init();

	ADXL345_IdRead(&devid);
	if (devid == 0xE5)
		printk("Found ADXL345\n");
	
	return 0;
}

static void __exit stop_accel(void) {

	iounmap(I2C0_ptr);
	iounmap (SYSMGR_ptr);
	device_destroy(accel_class, accel_no);
	cdev_del(accel_cdev);
	class_destroy(accel_class);
	unregister_chrdev_region(accel_no, 1);

}


static int device_open(struct inode * inode, struct file * file) {
	return SUCCESS;
}

static int device_release(struct inode * inode, struct file * file) {
	return 0;
}

//Returns New XX YY ZZ SS, SS = Scaling Factor
static ssize_t accel_read (struct file * filp, char * buffer, size_t length, loff_t *offset) {
	
	if (!ind_read) {
		if(ADXL345_IsDataReady()) {
			ADXL345_XYZ_Read(XYZ);
			xmg = XYZ[0]*mg_per_lsb;
			ymg = XYZ[1]*mg_per_lsb; 
			zmg = XYZ[2]*mg_per_lsb;
			new_data = 1;
		}
		else {
			new_data = 0;
		}
		sprintf(reg_read, "%d %d %d %d %d\n", new_data, xmg, ymg, zmg, mg_per_lsb);
	}
	
	if ((reg_read[ind_read] != '\0') && length) {
		put_user(reg_read[ind_read++], buffer++);
		length--;
	}
	else
		ind_read = 0;
	return ind_read;
}

static void mux_init(void) {

	volatile unsigned int *gpio7_ptr, *gpio8_ptr, *i2c0fpga_ptr; //Mux pointer

	gpio7_ptr = SYSMGR_ptr + SYSMGR_GENERALIO7;
	gpio8_ptr = SYSMGR_ptr + SYSMGR_GENERALIO8;
	i2c0fpga_ptr =  SYSMGR_ptr + SYSMGR_I2C0USEFPGA;

	*i2c0fpga_ptr = 0;
	*gpio7_ptr = 1;
	*gpio8_ptr = 1;
	printk("gio7: %#x\ngio8: %#x\ni2c0fpga: %#x\n", *gpio7_ptr, *gpio8_ptr, *i2c0fpga_ptr);
}

static int I2C0_Init(void) {

	//Abort tranmission and disable Controller for config
	printk("%d\n%d\n", I2C0_TAR, I2C0_FS_SCL_HCNT * 1);
	printk("%p\n%p\n", I2C0_ptr,  I2C0_ptr + (I2C0_FS_SCL_HCNT * 1));
	*(I2C0_ptr + I2C0_ENABLE) = 0x2;
	//Wait for disable status

	if (!(I2C0_OnOff(2))) {
		printk("Unable to disable\n");
		return 0;
	}
	printk("Configuring\n");

	// 7-Bit addressing, FastMode (400kb/s), Master Mode
	*(I2C0_ptr + I2C0_CON) = 0x65;

	//Set target address to be ADXL345 devid 0x53 and 7bit addressing
	*(I2C0_ptr + I2C0_TAR) = 0x53;

	//Minimum period is to be 2.5us but minimum high period is 0.6us
	//and minimum low period is 1.3us so 0.3us is added to both.
	*(I2C0_ptr + I2C0_FS_SCL_HCNT) = 60 + 30;
	*(I2C0_ptr + I2C0_FS_SCL_LCNT) = 130 + 30;

	//Enable the controller
	printk("Renabling controller\n");
	if (!(I2C0_OnOff(1))) {
		printk("Unable to enable\n");
		return 0;
	}
	printk("ic_tar: %#x\nic_con: %#x\n", *(I2C0_ptr + I2C0_TAR), *(I2C0_ptr + I2C0_CON));
	return 1;
}

void ADXL345_Init(void) {

	u8 data_format = 0x03;
	u8 bw_rate = 0x07;

	//+-16 range, 10 bits
	ADXL345_REG_WRITE(ADXL345_DATA_FORMAT, data_format);
	ADXL345_REG_WRITE(ADXL345_BW_RATE, bw_rate);

	//Using threshold for new data
	ADXL345_REG_WRITE(ADXL345_THRESH_ACT, 0x04);
	ADXL345_REG_WRITE(ADXL345_THRESH_INACT, 0x02);
	ADXL345_REG_WRITE(ADXL345_TIME_INACT, 0x02);
	ADXL345_REG_WRITE(ADXL345_ACT_INACT_CTL, 0xFF);
	ADXL345_REG_WRITE(ADXL345_INT_ENABLE, 0x18);

	//Reset Measurement config
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x00); //standby
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x08);	
}

static int I2C0_OnOff(unsigned int onoff) {
	int ti2c_poll = 1;
	int MAX_T_POLL_COUNT = 100;
	int poll_count = 0;

	int good = 0;
	*(I2C0_ptr + I2C0_ENABLE) = onoff;

	while (((*(I2C0_ptr + I2C0_ENABLE_STATUS) & 0x1) == (onoff - 1)) & (poll_count < MAX_T_POLL_COUNT)) {
		poll_count++;
		msleep(ti2c_poll);
		*(I2C0_ptr + I2C0_ENABLE) = onoff;
	}
	if (poll_count < 10)
		good = 1;
	else
		printk("Unable to proceed with 1: Off 2: On --> %d\n", onoff);

	return good;
}

/* Single Byte Read */
static void ADXL345_REG_READ(u8 address, u8 * value) {

	//Send address and start signal
	*(I2C0_ptr + I2C0_DATA_CMD) = address + 0x400;

	//send read signal
	*(I2C0_ptr + I2C0_DATA_CMD) = 0x100;

	//wait for response

	while(*(I2C0_ptr + I2C0_RXFLR) == 0) {
		continue;
	}
	*value = *(I2C0_ptr + I2C0_DATA_CMD) & 0xFF;
}

/* Single byte Write */
static void ADXL345_REG_WRITE(u8 address, u8 value) {

	*(I2C0_ptr + I2C0_DATA_CMD) = address + 0x400;
	*(I2C0_ptr + I2C0_DATA_CMD) = value;
}

/* Multiple Byte Write */
static void ADXL345_REG_MULTI_READ(u8 address, u8 values[], u8 len) {

	int i = 0;
	int nth_byte = 0;
	*(I2C0_ptr + I2C0_DATA_CMD) = address + 0x400;

	//send read signal multiple times to prevent overwritten data at 
	//inconsistent times

	for(i = 0; i < len; i++)
		*(I2C0_ptr + I2C0_DATA_CMD) = 0x100;

	while(len) {
		if (*(I2C0_ptr + I2C0_RXFLR) > 0) {
			values[nth_byte] = *(I2C0_ptr + I2C0_DATA_CMD) & 0xFF;
			nth_byte++;
			len--;
		}
	}
}

static int ADXL345_IsDataReady(void) {
	int bReady = 0;
	u8 data8;

	ADXL345_REG_READ(ADXL345_INT_SOURCE, &data8);
	if (data8 & ADXL345_ACTIVITY) {
		ADXL345_REG_READ(ADXL345_INT_SOURCE, &data8);
		if (data8 & ADXL345_DATAREADY) {
			bReady = 1;
		}
	}
	return bReady;
}

static void ADXL345_XYZ_Read(s16 szData16[3]) {
	u8 szData8[6];
	ADXL345_REG_MULTI_READ(0x32, (u8 *) &szData8, sizeof(szData8));

	szData16[0] = (szData8[1] << 8) | szData8[0];
	szData16[1] = (szData8[3] << 8) | szData8[2];
	szData16[2] = (szData8[5] << 8) | szData8[4];
}


static void ADXL345_IdRead(u8 *pId) {
	ADXL345_REG_READ(ADXL345_DEVID, pId);
}

module_init (init_accel);
module_exit (stop_accel);
