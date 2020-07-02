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
#define ROUNDED_DIVISION(n, d) (((n<0) ^ (d < 0)) ? ((n- d/2)/d) : ((n+d/2)/d))

/* Kernel Character Device Driver /dev/accel */
static int device_open (struct inode * inode, struct file * file);
static int device_release (struct inode * inode, struct file * filp);
static ssize_t accel_read (struct file * filp, char * buffer, size_t length, loff_t *offset);
static ssize_t accel_write (struct file * filp, const char * buffer, size_t length, loff_t *offset);
static int __init init_accel(void);
static void __exit stop_accel(void);

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
static void ADXL345_Calibrate(void);
static void ADXL345_updateFormat(char command[], int len);
static void ADXL345_updateRate(char command[], int len);
static int get_command(char * arr);
static void ADXL345_Calibrate(void);

/* Character Kernel Variables */
static dev_t accel_no = 0;
static struct cdev * accel_cdev = NULL;
static struct class * accel_class = NULL;

static struct file_operations accel_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.release = device_release,
	.read = accel_read,
	.write = accel_write
};

/* Module Variables */
static volatile int * I2C0_ptr, * SYSMGR_ptr;
static u8 devid;
static u8 mg_per_lsb = 3;
static u16 XYZ[3];
static char reg_read[256], reg_write[256];
static s16 new_data, xmg, ymg, zmg;
static unsigned int ind_read = 0, ind_write = 0;
static unsigned int write_Empty = 0, calibrate = 0;
static char * commands[5] = {"device", "init", "calibrate", "format", "rate"};

static int __init init_accel(void) {

	int err = 0;
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
	
	if (!ind_write && write_Empty) {
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
		sprintf(reg_write, "%d %d %d %d %d\n", new_data, xmg, ymg, zmg, mg_per_lsb);
	}
	
	if ((reg_write[ind_write] != '\0') && length) {
		put_user(reg_write[ind_write++], buffer++);
		length--;
	}
	else {
		write_Empty = 1;
		ind_write = 0;
	}
	return ind_write;
}

/* Check against commands[] */
static int get_command(char * arr) {
	int i = 0;
	int command_ind = -1;
	//Check for commands
	for( i = 0; i < 5; i++) {
		if (!(strcmp(commands[i], arr))) {
			command_ind = i;
			i = 6;
		}
	}
	return command_ind;
}

static ssize_t accel_write (struct file * filp, const char * buffer, size_t length, loff_t *offset) {
	int command = 10;
	char commandStr[32];
	int i = 0;
	//Read buffer input
	if (length < 256) {
		for(ind_read = 0; ind_read < length-1; ind_read++) {
			reg_read[ind_read] = buffer[ind_read];
		}
		reg_read[ind_read++] = '\0';
	}
	//Seperate Command
	for(i = 0; i < 32; i++) {
		if ((reg_read[i] != ' ') && reg_read[i] != '\0') {
			commandStr[i] = reg_read[i];
		}
		else {
			commandStr[i] = '\0';
			i = 33;
		}
	}
	if (i == 32) 
		commandStr[--i] = '\0';
	//Get command
	command = get_command(commandStr);
	switch (command) {

		case 0 :
				printk("device\n");
				printk(KERN_INFO "%#x\n", devid);
				sprintf(reg_write, "%#x\n", devid);
				write_Empty = 0;
				break;
		case 1 : 
				printk("init\n");
				ADXL345_Init();
				break;
		case 2 : 
				printk("calibrate\n");
				calibrate = 1;
				ADXL345_Calibrate();
				calibrate = 0;
				break;
		case 3 : 
				printk("format\n");
				ADXL345_updateFormat(reg_read, ind_read);
				break;
		case 4 : 
				printk("rate\n");
				ADXL345_updateRate(reg_read, ind_read);
				break;
		default : printk("Default: Not a valid command\n");
	}
	//printk("Command: %d, length: %zu, reg_write: %d\nreg_write: %s", command, length, ind_write, reg_write);
	return ind_read;
}

static void ADXL345_updateFormat(char command[], int len) {
	int fvalue, gvalue;
	u8 range = -1;
	u8 format, oldFormat, newFormat;
	char fstr[2];
	char gstr[4];
	int i = 0;
	int k = 0;
	//Get rid of 'format'
	while (command[i++] != ' ') {
		continue;
	}
	// Get resolution value
	fstr[0] = command[i++];
	fstr[1] = '\0';
	i++; //get rid of space
	kstrtouint(fstr, 10, &fvalue);
	if (fvalue < 2) {
		for (k = 0; k < 4; k++) {
			if ((command[i] != ' ') && (command[i] != '\0')) {
				gstr[k] = command[i++];
			}
			else {
				gstr[k] = '\0';
				k = 5;
			}
		}
		if (k == 4) 
			gstr[--k] = '\0';
		kstrtoint(gstr, 10, &gvalue);
		gvalue = gvalue * gvalue;
		switch (gvalue) {
			case (4) : range = 0x0;
						break; 
			case (16) : range = 0x1;
						break;
			case (64) : range = 0x2;
						break;
			case (256) : range = 0x3;
						break;
			default : printk("Invalid Resolution Value. Aborting command\n");
		}

		if (range > -1) {
			format = (fvalue << 3) | range;
			printk("fstr: %s, gstr: %s, range: %d\n", fstr, gstr, range);
			ADXL345_REG_READ(ADXL345_DATA_FORMAT, &oldFormat);
			ADXL345_REG_WRITE(ADXL345_DATA_FORMAT, format);
			ADXL345_REG_READ(ADXL345_DATA_FORMAT, &newFormat);
			printk("oldFormat: %#x, newFormat: %#x\n", oldFormat, newFormat);
		}
	}
	else 
		printk("Invalid Resolution Value. Aborting command\n");
}


static void ADXL345_updateRate(char command[], int len) {
	int i = 0, k;
	unsigned int rate;
	u8 newRate, oldRate;
	char rateStr[10];

	//Get rid of 'rate'
	while (command[i++] != ' ') {
		continue;
	}
	for(k = 0; k < 10; k++) {
		if (command[i] != ' ' && command[i] != '\0') {
			rateStr[k] = command[i++];
		}
		else {
			rateStr[k] = '\0';
			k = 11;
		}
	}
	if (k == 10) {
		rateStr[--k] = '\0';
	}
	kstrtouint(rateStr, 10, &rate);
	if (rate >= 0 && rate <= 15) {
		ADXL345_REG_READ(ADXL345_BW_RATE, &oldRate);
		ADXL345_REG_WRITE(ADXL345_BW_RATE, (u8) rate);
		ADXL345_REG_READ(ADXL345_BW_RATE, &newRate);
		printk("oldRate: %#x, newRate: %#x\n", oldRate, newRate);
	}
	else {
		printk("Invalid rate. Try a value from 0 to 15 "
			"for 0.098 Hz to 3600 Hz.\nEach decrement will halves the value\n"
			"For Example:\n15 for 3600 Hz\n14 for 1600 Hz\n10 for 100Hz\n4 for 1.563 Hz\n");
	}

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
	if ((data8 & ADXL345_ACTIVITY) | calibrate) {
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

/* Courtesy Altera Open Source FPGA Driver */
static void ADXL345_Calibrate(void) {
	int average_x = 0;
	int average_y = 0;
	int average_z = 0;
	int i = 0;
	s16 XYZ_cal[3];
	s8 offset_x, offset_y, offset_z; 
	u8 saved_bw, saved_dataformat;

	//stop measure
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x00);

	//get current offsets
	ADXL345_REG_READ(ADXL345_REG_OFSX, (u8 *) &offset_x);
	ADXL345_REG_READ(ADXL345_REG_OFSY, (u8 *) &offset_y);
	ADXL345_REG_READ(ADXL345_REG_OFSZ, (u8 *) &offset_z);

	//use 100Hz rate for calibrate. Save the current rate.
	ADXL345_REG_READ(ADXL345_BW_RATE, &saved_bw);
	ADXL345_REG_WRITE(ADXL345_BW_RATE, 0xA);

	//use 16g range, full resolution. Save the current format.
	ADXL345_REG_READ(ADXL345_DATA_FORMAT, &saved_dataformat);
	ADXL345_REG_WRITE(ADXL345_DATA_FORMAT, 0xB);

	// Start measure
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x08);

	while (i < 32) {
		//Note: use DATA_READY here, can't use acitivty because board is stationary.
		if (ADXL345_IsDataReady()) {
			ADXL345_XYZ_Read(XYZ_cal);
			average_x += XYZ_cal[0];
			average_y += XYZ_cal[1];
			average_z += XYZ_cal[2];
			i++;
		}	
	}

	average_x = ROUNDED_DIVISION(average_x, 32);
	average_y = ROUNDED_DIVISION(average_y, 32);
	average_z = ROUNDED_DIVISION(average_z, 32);

	//stop measure
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x00);

	printk("Average X=%d, Y=%d, Z=%d\n", average_x, average_y, average_z);

	//Calculate the offsets (LSB 15.6mg)

	offset_x += ROUNDED_DIVISION(0-average_x, 4);
	offset_y += ROUNDED_DIVISION(0-average_y, 4);
	offset_z += ROUNDED_DIVISION(256-average_z, 4);

	printk("Calibration: Offset_x: %d, offset_y: %d, offset_z: %d (LSB: 15.6 mg)\n", offset_x, offset_y, offset_z);

	//set the offset register
	ADXL345_REG_WRITE(ADXL345_REG_OFSX, offset_x);
	ADXL345_REG_WRITE(ADXL345_REG_OFSY, offset_y);
	ADXL345_REG_WRITE(ADXL345_REG_OFSZ, offset_z);

	//restore original bw rate
	ADXL345_REG_WRITE(ADXL345_BW_RATE, saved_bw);

	//restore original data format
	ADXL345_REG_WRITE(ADXL345_DATA_FORMAT, saved_dataformat);

	//Start Measure
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x08);
}

module_init (init_accel);
module_exit (stop_accel);
