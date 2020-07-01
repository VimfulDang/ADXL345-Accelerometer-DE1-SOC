#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "../address_map_arm.h"
#include <stdint.h>
#include <signal.h>


#define ADXL345_DEVID				0x00
#define ADXL345_BW_RATE				0x2C
#define ADXL345_POWER_CTL			0x2D
#define ADXL345_THRESH_ACT			0x24
#define ADXL345_THRESH_INACT		0x25
#define ADXL345_TIME_INACT			0x26
#define ADXL345_ACT_INACT_CTL		0x27
#define ADXL345_INT_SOURCE			0x30
#define ADXL345_DATA_FORMAT			0x31

static unsigned int * base_ptr;
static void * i2c0base;
static int fd_i2c0base = -1;

int open_physical(int);
void * map_physical (int, unsigned int, unsigned int);
void close_physical(int);
int unmap_physical(void *, unsigned int);
void mux_init();
int I2C0_Init();
void ADXL345_REG_READ(uint8_t, uint8_t *);
void ADXL345_REG_WRITE(uint8_t, uint8_t );
void ADXL345_REG_MULTI_READ(uint8_t address, uint8_t values[], uint8_t len);
void ADXL345_init();
int ADXL345_IsDataReady();
void ADXL345_XYZ_Read(int16_t szData16[3]);
void ADXL345_IdRead(uint8_t *pId);
int I2C0_onoff(int);

void catchSIGINT (int signum) {
	printf("Unmapping\n");
	mux_init(i2c0base);
	unmap_physical(i2c0base, I2C0_SPAN);
	close_physical(fd_i2c0base);
}

int main(void) {

	void * sysmgrbase; 
	int fd_sysmgr = -1;
	uint8_t devid;
	uint8_t mg_per_lsb = 3.2;
	uint16_t XYZ[3];

	signal(SIGINT, catchSIGINT);
	//Configure MUX to connect I2C0 controller to ADXL345
	if ((fd_sysmgr = open_physical(fd_sysmgr)) == -1) {
		return(-1);
	}

	if ((sysmgrbase = map_physical(fd_sysmgr, SYSMGR_BASE, SYSMGR_SPAN)) == NULL) {
		return(-1);
	}

	printf("%p\n", sysmgrbase);
	printf("Starting mux_init()\n");
	mux_init(sysmgrbase);
	printf("Finished mux_init()\n");

	if ((fd_i2c0base = open_physical(fd_i2c0base)) == -1) {
		return(-1);
	}

	if ((i2c0base = map_physical(fd_i2c0base, I2C0_BASE, I2C0_SPAN)) == NULL) {
		return(-1);
	}

	base_ptr = (unsigned int *) i2c0base;
	printf("Starting I2C0_Init()\n");
	I2C0_Init();
	printf("Getting ID\n");
	ADXL345_REG_READ(ADXL345_DEVID, &devid);
	printf("%#x\n", devid);
	if (devid == 0xE5) {
		printf("Found ADXL345\n");
	}

	//clean up
	unmap_physical(i2c0base, I2C0_SPAN);
	close_physical(fd_i2c0base);
	unmap_physical(sysmgrbase, SYSMGR_SPAN);
	close_physical(fd_sysmgr);

	return 0;
}

int open_physical(int fd) {
	if (fd == -1) {
		if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
			printf("ERROR: could not open \"/dev/mem\"...\n");
			return(-1);
		}
	}

	return fd;
}

void * map_physical(int fd, unsigned int base, unsigned int span) {
	void * virtual_base;

	// Get a mapping from physical addresses to virtual addresses
	virtual_base = mmap(NULL, span, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, base);
	if (virtual_base == NULL) {
		printf("ERROR: mmap() failed...\n");
		close(fd);
		return(NULL);
	}

	return virtual_base;
}

int unmap_physical(void * virtual_base, unsigned int span)
{
   if (munmap (virtual_base, span) != 0)
   {
      printf ("ERROR: munmap() failed...\n");
      return (-1);
   }
   return 0;
}

void close_physical (int fd) {
	close(fd);
}

void mux_init(void * sysmgrbase_ptr) {

	volatile unsigned int *gpio7_ptr, *gpio8_ptr, *i2c0fpga_ptr; //Mux pointer

	gpio7_ptr = (unsigned int *) (sysmgrbase_ptr + (SYSMGR_GENERALIO7 * 4));
	gpio8_ptr = (unsigned int *) (sysmgrbase_ptr + (SYSMGR_GENERALIO8 * 4));
	i2c0fpga_ptr = (unsigned int *) (sysmgrbase_ptr + (SYSMGR_I2C0USEFPGA * 4));

	*gpio7_ptr = 1;
	*gpio8_ptr = 1;
	*i2c0fpga_ptr = 0;
	printf("gpio7_ptr: %#x\ngpio8_ptr: %#x\ni2c0fpga_ptr: %#x\n", *gpio7_ptr, *gpio8_ptr, *i2c0fpga_ptr);
}

int I2C0_onoff(int onoff) {
	int ti2c_poll = 2500;
	int MAX_T_POLL_COUNT = 10;
	int poll_count = 0;

	int good = 0;
	*(base_ptr + (I2C0_ENABLE * 4)) = onoff + 1;

	while (((*(base_ptr + (I2C0_ENABLE_STATUS * 4)) & 0x1) == onoff) & (poll_count < MAX_T_POLL_COUNT)) {
		poll_count++;
		usleep(ti2c_poll);
		*(base_ptr + (I2C0_ENABLE * 4)) = onoff + 1;
	}
	if (poll_count < 10)
		good = 1;

	return good;
}

int I2C0_Init() {

	//Abort tranmission and disable Controller for config
	*(base_ptr + (I2C0_ENABLE * 4)) = 0x2;
	//Wait for disable status

	if (!(I2C0_onoff(1))) {
		printf("Unable to disable\n");
		return 0;
	}
	printf("Configuring\n");

	// 7-Bit addressing, FastMode (400kb/s), Master Mode
	*(base_ptr + (I2C0_CON * 4)) = 0x65;

	//Set target address to be ADXL345 devid 0x53 and 7bit addressing
	*(base_ptr + (I2C0_TAR * 4)) = 0x53;

	//Minimum period is to be 2.5us but minimum high period is 0.6us
	//and minimum low period is 1.3us so 0.3us is added to both.
	*(base_ptr + (I2C0_FS_SCL_HCNT * 4)) = 60 + 30;
	*(base_ptr + (I2C0_FS_SCL_LCNT * 4)) = 130 + 30;

	//Enable the controller
	*(base_ptr + (I2C0_ENABLE * 4)) = 0x1;
	printf("Renabling controller\n");
	if (!(I2C0_onoff(0))) {
		printf("Unable to enable\n");
		return 0;
	}
	printf("%#x\n", *(base_ptr + (I2C0_ENABLE_STATUS * 4)));
	return 1;
}

void ADXL345_REG_READ(uint8_t address, uint8_t * value) {

	//Send address and start signal
	*(base_ptr + (I2C0_DATA_CMD * 4)) = address + 0x400;

	//send read signal
	*(base_ptr + (I2C0_DATA_CMD * 4)) = 0x100;

	//wait for response
	while(*(base_ptr + (I2C0_RXFLR * 4)) == 0) {
		usleep(500000);
		*(base_ptr + (I2C0_DATA_CMD * 4)) = 0x100;
		printf("%#x\n", *(base_ptr + (I2C0_RXFLR * 4)));
	}

	*value = *(base_ptr + (I2C0_DATA_CMD * 4)) | 0xFF;
}

void ADXL345_REG_WRITE(uint8_t address, uint8_t value) {

	*(base_ptr + (I2C0_DATA_CMD * 4)) = address + 0x400;

	*(base_ptr + (I2C0_DATA_CMD * 4)) = value;
}

void ADXL345_REG_MULTI_READ(uint8_t address, uint8_t values[], uint8_t len) {

	int i = 0;
	int nth_byte = 0;
	*(base_ptr + (I2C0_DATA_CMD * 4)) = address + 0x400;

	//send read signal multiple times to prevent overwritten data at 
	//inconsistent times

	for(i = 0; i < len; i++)
		*(base_ptr + (I2C0_DATA_CMD * 4)) = 0x100;

	while(len) {
		if (*(base_ptr + (I2C0_RXFLR * 4)) > 0) {
			values[nth_byte] = *(base_ptr + (I2C0_DATA_CMD * 4)) | 0xFF;
			nth_byte++;
			len--;
		}
	}
}

void ADXL345_init() {

	uint8_t data_format = 0x03;
	uint8_t bw_rate = 0x07;

	//+-16 range, 10 bits
	ADXL345_REG_WRITE(ADXL345_DATA_FORMAT, data_format);
	ADXL345_REG_WRITE(ADXL345_BW_RATE, bw_rate);

	//Using threshold for new data
	ADXL345_REG_WRITE(ADXL345_THRESH_ACT, 0x04);
	ADXL345_REG_WRITE(ADXL345_THRESH_INACT, 0x02);
	ADXL345_REG_WRITE(ADXL345_TIME_INACT, 0x02);
	ADXL345_REG_WRITE(ADXL345_ACT_INACT_CTL, 0xFF);

	//Reset Measurement config
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x00); //standby
	ADXL345_REG_WRITE(ADXL345_POWER_CTL, 0x08);	
}

int ADXL345_IsDataReady() {
	int bReady = 0;
	uint8_t data8;

	ADXL345_REG_READ(ADXL345_INT_SOURCE, &data8);
	if (data8 & 0x08)
		bReady = 1;

	return bReady;
}

//Read acceleration data of all three axes

void ADXL345_XYZ_Read(int16_t szData16[3]) {
	uint8_t szData8[6];
	ADXL345_REG_MULTI_READ(0x32, (uint8_t *) &szData8, sizeof(szData8));

	szData16[0] = (szData8[1] << 8) | szData8[0];
	szData16[1] = (szData8[3] << 8) | szData8[2];
	szData16[2] = (szData8[5] << 8) | szData8[4];
}

void ADXL345_IdRead(uint8_t *pId) {
	ADXL345_REG_READ(ADXL345_DEVID, pId);
}
