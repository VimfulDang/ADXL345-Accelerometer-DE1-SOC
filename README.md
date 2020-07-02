# ADXL345-Accelerometer-DE1-SOC

ADXL345_driver.c

The driver must create the file /dev/accel in the Linux filesystem. A read of this file should return accelerometer  
data in the format R XXXX YYYY ZZZZ SS, where R is 1 if new accelerometer data is being provided, XXXX, YYYY,   
and ZZZZ are acceleration data in the x, y, and z axes, and SS is the scale factor in mg/LSB for the acceleration data.   
As an example, if the ADXL345 has new data to report,  then a read of the file might return: "1 0 -1 32 31",   
which would represent 0 mg acceleration in the x axis, -31mg acceleration in the y axis, and 992 mg acceleration   
in the z axis. If you were to perform another read from /dev/accel immediately, then the device might not be ready   
to provide new data; it would then respond with "0 0 -1 32 31", indicating old data.  

Added new functionality through writing to the driver:
"device" to retrieve device ID  
"init" to re-initialize   
"calibrate" ....    
"format -f -g" will change the resolution between 13bits and 10bits. And +- 2/4/8/16g. "format 1 +16" will result in 13bits resolution, where LSB is 3.9mg  
"rate -x" will change the sampling rate from 0.098 Hz to 3200 Hz with values -x from 0 to 15. Each decrement will halves the sampling rate such as 14 will be 1600 Hz.  

ADXL345_user.c  
Developing ADXL345 driver in user space by mapping hardware addresses to virtual addresses using /dev/mem and mmap(). The driver configures the sensor to 10 bits resolution at 12.5 Hz. 
