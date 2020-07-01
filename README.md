# ADXL345-Accelerometer-DE1-SOC

ADXL345_driver.c

The driver must create the file /dev/accel in the Linux filesystem. A read of this file should return accelerometer 
data in the format R XXXX YYYY ZZZZ SS, where R is 1 if new accelerometer data is being provided, XXXX, YYYY, 
and ZZZZ are acceleration data in the x, y, and z axes, and SS is the scale factor in mg/LSB for the acceleration data. 
As an example, if the ADXL345 has new data to report,  then a read of the file might return: "1 0 -1 32 31", 
which would represent 0 mg acceleration in the x axis, -31mg acceleration in the y axis, and 992 mg acceleration 
in the z axis. If you were to perform another read from /dev/accel immediately, then the device might not be ready 
to provide new data; it would then respond with "0 0 -1 32 31", indicating old data.

ADXL345_user.c
Developing ADXL345 driver in user space by mapping hardware addresses to virtual addresses using /dev/mem and mmap(). The driver configures the sensor to 10 bits resolution at 12.5 Hz. 
