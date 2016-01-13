// CMPS11 example c code for Raspberry pi.
//
// Reads the software version, bearing pitch and roll form
// the CMPS11 and displays them on the screen.
//
// By James Henderson, 2012

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>

#define AdressGyro 0x68
#define AdressGyroX 0x1D
#define AdressAccel 0x53
#define AdressAccelX 0x32

void readGyro(short int* Gyro);
float convertGyro(short int raw);
float convertAccel(short int raw);
void readFromI2C(int device, unsigned char address, int number_of_bytes, unsigned char* result);
void writeToI2C(int device, unsigned char address, unsigned char data);
void readAccel(short int* Accel);

int fd;	

int main(int argc, char **argv)
{
	
	printf("**** CMPS11 example program ****\n");
	
														// File descrition
	char *fileName = "/dev/i2c-1";								// Name of the port we will be using
									// Address of CMPS11 shifted right one bit
										// Buffer for data being read/ written on the i2c bus
	short int gyro_raw[3];
	short int accel_raw[3];
	
	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}	
	struct timespec tNow;
	writeToI2C(AdressAccel,0x2D,0x08);
	
	writeToI2C(AdressAccel,0x31,0x0B);
	
		long t;
	while(1){
		readGyro(gyro_raw);
		readAccel(accel_raw);
		clock_gettime(CLOCK_REALTIME,&tNow);
	
		t=tNow.tv_nsec;
	
		printf("%ld\n",t);
	}
}

void readFromI2C(int device, unsigned char address, int number_of_bytes, unsigned char* result){	
	
	if (ioctl(fd, I2C_SLAVE, device) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}
	
	write(fd,&address,1);
	read(fd,result,number_of_bytes);	
}

void writeToI2C(int device, unsigned char address, unsigned char data){
	
	if (ioctl(fd, I2C_SLAVE, device) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}	
	unsigned char buffer[2];
	buffer[0]=address;
	buffer[1]=data;
	write(fd,buffer,2);
}

void readGyro(short int* Gyro){
	unsigned char buffer[7];
	readFromI2C(AdressGyro,AdressGyroX,6,buffer);
	Gyro[0]=(short int)buffer[0]<<8 | (short int)buffer[1];
	Gyro[1]=(short int)buffer[2]<<8 | (short int)buffer[3];
	Gyro[2]=(short int)buffer[4]<<8 | (short int)buffer[5];
}

void readAccel(short int* Accel){
	unsigned char buffer[7];
	readFromI2C(AdressAccel,AdressAccelX,6,buffer);
	Accel[0]=(short int)buffer[0]<<8 | (short int)buffer[1];
	Accel[1]=(short int)buffer[2]<<8 | (short int)buffer[3];
	Accel[2]=(short int)buffer[4]<<8 | (short int)buffer[5];
}


float convertGyro(short int raw){
	return raw * 0.069565;
}

float convertAccel(short int raw){
	return raw / 256;
}

void crossProduct(float* v1, float* v2, float* result){	
  result[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
  result[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
  result[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

void getAccelAngles(float* accel, float* result){
	result[0] = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	float temp1[3];
	float temp2[3];
	float X[3] = {1, 0, 0};
	crossProduct(accel,X,temp1);
	crossProduct(X,temp1,temp2);
	result[1] =  atan2(temp2[1], temp2[2]);
}

