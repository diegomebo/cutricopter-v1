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

int main(int argc, char **argv)
{
	
	printf("**** CMPS11 example program ****\n");
	
	int fd;														// File descrition
	char *fileName = "/dev/i2c-1";								// Name of the port we will be using
	int  address = 0x68;										// Address of CMPS11 shifted right one bit
	unsigned char buf[10];										// Buffer for data being read/ written on the i2c bus
	
	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}
	
	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}
	
	buf[0] = 22;													// this is the register we wish to read from
	
	if ((write(fd, buf, 1)) != 1) {								// Send register to read from
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		
		short int result = buf[0];
		printf("Bearing as decimal: %d\n",result);
		
	}
	
	buf[0]=22;
	buf[1]=24;
	write(fd,buf,2);
	
	if ((write(fd, buf, 1)) != 1) {								// Send register to read from
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		
		short int result = buf[0];
		printf("Bearing as decimal: %d\n",result);
		
	}
	
	/*
	while(1){
	
	buf[0] = 0x1D;													// this is the register we wish to read from
	
	if ((write(fd, buf, 1)) != 1) {								// Send register to read from
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 6) != 6) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		
		unsigned char highByte = buf[2];
		unsigned char lowByte = buf[3];
		short int result = (highByte <<8) | lowByte;// Calculate the bearing as a word
		printf("Bearing as decimal: %d\n",result);
		
	}
	}
	*/
	return 0;
}
