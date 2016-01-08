#ifndef EASY_I2C
#def EASY_I2C

void readFromI2C(int device, unsigned char address, int number_of_bytes, unsigned char* result){	
	write(device,&address,1);
	read(device,result,number_of_bytes);	
}

void writeToI2C(int device, unsigned char address, unsigned char data){
	unsigned char buffer[2];
	buffer[0]=address;
	buffer[1]=data;
	write(device,buffer,2);
}



#endif
