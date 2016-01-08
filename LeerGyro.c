#define AdressGyro 0x68
#define AdressGyroX 0x1D
void leergyro(short int* Gyro){
	unsigned char buffer[7];
	readFromI2C(AdressGyro,AdressGyroX,6,buffer);
	Gyro[0]=(short int)buffer[0]<<8 | (short int)buffer[1];
	Gyro[1]=(short int)buffer[2]<<8 | (short int)buffer[3];
	Gyro[2]=(short int)buffer[4]<<8 | (short int)buffer[5];
}