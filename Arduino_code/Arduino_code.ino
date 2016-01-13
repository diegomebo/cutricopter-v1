#include <Wire.h>

// Gyro
//
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define SCALING_FACTOR 13     // Scaling factor - used when converting to angle

// Accelerometer
//
#define ADXL345_ADDR (0x53)  // The adress of ADXL345 
#define ADXL345_X_ADDR (0x32)// Start address for x-axis
#define ACC_SENS 256         // Sensitivity. 13 bit adc, +/- 16 g. Calculated as: (2^13)/(16*2)
#define ASSUME_1G_ACC 0      // Assuming the total gravitation is 1. True if only earth gravitation has influence.  


// Magnetometer
//
#define HMC_ADDR 0x1E        // The address of HMC5883
#define HMC_X_ADDR (0x03)    // Start address for x-axis. 
#define SAMPLERATE 128       // Samplerate of sensors (in hz, samples per second)


#define DEBUG

unsigned char ADXL345_ID = 0;
unsigned char ITG3205_ID = 0;
unsigned char HMC_ID = 0;

byte sensorBuffer[10];
long accRaw[3];              // Raw readings from accelerometer
float accG[3];               // G-force in each direction
float accAngle[3];           // Measured angle from accelerometer
float R;                     // Unit vector - total G.

float gyroRaw[3];            // Raw readings from gyro
float angle[3];              // Angle from gyro
float angleRaw[3];           // Temp for angle-calculation

float magRaw[3];             // Raw readings from magnetometer
float magAngle[3];           // Measured angles from magnetometer
float mx = 0;                // Calculated magnetometer value in x-direction with pan/tilt compensation
float my = 0;                // Calculated magnetometer value in y-direction with pan/tilt compensation

long accOffset[3] = {0, 0, 0};
float gyroOff[3] = {0, 0, 0};

void setup() {
  Wire.begin();
  InitSensors();
  Calibrate();
  Serial.begin(9600);

}

void loop() {

  UpdateSensors();
  AccelCalc();

#ifdef DEBUG
  Serial.print("ACEL:");
  Serial.print(accG[0]);
  Serial.print(",");
  Serial.print(accG[1]);
  Serial.print(",");
  Serial.println(accG[2]);

  
  Serial.print("GIRO:");
  Serial.print(gyroRaw[0]);
  Serial.print(",");
  Serial.print(gyroRaw[1]);
  Serial.print(",");
  Serial.println(gyroRaw[2]);
  

  Serial.print("MAG:");
  Serial.print(magRaw[0]);
  Serial.print(",");
  Serial.print(magRaw[1]);
  Serial.print(",");
  Serial.println(magRaw[2]);
  
  delay(100);

#endif

}


void InitSensors()
{
  ReadFromI2C(ITG3205_ADDR, 0x00, 1);
  ITG3205_ID = sensorBuffer[0];

  ReadFromI2C(ADXL345_ADDR, 0x00, 1);
  ADXL345_ID = sensorBuffer[0];

  // Accelerometer increase G-range (+/- 16G)
  WriteToI2C(ADXL345_ADDR, 0x31, 0b00001011);
  ReadFromI2C(HMC_ADDR, 0x00, 1);
  HMC_ID = sensorBuffer[0];

  WriteToI2C(ITG3205_ADDR, 22, 24);

  //  ADXL345 POWER_CTL
  WriteToI2C(ADXL345_ADDR, 0x2D, 0);
  WriteToI2C(ADXL345_ADDR, 0x2D, 16);
  WriteToI2C(ADXL345_ADDR, 0x2D, 8);

  // HMC5883
  // Run in continuous mode
  WriteToI2C(HMC_ADDR, 0x02, 0x00);
}



// Function used to write to I2C:
void WriteToI2C(int device, byte address, byte val)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

// Function to read from I2C
void ReadFromI2C(int device, char address, char bytesToRead)
{
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(device);
  Wire.requestFrom(device, bytesToRead);

  char i = 0;
  while ( Wire.available() )
  {
    sensorBuffer[i++] = Wire.read();
  }
  Wire.endTransmission();
}

void UpdateSensors()
{

  // Read x, y, z acceleration, pack the data.
  ReadFromI2C(ADXL345_ADDR, ADXL345_X_ADDR, 6);
  accRaw[0] = ((int)sensorBuffer[0] | ((int)sensorBuffer[1] << 8)) * -1;
  accRaw[1] = ((int)sensorBuffer[2] | ((int)sensorBuffer[3] << 8)) * -1;
  accRaw[2] = (int)sensorBuffer[4] | ((int)sensorBuffer[5] << 8);


  // Read x, y, z from gyro, pack the data
  ReadFromI2C(ITG3205_ADDR, ITG3205_X_ADDR, 6);
  gyroRaw[0] = (int)sensorBuffer[1] | ((int)sensorBuffer[0] << 8);
  gyroRaw[1] = ( (int)sensorBuffer[3] | ((int)sensorBuffer[2] << 8) ) * -1;
  gyroRaw[2] = ( (int)sensorBuffer[5] | ((int)sensorBuffer[4] << 8) ) * -1;


  // Read x, y, z from magnetometer;
  ReadFromI2C(HMC_ADDR, HMC_X_ADDR, 6);
  for (unsigned char i = 0; i < 3; i++)
  {
    magRaw[i] = (int)sensorBuffer[(i * 2) + 1] | ((int)sensorBuffer[i * 2] << 8);
  }
}

void AccelCalc() {
  accG[0] = (float)accRaw[0] - accOffset[0];
  accG[1] = (float)accRaw[1] - accOffset[1];
  accG[2] = (float)accRaw[2] - accOffset[2];
}

void Calibrate() {
  delay(1000);

  accOffset[0] = 0;
  accOffset[1] = 0;
  accOffset[2] = 0;

  gyroOff[0] = 0;
  gyroOff[1] = 0;
  gyroOff[2] = 0;

  for (int i = 0; i < 100; i++) {
    UpdateSensors();
    accOffset[0] += accRaw[0];
    accOffset[1] += accRaw[1];
    accOffset[2] += accRaw[2] - 1;

    gyroOff[0] += gyroRaw[0];
    gyroOff[1] += gyroRaw[1];
    gyroOff[2] += gyroRaw[2];
    delay(10);
  }

  accOffset[0] = accOffset[0] / 100;
  accOffset[1] = accOffset[1] / 100;
  accOffset[2] = accOffset[2] / 100;

  gyroOff[0] = gyroOff[0] / 100;
  gyroOff[1] = gyroOff[1] / 100;
  gyroOff[2] = gyroOff[2] / 100;

}
