/*
  Two Wheel Balancing Robot Project - Test Program - MPU6050
  Author: Jichun Qu
  Date: 07.2017

  Robot's angle is calculated from two sensors: accelerometer and gyroscope.
  In this project only two dimensions of the acclerometer and one dimension of gyroscope are used.
  MPU6050 module provides the sensor values via I2C
*/

#include<Wire.h>  //I2C

#define DEBUG_MODE 0

//I2C
#define PWR_MGMT_1    0x6B
#define MPU6050ADDR   0x68  //slave address
#define ACCEL_XOUT_H  0x3B

int accBiasX, accBiasY, accBiasZ;
int gyroBiasX, gyroBiasY, gyroBiasZ;
int accX, accY, accZ, gyroX, gyroY, gyroZ;
int temperature;
uint32_t lastTime;

double angleMeas; //angle measured online

void readSensor(void)
{
  Wire.beginTransmission(MPU6050ADDR);
  Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050ADDR, 14, true); // request a total of 14 registers
  accX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void initMPU6050(void)
{
  Wire.begin();
  Wire.beginTransmission(MPU6050ADDR);
  Wire.write(PWR_MGMT_1);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

//calibrate bias of the accelerometer and gyroscope
void calibrateSensor(void)
{
  unsigned int i;
  long accXTemp, accYTemp, accZTemp, gyroXTemp, gyroYTemp, gyroZTemp;

  accXTemp = 0;
  accYTemp = 0;
  accZTemp = 0;
  gyroXTemp = 0;
  gyroYTemp = 0;
  gyroZTemp = 0;

  for (i = 0; i <= 1000; i++)
  {
    readSensor();

    accXTemp += accX;
    accYTemp += accY;
    accZTemp += accZ;
    gyroXTemp += gyroX;
    gyroYTemp += gyroY;
    gyroZTemp += gyroZ;
    delay(1);
  }

  accBiasX = accXTemp / 1000;
  accBiasY = accYTemp / 1000;
  accBiasZ = accZTemp / 1000 - 16384; //1g = 16384, AFS_SEL = 0, +-2g, according to datasheet.
  gyroBiasX = gyroXTemp / 1000;
  gyroBiasY = gyroYTemp / 1000;
  gyroBiasZ = gyroZTemp / 1000;

#if 0
#if (DEBUG_MODE == 1)
  Serial.print("Bias X Accelerometer: "); Serial.print(accBiasX); Serial.print("\n");
  Serial.print("Bias Y Accelerometer: "); Serial.print(accBiasY); Serial.print("\n");
  Serial.print("Bias Z Accelerometer: "); Serial.print(accBiasZ); Serial.print("\n");
  Serial.print("Bias X Gyroscope: "); Serial.print(gyroBiasX); Serial.print("\n");
  Serial.print("Bias Y Gyroscope: "); Serial.print(gyroBiasY); Serial.print("\n");
  Serial.print("Bias Z Gyroscope: "); Serial.print(gyroBiasZ); Serial.print("\n");
#endif
#endif
}

//Get angle from the accelerometer. Uint: degree
double getAccRoll(void)
{
  double accRoll;

  //calculate angle value
  accRoll = (atan2((accY - accBiasY), (accZ - accBiasZ))) * RAD_TO_DEG;

  if (accRoll <= 360 && accRoll >= 180) {
    accRoll = 360 - accRoll;
  }

  return accRoll;
}

//Get angle from the gyroscope. Uint: degree
double getGyroRoll(void)
{
  double gyroRoll;

  //integrate gyroscope value in order to get angle value
  gyroRoll = ((gyroX - gyroBiasX ) / 131) * ((double)(micros() - lastTime) / 1000000); //FS_SEL=0 131LSB degree/second, according to datasheet

  return gyroRoll;
}

void setup()
{
  Serial.begin(115200);
  initMPU6050();

#if (DEBUG_MODE == 1)
  Serial.print("Communication with sensor established\n");
#endif

  calibrateSensor();

#if (DEBUG_MODE == 1)
  Serial.print("Sensor Initialized\n");
#endif

  delay(1000);
}

void loop()
{
  //sensor measure raw data
  readSensor();

  //complementary filter for angle calculation
  angleMeas = 0.98 * (angleMeas + getGyroRoll()) + 0.02 * (getAccRoll());

  Serial.println(angleMeas);

  lastTime = micros(); // Reset the timer

  delay(10);

#if (DEBUG_MODE == 1)
  Serial.print("AcX = ");  Serial.print(accX);
  Serial.print(" | AcY = "); Serial.print(accY);
  Serial.print(" | AcZ = "); Serial.print(accZ);
  Serial.print(" | Tmp = "); Serial.print(temperature / 340.00 + 36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(gyroX);
  Serial.print(" | GyY = "); Serial.print(gyroY);
  Serial.print(" | GyZ = "); Serial.println(gyroZ);
#endif
}



