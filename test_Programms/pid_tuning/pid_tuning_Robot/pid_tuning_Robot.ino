/*
  Two Wheel Balancing Robot Project - Test Program - PID Tuning
  Author: Jichun Qu
  Date: 07.2017
  
  Library Version:
  FreeRTOS  - 9.0.0
  RF24      - 1.3.0

  Robot's angle is calculated from two sensors: accelerometer and gyroscope. 
  In this project only two dimensions of the acclerometer and one dimension of gyroscope are used.
  MPU6050 module provides the sensor values via I2C
  
  PID algorithm is used for controlling the robot. In order to get better control of the robot,
  mass center should be as high as possible. e.g. put battery pack on the top of the robot; Mass center
  should be very close to the center of the robot.

  L293D chip is used as a interface between uC and Motors. Speed of the motors is controlled by duty cycle (PWM)
  from the uC. Brushed DC motor's output is not linear to the input. In order to get a better control. motor encoder
  is recommended. However, motor encoder hasn't been used in this project yet. 

  Two motors are used as actuators. Note: the motors MUST have enough torque, otherwise
  the robot couldn't move. Motor speed is not a very important factor. Low speed motor can be compensated
  by large wheels.

  PID parameters are sent via NRF24L01+ module from remote controller to the robot.
  Robot's angle is sent via NRF24L01+ module from robot to the remote controller.
  Communication protocol betwwen NRF24L01+ and the uC is SPI.
    
  Because the mass center of the robot is not exactly in the middle, in order to keep robot standing, 
  two push buttons are used to adjust setpoint angle, one for increase, one for decrease.

  FreeRTOS is used. The task for controlling the robot higher priority than the other task
  for transmitting and receiving information.
*/

#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include<Wire.h>
#include "RF24.h"
#include "nRF24L01.h"

#define MEGA 0
#if (MEGA == 0)
#define NANO 1
#else
#define NANO 0
#endif

#define DEBUG_MODE 0

#define TASKTEST  0
#define TASK1PIN  A0
#define TASK2PIN  A1

//I2C
#define PWR_MGMT_1    0x6B
#define MPU6050ADDR   0x68  //slave address
#define ACCEL_XOUT_H  0x3B

//Motor Pins
#if (MEGA == 1)
#define MOTOR1ENABLE  2 //PWM
#define MOTOR2ENABLE  3 //PWM
#define MOTOR1PIN1    41
#define MOTOR1PIN2    40
#define MOTOR2PIN1    43
#define MOTOR2PIN2    42
#else
#define MOTOR1ENABLE  3 //PWM
#define MOTOR2ENABLE  5 //PWM
#define MOTOR1PIN1    2
#define MOTOR1PIN2    4
#define MOTOR2PIN1    6
#define MOTOR2PIN2    7
#endif

typedef struct
{
  float proportionalValue;
  float integralValue;
  float derivativelValue;
}pidValue_t;

typedef struct
{
  float angle;
  float reserved1;
  float reserved2;
}feedbackValue_t;

pidValue_t pidValueSender, pidValueReceiver;
feedbackValue_t feedbackValue;
float angleGet;
float angleErrorNew, angleErrorOld;
const float angleSet = 0; //angle setpoint

void vTaskReceiver(void *pvParameters);
void vTaskController(void *pvParameters);

/*--------------Functions------------------*/
void readSensor(int* pAccX, int* pAccY, int* pAccZ, int* pGyroX, int* pGyroY, int* pGyroZ)
{
  int temperature;

  Wire.beginTransmission(MPU6050ADDR);
  Wire.write(ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050ADDR, 14, true); // request a total of 14 registers
  *pAccX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  *pAccY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  *pAccZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  *pGyroX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  *pGyroY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  *pGyroZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
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
void calibrateSensor(int* pAccBiasX, int* pAccBiasY, int* pAccBiasZ, int* pGyroBiasX, int* pGyroBiasY, int* pGyroBiasZ)
{
  unsigned int i;
  int accX, accY, accZ, gyroX, gyroY, gyroZ;
  long accXTemp, accYTemp, accZTemp, gyroXTemp, gyroYTemp, gyroZTemp;

  accXTemp = 0;
  accYTemp = 0;
  accZTemp = 0;
  gyroXTemp = 0;
  gyroYTemp = 0;
  gyroZTemp = 0;

  for (i = 0; i <= 1000; i++)
  {
    readSensor(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    accXTemp += accX;
    accYTemp += accY;
    accZTemp += accZ;
    gyroXTemp += gyroX;
    gyroYTemp += gyroY;
    gyroZTemp += gyroZ;
    delay(1);
  }

  *pAccBiasX = accXTemp / 1000;
  *pAccBiasY = accYTemp / 1000;
  *pAccBiasZ = accZTemp / 1000 - 16384; //1g = 16384, AFS_SEL = 0, +-2g, according to datasheet.
  *pGyroBiasX = gyroXTemp / 1000;
  *pGyroBiasY = gyroYTemp / 1000;
  *pGyroBiasZ = gyroZTemp / 1000;

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
float getAccRoll(int accY, int accBiasY, int accZ, int accBiasZ)
{
  float accRoll;

  //calculate angle value
  accRoll = (atan2((accY - accBiasY), (accZ - accBiasZ))) * RAD_TO_DEG;

  if (accRoll <= 360 && accRoll >= 180) {
    accRoll = 360 - accRoll;
  }

  return accRoll;
}

//Get angle from the gyroscope. Uint: degree
float getGyroRoll(int gyroX, int gyroBiasX, uint32_t lastTime)
{
  float gyroRoll;

  //integrate gyroscope value in order to get angle value
  gyroRoll = ((gyroX - gyroBiasX ) / 131) * ((float)(micros() - lastTime) / 1000000); //FS_SEL=0 131LSB degree/second, according to datasheet

  return gyroRoll;
}

void initMotor()
{
  // Set pins as outputs
  pinMode(MOTOR1ENABLE, OUTPUT);
  pinMode(MOTOR2ENABLE, OUTPUT);
  pinMode(MOTOR1PIN1, OUTPUT);
  pinMode(MOTOR1PIN2, OUTPUT);
  pinMode(MOTOR2PIN1, OUTPUT);
  pinMode(MOTOR2PIN2, OUTPUT);

  // Set direction to none direction
  digitalWrite(MOTOR1PIN1, HIGH);
  digitalWrite(MOTOR1PIN2, HIGH);
  digitalWrite(MOTOR2PIN1, HIGH);
  digitalWrite(MOTOR2PIN2, HIGH);
}

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  memset((void*)(&pidValueSender),0,sizeof(pidValueSender));
  memset((void*)(&pidValueReceiver),0,sizeof(pidValueReceiver));
  memset((void*)(&feedbackValue),0,sizeof(feedbackValue_t));

#if (TASKTEST == 1)
  pinMode(TASK1PIN, OUTPUT);
  pinMode(TASK2PIN, OUTPUT);

  digitalWrite(TASK1PIN, HIGH);
  digitalWrite(TASK2PIN, HIGH);
#endif

  //Pre-emptive tasks: configUSE_PREEMPTION = 1 in FreeRTOSConfig.h
  xTaskCreate(
    vTaskReceiver
    ,  (const char *)"Receiver"   // A name just for humans
    ,  170  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL);

  xTaskCreate(
    vTaskController
    ,  (const char *) "Controller"
    ,  170  // Stack size in bytes
    ,  NULL
    ,  2  // Priority
    ,  NULL);


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void vTaskReceiver(void *pvParameters)
{
#if (MEGA == 1)
  /* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 30 & 31. 30: CE Pin, 31: CSN Pin. */
  RF24 radio(30, 31);
#else
  /* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10. 9: CE Pin, 10: CSN Pin. */
  RF24 radio(9, 10);
#endif

  //float pReceive;
  uint8_t addresses[][6] = {"1Node", "2Node"};
  bool result;
  float receivedValue;

  result = radio.begin();
#if (DEBUG_MODE == 1)
  if (result == true)
    Serial.println("Response from chip");
  else
    Serial.println("No response from chip");
#endif

  result = radio.setDataRate(RF24_2MBPS);
#if (DEBUG_MODE == 1)
  if (result == true)
    Serial.println("Set Data Rate Success");
  else
    Serial.println("Set Data Rate Failed");
#endif

  radio.setPayloadSize(sizeof(pidValue_t));
  radio.setPALevel(RF24_PA_MAX);  //RF24_PA_LOW RF24_PA_HIGH RF24_PA_MAX
  radio.setAutoAck(false);  //disable acknowledgement
  radio.disableCRC(); //disable CRC checksum calculation
  radio.maskIRQ(false, false, false); //Disable Interrupt for tx_ok, tx_fail, rx_ready
  radio.setChannel(120);  //0-125. Higher frequency band minimize interference from other signals like WiFi.
  radio.setRetries(10, 10); //each retry: 250us * ´10 = 2.5ms. 10 retries. Maximum delay: 25ms

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  // Start the radio listening for data
  radio.startListening();

  for (;;)
  {
#if (TASKTEST == 1)
    digitalWrite(TASK2PIN, LOW);
#endif

    if (radio.available())
    {
#if (DEBUG_MODE == 1)
      Serial.print("Data received\n");
#endif

      while (radio.available()) // While there is data ready
      {
        radio.read( &pidValueSender, sizeof(pidValue_t) );  // Get the payload
      }
      
      radio.stopListening();  // First, stop listening so we can talk

      taskENTER_CRITICAL();
      pidValueReceiver.proportionalValue = pidValueSender.proportionalValue;
      pidValueReceiver.integralValue = pidValueSender.integralValue;
      pidValueReceiver.derivativelValue = pidValueSender.derivativelValue;
      feedbackValue.angle = angleGet; //copy the golbal variable
      taskEXIT_CRITICAL();

      result = radio.write( &feedbackValue, sizeof(feedbackValue_t) );              // Send the final one back.

#if (DEBUG_MODE == 1)
      if (result == true)
        Serial.println("Write Success");
      else
        Serial.println("Write Failed");
#endif

      radio.startListening(); // Now, resume listening so we catch the next packets.

#if (DEBUG_MODE == 1)
    Serial.print(" Sent Proportional: ");
    Serial.print(pidValueSender.proportionalValue);
    Serial.print(" Sent IntegralValue: ");
    Serial.print(pidValueSender.integralValue);
    Serial.print(" Sent derivativelValue: ");
    Serial.print(pidValueSender.derivativelValue);
    Serial.print("\n");

    Serial.print(" Received angle: ");
    Serial.print(feedbackValue.angle);
    Serial.print("\n");
#endif
    }

#if (TASKTEST == 1)
    digitalWrite(TASK2PIN, HIGH);
#endif
  }
}

void vTaskController(void *pvParameters)
{
  int accBiasX, accBiasY, accBiasZ;
  int gyroBiasX, gyroBiasY, gyroBiasZ;
  int accX, accY, accZ, gyroX, gyroY, gyroZ;
  uint32_t lastTime;

  float pPart, iPart, dPart;
  int motorInput;

  initMPU6050();

  calibrateSensor(&accBiasX, &accBiasY, &accBiasZ, &gyroBiasX, &gyroBiasY, &gyroBiasZ);

  angleGet = 0;
  angleErrorNew = 0;
  angleErrorOld = 0;

  for (;;)
  {
#if (TASKTEST == 1)
    digitalWrite(TASK1PIN, LOW);
#endif

    //sensor measure raw data
    readSensor(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

    //complementary filter for angle calculation
    angleGet = 0.98 * (angleGet + getGyroRoll(gyroX, gyroBiasX, lastTime)) + 0.02 * (getAccRoll(accY, accBiasY, accZ, accBiasZ));

    //save the old error value and update the new error value
    angleErrorOld = angleErrorNew;
    angleErrorNew = angleGet - angleSet;

    Serial.println(angleGet);

    lastTime = micros(); // Reset the timer

#if 0
#if (DEBUG_MODE == 1)
    Serial.print("AcX = ");  Serial.print(accX);
    Serial.print(" | AcY = "); Serial.print(accY);
    Serial.print(" | AcZ = "); Serial.print(accZ);
    Serial.print(" | GyX = "); Serial.print(gyroX);
    Serial.print(" | GyY = "); Serial.print(gyroY);
    Serial.print(" | GyZ = "); Serial.println(gyroZ);
#endif
#endif

    //PID control
    pPart = pidValueSender.proportionalValue * angleErrorNew;
    iPart += pidValueSender.integralValue * angleErrorNew;
    dPart = pidValueSender.derivativelValue * (angleErrorNew - angleErrorOld);

    //output value to actuator
    motorInput = (int)(pPart + iPart + dPart);

    //limit the range: -225 to +255
    if (motorInput > 255)
      motorInput = 255;
    else if (motorInput < -255)
      motorInput = -255;

#if (DEBUG_MODE == 1)
    Serial.print(" pPart: ");
    Serial.print(pPart);
    Serial.print(" iPart: ");
    Serial.print(iPart);
    Serial.print(" dPart: ");
    Serial.print(dPart);
    Serial.print(" to motor: ");
    Serial.println(motorInput);
#endif

    // Sets direction
    if (motorInput > 0)
    { // forward
      digitalWrite(MOTOR1PIN1, LOW);
      digitalWrite(MOTOR1PIN2, HIGH);
      digitalWrite(MOTOR2PIN1, LOW);
      digitalWrite(MOTOR2PIN2, HIGH);
    }
    else if (motorInput < 0)
    { // backward
      digitalWrite(MOTOR1PIN1, HIGH);
      digitalWrite(MOTOR1PIN2, LOW);
      digitalWrite(MOTOR2PIN1, HIGH);
      digitalWrite(MOTOR2PIN2, LOW);
    }

    // Checks velocity fits the max ouptut range
    motorInput = abs(motorInput);    // Absolute value of velocity

    // Writes the PWM
    analogWrite(MOTOR1ENABLE, motorInput);
    analogWrite(MOTOR2ENABLE, motorInput);

#if (DEBUG_MODE == 1)
    Serial.print("Motor1 and Motor2 speed: ");
    Serial.print(motorInput);
    Serial.print("\n");
#endif

#if (TASKTEST == 1)
    digitalWrite(TASK1PIN, HIGH);
#endif

    vTaskDelay(1); // wait for 15ms. One tick = 15 miliseconds
  }
}


