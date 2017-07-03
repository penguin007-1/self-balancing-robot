/*
  Two Wheel Balancing Robot Project - Test Program - Motor
  Author: Jichun Qu
  Date: 07.2017

  L293D chip is used as a interface between uC and Motors. Speed of the motors is controlled by duty cycle (PWM)
  from the uC.
*/

#define DEBUG_MODE 1

#define MEGA 0
#if (MEGA == 1)
#define MOTOR1ENABLE  2
#define MOTOR2ENABLE  3
#define MOTOR1PIN1    40
#define MOTOR1PIN2    41
#define MOTOR2PIN1    42
#define MOTOR2PIN2    43
#else
#define MOTOR1ENABLE  3
#define MOTOR2ENABLE  5
#define MOTOR1PIN1    2
#define MOTOR1PIN2    4
#define MOTOR2PIN1    6
#define MOTOR2PIN2    7
#endif

int speed;

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

void actuator(double value)
{
  // Sets direction
  if (value > 0) // forward
  {
    digitalWrite(MOTOR1PIN1, LOW);
    digitalWrite(MOTOR1PIN2, HIGH);
    digitalWrite(MOTOR2PIN1, LOW);
    digitalWrite(MOTOR2PIN2, HIGH);
  }
  else  // backward
  {
    digitalWrite(MOTOR1PIN1, HIGH);
    digitalWrite(MOTOR1PIN2, LOW);
    digitalWrite(MOTOR2PIN1, HIGH);
    digitalWrite(MOTOR2PIN2, LOW);
  }

  value = abs(value);    // Absolute value of velocity

  if (value > 255)
    value = 255;

  // Writes the PWM
  analogWrite(MOTOR1ENABLE, (uint8_t)value);
  analogWrite(MOTOR2ENABLE, (uint8_t)value);
}

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  initMotor();

#if (DEBUG_MODE == 1)
  Serial.print("Motor initialized\n");
#endif

  speed = 0;//PWM value, -255 to +255
}

void loop() 
{
  actuator(255);
  delay(5000);  //delay 5s

  actuator(-255);
  delay(5000);  //delay 5s
}
