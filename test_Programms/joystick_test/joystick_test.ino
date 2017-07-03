/*
  Two Wheel Balancing Robot Project - Test Program - Joystick
  Author: Jichun Qu
  Date: 07.2017

  uC reads the value of joystick in x and y direction.
*/

#define xDirection  2 //A2
#define yDirection  3 //A3

int xValue;
int yValue;

void setup() 
{
  Serial.begin(115200);

  xValue = 0;
  yValue = 0;
}

void loop() 
{
  xValue = analogRead(xDirection);
  yValue = analogRead(yDirection);

  Serial.print(" X Direction Value: ");
  Serial.print(xValue);
  
  Serial.print(" Y Direction Value: ");
  Serial.print(yValue);
  Serial.print("\n");

  delay(200);
}
