/*
  Two Wheel Balancing Robot Project - Test Program - PID Tuning
  Author: Jichun Qu
  Date: 07.2017
  
  Library Version:
  LiquidCrystal - 1.0.5
  RF24          - 1.3.0

  PID parameters are sent via NRF24L01+ module.
  Four pushbuttons are used to adjust two parameters.
  
  Robot's angle is sent via NRF24L01+ module from robot to the remote controller.
  Communication protocol betwwen NRF24L01+ and the uC is SPI.

  A 16 * 2 LCD screen is used to display information (Movement command, setpoint, angle).
*/

#include <LiquidCrystal.h>  //https://www.arduino.cc/en/Tutorial/HelloWorld
#include <SPI.h>
#include "RF24.h" //http://tmrh20.github.io/RF24/classRF24.html
#include "nRF24L01.h"

#define DEBUG_MODE 0

#define LCD_RS  A0
#define LCD_ENABLE  A1
#define LCD_D4  8
#define LCD_D5  7
#define LCD_D6  6
#define LCD_D7  5

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

typedef struct
{
  float proportionalValue;
  float integralValue;
  float derivativelValue;
} pidValue_t;

typedef struct
{
  float angle;
  float reserved1;
  float reserved2;
} feedbackValue_t;

pidValue_t pidValue;
feedbackValue_t feedbackValue;

RF24 radio(9, 10);  //Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10
uint8_t addresses[][6] = {"1Node", "2Node"};
bool result;

//GPIO pins setup for pin change interrupt
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup()
{
  Serial.begin(115200);
  lcd.begin(16, 2);

  memset((void*)(&pidValue), 0, sizeof(pidValue_t));
  memset((void*)(&feedbackValue), 0, sizeof(feedbackValue_t));

  //default values
  pidValue.proportionalValue = 1.5; 
  pidValue.integralValue = .2;
  pidValue.derivativelValue = 2;

  result = radio.begin();
  lcd.setCursor(0, 0);
  lcd.print("radio.begin()");
  if (result == true)
  {
    lcd.setCursor(0, 1);
    lcd.print("okay");
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print("failed");
  }
  delay(1000); //delay 1s

  result = radio.setDataRate(RF24_2MBPS); //RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
  lcd.setCursor(0, 0);
  lcd.print("setDataRate: 2M");
  if (result == true)
  {
    lcd.setCursor(0, 1);
    lcd.print("okay");
  }
  else
  {
    lcd.setCursor(0, 1);
    lcd.print("failed");
  }
  delay(1000); //delay 1s

  radio.setPayloadSize(sizeof(pidValue_t));
  radio.setPALevel(RF24_PA_MAX);  //RF24_PA_LOW RF24_PA_HIGH RF24_PA_MAX
  radio.setAutoAck(false);  //disable acknowledgement
  radio.disableCRC(); //disable CRC checksum calculation
  radio.maskIRQ(false, false, false); //Disable Interrupt for tx_ok, tx_fail, rx_ready
  radio.setChannel(120);  //0-125. Higher frequency band minimize interference from other signals like WiFi.
  radio.setRetries(10, 10); //each retry: 250us * ´10 = 2.5ms. 10 retries. Maximum delay: 25ms

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  // Start the radio listening for data
  radio.startListening();

  lcd.setCursor(0, 0);
  lcd.print("Chip nRF24L01+");
  lcd.setCursor(0, 1);
  lcd.print("Initialized");
  delay(1000); //delay 1s

  lcd.clear();

  //Configure digital pin 2 and 3 for interrupt and hook up the callback functions
  attachInterrupt(digitalPinToInterrupt(2), pIncrement, RISING);
  attachInterrupt(digitalPinToInterrupt(3), pDecrement, RISING);

  //set up pin change interrupt
  pciSetup(4);  
  pciSetup(A5);
}

void loop()
{
  radio.stopListening();                                    // First, stop listening so we can talk.

#if (DEBUG_MODE == 1)
  Serial.println(F("Now sending"));
#endif

  lcd.setCursor(0, 0);
  lcd.print(pidValue.proportionalValue);

  lcd.setCursor(6, 0);
  lcd.print(pidValue.integralValue);

  lcd.setCursor(12, 0);
  lcd.print(pidValue.derivativelValue);

  if (!radio.write( &pidValue , sizeof(pidValue_t) ))
  {
#if (DEBUG_MODE == 1)
    Serial.println(F("failed"));
#endif

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending failed !");
  }

  radio.startListening(); // Now, continue listening

#if (DEBUG_MODE == 1)
  Serial.print(" Sent Proportional: ");
  Serial.print(pidValue.proportionalValue);
  Serial.print(" Sent IntegralValue: ");
  Serial.print(pidValue.integralValue);
  Serial.print(" Sent derivativelValue: ");
  Serial.print(pidValue.derivativelValue);
  Serial.print("\n");

  Serial.print(" Received angle: ");
  Serial.print(feedbackValue.angle);
  Serial.print("\n");
#endif

  while (radio.available())
  {
    radio.read( &feedbackValue, sizeof(feedbackValue_t) );

    lcd.setCursor(0, 1);
    lcd.print("Angle: ");

    lcd.setCursor(7, 1);
    lcd.print(feedbackValue.angle);

    lcd.setCursor(12, 1);
    lcd.print("okay");

    //print to Serial Monitor or Serial Plotter
    Serial.println(feedbackValue.angle);

#if (DEBUG_MODE == 1)
    Serial.print(" Sent Proportional: ");
    Serial.print(pidValue.proportionalValue);
    Serial.print(" Sent IntegralValue: ");
    Serial.print(pidValue.integralValue);
    Serial.print(" Sent derivativelValue: ");
    Serial.print(pidValue.derivativelValue);
    Serial.print("\n");

    Serial.print(" Received angle: ");
    Serial.print(feedbackValue.angle);
    Serial.print("\n");
#endif
  }

  // Try again 10ms later
  delay(10);
}

/*--------------------Callback Functions---------------------------*/
void pIncrement()
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(2) == HIGH) //filter jitter  
    pidValue.proportionalValue += 0.1; 

  sei();  //Enable global interrupt
}

void pDecrement()
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(3) == HIGH) //filter jitter
    pidValue.proportionalValue -= 0.1;
  
  sei();  //Enable global interrupt
}

ISR (PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(A5) == HIGH) //filter jitter  
    pidValue.derivativelValue += 0.1;  

  sei();  //Enable global interrupt
}

ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(4) == HIGH) //filter jitter  
    pidValue.derivativelValue -= 0.1;
  
  sei();  //Enable global interrupt
}





