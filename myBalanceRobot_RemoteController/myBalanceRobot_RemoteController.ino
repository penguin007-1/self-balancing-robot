/*
  Two Wheel Balancing Robot Project - Remote Controller Part
  Author: Jichun Qu
  Date: 07.2017
  
  Library Version:
  LiquidCrystal - 1.0.5
  RF24          - 1.3.0

  Robot's movement command (Forward Backward Left Right) are sent via NRF24L01+ module.
  Robot's angle is sent via NRF24L01+ module from robot to the remote controller.
  Communication protocol betwwen NRF24L01+ and the uC is SPI.
  
  uC takes Joystick's value as input, which represnets the commands.
  
  Because the mass center of the robot is not exactly in the middle, in order to keep robot standing, 
  two push buttons are used to adjust setpoint angle, one for increase, one for decrease.

  A 16 * 2 LCD screen is used to display information (Movement command, setpoint, angle).
*/


#include <LiquidCrystal.h>  //https://www.arduino.cc/en/Tutorial/HelloWorld
#include <SPI.h>
#include <RF24.h> //http://tmrh20.github.io/RF24/classRF24.html
#include <nRF24L01.h>

#define DEBUG_MODE 0

#define LCD_RS  A0  //resiger select
#define LCD_ENABLE  A1
#define LCD_D4  8 //4-bit mode
#define LCD_D5  7
#define LCD_D6  6
#define LCD_D7  5

#define INTERRUPT_PIN1  2 
#define INTERRUPT_PIN2  3

#define xDirection  2 //Pin A2  Forward or Backward  
#define yDirection  3 //Pin A3  Left or Right

typedef struct
{
  float angleOffset;
  float angleSet; //forward or backward
  int8_t motorTurn;  //left or right  
}txPkt_t;

typedef struct
{
  float angleGet;
  float reserved1;
  int8_t reserved2;
}rxPkt_t;

txPkt_t txPkt;
rxPkt_t rxPkt;

LiquidCrystal lcd(LCD_RS, LCD_ENABLE, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

uint16_t xJoystickValue;
uint16_t yJoystickValue;
const float angleForward = 1.5; //increment for new setpoint. unit: degree
const float angleBackward = -1.5;
bool result;

RF24 radio(9, 10);  //Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10. 9: CE Pin, 10: CSN Pin.
uint8_t addresses[][6] = {"1Node", "2Node"};
long loopCnt;

void setup()
{
  Serial.begin(115200);
  lcd.begin(16, 2); //Initalize LCD

  xJoystickValue = 0;
  yJoystickValue = 0;

  memset(((void*)&txPkt), 0, sizeof(txPkt_t));
  memset(((void*)&rxPkt), 0, sizeof(rxPkt_t));

  loopCnt = 0;

  result = radio.begin(); //Initialize NRF24L01+
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

  radio.setPayloadSize(sizeof(txPkt_t));
  radio.setPALevel(RF24_PA_MAX);  //RF24_PA_LOW RF24_PA_HIGH RF24_PA_MAX
  radio.setAutoAck(false);  //disable acknowledgement
  radio.disableCRC(); //disable CRC checksum calculation
  radio.maskIRQ(false, false, false); //Disable Interrupt for tx_ok, tx_fail, rx_ready
  radio.setChannel(120);  //0-125. Higher frequency band minimize possibility of interference from other signals like WiFi.
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
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN1), offsetIncrement, RISING);  //pin 2
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), offsetDecrement, RISING);  //pin 3
}

void loop()
{
  loopCnt++;

  xJoystickValue = analogRead(xDirection);
  yJoystickValue = analogRead(yDirection);

  lcd.setCursor(0, 0);
  if(xJoystickValue > 900)  //Forward
  {
    txPkt.angleSet = angleForward;         
    lcd.print("F");
  }
  else if(xJoystickValue < 100) //Backward
  {
    txPkt.angleSet = angleBackward; 
    lcd.print("B");
  }
  else  //Stand
  {
    txPkt.angleSet = 0;
    lcd.print("S");
  }

  lcd.setCursor(2, 0);
  if(yJoystickValue < 100)   //Left
  {
    if(loopCnt & 0x01)   //Continous sending may cause stability issue, because motors can't be used for self balancing.
      txPkt.motorTurn = -1;
    else
      txPkt.motorTurn = 0;
      
    lcd.print("L");
  }
  else if(yJoystickValue > 900) //Right
  {
    if(loopCnt & 0x01)   //Continous sending may cause stability issue, because motors can't be used for self balancing.
     txPkt.motorTurn = 1;
    else
      txPkt.motorTurn = 0;
     
    lcd.print("R");
  }
  else  //Stand
  {
    txPkt.motorTurn = 0;
    lcd.print("M");
  }

  if((rxPkt.angleGet > 2) || (rxPkt.angleGet < -2)) //too much deviation, don't move or turn
    txPkt.motorTurn = 0;

  radio.stopListening();  // First, stop listening so we can talk.

#if (DEBUG_MODE == 1)
  Serial.println(F("Now sending"));
#endif

  lcd.setCursor(4, 0);
  lcd.print("Offset: ");

  lcd.setCursor(12, 0);
  lcd.print(txPkt.angleOffset);

  if (!radio.write( &txPkt , sizeof(txPkt_t) ))
  {
#if (DEBUG_MODE == 1)
    Serial.println(F("failed"));
#endif

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending failed !");
  }

  radio.startListening(); // Now, continue listening

  while (radio.available())
  {
    radio.read( &rxPkt.angleGet, sizeof(rxPkt_t) );

    lcd.setCursor(0, 1);
    lcd.print("Angle Get: ");

    lcd.setCursor(11, 1);
    lcd.print(rxPkt.angleGet);
    
    Serial.println(rxPkt.angleGet); //print to Serial Monitor or Serial Plotter

#if (DEBUG_MODE == 1)
    Serial.print("txPkt.angleOffset: ");
    Serial.print(txPkt.angleOffset);
    Serial.print("\n");

    Serial.print("rxPkt.angleGet");
    Serial.print(rxPkt.angleGet);
    Serial.print("\n");
#endif
  } //while (radio.available())

  // Try again 10ms later
  delay(10);
} //loop

/*--------------------Callback Functions---------------------------*/
void offsetIncrement()
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(INTERRUPT_PIN1) == HIGH) //filter jitter 
    txPkt.angleOffset += 0.1;

  sei();  //Enable global interrupt 
}

void offsetDecrement()
{
  cli(); //Disable global interrupt

  delay(20000);

  if (digitalRead(INTERRUPT_PIN2) == HIGH) //filter jitter
    txPkt.angleOffset -= 0.1;
  
  sei();  //Enable global interrupt 
}


