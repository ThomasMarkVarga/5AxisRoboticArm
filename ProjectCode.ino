/* Include the HCPCA9685 library */
#include "HCPCA9685.h"
#include <Servo.h>

/* I2C slave address for the device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define  I2CAdd 0x40


/* Create an instance of the library */
HCPCA9685 HCPCA9685(I2CAdd);

/* pin mumbers */
const int buttonPin1 = 2;
const int buttonPin2 = 3;

/* variables */
const int servoStop = 175;
const int servoSpeed = 25;
int potVal, potVal1, potVal2, potVal3, potVal4;
int potStop, potStop1, potStop2, potStop3, potStop4;
unsigned int servoStart;

int buttonState1;
int buttonState2;
int gripperState = 0;
int baseState = 0;

Servo servoGripper;

void moveServo(int potVal, int servoPin, int &potStop)
{
  if(potVal > potStop + 3)
  {
    int potSpeed = 20;
    servoStart = servoStop  + (servoSpeed + potSpeed);
    HCPCA9685.Servo(servoPin, servoStart);
  }
  if(potVal < potStop - 3)
  {
    int potSpeed = 20;
    servoStart = servoStop - 25 - (servoSpeed - potSpeed);
    HCPCA9685.Servo(servoPin, servoStart);
  }

  else HCPCA9685.Servo(servoPin, servoStart);

  potStop = potVal;
  servoStart = 175;
}

void setup() 
{
  /* Initialise the library and set it to 'servo mode' */ 
  HCPCA9685.Init(SERVO_MODE);

  /* Wake the device up */
  HCPCA9685.Sleep(false);

  Serial.begin(9600);

  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  servoGripper.attach(9);
}

void loop() 
{

  /* servo 0 */
  potVal = map(analogRead(0), 0, 1023, 0, 350);
  
  if(potVal > potStop + 3)
  {
    int potSpeed = 20;
    servoStart = 200  + (servoSpeed + potSpeed);
    HCPCA9685.Servo(0, servoStart);
  }
  else if(potVal < potStop - 3)
  {
    int potSpeed = 20;
    servoStart = servoStop - 25 - (servoSpeed - potSpeed);
    HCPCA9685.Servo(0, servoStart);
  }
  else HCPCA9685.Servo(0, 197);

  potStop = potVal;
  servoStart = 175;

  /* servo 1 */
  potVal1 = map(analogRead(1), 0, 1023, 0, 350);
  if(potVal1 > potStop1 + 3)
  {
    int potSpeed = 20;
    servoStart = servoStop + (servoSpeed + potSpeed) - 15;
    HCPCA9685.Servo(1, servoStart);
  }
  else if(potVal1 < potStop1 - 3)
  {
    int potSpeed = 20;
    servoStart = servoStop - 25 - (servoSpeed - potSpeed);
    HCPCA9685.Servo(1, servoStart);
  }
  else HCPCA9685.Servo(1, 171);

  potStop1 = potVal1;
  servoStart = 175;
  /* servo 2 */
  potVal2 = map(analogRead(2), 0, 1023, 0, 350);
  moveServo(potVal2, 2, potStop2);
  /* servo 3 */
  potVal3 = map(analogRead(3), 0, 1023, 0, 350);
  moveServo(potVal3, 3, potStop3);

  /* servo gripper */
  buttonState1 = digitalRead(buttonPin1);
  delay(100);

  if(buttonState1 == HIGH)
  {
    if(gripperState == 0)
    {
        gripperState = 1;
        servoGripper.write(20);
    }
    else if(gripperState == 1)
    {
      gripperState = 0;
      servoGripper.write(120); //240 - full open 50 - closed
    }
  }

  /* servo base */
  buttonState2 = digitalRead(buttonPin2);

  if(buttonState2 == HIGH)
  {
    if(baseState == 0)
    {
      baseState = 1;
      HCPCA9685.Servo(4, 300);
      delay(550);
    }
    else
    {
      baseState = 0;
      HCPCA9685.Servo(4, 50);
      delay(550);
    }
  }
  else HCPCA9685.Servo(4, 175);
  delay(15);
}
