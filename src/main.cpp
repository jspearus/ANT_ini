#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <HCSR04.h>
#include <PWMServo.h>

PWMServo grip; // create servo object to control a servo
// twelve servo objects can be created on most boards

UltraSonicDistanceSensor distanceSensor(9, 10); // trig, echo?
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

#define SERVO_FREQ 50 //

// FUNCTOIN DECLARATIONS

void center();
void home();
void stand();
void sit();
void step_forward();

void moveLegLeftFront(int pos);

void printCurPos();
void serialEvent();

float dist = 0;
int mode = 0;
int leftKneeStep = 2400;
int rightKneeStep = 600;
int leftAnkleStep = 2400;
int rightAnkleStep = 600;
String Data_In = "";

// Home Servo Positions
int homeLval[] = {1300, 1300, 0, 0, 1500, 2400, 2400, 0, 1500, 2400, 2400, 0, 1500, 2400, 2400, 0};
int homeRval[] = {1500, 0, 0, 0, 1500, 600, 600, 0, 1500, 600, 600, 0, 1500, 600, 600, 0};
// Stand Servo Positions
int standLval[] = {1500, 1410, 1950};
int standRval[] = {1500, 1590, 1050};
// Sit Servo Positions
int sitLval[] = {1500, 2400, 2400};
int sitRval[] = {1500, 600, 600};

// Current Servo Postions
int curLval[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int curRval[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// Previous Servo Postion
int preLval[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int preRval[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
void setup()
{
  Serial.begin(115200);

  pwmL.begin();
  pwmR.begin();

  pwmL.setOscillatorFrequency(27000000);
  pwmL.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pwmR.setOscillatorFrequency(27000000);
  pwmR.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  grip.attach(8);
  grip.write(80);
  delay(200);
  grip.write(130);
  delay(2000);
  grip.write(90);
  home();
}

void loop()
{
  // grip.write(80); // grip open
  // grip.write(130); // grip closed
  dist = distanceSensor.measureDistanceCm();
  delay(50);
  if (dist > 0)
  {
    if (dist < 15 && mode == 0)
    {
      grip.write(130);
      delay(2000);
      grip.write(90);
      stand();
    }
    else if (dist < 15 && mode == 1)
    {
      step_forward();
      grip.write(130);
      delay(2000);
      grip.write(90);
      sit();
    }
  }
}

void center()
{
  for (int i = 0; i < 16; i++)
  {
    pwmL.writeMicroseconds(i, 1500);
    delay(100);
    pwmR.writeMicroseconds(i, 1500);
    delay(100);
  }
}

void home()
{
  mode = 0;
  grip.write(110);                        // grip closed
  pwmL.writeMicroseconds(0, homeLval[0]); // head tilt min 1150 UP - max 1800 Down
  pwmL.writeMicroseconds(1, homeLval[1]); // head roll
  pwmR.writeMicroseconds(0, homeRval[0]); // Tail Pan
  delay(50);
  for (int i = 4; i < 16; i = i + 4)
  {
    pwmL.writeMicroseconds(i, homeLval[i]);
    delay(100);
    pwmL.writeMicroseconds(i + 1, homeLval[i + 1]);
    delay(100);
    pwmL.writeMicroseconds(i + 2, homeLval[i + 2]);
    delay(100);
    pwmR.writeMicroseconds(i, homeRval[i]);
    delay(100);
    pwmR.writeMicroseconds(i + 1, homeRval[i + 1]);
    delay(100);
    pwmR.writeMicroseconds(i + 2, homeRval[i + 2]);
    delay(100);
  }
  pwmR.writeMicroseconds(0, 1600);
  for (int i = 0; i < 16; i++)
  {
    curLval[i] = homeLval[i];
    curRval[i] = homeRval[i];
  }
  printCurPos();
}

void stand()
{
  mode = 1;
  int LeftPos = 2400;
  int RightPos = 600;
  pwmL.writeMicroseconds(0, 1500);
  pwmL.writeMicroseconds(1, 1500);
  pwmR.writeMicroseconds(0, 1500);
  // for (int j = 0; j < 100; j++)
  for (int j = 0; j < 1000; j = j + 10)
  {
    for (int i = 1; i < 4; i++)
    {
      curLval[(i * 4) + 1] = LeftPos - j;
      curLval[(i * 4) + 2] = LeftPos - (j / 2.2);
      curRval[(i * 4) + 1] = RightPos + j;
      curRval[(i * 4) + 2] = RightPos + (j / 2.2);

      pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1]);
      pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2]);

      pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1]);
      pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2]);
    }
  }
  printCurPos();
}
void sit()
{
  mode = 0;
  int LeftPos = 2400;
  int RightPos = 600;
  pwmL.writeMicroseconds(0, 1500);
  pwmL.writeMicroseconds(1, 1500);
  pwmR.writeMicroseconds(0, 1500);
  for (int j = 1000; j > 0; j = j - 10)
  {
    for (int i = 1; i < 4; i++)
    {
      curLval[(i * 4) + 1] = LeftPos - j;
      curLval[(i * 4) + 2] = LeftPos - (j / 2.2);
      curRval[(i * 4) + 1] = RightPos + j;
      curRval[(i * 4) + 2] = RightPos + (j / 2.2);

      pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1]);
      pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2]);

      pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1]);
      pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2]);
    }
  }
  printCurPos();
}

void step_forward()
{
  delay(500);
}

void moveLegLeftFront(int pos)
{
}

void printCurPos()
{
  for (int i = 0; i < 16; i++)
  {
    Serial.print(curLval[i]);
    Serial.print(", ");
  }
  Serial.println("");
  for (int i = 0; i < 16; i++)
  {

    Serial.print(curRval[i]);
    Serial.print(", ");
  }
  Serial.println("");
}

void serialEvent()
{ // From Desktop == Debug/program
  while (Serial.available())
  {
    // add it to the inputString:
    Data_In = Serial.readStringUntil('#');
    if (Data_In == "step")
    {
      step_forward();
    }
    else if (Data_In == "stand")
    {
      stand();
    }
    else if (Data_In == "sit")
    {
      sit();
    }
    else if (Data_In == "home")
    {
      home();
    }
    else
    {
      Serial.println("unknown command...");
    }
  }
}