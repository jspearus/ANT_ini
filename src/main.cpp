#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <HCSR04.h>
#include <PWMServo.h>

PWMServo grip; // create servo object to control a servo
// twelve servo objects can be created on most boards
UltraSonicDistanceSensor distanceSensor(9, 10); // trig, echo
// setup ultrasonic sensor

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
void attack();
void step_forward();
void step_backward();
void turn_right();
void turn_left();

void moveGrip(int pos);
void moveLeftLeg(int leg, int pos, int hieght, int steps);
void moveRightLeg(int leg, int pos, int hieght, int steps);

void printCurPos();
void serialEvent();

float dist = 0;
int mode = 0;
int leftKneeStep = 0;
int rightKneeStep = 0;
int leftAnkleStep = 0;
int rightAnkleStep = 0;
String Data_In = "";

// Home Servo Positions
int homeLval[] = {1300, 1300, 0, 0, 1600, 2200, 2200, 0, 1500, 2200, 2200, 0, 1400, 2200, 2200, 0};
int homeRval[] = {1500, 0, 0, 0, 1600, 750, 750, 0, 1500, 750, 750, 0, 1400, 750, 750, 0};
// Stand Servo Positions
int standLval[] = {1500, 1800, 2300}; // 1500, 1410, 1950
int standRval[] = {1500, 1200, 700}; // 1500, 1590, 1050
// Sit Servo Positions
int sitLval[] = {1500, 2200, 2200};  //1500, 2400, 2400
int sitRval[] = {1500, 750, 750};  // 1500, 600, 600

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
  moveGrip(80);
  delay(200);
  moveGrip(130);
  delay(2000);
  moveGrip(90);
  home();
}

void loop()
{
  // moveGrip(80); // grip open
  // moveGrip(130); // grip closed
  dist = distanceSensor.measureDistanceCm();
  delay(50);
  if (dist > 0)
  {
    if (dist < 15 && mode == 0)
    {

      moveGrip(130);
      delay(2000);
      moveGrip(90);
      stand();
    }
    else if (dist < 15 && mode == 1)
    {
      attack();
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
  moveGrip(110);                          // grip closed
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
  Serial.println("sit,0");
}

void stand()
{
  mode = 1;
  leftKneeStep = (standLval[1] - curLval[5]) / 100;
  leftAnkleStep = (standLval[2] - curLval[6]) / 100;
  rightKneeStep = (standRval[1] - curRval[5]) / 100;
  rightAnkleStep = (standRval[2] - curRval[6]) / 100;
  pwmL.writeMicroseconds(0, 1500);
  pwmL.writeMicroseconds(1, 1500);
  pwmR.writeMicroseconds(0, 1500);

  for (int j = 1; j < 101; j++)
  {
    for (int i = 1; i < 4; i++)
    { // move to standing position
      curLval[(i * 4) + 1] = curLval[(i * 4) + 1] + leftKneeStep;
      curLval[(i * 4) + 2] = curLval[(i * 4) + 2] + leftAnkleStep;
      curRval[(i * 4) + 1] = curRval[(i * 4) + 1] + rightKneeStep;
      curRval[(i * 4) + 2] = curRval[(i * 4) + 2] + rightAnkleStep;

      pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1]);
      pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2]);

      pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1]);
      pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2]);
    }
  }
  for (int i = 1; i < 4; i++)
  { // lift each leg up and down one at a time
    pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1] + 200);
    pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2] + 100);
    delay(100);
    pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2]);
    delay(10);
    pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1]);
    delay(100);
    pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1] - 200);
    pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2] - 100);
    delay(100);
    pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2]);
    delay(10);
    pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1]);
    delay(100);
  }
  Serial.println("stand,0");
}
void sit()
{ // move to sitting position
  mode = 0;
  leftKneeStep = (sitLval[1] - curLval[5]) / 100;
  leftAnkleStep = (sitLval[2] - curLval[6]) / 100;
  rightKneeStep = (sitRval[1] - curRval[5]) / 100;
  rightAnkleStep = (sitRval[2] - curRval[6]) / 100;
  pwmL.writeMicroseconds(0, 1500);
  pwmL.writeMicroseconds(1, 1500);
  pwmR.writeMicroseconds(0, 1500);
  for (int j = 1; j < 101; j++)
  {
    for (int i = 1; i < 4; i++)
    {
      curLval[(i * 4) + 1] = curLval[(i * 4) + 1] + leftKneeStep;
      curLval[(i * 4) + 2] = curLval[(i * 4) + 2] + leftAnkleStep;
      curRval[(i * 4) + 1] = curRval[(i * 4) + 1] + rightKneeStep;
      curRval[(i * 4) + 2] = curRval[(i * 4) + 2] + rightAnkleStep;

      pwmL.writeMicroseconds((i * 4) + 1, curLval[(i * 4) + 1]);
      pwmL.writeMicroseconds((i * 4) + 2, curLval[(i * 4) + 2]);

      pwmR.writeMicroseconds((i * 4) + 1, curRval[(i * 4) + 1]);
      pwmR.writeMicroseconds((i * 4) + 2, curRval[(i * 4) + 2]);
    }
  }
  Serial.println("sit,0");
}
void attack()
{
  moveGrip(80);
  moveLeftLeg(0, 200, 0, 0);
  moveRightLeg(0, 200, 0, 0);
  moveLeftLeg(1, 200, 0, 0);
  moveRightLeg(1, 200, 0, 0);
  moveLeftLeg(2, 200, 0, 0);
  moveRightLeg(2, 200, 0, 0);
  delay(1000);
  moveLeftLeg(0, -200, 0, 0);
  moveRightLeg(0, -200, 0, 0);
  moveLeftLeg(1, -200, 0, 0);
  moveRightLeg(1, -200, 0, 0);
  moveLeftLeg(2, -200, 0, 0);
  moveRightLeg(2, -200, 0, 0);
  delay(100);
  moveGrip(130);
  delay(500);
  moveGrip(100);
}

void step_forward()
{
  moveLeftLeg(1, 200, 200, 70);
  moveRightLeg(1, 200, 200, 70);
  moveLeftLeg(0, 200, 200, 100);
  moveRightLeg(0, 200, 200, 100);
  delay(100);
  moveLeftLeg(0, -200, 0, 0);
  moveRightLeg(0, -200, 0, 0);
  moveLeftLeg(1, -200, 0, 0);
  moveRightLeg(1, -200, 0, 0);
  moveLeftLeg(2, -200, 0, 0);
  moveRightLeg(2, -200, 0, 0);
  delay(100);
  moveLeftLeg(2, 200, 200, 100);
  moveRightLeg(2, 200, 200, 100);
}

void step_backward()
{
  moveLeftLeg(1, -200, 200, 70);
  moveRightLeg(1, -200, 200, 70);
  moveLeftLeg(2, -200, 200, 100);
  moveRightLeg(2, -200, 200, 100);
  delay(100);
  moveLeftLeg(0, 200, 0, 0);
  moveRightLeg(0, 200, 0, 0);
  moveLeftLeg(1, 200, 0, 0);
  moveRightLeg(1, 200, 0, 0);
  moveLeftLeg(2, 200, 0, 0);
  moveRightLeg(2, 200, 0, 0);
  delay(100);
  moveLeftLeg(0, -200, 200, 100);
  moveRightLeg(0, -200, 200, 100);
}

void turn_right()
{
  moveLeftLeg(1, 200, 200, 70);
  moveRightLeg(1, -200, 200, 70);
  moveLeftLeg(0, 200, 200, 100);
  moveRightLeg(2, -200, 200, 100);
  delay(100);
  moveLeftLeg(0, -200, 0, 0);
  moveRightLeg(0, 200, 0, 0);
  moveLeftLeg(1, -200, 0, 0);
  moveRightLeg(1, 200, 0, 0);
  moveLeftLeg(2, -200, 0, 0);
  moveRightLeg(2, 200, 0, 0);
  delay(100);
  moveLeftLeg(2, 200, 200, 100);
  moveRightLeg(0, -200, 200, 100);
}
void turn_left()
{
  moveLeftLeg(1, -200, 200, 70);
  moveRightLeg(1, 200, 200, 70);
  moveLeftLeg(0, -200, 200, 100);
  moveRightLeg(2, 200, 200, 100);
  delay(100);
  moveLeftLeg(0, 200, 0, 0);
  moveRightLeg(0, -200, 0, 0);
  moveLeftLeg(1, 200, 0, 0);
  moveRightLeg(1, -200, 0, 0);
  moveLeftLeg(2, 200, 0, 0);
  moveRightLeg(2, -200, 0, 0);
  delay(100);
  moveLeftLeg(2, -200, 200, 100);
  moveRightLeg(0, 200, 200, 100);
}

void moveGrip(int pos)
{ // 80 - grip open
  // 130 -  grip closed
  pos = constrain(pos, 80, 130);
  grip.write(pos);
}

void moveLeftLeg(int leg, int pos, int hieght, int steps)
{
  if (mode != 0)
  {
    int Leg[] = {4, 8, 12};
    int legS = Leg[leg];
    curLval[legS] = curLval[legS] + pos;
    // for (int i = 1; i < (steps + 1); i++)
    // {
    pwmL.writeMicroseconds(legS + 1, curLval[legS + 1] + hieght);
    pwmL.writeMicroseconds(legS + 2, curLval[legS + 2] + hieght);
    delay(steps);
    pwmL.writeMicroseconds(legS, curLval[legS]);
    delay(steps);
    pwmL.writeMicroseconds(legS + 1, curLval[legS + 1]);
    pwmL.writeMicroseconds(legS + 2, curLval[legS + 2]);
    // }
  }
}
void moveRightLeg(int leg, int pos, int hieght, int steps)
{
  if (mode != 0)
  {
    int Leg[] = {12, 8, 4};
    int legS = Leg[leg];
    curRval[legS] = curRval[legS] - pos;
    // for (int i = 1; i < (steps + 1); i++)
    // {
    pwmR.writeMicroseconds(legS + 1, curRval[legS + 1] - hieght);
    pwmR.writeMicroseconds(legS + 2, curRval[legS + 2] - hieght);
    delay(steps);
    pwmR.writeMicroseconds(legS, curRval[legS]);
    delay(steps);
    pwmR.writeMicroseconds(legS + 1, curRval[legS + 1]);
    pwmR.writeMicroseconds(legS + 2, curRval[legS + 2]);
    // }
  }
}

void printCurPos()
{
  Serial.print("Left: ");
  for (int i = 0; i < 16; i++)
  {
    Serial.print(curLval[i]);
    Serial.print(", ");
  }
  Serial.println("");
  Serial.print("Right: ");
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
    if (Data_In == "whoRu")
    {
      Serial.println("ANT,0");
    }
    else if (Data_In == "stepf")
    {
      step_forward();
    }
    else if (Data_In == "stepb")
    {
      step_backward();
    }
    else if (Data_In == "stepr")
    {
      turn_right();
    }
    else if (Data_In == "stepl")
    {
      turn_left();
    }
    else if (Data_In == "stand")
    {
      stand();
    }
    else if (Data_In == "attack")
    {
      attack();
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