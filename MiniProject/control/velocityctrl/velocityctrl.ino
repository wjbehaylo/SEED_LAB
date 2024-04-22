// name: velocity control
// purpose: control the voltage for 2 motors to reach a desired velocity, while tracking position and velocity

//////// includes ////////
#include <math.h>

//////// defines ////////
#define PI 3.1416
#define CW 0 // clockwise
#define CCW 1 // counterclockwise

#define M1DIR 7 // motor 1 direction pin
#define M2DIR 8 // motor 2 direction pin
#define M1PWM 9 // motor 1 pwm pin
#define M2PWM 10 // motor 2 pwm pin
#define M1FB A0 // motor 1 feeback pin
#define M2FB A1 // motor 2 feedback pin
#define MED 4 // motor enable/disable pin

#define CLK1 2 // encoder 1 clock pin
#define CLK2 3 // encoder 2 clock pin
#define DT1 5 // encoder 1 data pin
#define DT2 6 // encoder 2 data pin
#define TPR 1600 // encoder ticks per rotation
#define DTIME 800 // encoder debounce time us (time high or low before encoder state change)

#define BATTVOLT 7.8 // battery voltage

#define MATLAB 0 // print for MATLAB

//////// variables ////////
volatile short tick = 0; // encoder has ticked
volatile short direction1; // encoder 1 direction
volatile short direction2; // encoder 2 direction
volatile long ticks1 = 0; // encoder 1 ticks
volatile long ticks2 = 0; // encoder 2 ticks
volatile unsigned long ticktime1; // time of most recent encoder 1 tick (us)
volatile unsigned long ticktime2; // time of most recent encoder 2 tick (us)
volatile unsigned long tickperiod1; // period between most recent two encoder 1 ticks (us)
volatile unsigned long tickperiod2; // period between most recent two encoder 2 ticks (us)

float tps1; // encoder 1 ticks per second
float tps2; // encoder 2 ticks per second
float tpssqr1; // encoder 1 ticks per second^2
float tpssqr2; // encoder 2 ticks per second^2

long starttime; // loop start time (ms)

//////// headers ////////
int ticks2milim();

//////// setup ////////
void setup() 
{  
  //// pins ////
  // motor:
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M1FB, INPUT);
  pinMode(M2FB, INPUT);
  pinMode(MED, OUTPUT);
  // encoder:
  pinMode(CLK1, INPUT_PULLUP);
  pinMode(CLK2, INPUT_PULLUP);
  pinMode(DT1, INPUT_PULLUP);
  pinMode(DT2, INPUT_PULLUP);
  //// motor ////
  digitalWrite(MED, 1); // enable motor voltage
  digitalWrite(M1DIR, 0); // set motor voltage direction
  digitalWrite(M2DIR, 1); // set motor voltage direction
  analogWrite(M1PWM, 0); // set motor voltage magnitude
  analogWrite(M2PWM, 0); // set motor voltage magnitude
  //// communication ////
  Serial.begin(9600); // set UART bits per second
  //// interrupts ////
  attachInterrupt(digitalPinToInterrupt(CLK1), clk1Change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK2), clk2Change, CHANGE);
  //// stablise ////
  delay(1000);
  starttime = millis();
}

//////// loop ////////
void loop() 
{
  //// variables ////  
  float volt1; // motor 1 voltage
  float volt2; // motor 2 voltage
  float amp1 = analogRead(M1FB) / 1024 / 0.56; // motor 1 amps
  float amp2 = analogRead(M2FB) / 1024 / 0.56; // motor 2 amps

  static float desiredspeed1 = -2; // desired speed (rad/s)
  static float desiredspeed2 = -0.5; // desired speed (rad/s)
  static float actualspeed1; // actual speed (rad/s)
  static float actualspeed2; // actual speed (rad/s)
  static float kp = 8; // proportial control
  static float error; // difference between desired and actual speed (rad/s)
  static unsigned short pwm;

  long time = millis();
  static long printtime = 0; // previous print time

  //// motor 1 feeback ////
  actualspeed1 = calctps(tickperiod1, ticktime1, direction1) * (PI / TPR);
  error = desiredspeed1 - actualspeed1;
  volt1 = kp * error;
  if((abs(volt1) > BATTVOLT) && (volt1 > 0)) volt1 = BATTVOLT; // rail voltage at battery voltage
  if((abs(volt1) > BATTVOLT) && (volt1 < 0)) volt1 = -BATTVOLT; // rail voltage at battery voltage
  // set direction:
  if(volt1 >= 0) digitalWrite(M1DIR, CW);
  else if(volt1 < 0) digitalWrite(M1DIR, CCW);
  // set magnitude:
  pwm = 255 * (abs(volt1) / BATTVOLT);
  analogWrite(M1PWM, pwm);

  //// motor 2 feeback ////
  actualspeed2 = calctps(tickperiod2, ticktime2, direction2) * (PI / TPR);
  error = desiredspeed2 - actualspeed2;
  volt2 = kp * error;
  if((abs(volt2) > BATTVOLT) && (volt2 > 0)) volt2 = BATTVOLT; // rail voltage at battery voltage
  if((abs(volt2) > BATTVOLT) && (volt2 < 0)) volt2 = -BATTVOLT; // rail voltage at battery voltage
  // set direction:
  if(volt2 >= 0) digitalWrite(M2DIR, CW);
  else if(volt2 < 0) digitalWrite(M2DIR, CCW);
  // set magnitude:
  pwm = 255 * (abs(volt2) / BATTVOLT);
  analogWrite(M2PWM, pwm);

  //// display ////
  if(MATLAB)
  {
    if((time - starttime) > 1000) desiredspeed1 = 1;
    if(((time - starttime) < 3000) && ((time - printtime) > 20)) // if has been less than 1s from start and it has been 20ms since last print
    {
      Serial.print(time - starttime); Serial.print(" "); Serial.print(volt1); Serial.print(" "); Serial.println(actualspeed1);
      printtime = time;
    }
  }
  else if((time - printtime) > 800) // if it has been 400ms since last print
  {
    Serial.print("\n\n");
    // motor 1:
    Serial.print("MOTOR 1 at "); Serial.print(volt1); Serial.print("V "); Serial.print(amp1); Serial.println("A");
    Serial.print("desired speed: "); Serial.println(desiredspeed1);
    Serial.print("actual speed: "); Serial.println(actualspeed1);
    Serial.print("tick speed: "); Serial.println(calctps(tickperiod1, ticktime1, direction1));
    Serial.print("ticks: "); Serial.println(ticks1);
    Serial.print("time: "); Serial.println((float) millis() / 1000);
    // motor 2:
    Serial.print("\n");
    Serial.print("MOTOR 2 at "); Serial.print(volt2); Serial.print("V "); Serial.print(amp2); Serial.println("A");
    Serial.print("desired speed: "); Serial.println(desiredspeed2);
    Serial.print("actual speed: "); Serial.println(actualspeed2);
    Serial.print("tick speed: "); Serial.println(calctps(tickperiod2, ticktime2, direction2));
    Serial.print("ticks: "); Serial.println(ticks2);
    Serial.print("time: "); Serial.println((float) millis() / 1000);
    printtime = time;
  }
}

//////// interupts ////////
void clk1Change()
{
  //// variables ////
  static short encoder = digitalRead(CLK1); // state of encoder
  short clock; // encoder clock signal
  short data; // encoder data signal
  unsigned long time; // time of current interrupt
  static unsigned long rtime = 0; // time of last rising edge
  static unsigned long ftime = 0; // time of last falling edge
  short update = 0;

  //// update signals ////
  time = micros();
  clock = digitalRead(CLK1);
  data = digitalRead(DT1);

  //// debounce ////
  if((encoder == LOW) && (clock == 1)) // if looking for rising edge
  {
    if((time - ftime) < DTIME) return; // if time since falling edge is less than DTIMEus return
    else // rising edge found
    {
      rtime = time;
      encoder = HIGH;
      update = 1;
    }
  }
  else if((encoder == HIGH) && (clock == 0)) // if looking for falling edge
  {
    if((time - rtime) < DTIME) return; // if time since rising edge is less than DTIMEus return
    else // falling edge found
    {
      ftime = time;
      encoder = LOW;
      update = 1;
    }
  }

  //// track ticks and time ////
  if(update == 1)
  {
    // time:
    tickperiod1 = time - ticktime1;
    ticktime1 = time;
    //Serial.print("tp: "); Serial.println(tickperiod1); // DEBUG
    //Serial.print("tt: "); Serial.println(ticktime1); // DEBUG
    // position:
    if(clock != data) // clock leading
    {
      direction1 = CW;
      ticks1++;
    }
    else if(clock == data) // data leading
    {
      direction1 = CCW;
      ticks1--;
    }
    tick = 1; // indicate a tick has occurred
  }
}
void clk2Change()
{
  //// variables ////
  static short encoder = digitalRead(CLK2); // state of encoder
  short clock; // encoder clock signal
  short data; // encoder data signal
  unsigned long time; // time of current interrupt
  static unsigned long rtime = 0; // time of last rising edge
  static unsigned long ftime = 0; // time of last falling edge
  short update = 0;

  //// update signals ////
  time = micros();
  clock = digitalRead(CLK2);
  data = digitalRead(DT2);

  //// debounce ////
  if((encoder == LOW) && (clock == 1)) // if looking for rising edge
  {
    if((time - ftime) < DTIME) return; // if time since falling edge is less than DTIMEus return
    else // rising edge found
    {
      rtime = time;
      encoder = HIGH;
      update = 1;
    }
  }
  else if((encoder == HIGH) && (clock == 0)) // if looking for falling edge
  {
    if((time - rtime) < DTIME) return; // if time since rising edge is less than DTIMEus return
    else // falling edge found
    {
      ftime = time;
      encoder = LOW;
      update = 1;
    }
  }

  //// track ticks and time ////
  if(update == 1)
  {
    // time:
    tickperiod2 = time - ticktime2;
    ticktime2 = time;
    // position:
    if(clock != data) // clock leading
    {
      direction2 = CW;
      ticks1++;
    }
    else if(clock == data) // data leading
    {
      direction2 = CCW;
      ticks1--;
    }
    tick = 1; // indicate a tick has occurred
  }
}

//////// functions ////////
float calctps(unsigned long tickperiod, unsigned long ticktime, short direction) // calculates ticks per second from tick period, tick time, and time
{
  //// variables ////
  long time = micros();
  float tps;

  //// calculate ////
  if((time - ticktime) > tickperiod) // if current tick period (incomplete) has exceeded previous tick period
    tickperiod = time - ticktime; // update tick period
  tps = 1 / ((float)tickperiod / 1000000); // 1e-6 seconds per 1 micro second
  if(direction == CCW) tps = -tps;

  //// return ////
  return(tps);
}
unsigned short speed2volt(float desiredspeed, float actualspeed) {} // feedback control system
int ticks2rad() {} // converts ticks to radians
int rad2milim() {} // converts radians to milimeters

// author: jack martin
// date: 1/30/2024