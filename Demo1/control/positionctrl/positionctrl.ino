// name: position control
// purpose: control the voltage for 2 motors to reach a desired position, while tracking position and velocity

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
#define DTIME 10 // encoder debounce time (us) - time high or low before encoder state change

#define ROTCTRLRATIO 0.6969 // rotations for position control unit input
#define TICKCTRLRATIO 1115 // ticks for position control unit input
#define WHEELDIAMETER 145 // motor wheel diameter (mm)
#define AXLELENGTH 350 // motor axle length (mm)
#define ROTCIRCUM AXLELENGTH * PI // circumfrance of rotation (mm)
#define BATTVOLT 7.8 // battery voltage

#define MATLAB 0 // print for MATLAB

//////// variables ////////
static float desiredposition[2]; // desired position (rad)
static float actualposition[2]; // actual position (rad)
static float desiredvelocity[2]; // desired velocity (rad/s)
static float actualvelocity[2]; // actual velocity (rad/s)

volatile short tick = 0; // encoder has ticked
volatile short direction[2]; // encoder direction
volatile long ticks[2] = {0, 0}; // encoder ticks
volatile unsigned long ticktime[2]; // time of most recent encoder tick (us)
volatile unsigned long tickperiod[2]; // period between most recent two encoder ticks (us)

float tps[2]; // encoder ticks per second
float tpssqr[2]; // encoder ticks per second^2

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
  static unsigned short dirpin[2] = {M1DIR, M2DIR}; // motor direction pin
  static unsigned short pwmpin[2] = {M1PWM, M2PWM}; // motor pwm pin

  float volt[2]; // motor voltage
  float amp[2]; // motor amps
  amp[0] = analogRead(M1FB) / 1024 / 0.56; // compute motor 1 amps
  amp[1] = analogRead(M2FB) / 1024 / 0.56; // compute motor 2 amps

  static float velkp[2] = {8, 8}; // velocity proportional gain
  static float velerror; // difference between desired and actual velocity (rad/s)
  static float poskp[2] = {2.99, 3.22}; // position proportional gain
  static float poski[2] = {0.33, 0.33}; // position integral gain
  static float poserror; // diffrence between desired and actual position (rad)
  static float prevposerror[2] = {0, 0}; // previous diffrence between desired and actual position (rad)
  static float posinterror[2] = {0, 0}; // integral of difference between desired and actual position (rad)

  static int ctrltime; // time of feedback control calculation (us)
  static int prevctrltime; // previous time of feedback control calculation (us)
  unsigned short ctrlbusy; 
  unsigned short pwm;

  long time = millis();
  static long printtime = 0; // previous print time

  //// external ////
  desiredposition[0] = (1 / ROTCTRLRATIO) * 0.5; 
  //move(-600);
  //rotate(180);

  //// motor feeback ////
  prevctrltime = ctrltime;
  ctrltime = prevctrltime;
  for(int mot = 0; mot < 2; mot++)
  {
    // track:
    actualposition[mot] = ticks2rad(ticks[mot]);
    actualvelocity[mot] = ticks2rad(calctps(tickperiod[mot], ticktime[mot], direction[mot]));
    // control:
    poserror = desiredposition[mot] - actualposition[mot];
    posinterror[mot] = posinterror[mot] + ((prevposerror[mot] + poserror) / 2) * ((ctrltime - prevctrltime) / 1000000);
    desiredvelocity[mot] = poskp[mot] * poserror + poski[mot] * posinterror[mot];
    velerror = desiredvelocity[mot] - actualvelocity[mot];
    volt[mot] = velkp[mot] * velerror;
    if(abs(volt[mot]) > BATTVOLT) // if voltage exceeds battery voltage
    {
      if(volt[mot] > 0) volt[mot] = BATTVOLT; // rail voltage at battery voltage
      else volt[mot] = -BATTVOLT; // rail voltage at battery voltage
    }
    // set direction:
    if(volt[mot] >= 0) digitalWrite(dirpin[mot], CW);
    else if(volt[mot] < 0) digitalWrite(dirpin[mot], CCW);
    // set magnitude:
    pwm = 255 * (abs(volt[mot]) / BATTVOLT);
    analogWrite(pwmpin[mot], pwm);
  }

  //// display ////
  if(MATLAB)
  {
    if((time - starttime) > 1000) desiredvelocity[0] = 1;
    if(((time - starttime) < 3000) && ((time - printtime) > 20)) // if has been less than 1s from start and it has been 20ms since last print
    {
      Serial.print(time - starttime); Serial.print(" "); Serial.print(volt[0]); Serial.print(" "); Serial.println(actualvelocity[0]);
      printtime = time;
    }
  }
  else if((time - printtime) > 800) // if it has been 400ms since last print
  {
    Serial.print("\n\n");
    // motor 1:
    Serial.print("MOTOR 1 at "); Serial.print(volt[0]); Serial.print("V "); Serial.print(amp[0]); Serial.println("A");
    Serial.print("desired position: "); Serial.println(desiredposition[0]);
    Serial.print("actual position: "); Serial.println(actualposition[0]);
    Serial.print("desired velocity: "); Serial.println(desiredvelocity[0]);
    Serial.print("actual velocity: "); Serial.println(actualvelocity[0]);
    Serial.print("tick velocity: "); Serial.println(calctps(tickperiod[0], ticktime[0], direction[0]));
    Serial.print("ticks: "); Serial.println(ticks[0]);
    Serial.print("time: "); Serial.println((float) millis() / 1000);
    // motor 2:
    Serial.print("\n");
    Serial.print("MOTOR 2 at "); Serial.print(volt[1]); Serial.print("V "); Serial.print(amp[1]); Serial.println("A");
    Serial.print("desired position: "); Serial.println(desiredposition[1]);
    Serial.print("actual position: "); Serial.println(actualposition[1]);
    Serial.print("desired velocity: "); Serial.println(desiredvelocity[1]);
    Serial.print("actual velocity: "); Serial.println(actualvelocity[1]);
    Serial.print("tick velocity: "); Serial.println(calctps(tickperiod[1], ticktime[1], direction[1]));
    Serial.print("ticks: "); Serial.println(ticks[1]);
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
    tickperiod[0] = time - ticktime[0];
    ticktime[0] = time;
    //Serial.print("tp: "); Serial.println(tickperiod1); // DEBUG
    //Serial.print("tt: "); Serial.println(ticktime1); // DEBUG
    // position:
    if(clock != data) // clock leading
    {
      direction[0] = CW;
      ticks[0]++;
    }
    else if(clock == data) // data leading
    {
      direction[0] = CCW;
      ticks[0]--;
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
    tickperiod[1] = time - ticktime[1];
    ticktime[1] = time;
    // position:
    if(clock != data) // clock leading
    {
      direction[1] = CW;
      ticks[1]++;
    }
    else if(clock == data) // data leading
    {
      direction[1] = CCW;
      ticks[1]--;
    }
    tick = 1; // indicate a tick has occurred
  }
}

//////// control ////////
void move(float distance)
{
  desiredposition[0] = milim2rad(distance);
  desiredposition[1] = milim2rad(distance);
}
void rotate(float angle)
{
  float milim = ROTCIRCUM * (angle / 360);
  float rot = milim / (WHEELDIAMETER * PI);
  float rad = rot * PI;
  desiredposition[0] = rad;
  desiredposition[1] = -rad;
}

//////// conversions ////////
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
float ticks2rad(int ticks) {return(ticks * (PI / TPR));} // converts ticks to radians
float rad2ticks(int rad) {return(rad * (TPR / PI));} // converts radians to ticks
float rad2milim(float rad) {return(rad * (WHEELDIAMETER * PI));} // converts radians to milimeters
float milim2rad(float mm) {return(mm / (WHEELDIAMETER * PI));} // converts milimeters to radians

// author: jack martin, jack marley
// date: 1/30/2024