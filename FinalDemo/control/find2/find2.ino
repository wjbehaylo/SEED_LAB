// Program Name: Move
// Purpose: Receive an angle, spin, move one foot, and interact with a Raspberry Pi for commands.

//////// Includes ////////
#include <math.h> // Include the math library for mathematical operations.
#include <Wire.h> // Include the Wire library for I2C communication.

//////// Defines ////////
#define DEBUG 0 // Debug mode disabled.
#define PI 3.1416 // Define Pi for calculations involving circles.
#define CW 0 // Define clockwise rotation as 0.
#define CCW 1 // Define counterclockwise rotation as 1.
// Define pin numbers for motor and encoder interfaces.
#define M1DIR 7 // Motor 1 direction control pin.
#define M2DIR 8 // Motor 2 direction control pin.
#define M1PWM 9 // Motor 1 PWM signal pin.
#define M2PWM 10 // Motor 2 PWM signal pin.
#define M1FB A0 // Motor 1 feedback pin for current sensing.
#define M2FB A1 // Motor 2 feedback pin for current sensing.
#define MED 4 // Motor enable/disable pin.
#define CLK1 2 // Encoder 1 clock pin.
#define CLK2 3 // Encoder 2 clock pin.
#define DT1 5 // Encoder 1 data pin.
#define DT2 6 // Encoder 2 data pin.
#define TPR 1600 // Ticks per rotation for the encoder.
#define DTIME 80 // Debounce time in microseconds for encoder signal.

// Define mechanical properties and conversion factors.
#define WHEELDIAMETER 150 // Wheel diameter in millimeters.
#define WHEELCIRCUM WHEELDIAMETER * PI // Calculate wheel circumference.
#define AXLELENGTH 355.6 // Length between wheels in millimeters.
#define ROTCIRCUM AXLELENGTH * PI // Circumference of rotation path.
#define BATTVOLT 7.8 // Operating voltage for the battery.

// Conversion factors for motion control.
#define TICK2FT 0.0009813 // Convert encoder ticks to feet.
#define FT2TICK 1019 // Convert feet to encoder ticks.
#define TICK2ROT 0.095612787 // Convert ticks to degrees of rotation.
#define ROT2TICK 10.458852 // Convert degrees of rotation to encoder ticks.

// Control parameters for motion.
#define DIFFINC 1.28 // Increment for adjusting position difference in ticks.
#define BASEINC 1.28 // Increment for adjusting base position in ticks.

// I2C communication address.
#define MY_ADDR 8 // Address of this device on the I2C bus.
#define ACK_360_TURN_DONE 2 // 360 degree turn done
#define ACK_90_TURN_DONE 3 // 90 degree turn done 
#define ACK_MOVE_COMPLETE 4 // 

// state variable to send to PI
int stateToPi = 1;

//////// Variables ////////
//typedef state 
enum State {WAIT, FULLSEARCH, SEARCH, ROTATE, MOVE, DONE}; // State machine for different modes of operation.
State state = SEARCH; // maybe delete
short ctrlBusy = 0; // Flag to indicate if a control operation is currently ongoing.
short markerCount = 0; // Counter for the number of markers found.
long startTime; // Record the start time of the loop.

// Control variables for motion.
float setAngle = 0; // Desired rotation angle in degrees.
float setPosition = 0; // Desired linear movement in feet.
float setPositionDiff; // Tick difference for precise positioning.
float setPositionBase; // Base tick position for broad movements.
float angKp = 0; // Proportional gain for angular control.
float angKi = 0; // Integral gain for angular control.
float posKp = 0; // Proportional gain for positional control.
float posKi = 0; // Integral gain for positional control.
float veldKp = 0; // Velocity differential proportional gain.
float velbKp = 0; // Velocity base proportional gain.
float maxVolt = 0; // Maximum allowable voltage for motors.
float desiredPositionDiff = 0; // Desired positional difference in ticks for target position.
float desiredPositionBase = 0; // Base positional difference in ticks for target position.
float desiredVelocityDiff; // Desired velocity difference in ticks per second.
float desiredVelocityBase; // Desired base velocity in ticks per second.
float voltageDiff; // Voltage difference applied to motors for differential control.
float voltageBase; // Base voltage applied to motors.
float posDiffError = 0; // Current error in position difference.
float prevPosDiffError = 0; // Previous error in position difference.
float posDiffIntError = 0; // Integrated error for position difference.
float maxPosDiffIntError = 80; // Maximum allowed integrated error for position difference.
float posBaseError = 0; // Current error in base position.
float prevPosBaseError = 0; // Previous error in base position.
float posBaseIntError = 0; // Integrated error for base position.
float maxPosBaseIntError = 80; // Maximum allowed integrated error for base position.
float velDiffError; // Current error in velocity difference.
float velBaseError; // Current error in base velocity.
long ctrltime; // Timestamp for the last control action.
long prevCtrltime; // Timestamp for the previous control action.

// Encoder readings and calculated values.
volatile short tick = 0; // Flag to indicate that an encoder has registered a tick.
volatile short direction[2]; // Array to hold direction of rotation for each encoder.
volatile long ticks[2] = {0, 0}; // Array to count ticks from each encoder.
volatile unsigned long ticktime[2]; // Array to hold the timestamp of the last tick for each encoder.
volatile unsigned long tickperiod[2]; // Array to hold the time period between the last two ticks for each encoder.

// Computed motion parameters.
float tps[2]; // Array to hold the calculated ticks per second for each encoder.
float tpssqr[2]; // Array to hold the square of ticks per second for future use.
static float actualAngle; // Variable to hold the calculated actual angle based on encoder readings.
static float actualPosition; // Variable to hold the calculated actual position based on encoder readings.
static float actualPositionDiff; // Variable to hold the calculated difference in position based on encoder readings.
static float actualPositionBase; // Variable to hold the base position calculated from encoder readings.
static float actualVelocity[2]; // Array to hold the calculated velocity for each encoder.
static float actualVelocityDiff; // Variable to hold the calculated velocity difference between the two encoders.
static float actualVelocityBase; // Variable to hold the base velocity calculated from encoder readings.
static float voltage[2]; // Array to hold the voltage applied to each motor.
float amperage[2]; // Array to hold the current draw of each motor.

// I2C communication variables.
volatile uint8_t offset = 0; // Variable to hold the offset in the received I2C message.
volatile uint8_t instruction[32] = {0}; // Array to hold the instructions received via I2C.
volatile uint8_t msgLength = 0; // Variable to hold the length of the received I2C message.
volatile uint8_t start = 0; // Flag to indicate that the start command has been received via I2C.
volatile uint8_t markerRecived = 0; // Flag to indicate that a marker position has been received.
volatile float recivedAngle = 0; // Variable to hold the received angle of a marker.
volatile float recivedPosition = 0; // Variable to hold the received position of a marker.
volatile float markerAngle = 0; // Variable to hold the angle of the current marker being targeted.
volatile float markerPosition = 0; // Variable to hold the position of the current marker being targeted.
int flagToPi = 0; // Flag to signal to the Raspberry Pi that the Arduino is ready for further instructions.

// Union for converting between bytes and floats for I2C communication.
union FloatUnion 
{
  uint8_t bytes[4]; // Array to hold bytes that represent a float.
  float floatValue; // Float representation of the bytes.
};

volatile FloatUnion byteFloat; // maybe delete

//////// Function Prototypes ////////
int ticks2milim(); // Function to convert ticks to millimeters.
float milim2rad(float mm);

//////// Setup Routine ////////
void setup() 
{
  // Initialize pin modes for motors and encoders.
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M1FB, INPUT);
  pinMode(M2FB, INPUT);
  pinMode(MED, OUTPUT);

  // Initialize motor control to off and set initial directions.
  digitalWrite(MED, HIGH); // Enable motor driver.
  digitalWrite(M1DIR, CW); // Set motor 1 direction to clockwise.
  digitalWrite(M2DIR, CCW); // Set motor 2 direction to counterclockwise.
  analogWrite(M1PWM, 0); // Set motor 1 PWM to 0 (stop).
  analogWrite(M2PWM, 0); // Set motor 2 PWM to 0 (stop).

  // Initialize serial communication at 2 Mbps for debugging.
  Serial.begin(2000000);

  // Initialize I2C communication as a slave with address MY_ADDR.
  Wire.begin(MY_ADDR);
  Wire.onRequest(requestFromPi); // Register callback for sending data to Pi.
  Wire.onReceive(receiveFromPi); // Register callback for receiving data from Pi.

  // Attach interrupts to encoder pins to handle changes.
  attachInterrupt(digitalPinToInterrupt(CLK1), clk1Change, CHANGE); // Setup interrupt for encoder 1.
  attachInterrupt(digitalPinToInterrupt(CLK2), clk2Change, CHANGE); // Setup interrupt for encoder 2.

  // General initial delay for system stabilization.
  delay(1000);
  startTime = millis(); // Record start time of the program for time-based calculations.
}

//////// Main Loop ////////
void loop() 
{
  // Local variable declarations.
  long time = millis(); // Update current time for each loop iteration.
  static long printtime = 0; // Variable to manage print intervals for debugging.
  // static int fullCircleTicks = 0; // make it go in circle

  // State machine handling various operation modes of the robot.
  switch (state) 
  {
    case WAIT:
        // Check if the start command has been received to proceed.
        if(start == true) 
        { 
          state = FULLSEARCH;
        }
        break;

    case FULLSEARCH: 
        // Perform actions related to a full search for markers.
        if(ctrlBusy == 0) 
        { // Check if not currently busy with control operations.
            resetCtrl(); // Reset control variables and parameters.
            resetLocal(); // Reset local parameters and variables.
            setAngle = 360;
            stateToPi = ACK_360_TURN_DONE; /// 360 degree turn done value is 2
            maxVolt = 1.8; // Set maximum voltage for this operation mode.
            setK(18, 8, 180, 3, 0.0008, 0.002); // Set PID gains for control.
            setPositionDiff = rotdeg2ticks(360); // Convert desired angle to tick difference.
            setPositionBase = feet2ticks(setPosition); // Convert desired position to base position in ticks.
            ctrlBusy = 1; // Indicate that control operation is now active.
        }
        if(markerRecived) 
        { // Check if a marker has been detected.
            if(recivedPosition < markerPosition) 
            { // Check if the new marker is closer than the previous one.
                markerAngle = -1 * recivedAngle; // Update to new marker angle.
                markerPosition = recivedPosition; // Update to new marker position.
            }
            markerRecived = 0; // Reset marker received flag.
        } 
        // maybe else if
        if(apCtrlDone(1, 5, 1.1))  /// is this supposed to be apCtrlDone??? you had it as ctrldone but that is not a variable
        { // Check if current control task is complete.
            setK(0, 0, 0, 0, 0, 0); // Reset PID gains to zero.
            ctrlBusy = 0; // Indicate that control is no longer active.
            state = ROTATE; // Transition to MOVE state to approach the marker.
        }
        break;

    case ROTATE:        
        if(ctrlBusy == 0) 
        { // Check if not currently busy with control operations.
            resetCtrl();
            resetLocal();
            maxVolt = 4;
            setK(34, 8, 100, 3, 0.001, 0.002);
            setAngle = actualAngle + markerAngle;
            setPosition = actualPosition;
            setPositionDiff = rotdeg2ticks(setAngle); // Convert angle to tick difference for rotation.
            setPositionBase = feet2ticks(setPosition); // Convert position to base position in ticks.
            ctrlBusy = 1; // Indicate that control operation is now active.
        }
        if(apCtrlDone(1, 0.4, 1.1)) { // Check if the movement control is completed.
          setK(0, 0, 0, 0, 0, 0); // Reset PID gains to zero.
          ctrlBusy = 0; // Indicate that control is no longer active.
          stateToPi = ACK_SECOND_TURN_DONE;
          state = MOVE;
        }
        break;

    case MOVE:
        // Handle movement towards a specific marker.
        if(ctrlBusy == 0) { // Check if not currently busy with control operations.
            resetCtrl(); // Reset control variables and parameters.
            resetLocal(); // Reset local parameters and variables.
            maxVolt = 4; // Set a higher maximum voltage for movement.
            setK(60, 1.4, 48, 0.2, 0.002, 0.001); // Set PID gains for movement.
            setAngle = actualAngle;
            setPosition = actualPosition + markerPosition; // Update position to include movement towards marker.
            setPositionDiff = rotdeg2ticks(setAngle); // Convert angle to tick difference for rotation.
            setPositionBase = feet2ticks(setPosition); // Convert position to base position in ticks.
            ctrlBusy = 1; // Indicate that control operation is now active.

        }
        if(apCtrlDone(1, 0.4, 1.1)) { // Check if the movement control is completed.
            setK(0, 0, 0, 0, 0, 0); // Reset PID gains to zero.
            ctrlBusy = 0; // Indicate that control is no longer active.
            if (markerCount == 0) 
            {
              setAngle = actualAngle - 90;  
              setPosition = actualPosition;
              stateToPi = ACK_90_TURN_DONE;
              state = ROTATE;
            }
            else if(markerCount < 7) state = SEARCH; // If more markers to process, go to SEARCH state.
            else state = DONE; // Otherwise, finish the operation in DONE state.
        }
        break;

    case SEARCH:
        // Search for additional markers after completing an interaction with one.
        
        if(ctrlBusy == 0) 
        { // Check if not currently busy with control operations.
            //setAngle = 360;
            resetCtrl(); // Reset control variables and parameters.
            resetLocal(); // Reset local parameters and variables.
            maxVolt = 7; // Set maximum voltage for search mode.
            setK(18, 8, 180, 3, 0.0008, 0.002); // Set PID gains for search mode.
            desiredVelocityDiff = 1200; // TODO turn into ratio THIS HAS A RADIUS OF 17CM
            desiredVelocityBase = 1100; // TODO turn into ratio
            float circleFraction = 180; // Define the fraction of the circle to complete (half circle here).
            float desiredDistance = (circleFraction / 360.0) * 7469.660; // Calculate the distance based on circle fraction.
            float requiredTicks = milim2ticks(desiredDistance); // Convert the distance to encoder ticks.
            stateToPI = ACK_MOVE_COMPLETE; // completed circle 
            ctrlBusy = 1; // Indicate that control operation is now active.
        }
        else if(markerRecived) 
        { // Check if a new marker has been detected.

            setK(0, 0, 0, 0, 0, 0); // Disable current control settings.
            resetCtrl(); // Reset control variables.
            resetLocal(); // Reset local parameters.
            ctrlBusy = 0; // Indicate that control is no longer active.
            markerCount++; // Increment marker count as a new marker has been found.
            markerAngle = -1 * recivedAngle; // Update to new marker angle. Made negative because we might be be opposite on turning versus PI
            markerPosition = recivedPosition; // Update to new marker position.
            markerRecived = 0; // Reset marker received flag.
            //// send acknowledgement state to pi that we have completed moving 
            //request(); // send information to pi that we are done moving 
            state = ROTATE; // Transition to MOVE state to approach the new marker.
        }

        break;

    case DONE:
        // Conclude all motor activities and clean up.
        setVolt(0, M1PWM, M1DIR); // Turn off motor 1.
        setVolt(0, M2PWM, M2DIR); // Turn off motor 2.
        break;
  }

  // Additional operational code for ramping or adjusting positions and velocities as needed.
  // Adjust position and velocity increments based on set and desired values.
  if(abs(setPositionDiff - desiredPositionDiff) > DIFFINC) { // Check if the difference is greater than the increment threshold.
    if((setPositionDiff - desiredPositionDiff) > 0) 
        desiredPositionDiff += DIFFINC; // Increment desired position difference to approach set position.
    else 
        desiredPositionDiff -= DIFFINC; // Decrement desired position difference to approach set position.
  } else if(abs(setPositionDiff - desiredPositionDiff) > 0) 
        desiredPositionDiff += (setPositionDiff - desiredPositionDiff); // Adjust the last small difference.

  if(abs(setPositionBase - desiredPositionBase) > BASEINC) { // Similar adjustment for base position.
    if((setPositionBase - desiredPositionBase) > 0) 
        desiredPositionBase += BASEINC; // Increment desired base position.
    else 
        desiredPositionBase -= BASEINC; // Decrement desired base position.
  } else if(abs(setPositionBase - desiredPositionBase) > 0) 
        desiredPositionBase += (setPositionBase - desiredPositionBase); // Final adjustment for base position.

  // Monitoring and debug outputs to track real-time operation of the robot.
  long tick0DEBUG = ticks[0]; // Debugging variable to display ticks from encoder 0.
  long tick1DEBUG = ticks[1]; // Debugging variable to display ticks from encoder 1.
  amperage[0] = analogRead(M1FB) / 1024 / 0.56; // Compute current draw of motor 1.
  amperage[1] = analogRead(M2FB) / 1024 / 0.56; // Compute current draw of motor 2.
  actualPositionDiff = (ticks[1] - ticks[0]) / 2; // Calculate position difference from encoder ticks.
  actualPositionBase = ticks[1] - actualPositionDiff; // Calculate base position from encoder ticks.
  actualVelocity[0] = calctps(tickperiod[0], ticktime[0], direction[0]); // Calculate velocity of motor 1 from encoder data.
  actualVelocity[1] = calctps(tickperiod[1], ticktime[1], direction[1]); // Calculate velocity of motor 2 from encoder data.
  actualVelocityDiff = (actualVelocity[1] - actualVelocity[0]) / 2; // Calculate velocity difference between motors.
  actualVelocityBase = actualVelocity[1] - actualVelocityDiff; // Calculate base velocity from the average of both motors.

  // Control logic to manage motor speeds and directions based on current and desired states.
  prevCtrltime = ctrltime; // Update control time variables for timing control loops.
  ctrltime = micros(); // Update current control time.
  if(state == SEARCH) vCtrl(desiredVelocityDiff, desiredVelocityBase); // Activate velocity control if in SEARCH state.
  else if(state != WAIT && state != DONE) apCtrl(desiredPositionDiff, desiredPositionBase); // Activate position control if appropriate.

  // Debug outputs for system monitoring and troubleshooting.
  actualAngle = ticks2rotdeg(actualPositionDiff); // Convert tick difference to rotational degrees.
  actualPosition = ticks2feet(actualPositionBase); // Convert base position ticks to feet.
  if(DEBUG && ((time - printtime) > 1600)) { // Conditional debug output based on timing.
    Serial.print("\n\n");
    Serial.println("ROBOT");
    Serial.print("state: "); Serial.println(state);
    Serial.print("reached: "); Serial.print(reached(setAngle, actualAngle, 1)); Serial.print(" "); Serial.print(reached(setPosition, actualPosition, 0.05)); Serial.print(" "); Serial.println(steady(voltage, 0.7));
    //Serial.print("angle: "); Serial.print(setAngle); Serial.print(" degrees (set), "); Serial.print(ticks2rotdeg(desiredPositionDiff)); Serial.print(" degrees (desired), "); Serial.print(actualAngle); Serial.print(" degrees (actual), "); Serial.print(angle); Serial.println(" degrees (requested)");
    Serial.print("position: "); Serial.print(setPosition); Serial.print(" feet (set), "); Serial.print(ticks2feet(desiredPositionBase)); Serial.print(" feet (desired), "); Serial.print(actualPosition); Serial.println(" feet (actual)");
    Serial.print("difference error: "); Serial.print(posDiffError); Serial.print("p, "); Serial.print(posDiffIntError); Serial.println("i");
    Serial.print("base error: "); Serial.print(posBaseError); Serial.print("p, "); Serial.print(posBaseIntError); Serial.println("i");
    Serial.print("position difference: "); Serial.print(desiredPositionDiff); Serial.print(" ticks (desired), "); Serial.print(actualPositionDiff); Serial.println(" ticks (actual)");
    Serial.print("position base: "); Serial.print(desiredPositionBase); Serial.print(" ticks (desired), "); Serial.print(actualPositionBase); Serial.println(" ticks (actual)");
    Serial.print("velocity difference: "); Serial.print(desiredVelocityDiff); Serial.print(" ticks/s (desired), "); Serial.print(actualVelocityDiff); Serial.println(" ticks/s (actual)");
    Serial.print("velocity base: "); Serial.print(desiredVelocityBase); Serial.print(" ticks/s (desired), "); Serial.print(actualVelocityBase); Serial.println(" ticks/s (actual)");
    Serial.print("voltage difference: "); Serial.print(voltageDiff); Serial.println("V");
    Serial.print("voltage base: "); Serial.print(voltageBase); Serial.println("V");
    Serial.print("motor voltage: "); Serial.print(voltage[0]); Serial.print("V, "); Serial.print(voltage[1]); Serial.println("V");
    // Additional motor diagnostics.
    Serial.print("\n");
    Serial.print("MOTOR 1 at "); Serial.print(voltage[0]); Serial.print("V "); Serial.print(amperage[0]); Serial.println("A");
    Serial.print("actual position: "); Serial.print(tick0DEBUG); Serial.println(" ticks");
    // Serial.print("actual velocity: "); Serial.print(actualVelocity[0]); Serial.println(" ticks");
    // Serial.print("time: "); Serial.println((float) millis() / 1000);
    // Additional motor diagnostics for the second motor.
    Serial.print("\n");
    Serial.print("MOTOR 2 at "); Serial.print(voltage[1]); Serial.print("V "); Serial.print(amperage[1]); Serial.println("A");
    Serial.print("actual position: "); Serial.print(tick1DEBUG); Serial.println(" ticks");
    // Serial.print("actual velocity: "); Serial.print(actualVelocity[1]); Serial.println(" ticks");
    // Serial.print("time: "); Serial.println((float) millis() / 1000);
    printtime = time; // Update last print time for debug output control.
  }
}
// deez nutz //
//////// Interrupt Handlers ////////
// Handle changes in encoder signals for motor 1.
void clk1Change()
{
  // Local variable declarations for interrupt handling.
  static short encoder = digitalRead(CLK1); // Current state of the encoder.
  short clock; // Current state of the clock signal.
  short data; // Current state of the data signal.
  unsigned long time; // Current timestamp for the interrupt.
  static unsigned long rtime = 0; // Timestamp for the last rising edge.
  static unsigned long ftime = 0; // Timestamp for the last falling edge.
  short update = 0; // Flag to indicate if an update is needed based on signal changes.

  // Process encoder signal changes.
  time = micros(); // Get current time in microseconds.
  clock = digitalRead(CLK1); // Read the current state of the clock signal.
  data = digitalRead(DT1); // Read the current state of the data signal.

  // Debounce logic to ensure reliable signal processing.
  if((encoder == LOW) && (clock == HIGH)) { // Check for a rising edge.
    if((time - ftime) < DTIME) return; // Return if the time since the last falling edge is less than the debounce time.
    else { // Process a valid rising edge.
      rtime = time; // Update the rising edge timestamp.
      encoder = HIGH; // Update the encoder state to high.
      update = 1; // Set the update flag.
    }
  } else if((encoder == HIGH) && (clock == LOW)) { // Check for a falling edge.
    if((time - rtime) < DTIME) return; // Return if the time since the last rising edge is less than the debounce time.
    else { // Process a valid falling edge.
      ftime = time; // Update the falling edge timestamp.
      encoder = LOW; // Update the encoder state to low.
      update = 1; // Set the update flag.
    }
  }

  // Update encoder tick tracking based on valid signal changes.
  if(update == 1) {
    // Update tick period and time tracking for encoder 1.
    tickperiod[0] = time - ticktime[0]; // Calculate the period between the current and last ticks.
    ticktime[0] = time; // Update the last tick time.
    // Determine the direction of rotation based on the relationship between clock and data signals.
    if(clock != data) { // If the clock signal leads the data signal.
      direction[0] = CW; // Set direction to clockwise.
      ticks[0]++; // Increment the tick count.
    } else if(clock == data) { // If the data signal leads the clock signal.
      direction[0] = CCW; // Set direction to counterclockwise.
      ticks[0]--; // Decrement the tick count.
    }
    tick = 1; // Set the tick flag to indicate a tick occurrence.
  }
}
// Handle changes in encoder signals for motor 2.
void clk2Change()
{
  // Local variable declarations for interrupt handling.
  static short encoder = digitalRead(CLK2); // Current state of the encoder.
  short clock; // Current state of the clock signal.
  short data; // Current state of the data signal.
  unsigned long time; // Current timestamp for the interrupt.
  static unsigned long rtime = 0; // Timestamp for the last rising edge.
  static unsigned long ftime = 0; // Timestamp for the last falling edge.
  short update = 0; // Flag to indicate if an update is needed based on signal changes.

  // Process encoder signal changes.
  time = micros(); // Get current time in microseconds.
  clock = digitalRead(CLK2); // Read the current state of the clock signal.
  data = digitalRead(DT2); // Read the current state of the data signal.

  // Debounce logic to ensure reliable signal processing.
  if((encoder == LOW) && (clock == HIGH)) { // Check for a rising edge.
    if((time - ftime) < DTIME) return; // Return if the time since the last falling edge is less than the debounce time.
    else { // Process a valid rising edge.
      rtime = time; // Update the rising edge timestamp.
      encoder = HIGH; // Update the encoder state to high.
      update = 1; // Set the update flag.
    }
  } else if((encoder == HIGH) && (clock == LOW)) { // Check for a falling edge.
    if((time - rtime) < DTIME) return; // Return if the time since the last rising edge is less than the debounce time.
    else { // Process a valid falling edge.
      ftime = time; // Update the falling edge timestamp.
      encoder = LOW; // Update the encoder state to low.
      update = 1; // Set the update flag.
    }
  }

  // Update encoder tick tracking based on valid signal changes.
  if(update == 1) {
    // Update tick period and time tracking for encoder 2.
    tickperiod[1] = time - ticktime[1]; // Calculate the period between the current and last ticks.
    ticktime[1] = time; // Update the last tick time.
    // Determine the direction of rotation based on the relationship between clock and data signals.
    if(clock != data) { // If the clock signal leads the data signal.
      direction[1] = CCW; // Set direction to counterclockwise.
      ticks[1]--; // Decrement the tick count.
    } else if(clock == data) { // If the data signal leads the clock signal.
      direction[1] = CW; // Set direction to clockwise.
      ticks[1]++; // Increment the tick count.
    }
    tick = 1; // Set the tick flag to indicate a tick occurrence.
  }
}

//////// Control Functions ////////
// Set the PID control gains for various control tasks.
void setK(float aKp, float aKi, float pKp, float pKi, float vdKp, float vbKp)
{
  // Assign provided gains to global variables for control.
  angKp = aKp; // Set angular proportional gain.
  angKi = aKi; // Set angular integral gain.
  posKp = pKp; // Set positional proportional gain.
  posKi = pKi; // Set positional integral gain.
  veldKp = vdKp; // Set velocity differential proportional gain.
  velbKp = vbKp; // Set velocity base proportional gain.
}

// Apply a voltage to a motor using PWM based on the desired voltage and direction.
void setVolt(float voltage, short mpin, short dpin)
{
  // Local variable declaration.
  short pwm; // Variable to hold the calculated PWM value.
  
  // Set motor direction based on the sign of the voltage.
  if(voltage >= 0) digitalWrite(dpin, CW); // Set direction to clockwise if voltage is positive.
  else if(voltage < 0) digitalWrite(dpin, CCW); // Set direction to counterclockwise if voltage is negative.

  // Calculate the PWM value based on the absolute value of the voltage.
  pwm = 255 * (abs(voltage) / BATTVOLT); // Calculate PWM value as a fraction of maximum voltage.
  
  // Apply the calculated PWM value to the motor.
  analogWrite(mpin, pwm); // Write the PWM value to the motor's PWM pin.
}

// Control the velocity of the robot based on desired differences and base velocities.
void vCtrl(float desiredVelocityDiff, float desiredVelocityBase)
{
  // Calculate the current velocity errors.
  velDiffError = desiredVelocityDiff - actualVelocityDiff; // Calculate difference in desired and actual velocity differences.
  velBaseError = desiredVelocityBase - actualVelocityBase; // Calculate difference in desired and actual base velocities.
  
  // Calculate the voltage adjustments needed based on velocity errors and control gains.
  voltageDiff = veldKp * velDiffError; // Calculate voltage difference needed.
  voltageBase = velbKp * velBaseError; // Calculate base voltage needed.
  
  // Calculate the final voltages to apply to each motor.
  voltage[0] = rail(voltageBase - voltageDiff, maxVolt); // Calculate voltage for motor 1.
  voltage[1] = rail(voltageBase + voltageDiff, maxVolt); // Calculate voltage for motor 2.
  
  // Apply the calculated voltages to the motors.
  setVolt(voltage[0], M1PWM, M1DIR); // Set voltage for motor 1.
  setVolt(voltage[1], M2PWM, M2DIR); // Set voltage for motor 2.
}

// Control the position of the robot based on desired differences and base positions.
void apCtrl(float desiredPositionDiff, float desiredPositionBase)
{
  // Update previous errors for integral control calculation.
  prevPosDiffError = posDiffError; // Store the previous position difference error.
  posDiffError = desiredPositionDiff - actualPositionDiff; // Calculate the new position difference error.
  
  // Calculate integral error if within threshold, otherwise reduce it to prevent wind-up.
  if(abs(posDiffError) < rotdeg2ticks(10)) 
    posDiffIntError += ((prevPosDiffError + posDiffError) / 2) * ((float)(ctrltime - prevCtrltime) / 1000000); // Update integral error for position difference.
  else 
    posDiffIntError /= 2; // Reduce integral error to prevent wind-up.
  
  // Repeat the process for base position control.
  prevPosBaseError = posBaseError; // Store the previous base position error.
  posBaseError = desiredPositionBase - actualPositionBase; // Calculate the new base position error.
  
  // Calculate integral error for base position if within threshold, otherwise reduce it.
  if(abs(posBaseError) < feet2ticks(0.5)) 
    posBaseIntError += ((prevPosBaseError + posBaseError) / 2) * ((float)(ctrltime - prevCtrltime) / 1000000); // Update integral error for base position.
  else 
    posBaseIntError /= 2; // Reduce integral error to prevent wind-up.
  
  // Calculate desired velocities based on positional errors and control gains.
  desiredVelocityDiff = angKp * posDiffError + angKi * posDiffIntError; // Calculate desired velocity difference.
  desiredVelocityBase = posKp * posBaseError + posKi * posBaseIntError; // Calculate desired base velocity.
  
  // Apply velocity control with the calculated desired velocities.
  vCtrl(desiredVelocityDiff, desiredVelocityBase);
}

// Placeholder for circular control, which would handle circular movements specifically.
void cirCtrl(float desiredPosition, float velocityRatio, short direction)
{
  // This function is a placeholder to demonstrate where circular control logic would be implemented.
}

// Reset control-related variables to initial states.
void resetCtrl()
{
  // Reset all previous and integral errors for clean control operation.
  prevPosDiffError = 0; // Reset previous position difference error.
  posDiffIntError = 0; // Reset integral position difference error.
  prevPosBaseError = 0; // Reset previous base position error.
  posBaseIntError = 0; // Reset integral base position error.
  ctrltime = 0; // Reset last control time.
  prevCtrltime = 0; // Reset previous control time.
}

// Reset local variables related to position and angle to match current actual values.
void resetLocal()
{
  // Update set positions and angles to current actual values for consistency.
  setAngle = actualAngle; // Update set angle to current actual angle.
  setPositionDiff = actualPositionDiff; // Update set position difference to current actual position difference.
  desiredPositionDiff = actualPositionDiff; // Align desired position difference with current actual value.
  setPosition = actualPosition; // Update set position to current actual position.
  setPositionBase = actualPositionBase; // Update set base position to current actual base position.
  desiredPositionBase = actualPositionBase; // Align desired base position with current actual value.
}

// Check if the control operation for positioning or angling is complete.
short apCtrlDone(float desAngThesh, float desPosThresh, float voltThresh)
{
  // Determine if the desired state has been reached within specified thresholds.
  if(reached(setAngle, actualAngle, desAngThesh) && reached(setPosition, actualPosition, desPosThresh) && steady(voltage, voltThresh)) 
    return(1); // Return 1 if all conditions are met.
  else 
    return(0); // Return 0 if any conditions are not met.
}

// Check if the actual value has reached the set value within a specified threshold.
short reached(float set, float actual, float maxDifference)
{
  // Calculate the absolute difference between set and actual values.
  float difference = abs(set - actual);
  
  // Check if the difference is within the allowed threshold.
  if(difference < maxDifference) 
    return(1); // Return 1 if within threshold.
  else 
    return(0); // Return 0 if outside the threshold.
}

// Check if the applied voltages are stable within a specified threshold.
short steady(float voltage[2], float threshold)
{
  // Check if both voltages are below the stability threshold.
  if((abs(voltage[0]) < threshold) && (abs(voltage[1]) < threshold)) 
    return(1); // Return 1 if both are stable.
  else 
    return(0); // Return 0 if either is unstable.
}

// Limit the value to a maximum allowable value, adjusting as necessary.
float rail(float value, float maxValue)
{
  // Check if the absolute value exceeds the maximum allowed.
  if(abs(value) > maxValue) {
    // Adjust the value to the maximum allowed, maintaining the sign.
    if(value > 0) 
      value = maxValue; // Limit to positive maximum.
    else 
      value = -maxValue; // Limit to negative maximum.
  }
  // Return the adjusted or original value.
  return(value);
}

//////// Communication Functions ////////
// Handle receiving data from the Raspberry Pi via I2C.
void receiveFromPi()
{
  // Read the first byte from the I2C bus, which typically indicates the command or data type.
  offset = Wire.read(); // Read the offset byte which determines the type of data following.
  Serial.println("i2c");
  // Read remaining bytes from the I2C bus until no more are available.
  while(Wire.available()) {
    // Store each byte in the instruction array and increment the message length counter.
    instruction[msgLength] = Wire.read(); // Store the byte in the instruction array.
    msgLength++; // Increment the counter for each byte read.
  }

  // Process the received message based on its length and content.
  if (msgLength == 1 && instruction[0] == 0) 
  {
    // If a single byte with value 0 is received, interpret it as a start command.
    start = 1; // Set the start flag to initiate operations.
  }
  else if (msgLength == 8) 
  {
    // If eight bytes are received, interpret the first four as one floating-point value and the next four as another.
    // This is typically used to receive two floating-point numbers in sequence.
    byteFloat.bytes[0] = instruction[7]; // Assign the bytes in reverse order to convert to float.
    byteFloat.bytes[1] = instruction[6];
    byteFloat.bytes[2] = instruction[5];
    byteFloat.bytes[3] = instruction[4];
    recivedPosition = byteFloat.floatValue; // Convert the first four bytes to a float representing an angle.
    recivedPosition = recivedPosition * 3.28084; // conversion from meter we are getting from PI into feet
    byteFloat.bytes[0] = instruction[3]; // Repeat for the next four bytes.
    byteFloat.bytes[1] = instruction[2];
    byteFloat.bytes[2] = instruction[1];
    byteFloat.bytes[3] = instruction[0];
    recivedAngle = byteFloat.floatValue; // Convert the second set of four bytes to a float representing a position.
    markerRecived = 1; // Set the marker received flag to indicate that new marker data has been received.
    Serial.println(recivedAngle);
    Serial.println(recivedPosition);
  }
  // Reset the message length for the next reception.
  msgLength = 0;
}

// Handle requests from the Raspberry Pi for data via I2C.
void requestFromPi()
{
  // Read the first byte from the I2C bus when the Raspberry Pi requests data.
  // This byte often specifies the type of data the Raspberry Pi is requesting.
  offset = Wire.read(); // Read the first byte to determine the requested data type.

  // Read any additional data if available. This part is typically used to receive a full message or command.
  while (Wire.available()) {
    // Read each subsequent byte and store the first in the instruction array.
    // This example assumes only one byte of significant data follows the initial request, which is not typical in complex I2C communications.
    instruction[0] = Wire.read(); // Read and store the byte.
    msgLength++; // Increment the message length counter.
  }
}

void sendStateToPi() 
{
    Wire.write(stateToPi);  // Send the current state as a single byte
}
// Send an acknowledgment to the Raspberry Pi, typically to confirm receipt of a command or data.
//void sendAckToPi()
//{
//  // Check if a flag is set to send an acknowledgment.
//  if(flagToPi == 1) {
//    // If the flag is set, send a byte back to the Raspberry Pi to indicate acknowledgment.
//   Wire.write('1'); // Send a byte representing the acknowledgment.
//  }
//  // Reset the flag after sending the acknowledgment.
//  flagTo Pi = 0;
//}

//////// Conversion Functions ////////
// Convert tick period and tick time to ticks per second for encoder calculations.
float calctps(unsigned long tickperiod, unsigned long ticktime, short direction)
{
  // Local variable declaration.
  long time = micros(); // Get the current time in microseconds.

  // Adjust the tick period if the current period has exceeded the last recorded tick period.
  if((time - ticktime) > tickperiod) 
    tickperiod = time - ticktime; // Update the tick period to the current incomplete period if it's longer than the last complete period.

  // Calculate ticks per second based on the tick period.
  float tps = 1 / ((float)tickperiod / 1000000); // Calculate ticks per second from the tick period.

  // Adjust the sign of ticks per second based on the direction of rotation.
  if(direction == CCW) 
    tps = -tps; // If the direction is counterclockwise, make ticks per second negative.

  // Return the calculated ticks per second.
  return(tps);
}

// Convert encoder ticks to degrees of robot rotation.
float ticks2rotdeg(float ticks)
{
  // Convert ticks directly to degrees using a predefined conversion factor.
  return(ticks * TICK2ROT); // Multiply ticks by the conversion factor to get degrees.
}

// Convert degrees of robot rotation to encoder ticks.
float rotdeg2ticks(float deg)
{
  // Convert degrees directly to ticks using a predefined conversion factor.
  return(deg * ROT2TICK); // Multiply degrees by the conversion factor to get ticks.
}

// Convert encoder ticks to feet for linear measurements.
float ticks2feet(float ticks)
{
  // Convert ticks directly to feet using a predefined conversion factor.
  return(ticks * TICK2FT); // Multiply ticks by the conversion factor to get feet.
}

// Convert feet to encoder ticks for reverse calculations.
float feet2ticks(float feet)
{
  // Convert feet directly to ticks using a predefined conversion factor.
  return(feet * FT2TICK); // Multiply feet by the conversion factor to get ticks.
}

// Convert encoder ticks to radians for angular calculations.
float ticks2rad(float ticks)
{
  // Convert ticks directly to radians using a predefined conversion factor.
  return(ticks * (PI / TPR)); // Multiply ticks by the conversion factor (based on Pi and ticks per rotation) to get radians.
}

// Convert radians to encoder ticks for reverse angular calculations.
float rad2ticks(float rad)
{
  // Convert radians directly to ticks using a predefined conversion factor.
  return(rad * (TPR / PI)); // Multiply radians by the conversion factor (based on ticks per rotation and Pi) to get ticks.
}

// Convert radians to millimeters for linear distance calculations in circular motions.
float rad2milim(float rad)
{
  // Convert radians directly to millimeters using the wheel diameter and Pi.
  return(rad * (WHEELDIAMETER * PI)); // Multiply radians by the product of wheel diameter and Pi to get millimeters.
}

// Convert millimeters to radians for reverse calculations in circular motions.
float milim2rad(float mm)
{
  // Convert millimeters directly to radians using the wheel diameter and Pi.
  return(mm / (WHEELDIAMETER * PI)); // Divide millimeters by the product of wheel diameter and Pi to get radians.
}

// Function to convert millimeters to encoder ticks
int milim2ticks(float mm) 
{
    float wheelCircumference = PI * WHEELDIAMETER;
    float mmPerTick = wheelCircumference / TPR;
    return (mm / mmPerTick);
}

// Program Authorship and Metadata
// Authors: Jack Martin, Jack Marley, Walter Behaylo, Holden Drew
// Creation Date: March 29, 2024
// This program is designed for educational and development purposes in robotic control systems involving positional and angular movements, interfacing with external devices via I2C, and providing a robust platform for testing various motion control scenarios in a simulated or real-world environment.
