/*
State machine m2 is for motion control
the states are as follows:
1: Stopped
2: Forward 12 o'clock - columnCounter ++
3: Turn left 12 to 9 o'clock
4: Forward 9 o'clock at top
5: Turn left 9 to 6 o'clock
6: Forward 6 o'clock  - columnCounter ++
7: Turn right 6 to 9 o'clock
8: Forward 9 o'clock at bottom
9: Turn right 9 to 12 o'clock
10: Return from top - turn left 9 to 3 o'clock
11: Return from top - forward 3 o'clock (until end of roof or columnCounter*nextColEncCount
12: Return from top - turn right 3 to 6 o'clock
13: Return from top - forward 6 o'clock (until end of roof or maxEncCount)
14: Return from bottom - turn left 9 to 3 o'clock
15: Return from bottom - forward 3 o'clock (until end of roof or columnCounter*nextColEncCount

20: Move forward one step
21: Turn left one step
222: Turn right one step

The assumption is that the robot is placed in the lower right
corner of the roof and is pointed up the roof.

Note that state transitions must happen in the body of the state.
If a state transition happens in the header, then the header of
the next state will be skipped.  For example, if we are in the
forward-left state and then call m2.Set(m2s2h, m2s2b), then
the next state will be m2s2b and m2s2h will not be called.

Forward and Reverse states check whether we are done scanning the
roof or whether we returned back home.  The end columns are
detected by satisfying one of the following two conditions:
1. The number of encoder counts (right encoder) for the forward
   or reverse motion is less than 100.
2. Both the front and back IR proximity sensors on one side of the
   robot indicate no roof present.
*/

// Print message in body of state only once
String mot_prev_s="";
void mot_serial_print_once(String s)
{
  if (s!=mot_prev_s) Serial.println(s);
  mot_prev_s = s;
}

// Heading variables
float start_euler_x, stop_euler_x;

float angle_diff(float angle1, float angle2)
{
  // Return the angular difference, angle1 - angle2.
  // Returns the smallest angular distance between the angles
  // and takes the discontinuity at 0,360 into account.
  // The result is signed.
  // The result is the number of degrees to rotate
  // counterclockwise from 1 to 2.
  // If the difference is greater than 180, then return
  // angle1 - angle2 - 360.
  // Example:
  // angle1 = 270
  // angle2 = 45
  // The function returns 270-45-360 = -135
  // So, angle1 must be rotated clockwise by 135 deg
  // to reach angle2.
  float diff = angle1 - angle2;
  if (diff < 180) {
    return diff;
  } else {
    return diff - 360;
  }
}

void straighten_up()
{
  // Straighten up
  float dtheta = angle_diff(euler.x(), start_euler_x);
  // dtheta = euler.x() - start_euler_x; // delta degrees, + is right, - is left
  
  // Don't let the speed get lower than 140
  int left_speed = max(forward_speed + 20 * dtheta, 140);
  int right_speed = max(forward_speed - 20 * dtheta, 140);
  analogWrite(leftMotorForwardPin, left_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, right_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

void fine_tune_angle()
{
  // Fine tune angle
  // dtheta negative: need to move robot counterclockwise
  // dtheta positive: need to move robot clockwise
  float dtheta = angle_diff(stop_euler_x, euler.x());
  if (dtheta > 0) {
    // Turn clockwise
    // slow down as we get close to our target angle, dtheta=0
    // dtheta should nominally go from 90 to 0 degrees
    int spd = turning_speed - (90 - dtheta);
    //int spd = turning_speed;
    analogWrite(leftMotorForwardPin, spd);
    analogWrite(leftMotorBackwardPin, 0);
    analogWrite(rightMotorForwardPin, 0);
    analogWrite(rightMotorBackwardPin, spd);
  } else {
    // Turn counterclockwise
    // dtheta goes from -90 to 0 deg
    int spd = turning_speed - (90 + dtheta);
    //int spd = turning_speed;
    analogWrite(leftMotorForwardPin, 0);
    analogWrite(leftMotorBackwardPin, spd);
    analogWrite(rightMotorForwardPin, spd);
    analogWrite(rightMotorBackwardPin, 0);    
  }
}

// State 1 --------------------------------------------------------
// Stopped

State m2s1h()
{
  Serial.println("m2s1h - stopped");
  m2_state = 1;

  // Stop motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s1b()
{
  mot_serial_print_once("m2s1b - stopped");
}

// State 2 --------------------------------------------------------
// Forward 12 o'clock

State m2s2h()
{
  Serial.println("m2s2h - forward 12 o'clock");
  m2_state = 2;

  // Initial orientation
  start_euler_x = euler.x();
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Increment column counter
  columnCounter++;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s2b()
{
  mot_serial_print_once("m2s2b - forward 12 o'clock");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  
  // Did we reach the maximum encoder count? or
  // Did we reach the end of the roof?
  if ((right_diff > maxEncCount) || frontRoofMissing){
    // Yes, move to next state: turn left 12 to 9 o'clock
    m2.Set(m2s3h, m2s3b);
    return;
  }

  straighten_up();
}

// State 3 --------------------------------------------------------
// Turn left 12 to 9 o'clock

State m2s3h()
{
  Serial.println("m2s3h - turn left 12 to 9 o'clock");
  m2_state = 3;

  // Initial orientation
  // Turning left is a change of -90 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x - 90;
  if (stop_euler_x < 0)
    stop_euler_x += 360;

  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = false;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, turning_speed);
  analogWrite(rightMotorForwardPin, turning_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s3b()
{
  mot_serial_print_once("m2s3b - turn left 12 to 9 o'clock");

  // Are we done turning yet? (within +/-1 degree?)
  if ((stop_euler_x - 1 < euler.x()) && (euler.x() < stop_euler_x + 1) 
                                     && (gyro.x() < 0.5)) {

    // Debug
    analogWrite(leftMotorForwardPin, 0);
    analogWrite(leftMotorBackwardPin, 0);
    analogWrite(rightMotorForwardPin, 0);
    analogWrite(rightMotorBackwardPin, 0);    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String(start_euler_x));
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(String(euler.x()));
    lcd.print("                ");
    while(1) {
      euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); 
      lcd.setCursor(8, 1);
      lcd.print(String(euler.x()));
    }
                                      
    // Yes, move to next state: forward 9 o'clock at top
    m2.Set(m2s4h, m2s4b);
    return;
  }

  fine_tune_angle();
}

// State 4 --------------------------------------------------------
// Forward 9 o'clock at top

State m2s4h()
{
  Serial.println("m2s4h - forward 9 o'clock at top");
  m2_state = 4;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s4b()
{
  mot_serial_print_once("m2s4b - forward 9 o'clock at top");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;

  // Did we reach the maximum encoder count?
  if (right_diff > nextColEncCount) {
    // Yes, move to next state: turn left 9 to 6 o'clock
    m2.Set(m2s5h, m2s5b);
    return;
  }

  // Did we reach the end of the roof? or
  // Did we reach the max column count?
  if (frontRoofMissing || (columnCounter >= maxColumnCount)) {
    // Yes, move to next state: return home from top
    m2.Set(m2s10h, m2s10b);
    return;
  }
  
  straighten_up();
}


// State 5 --------------------------------------------------------
// Turn left 9 to 6 o'clock

State m2s5h()
{
  Serial.println("m2s5h - turn left 9 to 6 o'clock");
  m2_state = 5;

  // Initial orientation
  // Turning left is a change of -90 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x - 90;
  if (stop_euler_x < 0)
    stop_euler_x += 360;  

  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = false;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, turning_speed);
  analogWrite(rightMotorForwardPin, turning_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s5b()
{
  mot_serial_print_once("m2s5b - turn left 9 to 6 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // negative val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // positive val

  // Are we done turning yet?
  //if ((right_diff > turnCounts) || (-left_diff > turnCounts)) {
  if ((stop_euler_x-20 < euler.x()) && (euler.x() < stop_euler_x)) {    
    // Yes, move to next state: forward 6 o'clock
    m2.Set(m2s6h, m2s6b);
    return;
  }
}

// State 6 --------------------------------------------------------
// Forward 6 o'clock

State m2s6h()
{
  Serial.println("m2s6h - forward 6 o'clock");
  m2_state = 6;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Increment column counter
  columnCounter++;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s6b()
{
  mot_serial_print_once("m2s6b - forward 6 o'clock");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  
  // Did we reach the maximum encoder count? or
  // Did we reach the end of the roof?
  if ((right_diff > maxEncCount) || frontRoofMissing){
    // Yes, move to next state: turn right 6 to 9 o'clock
    m2.Set(m2s7h, m2s7b);
    return;
  }
  
  straighten_up();
}

// State 7 --------------------------------------------------------
// Turn right 6 to 9 o'clock

State m2s7h()
{
  Serial.println("m2s7h - turn right 6 to 9 o'clock");
  m2_state = 7;

  // Initial orientation
  // Turning right is a change of +90 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x + 90;
  if (stop_euler_x > 360)
    stop_euler_x -= 360;
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = false;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, turning_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, turning_speed);
}

State m2s7b()
{
  mot_serial_print_once("m2s7b - turn right 6 to 9 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // positive val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // negative val

  // Are we done turning yet?
  //if ((-right_diff > turnCounts) || (left_diff > turnCounts)) {
  if ((stop_euler_x < euler.x()) && (euler.x() < stop_euler_x + 20)) {        
    // Yes, move to next state: forward 9 o'clock at bottom
    m2.Set(m2s8h, m2s8b);
    return;
  }
}

// State 8 --------------------------------------------------------
// Forward 9 o'clock at bottom

State m2s8h()
{
  Serial.println("m2s8h - forward 9 o'clock at bottom");
  m2_state = 8;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s8b()
{
  mot_serial_print_once("m2s8b - forward 9 o'clock at bottom");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // positive val

  // Did we reach the maximum encoder count?
  if (right_diff > nextColEncCount) {
    // Yes, move to next state: turn right 9 to 12 o'clock
    m2.Set(m2s9h, m2s9b);
    return;
  }

  // Did we reach the end of the roof? or
  // Did we reach the max column count?
  if (frontRoofMissing || (columnCounter >= maxColumnCount)) {
    // Yes, move to next state: return home from bottom
    m2.Set(m2s14h, m2s14b);
    return;
  }
  
  straighten_up();
}

// State 9 --------------------------------------------------------
// Turn right 9 to 12 o'clock

State m2s9h()
{
  Serial.println("m2s9h - turn right 6 to 9 o'clock");
  m2_state = 9;

  // Initial orientation
  // Turning right is a change of +90 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x + 90;
  if (stop_euler_x > 360)
    stop_euler_x -= 360;
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = false;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, turning_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, turning_speed);
}

State m2s9b()
{
  mot_serial_print_once("m2s9b - turn right 6 to 9 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // positive val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // negative val

  // Are we done turning yet?
  //if ((-right_diff > turnCounts) || (left_diff > turnCounts)) {
  if ((stop_euler_x < euler.x()) && (euler.x() < stop_euler_x + 20)) {      
    // Yes, move to next state: forward 12 o'clock
    m2.Set(m2s2h, m2s2b);
    return;
  }
}

// State 10 --------------------------------------------------------
// Return from top - turn left 9 to 3 o'clock

State m2s10h()
{
  Serial.println("m2s10h - return from top - turn left 9 to 3 o'clock");
  m2_state = 10;

  // Initial orientation
  // Turning left around is a change of -180 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x - 180;
  if (stop_euler_x < 0)
    stop_euler_x += 360;
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = false;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, turning_speed);
  analogWrite(rightMotorForwardPin, turning_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s10b()
{
  mot_serial_print_once("m2s10b - return from top - turn left 9 to 3 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // negative val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // positive val

  // Are we done turning 180 degrees yet?
  //if ((right_diff > 2*turnCounts) || (-left_diff > 2*turnCounts)) {
  if ((stop_euler_x + 20 < euler.x()) && (euler.x() < stop_euler_x)) {
    // Yes, move to next state: Return from top - forward 3 o'clock 
    m2.Set(m2s11h, m2s11b);
    return;
  }
}

// State 11 --------------------------------------------------------
// Return from top - forward 3 o'clock

State m2s11h()
{
  Serial.println("m2s11h - Return from top - forward 3 o'clock");
  m2_state = 11;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s11b()
{
  mot_serial_print_once("m2s11b - Return from top - forward 3 o'clock");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  
  // Did we reach the maximum encoder count? or
  // Did we reach the end of the roof?
  if ((right_diff > columnCounter*nextColEncCount) || frontRoofMissing){
    // Yes, move to next state: Return from top - turn right 3 to 6 o'clock
    m2.Set(m2s12h, m2s12b);
    return;
  }
  
  straighten_up();
}


// State 12 --------------------------------------------------------
// Return from top - turn right 3 to 6 o'clock

State m2s12h()
{
  Serial.println("m2s12h - Return from top - turn right 3 to 6 o'clock");
  m2_state = 12;

  // Initial orientation
  // Turning right is a change of +90 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x + 90;
  if (stop_euler_x > 360)
    stop_euler_x -= 360;
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = false;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, turning_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, turning_speed);
}

State m2s12b()
{
  mot_serial_print_once("m2s12b - Return from top - turn right 3 to 6 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // positive val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // negative val

  // Are we done turning yet?
  //if ((-right_diff > turnCounts) || (left_diff > turnCounts)) {
  if ((stop_euler_x < euler.x()) && (euler.x() < stop_euler_x + 20)) {
    // Yes, move to next state: Return from top - forward 6 o'clock
    m2.Set(m2s13h, m2s13b);
    return;
  }
}

// State 13 --------------------------------------------------------
// Return from top - forward 3 o'clock

State m2s13h()
{
  Serial.println("m2s13h - Return from top - forward 6 o'clock");
  m2_state = 13;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s13b()
{
  mot_serial_print_once("m2s13b - Return from top - forward 6 o'clock");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  
  // Did we reach the maximum encoder count? or
  // Did we reach the end of the roof?
  if ((right_diff > maxEncCount) || frontRoofMissing){
    // Yes, move to next state: Stopped
    returnedHome = true;
    m2.Set(m2s1h, m2s1b);
    return;
  }
  
  straighten_up();
}

// State 14 --------------------------------------------------------
// Return from bottom - turn left 9 to 3 o'clock

State m2s14h()
{
  Serial.println("m2s14h - Return from bottom - turn left 9 to 3 o'clock");
  m2_state = 14;

  // Initial orientation
  // Turning left around is a change of -180 deg.
  start_euler_x = euler.x();
  stop_euler_x = start_euler_x - 180;
  if (stop_euler_x < 0)
    stop_euler_x += 360;
    
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = false;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, turning_speed);
  analogWrite(rightMotorForwardPin, turning_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s14b()
{
  mot_serial_print_once("m2s14b - Return from bottom - turn left 9 to 3 o'clock");

  // Compute differential encoder count
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;    // negative val
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot; // positive val

  // Are we done turning 180 degrees yet?
  //if ((right_diff > 2*turnCounts) || (-left_diff > 2*turnCounts)) {
  if ((stop_euler_x - 20 < euler.x()) && (euler.x() < stop_euler_x)) {
    // Yes, move to next state: Return from bottom - forward 3 o'clock
    m2.Set(m2s15h, m2s15b);
    return;
  }
}

// State 15 --------------------------------------------------------
// Return from bottom - forward 3 o'clock

State m2s15h()
{
  Serial.println("m2s15h - Return from bottom - forward 3 o'clock");
  m2_state = 15;

  // Initial orientation
  start_euler_x = euler.x();
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;
  
  // Start motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s15b()
{
  mot_serial_print_once("m2s15b - Return from bottom - forward 3 o'clock");

  // Compute differential encoder count
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  
  // Did we reach the maximum encoder count? or
  // Did we reach the end of the roof?
  if ((right_diff > columnCounter*nextColEncCount) || frontRoofMissing){
    // Yes, move to next state: Stopped
    returnedHome = true;
    m2.Set(m2s1h, m2s1b);
    return;
  }
  
  straighten_up();
}


// State 20 --------------------------------------------------------
// Move forward one step

State m2s20h()
{
  Serial.println("m2s20h - move forward one step");
  m2_state = 20;
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = true;
  
  // Stop motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s20b()
{
  mot_serial_print_once("m2s20b - move forward one step");

  if (m2.Timeout(50)) {
    // Go to new state: Stopped
    m2.Set(m2s1h, m2s1b);
    return;
  }
}

// State 21 --------------------------------------------------------
// Turn left one step

State m2s21h()
{
  Serial.println("m2s21h - turn left one step");
  m2_state = 21;
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = false;
  rightMotorForward = true;
  
  // Stop motors
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, forward_speed);
  analogWrite(rightMotorForwardPin, forward_speed);
  analogWrite(rightMotorBackwardPin, 0);
}

State m2s21b()
{
  mot_serial_print_once("m2s21b - turn left one step");

  if (m2.Timeout(50)) {
    // Go to new state: Stopped
    m2.Set(m2s1h, m2s1b);
    return;
  }
}

// State 22 --------------------------------------------------------
// Turn right one step

State m2s22h()
{
  Serial.println("m2s22h - turn right one step");
  m2_state = 22;
  
  // Note motor direction (for proper sign to encoder counts)
  leftMotorForward = true;
  rightMotorForward = false;
  
  // Stop motors
  analogWrite(leftMotorForwardPin, forward_speed);
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, forward_speed);
}

State m2s22b()
{
  mot_serial_print_once("m2s22b - turn right one step");

  if (m2.Timeout(50)) {
    // Go to new state: Stopped
    m2.Set(m2s1h, m2s1b);
    return;
  }
}


