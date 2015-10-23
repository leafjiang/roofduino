/*
State machine m2 is for motion control
the states are as follows:
1: Stopped
2: Forward
3: Reverse
4: Move back right
5: Move back left
6: Move forward right
7: Move forward left

The assumption is that the robot is placed in the lower right
corner of the roof and is pointed up the roof.

Forward and Reverse states check whether we are done scanning the
roof or whether we returned back home.  The end columns are
detected by satisfying one of the following two conditions:
1. The number of encoder counts (right encoder) for the forward
   or reverse motion is less than 100.
2. Both the front and back IR proximity sensors on one side of the
   robot indicate no roof present.
*/

void stop_motors()
{
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void move_forward()
{
  // Keep track of motor direction
  leftMotorForward = true;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void move_backward()
{
  // Keep track of motor direction
  leftMotorForward = false;
  rightMotorForward = false;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;

  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);  
}

void turn_left()
{
  // Keep track of motor direction
  leftMotorForward = false;
  rightMotorForward = true;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;

  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void turn_right()
{
  // Keep track of motor direction
  leftMotorForward = true;
  rightMotorForward = false;

  // Keep track of encoder counters
  leftMotorCounterSnapshot = leftMotorCounter;
  rightMotorCounterSnapshot = rightMotorCounter;

  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);  
}

void are_we_home_yet()
{
  // Are we home yet?
  // We are home if either the length of this column is less than
  // 100 encoder counts, or if the IR proximity sensors on the
  // right side detect no roof.
  if ( (L < 100 && (!irFrontRight || !irBackRight)) ||
       (!irFrontRight && !irBackRight) ) {
    // Yes, we are home.
    homeward_bound = false;
    at_home = true;
    // Stop motors
    m2.Set(m2s1h, m2s1b);
    // Update LCD display
    m1.Set(m1s1h, m1s1b);
  }
}

void are_we_done_yet()
{
  // Are we done scanning the roof?
  // We are done if either the length of the column is less than
  // 100 encoder counts, or if the IR proximity sensors on the
  // left side detect no roof.
  if ( (L < 100 && (!irFrontLeft || !irBackLeft)) ||
       (!irFrontLeft && !irBackLeft) ) {
    // Yes, we are done.
    homeward_bound = true;
    // Stop motors
    // m2.Set(m2s1h, m2s1b);
    // Update LCD display
    // m1.Set(m1s1h, m1s1b);
  }
}  

// State 1 --------------------------------------------------------
// Stopped

State m2s1h()
{
  stop_motors();
}

State m2s1b()
{
}

// State 2 --------------------------------------------------------
// Forward

State m2s2h()
{
  move_forward();
}

State m2s2b()
{
  // Balance encoder counts.
  // Check encoder counts.  If not even, change power to
  // left and right motors
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  L = abs(right_diff);
  long d = right_diff - left_diff;
  if (d > 1) {
    // Left motor needs to catch up
    // Left on, right off
    digitalWrite(leftMotorForwardPin, HIGH);
    digitalWrite(leftMotorBackwardPin, LOW);
    digitalWrite(rightMotorForwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, LOW);      
  } else if (d < -1) {
    // Right motor needs to catch up
    // Left off, right on
    digitalWrite(leftMotorForwardPin, LOW);
    digitalWrite(leftMotorBackwardPin, LOW);
    digitalWrite(rightMotorForwardPin, HIGH);
    digitalWrite(rightMotorBackwardPin, LOW);  
  } else {
    // Normal operation: both motors forward
    digitalWrite(leftMotorForwardPin, HIGH);
    digitalWrite(leftMotorBackwardPin, LOW);
    digitalWrite(rightMotorForwardPin, HIGH);
    digitalWrite(rightMotorBackwardPin, LOW);
  }

  if (homeward_bound) {
    // Are we home yet?
    are_we_home_yet();
  } else {
    // Are we done yet?
    are_we_done_yet();
  }
  
  // Move to next column?
  // We move to the next column, if ultrasound range finder 
  // finds an obstruction or if the front IR proximity sensors
  // don't detect a roof.
  if (cm < 10 || !irFrontRight || !irFrontLeft) {
    // Yes, move to next column
    // Are we going home?
    if (homeward_bound) {
      // Yes, we are homeward bound.
      // Move back-right
      m2.Set(m2s4h, m2s4b);
    } else {
      // No, continue scanning the roof.
      // Move back-left
      m2.Set(m2s5h, m2s5b);
    }
  }
}

// State 3 --------------------------------------------------------
// Backward

State m2s3h()
{
  move_backward();
}

State m2s3b()
{
  // Balance encoder counts.
  // Check encoder counts.  If not even, change power to
  // left and right motors
  long left_diff = leftMotorCounter - leftMotorCounterSnapshot;
  long right_diff = rightMotorCounter - rightMotorCounterSnapshot;
  L = abs(right_diff);
  long d = right_diff - left_diff;
  if (d > 1) {
    // Left motor needs to catch up
    // Left on, right off
    digitalWrite(leftMotorForwardPin, LOW);
    digitalWrite(leftMotorBackwardPin, HIGH);
    digitalWrite(rightMotorForwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, LOW);      
  } else if (d < -1) {
    // Right motor needs to catch up
    // Left off, right on
    digitalWrite(leftMotorForwardPin, LOW);
    digitalWrite(leftMotorBackwardPin, LOW);
    digitalWrite(rightMotorForwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, HIGH);  
  } else {
    // Normal operation: both motors backward
    digitalWrite(leftMotorForwardPin, LOW);
    digitalWrite(leftMotorBackwardPin, HIGH);
    digitalWrite(rightMotorForwardPin, LOW);
    digitalWrite(rightMotorBackwardPin, HIGH);
  }

  if (homeward_bound) {
    // Are we home yet?
    are_we_home_yet();
  } else {
    // Are we done yet?
    are_we_done_yet();
  }

  // Move to next column?
  // We move to the next column, if the front IR proximity sensors
  // don't detect a roof.
  if (!irBackRight || !irFrontLeft) {
    // Yes, move to next column
    // Are we going home?
    if (homeward_bound) {
      // Yes, we are homeward bound.
      // Move forward-right
      m2.Set(m2s6h, m2s6b);
    } else {
      // No, continue scanning the roof.
      // Move forward-left
      m2.Set(m2s7h, m2s7b);
    }
  }

}


// State 4 --------------------------------------------------------
// Go back right (S-maneuver)

State m2s4h()
{
  turn_left();                                // 1. rotate
}

State m2s4b()
{
  if (m1.Timeout(3000)) m2.Set(m2s3h, m2s3b);  // 4. done, go back
  else if (m1.Timeout(2000)) turn_right();     // 3. straighten up
  else if (m1.Timeout(1000)) move_backward();  // 2. go backward
}


// State 5 --------------------------------------------------------
// Go back left (S-maneuver)

State m2s5h()
{
  turn_right();                                // 1. rotate
}

State m2s5b()
{
  if (m1.Timeout(3000)) m2.Set(m2s3h, m2s3b);  // 4. done, go back
  else if (m1.Timeout(2000)) turn_left();      // 3. straighten up
  else if (m1.Timeout(1000)) move_backward();  // 2. go backward
}


// State 6 --------------------------------------------------------
// Go forward right (S-maneuver)

State m2s6h()
{
  turn_right();                                // 1. rotate
}

State m2s6b()
{
  if (m1.Timeout(3000)) m2.Set(m2s2h, m2s2b); // 4. done, go forw
  else if (m1.Timeout(2000)) turn_left();     // 3. straighten up
  else if (m1.Timeout(1000)) move_forward();  // 2. go forward
}


// State 7 --------------------------------------------------------
// Go forward left (S-maneuver)

State m2s7h()
{
  turn_left();                                // 1. rotate
}

State m2s7b()
{
  if (m1.Timeout(3000)) m2.Set(m2s2h, m2s2b); // 4. done, go forw
  else if (m1.Timeout(2000)) turn_right();    // 3. straighten up
  else if (m1.Timeout(1000)) move_forward();  // 2. go forward
}

