/*
State machine m1 is for LCD control
the states are as follows:
1 = startup screen
2 = select program, 1. roof
3 = select program, 2. square
4 = roof program count down
5 = square program count down
6 = roof program. go!
7 = square program, go!
For all programs, the robot must be placed in the lower right
corner of the area to be scanned.
*/

boolean once1, once2, once3, once4;

String lcd_prev_s="";
void lcd_serial_print_once(String s)
{
  if (s!=lcd_prev_s) Serial.println(s);
  lcd_prev_s = s;
}

void lcdUpdateStatus()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(leftMotorCounter));
  lcd.setCursor(8, 0);
  lcd.print(String(rightMotorCounter));
  lcd.setCursor(14, 0);
  lcd.print(m2_state);
  lcd.setCursor(0, 1);
  lcd.print(String(adcVals[0]));
  lcd.setCursor(9, 1);
  lcd.print(String(adcVals[15]));
  lcd.setCursor(7, 0);
  if (frontObstacle) lcd.print("x");
  else lcd.print(char(165));
  lcd.setCursor(7, 1);
  lcd.print(String(gyro_cal));
  //lcd.setCursor(15,1);
  //lcd.print(String(gyro_cal));
  lcd.setCursor(6, 1);
  if (irFrontLeft) lcd.print(char(165));
  else lcd.print("x");
  lcd.setCursor(8, 1);
  if (irFrontRight) lcd.print(char(165));
  else lcd.print("x");
}

// State 1 --------------------------------------------------------
// Startup screen

State m1s1h()
{
  Serial.println("m1s1h - startup screen");
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roofduino");
  lcd.setCursor(0, 1);
  lcd.print("Version 1.0.0");
}

State m1s1b()
{
  lcd_serial_print_once("m1s1b - startup screen");
  if(m1.Timeout(2000)){ // Maximum 2 seconds idle time
    // Change to state 2
    m1.Set(m1s2h, m1s2b);
  }
}

// State 2 --------------------------------------------------------
// Select program
// 1. Roof

State m1s2h()
{    
  Serial.println("m1s2h - roof");  
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Select program");
  lcd.setCursor(0, 1);  
  lcd.print("1. Roof");
}

State m1s2b()
{
  lcd_serial_print_once("m1s2b - roof"); 
  if (buttonSelect == LOW) {
    // Selected "roof" program
    // Change to state 4
    m1.Set(m1s4h, m1s4b);
  }
  if (buttonDown == LOW || buttonUp == LOW) {
    // Scroll to another program
    // Change to state 3
    m1.Set(m1s3h, m1s3b);
  }
}

// State 3 --------------------------------------------------------
// Select program
// 2. Square

State m1s3h()
{    
  Serial.println("m1s3h - square");
  
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Select program");
  lcd.setCursor(0, 1);
  lcd.print("2. Square");
}

State m1s3b()
{ 
  lcd_serial_print_once("m1s3b - square");
  if (buttonSelect == LOW) {
    // Selected "square" program
    // Change to state 5
    m1.Set(m1s5h, m1s5b);
  }
  if (buttonUp == LOW || buttonDown == LOW) {
    // Scroll back up one level
    // Change to state 2
    m1.Set(m1s2h, m1s2b);
  }
}

// State 4 --------------------------------------------------------
// Roof program
// Count down

State m1s4h()
{    
  Serial.println("m1s4h - roof count down");
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roof program");
  lcd.setCursor(0, 1);
  //lcd.print("3.. ");
  lcd.print("Gyro cal: ");
  lcd.print(gyro_cal);
  once1 = true;
  once2 = true;
  once3 = true;
}

State m1s4b()
{
  lcd_serial_print_once("m1s4b - roof count down");
  if (m1.Timeout(4000)) {
    m1.Set(m1s6h, m1s6b);
    return;
  } else if (m1.Timeout(3000) && once3) {
    //lcd.print("Go!");
    lcd.setCursor(0,1);
    lcd.print("ADC[15]: ");
    lcd.print(adcVals[15]);
    lcd.print("  ");    
    once3 = false;
  } else if (m1.Timeout(2000) && once2) {
    //lcd.print("1.. ");
    lcd.setCursor(0,1);
    lcd.print("ADC[0]: ");
    lcd.print(adcVals[0]);
    lcd.print("  ");
    once2 = false;
  } else if (m1.Timeout(1000) && once1) {  
    lcd.setCursor(0,1);
    lcd.print("Front: ");
    lcd.print(frontCm);
    lcd.print(" cm");
    once1 = false;
  }
}

// State 5 --------------------------------------------------------
// Square program
// Count down

State m1s5h()
{    
  Serial.println("m1s5h - square count down");
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Square program");
  lcd.setCursor(0, 1);
  lcd.print("3.. ");
  once1 = true;
  once2 = true;
  once3 = true;
}

State m1s5b()
{
  lcd_serial_print_once("m1s5b - square count down");
  if (m1.Timeout(2000)) {
    m1.Set(m1s7h, m1s7b);
    return;
  } else if (m1.Timeout(1500) && once3) {
    lcd.print("Go!");
    once3 = false;
  } else if (m1.Timeout(1000) && once2) {
    lcd.print("1.. ");
    once2 = false;
  } else if (m1.Timeout(500) && once1) {
    lcd.print("2.. ");
    once1 = false;
  }
}

// State 6 --------------------------------------------------------
// Roof program... Go!

State m1s6h()
{    
  Serial.println("m1s6h - roof go");
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roof program");
  lcd.setCursor(0, 1);
  lcd.print("Running...");

  // Set global variables
  leftMotorCounter = 0;
  rightMotorCounter = 0;
  returnedHome = false;
  emergencyStop = false;
  
  // Turn on motors (forward)
  m2.Set(m2s2h, m2s2b);  
}

State m1s6b()
{
  lcd_serial_print_once("m1s6b - roof go");
  // Save ADC and encoder data to SD card
  saveData();
  
  // Update LCD display once in a while
  lcdUpdateStatus();

  // If returned to home, done.  goto LCD done state
  if (returnedHome) {
    m1.Set(m1s9h, m1s9b);
    return;
  }
  
  // If we pressed a button, done. Emergency stop.
  if (buttonDown == LOW || buttonUp == LOW || buttonLeft == LOW
       || buttonRight == LOW || buttonSelect == LOW || emergencyStop) {
    m1.Set(m1s8h, m1s8b); // emergency stop
  }
}

// State 7 --------------------------------------------------------
// Square program... Go!

State m1s7h()
{
  Serial.println("m1s7h - square go");

  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roof program");
  lcd.setCursor(0, 1);
  lcd.print("Running...");
  
}

State m1s7b()
{
  lcd_serial_print_once("m1s7b - square go");
  
  // Update LCD display once in a while
  lcdUpdateStatus();

  // If we pressed a button, done
  if (buttonDown == LOW || buttonUp == LOW || buttonLeft == LOW
       || buttonRight == LOW || buttonSelect == LOW) {
    m1.Set(m1s9h, m1s9b); // done
  }
}


// State 8 --------------------------------------------------------
// Error or emergency stop

State m1s8h()
{
  Serial.println("m1s8h - error");

  once1 = true;
  once2 = true;
  once3 = true;
  once4 = true;
}

State m1s8b()
{
  lcd_serial_print_once("m1s8b - error");

  if (m1.Timeout(6000)) {
    m1.Set(m1s9h, m1s9b);
    return;
  } else if (m1.Timeout(5000) && once4) {
    //lcd.print("Go!");
    lcd.setCursor(0,1);
    lcd.print("ADC[15]: ");
    lcd.print(adcVals[15]);
    lcd.print("  ");    
    once4 = false;
  } else if (m1.Timeout(4000) && once3) {
    //lcd.print("1.. ");
    lcd.setCursor(0,1);
    lcd.print("ADC[0]: ");
    lcd.print(adcVals[0]);
    lcd.print("  ");
    once3 = false;
  } else if (m1.Timeout(3000) && once2) {  
    lcd.setCursor(0,1);
    lcd.print("Front: ");
    lcd.print(frontCm);
    lcd.print(" cm");
    once2 = false;
  } else if (m1.Timeout(2000) && once1) {  
    lcd.clear();
    //         0123456789ABCDEF
    lcd.print("Emergency stop");
    lcd.setCursor(0, 1);
    lcd.print("Gyro cal: ");
    lcd.print(gyro_cal);
    once1 = false;
  }
}


// State 9 --------------------------------------------------------
// Done

State m1s9h()
{
  Serial.println("m1s9h - done");

  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Program done");
  lcd.setCursor(0, 1);
  lcd.print("Resetting...");
}

State m1s9b()
{
  lcd_serial_print_once("m1s9b - done");

  if (m1.Timeout(3000)) m1.Set(m1s1h, m1s1b);
}


