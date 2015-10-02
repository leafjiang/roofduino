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

// State 1 --------------------------------------------------------
// Startup screen

State m1s1h()
{
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roofduino");
  lcd.setCursor(0, 1);
  lcd.print("Version 1.0.0");
}

State m1s1b()
{
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
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Select program");
  lcd.setCursor(0, 1);  
  lcd.print("1. Roof");
}

State m1s2b()
{ 
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
  homeward_bound = false;
  
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Select program");
  lcd.setCursor(0, 1);
  lcd.print("2. Square");
}

State m1s3b()
{ 
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
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roof program");
  lcd.setCursor(0, 1);
  lcd.print("3.. ");
}

State m1s4b()
{
  if (m1.Timeout(2000)) m1.Set(m1s6h, m1s6bd);
  else if (m1.Timeout(1500)) lcd.print("Go!");
  else if (m1.Timeout(1000)) lcd.print("1.. ");
  else if (m1.Timeout(500)) lcd.print("2.. ");
}

// State 5 --------------------------------------------------------
// Square program
// Count down

State m1s5h()
{    
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Square program");
  lcd.setCursor(0, 1);
  lcd.print("3.. ");
}

State m1s5b()
{
  if (m1.Timeout(2000)) m1.Set(m1s7h, m1s7b);
  else if (m1.Timeout(1500)) lcd.print("Go!");
  else if (m1.Timeout(1000)) lcd.print("1.. ");
  else if (m1.Timeout(500)) lcd.print("2.. ");
}

// State 6 --------------------------------------------------------
// Roof program... Go!

State m1s6h()
{    
  lcd.clear();
  //         0123456789ABCDEF
  lcd.print("Roof program");
  lcd.setCursor(0, 1);
  lcd.print("Running...");

  // Set global variables
  leftMotorForward = true;
  rightMotorForward = true;
  leftMotorCounter = 0;
  rightMotorCounter = 0;
  at_home = false;
  // Bounding box: Only for square mode
  //posCounterLim = 1234567;
  //numColumns = 123;
  
  // Turn on motors (forward)
  m2.Set(m2s2h, m2s2b);  
}

State m1s6b()
{
  // Save ADC and encoder data to SD card
  saveData();
  
  // Update LCD display once in a while
  updateLcd();

  // If returned to home, done.  goto state 1
  if (at_home) m1.Set(m1s1h, m1s1b);
  
  // If we pressed a button, done. goto state 1
  if (buttonDown == LOW || buttonUp == LOW || buttonLeft == LOW
       || buttonRight == LOW || buttonSelect == LOW)
    m1.Set(m1s1h, m1s1b);
}

void saveData()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read sensor values and append to the string:
  // first, write adc values
  for (int I; I<16; I++) {
    dataString += String(adcVals[I]);
    dataString += ",";    
  }
  // second, write encoder values 
  dataString += String(leftMotorCounter);
  dataString += ","
  dataString += String(rightMotorCounter);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void updataLcd()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(leftMotorCounter));
  lcd.setCursor(0, 15);
  if (homeward_bound) lcd.print("H");  // Going back home
  else lcd.print("S");                 // Scanning
  lcd.setCursor(1, 0);
  lcd.print(String(adcVals[2]));
  lcd.setCursor(1, 8);
  lcd.print(String(adcVals[13]));
}

// State 7 --------------------------------------------------------
// Square program... Go!

State m1s7h()
{
  // Not implemented yet.  Just go back to state 1.
  m1.Set(m1s1h, m1s1b);
}

State m1s7b()
{
}

