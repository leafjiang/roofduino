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

// Helper variables so that each print() is executed only once
bool once1 = true;
bool once2 = true;
bool once3 = true;

String lcd_prev_s="";
void lcd_serial_print_once(String s)
{
  if (s!=lcd_prev_s) Serial.println(s);
  lcd_prev_s = s;
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
  homeward_bound = false;
  
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
  lcd.print("3.. ");
  once1 = true;
  once2 = true;
  once3 = true;
}

State m1s4b()
{
  lcd_serial_print_once("m1s4b - roof count down");
  if (m1.Timeout(2000)) m1.Set(m1s6h, m1s6b);
  else if (m1.Timeout(1500) && once3) {
    lcd.print("Go!");
    once3 = false;
  }
  else if (m1.Timeout(1000) && once2) {
    lcd.print("1.. ");
    once2 = false;
  }
  else if (m1.Timeout(500) && once1) {
    lcd.print("2.. ");
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
  if (m1.Timeout(2000)) m1.Set(m1s7h, m1s7b);
  else if (m1.Timeout(1500) && once3) {
    lcd.print("Go!");
    once3 = false;
  }
  else if (m1.Timeout(1000) && once2) {
    lcd.print("1.. ");
    once2 = false;
  }
  else if (m1.Timeout(500) && once1) {
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
  lcd_serial_print_once("m1s6b - roof go");
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
    // TODO: bug. does not rerun header functions when re-entering states
}

void saveData()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read sensor values and append to the string:
  // write adc values
  for (int I; I<16; I++) {
    dataString += String(adcVals[I]);
    dataString += ",";    
  }
  // write encoder values 
  dataString += String(leftMotorCounter);
  dataString += ",";
  dataString += String(rightMotorCounter);
  dataString += ",";
  // write ping values
  dataString += String(frontCm);
  dataString += ",";
  dataString += String(backCm);  
  dataString += ",";
  // write timestamp
  dataString += millis();

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}

void updateLcd()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(String(leftMotorCounter));
  lcd.setCursor(8, 0);
  lcd.print(String(rightMotorCounter));
  lcd.setCursor(15, 1);
  if (homeward_bound) lcd.print("H");  // Going back home
  else lcd.print("S");                 // Scanning
  lcd.setCursor(0, 1);
  lcd.print(String(adcVals[0]));
  lcd.setCursor(9, 1);
  lcd.print(String(adcVals[15]));
  lcd.setCursor(7, 0);
  if (frontObstacle) lcd.print("x");
  else lcd.print(char(165));
  lcd.setCursor(7, 1);
  if (backRoofMissing) lcd.print("x");
  else lcd.print(char(165));
  lcd.setCursor(15,0);
  if (rightMotorForward) lcd.print("F");
  else lcd.print("B");
  lcd.setCursor(6, 1);
  if (irFrontLeft) lcd.print(char(165));
  else lcd.print("x");
  lcd.setCursor(8, 1);
  if (irFrontRight) lcd.print(char(165));
  else lcd.print("x");
}

// State 7 --------------------------------------------------------
// Square program... Go!

State m1s7h()
{
  Serial.println("m1s7h - square go");
  // Not implemented yet.  Just go back to state 1.
  m1.Set(m1s1h, m1s1b);
}

State m1s7b()
{
  lcd_serial_print_once("m1s7b - square go");
}

