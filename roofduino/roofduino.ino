// roofduino.ino
// 2015-09-21 LAJ - Document created

// Button debounce library
#include <Bounce2.h>
const int buttonUpPin = 28;
const int buttonDownPin = 29;   // RIGHT -- WHITE
const int buttonLeftPin = 30;
const int buttonRightPin = 31;
const int buttonSelectPin = 32; // RST -- GRAY
Bounce debouncerUp = Bounce(); 
Bounce debouncerDown = Bounce(); 
Bounce debouncerLeft = Bounce(); 
Bounce debouncerRight = Bounce(); 
Bounce debouncerSelect = Bounce(); 
int buttonUp = HIGH;  // State of buttonUp.  LOW is activated.
int buttonDown = HIGH;
int buttonLeft = HIGH;
int buttonRight = HIGH;
int buttonSelect = HIGH;


// HC-SR04 ultrasonic rangefinder  returns the distance to the
// closest object in range. To do this, it sends a pulse to the 
// sensor to initiate a reading, then listens for a pulse to 
// return.  The length of the returning pulse is proportional to 
// the distance of the object from the sensor.
const int frontTrigPin = 2;
const int frontEchoPin = 4; // needs to be on a pin that supports interrupts?
long frontCm = 0;  // Ultrasound rangefinder distance [cm]
boolean frontObstacle = LOW; // HIGH = obstacle
int frontObstacleCount = 0;  // Must detect obstacle a few times to declare an obstacle
/*
const int backTrigPin = 16;
const int backEchoPin = 17; // needs to be on a pin that supports interrupts? no
long backCm = 0;  // Ultrasound rangefinder distance [cm]
boolean backRoofMissing = false;
int backRoofMissingCount = 0; // Must detect missing roof a few times before delaring a missing roof
*/

// Sharp sensor.  Height measurement
const int adcPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9,
                         A10, A11, A12, A13, A14, A15};
//const float alpha = 0.0952; // 20-period EMA.  http://stockcharts.com/school/doku.php?id=chart_school:technical_indicators:moving_averages
const float alpha = 1; // No averaging
float adcVals[16];  // Smoothed ADC values (10 bit ADC: 0..1023)
boolean frontRoofMissing = false;
int frontRoofMissingCount = 0; // Must detect missing roof a few times before delaring a missing roof


// IR proximity sensor pins
// HIGH = roof detected
// LOW = roof not detected
const int irFrontRightPin = 33;
const int irFrontLeftPin = 34;
const int irBackRightPin = 35;
const int irBackLeftPin = 36;
// ... and their associated values
boolean irFrontRight = HIGH;
boolean irFrontLeft = HIGH;
boolean irBackRight = HIGH;
boolean irBackLeft = HIGH;


// Motor encoders
// Encoder wheels have 20 slots
// Running the motor at 6V (0.13 A) spins it 38 times in 10 seconds
// or 228 RPM. No load.
// The diameter of the wheel is 65 mm.
// Therefore the speed is 3.8 Hz * pi*65 mm = 776 mm/s.
// The encoder count rate is 3.8 Hz * 20 = 76 Hz (13.1 ms)
const int encLeftPin = 18;  // Only some pins can be used for interrupts:
const int encRightPin = 19; //    2, 3, 18, 19, 20, 21 
unsigned long last_isr_enc_left;  // millis() timestamp of isr call for debounce
unsigned long last_isr_enc_right;
boolean leftMotorForward = true;  // true=forward, false=backward
boolean rightMotorForward = true; // true=forward, false=backward
long leftMotorCounter = 0;
long rightMotorCounter = 0;
long leftMotorCounterSnapshot = 0;
long rightMotorCounterSnapshot = 0;
int columnCounter = 0;  // Number of columns scanned so far
int maxColumnCount = 3;
const int maxEncCount = 200;  // Maximum counts allowed in forward or backward direction
const int turnCounts = 22;  // Number of encoder counts for one wheel to turn 90 degrees
const int nextColEncCount = 50; // Number of enocder counts to move forward to next column
boolean returnedHome = false;
boolean emergencyStop = false;
// Forward and turning speeds: 0..255
int forward_speed = 125;
int turning_speed = 200;


// Motor pins
// PWM pins: 2-13, and 44-46
// PWM: 0=off, 255=on (100% duty cycle)
// PWM values for 6V:
// 0   = completely off
// 32  = doesn't move, even with kick start
// 64  = moves after kick start
// 96  = moves after kick start
// 128 = moves, no kick start needed
// 192 = moves
// 255 = moves fast
const int leftMotorBackwardPin = 8;
const int leftMotorForwardPin = 9;
const int rightMotorBackwardPin = 10;
const int rightMotorForwardPin = 11;


// 9 DOF sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55); 
// Possible vector values can be:
// - VECTOR_ACCELEROMETER - m/s^2 = LINEARACCEL + GRAVITY
// - VECTOR_MAGNETOMETER  - uT
// - VECTOR_GYROSCOPE     - rad/s
// - VECTOR_EULER         - degrees
// - VECTOR_LINEARACCEL   - m/s^2
// - VECTOR_GRAVITY       - m/s^2
// The acceleration sensor is both exposed to the gravity force and to
// accelerations applied to the sensor due to movement. 
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> accel;
// Get the four calibration values (0..3)
// Any sensor data reporting 0 should be ignored,
// 3 means 'fully calibrated"
uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
int8_t temp;


// LCD
#include <LiquidCrystal.h>
// LCD RS pin to digital pin 22      purple
// LCD Enable pin to digital pin 23  gray
// LCD D4 pin to digital pin 24      orange
// LCD D5 pin to digital pin 25      yellow
// LCD D6 pin to digital pin 26      green
// LCD D7 pin to digital pin 27      blue
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);


// SD card
// * SD card attached to SPI bus as follows:
// ** MOSI - pin 51
// ** MISO - pin 50
// ** CLK - pin 52
// ** CS - pin 53
#include <SPI.h>
#include <SD.h>
const int chipSelectPin = 53;


// IR remote
// Interrupt on pin 3
const int irRemotePin = 3;
unsigned long last_isr_ir_remote;  // millis() timestamp of isr call
boolean irRemoteUp = false;    // "2" button
boolean irRemoteLeft = false;  // "4" button
boolean irRemoteRight = false; // "6" button
boolean irRemoteStop = false;  // "3" button
int msg_index = 0;
unsigned long time31 = 0;  // falling edge 31
unsigned long time32 = 0;  // falling edge 32


// The state machine library downloaded from
// http://playground.arduino.cc/Code/SMlib
// Download SM.zip to roofduino/roofduino directory
// Install by selecting from the menu:
// Sketch -> Include Library -> Add .ZIP library
#include <SM.h>
// Define machines and then states for those machines
// Machine 1: LCD display and push buttons.
// Machine 2: Motion control

State m1s1h(); State m1s1b();
State m1s2h(); State m1s2b();
State m1s3h(); State m1s3b();
State m1s4h(); State m1s4b();
State m1s5h(); State m1s5b();
State m1s6h(); State m1s6b();
State m1s7h(); State m1s7b();
State m1s8h(); State m1s8b();
State m1s9h(); State m1s9b();

State m2s1h(); State m2s1b();
State m2s2h(); State m2s2b();
State m2s3h(); State m2s3b();
State m2s4h(); State m2s4b();
State m2s5h(); State m2s5b();
State m2s6h(); State m2s6b();
State m2s7h(); State m2s7b();
State m2s8h(); State m2s8b();
State m2s9h(); State m2s9b();
State m2s10h(); State m2s10b();
State m2s11h(); State m2s11b();
State m2s12h(); State m2s12b();
State m2s13h(); State m2s13b();
State m2s14h(); State m2s14b();
State m2s15h(); State m2s15b();
State m2s20h(); State m2s20b();
State m2s21h(); State m2s21b();
State m2s22h(); State m2s22b();

SM m1(m1s1h, m1s1b);//machine1
SM m2(m2s1h, m2s1b);//machine2

int m2_state = 1;  // Keeps track of state of machine 2 (motors) to print on LCD


void setup() {
  // Open serial communications
  Serial.begin(115200);
  Serial.println("Starting roodfduino...");

  // Setup the first button with an internal pull-up
  pinMode(buttonUpPin, INPUT_PULLUP);
  debouncerUp.attach(buttonUpPin);
  debouncerUp.interval(5); // interval in ms
  
  // Setup the second button with an internal pull-up
  pinMode(buttonDownPin, INPUT_PULLUP);
  debouncerDown.attach(buttonDownPin);
  debouncerDown.interval(5); // interval in ms

  // Setup the third button with an internal pull-up
  pinMode(buttonLeftPin, INPUT_PULLUP);
  debouncerLeft.attach(buttonLeftPin);
  debouncerLeft.interval(5); // interval in ms

  // Setup the fourth button with an internal pull-up
  pinMode(buttonRightPin, INPUT_PULLUP);
  debouncerRight.attach(buttonRightPin);
  debouncerRight.interval(5); // interval in ms

  // Setup the fifth button with an internal pull-up
  pinMode(buttonSelectPin, INPUT_PULLUP);
  debouncerSelect.attach(buttonSelectPin);
  debouncerSelect.interval(5); // interval in ms
  
  // Setup IR proximity pins
  pinMode(irFrontRightPin, INPUT);
  pinMode(irFrontLeftPin, INPUT);
  pinMode(irBackRightPin, INPUT);
  pinMode(irBackLeftPin, INPUT);

  // Motor optical encoder pins
  // Puns on ATmega-based arduinos are triggered equally on
  // rising or falling edges, so it is up to the software to
  // determine what happened (rise or fall)
  // For some unknown reason, RISING and FALLING behave like CHANGE
  attachInterrupt(digitalPinToInterrupt(encLeftPin), isr_enc_left, 
                  CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRightPin), isr_enc_right, 
                  CHANGE);               
                  
  // Motors off
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);
  analogWrite(leftMotorForwardPin, 0);  // PWM: 0=off, 255=on (100% duty cycle)
  analogWrite(leftMotorBackwardPin, 0);
  analogWrite(rightMotorForwardPin, 0);
  analogWrite(rightMotorBackwardPin, 0);


  // Setup liquid crystal
  lcd.begin(16,2);
  lcd.print("Wowbot says,");
  lcd.setCursor(0,1);
  lcd.print("Hello world!");   

  // Debug: stop program here
  //Serial.print("stopped");
  //while(1);


  // Initialize 9 DOF sensor
  if(!bno.begin()) {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);    
  bno.setExtCrystalUse(true);
  displaySensorDetails();
  displaySensorStatus();
  /* Display the current temperature */
  temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  
  // Setup SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectPin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");  

  // IR remote
  attachInterrupt(digitalPinToInterrupt(irRemotePin), isr_ir_remote, 
                  CHANGE);    

  /*
  // Debug: motor test
  analogWrite(leftMotorForwardPin, 0);
  analogWrite(leftMotorBackwardPin, 250);
  analogWrite(rightMotorForwardPin, 250);
  analogWrite(rightMotorBackwardPin, 0);  
  */
  
}

unsigned long loop_start;

void loop() {

  loop_start = millis();

  get_sensors();             // Read all sensors, like proximity, etc...
  EXEC(m1);                  // Execute the State Machine: LCD
  EXEC(m2);                  // Motion  

  //provide_feedback();        // Send status updates via Serial
  //get_serial();              // Grab commands from Serial 

  // Benchmark loop duration:
  Serial.print("loop duration (ms): ");
  Serial.println(millis()-loop_start);
  // 10 ms: before roof program
  // 83 ms: while running roof program
  // 77 ms:   minus 9 DOF sensor reading
  // 80 ms:   minus PING sensor reading
  // 20 ms:   minus saveData to SD card
  // 700 ms: Happens when the ping sensor is unplugged (times out waiting for echo)
}


// Read all sensor values and update event variables
void get_sensors()
{
  // Get pushbutton values
  // Update the Bounce instances:
  debouncerUp.update();
  debouncerDown.update();
  debouncerLeft.update();
  debouncerRight.update();
  debouncerSelect.update();
  // Get the updated value:
  buttonUp = debouncerUp.read(); // buttonUp == LOW when button pressed
  buttonDown = debouncerDown.read();
  buttonLeft = debouncerLeft.read();
  buttonRight = debouncerRight.read();
  buttonSelect = debouncerSelect.read();

  // debug stuff
  /*
  Serial.print("U:");
  Serial.print(buttonUp);
  Serial.print(" D:");
  Serial.print(buttonDown);
  Serial.print(" L:");
  Serial.print(buttonLeft);
  Serial.print(" R:");
  Serial.print(buttonRight);
  Serial.print(" S:");
  Serial.print(buttonSelect);
  Serial.println();
  */

  // Get ultrasonic ranger values
  // If unplugged, comment out, otherwise will wait 700 ms for return!
  //frontCm = get_ultrasound(frontTrigPin, frontEchoPin);
  //backCm = get_ultrasound(backTrigPin, backEchoPin);

  // Front obstacle detection
  if (frontCm < 15)
    frontObstacleCount++;
  else
    frontObstacleCount = 0;
  if (frontObstacleCount > 1)
    frontObstacle = HIGH;
  else
    frontObstacle = LOW;

  /*
  // Any measurement above 3400 cm is spurious
  if (backCm > 3400) {
    backCm = 0;
  }

  // Missing roof detection
  if (backCm > 6)
    backRoofMissingCount++;
  else
    backRoofMissingCount = 0;
  if (backRoofMissingCount > 1)
    backRoofMissing = HIGH;
  else
    backRoofMissing = LOW;
  */ 

  // debug stuff
  /*
  Serial.print("front ");
  Serial.print(frontCm);
  Serial.print(" cm, back ");
  Serial.print(backCm);
  Serial.print(" cm");
  Serial.println();
  lcd.setCursor(0, 0);
  lcd.print("front ");
  lcd.print(frontCm);
  lcd.print(" cm          ");
  lcd.setCursor(0, 1);
  lcd.print("back ");
  lcd.print(backCm);
  lcd.print(" cm          ");
  */
  // Get IR proximity sensor values
  /*  These sensors don't work well.  Unreliable.
  irFrontRight = digitalRead(irFrontRightPin);
  irFrontLeft = digitalRead(irFrontLeftPin);
  irBackRight = digitalRead(irBackRightPin);
  irBackLeft = digitalRead(irBackLeftPin);
  */

  // debug stuff
  /*
  Serial.print("FR:");
  Serial.print(irFrontRight);
  Serial.print(" FL:");
  Serial.print(irFrontLeft);
  Serial.print(" BR:");
  Serial.print(irBackRight);
  Serial.print(" BL:");
  Serial.print(irBackLeft);
  Serial.println();
  */

  // Analog-to-digital converters
  // Get height sensor values
  for (int I=0; I<16; I++) {  // For each pin
    // Exponential moving average
    adcVals[I] = alpha * analogRead(adcPins[I]) 
                 + (1-alpha) * adcVals[I];
  }

  // debug stuff
  // ADCs measure 350-ish.  Lower values = farther away
  // Max value seems to be around 500-ish (paper close to sensor)
  /*
  for (int I=0; I<16; I++) {
    Serial.print(adcVals[I]);
    Serial.print(" ");
  }
  Serial.println();
  */

  // Simulate front IR proximity sensors with analog inputs
  //   Left side
  if (adcVals[0] < 230)
    irFrontLeft = LOW; // no roof detected
  else
    irFrontLeft = HIGH;
  //   Right side
  if (adcVals[15] < 230)
    irFrontRight = LOW; // no roof detected
  else
    irFrontRight = HIGH;

  // Front roof missing detector
  if (!irFrontLeft || !irFrontRight)
    frontRoofMissingCount++;
  else
    frontRoofMissingCount = 0;
  if (frontRoofMissingCount > 0)
    frontRoofMissing = HIGH;
  else
    frontRoofMissing = LOW;

  /*
  // Simulate back IR proximity sensors with back ping data
  if (backRoofMissing) {
    irBackLeft = LOW;
    irBackRight = LOW;    
  } else {
    irBackLeft = HIGH;
    irBackRight = HIGH;    
  }
  */

  // Get motor encoder values
  // Don't need to do anything here since these encoders are
  // attached to pins that will generate an interrupt and then
  // the interrupt service routines will update the counters.

  // debug stuff
  /*
  Serial.print("Left motor enc:");
  Serial.print(leftMotorCounter);
  Serial.print(" Right motor enc:");
  Serial.print(rightMotorCounter);
  Serial.println();
  */

  
  // Get 9 DOF sensor event
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // 0..359.999 deg
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); // rad/s
  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getCalibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal);
  /*
  // debug
  imu::Quaternion euler_quat = bno.getQuat();
  Serial.print("X: ");
  Serial.print(euler.x(), 4); // orientation of robot. Clockwise is positive direction.
  Serial.print(" Y: ");
  Serial.print(euler.y(), 4);
  Serial.print(" Z: ");
  Serial.print(euler.z(), 4); 
//  Serial.print(" w: ");
//  Serial.print(euler_quat.w(), 4);
//  Serial.print(" x: ");
//  Serial.print(euler_quat.x(), 4);
//  Serial.print(" y: ");
//  Serial.print(euler_quat.y(), 4);
//  Serial.print(" z: ");
//  Serial.print(euler_quat.z(), 4); 
  Serial.print(" aX: ");
  Serial.print(accel.x(), 4);
  Serial.print(" aY: ");
  Serial.print(accel.y(), 4);
  Serial.print(" aZ: ");
  Serial.print(accel.z(), 4);
  displayCalStatus();
  */
  // Wait the specified delay before requesting nex data
  //delay(BNO055_SAMPLERATE_DELAY_MS);

  // Disable obstacle sensors
  //backRoofMissing = false;
  //frontRoofMissing = false;
  //frontObstacle = false;

  /*
  // IR Remote debug 
  if (irRemoteUp) {
    Serial.println("IR remote: UP");
    irRemoteUp = false;
  }
  if (irRemoteStop) {
    Serial.println("IR remote: STOP");
    irRemoteStop = false;
  }
  if (irRemoteLeft) {
    Serial.println("IR remote: LEFT");
    irRemoteLeft = false;
  }
  if (irRemoteRight) {
    Serial.println("IR remote: RIGHT");
    irRemoteRight = false;
  }
  */
}

void saveData()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // write timestamp
  dataString += millis();
  dataString += ",";

  // write encoder values 
  dataString += String(leftMotorCounter);
  dataString += ",";
  dataString += String(rightMotorCounter);
  dataString += ",";

  // write ping values
  dataString += String(frontCm);
  dataString += ",";

  // write 9 DOF imu values
  dataString += String(temp);
  dataString += ",";
  dataString += String(euler.x()); // degrees
  dataString += ",";
  dataString += String(euler.y());  
  dataString += ",";
  dataString += String(euler.z());  
  dataString += ",";  
  dataString += String(accel.x()); // m/s/s
  dataString += ",";
  dataString += String(accel.y());  
  dataString += ",";
  dataString += String(accel.z());  
  dataString += ",";    
  dataString += String(sys_cal);  
  dataString += ",";  
  dataString += String(gyro_cal);
  dataString += ",";
  dataString += String(accel_cal);  
  dataString += ",";
  dataString += String(mag_cal);      


  // read sensor values and append to the string:
  // write adc values
  for (int I; I<16; I++) {
    dataString += ",";    
    dataString += String(adcVals[I]);
  }

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

