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
int buttonUp = HIGH; // State of buttonUp.  LOW is activated.
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
//
const int backTrigPin = 18;
const int backEchoPin = 19; // needs to be on a pin that supports interrupts?
long backCm = 0;  // Ultrasound rangefinder distance [cm]
boolean backRoofMissing = LOW;  // HIGH = roof missing
int backRoofMissingCount = 0; // Must detect missing roof a few times before delaring a missing roof


// Sharp sensor.  Height measurement
const int adcPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9,
                         A10, A11, A12, A13, A14, A15};
const float alpha = 0.0952; // 20-period EMA.  http://stockcharts.com/school/doku.php?id=chart_school:technical_indicators:moving_averages
float adcVals[16];  // Smoothed ADC values (10 bit ADC: 0..1023)


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
const int encLeftPin = 20;  // Only some pins can be used for interrupts:
const int encRightPin = 21; //    2, 3, 18, 19, 20, 21 
boolean leftMotorForward = true;  // true=forward, false=backward
boolean rightMotorForward = true; // true=forward, false=backward
long leftMotorCounter = 0;
long rightMotorCounter = 0;
long leftMotorCounterSnapshot = 0;
long rightMotorCounterSnapshot = 0;
long L = 0;  // = |rightMotorCounter - rightMotorCounterSnapshot|
boolean homeward_bound = false;
boolean at_home = true;
unsigned long last_isr_enc_left;  // millis() timestamp of isr call for debounce
unsigned long last_isr_enc_right;


// Motor pins
const int leftMotorForwardPin = 37;
const int leftMotorBackwardPin = 38;
const int rightMotorForwardPin = 39;
const int rightMotorBackwardPin = 40;


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


// The state machine library downloaded from
// http://playground.arduino.cc/Code/SMlib
// Download SM.zip to roofduino/roofduino directory
// Install by selecting from the menu:
// Sketch -> Include Library -> Add .ZIP library
#include <SM.h>
// Define machines and then states for those machines
// Machine 1: LCD display and push buttons.
// Machine 2: Motion control

State m1s1h();
State m1s1b();
State m1s2h();
State m1s2b();
State m1s3h();
State m1s3b();
State m1s4h();
State m1s4b();
State m1s5h();
State m1s5b();
State m1s6h();
State m1s6b();
State m1s7h();
State m1s7b();

State m2s1h();
State m2s1b();
State m2s2h();
State m2s2b();
State m2s3h();
State m2s3b();
State m2s4h();
State m2s4b();
State m2s5h();
State m2s5b();
State m2s6h();
State m2s6b();
State m2s7h();
State m2s7b();

SM m1(m1s1h, m1s1b);//machine1
SM m2(m2s1h, m2s1b);//machine2


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
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);

  // Setup liquid crystal
  lcd.begin(16,2);
  lcd.print("Wowbot says,");
  lcd.setCursor(0,1);
  lcd.print("Hello world!");   

  // Debug: stop program here
  //Serial.print("stopped");
  //while(1);

  // Setup SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectPin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");  
}

void loop() {

  get_sensors();             // Read all sensors, like proximity, etc...
  EXEC(m1);                  // Execute the State Machine: LCD
  EXEC(m2);                  // Motion  

  // debug
  //move_forward();
  
  //provide_feedback();        // Send status updates via Serial
  //get_serial();              // Grab commands from Serial 
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
  frontCm = get_ultrasound(frontTrigPin, frontEchoPin);
  backCm = get_ultrasound(backTrigPin, backEchoPin);

  // Front obstacle detection
  if (frontCm < 10)
    frontObstacleCount++;
  else
    frontObstacleCount = 0;
  if (frontObstacleCount > 1)
    frontObstacle = HIGH;
  else
    frontObstacle = LOW;

  // Missing roof detection
  if (backCm > 6)
    backRoofMissingCount++;
  else
    backRoofMissingCount = 0;
  if (backRoofMissingCount > 1)
    backRoofMissing = HIGH;
  else
    backRoofMissing = LOW;

  // debug stuff
  /*
  Serial.print("front ");
  Serial.print(frontCm);
  Serial.print(" cm, back ");
  Serial.print(backCm);
  Serial.print(" cm");
  Serial.println();
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
  if (adcVals[0] < 250)
    irFrontLeft = LOW; // no roof detected
  else
    irFrontLeft = HIGH;
  //   Right side
  if (adcVals[15] < 250)
    irFrontRight = LOW; // no roof detected
  else
    irFrontRight = HIGH;
  
  // Simulate back IR proximity sensors with back ping data
  if (backRoofMissing) {
    irBackLeft = LOW;
    irBackRight = LOW;    
  } else {
    irBackLeft = HIGH;
    irBackRight = HIGH;    
  }

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
}

// Interrupt service routines

void isr_enc_left()
{
  // Debounce encoder
  // Call a change a bounce if it is close in time to the last change
  unsigned long time_diff = millis() - last_isr_enc_left;
  
  if (time_diff > 5) {
    // Valid interrupt.  Increment counter.
    // todo: check for overflow/underflow of counter?
    // seems unlikely to overflow, because long type stores
    // 32 bits (4 bytes), from -2,147,483,648 to 2,147,483,647.
    if (leftMotorForward)
      leftMotorCounter++;
    else
      leftMotorCounter--;
  }

  // Timestamp this interrupt
  last_isr_enc_left = millis();
}

void isr_enc_right()
{
  // Debounce encoder
  // Call a change a bounce if it is close in time to the last change
  unsigned long time_diff = millis() - last_isr_enc_right;

  if (time_diff > 5) {
    // todo: check for overflow/underflow of counter?
    if (rightMotorForward)
      rightMotorCounter++;
    else
      rightMotorCounter--;
  }

  // Timestamp this interrupt
  last_isr_enc_right = millis();
}



