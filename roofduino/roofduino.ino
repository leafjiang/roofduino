// roofduino.ino
// 2015-09-21 LAJ - Document created

// Button debounce library
#include <Bounce2.h>
const int buttonUpPin = 28;
const int buttonDownPin = 29;
const int buttonLeftPin = 30;
const int buttonRightPin = 31;
const int buttonSelectPin = 32;
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
const int trigPin = 2;
const int echoPin = 4; // needs to be on a pin that supports interrupts?
long cm = 0;  // Ultrasound rangefinder distance [cm]


// Sharp sensor.  Height measurement
const int adcPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9,
                         A10, A11, A12, A13, A14, A15};
const float alpha = 0.0952; // 20-period EMA.  http://stockcharts.com/school/doku.php?id=chart_school:technical_indicators:moving_averages
float adcVals[16];  // Smoothed ADC values (10 bit ADC)


// IR proximity sensor pins
const int irFrontRightPin = 33;
const int irFrontLeftPin = 34;
const int irBackRightPin = 35;
const int irBackLeftPin = 36;
// ... and their associated values
boolean irFrontRight = LOW;
boolean irFrontLeft = LOW;
boolean irBackRight = LOW;
boolean irBackLeft = LOW;


// Motor encoders
const int encLeftPin = 20;  // Only some pins can be used for interrupts
const int encRighPin = 21;  // Another interrupt pin
boolean leftMotorForward = true;  // true=forward, false=backward
boolean rightMotorForward = true; // true=forward, false=backward
long leftMotorCounter = 0;
long rightMotorCounter = 0;
long leftMotorCounterSnapshot = 0;
long rightMotorCounterSnapshot = 0;
long L = 0;  // = |rightMotorCounter - rightMotorCounterSnapshot|
boolean homeward_bound = false;
boolean at_home = true;

// Motor pins
const int leftMotorForwardPin = 37;
const int leftMotorBackwardPin = 38;
const int rightMotorForwardPin = 39;
const int rightMotorBackwardPin = 40;


// LCD
#include <LiquidCrystal.h>
// LCD RS pin to digital pin 22
// LCD Enable pin to digital pin 23
// LCD D4 pin to digital pin 24
// LCD D5 pin to digital pin 25
// LCD D6 pin to digital pin 26
// LCD D7 pin to digital pin 27
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);


// SD card
// * SD card attached to SPI bus as follows:
// ** MOSI - pin 51
// ** MISO - pin 50
// ** CLK - pin 52
// ** CS - pin 53
#include <SPI.h>
#include <SD.h>
const int chipSelesctPin = 53;


// The state machine library downloaded from
// http://playground.arduino.cc/Code/SMlib
// Download SM.zip to roofduino/roofduino directory
// Install by selecting from the menu:
// Sketch -> Include Library -> Add .ZIP library
#include <SM.h>
// Define machines and then states for those machines
// Machine 1: LCD display and push buttons.
// Machine 2: Motion control
SM m1(m1s1h, m1s1b);//machine1
SM m2(m2s1h, m2s1b);//machine2


void setup() {
  // Open serial communications
  Serial.begin(115200);
  Serial.println("Starting roodfduino...")
  
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
  attachInterrupt(digitalPinToInterrupt(encLeftPin), isr_enc_left, 
                  RISING);
  attachInterrupt(digitalPinToInterrupt(encRightPin), isr_enc_right, 
                  RISING);

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
  lcd.print("hello, world!");   

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
  
  // Get ultrasonic ranger values
  cm = get_ultrasound();
  // debug stuff
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();
  // todo
  // Interpret ultrasonic ranger results
  // if (val < 10 cm)
  //   obstacle_front = true;
  // else
  //   obstacle_front = false;

  // Get IR proximity sensor values
  irFrontRight = digitalRead(irFrontRightPin);
  irFrontLeft = digitalRead(irFrontLeftPin);
  irBackRight = digitalRead(irBackRightPin);
  irBackLeft = digitalRead(irBackLeftPin);

  // Get height sensor values
  for (int I=0; I<16; I++) {  // For each pin
    // Exponential moving average
    adcVals[I] = alpha * analogRead(adcPins[I]) 
                 + (1-alpha) * adcVals[I];
  }

  // Get motor encoder values
  // Don't need to do anything here since these encoders are
  // attached to pins that will generate an interrupt and then
  // the interrupt service routines will update the counters.
}

// Interrupt service routines

void isr_enc_left()
{
  // todo: check for overflow/underflow of counter?
  // seems unlikely to overflow, because long type stores
  // 32 bits (4 bytes), from -2,147,483,648 to 2,147,483,647.
  if (leftMotorForward)
    leftMotorCounter++;
  else
    leftMotorCounter--;
}

void isr_enc_right()
{
  // todo: check for overflow/underflow of counter?
  if (rightMotorForward)
    rightMotorCounter++;
  else
    rightMotorCounter--;
}

