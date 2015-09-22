// roofduino.ino
// 2015-09-21 LAJ - Document created

// Button debounce library
#include <Bounce2.h>
const int buttonUpPin = 5;
const int buttonDnPin = 6;
Bounce debouncer1 = Bounce(); 
Bounce debouncer2 = Bounce(); 
int buttonUp = HIGH; // State of buttonUp.  LOW is activated.
int buttonDn = HIGH; // State of buttonDn.  LOW is activated.


// HC-SR04 ultrasonic rangefinder  returns the distance to the
// closest object in range. To do this, it sends a pulse to the 
// sensor to initiate a reading, then listens for a pulse to 
// return.  The length of the returning pulse is proportional to 
// the distance of the object from the sensor.
const int trigPin = 2;
const int echoPin = 4;
long cm = 0;  // Ultrasound rangefinder distance [cm]


// Sharp sensor.  Height measurement
const int adcPins[16] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9,
                         A10, A11, A12, A13, A14, A15};
const float alpha = 0.0952; // 20-period EMA.  http://stockcharts.com/school/doku.php?id=chart_school:technical_indicators:moving_averages
float adcVals[16];  // Smoothed ADC values (10 bit ADC)


// IR proximity sensor pins
const int irFrontRightPin = 10;
const int irFrontLeftPin = 11;
const int irBackRightPin = 12;
const int irBackLeftPin = 13;
// ... and their associated values
boolean irFrontRight = LOW;
boolean irFrontLeft = LOW;
boolean irBackRight = LOW;
boolean irBackLeft = LOW;


// Motor encoders
// Digitial pins usable for interrupts: 2, 3, 18, 19, 20, 21
const int encLeftPin = 20;
const int encRighPin = 21;
boolean leftMotorForward = true;  // true=forward, false=backward
boolean rightMotorForward = true; // true=forward, false=backward
long leftMotorCounter = 0;
long rightMotorCounter = 0;


// The state machine library downloaded from
// http://playground.arduino.cc/Code/SMlib
// Download SM.zip to roofduino/roofduino directory
// Install by selecting from the menu:
// Sketch -> Include Library -> Add .ZIP library
#include <SM.h>
// Define machines and then states for those machines
// Machine 1: LCD display and push buttons.  Master controller
//   State 1: select program
//   State 2: count down, 1..2..3..Go!
//   State 3: display running status.  Also check values to update motion
//   State 4: configuration -- set measurement averaging, speed
// Machine 2: Motion control
//   State 1: Stopped
//   State 1: Forward
//   State 2: Reverse
//   State 3: Turn right
//   State 4: Turn left
//   State 5: Avoid front obstacle (usually detected by ultrasonic ranger)
//   State 6: Avoid front-right obstacle (IR proximity detector)
//   State 7: Avoid front-left obstacle (IR proximity detector)
//   State 8: Avoid back-right obstacle (IR proximity detector)
//   State 9: Avoid back-left obstacle (IR proximity detector)
//   State 10: Return home
//   State 11: Goto next column (e.g., back left, up, down)


// Machine 3: Height measurement, 16 optical range finders
//   State 1: Idle
//   State 2: Grab data
//   State 3: Save to SD card
// Machine 4: Ultrasonic range finder
//   State 1: Idle
//   State 2: Grab data
//   State 3: Close
//   State 4: Far
// Machine 5: Obstacle avoidance sensors
//   State 1: No obstacles
//   State 2: Obstacle in front
//   State 3: No roof, front right
//   State 4: No roof, front left
//   State 5: No roof, back right
//   State 6: No roof, back left
//
// Events:
// Ultrasonic ranger
//   range < 10 cm, range > 10 cm
// IR proximity sensor
//   front-left, front-right, back-right, back-left
// Button press
//   up, dn, left, right, enter 
SM m1(m1s1h, m1s1b);//machine1
SM m2(m2s1h, m2s1b);//machine2
SM m3(m3s1h, m3s1b);//machine3
SM Blink(On);//machine to blink led continously


#define SOME_PIN 15

void setup() {
  // Open serial communications
  Serial.begin(115200);
  Serial.println("Starting roodfduino...")
  
  // Setup the first button with an internal pull-up :
  pinMode(BUTTON_PIN_1,INPUT_PULLUP);
  // After setting up the button, setup the Bounce instance :
  debouncer1.attach(buttonUpPin);
  debouncer1.interval(5); // interval in ms
  
   // Setup the second button with an internal pull-up :
  pinMode(BUTTON_PIN_2,INPUT_PULLUP);
  // After setting up the button, setup the Bounce instance :
  debouncer2.attach(buttonDnPin);
  debouncer2.interval(5); // interval in ms

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
}

void loop() {

  get_sensors();             // Read all sensors, like proximity, etc...
  EXEC(m1);                  // Execute the State Machine
  provide_feedback();        // Send status updates via Serial
  get_serial();              // Grab commands from Serial 

}

// Event variables
// Event variables are read by each state function to determine
// whether to transition to another state.
boolean obstacle_front = false;


// Read all sensor values and update event variables
void get_sensors()
{
  // Get pushbutton values
  // Update the Bounce instances:
  debouncer1.update();
  debouncer2.update();
  // Get the updated value:
  buttonUp = debouncer1.read(); // buttonUp == LOW when button pressed
  buttonDn = debouncer2.read(); // buttonDn == LOW when button pressed
  
  // Get ultrasonic ranger values
  cm = get_ultrasound();
  // debug stuff
  Serial.print(cm);
  Serial.print(" cm");
  Serial.println();
  // todo
  // Interpret ultrasonic ranger results
  if (val < 10 cm)
    obstacle_front = true;
  else
    obstacle_front = false;

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

