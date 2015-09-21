// roofduino.ino
// 2015-09-21 LAJ - Document created

// Button debounce library
#include <Bounce2.h>
Bounce  bouncer  = Bounce(); 


// The state machine library downloaded from
// http://playground.arduino.cc/Code/SMlib
// Download SM.zip to roofduino/roofduino directory
// Install by selecting from the menu:
// Sketch -> Include Library -> Add .ZIP library
#include <SM.h>
// Define machines and then states for those machines
// Machine 1: lcd display
//   State 1: select program
//   State 2: 1..2..3..Go!
//   State 3: running status
//   State 4: configuration
// Machine 2: DC motor drive
//   State 1: Forward
//   State 2: Backward
//   State 3: Turn right
//   State 4: Turn left
//   State 5: Off
// Machine 3: Optical range finders
//   State 1: Grab data
//   State 2: Save to SD card
//   Finished
// Machine 4: 
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

  // Setup button debounce
  pinMode( inputPin ,INPUT);
  // Activate internal pull-up (optional) 
  digitalWrite( inputPin ,HIGH);
  // After setting up the button, setup the object
  bouncer.attach( inputPin );
  bouncer.interval(5);
}

void loop() {
  // put your main code here, to run repeatedly:

  //lcd_select_program();

  //lcd_1_2_3_go();

  //lcd_update_status();

  //stop_pressed();


}
