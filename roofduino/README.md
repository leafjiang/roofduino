# Library installation instructions
Copy and extract
```
Adafruit_BNO055-master.zip
Adafruit_Sensor-master.zip
Bounce2-master.zip
SM.zip 
```
into
```
C:\Users\leaf\Documents\Arduino\libraries
```
on Windows or
```
~/Documents/Arduino/libraries
```
on Mac and Linux.  Restart Arduino IDE. On linux, I had to manually add zip libraries by clicking on Sketch -> Include library -> Add .zip library

Select Arduino/Mega 2560 board, otherwise you get the error that A8 was not defined (default is UNO board).  Select board under Tools -> Board.

Serial port
* Confirm that FTDI chip is getting detected correctly with `dmesg`
* Add user to the dialout group to access `/dev/ttyACM0` serial port using the command `sudo adduser $USER dialout`  then logout and then login
* Set baud in serial monitor to 115200 baud

To install latest Arduino on Ubuntu 14.04, follow [these instructions](http://ubuntuhandbook.org/index.php/2015/11/install-arduino-ide-1-6-6-ubuntu/).

Install SRAM library through Sketch -> Include Library -> Mange Libraries
Select SRAM by panStamp v. 1.0.1
Installed into `~/Documents/Arduino/libraries`
