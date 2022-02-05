
In order to use this folder, you must do the followings.

1.) Install Arduino (for Ubuntu 20.04)
```
sudo apt-get update
sudo apt-get upgrade
cd
mkdir arduino
cd arduino/
wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz
cd arduino-1.8.15/
sudo ./install.sh
```

2.) Find your serial port 
```
ls -l /dev/ttyACM*
```
Remember this port. You will need when you run serial node.

3.) Installing rosserial
```
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino
```
4.) Setup directory for your sketchbook 
```
cd <This is where your sketchbook will be>
mkdir -p <sketchbook>/libraries
cd <sketchbook>/libraries
rosrun rosserial_arduino make_libraries.py .
```
5.) Set your sketchbook in Arduino IDE

  File>Preferences> choose <sketchbook>

6.) Installing Arduino megaAVR Package via Arduino IDE

  >This step is Optional. Only neccessary if you use Arduino megaAVR boards such as Uno WiFi Rev2.

  >>Tools > Board > Boards Manager 
  
  >Scroll down until you see "Arduino megaAVR Boards" and click the install button.

7.) Choose your serial port and your board

In Arduino IDE, 
  
  >select your serial port
  
  >>Tools > Port > /dev/ttyACM* (the same one from step 2)
  
  >(if your board is Arduino Mega 2560)
  >>Tools > Board > Arduino Mega or Mega 2560
  
  >(if your board is Arduino Uno WiFi Rev2)
  >>Tools > Board > Arduino megaAVR Boards > Arduino Uno WiFi Rev2

8.) Load the folders to your sketchbook 
  

  
