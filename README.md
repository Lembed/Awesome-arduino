# awesome-arduino
A curated list of awesome Arduino hardwares, libraries and projects
- [Awesome Arduino](#awesome-Arduino)
    - [IDE](#ide)
    - [Librarys](#librarys)
    - [Hardware](#hardware)
    - [Tutorials](#tutorials)
    - [Projects](#projects)
- [Contributing](#contributing)

## IDE

*Arduino developments*

* [arduino](https://github.com/arduino/Arduino) - office arduino ide
* [ESP8266 IDE](https://github.com/esp8266/Arduino) - Arduino IDE for ESP8266
* [Stino](https://github.com/Robot-Will/Stino) - A Sublime Text Plugin for Arduino
* [attiny](https://github.com/damellis/attiny) - ATtiny microcontroller support for the Arduino IDE

## Librarys

*Hardware dependent library*
* [rtclib](https://github.com/jcw/rtclib) - a lightweight date and time library for JeeNodes and Arduino's
* [si4432](https://github.com/theGanymedes/si4432) - Si4432 Library for Arduino 
* [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 library for RFM69W and RFM69HW (semtech SX1231, SX1231H) 
* [RFM12B](https://github.com/LowPowerLab/RFM12B) - Arduino RFM12B library
* [SPIFlash](https://github.com/LowPowerLab/SPIFlash) - Arduino library for read/write access to SPI flash memory chips 
* [ov7670-no-ram-arduino-uno](https://github.com/ComputerNerd/ov7670-no-ram-arduino-uno) -  arduino library for ov7670
* [DHT-sensor-library](https://github.com/adafruit/DHT-sensor-library) - Arduino library for DHT11DHT22, etc Temp & Humidity Sensors
* [Arduino-DHT22](https://github.com/nethoncho/Arduino-DHT22) - Arduino library for the DHT22 humidity and temperature sensor
* [rfid](https://github.com/miguelbalboa/rfid) - Arduino RFID Library for MFRC522
* [433Utils](https://github.com/ninjablocks/433Utils) - a collection of code and documentation to assist you usage of RF 433MHz modules 
* [esp-link](https://github.com/jeelabs/esp-link) - esp8266 wifi-serial bridge, outbound TCP, and arduino/AVR/LPC/NXP programmer 
* [Arduino-Temperature-Control-Library](https://github.com/milesburton/Arduino-Temperature-Control-Library) - Arduino Library for Maxim Temperature Integrated Circuits
* [RF24](https://github.com/maniacbug/RF24) - Arduino driver for nRF24L01
* [due_can](https://github.com/collin80/due_can) - Object oriented canbus library for Arduino Due compatible boards 
* [FastLED](https://github.com/FastLED/FastLED) - easily & efficiently controlling a wide variety of LED chipsets for arduino
* [iot-playground](https://github.com/iot-playground/Arduino) - ESP8266 EasyIoT library and sensor examples
* [espduino](https://github.com/tuanpmt/espduino) - ESP8266 network client (mqtt, restful) for Arduino
* [esp_mqtt](https://github.com/tuanpmt/esp_mqtt) - MQTT client library for ESP8266 Soc
* [esp_bridge](https://github.com/tuanpmt/esp_bridge) - ESP8266 firmware SLIP Command, support mqtt, restful client
* [Ultrasonic-HC-SR04](https://github.com/JRodrigoTech/Ultrasonic-HC-SR04) - Ultrasonic HC-SR04 library for Arduino
* [MPU-9250](https://github.com/kriswiner/MPU-9250) - Arduino sketch for MPU-9250 9DoF with AHRS sensor fusion 
* [MPU-6050](https://github.com/kriswiner/MPU-6050) - Basic MPU-6050 Arduino sketch of sensor function
* [MPU-9150](https://github.com/kriswiner/MPU-9150) - Arduino sketch for MPU-9150 9DoF with AHRS sensor fusion 
* [LSM9DS0](https://github.com/kriswiner/LSM9DS0) - LSM9DS0 9DOF sensor AHRS sketch 
* [arduino-ds1302](https://github.com/msparks/arduino-ds1302) - Arduino library for the DS1302 Real Time Clock chip 
* [xbee-arduino](https://github.com/andrewrapp/xbee-arduino) - Arduino library for communicating with XBees in API mode
* [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt) - New Arduino interrupt library, designed for Arduino Uno/Mega 2560/Leonardo/Due 

*Bootloader*
* [HoodLoader2](https://github.com/NicoHood/HoodLoader2) - 16u2 Bootloader to reprogram 16u2 + 328/2560 with Arduino IDE 

*Hardware undependent library*

* [johnny-five](https://github.com/rwaldron/johnny-five) - JavaScript Robotics and IoT programming framework, Based on Arduino Firmata Protocol
* [grbl](https://github.com/grbl/grbl) - An open source, embedded, high performance g-code-parser and CNC milling controller run on a straight Arduino
* [Arduino-IRremote](https://github.com/z3t0/Arduino-IRremote) - Infrared remote library for Arduino: send and receive infrared signals ....
* [IRLib](https://github.com/cyborg5/IRLib) - An Arduino library for encoding and decoding infrared remote signals 
* [i2cdevlib](https://github.com/jrowberg/i2cdevlib) - I2C device library collection for AVR/Arduino or other C++-based MCUs
* [keysweeper](https://github.com/samyk/keysweeper) - KeySweeper is a stealthy Arduino-based device, camouflaged as a functioning USB ....
* [aJson](https://github.com/interactive-matter/aJson) - an Arduino library to enable JSON processing with Arduino
* [ArduinoJson](https://github.com/bblanchon/ArduinoJson) - An efficient JSON library for embedded systems
* [USB_Host_Shield_2.0](https://github.com/felis/USB_Host_Shield_2.0) - Revision 2.0 of USB Host Library for Arduino
* [pubsubclient](https://github.com/knolleary/pubsubclient) - A client library for the Arduino Ethernet Shield that provides support for MQTT
* [WiringPi](https://github.com/WiringPi/WiringPi) - Gordon's Arduino wiring-like WiringPi Library for the Raspberry Pi
* [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library) -  a pid library for arduino writed by c++
* [firmata](https://github.com/firmata/arduino) - firmata firmware for arduino
* [uSpeech](https://github.com/arjo129/uSpeech) - Speech recognition toolkit for the arduino
* [Low-Power](https://github.com/rocketscream/Low-Power) - Low Power Library for Arduino
* [ArduinoOBD](https://github.com/stanleyhuangyc/ArduinoOBD) - OBD-II library and sketches for Arduino
* [hiduino](https://github.com/ddiakopoulos/hiduino) - Native USB-MIDI on the Arduino
* [Brain](https://github.com/kitschpatrol/Brain) - Arduino library for reading Neurosky EEG brainwave data
* [Cryptosuite](https://github.com/Cathedrow/Cryptosuite) - Cryptographic suite for Arduino (SHA, HMAC-SHA) 
* [Timer](https://github.com/JChristensen/Timer) - A fork of Simon Monk's Arduino Timer library
* [PJON](https://github.com/gioblu/PJON) - One wire multi master device communications bus system for Arduino 
* [HID](https://github.com/NicoHood/HID) - enhanced HID functions for Arduino
* [Cosa](https://github.com/mikaelpatel/Cosa) - An Object-Oriented Platform for Arduino Programming
* [NDEF](https://github.com/don/NDEF) - Read and Write NDEF Messages to NFC tags with Arduino
* [rosserial](https://github.com/ros-drivers/rosserial) - ROS client library for small, embedded devices, such as Arduino
* [SdFat](https://github.com/greiman/SdFat) - Arduino FAT16/FAT32 Library 
* [Arduino-Communicator](https://github.com/jeppsson/Arduino-Communicator) - Very simple Android application for communicating with Arduino
* [Arduino-PID-AutoTune-Library](https://github.com/br3ttb/Arduino-PID-AutoTune-Library) - 
* [TinyGPS](https://github.com/mikalhart/TinyGPS) - A compact Arduino NMEA (GPS) parsing library 
* [Arduino-EEPROMEx](https://github.com/thijse/Arduino-EEPROMEx) - Extended EEPROM library for Arduino 
* [Arduino-CmdMessenger](https://github.com/thijse/Arduino-CmdMessenger) - CmdMessenger Communication library for Arduino
* [arduino-libs-manchester](https://github.com/mchr3k/arduino-libs-manchester) - Arduino Manchester Encoding
* [ShiftPWM](https://github.com/elcojacobs/ShiftPWM) - Arduino Library for software PWM with shift registers 
* [Arduino-EasyTransfer](https://github.com/madsci1016/Arduino-EasyTransfer) - An Easy way to Transfer data between Arduinos
* [Arduino-SerialCommand](https://github.com/kroimon/Arduino-SerialCommand) - A Wiring/Arduino library to tokenize and parse commands received over a serial port.
* [TMRpcm](https://github.com/TMRh20/TMRpcm) - Arduino library for asynchronous playback of PCM/WAV files direct from SD card
* [Mozzi](https://github.com/sensorium/Mozzi) - a sound synthesis library for Arduino
* [StandardCplusplus](https://github.com/maniacbug/StandardCplusplus) - Standard C++ for Arduino (port of uClibc++) 
* [ArdOSC](https://github.com/recotana/ArdOSC) - Open Sound Control(OSC) Library for Arduino 
* [arduino-serial](https://github.com/todbot/arduino-serial) - Example C and Java host code to talking to an arduino or other "serial" device
* [Bounce2](https://github.com/thomasfredericks/Bounce2) - Debouncing library for Arduino or Wiring 
* [ArduinoThread](https://github.com/ivanseidel/ArduinoThread) - A simple way to run Threads on Arduino
* [AdaEncoder](https://github.com/GreyGnome/AdaEncoder) - Library for handling quadrature encoders for the Arduino microcontroller
* [PinChangeInt](https://github.com/GreyGnome/PinChangeInt) - Pin Change Interrupt library for the Arduino

*3D Printer .*

* [Repetier-Firmware](https://github.com/repetier/Repetier-Firmware) - Firmware for Arduino based RepRap 3D printer
* [3D_Printer](https://github.com/underverk/3D_Printer) - Underverk's 3D printer 

*NET Library.*

* [Webduino](https://github.com/sirleech/Webduino) - Arduino WebServer library
* [TinyWebServer](https://github.com/ovidiucp/TinyWebServer) - Small web server for Arduino, fits in 10KB ROM, less than 512 bytes RAM
* [RESTduino](https://github.com/jjg/RESTduino) - A sketch to provide a REST-like interface to the Arduino+Ethernet Shield
* [aREST](https://github.com/marcoschwartz/aREST) - A RESTful environment for Arduino
* [ArduinoWebsocketClient](https://github.com/krohling/ArduinoWebsocketClient) - Websocket client for Arduino
* [ArduinoWebsocketServer](https://github.com/ejeklint/ArduinoWebsocketServer) - a Websocket server running on an Arduino
* [esp8266](https://github.com/ssokol/esp8266) - ESP8266 Wifi library and sample code for Arduino 
* [arduino_uip](https://github.com/ntruchsess/arduino_uip) - A plugin-replacement of the stock Arduino Ethernet library
* [HttpClient](https://github.com/nmattisson/HttpClient) - Http Client Library for the Spark Core
* [arduino-restclient](https://github.com/csquared/arduino-restclient) - Arduino RESTful HTTP Request Library 
* [socket.io-arduino-client](https://github.com/billroy/socket.io-arduino-client) - A socket.io client for the Arduino Ethernet shield 
* [Arduino-IPv6Stack](https://github.com/telecombretagne/Arduino-IPv6Stack) - IPv6 stack for Arduino and Xbee based on Contiki OS network stack 
* [MQTT-SN-Arduino](https://github.com/boriz/MQTT-SN-Arduino) - Connecting mesh network to the MQTT broker and tunneling MQTT protocol over Websocket

*Command line library*

* [platformio](https://github.com/platformio/platformio) - Cross-platform code builder and the missing library manager ....
* [arduino-cmake](https://github.com/queezythegreat/arduino-cmake) - Arduino CMake Build system 
* [bitlash](https://github.com/billroy/bitlash) - a programmable command shell for arduino
* [arduino-shell](https://github.com/nuket/arduino-shell) - A command shell to interact with built-in Arduino features

*Display library*

* [Arduino_LCD_Menu](https://github.com/DavidAndrews/Arduino_LCD_Menu) -  creates menu systems primarily useful for 16x2 or 16x4 LCD displays
* [LiquidTWI2](https://github.com/lincomatic/LiquidTWI2) - high speed I2C LCD Library for Arduino, which supports MCP23008 and MCP23017 
* [MENWIZ](https://github.com/brunialti/MENWIZ) - LCD menu library: short user code to manage complex menu structures 
* [u8glib](https://github.com/olikraus/u8glib) - Arduino Monochrom Graphics Library for LCDs and OLEDs
* [ucglib](https://github.com/olikraus/ucglib) - Arduino True Color Library for TFTs and OLEDs

*Host side Control library*

* [Python-Arduino-Command-API](https://github.com/thearn/Python-Arduino-Command-API) - A Python library for communicating with Arduino microcontroller boards
* [hidapi](https://github.com/signal11/hidapi) - A Simple library for communicating with USB and Bluetooth HID devices on Linux, Mac, and Windows. 
* [BlocklyDuino](https://github.com/BlocklyDuino/BlocklyDuino) - a web-based visual programming editor for arduino
* [noduino](https://github.com/sbstjn/noduino) - JavaScript and Node.js Framework for controlling Arduino with HTML and WebSockets
* [ino](https://github.com/amperka/ino) - Command line toolkit for working with Arduino hardware
* [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android) - Android USB host serial driver library for CDC, FTDI, Arduino and other devices
* [rad](https://github.com/atduskgreg/rad) - Ruby Arduino Development
* [ardublock](https://github.com/taweili/ardublock) - a Block Programming Language for Arduino
* [arduino-api](https://github.com/plotly/arduino-api) - Arduino library for real-time logging and streaming data to online plotly graphs
* [dino](https://github.com/austinbv/dino) - Dino is a ruby gem that helps you bootstrap prototyping with an Arduino
* [duino](https://github.com/ecto/duino) - Arduino framework for node.js

## Hardware

*Hardware board for arduino diy.*

* [Teensy](https://www.pjrc.com/teensy/) - The Teensy is a complete USB-based microcontroller development system


## Tutorials

*Tutorials for arduino flash.*

* [Arduino-Tutorial-Series](https://github.com/sciguy14/Arduino-Tutorial-Series) - These are the supporting materials for my popular series of Tutorials on the Arduino Microcontroller Platform
* [esp8266](https://github.com/raburton/esp8266) - Various bits of code for ESP8266
* [arduino_sketches](https://github.com/nickgammon/arduino_sketches) - Publicly-released sketches for the Arduino microprocessor 
* [DUEZoo](https://github.com/manitou48/DUEZoo) - Some proof-of-concept sketches and results for Arduino DUE
* [programming_arduino](https://github.com/simonmonk/programming_arduino) - Code for the book Programming Arduino: Getting Started with Sketches 
* [ArduinoExamples](https://github.com/renaun/ArduinoExamples) - Arduino Blink Chrome App

## Projects

*Projects with Arduino*

* [Erbbie Desktop Garden](http://www.instructables.com/id/Erbbie-Desktop-Smart-Garden/?ALLSTEPS) - is a DIY desktop smart garden designed to give everyone a green thumb
* [Smart Garden](http://www.instructables.com/id/Smart-Garden-1/?ALLSTEPS) - clone of Erbbie
* [arduinoscope](https://github.com/konsumer/arduinoscope) - An oscilloscope, using arduino and processing/node
* [ArduinoPlot](https://github.com/gregpinero/ArduinoPlot) - Real time Plot Numeric Values sent from Arduino over Serial Port 
* [ArduinoISP](https://github.com/rsbohn/ArduinoISP) - Use the Arduino to program AVR chips


## Contributing
* [Contributing](https://github.com/lembed/awesome-arduino/blob/master/CONTRIBUTING.md)

Your contributions are always welcome!

[![Analytics](https://ga-beacon.appspot.com/UA-67438080-1/awesome-arduino/readme?pixel)](https://github.com/Lembed/awesome-arduino)
