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
* [arduino-cmake](https://github.com/queezythegreat/arduino-cmake) - Arduino CMake Build system 

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

*Bootloader*
* [HoodLoader2](https://github.com/NicoHood/HoodLoader2) - 16u2 Bootloader to reprogram 16u2 + 328/2560 with Arduino IDE 

*Hardware undependent library*

* [johnny-five](https://github.com/rwaldron/johnny-five) - JavaScript Robotics and IoT programming framework, Based on Arduino Firmata Protocol
* [grbl](https://github.com/grbl/grbl) - An open source, embedded, high performance g-code-parser and CNC milling controller run on a straight Arduino
* [Arduino-IRremote](https://github.com/z3t0/Arduino-IRremote) - Infrared remote library for Arduino: send and receive infrared signals ....
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

*Command line library*

* [platformio](https://github.com/platformio/platformio) - Cross-platform code builder and the missing library manager ....
* [arduino-cmake](https://github.com/queezythegreat/arduino-cmake) - Arduino CMake Build system 
* [bitlash](https://github.com/billroy/bitlash) - a programmable command shell for arduino

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


## Projects

*Projects with Arduino*

* Domotic
  * [Erbbie Desktop Garden](http://www.instructables.com/id/Erbbie-Desktop-Smart-Garden/?ALLSTEPS) - is a DIY desktop smart garden designed to give everyone a green thumb
  * [Smart Garden](http://www.instructables.com/id/Smart-Garden-1/?ALLSTEPS) - clone of Erbbie


## Contributing
* [Contributing](https://github.com/lembed/awesome-arduino/blob/master/CONTRIBUTING.md)

Your contributions are always welcome!

[![Analytics](https://ga-beacon.appspot.com/UA-67438080-1/awesome-arduino/readme?pixel)](https://github.com/Lembed/awesome-arduino)
