# awesome-arduino
A curated list of awesome Arduino hardware, libraries and projects
- [Awesome Arduino](#awesome-Arduino)
 	- [Libraries](#libraries)
    - [HardwareBoard](#hardwareboard)
    - [IDE](#ide)
    - [Tutorials](#tutorials)
    - [Projects](#projects)
- [Contributing](#contributing)


## Libraries

*Hardware dependent library*
* [rtclib](https://github.com/jcw/rtclib) - A lightweight date and time library for JeeNodes and Arduinos
* [RFM69](https://github.com/LowPowerLab/RFM69) - RFM69 library for RFM69W and RFM69HW (Semtech SX1231, SX1231H)
* [RFM12B](https://github.com/LowPowerLab/RFM12B) - Arduino RFM12B library
* [SPIFlash](https://github.com/LowPowerLab/SPIFlash) - Arduino library for read/write access to SPI flash memory chips
* [Winbond_flash](https://github.com/Marzogh/SPIFlash) - SPI library for Winbond Flash Memory modules
* [ov7670-no-ram-arduino-uno](https://github.com/ComputerNerd/ov7670-no-ram-arduino-uno) -  Arduino library for OV7670
* [DHT-sensor-library](https://github.com/adafruit/DHT-sensor-library) - Arduino library for DHT11DHT22, etc Temp & Humidity Sensors
* [Arduino-DHT22](https://github.com/nethoncho/Arduino-DHT22) - Arduino library for the DHT22 humidity and temperature sensor
* [rfid](https://github.com/miguelbalboa/rfid) - Arduino RFID Library for MFRC522
* [433Utils](https://github.com/ninjablocks/433Utils) - A collection of code and documentation to assist your usage of RF 433MHz modules
* [esp-link](https://github.com/jeelabs/esp-link) - ESP8266 wifi-serial bridge, outbound TCP, and Arduino/AVR/LPC/NXP programmer
* [Arduino-Temperature-Control-Library](https://github.com/milesburton/Arduino-Temperature-Control-Library) - Arduino Library for Maxim Temperature Integrated Circuits
* [RF24](https://github.com/maniacbug/RF24) - Arduino driver for nRF24L01
* [RF24](https://github.com/TMRh20/RF24) - Optimized fork of nRF24L01 for Arduino and Raspberry Pi
* [RF24Mesh](https://github.com/TMRh20/RF24Mesh) - Mesh Networking for RF24Network
* [due_can](https://github.com/collin80/due_can) - Object oriented CANbus library for Arduino Due compatible boards
* [FastLED](https://github.com/FastLED/FastLED) - Easily & efficiently controlling a wide variety of LED chipsets for Arduino
* [iot-playground](https://github.com/iot-playground/Arduino) - ESP8266 EasyIoT library and sensor examples
* [espduino](https://github.com/tuanpmt/espduino) - ESP8266 network client (MQTT, restful) for Arduino
* [esp_mqtt](https://github.com/tuanpmt/esp_mqtt) - MQTT client library for ESP8266 Soc
* [esp_bridge](https://github.com/tuanpmt/esp_bridge) - ESP8266 firmware SLIP Command, support mqtt, RESTful client
* [Ultrasonic-HC-SR04](https://github.com/JRodrigoTech/Ultrasonic-HC-SR04) - Ultrasonic HC-SR04 library for Arduino
* [MPU-9250](https://github.com/kriswiner/MPU-9250) - Arduino sketch for MPU-9250 9DoF with AHRS sensor fusion
* [MPU-6050](https://github.com/kriswiner/MPU-6050) - Basic MPU-6050 Arduino sketch of sensor function
* [MPU-9150](https://github.com/kriswiner/MPU-9150) - Arduino sketch for MPU-9150 9DoF with AHRS sensor fusion
* [LSM9DS0](https://github.com/kriswiner/LSM9DS0) - LSM9DS0 9DOF sensor AHRS sketch
* [arduino-ds1302](https://github.com/msparks/arduino-ds1302) - Arduino library for the DS1302 Real Time Clock chip
* [xbee-arduino](https://github.com/andrewrapp/xbee-arduino) - Arduino library for communicating with XBees in API mode
* [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt) - New Arduino interrupt library, designed for Arduino Uno/Mega 2560/Leonardo/Due
* [DS3232RTC](https://github.com/JChristensen/DS3232RTC) - Arduino Library for Maxim Integrated DS3232 and DS3231 Real-Time Clocks
* [ds3231](https://github.com/rodan/ds3231) - Arduino library for DS3231 RTC
* [RTClib](https://github.com/adafruit/RTClib) -  RTC library
* [arduino-ds1302](https://github.com/msparks/arduino-ds1302) - Arduino library for the DS1302 Real Time Clock chip
* [HX711](https://github.com/bogde/HX711) - An Arduino library Semiconductor HX711 ADC for Weight Scales.
* [MiniPirate](https://github.com/chatelao/MiniPirate) - Arduino Serial Command Tool
* [LedControl](https://github.com/wayoda/LedControl) - An Arduino library for MAX7219 and MAX7221 Led display drivers
* [arduino-BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral) - Arduino library for creating custom BLE peripherals with Nordic Semiconductor's nRF8001 or nR51822
* [BTLE](https://github.com/floe/BTLE) - Library for basic Bluetooth Low Energy with the nRF24L01+
* [BH1750](https://github.com/claws/BH1750) - An Arduino library for the digital light sensor BH1750FVI
* [Arduino-GP2Y0A21YK-library](https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library) - Arduino library for the Sharp GP2Y0A21YK IR Distance sensor
* [Arduino-distance-sensor-library](https://github.com/jeroendoggen/Arduino-distance-sensor-library) - Arduino library for distance sensors
* [arduino-BLEPeripheral](https://github.com/sandeepmistry/arduino-BLEPeripheral) - Library for creating custom BLE peripherals with Nordic Semiconductor's nRF8001 or nR51822
* [Arduino-Temperature-Control-Library](https://github.com/milesburton/Arduino-Temperature-Control-Library) - Arduino Library for Maxim Temperature Integrated Circuits
* [RGB-matrix-Panel](https://github.com/adafruit/RGB-matrix-Panel) - Arduino library and example code for the 16x32 RGB matrix panels
* [Time](https://github.com/PaulStoffregen/Time) - Time library for Arduino
* [modbusino](https://github.com/stephane/modbusino) - Small Modbus slave, RTU (serial) for Arduino
* [simplemodbusng](https://github.com/angeloc/simplemodbusng) - Modbus RTU Slave/Master for the Arduino

*Bootloader*

* [HoodLoader2](https://github.com/NicoHood/HoodLoader2) - 16u2 Bootloader to reprogram 16u2 + 328/2560 with Arduino IDE
* [optiboot](https://github.com/Optiboot/optiboot) - Small and Fast Bootloader for Arduino and other Atmel AVR chips

*Hardware independent library*

* [Arduino-IRremote](https://github.com/z3t0/Arduino-IRremote) - Infrared remote library for Arduino: send and receive infrared signals ....
* [IRLib](https://github.com/cyborg5/IRLib) - An Arduino library for encoding and decoding infrared remote signals
* [i2cdevlib](https://github.com/jrowberg/i2cdevlib) - I2C device library collection for AVR/Arduino or other C++-based MCUs
* [keysweeper](https://github.com/samyk/keysweeper) - KeySweeper is a stealthy Arduino-based device, camouflaged as a functioning USB ....
* [aJson](https://github.com/interactive-matter/aJson) - an Arduino library to enable JSON processing with Arduino
* [ArduinoJson](https://arduinojson.org) - C++ JSON library for IoT. Simple and efficient.
* [json-streaming-parser](https://github.com/squix78/json-streaming-parser) - Library for parsing potentially huge json streams on devices with scarce memory
* [USB_Host_Shield_2.0](https://github.com/felis/USB_Host_Shield_2.0) - Revision 2.0 of USB Host Library for Arduino
* [pubsubclient](https://github.com/knolleary/pubsubclient) - A client library for the Arduino Ethernet Shield that provides support for MQTT
* [WiringPi](https://github.com/WiringPi/WiringPi) - Gordon's Arduino wiring-like WiringPi Library for the Raspberry Pi
* [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library) - A pid library for Arduino written in C++
* [firmata](https://github.com/firmata/arduino) - Firmata firmware for Arduino
* [uSpeech](https://github.com/arjo129/uSpeech) - Speech recognition toolkit for the Arduino
* [Talkie](https://github.com/going-digital/Talkie) - Speech library for Arduino
* [Low-Power](https://github.com/rocketscream/Low-Power) - Low Power Library for Arduino
* [ArduinoOBD](https://github.com/stanleyhuangyc/ArduinoOBD) - OBD-II library and sketches for Arduino
* [hiduino](https://github.com/ddiakopoulos/hiduino) - Native USB-MIDI on the Arduino
* [Brain](https://github.com/kitschpatrol/Brain) - Arduino library for reading Neurosky EEG brainwave data
* [Cryptosuite](https://github.com/Cathedrow/Cryptosuite) - Cryptographic suite for Arduino (SHA, HMAC-SHA)
* [Timer](https://github.com/JChristensen/Timer) - A fork of Simon Monk's Arduino Timer library
* [PJON](https://github.com/gioblu/PJON) - One wire multi master device communications bus system for Arduino
* [HID](https://github.com/NicoHood/HID) - Enhanced HID functions for Arduino
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
* [Mozzi](https://github.com/sensorium/Mozzi) - A sound synthesis library for Arduino
* [StandardCplusplus](https://github.com/maniacbug/StandardCplusplus) - Standard C++ for Arduino (port of uClibc++)
* [ArdOSC](https://github.com/recotana/ArdOSC) - Open Sound Control(OSC) Library for Arduino
* [OSC](https://github.com/CNMAT/OSC) - Arduino and Teensy implementation of OSC encoding
* [arduino-serial](https://github.com/todbot/arduino-serial) - Example C and Java host code to talking to an Arduino or other "serial" device
* [Bounce2](https://github.com/thomasfredericks/Bounce2) - Debouncing library for Arduino or Wiring
* [ArduinoThread](https://github.com/ivanseidel/ArduinoThread) - A simple way to run Threads on Arduino
* [AdaEncoder](https://github.com/GreyGnome/AdaEncoder) - Library for handling quadrature encoders for the Arduino microcontroller
* [PinChangeInt](https://github.com/GreyGnome/PinChangeInt) - Pin Change Interrupt library for the Arduino
* [Arduino-RFID](https://github.com/mattwilliamson/Arduino-RFID) - Arduino RFID reader with computer serial client
* [arcore](https://github.com/rkistner/arcore) - MIDI-USB Support for Arduino
* [arduino-base64](https://github.com/adamvr/arduino-base64) - A base64 library for the arduino platform, written in C
* [DirectIO](https://github.com/mmarchetti/DirectIO) - Fast, simple I/O library for Arduino
* [arduino-EventManager](https://github.com/igormiktor/arduino-EventManager) - An event handling system for Arduino
* [OneButton](https://github.com/mathertel/OneButton) - An Arduino library for using a single button for multiple purpose input
* [JTAG](https://github.com/mrjimenez/JTAG) - JTAG library for Arduino
* [Sleep_n0m1](https://github.com/n0m1/Sleep_n0m1) - A library that sets the Arduino into sleep mode for a specified length of time
* [AESLib](https://github.com/DavyLandman/AESLib) - Arduino Library for AES Encryption
* [MemoryFree](https://github.com/maniacbug/MemoryFree) - Arduino MemoryFree library
* [souliss](https://github.com/souliss/souliss) - Arduino based Distributed Networking Framework for Smart Homes and IoT
* [LED-Matrix](https://github.com/marcmerlin/LED-Matrix) - Single/bi/tri-color LED Matrix PWM driver for arduino
* [OneWire](https://github.com/PaulStoffregen/OneWire) - Library for Dallas/Maxim 1-Wire Chips
* [TinyEKF](https://github.com/simondlevy/TinyEKF) - Lightweight C/C++ Extended Kalman Filter with Arduino example
* [arduino_midi_library](https://github.com/FortySevenEffects/arduino_midi_library) - MIDI for Arduino
* [FreeRTOS-Arduino](https://github.com/greiman/FreeRTOS-Arduino) - FreeRTOS 8.2.3 Arduino Libraries
* [Arduino_FreeRTOS_Library](https://github.com/feilipu/Arduino_FreeRTOS_Library) - A FreeRTOS Library for all Arduino AVR Devices
* [Automaton](https://github.com/tinkerspy/Automaton) - Reactive State Machine Framework for Arduino
* [LinkedList](https://github.com/ivanseidel/LinkedList) - A fully implemented LinkedList made to work with Arduino projects
* [VirtualUsbKeyboard](https://github.com/practicalarduino/VirtualUsbKeyboard) - Virtual an Arduino as a HID device
* [virtual-shields-arduino](https://github.com/ms-iot/virtual-shields-arduino) - Windows Virtual Shields for Arduino library

*3D Printer .*

* [Repetier-Firmware](https://github.com/repetier/Repetier-Firmware) - Firmware for Arduino based RepRap 3D printer
* [3D_Printer](https://github.com/underverk/3D_Printer) - Underverk's 3D printer
* [grbl](https://github.com/grbl/grbl) - An open source, embedded, high performance g-code-parser and CNC milling controller run on a straight Arduino

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
* [ArduinoDuePolarSSLClient](https://github.com/tomconte/ArduinoDuePolarSSLClient) - Sample Arduino Due sketch connecting to a PolarSSL server using PSK authentication
* [arduino-mqtt](https://github.com/256dpi/arduino-mqtt) - MQTT library for Arduino based on the Eclipse Paho projects
* [aWOT](https://github.com/lasselukkari/aWOT) - Arduino web server library
* [arduinoWebSockets](https://github.com/Links2004/arduinoWebSockets) - WebSocket Server and Client for Arduino
* [HttpClient](https://github.com/amcewen/HttpClient) - Arduino HTTP library
* [aws-iot-device-sdk-arduino-yun](https://github.com/aws/aws-iot-device-sdk-arduino-yun) - SDK for connecting to AWS IoT from an Arduino Yún.
* [HttpClient](https://github.com/amcewen/HttpClient) - Arduino HTTP library
* [microcoap](https://github.com/1248/microcoap) - A small CoAP implementation for microcontrollers

*Command line library*

* [platformio](https://github.com/platformio/platformio) - Cross-platform code builder and the missing library manager ....
* [arduino-cmake](https://github.com/queezythegreat/arduino-cmake) - Arduino CMake Build system
* [bitlash](https://github.com/billroy/bitlash) - A programmable command shell for Arduino
* [arduino-shell](https://github.com/nuket/arduino-shell) - A command shell to interact with built-in Arduino features

*Display library*

* [Arduino_LCD_Menu](https://github.com/DavidAndrews/Arduino_LCD_Menu) - Creates menu systems primarily useful for 16x2 or 16x4 LCD displays
* [LiquidMenu](https://github.com/VaSe7u/LiquidMenu) - Menu creation library for HD44780 LCDs (parallel and I2C), wraps LiquidCrystal.
* [LiquidTWI2](https://github.com/lincomatic/LiquidTWI2) - high speed I2C LCD Library for Arduino, which supports MCP23008 and MCP23017
* [MENWIZ](https://github.com/brunialti/MENWIZ) - LCD menu library: Short user code to manage complex menu structures
* [u8glib](https://github.com/olikraus/u8glib) - Arduino Monochrome Graphics Library for LCDs and OLEDs
* [ucglib](https://github.com/olikraus/ucglib) - Arduino True Color Library for TFTs and OLEDs

*Binding and API library*

* [johnny-five](https://github.com/rwaldron/johnny-five) - JavaScript Robotics and IoT programming framework, Based on Arduino Firmata Protocol
* [Python-Arduino-Command-API](https://github.com/thearn/Python-Arduino-Command-API) - A Python library for communicating with Arduino microcontroller boards
* [hidapi](https://github.com/signal11/hidapi) - A Simple library for communicating with USB and Bluetooth HID devices on Linux, Mac, and Windows.
* [BlocklyDuino](https://github.com/BlocklyDuino/BlocklyDuino) - a web-based visual programming editor for arduino
* [noduino](https://github.com/sbstjn/noduino) - JavaScript and Node.js Framework for controlling Arduino with HTML and WebSockets
* [ino](https://github.com/amperka/ino) - Command line toolkit for working with Arduino hardware
* [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android) - Android USB host serial driver library for CDC, FTDI, Arduino and other devices
* [serial-port-json-server](https://github.com/johnlauer/serial-port-json-server) - A serial port JSON websocket server communicate with Arduino
* [PhysicaloidLibrary](https://github.com/ksksue/PhysicaloidLibrary) - Android Library for communicating with physical-computing boards
* [blynk-server](https://github.com/blynkkk/blynk-server) - Platform with iOS and Android apps to control Arduino
* [rad](https://github.com/atduskgreg/rad) - Ruby Arduino Development
* [ardublock](https://github.com/taweili/ardublock) - A Block Programming Language for Arduino
* [arduino-api](https://github.com/plotly/arduino-api) - Arduino library for real-time logging and streaming data to online plotly graphs
* [dino](https://github.com/austinbv/dino) - Dino is a ruby gem that helps you bootstrap prototyping with an Arduino
* [duino](https://github.com/ecto/duino) - Arduino framework for node.js
* [pyFirmata](https://github.com/tino/pyFirmata) - Python interface for the Firmata  protocol
* [JArduino](https://github.com/SINTEF-9012/JArduino) - Program your Arduino in Java
* [Python-Arduino-Command-API](https://github.com/thearn/Python-Arduino-Command-API) - A Python library for communicating with Arduino microcontroller boards
* [hwio](https://github.com/mrmorphic/hwio) - Go library for hardware I/O control, in the programming style of Arduino

*RPC*

* [ArduRPC](https://github.com/DinoTools/ArduRPC) - RPC library for Arduino and other microcontroller based boards
* [arduino-json-rpc](https://github.com/cloud-rocket/arduino-json-rpc) - Simple JSON-RPC server implementation for Arduino

*Math library*

* [Arduino-signal-filtering-library](https://github.com/jeroendoggen/Arduino-signal-filtering-library) - Arduino library for signal filtering
* [Gaussian](https://github.com/ivanseidel/Gaussian) - Gaussian filter for Arduino

## HardwareBoard

*Hardware board for arduino diy.*

* [Teensy](https://www.pjrc.com/teensy/) - The Teensy is a complete USB-based microcontroller development system
* [blackmagic](https://github.com/blacksphere/blackmagic) - In application debugger for ARM Cortex microcontrollers
* [stm32plus](https://github.com/andysworkshop/stm32plus) - C++ library for the STM32 F0, F100, F103, F107 and F4

## IDE

*Arduino developments*

* [arduino](https://github.com/arduino/Arduino) - Official Arduino IDE
* [ESP8266 IDE](https://github.com/esp8266/Arduino) - Arduino IDE for ESP8266
* [Stino](https://github.com/Robot-Will/Stino) - A Sublime Text Plugin for Arduino
* [attiny](https://github.com/damellis/attiny) - ATtiny microcontroller support for the Arduino IDE
* [arduinounit](https://github.com/mmurdoch/arduinounit) - A unit testing framework for Arduino libraries
* [Arduino-Makefile](https://github.com/sudar/Arduino-Makefile) - Makefile for Arduino sketches
* [arduino_sketches](https://github.com/nickgammon/arduino_sketches) - Publicly-released sketches for the Arduino microprocessor
* [arduino-builder](https://github.com/arduino/arduino-builder) - A command line tool for compiling Arduino sketches
* [arduinounit](https://github.com/mmurdoch/arduinounit) - A unit testing framework for Arduino libraries
* [arduino-eclipse-plugin](https://github.com/jantje/arduino-eclipse-plugin) - A plugin to make programming the Arduino in Eclipse easy
* [dueboot](https://github.com/jensnockert/dueboot) - Rust on the Arduino Due
* [Arduino-Designer](https://github.com/mbats/arduino) - Graphic Arduino designer based on Eclipse
* [XOD](https://xod.io/) - Open source visual programming language and IDE

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

* [arduinoscope](https://github.com/konsumer/arduinoscope) - An oscilloscope, using Arduino and Processing/node
* [ArduinoPlot](https://github.com/gregpinero/ArduinoPlot) - Real time Plot Numeric Values sent from Arduino over Serial Port
* [ArduinoISP](https://github.com/rsbohn/ArduinoISP) - Use the Arduino to program AVR chips
* [arduino-sms-alarm](https://github.com/mattwilliamson/arduino-sms-alarm) - An Arduino based burglar alarm that sends an SMS when motion is detected
* [QUADCOPTER_V2](https://github.com/vjaunet/QUADCOPTER_V2) - A quadcopter project based on Raspberry Pi and Arduino
* [Arduino Time Lapse Panorama Controller](http://www.instructables.com/id/Arduino-Time-Lapse-Panorama-Controller/) - The controller will rotate your GoPro over a set angle for a set duration or will rotate your GoPro for a full rotation for a set duration
* [Erbbie Desktop Garden](http://www.instructables.com/id/Erbbie-Desktop-Smart-Garden/?ALLSTEPS) - A DIY desktop smart garden designed to give everyone a green thumb
* [Smart Garden](http://www.instructables.com/id/Smart-Garden-1/?ALLSTEPS) - clone of Erbbie
* [Temperature controlled craft beer coolbox](http://www.instructables.com/id/temperature-controlled-craft-beer-coolbox/) - A coolbox in which you can control the temperature in three separate compartments

## Usage
work in 'git bash' at Windows with Git installed or Linux OS

```bash
sh update.sh

usage()
{
    echo ""
    echo " usage:"
    echo ""
    echo "./update.sh <cmd>"
    echo "    where <cmd> is one of:"
    echo "      --install-or-update     (does full installation or update.)"
    echo "      --remove 		        (removes all installed)"
    echo ""
    echo "example:"
    echo '    $ ./update.sh --install-or-update'
}
```

## Contributing
* [Contributing](https://github.com/lembed/awesome-arduino/blob/master/CONTRIBUTING.md)

Your contributions are always welcome!

[![Analytics](https://ga-beacon.appspot.com/UA-67438080-1/awesome-arduino/readme?pixel)](https://github.com/Lembed/awesome-arduino)
