Trail Counter
=============

The trail counter project is an attempt to create low-cost sensors that can count park trail users, and communicate by Bluetooth current trail user counts to a phone or other device. This project may consist of three components in the end:

1. A sensor (this project)
2. An iOS and/or Android application to communicate with the sensors
3. A website/database to collect and map trail count information

This is the Arduino-compatible code for the sensor portion of the Trail Counter project. It assumes an Adafruit Feather 32u4 Bluefruit LE board, with a PIR sensor data line attached to GPIO pin 0, (marked "0 - Rx" on the board).

The program simply creates an interrupt handler routine to update a counter every time the PIR sensor triggers. It will output the current count over a Bluetooth connection operating in UART mode in response to the string "count" coming in over the UART. It uses a lot of template code provided by the Adafruit folks.

Information about the Feather 32u4 can be found [here](https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le/overview)

Setup
=====

Parts needed:

* [Adafruit Feather 32u4 Bluefruit LE](https://www.adafruit.com/product/2829)
* [PIR sensor and conditioning circuit](https://www.adafruit.com/product/189)
* [3.7V LIon battery](https://www.adafruit.com/product/2750)

Connect the red PIR wire to the BAT pin on the Feather.
Connect the black PIR wire to the GND pin on the Feather.
Connect the yellow PIR wire to the 0 RX pin on the Feather.

Upload this program to the Feather using the Arduino IDE. You may need to install some Arduino libraries. See this [tutorial](https://learn.adafruit.com/adafruit-feather-32u4-bluefruit-le). You will also need to load the Low Power library from [RocketScream] (https://github.com/rocketscream/Low-Power). Download a ZIP of the repo, and load it in the Arduino IDE using Sketch > Include Library > Add .ZIP Library... 

Install the [iOS](https://learn.adafruit.com/bluefruit-le-connect-for-ios) or [Android](https://play.google.com/store/apps/details?id=com.adafruit.bluefruit.le.connect&hl=en) Bluefruit App.

Low Power Mode
==============
To run the device in low power mode, you need to solder a wire between a couple spots on the board. It is precise work, use a magnifying glass. First find the resistor connected to the blue LED that lights up when Bluetooth is active. Carefully solder a wire from the side of the resistor opposite the LED, and solder the other end of the wire to pin 1/TX on the board (pretty much right next to the resistor). If you successfully do this, uncomment the following line in the code to enable the low power mode. 

`    #define BT_CONNECTED_INTERRUPT`

WARNING: If you uncomment this line of code without the wire connection, the chip will only come out of sleep mode every 8 seconds, making a bluetooth connection somewhat unresponsive.


Run It
======

Plug the battery into the Feather.

Start the Bluefruit iOS or Android app. Connect to the "Adafruit Bluefruit LE" device listed, choose UART mode.

Type "count" and press SEND. It should return the current sensor count. 
Type "reset" and the count will be reset to zero.
Type "bat" to get the battery voltage level.
Type "slept" to get the number of times the device went to sleep since the last restart.
