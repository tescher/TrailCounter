/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MANUAL,OFF"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Counts triggered from the Passive Infrared device
volatile unsigned long PIR_count;
volatile unsigned int count_addr;
const byte PIR_pin = 0;

// Interrupt handler triggered by PIR. Update the count and flash the LED
void update_count() {
  ble.sendCommandCheckOK("AT+HWModeLED=MANUAL,ON");
  delay(3000);
  ble.sendCommandCheckOK("AT+HWModeLED=MANUAL,OFF");
  PIR_count++;
  if (count_addr < EEPROM.length() - 4) {
    count_addr += 4;
  } else {
    count_addr = 4;
  }    
  unsigned long count = PIR_count;
  EEPROM.put(count_addr, count);
  if (count_addr < EEPROM.length() - 4) {
    EEPROM.put(count_addr + 4, 0L);
  } else {
    EEPROM.put(4, 0L); // Beginning rollover, put 00 flag at the beginning
  }
}

// Reset the trail count in the EEPROM. Sets the first 4 bytes to 0xAF to signal that the EEPROM has been initialized
void reset_count() {
  // Stop counts for now
  detachInterrupt(digitalPinToInterrupt(PIR_pin));
  for (int i = 0; i < 4; i++) {
    EEPROM.write(i,0xAF);
  }
  for (int i = 4; i < EEPROM.length(); i++) {
    EEPROM.write(i,0);
  }
  PIR_count = 0;
  count_addr = 4; // First count location is past the initialization flag
  // Start counts again
  attachInterrupt(digitalPinToInterrupt(PIR_pin), update_count, RISING);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  /* while (!Serial);  // required for Flora & Micro */
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!


  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK(F("AT+HWModeLED=" MODE_LED_BEHAVIOUR));
    Serial.println(F("******************************"));
  }

  // Set up PIR reader pin
  pinMode(PIR_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_pin), update_count, RISING);

  // Initialize the EEPROM if necessary. If first 4 bytes are AFAFAFAF assume it is initialized
  // If already initialized, scan to find first 4 0x00 bytes. Previous 4 bytes are the current count.
  unsigned long flag;
  EEPROM.get(0, flag);
  if (flag != 0xAFAFAFAF) {
    reset_count();
    Serial.println(F("Init flag not found, resetting count."));
  } else {
    count_addr = 0;
    bool found_count = false;
    unsigned long flag;
    unsigned long count;
    do {
      EEPROM.get(count_addr + 4, flag);
      if (flag == 0) {  // Found our signal flag, previous 4 bytes should be the count
        if (count_addr == 0) { // If flag was right at the beginning, look at the end of the EEPROM for the count
          EEPROM.get(EEPROM.length() - 4, count); 
          count_addr = EEPROM.length() - 4;
        } else {
          EEPROM.get(count_addr, count);
        }
        found_count = true;
        PIR_count = count;
        Serial.print(F("Found count: "));
        Serial.println(count);
        Serial.print(F("at address: "));
        Serial.println(count_addr);
      } else {
        count_addr += 4;
      }  
    } while (!found_count && (count_addr + 4 < EEPROM.length()));
    if (!found_count) { // Didn't find count, something's corrupted. Reset
      Serial.println(F("Flag found, but no count found. Resetting count."));
      reset_count();
    }
  }
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Check for incoming characters from Bluefruit
  ble.println(F("AT+BLEUARTRX"));
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, it's in the buffer
  Serial.print(F("[Recv] ")); 
  Serial.println(ble.buffer);

  // Send count command
  if (strcmp(ble.buffer, "count") == 0) {
    ble.waitForOK();
    // Send count to Bluefruit
    Serial.print(F("[Sending count] "));
    Serial.println(PIR_count);

    ble.print(F("AT+BLEUARTTX="));
    ble.println(PIR_count);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send count?"));
    }
  } else {
    ble.waitForOK();
  }  

  // Reset count command
  if (strcmp(ble.buffer, "reset") == 0) {
    ble.waitForOK();
    // Reset the count
    reset_count();
    Serial.println(F("[Resetting count] "));

    ble.print(F("AT+BLEUARTTX="));
    ble.println("Count reset");

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send reset?"));
    }
  } else {
    ble.waitForOK();
  }  

}


