/*********************************************************************
 
 Mini Race Car kit -- Formula E Feather robot
 
 This is an example for our nRF51822 based Bluefruit LE modules
  
 Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <string.h>
#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Wire.h>

#include <Adafruit_MotorShield.h>
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
/*=========================================================================
    APPLICATION SETTINGS

? ? FACTORYRESET_ENABLE? ?  Perform a factory reset when running this sketch
? ?
? ?                         Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
? ?                         running this at least once is a good idea.
? ?
? ?                         When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.? If you are making changes to your
? ?                         Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.? Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
? ? ? ?
? ?                         Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
---------------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE      1
#define BTVERBOSE_MODE           1
/*=========================================================================*/

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M3 & M4 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(4);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(3);

//Name your RC here
String BROADCAST_NAME = "Racer";

String BROADCAST_CMD = String("AT+GAPDEVNAME=" + BROADCAST_NAME);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

// Set your forward, reverse, and turning speeds
#define ForwardSpeed                150
#define ReverseSpeed                150
#define TurningSpeed                100


/**************************************************************************/
/*!
    @brief  Sets up the HW and the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {

  /* . Initialise Serial Communication 9600 baud*/
  Serial.begin(9600);

  /* .Initialise motor shield library with 1.6KHz default freq */
  AFMS.begin();

  /* .turn on motors */
  L_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  
  R_MOTOR->setSpeed(0);
  R_MOTOR->run(RELEASE);

  /* Initialize the Bluetooth module */
  BLEsetup();
}

void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);

  readController();

}


//TODO: Accelerometer code to control the race car


/**************************************************************************/
/*!
    @brief  Control pads for the race car moving forward, backwards, right
            and left.
*/
/**************************************************************************/
bool isMoving = false;
unsigned long lastPress = 0;

bool readController() {
  
  uint8_t maxspeed;

 /* .Button Control */
  if (packetbuffer[1] == 'B') {

    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    if (pressed) {
      isMoving = true;

      /* Forward */
      if(buttnum == 5){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(FORWARD);
        maxspeed = ForwardSpeed;
        ble.println("Forward");
      }

      /* .Backwards */
      if(buttnum == 6){
        L_MOTOR->run(BACKWARD);
        R_MOTOR->run(BACKWARD);
        maxspeed = ReverseSpeed;
        ble.println("Backward");        
      }

      /* .Right */
      if(buttnum == 7){
        L_MOTOR->run(RELEASE);
        R_MOTOR->run(FORWARD);
        maxspeed = TurningSpeed;
        ble.println("Left");
      }

      /* .Left */
      if(buttnum == 8){
        L_MOTOR->run(FORWARD);
        R_MOTOR->run(RELEASE);
        maxspeed = TurningSpeed;
        ble.println("Right");        
      }

      lastPress = millis();

      /* . Gradually speed up the motors*/
      for (int speed=0; speed < maxspeed; speed+=5) {
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 250ms total to speed up
      }
      
  } else {
    
      isMoving = false;
      
      /* .Gradually slow down the motors */
      for (int speed = maxspeed; speed >= 0; speed-=5) {
        L_MOTOR->setSpeed(speed);
        R_MOTOR->setSpeed(speed);
        delay(5); // 50ms total to slow down
      }
      
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(RELEASE);
    }
    
  }
}

/**************************************************************************/
/*!
    @brief  Bluetooth setup
*/
/**************************************************************************/
void BLEsetup(){

  /*Initialize Bluetooth*/
  delay(500);
  Serial.begin(115200);
  Serial.println(F("Race Car"));
  Serial.println(F("-----------------------------------------"));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("Ok! - Bluefruit Bluetooth module found!") );

  /* Perform a factory reset to make sure everything is in a known state */
  if ( FACTORYRESET_ENABLE) {
    Serial.println(F("Performing a factory reset: "));
    if (! ble.factoryReset() ){
         error(F("Couldn't factory reset"));
    }
  }

  //Convert the name change command to a char array
  BROADCAST_CMD.toCharArray(buf, 60);

  /* .Change the broadcast device name here! */
  if(ble.sendCommandCheckOK(buf)){
    Serial.println("name changed");
  }
  delay(250);

  /* .reset to take effect */
  if(ble.sendCommandCheckOK("ATZ")){
    Serial.println("resetting");
  }
  delay(250);

  /* .Confirm name change */
  Serial.println(F("*****************"));
  Serial.println(F("Confirm name change with:"));
  ble.sendCommandCheckOK("AT+GAPDEVNAME");
  Serial.println(F("*****************"));

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  Serial.println(F("*****************"));
  Serial.println("Requesting Bluefruit info:");
  ble.info();
  Serial.println(F("*****************"));

  /* Instructions */
  Serial.println(F("Open Android application to control Race Car"));
  Serial.println();

  /* Only accept debug mode when testing */
  if ( BTVERBOSE_MODE ) {
    ble.verbose(true);  // debug mode on
  } else {
    ble.verbose(false); // debug mode off  
  }
  
  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  /* .Set Bluefruit to DATA mode */
  Serial.println(F("*****************"));
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);
  Serial.println(F("*****************"));
  
}
