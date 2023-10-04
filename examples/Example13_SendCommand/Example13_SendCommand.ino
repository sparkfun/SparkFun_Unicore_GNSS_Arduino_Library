/*
  Sending direct commands to the UM980 GNSS receiver
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  While the SparkFun UM980 Arduino library covers most of the features in the UM980, there
  may be a special command that is needed but not supported. This sketch shows how to send commands direct.
  
  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Triband GNSS RTK Breakout - UM980 (GPS-23286) https://www.sparkfun.com/products/23286

  Hardware Connections:
  Connect RX2 of the UM980 to pin 4 on the ESP32
  Connect TX2 of the UM980 to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a dual or triband GNSS antenna: https://www.sparkfun.com/products/21801
*/

int pin_UART1_TX = 4;
int pin_UART1_RX = 13;

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS

UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("UM980 comm over ESP UART1");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates.");
    while (1);
  }
  Serial.println("UM980 detected!");

  //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
  myGNSS.disableOutput();

  //sendCommand() sends the string directly and checks for the OK response
  //Returns true if the OK was detected
  if(myGNSS.sendCommand("UNMASK GPS") == true) Serial.println("GPS enabled");
  else Serial.println("GPS unmask error");

  myGNSS.sendCommand("MASK 10 GPS"); //Set the elevation mask angle as 10 degrees for GPS
}

void loop()
{

}
