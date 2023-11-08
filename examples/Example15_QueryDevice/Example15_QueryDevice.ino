/*
  Get the current configuration, version, mode, and mask from UM980
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to send the 'CONFIG' command to get a clear text response that the user
  can parse to see which settings are set.
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
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
  myGNSS.disableOutput();

  char response[1000] = {0};
  int responseLength = sizeof(response);

  Serial.println();
  responseLength = sizeof(response);
  myGNSS.sendQuery("MASK", response, &responseLength);
  Serial.printf("Mask response (%d bytes): \r\n%s\r\n", responseLength, response);

  //The config query can be more than 700 bytes
  Serial.println();
  responseLength = sizeof(response);
  myGNSS.sendQuery("CONFIG", response, &responseLength);
  Serial.printf("Config response (%d bytes): \r\n%s\r\n", responseLength, response);

  Serial.println();
  responseLength = sizeof(response);
  myGNSS.sendQuery("MODE", response, &responseLength);
  Serial.printf("Mode response (%d bytes): \r\n%s\r\n", responseLength, response);

  Serial.println();
  responseLength = sizeof(response);
  myGNSS.sendQuery("VERSION", response, &responseLength);
  Serial.printf("Version response (%d bytes): \r\n%s\r\n", responseLength, response);
}

void loop()
{
  if (Serial.available()) ESP.restart();
}
