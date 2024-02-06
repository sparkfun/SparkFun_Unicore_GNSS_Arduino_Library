/*
  Enable or disable various constellations to be included in position calculation (GPS, GLO, BDS, GAL, QZSS)
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the UM980 into a Base mode configuration using specified coordinates.
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
  Serial.println("SparkFun UM980 Example 8");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates.");
    while (1);
  }
  Serial.println("UM980 detected!");

  //We can enable/disable constellations and check if the command was successful
  if (myGNSS.enableConstellation("GPS") == false)
    Serial.println("GPS Enable Failed");
  else
    Serial.println("GPS Enable Successful");

  //We can batch commands together and check the overall success
  bool response = true;
  if (!myGNSS.enableConstellation("BDS"))
  {
    response = false;
    Serial.println("Failed to enable BDS constellation");
  }
  if (!myGNSS.enableConstellation("GAL"))
  {
    response = false;
    Serial.println("Failed to enable GAL constellation");
  }
  if (!myGNSS.disableConstellation("GLO"))
  {
    response = false;
    Serial.println("Failed to disable GLO constellation");
  }
  if (!myGNSS.enableConstellation("QZSS"))
  {
    response = false;
    Serial.println("Failed to enable QZSS constellation");
  }

  //Save the current configuration into non-volatile memory (NVM)
  if (!myGNSS.saveConfiguration())
  {
    response = false;
    Serial.println("Failed to save the configuration");
  }

  if (response == true)
    Serial.println("Configuration complete!");
  else
    Serial.println("Configuration failed!");
}

void loop()
{
  //Read in NMEA from the UM980
  //  while (SerialGNSS.available())
  //    Serial.write(SerialGNSS.read());
}
