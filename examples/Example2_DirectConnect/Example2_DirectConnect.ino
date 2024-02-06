/*
  Echo all characters to the serial terminal.
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This sketch simply echoes chars coming from the UM980 and sends chars
  to the UM980. This allows a user to directly enter command strings into the UM980
  while still connected to the Arduino. Good for viewing raw output from a given command.

  For example, type CONFIG to see the module's current configuration response.

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
  Serial.println("SparkFun UM980 Example 2");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  //myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  bool response = true;

  //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
  response &= myGNSS.disableOutput();

  Serial.println("All characters now being echoed to UM980");
  Serial.println("Send CONFIG to see the current configuration");
  Serial.println("Be sure both NL & CR is turned on!");
}

void loop()
{
  while(SerialGNSS.available())
    Serial.write(SerialGNSS.read());

  while(Serial.available())
    SerialGNSS.write(Serial.read());
}
