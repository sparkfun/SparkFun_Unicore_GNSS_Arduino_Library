/*
  Enable NMEA out COM1 (USB-C) for viewing in u-center (u-blox's software) or UPrecise (Unicore's software)
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This sketch turns on all the major NMEA setences at 2Hz and prints the
  incoming serial out the Serial port. This is useful for viewing the GNSS
  data in a program like u-center.
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

  bool response = true;

  //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
  response &= myGNSS.disableOutput();

  //Enable GPS L1/L2/L5 signals
  if(myGNSS.sendCommand("UNMASK GPS") == true) Serial.println("GPS enabled");
  else Serial.println("GPS unmask error");

  if(myGNSS.sendCommand("UNMASK L1") == true) Serial.println("GPS L1 enabled");
  else Serial.println("GPS unmask L1 error");

  if(myGNSS.sendCommand("UNMASK L2") == true) Serial.println("GPS L2 enabled");
  else Serial.println("GPS unmask L2 error");

  if(myGNSS.sendCommand("UNMASK L5") == true) Serial.println("GPS L5 enabled");
  else Serial.println("GPS unmask L5 error");

  float outputRate = 0.5; //0.5 = 2 reports per second.

  char comPort[] = "COM1";
  response &= myGNSS.setNMEAPortMessage("GPGGA", comPort, outputRate);
  response &= myGNSS.setNMEAPortMessage("GPGSA", comPort, outputRate);
  response &= myGNSS.setNMEAPortMessage("GPGST", comPort, outputRate);
  response &= myGNSS.setNMEAPortMessage("GPGSV", comPort, outputRate);
  response &= myGNSS.setNMEAPortMessage("GPRMC", comPort, outputRate);
  response &= myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  //If any one command failed, it will force response to false
  if(response == false)
  {
    Serial.println("UM980 failed to configure. Freezing...");
    while(true);
  }

  Serial.println("NMEA now reporting on USB-C serial port");
}

void loop()
{
  while(SerialGNSS.available())
    Serial.write(SerialGNSS.read());
}
