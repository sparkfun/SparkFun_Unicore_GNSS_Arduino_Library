/*
  Setting UM980 as Base Station using averaged coordinates (60 seconds)
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the UM980 into a Base mode configuration using the average of positional
  fixes obtained over a 60 second period. We also turn on RTCM messages.
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

  myGNSS.disableOutput(); //Turn off all messages on this port during configuration

  //Set the output rate of NMEA messages. Users can set both of the message type and update rate.
  //Configure the port we are currently commuicating with on the UM980 (the ESP32 is connected to COM2)
  //1, 0.5, 0.2, 0.1, 0.05 corresponds to 1Hz, 2Hz, 5Hz, 10Hz, 20Hz respectively.
  //1, 2, 5 corresponds to 1Hz, 0.5Hz, 0.2Hz respectively.
  myGNSS.setNMEAMessage("GPGGA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGST", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPRMC", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSV", 1); //Message type, 1 report per second.

  //When the coordinates of base station are unknown, users can set the receiver to
  //automatically positioning for a period of time and get the average value as the
  //coordinates of the base station. This is similar to the u-blox 'Survey In' method.
  myGNSS.setModeBaseAverage(); //Average for 60 seconds (default)
  //myGNSS.setModeBaseAverage(120); //Average for 120 seconds.

  //Set the output rate of RTCM correction messages.
  //Unicore supports a *large* number of RTCM messages. We recommend the following for most applications:
  //RTCM1006, 1074, 1084, 1094, 1124, 1033
  myGNSS.setRTCMMessage("RTCM1006", 10);
  myGNSS.setRTCMMessage("RTCM1074", 1);
  myGNSS.setRTCMMessage("RTCM1084", 1);
  myGNSS.setRTCMMessage("RTCM1094", 1);
  myGNSS.setRTCMMessage("RTCM1124", 1);
  myGNSS.setRTCMMessage("RTCM1033", 10);

  myGNSS.setRTCMMessage("RTCM1033", 0); //Disable a specific message

  myGNSS.setRTCMMessage("RTCM1074", "COM1", 1); //Enable message on a specific port

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)
}

void loop()
{
  //Read in NMEA from the UM980
  //RTCM is binary and will appear as random characters.
  while (SerialGNSS.available())
    Serial.write(SerialGNSS.read());
}
