/*
  Enable NMEA messages on different ports on the UM980.
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to enable various NMEA sentences, at different rates, on different ports.
  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  This example pipes all NMEA sentences to the UART1 (the USB C port) and UART2 (connected to the microcontroller).

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
  Serial.println("SparkFun UM980 Example");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  //myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  myGNSS.disableOutput(); // Disables all messages on this port

  //Set the output rate of NMEA messages. Users can set both the message type and update rate.
  //1, 0.5, 0.2, 0.1, 0.05 corresponds to 1Hz, 2Hz, 5Hz, 10Hz, 20Hz respectively.
  //1, 2, 5 corresponds to 1Hz, 0.5Hz, 0.2Hz respectively.
  //Configure the port we are currently communicating with on the UM980 (the ESP32 is connected to COM2)
  myGNSS.setNMEAMessage("GPGGA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGST", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPRMC", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSV", 1); //Message type, 1 report per second.

  //Configure a specific port
  float outputRate = 0.2; //0.2 = 5 reports per second.
  char comName[] = "COM1"; //COM1, COM2, and COM3 are valid
  myGNSS.setNMEAPortMessage("GPGGA", comName, outputRate); //Message type, COM port, output rate.
  myGNSS.setNMEAPortMessage("GPGSA", comName, outputRate);
  myGNSS.setNMEAPortMessage("GPGST", comName, outputRate);
  myGNSS.setNMEAPortMessage("GPRMC", comName, outputRate);
  myGNSS.setNMEAPortMessage("GPGSV", comName, 1); //Limit GSV to 1Hz

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  Serial.println("Wait for UM980 to get fix and output NMEA...");
}

void loop()
{
  //Read in NMEA from the UM980
  while (SerialGNSS.available())
    Serial.write(SerialGNSS.read());
}