/*
  Reading Position and Time via Serial
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a UM980 GNSS module for its position and time data.
  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Note: Lat/Lon are doubles and the UM980 outputs 11 digits after the decimal.

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

unsigned long lastCheck = 0;

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
}

void loop()
{
  myGNSS.update(); //Regularly call to parse any new data

  if (millis() - lastCheck > 1000)
  {
    lastCheck = millis();

    //The get methods are updated whenever new data is parsed with the update() call.
    //By default, this data is updated once per second.

    Serial.print("Lat/Long/Alt: ");
    Serial.print(myGNSS.getLatitude(), 11); //Accurate 11 decimal places
    Serial.print("/");
    Serial.print(myGNSS.getLongitude(), 11);
    Serial.print("/");
    Serial.print(myGNSS.getAltitude(), 4); //Accurate to 4 decimal places
    Serial.println();

    Serial.print("Horizontal Speed: ");
    Serial.print(myGNSS.getHorizontalSpeed());
    Serial.print("m/s Vertical Speed: ");
    Serial.print(myGNSS.getVerticalSpeed());
    Serial.print("m/s Direction from North: ");
    Serial.print(myGNSS.getTrackGround());
    Serial.print("(degrees)");
    Serial.println();

    Serial.print("Date (yyyy/mm/dd): ");
    Serial.print(myGNSS.getYear());
    Serial.print("/");
    if (myGNSS.getMonth() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getMonth());
    Serial.print("/");
    if (myGNSS.getDay() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getDay());

    Serial.print(" Time (hh:mm:dd): ");
    if (myGNSS.getHour() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    if (myGNSS.getMinute() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    if (myGNSS.getSecond() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getSecond());
    Serial.print(".");
    if (myGNSS.getMillisecond() < 100)
      Serial.print("0");
    if (myGNSS.getMillisecond() < 10)
      Serial.print("0");
    Serial.print(myGNSS.getMillisecond());
    Serial.println();

    Serial.print("Satellites in view: ");
    Serial.print(myGNSS.getSIV());
    Serial.println();

    Serial.println();
  }
}
