/*
  Reading UM980 Statistics including ECEF
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a UM980 GNSS module for signal quality and fix type.
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

    //This is the polling method and requires a slight delay (around 135ms)
    //while the device responds to the request

    Serial.print("Lat/Long/Alt: ");
    Serial.print(myGNSS.getLatitude(), 11);
    Serial.print("/");
    Serial.print(myGNSS.getLongitude(), 11);
    Serial.print("/");
    Serial.println(myGNSS.getAltitude(), 4);

    Serial.print("Deviation of Lat/Long/Alt (m): ");
    Serial.print(myGNSS.getLatitudeDeviation(), 4);
    Serial.print("/");
    Serial.print(myGNSS.getLongitudeDeviation(), 4);
    Serial.print("/");
    Serial.println(myGNSS.getAltitudeDeviation(), 4);

    Serial.print("ECEF X/Y/Z (m): ");
    Serial.print(myGNSS.getEcefX(), 4);
    Serial.print("/");
    Serial.print(myGNSS.getEcefY(), 4);
    Serial.print("/");
    Serial.println(myGNSS.getEcefZ(), 4);

    Serial.print("Deviation of ECEF X/Y/Z (m): ");
    Serial.print(myGNSS.getEcefXDeviation(), 4);
    Serial.print("/");
    Serial.print(myGNSS.getEcefYDeviation(), 4);
    Serial.print("/");
    Serial.println(myGNSS.getEcefZDeviation(), 4);

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

    int timeStatus = myGNSS.getTimeStatus();
    Serial.print(" Time status: ");
    Serial.print(timeStatus);
    Serial.print(" - ");
    if (timeStatus == 0) Serial.print("Valid");
    else if (timeStatus == 3) Serial.print("Invalid!");
    else Serial.print("Unknown");
    Serial.println();

    Serial.print("Satellites tracked: ");
    Serial.println(myGNSS.getSatellitesTracked());
    Serial.print("Satellites used: ");
    Serial.println(myGNSS.getSatellitesUsed());

    int positionType = myGNSS.getPositionType();
    Serial.print("Position Type: ");
    Serial.print(positionType);
    Serial.print(" - ");
    if (positionType == 0) Serial.print("No solution");
    else if (positionType == 8) Serial.print("Velocity computed using instantaneous Doppler");
    else if (positionType == 16) Serial.print("Single point positioning");
    else if (positionType == 17) Serial.print("Pseudorange differential solution");
    else if (positionType == 18) Serial.print("SBAS positioning");
    else if (positionType == 32) Serial.print("L1 float solution");
    else if (positionType == 33) Serial.print("Ionosphere-free float solution");
    else if (positionType == 34) Serial.print("Narrow-lane float solution");
    else if (positionType == 48) Serial.print("L1 fixed solution");
    else if (positionType == 49) Serial.print("Wide-lane fixed solution");
    else if (positionType == 50) Serial.print("Narrow-lane fixed solution");
    else Serial.print("Unknown");
    Serial.println();

    int solutionStatus = myGNSS.getSolutionStatus();
    Serial.print("Solution Status: ");
    Serial.print(solutionStatus);
    Serial.print(" - ");
    if (solutionStatus == 0) Serial.print("Solution computed");
    else if (solutionStatus == 1) Serial.print("Insufficient observation");
    else if (solutionStatus == 2) Serial.print("No convergence, invalid solution");
    else if (solutionStatus == 4) Serial.print("Covariance matrix trace exceeds maximum");
    else Serial.print("Unknown");
    Serial.println();

    int rtkSolution = myGNSS.getRTKSolution();
    Serial.print("RTK Solution: ");
    Serial.print(rtkSolution);
    Serial.print(" - ");
    if (rtkSolution == 0) Serial.print("Unchecked");
    else if (rtkSolution == 1) Serial.print("Checked");
    else Serial.print("Unknown");
    Serial.println();

    int pseudorangeCorrection = myGNSS.getPseudorangeCorrection();
    Serial.print("Pseudorange Correction: ");
    Serial.print(pseudorangeCorrection);
    Serial.print(" - ");
    if (pseudorangeCorrection == 0) Serial.print("Unknown");
    else if (pseudorangeCorrection == 0x001) Serial.print("Klobuchar broadcast ephemeris correction");
    else if (pseudorangeCorrection == 0x010) Serial.print("SBAS ionospheric grid correction");
    else if (pseudorangeCorrection == 0x011) Serial.print("Multi-frequency correction");
    else if (pseudorangeCorrection == 0x100) Serial.print("Pseudorange differential correction");
    else Serial.print("Unknown");
    Serial.println();

    Serial.println();
  }
}