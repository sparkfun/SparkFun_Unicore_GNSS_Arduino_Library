/*
  Enable Galileo High Accuracy Service: https://gssc.esa.int/navipedia/index.php/Galileo_High_Accuracy_Service_(HAS)
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 9th, 2024
  License: MIT. Please see LICENSE.md for more information.

  The UM980 is capable of receiving the new Galileo E6 signal and obtain a sub 0.2m precision 
  location fix. This example shows how to enable E6 and HAS

  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Triband GNSS RTK Breakout - UM980 (GPS-23286) https://www.sparkfun.com/products/23286

  Hardware Connections:
  Connect RX2 (green wire) of the UM980 to pin 4 on the ESP32
  Connect TX2 (orange wire) of the UM980 to pin 13 on the ESP32
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

unsigned long startTime = 0;
unsigned long convergingStartTime = 0;
unsigned long timeToConvergence = 0;

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example 19");

  //The CONFIG response can be ~500 bytes over runing the ESP32 RX buffer of 256 bytes
  //Increase the size of the RX buffer
  SerialGNSS.setRxBufferSize(1024);

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  //myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  //E6 reception requires version 11833 or greater
  int um980Version = String(myGNSS.getVersion()).toInt(); //Convert the string response to a value

  Serial.print("UM980 firmware version: v");
  Serial.println(um980Version);

  if (um980Version < 11833)
  {
    Serial.println("E6 requires 11833 or newer. Please update the firmware on your UM980. Freezing...");
    while (true);
  }
  else
    Serial.println("Firmware is E6-PPP compatible.");

  if (myGNSS.isConfigurationPresent("CONFIG SIGNALGROUP 2") == false)
  {
    if (myGNSS.sendCommand("CONFIG SIGNALGROUP 2") == false)
      Serial.println("Signal group 2 command failed");
    else
    {
      Serial.println("Enabling signal group 2 causes the UM980 to reboot. This can take a few seconds.");

      while (1)
      {
        delay(1000); //Wait for device to reboot
        if (myGNSS.isConnected() == true) break;
        else Serial.println("Device still rebooting");
      }

      Serial.println("UM980 has completed reboot.");
    }
  }
  else
    Serial.println("Signal group 2 already enabled");

  if (myGNSS.isConfigurationPresent("CONFIG PPP ENABLE E6-HAS") == false)
  {
    if (myGNSS.sendCommand("CONFIG PPP ENABLE E6-HAS") == true)
      Serial.println("E6 service enabled");
    else
      Serial.println("E6 config error");
  }
  else
    Serial.println("E6 service already enabled");

  startTime = millis();

  Serial.println("E6 PPP should now be enabled. Sit back and watch the LLA deviations decrease below 0.2m!");
  Serial.println("r) Reset ESP");
  Serial.println("R) Factory reset UM980");
}

void loop()
{
  if (Serial.available())
  {
    byte incoming = Serial.read();
    if (incoming == 'r')
      ESP.restart();
    else if (incoming == 'R')
      um980Reset();
  }


  myGNSS.update(); //Regularly call to parse any new data

  if (millis() - lastCheck > 1000)
  {
    lastCheck = millis();
    printUpdate();
  }
}

void printUpdate()
{
  Serial.print("Lat/Long/Alt: ");
  Serial.print(myGNSS.getLatitude(), 11); //Accurate 11 decimal places
  Serial.print("/");
  Serial.print(myGNSS.getLongitude(), 11);
  Serial.print("/");
  Serial.print(myGNSS.getAltitude(), 4); //Accurate to 4 decimal places
  Serial.println();

  Serial.print("Deviation of Lat/Long/Alt (m): ");
  Serial.print(myGNSS.getLatitudeDeviation(), 4);
  Serial.print("/");
  Serial.print(myGNSS.getLongitudeDeviation(), 4);
  Serial.print("/");
  Serial.println(myGNSS.getAltitudeDeviation(), 4);

  Serial.print("Satellites in view: ");
  Serial.print(myGNSS.getSIV());
  Serial.println();

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
  else if (positionType == 68)
  {
    Serial.print("PPP solution converging");

    if (convergingStartTime == 0) convergingStartTime = millis();
    if (convergingStartTime > 0)
    {
      Serial.print(" - Seconds in converging phase: ");
      Serial.print((millis() - (convergingStartTime - startTime)) / 1000.0, 0);
      Serial.print("s");
    }
  }
  else if (positionType == 69)
  {
    Serial.print("Precise Point Positioning");

    if (timeToConvergence == 0) timeToConvergence = millis();
    if (timeToConvergence > 0)
    {
      Serial.print(" - Seconds in converging phase: ");
      Serial.print((millis() - (convergingStartTime - startTime)) / 1000.0, 0);
      Serial.print("s");

      Serial.print(" - Total time to convergence: ");
      Serial.print((timeToConvergence - startTime) / 1000.0, 0);
      Serial.print("s");
    }
  }
  else Serial.print("Unknown");
  Serial.println();

  Serial.println();
}

void um980Reset()
{
  while (Serial.available()) Serial.read(); //Clear RX buffer
  Serial.println("Press any key to factory reset the UM980");
  while (Serial.available() == 0) delay(1); //Wait for user to press a button

  // Clear saved configurations, satellite ephemerides, position information, and reset baud rate to 115200bps.
  if (myGNSS.factoryReset() == true)
    Serial.println("UM980 now reset to factory defaults");
  else
    Serial.println("Error resetting UM980 to factory defaults");

  Serial.println("Waiting for UM980 to reboot");

  while (1)
  {
    delay(1000); //Wait for device to reboot
    if (myGNSS.isConnected() == true) break;
    else Serial.println("Device still rebooting");
  }

  Serial.println("UM980 has completed reset");
}