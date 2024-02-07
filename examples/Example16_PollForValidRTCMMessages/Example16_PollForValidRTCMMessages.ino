/*
  Detect which RTCM messages are supported - currently 55!
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  The documentation for the UM980 does not explicitly state which RTCM messages are supported
  but by sending the enable command to the module we can insinuate which are supported.
  This sketch sends the 'RTCMxxxx' command and looks for an OK.
  The messages that successfully receive an OK will be reported at the completion of the sketch.
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

  68 RTCM messages supported: 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1009, 1010, 1011, 1012, 1013, 1019, 1020,
  1033, 1042, 1044, 1045, 1046, 1071, 1072, 1073, 1074, 1075, 1076, 1077, 1081, 1082, 1083, 1084, 1085, 1086, 1087,
  1091, 1092, 1093, 1094, 1095, 1096, 1097, 1101, 1102, 1103, 1104, 1105, 1106, 1107, 1111, 1112, 1113, 1114, 1115,
  1116, 1117, 1121, 1122, 1123, 1124, 1125, 1126, 1127, 1131, 1132, 1133, 1134, 1135, 1136, 1137,

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
  Serial.println("SparkFun UM980 Example 16");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  // myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
  myGNSS.disableOutput();

  uint16_t supportedMessages[200] = {0};
  int supportedCount = 0;

  int delayBetweenCommands = 10;
  char comPort[] = "COM3"; //Enable on a different port than the one we are using

  Serial.println("Scanning 1000 to 1300");
  for (int x = 1000 ; x <= 1300 ; x++)
  {
    char myTest[100] = {0};
    sprintf(myTest, "RTCM%04d", x);
    if(myGNSS.setNMEAPortMessage(myTest, comPort, 1) == true)
      supportedMessages[supportedCount++] = x;
    delay(delayBetweenCommands);
  }

  if(supportedCount > 0)
  {
    Serial.println();
    Serial.print(supportedCount);
    Serial.print(" RTCM messages supported: ");
    for(int x = 0 ; x < supportedCount ; x++)
    {
      Serial.print(supportedMessages[x]);
      Serial.print(", ");
    }
    Serial.println();
  }
}

void loop()
{
  if(Serial.available()) ESP.restart();
}
