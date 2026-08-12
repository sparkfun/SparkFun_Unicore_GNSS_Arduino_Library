/*
  Set RTCM 1033 Antenna Description
  By: Paul Clark
  SparkFun Electronics
  Date: August 12th, 2026
  License: MIT. Please see LICENSE.md for more information.

  When operating as a Base, the UM980 can generate RTCM 1033 messages which contain
  the Antenna Description. These default to:
  Antenna description: ADVNULLANTENNA
  Antenna serial number: a0001
  Antenna Setup ID: 0

  This example shows how to change the antenna description.

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

int pin_UART1_TX = 17; // ESP32 Thing Plus C
int pin_UART1_RX = 16;

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS

UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

unsigned long lastCheck = 0;

unsigned long startTime = 0;
unsigned long convergingStartTime = 0;
unsigned long timeToConvergence = 0;

// Custom Antenna Description
//const char antennaDescription[] = "CONFIG BASEANTENNAMODEL \"ADVNULLANTENNA\" \"Unknown\" 123 NO";
// Response is:
// $command,CONFIG BASEANTENNAMODEL "ADVNULLANTENNA" "Unknown" 123 NO,response: OK*1F\r\n


// GNSS Defaults
const char antennaDescription[] = "CONFIG BASEANTENNAMODEL \"ADVNULLANTENNA\" \"a0001\" 0 NO";
// Response is:
// $command,CONFIG BASEANTENNAMODEL "ADVNULLANTENNA" "a0001" 0 NO,response: OK*37\r\n


// Longest possible configuration. Description and serial number can be up to 31 chars
//const char antennaDescription[] = "CONFIG BASEANTENNAMODEL 0123456789012345678901234567890 0123456789012345678901234567890 123 USER";
// Response is:
// $command,CONFIG BASEANTENNAMODEL 0123456789012345678901234567890 0123456789012345678901234567890 123 USER,response: OK*50\r\n

// If the description or serial number contain spaces, they need to be included in quotes
// BUT the quotes are then included in the UM980's 31 character length check!
// Also, if we send "01234567890123456789012345678" the CONFIG response only contains "01234567890123456789012345678
// The trailing quote gets dropped... (This is with firmware 17548)
// So, this is the longest antenna description we can send which includes quotes:
//const char antennaDescription[] = "CONFIG BASEANTENNAMODEL \"0123456789012345678901234567\" \"0123456789012345678901234567\" 123 USER";
// Response is:
// $command,CONFIG BASEANTENNAMODEL "0123456789012345678901234567" "0123456789012345678901234567" 123 USER,response: OK*50\r\n


void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example 22");

  //The CONFIG response can be ~500 bytes over runing the ESP32 RX buffer of 256 bytes
  //Increase the size of the RX buffer
  SerialGNSS.setRxBufferSize(1024);

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  //myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS, "SFE_Unicore_GNSS_Library", output) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  int um980Version = String(myGNSS.getVersion()).toInt(); //Convert the string response to a value

  Serial.print("UM980 firmware version: v");
  Serial.println(um980Version);

  if (myGNSS.isConfigurationPresent("CONFIG BASEANTENNAMODEL") == true)
    Serial.println("BASEANTENNAMODEL has been set previously");

  //myGNSS.enablePrintParserTransitions();
  //myGNSS.enablePrintRxMessages();
  //myGNSS.enableRxMessageDump();
  //myGNSS.enablePrintBadChecksums();

  if (myGNSS.isConfigurationPresent(antennaDescription) == true)
    Serial.println("BASEANTENNAMODEL already configured");

  while (myGNSS.sendCommand(antennaDescription) == false)
    Serial.println("BASEANTENNAMODEL error. Retrying...");

  Serial.println("BASEANTENNAMODEL configured");

  //myGNSS.disablePrintBadChecksums();
  //myGNSS.disableRxMessageDump();
  //myGNSS.disablePrintRxMessages();
  //myGNSS.disablePrintParserTransitions();

  startTime = millis();

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

//----------------------------------------
// Output a buffer of data
//
// Inputs:
//   buffer: Address of a buffer of data to output
//   length: Number of bytes of data to output
//----------------------------------------
void output(uint8_t * buffer, size_t length)
{
    size_t bytesWritten;

    if (Serial)
    {
        while (length)
        {
            // Wait until space is available in the FIFO
            while (Serial.availableForWrite() == 0);

            // Output the character
            bytesWritten = Serial.write(buffer, length);
            buffer += bytesWritten;
            length -= bytesWritten;
        }
    }
}
