/*
  Checking if a config item is set
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 9th, 2024
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to query a UM980 GNSS module to see if a particular string
  shows up in the response to CONFIG.

  Below is an example response from the CONFIG command:

  $command,config,response: OK*54
  $CONFIG,ANTENNA,CONFIG ANTENNA POWERON*7A
  $CONFIG,NMEAVERSION,CONFIG NMEAVERSION V410*47
  $CONFIG,RTK,CONFIG RTK TIMEOUT 120*6C
  $CONFIG,RTK,CONFIG RTK RELIABILITY 3 1*76
  $CONFIG,PPP,CONFIG PPP TIMEOUT 120*6C
  $CONFIG,DGPS,CONFIG DGPS TIMEOUT 300*6C
  $CONFIG,RTCMB1CB2A,CONFIG RTCMB1CB2A ENABLE*25
  $CONFIG,ANTENNADELTAHEN,CONFIG ANTENNADELTAHEN 0.0000 0.0000 0.0000*3A
  $CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0*6E
  $CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP 2*16
  $CONFIG,ANTIJAM,CONFIG ANTIJAM AUTO*2B
  $CONFIG,AGNSS,CONFIG AGNSS DISABLE*70
  $CONFIG,BASEOBSFILTER,CONFIG BASEOBSFILTER DISABLE*70
  $CONFIG,COM1,CONFIG COM1 115200*23
  $CONFIG,COM2,CONFIG COM2 115200*23
  $CONFIG,COM3,CONFIG COM3 115200*23
  $CONFIG,BASEANTENNAMODEL,CONFIG BASEANTENNAMODEL "ADVNULLANTENNA NULL" "1234" 123 NO*1D

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

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example 18");

  //The CONFIG response can be ~500 bytes overrunning the ESP32 RX buffer of 256 bytes
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

  //The library can't identify specific settings but we can see if a specific setting
  //is present in the response to the CONFIG command
  //See example response above

  if (myGNSS.isConfigurationPresent("COM3 115200") == true)
    Serial.println("COM3 set to 115200");
  else
    Serial.println("COM3 NOT set to 115200");

  if (myGNSS.isConfigurationPresent("CONFIG PPP ENABLE E6-HAS") == true)
    Serial.println("E6 Enabled");
  else
    Serial.println("E6 not enabled");

  if (myGNSS.isConfigurationPresent("CONFIG SIGNALGROUP 2") == true)
    Serial.println("Signal group 2 enabled");
  else
    Serial.println("Signal group 2 not enabled");
}

void loop()
{

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
