/*
  Setting UM980 as Base Station using provided Geodetic or ECEF coordinates
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the UM980 into a Base mode configuration using specified coordinates.
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
#include <SparkFun_Extensible_Message_Parser.h>    //http://librarymanager/All#SparkFun_Extensible_Message_Parser

#define NMEA_PARSER_INDEX           0
#define RTCM_PARSER_INDEX           1

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] =
{
    sempNmeaPreamble,
    sempRtcmPreamble,
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char * const parserNames[] =
{
    "NMEA Parser",
    "RTCM Parser",
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

SEMP_PARSE_STATE *parse;               // State of the parsers

UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example 7");

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

  //Set the output rate of NMEA messages. Users can set both the message type and update rate.
  //Configure the port we are currently communicating with on the UM980 (the ESP32 is connected to COM2)
  //1, 0.5, 0.2, 0.1, 0.05 corresponds to 1Hz, 2Hz, 5Hz, 10Hz, 20Hz respectively.
  //1, 2, 5 corresponds to 1Hz, 0.5Hz, 0.2Hz respectively.
  myGNSS.setNMEAMessage("GPGGA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSA", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGST", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPRMC", 1); //Message type, 1 report per second.
  myGNSS.setNMEAMessage("GPGSV", 1); //Message type, 1 report per second.

  //When the coordinates of the base station are known, users can set the position
  //using Geodetic or ECEF coordinates.
  myGNSS.setModeBaseGeodetic(40.09029479, -105.18505761, 1560.089); //SparkFun HQ
  //myGNSS.setModeBaseECEF(-1280206.568, -4716804.403, 4086665.484); //SparkFun HQ

  //Set the output rate of RTCM correction messages.
  //Unicore supports a *large* number of RTCM messages. We recommend the following for most applications:
  //RTCM1006, 1074, 1084, 1094, 1124, 1033
  myGNSS.setRTCMMessage("RTCM1006", 10);
  myGNSS.setRTCMMessage("RTCM1074", 1);
  myGNSS.setRTCMMessage("RTCM1084", 1);
  myGNSS.setRTCMMessage("RTCM1094", 1);
  myGNSS.setRTCMMessage("RTCM1124", 1);
  myGNSS.setRTCMMessage("RTCM1033", 10);

  //myGNSS.setRTCMMessage("RTCM1033", 0); //Example of how to disable a specific message

  //myGNSS.setRTCMPortMessage("RTCM1074", "COM1", 1); //Example of how to enable a message on a specific port

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  // Initialize the parser
  parse = sempBeginParser(parserTable, parserCount,
                          parserNames, parserNameCount,
                          0, 3000, processMessage, "Example 17 Parser");
  if (!parse)
    while (1)
    {
      Serial.println("HALTED: Failed to initialize the parser!");
      sleep(15);
    }

  Serial.println("Output will be a mix of NMEA and binary RTCM");
}

void loop()
{
  // Read the raw data one byte at a time and update the parser state
  // based on the incoming byte
  while (SerialGNSS.available())
  {
    // Read the byte from the UM980
    uint8_t incoming = SerialGNSS.read();

    // Parse this byte
    sempParseNextByte(parse, incoming);
  }
}

// Call back from within parser, for end of message
// Process a complete message incoming from parser
void processMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
  SEMP_SCRATCH_PAD *scratchPad = (SEMP_SCRATCH_PAD *)parse->scratchPad;
  char *typeName;

  // Display the raw message
  // The type value is the index into the raw data array
  Serial.println();
  Serial.printf("Valid %s message: 0x%04x (%d) bytes\r\n",
                parserNames[type], parse->length, parse->length);
  dumpBuffer(parse->buffer, parse->length);
}

// Display the contents of a buffer
void dumpBuffer(const uint8_t *buffer, uint16_t length)
{
  int bytes;
  const uint8_t *end;
  int index;
  char line[128];
  uint16_t offset;

  end = &buffer[length];
  offset = 0;
  while (buffer < end)
  {
    // Determine the number of bytes to display on the line
    bytes = end - buffer;
    if (bytes > (16 - (offset & 0xf)))
      bytes = 16 - (offset & 0xf);

    // Display the offset
    sprintf(line, "0x%08lx: ", offset);

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      sprintf(&line[strlen(line)], "   ");

    // Display the data bytes
    for (index = 0; index < bytes; index++)
      sprintf(&line[strlen(line)], "%02x ", buffer[index]);

    // Separate the data bytes from the ASCII
    for (; index < (16 - (offset & 0xf)); index++)
      sprintf(&line[strlen(line)], "   ");
    sprintf(&line[strlen(line)], " ");

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      sprintf(&line[strlen(line)], " ");

    // Display the ASCII values
    for (index = 0; index < bytes; index++)
      sprintf(&line[strlen(line)], "%c", ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
    Serial.println(line);

    // Set the next line of data
    buffer += bytes;
    offset += bytes;
  }
}
