/*
  Enable an RTCM message on various ports, at various rates.
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

int pin_UART_TX = 4;
int pin_UART_RX = 13;

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS
#include <SparkFun_Extensible_Message_Parser.h> //http://librarymanager/All#SparkFun_Extensible_Message_Parser

//----------------------------------------
// Constants
//----------------------------------------

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] =
{
    sempRtcmPreamble
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char * const parserNames[] =
{
    "RTCM parser"
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

//----------------------------------------
// Locals
//----------------------------------------

SEMP_PARSE_STATE *parse;
UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example");

  // Initialize the parser
  parse = sempInitParser(parserTable, parserCount,
                         parserNames, parserNameCount,
                         0, 3000, processMessage, "RTCM_Test");
  if (!parse)
    reportFatalError("Failed to initialize the parser");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART_RX, pin_UART_TX);

  myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates.");
    while (1);
  }
  Serial.println("UM980 detected!");

  myGNSS.disableOutput(); // Disables all messages on this port

  //Configure the port on the UM980 we are currently commuicating with
  myGNSS.setRTCMMessage("RTCM1005", 1); //Message type, 1 report per second.

  //Configure a given port on the UM980 with a given message type
  myGNSS.setRTCMPortMessage("RTCM1074", "COM3", 1); //Message type, COM name, 1 report per second.

  myGNSS.setRTCMMessage("RTCM1124", 0); //Disable given message
  myGNSS.setRTCMPortMessage("RTCM1093", "COM1", 0); //Disable given message on a given port

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  Serial.println("RTCM messages are dumped in HEX if the CRC is correct");
}

void loop()
{
  while (SerialGNSS.available())
    // Update the parser state based on the incoming byte
    sempParseNextByte(parse, SerialGNSS.read());
}

// Output a line of text for the SparkFun Extensible Message Parser
void sempExtPrintLineOfText(const char *string)
{
  Serial.println(string);
}

// Call back from within parser, for end of message
// Process a complete message incoming from parser
void processMessage(SEMP_PARSE_STATE *parse, uint8_t type)
{
  SEMP_SCRATCH_PAD *scratchPad = (SEMP_SCRATCH_PAD *)parse->scratchPad;
  static bool displayOnce = true;

  // Display the raw message
  Serial.println();
  Serial.printf("Valid RTCM message: 0x%04x (%d) bytes\r\n", parse->length, parse->length);
  ex5DumpBuffer(parse->buffer, parse->length);

  // Display the parser state
  if (displayOnce)
  {
    displayOnce = false;
    Serial.println();
    sempPrintParserConfiguration(parse);
  }
}

// Display the contents of a buffer
void ex5DumpBuffer(const uint8_t *buffer, uint16_t length)
{
  int bytes;
  const uint8_t *end;
  int index;
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
    Serial.printf("0x%08lx: ", offset);

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      Serial.printf("   ");

    // Display the data bytes
    for (index = 0; index < bytes; index++)
      Serial.printf("%02x ", buffer[index]);

    // Separate the data bytes from the ASCII
    for (; index < (16 - (offset & 0xf)); index++)
      Serial.printf("   ");
    Serial.printf(" ");

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      Serial.printf(" ");

    // Display the ASCII values
    for (index = 0; index < bytes; index++)
      Serial.printf("%c", ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
    Serial.printf("\r\n");

    // Set the next line of data
    buffer += bytes;
    offset += bytes;
  }
}

// Print the error message every 15 seconds
void reportFatalError(const char *errorMsg)
{
  while (1)
  {
    Serial.print("HALTED: ");
    Serial.print(errorMsg);
    Serial.println();
    sleep(15);
  }
}
