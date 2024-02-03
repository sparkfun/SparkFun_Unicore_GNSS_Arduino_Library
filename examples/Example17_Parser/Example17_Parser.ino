/*
  This example configures the UM980 inside the Torch.

  NMEA, some RTCM, and BESTNAV (binary) are enabled and output at 1Hz.
*/

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS
#include <SparkFun_Extensible_Message_Parser.h>    //http://librarymanager/All#SparkFun_Extensible_Message_Parser

//----------------------------------------
// Constants
//----------------------------------------

int pin_UART1_TX = 4;
int pin_UART1_RX = 13;

#define COMPILE_CAPTURE_RAW_DATA_STREAM     0
#define COMPILE_PARSER_STATE_DISPLAY        0
#define CAPTURE_BYTE_COUNT                  8192 // 0 means infinite

#define NMEA_PARSER_INDEX           0
#define UNICORE_HASH_PARSER_INDEX   1
#define RTCM_PARSER_INDEX           2
#define UNICORE_BINARY_PARSER_INDEX 3

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] =
{
    sempNmeaPreamble,
    sempUnicoreHashPreamble,
    sempRtcmPreamble,
    sempUnicoreBinaryPreamble,
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char * const parserNames[] =
{
    "NMEA Parser",
    "Unicore Hash (#) Parser",
    "RTCM Parser",
    "Unicore Binary Parser",
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

#define CONVERT_TO_C_BYTES_PER_LINE     8 // Must be a power of 2
#define CONVERT_TO_C_BYTES_PER_MASK     (CONVERT_TO_C_BYTES_PER_LINE - 1)

//----------------------------------------
// Locals
//----------------------------------------

UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

unsigned long lastCheck = 0;

uint32_t dataOffset;                   // Offset in rawDataStream array
int32_t offset;                        // invalidCharacterBuffer offset
uint8_t invalidCharacterBuffer[20000]; // Data that was not parsed
SEMP_PARSE_STATE *parse;               // State of the parsers

//----------------------------------------
// UM980 Initialization Example
//----------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(250);
    Serial.println();
    Serial.println("SparkFun UM980 Example 17");

    //We must start the serial port before using it in the library
    SerialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

    // Enable output from the Unicore GNSS library
//    myGNSS.enableDebugging(); // Print all debug to Serial

    // Enable various debug options in the Unicore GNSS library
//    myGNSS.enablePrintParserTransitions();
//    myGNSS.enablePrintBadChecksums();
//    myGNSS.enablePrintRxMessages();
//    myGNSS.enableRxMessageDump();

    // Initialize the Unicore GNSS
    if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
    {
        Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
        while (true);
    }
    Serial.println("UM980 detected!");

    // Enable low level debugging for the parser in the Unicore GNSS library
//    myGNSS.enableParserDebug();
//    myGNSS.enableParserErrors();
//    myGNSS.printParserConfiguration();

    // Clear saved configurations, satellite ephemerides, position information, and reset baud rate to 115200bps.
    resetUM980();

    bool response = true;

    float outputRate = 1; //0.5 = 2 reports per second.

    char comPort[] = "COM3"; //UM980 UART3 is connected to ESP32 UART1 on the Torch

    //Enable some RTCM, NMEA, and a UM980 binary command called BESTNAVB
    response &= myGNSS.setRTCMPortMessage("RTCM1005", comPort, outputRate);
    response &= myGNSS.setRTCMPortMessage("RTCM1074", comPort, outputRate);

    response &= myGNSS.setNMEAPortMessage("GPGGA", comPort, outputRate);
    response &= myGNSS.setNMEAPortMessage("GPGSA", comPort, outputRate);
    response &= myGNSS.setNMEAPortMessage("GPGST", comPort, outputRate);
    response &= myGNSS.setNMEAPortMessage("GPGSV", comPort, outputRate);
    response &= myGNSS.setNMEAPortMessage("GPRMC", comPort, outputRate);
    response &= myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

    //If any one command fails, it will force response to false
    if (response == false)
    {
        Serial.println("UM980 failed to configure. Freezing...");
        while (true);
    }

    // Display the firmware version
    Serial.printf("%s", myGNSS.getVersionFull());

    //sendCommand() sends the string directly and checks for the OK response
    //Returns true if the OK was detected
    //"BESTNAVB 1" starts the output of BESTNAV binary message at 1Hz on the COM port
    //we are connected to (in this case COM3)
    if (myGNSS.sendCommand("BESTNAVB 1") == true)
        Serial.println("BESTNAVB enabled");
    else
        Serial.println("BESTNAVB error");

    // Initialize the parser
    parse = sempBeginParser(parserTable, parserCount,
                            parserNames, parserNameCount,
                            0, 3000, processMessage, "Example 17 Parser");
    if (!parse)
        reportFatalError("Failed to initialize the parser");

    if (COMPILE_CAPTURE_RAW_DATA_STREAM)
    {
        // Disable parser output
        sempDisableErrorOutput(parse);
        Serial.println("const uint8_t rawDataStream[] =");
        Serial.println("{");
    }
    else
        // Enable debugging for the parser
        sempEnableDebugOutput(parse);

    Serial.println("Mixture of NMEA, RTCM, and UM980 binary now reporting. Have fun!");
    Serial.println("----------------------------------------------------------------");
}

void loop()
{
    const char * startState;
    const char * endState;

    // Read the raw data one byte at a time and update the parser state
    // based on the incoming byte
    while (SerialGNSS.available())
    {
        // Read the byte from the UM980
        uint8_t incoming = SerialGNSS.read();

        // Remember this data byte
        // Ignore CR and LF following NMEA checksum
        if ((parse->type != 0) || ((incoming != '\r') && (incoming != '\n')))
            invalidCharacterBuffer[offset++] = incoming;

        // Parse this byte
        startState = getParseStateName(parse);
        sempParseNextByte(parse, incoming);
        endState = getParseStateName(parse);

        // Build a rawDataStream array
        if (COMPILE_CAPTURE_RAW_DATA_STREAM)
        {
            if ((!CAPTURE_BYTE_COUNT)
                || (dataOffset < CAPTURE_BYTE_COUNT))
                // Add the incoming data byte to the rawDataStream
                convertToC(incoming);
            else if (dataOffset == CAPTURE_BYTE_COUNT)
            {
                // Finish the rawDataStream array
                convertToCComment(true);
                dataOffset += 1;
            }
        }

        // Display the state changes
        else if (COMPILE_PARSER_STATE_DISPLAY)
            Serial.printf("Ex17: 0x%02x (%c), state: (%p) %s --> %s (%p)\r\n",
                          incoming,
                          ((incoming >= ' ') && (incoming < 0x7f)) ? incoming : '.',
                          startState, startState, endState, endState);
    }
}

void resetUM980()
{
    if (myGNSS.factoryReset() == true)
        Serial.println("UM980 now reset to factory defaults");
    else
        Serial.println("Error resetting UM980 to factory defaults");

    Serial.println("Waiting for UM980 to reboot");

    while (1)
    {
        delay(1000); //Wait for device to reboot
        if (myGNSS.isConnected() == true)
            break;
        Serial.println("Device still rebooting");
    }

    Serial.println("UM980 has completed reset");
}

//----------------------------------------
// Support Routines
//----------------------------------------

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

//----------------------------------------
// Parser Support Routines
//----------------------------------------

// Call back from within parser, for end of message
// Process a complete message incoming from parser
void processMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
    SEMP_SCRATCH_PAD *scratchPad = (SEMP_SCRATCH_PAD *)parse->scratchPad;
    char *typeName;

    // Dump the received messages
    if (!COMPILE_CAPTURE_RAW_DATA_STREAM)
    {
        // Display any invalid data
        if (offset > parse->length)
        {
            Serial.println();
            Serial.println("Unknown data");
            dumpBuffer(invalidCharacterBuffer, offset - parse->length);
        }
        offset = 0;

        // Display the raw message
        // The type value is the index into the raw data array
        Serial.println();
        Serial.printf("Valid %s message: 0x%04x (%d) bytes\r\n",
                parserNames[type], parse->length, parse->length);
        dumpBuffer(parse->buffer, parse->length);
    }
}

// Translate the state value into an ASCII state name
const char *getParseStateName(SEMP_PARSE_STATE *parse)
{
    const char *name;

    do
    {
        name = sempNmeaGetStateName(parse);
        if (name)
            break;
        name = sempNmeaGetStateName(parse);
        if (name)
            break;
        name = sempRtcmGetStateName(parse);
        if (name)
            break;
        name = sempUnicoreBinaryGetStateName(parse);
        if (name)
            break;
        name = sempUnicoreHashGetStateName(parse);
        if (name)
            break;
        name = sempGetStateName(parse);
    } while (0);
    return name;
}

//----------------------------------------
// Raw Data Stream Support Routines
//----------------------------------------

// Add a comment to the end of the line with the data offset
void convertToCComment(bool lastLine)
{
    static uint32_t lastOffset;

    uint32_t bytesOnLine;
    bool done;
    uint32_t nextOffset;
    uint32_t startOffset;

    // Determine the various offsets

    done = lastLine;
    nextOffset = dataOffset;
    bytesOnLine = nextOffset - lastOffset;
    lastOffset = dataOffset;
    if (bytesOnLine)
        done = false;
    if (!done)
    {
        // Determine the offset for the unfinished line
        startOffset = nextOffset - bytesOnLine;

        // Add spaces to align the comments
        while (bytesOnLine++ < CONVERT_TO_C_BYTES_PER_LINE)
            Serial.print("      ");

        // Add the end of line comment
        Serial.printf("  // 0x%08x (%d)\r\n", startOffset, startOffset);
    }

    // Finish the data structure
    if (lastLine)
    {
        Serial.print("}; ");

        // Add spaces to align the comments
        for (bytesOnLine = 0; bytesOnLine < CONVERT_TO_C_BYTES_PER_LINE; bytesOnLine++)
            Serial.print("      ");

        // Add the end of line comment
        Serial.printf("  // 0x%08x (%d)\r\n", nextOffset, nextOffset);
    }
}

// Add a data byte to the rawDataStream array, print the value in hex
// Add an end of line comment with the offset from the beginning of the
// buffer
void convertToC(uint8_t data)
{
    // Indent the line
    if (!(dataOffset & CONVERT_TO_C_BYTES_PER_MASK))
        Serial.print("   ");

    // Add the data byte
    Serial.printf(" 0x%02x,", data);
    dataOffset += 1;

    // Add the comment at the end of the line
    if (!(dataOffset & CONVERT_TO_C_BYTES_PER_MASK))
        convertToCComment(false);
}
