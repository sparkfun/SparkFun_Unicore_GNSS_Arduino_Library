/*
  This is a library to control Unicore GNSS receivers, with
  a focus on the UM980 Triband receiver. Other receivers in the
  same family should work: UM982, UM960, UM960L, etc.

  https://github.com/sparkfun/SparkFun_Unicore_GNSS_Arduino_Library
  Best used with the UM980 Breakout: https://www.sparkfun.com/products/23286

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include "SparkFun_Unicore_GNSS_Arduino_Library.h"
#include "Arduino.h"

//----------------------------------------
// Constants
//----------------------------------------

// parserTable index values
#define UM980_NMEA_PARSER_INDEX 0
#define UM980_UNICORE_HASH_PARSER_INDEX 1
#define UM980_RTCM_PARSER_INDEX 2
#define UM980_UNICORE_BINARY_PARSER_INDEX 3

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] = {
    sempNmeaPreamble,
    sempUnicoreHashPreamble,
    sempRtcmPreamble,
    sempUnicoreBinaryPreamble,
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char *const parserNames[] = {
    "UN980 NMEA Parser",
    "UM980 Unicore Hash (#) Parser",
    "UM980 RTCM Parser",
    "UM980 Unicore Binary Parser",
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

// Account for the largest message
#define BUFFER_LENGTH 3000

//----------------------------------------
// Globals
//----------------------------------------

UM980 *ptrUM980 = nullptr; // Global pointer for external parser access into library class

//----------------------------------------
// Parser support routines
//----------------------------------------

// Enable the display of bad checksum messages from the parser
void UM980::enablePrintBadChecksums()
{
    _printBadChecksum = true;
}

// Disable the display of bad checksum messages from the parser
void UM980::disablePrintBadChecksums()
{
    _printBadChecksum = false;
}

// Alternate checksum for NMEA parser needed during setup
bool badNmeaChecksum(SEMP_PARSE_STATE *parse)
{
    int alternateChecksum;
    bool badChecksum;
    int checksum;

    // Not a NMEA parser, no correction is possible
    if (parse->type >= UM980_RTCM_PARSER_INDEX)
        return false;

    // Older UM980 firmware during setup is improperly adding the '$'
    // into the checksum calculation.  Convert the received checksum
    // characters into binary.
    checksum = sempAsciiToNibble(parse->buffer[parse->length - 1]);
    checksum |= sempAsciiToNibble(parse->buffer[parse->length - 2]) << 4;

    // Determine if the checksum also includes the '$' or '#'
    alternateChecksum = parse->crc ^ (parse->type ? '#' : '$');
    badChecksum = (alternateChecksum != checksum);

    // Display bad checksums
    if ((!badChecksum) && ptrUM980->_printBadChecksum)
    {
        ptrUM980->debugPrintf("Unicore Lib: Message improperly includes %c in checksum", parse->buffer[0]);
        ptrUM980->dumpBuffer(parse->buffer, parse->length);
    }
    return badChecksum;
}

// Translate the state value into an ASCII state name
const char *um980GetStateName(SEMP_PARSE_STATE *parse)
{
    const char *name;

    do
    {
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

// Disable debug output from the parser
void UM980::disableParserDebug()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void UM980::enableParserDebug(Print *print)
{
    sempEnableDebugOutput(_sempParse, print);
}

// Disable debug output from the parser
void UM980::disableParserErrors()
{
    sempDisableDebugOutput(_sempParse);
}

// Enable debug output from the parser
void UM980::enableParserErrors(Print *print)
{
    sempEnableErrorOutput(_sempParse, print);
}

// Print the UM980 parser configuration
void UM980::printParserConfiguration(Print *print)
{
    sempPrintParserConfiguration(_sempParse, print);
}

//----------------------------------------
// UM980 support routines
//----------------------------------------

bool UM980::begin(HardwareSerial &serialPort, Print *parserDebug, Print *parserError)
{
    ptrUM980 = this;
    _hwSerialPort = &serialPort;

    // Initialize the parser
    _sempParse =
        sempBeginParser(parserTable, parserCount, parserNames, parserNameCount, 0, BUFFER_LENGTH, um980ProcessMessage,
                        "SFE_Unicore_GNSS_Library", parserError, parserDebug, badNmeaChecksum);
    if (!_sempParse)
    {
        debugPrintf("Unicore Lib: Failed to initialize the parser!");
        return false;
    }

    // We assume the user has started the serial port with proper pins and baud rate prior to calling begin()
    if (isConnected() == false)
    {
        sempStopParser(&_sempParse);
        return false;
    }
    return (true);
}

// Query the device with 'MODE', expect OK response
// Device may be booting and outputting other messages (ie, $devicename,COM3*65)
// Try a few times
bool UM980::isConnected()
{
    for (int x = 0; x < 3; x++)
    {
        disableOutput(); // Tell unit to stop transmitting

        // Wait until serial stops coming in
        uint16_t maxTime = 500;
        unsigned long startTime = millis();
        while (1)
        {
            delay(50);

            if (serialAvailable() == 0)
                break;
            while (serialAvailable())
                serialRead();

            if (millis() - startTime > maxTime)
                return (false);
        }

        if (sendQuery("MODE") == UM980_RESULT_OK)
            return (true);
        debugPrintf("UM980 failed to connect. Trying again.");
        delay(500);
    }
    return (false);
}

// If another task outside of this library is accessing the same Serial hardware, it can
// check to see if this library currently needs exclusive read/write access for a short period.
// If isBlocking is true, external consumers should not read/write to the Serial hardware
bool UM980::isBlocking()
{
    return (unicoreLibrarySemaphoreBlock);
}

// Calling this function with nothing sets the debug port to Serial
// You can also call it with other streams like Serial1, SerialUSB, etc.
void UM980::enableDebugging(Print &debugPort)
{
    _debugPort = &debugPort;
}
void UM980::disableDebugging()
{
    _debugPort = nullptr;
}

// Check for new data until there is no more
bool UM980::update()
{
    bool newData = false;

    unicoreLibrarySemaphoreBlock = true; // Allow external tasks to control serial hardware

    while (serialAvailable())
        newData = updateOnce();

    unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return (newData);
}

// Enable the display of parser transitions
void UM980::enablePrintParserTransitions()
{
    _printParserTransitions = true;
}

// Checks for new data once
// Used during sendString and sendQuery
// um980ProcessMessage() is called once the parser completes on a line
bool UM980::updateOnce()
{
    const char *endName;
    const char *startName = nullptr;
    SEMP_PARSE_ROUTINE startState;

    if (serialAvailable())
    {
        uint8_t incoming = serialRead();

        // Get the current state and state name
        if (_printParserTransitions)
        {
            startState = _sempParse->state;
            startName = um980GetStateName(_sempParse);
        }

        // Update the parser state based on the incoming byte
        sempParseNextByte(_sempParse, incoming);

        // Get the current state name
        if (_printParserTransitions)
        {
            endName = um980GetStateName(_sempParse);

            // Display the parser state transition
            debugPrintf("Unicore Lib: 0x%02x (%c), crc: 0x%08x, state: %s --> %s", incoming,
                        ((incoming >= ' ') && (incoming < 0x7f)) ? incoming : '.', _sempParse->crc, startName, endName);
        }
        return (true);
    }

    (void)startState; // Fix pesky warning-as-error

    return (false);
}

// Display the contents of a buffer
void UM980::dumpBuffer(const uint8_t *buffer, uint16_t length)
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
        sprintf(line, "0x%08lx: ", (long unsigned int)offset);

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
            sprintf(&line[strlen(line)], "%c",
                    ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
        debugPrintf("%s", line);

        // Set the next line of data
        buffer += bytes;
        offset += bytes;
    }
}

// Enable the display of received messages
void UM980::enablePrintRxMessages()
{
    _printRxMessages = true;
}

// Disable the display of received messages
void UM980::disablePrintRxMessages()
{
    _printRxMessages = false;
}

// Enable the hex dump of received messages
void UM980::enableRxMessageDump()
{
    _dumpRxMessages = true;
}

// Disable the hex dump of received messages
void UM980::disableRxMessageDump()
{
    _dumpRxMessages = false;
}

// Call back from within parser, for end of message
// Process a complete message incoming from parser
void um980ProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
    SEMP_SCRATCH_PAD *scratchPad = (SEMP_SCRATCH_PAD *)parse->scratchPad;

    if (ptrUM980->_printRxMessages)
    {
        // Display the raw message
        ptrUM980->debugPrintf("");
        switch (type)
        {
        case UM980_NMEA_PARSER_INDEX:
            ptrUM980->debugPrintf("Unicore Lib: Valid NMEA Sentence: %s, 0x%04x (%d) bytes",
                                  sempNmeaGetSentenceName(parse), parse->length, parse->length);
            break;

        case UM980_UNICORE_HASH_PARSER_INDEX:
            ptrUM980->debugPrintf("Unicore Lib: Valid Unicore Hash (#) Sentence: %s, 0x%04x (%d) bytes",
                                  sempUnicoreHashGetSentenceName(parse), parse->length, parse->length);
            break;

        case UM980_RTCM_PARSER_INDEX:
            ptrUM980->debugPrintf("Unicore Lib: Valid RTCM message: 0x%04x (%d) bytes", parse->length, parse->length);
            break;

        case UM980_UNICORE_BINARY_PARSER_INDEX:
            ptrUM980->debugPrintf("Unicore Lib: Valid Unicore message: 0x%04x (%d) bytes", parse->length,
                                  parse->length);
            break;
        }
    }

    // Dump the contents of the parsed messages
    if (ptrUM980->_dumpRxMessages)
        ptrUM980->dumpBuffer(parse->buffer, parse->length);

    // Process the message
    switch (type)
    {
    case UM980_UNICORE_BINARY_PARSER_INDEX:
        ptrUM980->unicoreHandler(parse->buffer, parse->length);
        break;

    case UM980_RTCM_PARSER_INDEX:
        break;

    case UM980_UNICORE_HASH_PARSER_INDEX:
        // Does this response contain the command we are looking for?
        if (strcasecmp((char *)scratchPad->unicoreHash.sentenceName, ptrUM980->commandName) == 0) // Found
        {
            ptrUM980->debugPrintf("Unicore Lib: Query response: %s", parse->buffer);
            ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_OK;
        }
        break;

    case UM980_NMEA_PARSER_INDEX:

        // Is this a NMEA response or command response?

        if (strcasecmp((char *)scratchPad->nmea.sentenceName, "command") != 0 &&
            strcasecmp((char *)scratchPad->nmea.sentenceName, "MASK") != 0 &&
            strcasecmp((char *)scratchPad->nmea.sentenceName, "CONFIG") != 0)
        {
            // command, MASK, CONFIG not found

            if (strcasecmp((char *)scratchPad->nmea.sentenceName, "GNGGA") == 0)
            {
                ptrUM980->debugPrintf("um980ProcessMessage GNGGA");
            }

            // Unknown response, ignore this message
            ptrUM980->debugPrintf("Unicore Lib: Message ignored: %s", parse->buffer);
        }
        else
        {
            // Does this response contain the command we are looking for?
            // It may be anywhere in the response:
            // $command,MODE,response: OK*5D
            char *responsePointer = strcasestr((char *)parse->buffer, ptrUM980->commandName);
            if (responsePointer != nullptr) // Found
            {
                // Display the command response
                ptrUM980->debugPrintf("Unicore Lib: Known command response: %s", parse->buffer);

                // Check to see if we got a command response
                responsePointer = strcasestr((char *)parse->buffer, "OK");
                if (responsePointer != nullptr) // Found
                {
                    ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_OK;
                    return;
                }

                responsePointer = strcasestr((char *)parse->buffer, "PARSING");
                if (responsePointer != nullptr) // Found
                {
                    ptrUM980->debugPrintf("Unicore Lib: Error response: %s", parse->buffer);
                    ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_ERROR;
                    return;
                }

                responsePointer = strcasestr((char *)parse->buffer, "CONFIG");
                if (responsePointer != nullptr) // Found
                {
                    ptrUM980->debugPrintf("CONFIG response: %s", parse->buffer);
                    ptrUM980->configHandler(parse->buffer, parse->length);
                    ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_CONFIG;
                    return;
                }
            }
            else
            {
                // Display the command response
                ptrUM980->debugPrintf("Unicore Lib: Unknown command response: %s", parse->buffer);
                ptrUM980->debugPrintf("Unicore Lib: Looking for command: %s", ptrUM980->commandName);
            }
        }
        break;
    }
}

// Mode commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Directly set a mode: setMode("ROVER");
bool UM980::setMode(const char *modeType)
{
    char command[50];
    snprintf(command, sizeof(command), "MODE %s", modeType);

    return (sendCommand(command));
}

// Directly set a base mode: setModeBase("40.09029479 -105.18505761 1560.089")
bool UM980::setModeBase(const char *baseType)
{
    char command[50];
    snprintf(command, sizeof(command), "BASE %s", baseType);

    return (setMode(command));
}

// Start base mode with given coordinates
bool UM980::setModeBaseGeodetic(double latitude, double longitude, double altitude)
{
    char command[50];
    snprintf(command, sizeof(command), "%0.11f %0.11f %0.6f", latitude, longitude, altitude);

    return (setModeBase(command));
}

// Start base mode with given coordinates
bool UM980::setModeBaseECEF(double coordinateX, double coordinateY, double coordinateZ)
{
    char command[50];
    snprintf(command, sizeof(command), "%0.4f %0.4f %0.4f", coordinateX, coordinateY, coordinateZ);

    return (setModeBase(command));
}

// Start base mode using self-optimization (similar to u-blox's Survey-In method)
bool UM980::setModeBaseAverage()
{
    return (setModeBaseAverage(60));
}

bool UM980::setModeBaseAverage(uint16_t averageTime)
{
    char command[50];
    snprintf(command, sizeof(command), "TIME %d", averageTime);

    return (setModeBase(command));
}

// Start rover mode: setModeRover("SURVEY")
bool UM980::setModeRover(const char *roverType)
{
    char command[50];
    snprintf(command, sizeof(command), "ROVER %s", roverType);

    return (setMode(command));
}
bool UM980::setModeRoverSurvey()
{
    return (setModeRover("SURVEY"));
}
bool UM980::setModeRoverUAV()
{
    return (setModeRover("UAV"));
}
bool UM980::setModeRoverAutomotive()
{
    return (setModeRover("AUTOMOTIVE"));
}
bool UM980::setModeRoverMow()
{
    return (setModeRover("SURVEY MOW")); // This fails for unknown reasons. Might be build7923 required, might not
                                         // be supported on UM980.
}

// Config commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Configure a given COM port to a given baud
// Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
bool UM980::setPortBaudrate(const char *comName, unsigned long newBaud)
{
    char command[50];
    snprintf(command, sizeof(command), "CONFIG %s %ld", comName, newBaud);

    return (sendCommand(command));
}

// Sets the baud rate of the port we are communicating on
// Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
bool UM980::setBaudrate(unsigned long newBaud)
{
    char command[50];
    snprintf(command, sizeof(command), "CONFIG %ld", newBaud);

    return (sendCommand(command));
}

// Enable Pulse Per Second signal with various settings
bool UM980::enablePPS(uint32_t widthMicroseconds, uint16_t periodMilliseconds, bool positivePolarity, int16_t rfDelay,
                      int16_t userDelay)
{
    char polarity[] = "POSITIVE";
    if (positivePolarity == false)
        strncpy(polarity, "NEGATIVE", sizeof(polarity));

    char command[50];
    snprintf(command, sizeof(command), "ENABLE GPS %s %ld %d %d %d", polarity, widthMicroseconds, periodMilliseconds,
             rfDelay, userDelay);

    return (configurePPS(command));
}

// Disable the PPS signal
bool UM980::disablePPS()
{
    return (configurePPS("DISABLE"));
}

bool UM980::configurePPS(const char *configString)
{
    char command[50];
    snprintf(command, sizeof(command), "CONFIG PPS %s", configString);

    return (sendCommand(command));
}

// Mask commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Available constellations: GPS, BDS, GLO, GAL, QZSS, IRNSS

// Enable a given constallation
// Returns true if successful
bool UM980::enableConstellation(const char *constellationName)
{
    char command[50];
    snprintf(command, sizeof(command), "%s", constellationName);

    return (enableSystem(command));
}
bool UM980::disableConstellation(const char *constellationName)
{
    char command[50];
    snprintf(command, sizeof(command), "%s", constellationName);

    return (disableSystem(command));
}

// Ignore satellites below a given elevation for a given constellation
bool UM980::setElevationAngle(int16_t elevationDegrees, const char *constellationName)
{
    char command[50];
    snprintf(command, sizeof(command), "%d %s", elevationDegrees, constellationName);

    return (disableSystem(command)); // Use MASK to set elevation angle
}

// Ignore satellites below a given elevation
bool UM980::setElevationAngle(int16_t elevationDegrees)
{
    char command[50];
    snprintf(command, sizeof(command), "%d", elevationDegrees);

    return (disableSystem(command)); // Use MASK to set elevation angle
}

// Ignore satellites below certain CN0 value
// C/N0, limits the observation data output of OBSV messages
bool UM980::setMinCNO(uint8_t dBHz)
{
    char command[50];
    snprintf(command, sizeof(command), "CN0 %d", dBHz);

    return (disableSystem(command)); // Use MASK to set CN0 value
}

// Enable a given frequency name
// See table 5-4 for list of frequency names
bool UM980::enableFrequency(const char *frequencyName)
{
    char command[50];
    snprintf(command, sizeof(command), "%s", frequencyName);

    return (enableSystem(command));
}

// Disable a given frequency name
bool UM980::disableFrequency(const char *frequencyName)
{
    char command[50];
    snprintf(command, sizeof(command), "%s", frequencyName);

    return (disableSystem(command));
}

// Called mask (disable) and unmask (enable), this is how to ignore certain constellations, or signal/frequencies,
// or satellite elevations Returns true if successful
bool UM980::enableSystem(const char *systemName)
{
    char command[50];
    snprintf(command, sizeof(command), "UNMASK %s", systemName);

    return (sendCommand(command));
}

bool UM980::disableSystem(const char *systemName)
{
    char command[50];
    snprintf(command, sizeof(command), "MASK %s", systemName);

    return (sendCommand(command));
}

// Data Output commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Set the output rate of a given message on a given COM port/Use
// 1, 0.5, 0.2, 0.1 corresponds to 1Hz, 2Hz, 5Hz, 10Hz respectively.
// Ex: GPGGA 0.5 <- 2 times per second
// Returns true if successful
bool UM980::setNMEAPortMessage(const char *sentenceType, const char *comName, float outputRate)
{
    char command[50];
    if (outputRate == 0)
        snprintf(command, sizeof(command), "UNLOG %s %s", comName, sentenceType);
    else
        snprintf(command, sizeof(command), "%s %s %0.2f", sentenceType, comName, outputRate);

    return (sendCommand(command));
}

// Set the output rate of a given message on the port we are communicating on
// 1, 0.5, 0.2, 0.1 corresponds to 1Hz, 2Hz, 5Hz, 10Hz respectively.
// Ex: GPGGA 0.5 <- 2 times per second
// Returns true if successful
bool UM980::setNMEAMessage(const char *sentenceType, float outputRate)
{
    char command[50];
    if (outputRate == 0)
        snprintf(command, sizeof(command), "UNLOG %s", sentenceType);
    else
        snprintf(command, sizeof(command), "%s %0.2f", sentenceType, outputRate);

    return (sendCommand(command));
}

// Set the output rate of a given RTCM message on a given COM port/Use
// 1, 0.5, 0.2, 0.1 corresponds to 1Hz, 2Hz, 5Hz, 10Hz respectively.
// Ex: RTCM1005 0.5 <- 2 times per second
// Returns true if successful
bool UM980::setRTCMPortMessage(const char *sentenceType, const char *comName, float outputRate)
{
    char command[50];
    if (outputRate == 0)
        snprintf(command, sizeof(command), "UNLOG %s %s", comName, sentenceType);
    else
        snprintf(command, sizeof(command), "%s %s %0.2f", sentenceType, comName, outputRate);

    return (sendCommand(command));
}

// Set the output rate of a given RTCM message on the port we are communicating on
// Ex: RTCM1005 10 <- Once every ten seconds
// Returns true if successful
bool UM980::setRTCMMessage(const char *sentenceType, float outputRate)
{
    char command[50];
    if (outputRate == 0)
        snprintf(command, sizeof(command), "UNLOG %s", sentenceType);
    else
        snprintf(command, sizeof(command), "%s %0.2f", sentenceType, outputRate);

    return (sendCommand(command));
}

// Other commands
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Disables all messages on this port
// Warning, each message has to be individually re-enabled
bool UM980::disableOutput()
{
    for (int x = 0; x < 5; x++)
    {
        if (sendCommand("UNLOG") == true)
        {
            stopAutoReports(); // Remove pointers so we will re-init next check
            return (true);
        }

        delay(10 * x);
    }

    return (false);
}

// Disable all messages on a given port
bool UM980::disableOutputPort(const char *comName)
{
    // We don't know if this is the COM port we are communicating on, so err on the side of caution.
    stopAutoReports(); // Remove pointers so we will re-init next check

    char command[50];
    snprintf(command, sizeof(command), "UNLOG %s", comName);

    return (sendCommand(command));
}

// We've issued an unlog, so the binary messages will no longer be coming in automatically
// Turn off pointers so the next time a getLatitude() is issued, the associated messsage is reinit'd
void UM980::stopAutoReports()
{
    if (packetBESTNAV != nullptr)
    {
        delete packetBESTNAV;
        packetBESTNAV = nullptr;
    }
    if (packetBESTNAVXYZ != nullptr)
    {
        delete packetBESTNAVXYZ;
        packetBESTNAVXYZ = nullptr;
    }
    if (packetRECTIME != nullptr)
    {
        delete packetRECTIME;
        packetRECTIME = nullptr;
    }
}

// Clear saved configurations, satellite ephemerides, position information, and reset baud rate to 115200bps.
bool UM980::factoryReset()
{
    return (sendCommand("FRESET"));
}

// Resetting the receiver will clear the satellite ephemerides, position information, satellite
// almanacs, ionosphere parameters and UTC parameters saved in the receiver.
bool UM980::reset()
{
    return (sendCommand("RESET"));
}

// Saves the current configuration into non-volatile memory (NVM),
// including LOG messages (except those triggered by ONCE), port
// configuration, etc.
bool UM980::saveConfiguration(uint16_t maxWait)
{
    return (sendCommand("SAVECONFIG", maxWait));
}

// Abstraction of the serial interface
// Useful if we ever need to support SoftwareSerial
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Enable printfs to various endpoints
// https://stackoverflow.com/questions/42131753/wrapper-for-printf
void UM980::debugPrintf(const char *format, ...)
{
    if (_debugPort == nullptr)
        return;

    va_list args;
    va_start(args, format);

    va_list args2;
    va_copy(args2, args);
    char buf[vsnprintf(nullptr, 0, format, args) + sizeof("\r\n")];

    vsnprintf(buf, sizeof buf, format, args2);

    // Add CR+LF
    buf[sizeof(buf) - 3] = '\r';
    buf[sizeof(buf) - 2] = '\n';
    buf[sizeof(buf) - 1] = '\0';

    _debugPort->write(buf, strlen(buf));

    va_end(args);
    va_end(args2);
}

// Discards any characters sitting in RX buffer
void UM980::clearBuffer()
{
    while (serialAvailable())
        serialRead();
}

uint16_t UM980::serialAvailable()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->available());
    }
    return (0);
}

uint8_t UM980::serialRead()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->read());
    }
    return (0);
}

void UM980::serialPrintln(const char *command)
{
    if (_hwSerialPort != nullptr)
    {
        _hwSerialPort->println(command);
    }
}

// Query and send functionality
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Send a command string (ie 'MODE ROVER') to the UM980
// Returns true if device reponded with OK to command
bool UM980::sendCommand(const char *command, uint16_t maxWaitMs)
{
    return (sendString(command, maxWaitMs) == UM980_RESULT_OK);
}

// Send a query string (ie 'MODE') to the UM980
// Looks for a query response ('#')
// Some commands like MASK or CONFIG have responses that begin with $
//'#' begins the responses to queries, ie 'MODE', ends with the result (ie MODE ROVER)
// #MODE,97,GPS,FINE,2283,499142000,0,0,18,22;MODE BASE -1280206.5680 -4716804.4030 4086665.4840,*60

//'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
// Contains reponse for caller
Um980Result UM980::sendQuery(const char *command, uint16_t maxWaitMs)
{
    Um980Result result;

    clearBuffer();

    // Send command and check for OK response
    result = sendString(command, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    strncpy(commandName, command, sizeof(commandName));
    commandResponse = UM980_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    unicoreLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            debugPrintf("Unicore Lib: Response timeout");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            return (UM980_RESULT_TIMEOUT_RESPONSE);
        }

        updateOnce(); // Will call um980ProcessMessage()

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_OK)
        {
            debugPrintf("Unicore Lib: Response received");
            break;
        }

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("Unicore Lib: Query failure");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            return (UM980_RESULT_RESPONSE_COMMAND_ERROR);
        }

        delay(1);
    }

    unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return (UM980_RESULT_OK);
}

// Send a string to the UM980
// Looks for a command response ('#' or '$')
//'#' begins the responses to queries, ie 'MODE', ends with the result (ie MODE ROVER)
//'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
//$command,badResponse,response: PARSING FAILD NO MATCHING FUNC  BADRESPONSE*40
// Returns UM980 result
Um980Result UM980::sendString(const char *command, uint16_t maxWaitMs)
{
    clearBuffer();

    debugPrintf("Unicore Lib: Sending command %s", command);
    strncpy(commandName, command, sizeof(commandName));      // Copy to class so that parsers can see it
    commandResponse = UM980_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    unicoreLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    serialPrintln(command);

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            debugPrintf("Unicore Lib: Command timeout");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            return (UM980_RESULT_TIMEOUT_RESPONSE);
        }

        updateOnce(); // Will call um980ProcessMessage()

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_OK)
        {
            debugPrintf("Unicore Lib: Command success");
            break;
        }

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("Unicore Lib: Command error");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            return (UM980_RESULT_RESPONSE_COMMAND_ERROR);
        }

        delay(1);
    }

    unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    return (UM980_RESULT_OK);
}

// Main Unicore handler and RAM inits
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#define CHECK_POINTER_BOOL(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return false;                                                                                              \
    }

#define CHECK_POINTER_VOID(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return;                                                                                                    \
    }

#define CHECK_POINTER_CHAR(packetPointer, initPointer)                                                                 \
    {                                                                                                                  \
        if (packetPointer == nullptr)                                                                                  \
            initPointer();                                                                                             \
        if (packetPointer == nullptr)                                                                                  \
            return ((char *)"Error");                                                                                  \
    }

// Cracks a given binary message into the applicable container
void UM980::unicoreHandler(uint8_t *response, uint16_t length)
{
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];

    if (messageID == messageIdBestnav)
    {
        debugPrintf("BestNav Handler");
        CHECK_POINTER_VOID(packetBESTNAV, initBestnav); // Check that RAM has been allocated

        lastUpdateGeodetic = millis(); // Update stale marker

        uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

        // Move data into given containers

        // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
        memcpy(&packetBESTNAV->data.solutionStatus, &data[offsetBestnavPsolStatus], sizeof(uint8_t));

        // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
        memcpy(&packetBESTNAV->data.positionType, &data[offsetBestnavPosType], sizeof(uint8_t));
        memcpy(&packetBESTNAV->data.velocityType, &data[offsetBestnavVelType], sizeof(uint8_t));

        memcpy(&packetBESTNAV->data.latitude, &data[offsetBestnavLat], sizeof(double));
        memcpy(&packetBESTNAV->data.longitude, &data[offsetBestnavLon], sizeof(double));
        memcpy(&packetBESTNAV->data.altitude, &data[offsetBestnavHgt], sizeof(double));
        memcpy(&packetBESTNAV->data.horizontalSpeed, &data[offsetBestnavHorSpd], sizeof(double));
        memcpy(&packetBESTNAV->data.verticalSpeed, &data[offsetBestnavVertSpd], sizeof(double));
        memcpy(&packetBESTNAV->data.trackGround, &data[offsetBestnavTrkGnd], sizeof(double));

        memcpy(&packetBESTNAV->data.latitudeDeviation, &data[offsetBestnavLatDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.longitudeDeviation, &data[offsetBestnavLonDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.heightDeviation, &data[offsetBestnavHgtDeviation], sizeof(float));

        memcpy(&packetBESTNAV->data.horizontalSpeedDeviation, &data[offsetBestnavHorspdStd], sizeof(float));
        memcpy(&packetBESTNAV->data.verticalSpeedDeviation, &data[offsetBestnavVerspdStd], sizeof(float));

        memcpy(&packetBESTNAV->data.satellitesTracked, &data[offsetBestnavSatsTracked], sizeof(uint8_t));
        memcpy(&packetBESTNAV->data.satellitesUsed, &data[offsetBestnavSatsUsed], sizeof(uint8_t));

        uint8_t extSolStat;
        memcpy(&extSolStat, &data[offsetBestnavExtSolStat], sizeof(uint8_t));
        packetBESTNAV->data.rtkSolution = extSolStat & 0x01;                   // 0 = unchecked, 1 = checked
        packetBESTNAV->data.pseudorangeCorrection = (extSolStat >> 1) & 0b111; // Limit to three bits
    }
    else if (messageID == messageIdRectime)
    {
        debugPrintf("RecTime Handler");
        CHECK_POINTER_VOID(packetRECTIME, initRectime); // Check that RAM has been allocated

        lastUpdateDateTime = millis();

        uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetRECTIME->data.timeStatus, &data[offsetRectimeClockStatus], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.timeOffset, &data[offsetRectimeOffset], sizeof(double));
        memcpy(&packetRECTIME->data.timeDeviation, &data[offsetRectimeOffsetStd], sizeof(double));
        memcpy(&packetRECTIME->data.year, &data[offsetRectimeUtcYear], sizeof(uint16_t));
        memcpy(&packetRECTIME->data.month, &data[offsetRectimeUtcMonth], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.day, &data[offsetRectimeUtcDay], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.hour, &data[offsetRectimeUtcHour], sizeof(uint8_t));
        memcpy(&packetRECTIME->data.minute, &data[offsetRectimeUtcMinute], sizeof(uint8_t));

        memcpy(&packetRECTIME->data.millisecond, &data[offsetRectimeUtcMillisecond], sizeof(uint32_t));
        packetRECTIME->data.second = round(packetRECTIME->data.millisecond / 1000.0);
        packetRECTIME->data.millisecond -= (packetRECTIME->data.second * 1000); // Remove seconds from milliseconds

        memcpy(&packetRECTIME->data.dateStatus, &data[offsetRectimeUtcStatus], sizeof(uint8_t));
    }
    else if (messageID == messageIdBestnavXyz)
    {
        debugPrintf("BestNavXyz Handler");
        CHECK_POINTER_VOID(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated

        lastUpdateEcef = millis(); // Update stale marker

        uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetBESTNAVXYZ->data.ecefX, &data[offsetBestnavXyzPX], sizeof(double));
        memcpy(&packetBESTNAVXYZ->data.ecefY, &data[offsetBestnavXyzPY], sizeof(double));
        memcpy(&packetBESTNAVXYZ->data.ecefZ, &data[offsetBestnavXyzPZ], sizeof(double));

        memcpy(&packetBESTNAVXYZ->data.ecefXDeviation, &data[offsetBestnavXyzPXDeviation], sizeof(float));
        memcpy(&packetBESTNAVXYZ->data.ecefYDeviation, &data[offsetBestnavXyzPYDeviation], sizeof(float));
        memcpy(&packetBESTNAVXYZ->data.ecefZDeviation, &data[offsetBestnavXyzPZDeviation], sizeof(float));
    }
    else if (messageID == messageIdVersion)
    {
        debugPrintf("Version Handler");
        CHECK_POINTER_VOID(packetVERSION, initVersion); // Check that RAM has been allocated

        lastUpdateVersion = millis(); // Update stale marker

        uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

        // Move data into given containers
        memcpy(&packetVERSION->data.modelType, &data[offsetVersionModuleType], sizeof(packetVERSION->data.modelType));
        memcpy(&packetVERSION->data.swVersion, &data[offsetVersionFirmwareVersion],
               sizeof(packetVERSION->data.swVersion));
        memcpy(&packetVERSION->data.efuseID, &data[offsetVersionEfuseID], sizeof(packetVERSION->data.efuseID));
        memcpy(&packetVERSION->data.compileTime, &data[offsetVersionCompTime], sizeof(packetVERSION->data.compileTime));
    }
    else
    {
        // Is this a NMEA sentence?
        if (response[0] == '$')
        {
            response[length] = '\0'; // Force terminator because strncasestr does not exist

            // The UM980 does not respond to binary requests when there is no GNSS reception.
            // Block BestNavB, etc commands if there is no fix.
            // Look for GNGGA NMEA then extract GNSS position status (spot 6).
            // $GNGGA,181535.00,,,,,0,00,9999.0,,,,,,*43
            char *responsePointer = strcasestr((char *)response, "GNGGA");
            if (responsePointer != nullptr) // Found
            {
                char gngga[100];
                strncpy(gngga, (const char *)response, length - 1); // Make copy before strtok

                debugPrintf("Unicore Lib: GNGGA message: %s\r\n", gngga);

                char *pt;
                pt = strtok(gngga, ",");
                int counter = 0;
                while (pt != NULL)
                {
                    int spotValue = atoi(pt);
                    if (counter++ == 6)
                        nmeaPositionStatus = spotValue;
                    pt = strtok(NULL, ",");
                }
            }
            else
            {
                // Unhandled NMEA message
                // debugPrintf("Unicore Lib: Unhandled NMEA sentence (%d bytes): %s\r\n", length, (char *)response);
            }
        }
        else
        {
            debugPrintf("Unicore Lib: Unknown message id: %d\r\n", messageID);
        }
    }
}

// Allocate RAM for packetVERSION and initialize it
bool UM980::initVersion()
{
    packetVERSION = new UNICORE_VERSION_t; // Allocate RAM for the main struct
    if (packetVERSION == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetVERSION->callbackPointerPtr = nullptr;
    //   packetVERSION->callbackData = nullptr;

    // Send command for single query
    if (sendCommand("VERSIONB") == false)
    {
        delete packetVERSION;
        packetVERSION = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("VERSION started");

    // Wait until response is received
    lastUpdateVersion = 0;
    uint16_t maxWait = 1000; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateVersion > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from VERSION start");
            delete packetVERSION;
            packetVERSION = nullptr;
            return (false);
        }
    }

    return (true);
}

// Allocate RAM for packetBESTNAV and initialize it
bool UM980::initBestnav(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("Unicore Lib: BestNav no fix");
        return (false);
    }

    packetBESTNAV = new UNICORE_BESTNAV_t; // Allocate RAM for the main struct
    if (packetBESTNAV == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAV->callbackPointerPtr = nullptr;
    //   packetBESTNAV->callbackData = nullptr;

    // Start outputting BESTNAV in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVB %d", rate);
    if (sendCommand(command) == false)
    {
        delete packetBESTNAV;
        packetBESTNAV = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("BestNav started");

    // Wait until first report is available
    lastUpdateGeodetic = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateGeodetic > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from BestNav start");
            delete packetBESTNAV;
            packetBESTNAV = nullptr;
            return (false);
        }
    }

    return (true);
}

// Allocate RAM for packetBESTNAVXYZ and initialize it
bool UM980::initBestnavXyz(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("Unicore Lib: BestNavXyz no fix");
        return (false);
    }

    packetBESTNAVXYZ = new UNICORE_BESTNAVXYZ_t; // Allocate RAM for the main struct
    if (packetBESTNAVXYZ == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAVXYZ->callbackPointerPtr = nullptr;
    //   packetBESTNAVXYZ->callbackData = nullptr;

    // Start outputting BESTNAVXYZ in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVXYZB %d", rate);
    if (sendCommand(command) == false)
    {
        delete packetBESTNAVXYZ;
        packetBESTNAVXYZ = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("BestNavXYZB started");

    // Wait until first report is available
    lastUpdateEcef = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateEcef > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from BestNavXyz start");
            delete packetBESTNAVXYZ;
            packetBESTNAVXYZ = nullptr;
            return (false);
        }
    }

    return (true);
}

// Allocate RAM for packetRECTIME and initialize it
bool UM980::initRectime(uint8_t rate)
{
    if ((startBinaryBeforeFix == false) && (isNmeaFixed() == false))
    {
        debugPrintf("Unicore Lib: RecTime no fix");
        return (false);
    }

    packetRECTIME = new UNICORE_RECTIME_t; // Allocate RAM for the main struct
    if (packetRECTIME == nullptr)
    {
        debugPrintf("Pointer alloc fail");
        return (false);
    }
    //   packetRECTIME->callbackPointerPtr = nullptr;
    //   packetRECTIME->callbackData = nullptr;

    debugPrintf("RecTime started");

    // Start outputting RECTIME in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "RECTIMEB %d", rate);
    if (sendCommand(command) == false)
    {
        delete packetRECTIME;
        packetRECTIME = nullptr; // Remove pointer so we will re-init next check
        return (false);
    }

    debugPrintf("RecTimeB started");

    // Wait until first report is available
    lastUpdateDateTime = 0;
    uint16_t maxWait = (1000 / rate) + 100; // Wait for one response to come in
    unsigned long startTime = millis();
    while (1)
    {
        update(); // Call parser
        if (lastUpdateDateTime > 0)
            break;
        if (millis() - startTime > maxWait)
        {
            debugPrintf("GNSS: Failed to get response from RecTime start");
            delete packetRECTIME;
            packetRECTIME = nullptr;
            return (false);
        }
    }

    return (true);
}

// All the general gets and sets
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

double UM980::getLatitude(uint16_t maxWait)
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.latitude);
}

double UM980::getLongitude()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.longitude);
}
double UM980::getAltitude()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.altitude);
}
double UM980::getHorizontalSpeed()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.horizontalSpeed);
}
double UM980::getVerticalSpeed()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.verticalSpeed);
}
double UM980::getTrackGround()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.trackGround);
}

float UM980::getLatitudeDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.latitudeDeviation);
}
float UM980::getLongitudeDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.longitudeDeviation);
}
float UM980::getAltitudeDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.heightDeviation);
}
float UM980::getHorizontalSpeedDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.horizontalSpeedDeviation);
}
float UM980::getVerticalSpeedDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.verticalSpeedDeviation);
}

uint8_t UM980::getSIV()
{
    return (getSatellitesTracked());
}
uint8_t UM980::getSatellitesTracked()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.satellitesTracked);
}
uint8_t UM980::getSatellitesUsed()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.satellitesUsed);
}

// 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
uint8_t UM980::getSolutionStatus()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.solutionStatus);
}

// 0 = no fix, 1 = dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckoning combined, 5 = time only
// fix
uint8_t UM980::getPositionType()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.positionType);
}
uint8_t UM980::getVelocityType()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.velocityType);
}

uint8_t UM980::getRTKSolution()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.rtkSolution);
}
uint8_t UM980::getPseudorangeCorrection()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.pseudorangeCorrection);
}

// Return the number of millis since last update
uint32_t UM980::getFixAgeMilliseconds()
{
    return (millis() - lastUpdateGeodetic);
}

double UM980::getEcefX()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefX);
}
double UM980::getEcefY()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefY);
}
double UM980::getEcefZ()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefZ);
}
float UM980::getEcefXDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefXDeviation);
}
float UM980::getEcefYDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefYDeviation);
}
float UM980::getEcefZDeviation()
{
    CHECK_POINTER_BOOL(packetBESTNAVXYZ, initBestnavXyz); // Check that RAM has been allocated
    return (packetBESTNAVXYZ->data.ecefZDeviation);
}

uint16_t UM980::getYear()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.year);
}
uint8_t UM980::getMonth()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.month);
}
uint8_t UM980::getDay()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.day);
}
uint8_t UM980::getHour()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.hour);
}
uint8_t UM980::getMinute()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.minute);
}
uint8_t UM980::getSecond()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.second);
}
uint16_t UM980::getMillisecond()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.millisecond);
}

uint8_t UM980::getTimeStatus()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeStatus);
}
uint8_t UM980::getDateStatus()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.dateStatus);
}

// Receiver clock offset relative to GPS time, s.
// Positive indicates that the receiver clock is ahead of GPS time. To calculate the GPS time, use the formula
// below: GPS time = receiver time - clock offset
double UM980::getTimeOffset()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeOffset);
}

// Standard deviation of the receiver clock offset, s.
double UM980::getTimeOffsetDeviation()
{
    CHECK_POINTER_BOOL(packetRECTIME, initRectime); // Check that RAM has been allocated
    return (packetRECTIME->data.timeDeviation);
}

uint8_t UM980::getModelType()
{
    CHECK_POINTER_BOOL(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.modelType);
}
char *UM980::getVersion()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.swVersion);
}
char *UM980::getID()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.efuseID);
}
char *UM980::getCompileTime()
{
    CHECK_POINTER_CHAR(packetVERSION, initVersion); // Check that RAM has been allocated
    return (packetVERSION->data.compileTime);
}

// Returns pointer to terminated response.
//$command,VERSION,response: OK*04
// #VERSION,92,GPS,FINE,2289,167126600,0,0,18,155;UM980,R4.10Build7923,HRPT00-S10C-P,2310415000001-MD22B1224961040,ff3bd496fd7ca68b,2022/09/28*45d62771
char *UM980::getVersionFull(uint16_t maxWaitMs)
{
    // Issue the version command
    Um980Result result = sendQuery("VERSION");

    // Process the response
    if (result == UM980_RESULT_OK)
        // Response sitting in buffer. Return pointer to buffer.
        return ((char *)_sempParse->buffer);
    else if (result == UM980_RESULT_TIMEOUT_RESPONSE)
        return ((char *)"Timeout");
    else if (result == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        return ((char *)"Error1");
    return ((char *)"Error2");
}

// Cracks a given CONFIG response into settings
void UM980::configHandler(uint8_t *response, uint16_t length)
{
    // We've received a response such as $CONFIG,COM3,CONFIG COM3 115200*23
    // See if it is the one we want
    char *responsePointer = strcasestr((char *)response, configStringToFind);
    if (responsePointer != nullptr) // Found
    {
        configStringFound = true;
    }
    else
    {
        // This config response was not what we were looking for
    }
}

// Given a string such as "CONFIG COM3 115200", query the device's config settings
// If the given string is in the CONFIG response, return true
// Send a CONFIG command and see if a specific string exists in the responses
// $command,config,response: OK*54
// $CONFIG,ANTENNA,CONFIG ANTENNA POWERON*7A
// $CONFIG,NMEAVERSION,CONFIG NMEAVERSION V410*47
// $CONFIG,RTK,CONFIG RTK TIMEOUT 120*6C
// $CONFIG,RTK,CONFIG RTK RELIABILITY 3 1*76
// $CONFIG,PPP,CONFIG PPP TIMEOUT 120*6C
// $CONFIG,DGPS,CONFIG DGPS TIMEOUT 300*6C
// $CONFIG,RTCMB1CB2A,CONFIG RTCMB1CB2A ENABLE*25
// $CONFIG,ANTENNADELTAHEN,CONFIG ANTENNADELTAHEN 0.0000 0.0000 0.0000*3A
// $CONFIG,PPS,CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0*6E
// $CONFIG,SIGNALGROUP,CONFIG SIGNALGROUP 2*16
// $CONFIG,ANTIJAM,CONFIG ANTIJAM AUTO*2B
// $CONFIG,AGNSS,CONFIG AGNSS DISABLE*70
// $CONFIG,BASEOBSFILTER,CONFIG BASEOBSFILTER DISABLE*70
// $CONFIG,COM1,CONFIG COM1 115200*23
// $CONFIG,COM2,CONFIG COM2 115200*23
// $CONFIG,COM3,CONFIG COM3 115200*23
bool UM980::isConfigurationPresent(const char *stringToFind, uint16_t maxWaitMs)
{
    Um980Result result;

    clearBuffer();

    // Send command and check for OK response
    result = sendString("CONFIG", maxWaitMs);
    if (result != UM980_RESULT_OK)
        // return (result);
        return (false);

    // Setup configStringToFind so configHandler() knows what to look for
    strncpy(configStringToFind, stringToFind, sizeof(configStringToFind));

    configStringFound = false; // configHandler() sets true if we find the intended string

    commandResponse = UM980_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    unicoreLibrarySemaphoreBlock = true; // Prevent external tasks from harvesting serial data

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            debugPrintf("Unicore Lib: Response timeout");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            // return (UM980_RESULT_TIMEOUT_RESPONSE);
            return (false);
        }

        updateOnce(); // Will call um980ProcessMessage() and configHandler()

        if (configStringFound == true)
        {
            // return (UM980_RESULT_CONFIG_PRESENT);
            return (true);
        }

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        {
            debugPrintf("Unicore Lib: Query failure");
            unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware
            // return (UM980_RESULT_RESPONSE_COMMAND_ERROR);
            return (false);
        }

        delay(1);
    }

    unicoreLibrarySemaphoreBlock = false; // Allow external tasks to control serial hardware

    // return (UM980_RESULT_OK);
    return (false);
}

// Returns true when GNGGA NMEA reports position status >= 1
bool UM980::isNmeaFixed()
{
    if (nmeaPositionStatus >= 1)
        return (true);
    return (false);
}

// By default, library will attempt to start RECTIME and BESTNAV regardless of GNSS fix
// This may lead to command timeouts as the UM980 does not appear to respond to BESTNAVB commands if 3D fix is not
// achieved. Set startBinartBeforeFix = false via disableBinaryBeforeFix() to block binary commands before a fix is
// achieved
void UM980::enableBinaryBeforeFix()
{
    startBinaryBeforeFix = true;
}
void UM980::disableBinaryBeforeFix()
{
    startBinaryBeforeFix = false;
}
