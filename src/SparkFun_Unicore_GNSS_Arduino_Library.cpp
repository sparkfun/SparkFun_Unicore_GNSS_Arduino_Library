/*
  This is a library to control Unicore GNSS receivers, with
  a focus on the UM980 Triband receiver. Other receivers in the
  same family should work: UM982, UM960, UM960L, etc.

  https://github.com/sparkfun/SparkFun_Unicore_GNSS_Arduino_Library
  Best used with the UM980 Breakout: https://www.sparkfun.com/products/xxxxx

  Development environment specifics:
  Arduino IDE 1.8.x

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

/*
  We use a simple ASCII interface to the UM980.
  + signifies it is TODO

  Configuration Commands
    Mode
      Base Fixed
      Base Average
      Rover
    Config
      Serial ports
      PPS
      +RTCM - See Appendix 2
      +UNDULATION
      +DGPS
      +RTK
      +STANDALONE
      +HEADING
      +SBAS
      +EVENT
      +SMOOTH
      +MMP
      +NMEA Version
      ...
    Mask
      Mask
       Elevation
       Frequency
      Unmask
    +Assisted position and time
    Data output
      NMEA (Defaults to NMEA)
        GPDTM
        GPGBS
        GPGGA
        GPGLL
        GPGNS
        GPGRS
        GPGSA
        GPGST
        GPGSV
        GPTHS
        GPRMC
        GPROT
        GPVTG
        GPZDA
      +Unicore special
    Other
      Unlog
      FReset
      Reset
      SaveConfig

  Data Query Commands

*/

#include "SparkFun_Unicore_GNSS_Arduino_Library.h"
#include "Arduino.h"

bool UM980::begin(HardwareSerial &serialPort)
{
    _hwSerialPort = &serialPort;
    _swSerialPort = nullptr;

    // We assume the user has started the serial port with proper pins and baud rate prior to calling begin()
    //_hwSerialPort->begin(115200);

    if (isConnected() == false)
    {
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
        char response[200];
        uint16_t maxResponseLength = sizeof(response);

        //debugPrintf("UM980: Sending MODE query."); // Response to query should start with #
        if (sendQuery("MODE", response, &maxResponseLength) == UM980_RESULT_OK)
            return (true);
        debugPrintf("UM980 failed to connect. Trying again.");
        delay(500);
    }
    return (false);
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

// Mode
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
    return (setModeRover(
        "SURVEY MOW")); // This fails for unknown reasons. Might be build7923 required, might not be supported on UM980.
}

// Config
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Configure a given COM port to a given baud
bool UM980::setPortBaudrate(const char *comName, unsigned long newBaud)
{
    char command[50];
    snprintf(command, sizeof(command), "CONFIG %s %d", comName, newBaud);

    return (sendCommand(command));
}

// Sets the baud rate of the port we are communicating on
bool UM980::setBaudrate(unsigned long newBaud)
{
    char command[50];
    snprintf(command, sizeof(command), "CONFIG %d", newBaud);

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
    snprintf(command, sizeof(command), "ENABLE GPS %s %d %d %d %d", polarity, widthMicroseconds, periodMilliseconds,
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

// Mask
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

// Called mask (disable) and unmask (enable), this is how to ignore certain constellations, or signal/frequencies, or
// satellite elevations
// Returns true if successful
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

// Data Output
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

// Other
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Disables all messages on this port
// Warning, each message has to be individually re-enabled
bool UM980::disableOutput()
{
    return (sendCommand("UNLOG"));
}

// Disable all messages on a given port
bool UM980::disableOutputPort(const char *comName)
{
    char command[50];
    snprintf(command, sizeof(command), "UNLOG %s", comName);

    return (sendCommand(command));
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
bool UM980::saveConfiguration()
{
    return (sendCommand("SAVECONFIG"));
}

// Lower level interface functions
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

// Send a command string (ie 'MODE ROVER') to the UM980
// Returns true if device reponded with OK to command
bool UM980::sendCommand(const char *command, uint16_t maxWaitMs)
{
    if (sendString(command, maxWaitMs) == UM980_RESULT_OK)
        return (true);
    return (false);
}

// Send a query string (ie 'MODE') to the UM980
// Looks for a query response ('#')
// Some commands like MASK or CONFIG have responses that begin with $
//'#' begins the responses to queries, ie 'MODE', ends with the result (ie MODE ROVER)
//'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
// Contains reponse for caller
Um980Result UM980::sendQuery(const char *command, char *response, uint16_t *maxResponseLength, uint16_t maxWaitMs)
{
    Um980Result result;

    clearBuffer();

    // Send command and check COMMAND OK response
    result = sendString(command, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    // Determine the response character we should expect
    // For CONFIG and MASK, the data will be prefixed with a $
    // For MODE and any Unicore command, the data will be prifixed with a #

    uint8_t characterToFind = '#'; // Default to looking for the start of a response to a MODE or Unicore command
    bool multiLineResponse = false;

    // Check for CONFIG or MASK
    char *commandConfigPointer = strcasestr(command, "CONFIG");
    if (commandConfigPointer != nullptr) // Found
    {
        characterToFind = '$';
        multiLineResponse = true;
        maxWaitMs = 100; // Because it is a multi-line response, we depend on a timeout to exit
    }

    char *commandMaskPointer = strcasestr(command, "MASK");
    if (commandMaskPointer != nullptr) // Found
    {
        characterToFind = '$';
        multiLineResponse = true; // Mask may be single or multi-line.
        maxWaitMs = 100;          // Because it is a multi-line response, we depend on a timeout to exit
    }

    result = getResponseAscii(characterToFind, response, maxResponseLength, multiLineResponse, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    debugPrintf("UM980 response length %d: %s", *maxResponseLength, response);

    result = checkChecksum(response); // Assumes response is \0 terminated
    if (result != UM980_RESULT_OK)
        return (result);

    // Verify this response is for this command
    char *commandPointer = strstr(response, command);
    if (commandPointer == nullptr) // Not found
    {
        debugPrintf("Wrong command");
        return (UM980_RESULT_WRONG_COMMAND);
    }

    return (UM980_RESULT_OK);
}

Um980Result UM980::sendQuery(const char *command, char *response, int *maxResponseLength, uint16_t maxWaitMs)
{
    return (sendQuery(command, response, (uint16_t *)maxResponseLength, maxWaitMs));
}

// Send a string to the UM980
// Looks for a command response ('$')
//'#' begins the responses to queries, ie 'MODE', ends with the result (ie MODE ROVER)
//'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
// Returns UM980 result
Um980Result UM980::sendString(const char *command, uint16_t maxWaitMs)
{
    clearBuffer();

    serialPrintln(command);

    char response[500];
    uint16_t responseLength = sizeof(response);
    Um980Result result;

    // All responses to command strings start with a $
    //$command,MASK,response: OK*4A
    result = getResponseAscii('$', response, &responseLength, false, maxWaitMs); // Single line response
    if (result != UM980_RESULT_OK)
        return (result);

    debugPrintf("UM980 response length %d: %s", responseLength, response);

    result = checkChecksum(response); // Assumes response is \0 terminated
    if (result != UM980_RESULT_OK)
        return (result);

    // Verify this response is for this command
    char *commandPointer = strstr(response, command);
    if (commandPointer == nullptr) // Not found
    {
        debugPrintf("Wrong command");
        return (UM980_RESULT_WRONG_COMMAND);
    }

    char *okPointer = strstr(
        response, ": OK"); // We could check for OK, but this should remove confusion with strings that may contain OK
    if (okPointer == nullptr)
    {
        debugPrintf("Command error");
        return (UM980_RESULT_COMMAND_ERROR); // Not found
    }

    return (UM980_RESULT_OK);
}

// Given the start character to look for
// Polls serial port until timout, or response received
// Returns OK if response is received with valid checksum
// Response is stored in response[] and null terminated
// maxResponseLength length is updated to contain the length of the response
Um980Result UM980::getResponseAscii(uint8_t characterToFind, char *response, uint16_t *maxResponseLength,
                                    bool multiLineResponse, uint16_t maxWaitMs)
{
    int responseLength = 0;

    // Find start byte
    //'#' begins the responses to queries, ie 'MODE', ends with the result (ie MODE ROVER)
    //'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
    Um980Result result = scanForCharacter(characterToFind, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    response[responseLength++] = characterToFind;

    // Find end byte '\n'
    // If this is a multiLine response then just wait for one
    bool foundEndByte = false;

    unsigned long lastByteReceivedTime = millis();
    while (1)
    {
        if (millis() - lastByteReceivedTime > maxWaitMs)
        {
            // If we found at least one endbyte, terminate the response and return length
            if (foundEndByte == true)
            {
                debugPrintf("Multi-line timeout");
                break;
            }

            debugPrintf("Timeout end byte");
            return (UM980_RESULT_TIMEOUT_END_BYTE);
        }

        if (serialAvailable())
        {
            byte incoming = serialRead();
            response[responseLength] = incoming;
            if (incoming == um980ASCIISyncEnd)
            {
                // If we are expecting a multi-line response, continue to scan until we timeout
                if (multiLineResponse == true)
                    foundEndByte = true;
                else
                    break;
            }

            responseLength++;
            if (responseLength == *maxResponseLength - 1) // Leave room for terminator
            {
                debugPrintf("Response overflow");
                return (UM980_RESULT_RESPONSE_OVERFLOW);
            }
        }
    }

    response[responseLength] = '\0'; // Terminate

    *maxResponseLength = responseLength; // Update caller's copy. Exclude the terminator in the length report.

    return (UM980_RESULT_OK);
}

// Send a string to the UM980, ie 'VERSIONB'
// Checks to see the command is confirmed with OK
// Returns the binary encoded response if CRC checks
Um980Result UM980::getResponseBinary(const char *command, uint8_t *response, uint16_t *maxResponseLength,
                                     uint16_t maxWaitMs)
{
    Um980Result result;

    // Send ASCII command and check for ASCII 'OK'
    result = sendString(command, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);
    // debugPrintf("Command sent success");

    uint16_t responseLength = 0;
    uint16_t expectedLength = 0;

    // debugPrintf("Scanning for start");

    // Find start byte 0xAA
    result = scanForCharacter(um980BinarySyncA, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    // debugPrintf("Start char found!");

    response[responseLength++] = um980BinarySyncA; // Store start byte so that offsets align

    // Begin gathering data
    unsigned long lastByteReceivedTime = millis();
    while (1)
    {
        if (millis() - lastByteReceivedTime > maxWaitMs)
        {
            debugPrintf("Timeout data byte");
            return (UM980_RESULT_TIMEOUT_DATA_BYTE);
        }

        if (serialAvailable())
        {
            uint8_t incoming = serialRead();

            lastByteReceivedTime = millis();

            response[responseLength] = incoming;

            if (responseLength == offsetHeaderMessageLength)
            {
                expectedLength = incoming; // LSB
            }
            if (responseLength == offsetHeaderMessageLength + 1)
            {
                expectedLength |= (uint16_t)incoming << 8; // MSB

                // The overall message length is header (24) + data (expectedLength) + CRC (4)
                expectedLength = um980HeaderLength + expectedLength + 4;
            }

            responseLength++;

            if (responseLength == *maxResponseLength)
            {
                debugPrintf("Response overflow");
                return (UM980_RESULT_RESPONSE_OVERFLOW);
            }

            if (expectedLength > 0 && responseLength == expectedLength)
                break; // Reached the end of the packet
        }
    }

    if (response[offsetHeaderSyncB] != um980BinarySyncB || response[offsetHeaderSyncC] != um980BinarySyncC)
    {
        debugPrintf("Bad start byte");
        return (UM980_RESULT_BAD_START_BYTE);
    }
    // debugPrintf("Good start bytes");

    debugPrintf("UM980 response length: %d", responseLength);

    //  for (int x = 0 ; x < responseLength ; x++)
    //    debugPrintf("%d) 0x%02X\r\n", x, response[x]);

    uint32_t sentenceCRC =
        ((uint32_t)response[responseLength - 4] << (8 * 0)) | ((uint32_t)response[responseLength - 3] << (8 * 1)) |
        ((uint32_t)response[responseLength - 2] << (8 * 2)) | ((uint32_t)response[responseLength - 1] << (8 * 3));
    uint32_t calculatedCRC =
        calculateCRC32(response, responseLength - 4); // CRC is calculated on entire messsage, sans CRC

    if (sentenceCRC != calculatedCRC)
    {
        debugPrintf("CRC failed. Sentence CRC: 0x%02X Calculated CRC: 0x%02X", sentenceCRC, calculatedCRC);
        return (UM980_RESULT_BAD_CRC);
    }

    // debugPrintf("CRC matches!");

    *maxResponseLength = responseLength;

    return (UM980_RESULT_OK);
}

// Scan serial until character is found
// Return if success or timeout
Um980Result UM980::scanForCharacter(uint8_t characterToFind, uint16_t maxWaitMs)
{
    unsigned long startTime = millis();

    // debugPrintf("Scanning for %c", characterToFind);

    while (1)
    {
        if (millis() - startTime > maxWaitMs)
        {
            debugPrintf("Timeout start byte");
            return (UM980_RESULT_TIMEOUT_START_BYTE);
        }

        if (serialAvailable())
        {
            uint8_t incoming = serialRead();
            // debugPrintf("Char found: 0x%02X %c", incoming, incoming);
            // Serial.write(incoming);
            if (incoming == characterToFind)
            {
                return (UM980_RESULT_OK);
            }
        }
    }
    return (UM980_RESULT_OK); // We should never get here
}

// Scans a repsonse for the first complete packet including header and Checksum
// Assumes response is null terminated
// Ex: $command,MODE ROVER,response: OK*21
// Checksum is 8-bit XOR, includes $, excludes * (which is different from NMEA that excludes both)
// https://www.scadacore.com/tools/programming-calculators/online-checksum-calculator/
// Returns OK if valid checksum
Um980Result UM980::checkChecksum(char *response)
{
    // Begin Checksum
    int calculatedChecksum = 0;
    int sentenceChecksum = 0;
    int packetLength = 0;

    for (packetLength = 0; packetLength < strlen(response); packetLength++) // Include first char ('$') in Checksum
    {
        if (response[packetLength] == '*')
        {
            // Get CRC then break
            if (strlen(response) - packetLength > 2)
            {
                char hexString[3] = {0}; // Need spot for terminator
                hexString[0] = response[packetLength + 1];
                hexString[1] = response[packetLength + 2];
                sentenceChecksum = (int)strtol(hexString, NULL, 16);
            }
            break; // Exclude * from checksum
        }

        calculatedChecksum ^= response[packetLength];
    }
    if (calculatedChecksum != sentenceChecksum)
    {
        debugPrintf("Checksum failed. calculatedChecksum: %02X sentenceChecksum: %02X", calculatedChecksum,
                    sentenceChecksum);
        return (UM980_RESULT_BAD_CHECKSUM);
    }

    return (UM980_RESULT_OK);
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
    else if (_swSerialPort != nullptr)
    {
        return (_swSerialPort->available());
    }
    return (0);
}

uint8_t UM980::serialRead()
{
    if (_hwSerialPort != nullptr)
    {
        return (_hwSerialPort->read());
    }
    else if (_swSerialPort != nullptr)
    {
        return (_swSerialPort->read());
    }
    return (0);
}

void UM980::serialPrintln(const char *command)
{
    if (_hwSerialPort != nullptr)
    {
        _hwSerialPort->println(command);
    }
    else if (_swSerialPort != nullptr)
    {
        _swSerialPort->println(command);
    }
}

const unsigned long crcTable[256] = {
    0x00000000UL, 0x77073096UL, 0xEE0E612CUL, 0x990951BAUL, 0x076DC419UL, 0x706AF48FUL, 0xE963A535UL, 0x9E6495A3UL,
    0x0EDB8832UL, 0x79DCB8A4UL, 0xE0D5E91EUL, 0x97D2D988UL, 0x09B64C2BUL, 0x7EB17CBDUL, 0xE7B82D07UL, 0x90BF1D91UL,
    0x1DB71064UL, 0x6AB020F2UL, 0xF3B97148UL, 0x84BE41DEUL, 0x1ADAD47DUL, 0x6DDDE4EBUL, 0xF4D4B551UL, 0x83D385C7UL,
    0x136C9856UL, 0x646BA8C0UL, 0xFD62F97AUL, 0x8A65C9ECUL, 0x14015C4FUL, 0x63066CD9UL, 0xFA0F3D63UL, 0x8D080DF5UL,
    0x3B6E20C8UL, 0x4C69105EUL, 0xD56041E4UL, 0xA2677172UL, 0x3C03E4D1UL, 0x4B04D447UL, 0xD20D85FDUL, 0xA50AB56BUL,
    0x35B5A8FAUL, 0x42B2986CUL, 0xDBBBC9D6UL, 0xACBCF940UL, 0x32D86CE3UL, 0x45DF5C75UL, 0xDCD60DCFUL, 0xABD13D59UL,
    0x26D930ACUL, 0x51DE003AUL, 0xC8D75180UL, 0xBFD06116UL, 0x21B4F4B5UL, 0x56B3C423UL, 0xCFBA9599UL, 0xB8BDA50FUL,
    0x2802B89EUL, 0x5F058808UL, 0xC60CD9B2UL, 0xB10BE924UL, 0x2F6F7C87UL, 0x58684C11UL, 0xC1611DABUL, 0xB6662D3DUL,
    0x76DC4190UL, 0x01DB7106UL, 0x98D220BCUL, 0xEFD5102AUL, 0x71B18589UL, 0x06B6B51FUL, 0x9FBFE4A5UL, 0xE8B8D433UL,
    0x7807C9A2UL, 0x0F00F934UL, 0x9609A88EUL, 0xE10E9818UL, 0x7F6A0DBBUL, 0x086D3D2DUL, 0x91646C97UL, 0xE6635C01UL,
    0x6B6B51F4UL, 0x1C6C6162UL, 0x856530D8UL, 0xF262004EUL, 0x6C0695EDUL, 0x1B01A57BUL, 0x8208F4C1UL, 0xF50FC457UL,
    0x65B0D9C6UL, 0x12B7E950UL, 0x8BBEB8EAUL, 0xFCB9887CUL, 0x62DD1DDFUL, 0x15DA2D49UL, 0x8CD37CF3UL, 0xFBD44C65UL,
    0x4DB26158UL, 0x3AB551CEUL, 0xA3BC0074UL, 0xD4BB30E2UL, 0x4ADFA541UL, 0x3DD895D7UL, 0xA4D1C46DUL, 0xD3D6F4FBUL,
    0x4369E96AUL, 0x346ED9FCUL, 0xAD678846UL, 0xDA60B8D0UL, 0x44042D73UL, 0x33031DE5UL, 0xAA0A4C5FUL, 0xDD0D7CC9UL,
    0x5005713CUL, 0x270241AAUL, 0xBE0B1010UL, 0xC90C2086UL, 0x5768B525UL, 0x206F85B3UL, 0xB966D409UL, 0xCE61E49FUL,
    0x5EDEF90EUL, 0x29D9C998UL, 0xB0D09822UL, 0xC7D7A8B4UL, 0x59B33D17UL, 0x2EB40D81UL, 0xB7BD5C3BUL, 0xC0BA6CADUL,
    0xEDB88320UL, 0x9ABFB3B6UL, 0x03B6E20CUL, 0x74B1D29AUL, 0xEAD54739UL, 0x9DD277AFUL, 0x04DB2615UL, 0x73DC1683UL,
    0xE3630B12UL, 0x94643B84UL, 0x0D6D6A3EUL, 0x7A6A5AA8UL, 0xE40ECF0BUL, 0x9309FF9DUL, 0x0A00AE27UL, 0x7D079EB1UL,
    0xF00F9344UL, 0x8708A3D2UL, 0x1E01F268UL, 0x6906C2FEUL, 0xF762575DUL, 0x806567CBUL, 0x196C3671UL, 0x6E6B06E7UL,
    0xFED41B76UL, 0x89D32BE0UL, 0x10DA7A5AUL, 0x67DD4ACCUL, 0xF9B9DF6FUL, 0x8EBEEFF9UL, 0x17B7BE43UL, 0x60B08ED5UL,
    0xD6D6A3E8UL, 0xA1D1937EUL, 0x38D8C2C4UL, 0x4FDFF252UL, 0xD1BB67F1UL, 0xA6BC5767UL, 0x3FB506DDUL, 0x48B2364BUL,
    0xD80D2BDAUL, 0xAF0A1B4CUL, 0x36034AF6UL, 0x41047A60UL, 0xDF60EFC3UL, 0xA867DF55UL, 0x316E8EEFUL, 0x4669BE79UL,
    0xCB61B38CUL, 0xBC66831AUL, 0x256FD2A0UL, 0x5268E236UL, 0xCC0C7795UL, 0xBB0B4703UL, 0x220216B9UL, 0x5505262FUL,
    0xC5BA3BBEUL, 0xB2BD0B28UL, 0x2BB45A92UL, 0x5CB36A04UL, 0xC2D7FFA7UL, 0xB5D0CF31UL, 0x2CD99E8BUL, 0x5BDEAE1DUL,
    0x9B64C2B0UL, 0xEC63F226UL, 0x756AA39CUL, 0x026D930AUL, 0x9C0906A9UL, 0xEB0E363FUL, 0x72076785UL, 0x05005713UL,
    0x95BF4A82UL, 0xE2B87A14UL, 0x7BB12BAEUL, 0x0CB61B38UL, 0x92D28E9BUL, 0xE5D5BE0DUL, 0x7CDCEFB7UL, 0x0BDBDF21UL,
    0x86D3D2D4UL, 0xF1D4E242UL, 0x68DDB3F8UL, 0x1FDA836EUL, 0x81BE16CDUL, 0xF6B9265BUL, 0x6FB077E1UL, 0x18B74777UL,
    0x88085AE6UL, 0xFF0F6A70UL, 0x66063BCAUL, 0x11010B5CUL, 0x8F659EFFUL, 0xF862AE69UL, 0x616BFFD3UL, 0x166CCF45UL,
    0xA00AE278UL, 0xD70DD2EEUL, 0x4E048354UL, 0x3903B3C2UL, 0xA7672661UL, 0xD06016F7UL, 0x4969474DUL, 0x3E6E77DBUL,
    0xAED16A4AUL, 0xD9D65ADCUL, 0x40DF0B66UL, 0x37D83BF0UL, 0xA9BCAE53UL, 0xDEBB9EC5UL, 0x47B2CF7FUL, 0x30B5FFE9UL,
    0xBDBDF21CUL, 0xCABAC28AUL, 0x53B39330UL, 0x24B4A3A6UL, 0xBAD03605UL, 0xCDD70693UL, 0x54DE5729UL, 0x23D967BFUL,
    0xB3667A2EUL, 0xC4614AB8UL, 0x5D681B02UL, 0x2A6F2B94UL, 0xB40BBE37UL, 0xC30C8EA1UL, 0x5A05DF1BUL, 0x2D02EF8DUL};

// Calculate and return the CRC of the given buffer
uint32_t UM980::calculateCRC32(uint8_t *charBuffer, uint16_t bufferSize)
{
    uint32_t crc = 0;
    for (uint16_t x = 0; x < bufferSize; x++)
        crc = crcTable[(crc ^ charBuffer[x]) & 0xFF] ^ (crc >> 8);
    return crc;
}

// Issue the RECTIME command and parse out pertinent data
Um980Result UM980::updateDateTime(uint16_t maxWaitMs)
{
    uint8_t response[500];
    uint16_t responseLength = sizeof(response);
    Um980Result result;

    result = getResponseBinary("RECTIMEB", response, &responseLength, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);
    // debugPrintf("Got binary result");

    // Confirm message ID
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];
    if (messageID != messageIdRectime)
        return (UM980_RESULT_WRONG_MESSAGE_ID);
    // debugPrintf("Good message ID");

    lastUpdateDateTime = millis();

    uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

    // Move data into given containers
    memcpy(&timeStatus, &data[offsetRectimeClockStatus], sizeof(uint8_t));
    memcpy(&timeOffset, &data[offsetRectimeOffset], sizeof(double));
    memcpy(&timeDeviation, &data[offsetRectimeOffsetStd], sizeof(double));
    memcpy(&year, &data[offsetRectimeUtcYear], sizeof(uint16_t));
    memcpy(&month, &data[offsetRectimeUtcMonth], sizeof(uint8_t));
    memcpy(&day, &data[offsetRectimeUtcDay], sizeof(uint8_t));
    memcpy(&hour, &data[offsetRectimeUtcHour], sizeof(uint8_t));
    memcpy(&minute, &data[offsetRectimeUtcMinute], sizeof(uint8_t));

    memcpy(&millisecond, &data[offsetRectimeUtcMillisecond], sizeof(uint32_t));
    second = round(millisecond / 1000.0);
    millisecond -= second * 1000; // Remove seconds from milliseconds

    memcpy(&dateStatus, &data[offsetRectimeUtcStatus], sizeof(uint8_t));

    return (UM980_RESULT_OK);
}

// Issue the BESTNAVB command and parse out pertinent data
Um980Result UM980::updateGeodetic(uint16_t maxWaitMs)
{
    uint8_t response[500];
    uint16_t responseLength = sizeof(response);
    Um980Result result;

    result = getResponseBinary("BESTNAVB", response, &responseLength, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    // Confirm message ID
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];
    if (messageID != messageIdBestnav)
        return (UM980_RESULT_WRONG_MESSAGE_ID);
    // debugPrintf("Good message ID");

    lastUpdateGeodetic = millis(); // Update stale marker

    uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

    // Move data into given containers

    // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
    memcpy(&solutionStatus, &data[offsetBestnavPsolStatus], sizeof(uint8_t));

    // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
    memcpy(&positionType, &data[offsetBestnavPosType], sizeof(uint8_t));

    memcpy(&latitude, &data[offsetBestnavLat], sizeof(double));
    memcpy(&longitude, &data[offsetBestnavLon], sizeof(double));
    memcpy(&altitude, &data[offsetBestnavHgt], sizeof(double));
    memcpy(&latitudeDeviation, &data[offsetBestnavLatDeviation], sizeof(float));
    memcpy(&longitudeDeviation, &data[offsetBestnavLonDeviation], sizeof(float));
    memcpy(&heightDeviation, &data[offsetBestnavHgtDeviation], sizeof(float));
    memcpy(&satellitesTracked, &data[offsetBestnavSatsTracked], sizeof(uint8_t));
    memcpy(&satellitesUsed, &data[offsetBestnavSatsUsed], sizeof(uint8_t));

    uint8_t extSolStat;
    memcpy(&extSolStat, &data[offsetBestnavExtSolStat], sizeof(uint8_t));
    rtkSolution = extSolStat & 0x01;// 0 = checked, 1 = unchecked
    pseudorangeCorrection = extSolStat >> 1;

    return (UM980_RESULT_OK);
}

// Issue the BESTNAVXYZB command and parse out pertinent data
Um980Result UM980::updateEcef(uint16_t maxWaitMs)
{
    uint8_t response[500];
    uint16_t responseLength = sizeof(response);
    Um980Result result;

    unsigned long startTime = millis();

    result = getResponseBinary("BESTNAVXYZB", response, &responseLength, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);
    debugPrintf("Got binary result");

    unsigned long stopTime = millis();

    debugPrintf("Request time: %d", stopTime - startTime);

    // Confirm message ID
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];
    if (messageID != messageIdBestnavXyz)
        return (UM980_RESULT_WRONG_MESSAGE_ID);
    debugPrintf("Good message ID");

    lastUpdateEcef = millis(); // Update stale marker

    uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

    // Move data into given containers
    memcpy(&ecefX, &data[offsetBestnavXyzPX], sizeof(double));
    memcpy(&ecefY, &data[offsetBestnavXyzPY], sizeof(double));
    memcpy(&ecefZ, &data[offsetBestnavXyzPZ], sizeof(double));

    memcpy(&ecefXDeviation, &data[offsetBestnavXyzPXDeviation], sizeof(float));
    memcpy(&ecefYDeviation, &data[offsetBestnavXyzPYDeviation], sizeof(float));
    memcpy(&ecefZDeviation, &data[offsetBestnavXyzPZDeviation], sizeof(float));

    memcpy(&satellitesTracked, &data[offsetBestnavXyzSatsTracked], sizeof(uint8_t));
    memcpy(&satellitesUsed, &data[offsetBestnavXyzSatsUsed], sizeof(uint8_t));

    memcpy(&solutionStatus, &data[offsetBestnavXyzPsolStatus], sizeof(uint8_t));
    memcpy(&positionType, &data[offsetBestnavXyzPosType], sizeof(uint8_t));

    uint8_t extSolStat;
    memcpy(&extSolStat, &data[offsetBestnavXyzExtSolStat], sizeof(uint8_t));
    rtkSolution = extSolStat & 0x01;
    pseudorangeCorrection = extSolStat >> 1;

    return (UM980_RESULT_OK);
}

bool UM980::staleGeodetic()
{
    if (millis() - lastUpdateGeodetic > dataFreshLimit_ms)
        return (true);
    return (false);
}

double UM980::getLatitude()
{
    if (staleGeodetic())
        updateGeodetic();
    return (latitude);
}
double UM980::getLongitude()
{
    if (staleGeodetic())
        updateGeodetic();
    return (longitude);
}
double UM980::getAltitude()
{
    if (staleGeodetic())
        updateGeodetic();
    return (altitude);
}
float UM980::getLatitudeDeviation()
{
    if (staleGeodetic())
        updateGeodetic();
    return (latitudeDeviation);
}
float UM980::getLongitudeDeviation()
{
    if (staleGeodetic())
        updateGeodetic();
    return (longitudeDeviation);
}
float UM980::getAltitudeDeviation()
{
    if (staleGeodetic())
        updateGeodetic();
    return (heightDeviation);
}

bool UM980::staleEcef()
{
    if (millis() - lastUpdateEcef > dataFreshLimit_ms)
        return (true);
    return (false);
}

double UM980::getEcefX()
{
    if (staleEcef())
        updateEcef();
    return (ecefX);
}
double UM980::getEcefY()
{
    if (staleEcef())
        updateEcef();
    return (ecefY);
}
double UM980::getEcefZ()
{
    if (staleEcef())
        updateEcef();
    return (ecefZ);
}
float UM980::getEcefXDeviation()
{
    if (staleEcef())
        updateEcef();
    return (ecefXDeviation);
}
float UM980::getEcefYDeviation()
{
    if (staleEcef())
        updateEcef();
    return (ecefYDeviation);
}
float UM980::getEcefZDeviation()
{
    if (staleEcef())
        updateEcef();
    return (ecefZDeviation);
}

uint8_t UM980::getSIV()
{
    return (getSatellitesTracked());
}
uint8_t UM980::getSatellitesUsed()
{
    if (staleGeodetic())
        updateGeodetic();
    return (satellitesUsed);
}
uint8_t UM980::getSatellitesTracked()
{
    if (staleGeodetic())
        updateGeodetic();
    return (satellitesTracked);
}

// 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
uint8_t UM980::getSolutionStatus()
{
    if (staleGeodetic())
        updateGeodetic();
    return (solutionStatus);
}

// 0 = no fix, 1 = dead reckoning only, 2 = 2D-fix, 3 = 3D-fix, 4 = GNSS + dead reckoning combined, 5 = time only fix
uint8_t UM980::getPositionType()
{
    if (staleGeodetic())
        updateGeodetic();
    return (positionType);
}

uint8_t UM980::getRTKSolution()
{
    if (staleGeodetic())
        updateGeodetic();
    return (rtkSolution);
}
uint8_t UM980::getPseudorangeCorrection()
{
    if (staleGeodetic())
        updateGeodetic();
    return (pseudorangeCorrection);
}

bool UM980::staleDateTime()
{
    if (millis() - lastUpdateDateTime > dataFreshLimit_ms)
        return (true);
    return (false);
}

uint16_t UM980::getYear()
{
    if (staleDateTime())
        updateDateTime();
    return (year);
}
uint8_t UM980::getMonth()
{
    if (staleDateTime())
        updateDateTime();
    return (month);
}
uint8_t UM980::getDay()
{
    if (staleDateTime())
        updateDateTime();
    return (day);
}
uint8_t UM980::getHour()
{
    if (staleDateTime())
        updateDateTime();
    return (hour);
}
uint8_t UM980::getMinute()
{
    if (staleDateTime())
        updateDateTime();
    return (minute);
}
uint8_t UM980::getSecond()
{
    if (staleDateTime())
        updateDateTime();
    return (second);
}
uint16_t UM980::getMillisecond()
{
    if (staleDateTime())
        updateDateTime();
    return (second);
}

uint8_t UM980::getTimeStatus()
{
    if (staleDateTime())
        updateDateTime();
    return (timeStatus);
}
uint8_t UM980::getDateStatus()
{
    if (staleDateTime())
        updateDateTime();
    return (dateStatus);
}

// Receiver clock offset relative to GPS time, s.
// Positive indicates that the receiver clock is ahead of GPS time. To calculate the GPS time, use the formula below:
// GPS time = receiver time - clock offset
double UM980::getTimeOffset()
{
    if (staleDateTime())
        updateDateTime();
    return (timeOffset);
}

// Standard deviation of the receiver clock offset, s.
double UM980::getTimeOffsetDeviation()
{
    if (staleDateTime())
        updateDateTime();
    return (timeDeviation);
}

// Return the number of millis since last updateGeodetic()
uint32_t UM980::getFixAgeMilliseconds()
{
    if (staleGeodetic())
        updateGeodetic();
    return (millis() - lastUpdateGeodetic);
}
