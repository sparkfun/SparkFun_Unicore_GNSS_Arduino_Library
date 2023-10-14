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
#include "parser.h"

UM980 *ptrUM980 = nullptr; // Global pointer for external parser access into library class

PARSE_STATE parse = {PARSE_STATE_WAITING_FOR_PREAMBLE};

bool UM980::begin(HardwareSerial &serialPort)
{
    ptrUM980 = this;
    _hwSerialPort = &serialPort;

    // We assume the user has started the serial port with proper pins and baud rate prior to calling begin()

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
        serialPrintln("UNLOG"); // Blindly tell unit to stop transmitting

        // Wait until serial stops coming in
        while (1)
        {
            delay(50);
            if (serialAvailable() == 0)
                break;
            while (serialAvailable())
                serialRead();
        }

        char response[200];

        uint16_t maxResponseLength = sizeof(response);

        // debugPrintf("UM980: Sending MODE query."); // Response to query should start with #
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

// Check for new data until there is no more
bool UM980::update()
{
    bool newData = false;
    while (serialAvailable())
        newData = updateOnce();
    return (newData);
}

// Checks for new data once
// Used during sendString and sendQuery
bool UM980::updateOnce()
{
    if (serialAvailable())
    {
        uint8_t incoming = serialRead();

        // Move byte into parser
        parse.buffer[parse.length++] = incoming;
        parse.length %= PARSE_BUFFER_LENGTH;

        // parse.state(&parse, incoming);
        //  Update the parser state based on the incoming byte
        switch (parse.state)
        {
        default:
            Serial.printf("Case not found! : %d\r\n", parse.state);
            // Drop to waiting for preamble
        case (PARSE_STATE_WAITING_FOR_PREAMBLE):
            waitForPreamble(&parse, incoming);
            break;

        case (PARSE_STATE_NMEA_FIRST_COMMA):
            nmeaFindFirstComma(&parse, incoming);
            break;
        case (PARSE_STATE_NMEA_FIND_ASTERISK):
            nmeaFindAsterisk(&parse, incoming);
            break;
        case (PARSE_STATE_NMEA_CHECKSUM1):
            nmeaChecksumByte1(&parse, incoming);
            break;
        case (PARSE_STATE_NMEA_CHECKSUM2):
            nmeaChecksumByte2(&parse, incoming);
            break;
        case (PARSE_STATE_NMEA_TERMINATION):
            nmeaLineTermination(&parse, incoming);
            break;

        case (PARSE_STATE_UNICORE_SYNC2):
            unicoreBinarySync2(&parse, incoming);
            break;
        case (PARSE_STATE_UNICORE_SYNC3):
            unicoreBinarySync3(&parse, incoming);
            break;
        case (PARSE_STATE_UNICORE_READ_LENGTH):
            unicoreBinaryReadLength(&parse, incoming);
            break;
        case (PARSE_STATE_UNICORE_READ_DATA):
            unicoreReadData(&parse, incoming);
            break;

        case (PARSE_STATE_RTCM_LENGTH1):
            rtcmReadLength1(&parse, incoming);
            break;
        case (PARSE_STATE_RTCM_LENGTH2):
            rtcmReadLength2(&parse, incoming);
            break;
        case (PARSE_STATE_RTCM_MESSAGE1):
            rtcmReadMessage1(&parse, incoming);
            break;
        case (PARSE_STATE_RTCM_MESSAGE2):
            rtcmReadMessage2(&parse, incoming);
            break;
        case (PARSE_STATE_RTCM_DATA):
            rtcmReadData(&parse, incoming);
            break;
        case (PARSE_STATE_RTCM_CRC):
            rtcmReadCrc(&parse, incoming);
            break;
        }
        return (true);
    }
    return (false);
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
    snprintf(command, sizeof(command), "CONFIG %s %d", comName, newBaud);

    return (sendCommand(command));
}

// Sets the baud rate of the port we are communicating on
// Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
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

//Abstraction of the serial interface
//Useful if we ever need to support SoftwareSerial
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


//Query and send functionality
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
// #MODE,97,GPS,FINE,2283,499142000,0,0,18,22;MODE BASE -1280206.5680 -4716804.4030 4086665.4840,*60

//'$' begins the responses to commands, ie 'MODE ROVER', ends with OK
// Contains reponse for caller
Um980Result UM980::sendQuery(const char *command, char *response, uint16_t *maxResponseLength, uint16_t maxWaitMs)
{
    Um980Result result;

    // Serial.println("Sending query");

    clearBuffer();

    // Send command and check COMMAND OK response
    result = sendString(command, maxWaitMs);
    if (result != UM980_RESULT_OK)
        return (result);

    parse.length = 0; // Reset parser
    strncpy(commandName, command, sizeof(commandName));
    commandResponse = UM980_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    // Serial.printf("commandName1: %s\r\n", commandName);

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            // Serial.println("sendQuery timeout");
            return (UM980_RESULT_TIMEOUT_RESPONSE);
        }

        updateOnce(); // Will call eomHandler()

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_OK)
            break;

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        {
            // Serial.println("sendQuery command error");
            return (UM980_RESULT_RESPONSE_COMMAND_ERROR);
        }

        delay(1);
    }

    // debugPrintf("Found OK to command");

    return (UM980_RESULT_OK);
}

Um980Result UM980::sendQuery(const char *command, char *response, int *maxResponseLength, uint16_t maxWaitMs)
{
    return (sendQuery(command, response, (uint16_t *)maxResponseLength, maxWaitMs));
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

    // Serial.println("Sending string");

    parse.length = 0; // Reset parser
    strncpy(commandName, command, sizeof(commandName));
    commandResponse = UM980_RESULT_RESPONSE_COMMAND_WAITING; // Reset

    serialPrintln(command);

    // Feed the parser until we see a response to the command
    int wait = 0;
    while (1)
    {
        if (wait++ == maxWaitMs)
        {
            // Serial.println("sednString Timeout");
            return (UM980_RESULT_TIMEOUT_RESPONSE);
        }

        updateOnce(); // Will call eomHandler()

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_OK)
        {
            // Serial.println("sendstring command ok");
            break;
        }

        if (commandResponse == UM980_RESULT_RESPONSE_COMMAND_ERROR)
        {
            // Serial.println("sendstring Command error");
            return (UM980_RESULT_RESPONSE_COMMAND_ERROR);
        }

        delay(1);
    }

    return (UM980_RESULT_OK);
}

// Scans a response for the * terminator character
// Assumes response is null terminated
// Used for visible CRC (for example VERSION query)
// Ex:
// #VERSION,97,GPS,FINE,2282,248561000,0,0,18,676;UM980,R4.10Build7923,HRPT00-S10C-P,2310415000001-MD22B1224962616,ff3bac96f31f9bdd,2022/09/28*7432d4ed
// CRC is calculated without the # or * characters
// Returns OK if valid CRC
Um980Result UM980::checkCRC(char *response)
{
    // Begin CRC
    uint32_t calculatedCRC = 0;
    uint32_t sentenceCRC = 0;
    int packetLength = 0;

    for (packetLength = 1; packetLength < strlen(response); packetLength++) // Remove # from CRC calculation
    {
        if (response[packetLength] == '*')
        {
            // Get CRC then break
            if (strlen(response) - packetLength > 8)
            {
                char hexString[8 + 1] = {0}; // Need spot for terminator
                hexString[0] = response[packetLength + 1];
                hexString[1] = response[packetLength + 2];
                hexString[2] = response[packetLength + 3];
                hexString[3] = response[packetLength + 4];
                hexString[4] = response[packetLength + 5];
                hexString[5] = response[packetLength + 6];
                hexString[6] = response[packetLength + 7];
                hexString[7] = response[packetLength + 8];
                sentenceCRC = strtoul(hexString, NULL, 16); // Unsigned Long variant of strtol
            }
            break; // Exclude * from CRC
        }

        calculatedCRC = crcTable[(calculatedCRC ^ response[packetLength]) & 0xFF] ^ (calculatedCRC >> 8);
    }

    if (packetLength == strlen(response))
        return (UM980_RESULT_MISSING_CRC);

    if (sentenceCRC != calculatedCRC)
    {
        debugPrintf("CRC failed. Sentence CRC: 0x%02X Calculated CRC: 0x%02X", sentenceCRC, calculatedCRC);
        return (UM980_RESULT_BAD_CRC);
    }

    return (UM980_RESULT_OK);
}

//Main Unicore handler and RAM inits
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

// Cracks a given binary message into the applicable container
void UM980::unicoreHandler(uint8_t *response, uint16_t length)
{
    uint16_t messageID = ((uint16_t)response[offsetHeaderMessageId + 1] << 8) | response[offsetHeaderMessageId];

    if (messageID == messageIdBestnav)
    {
        CHECK_POINTER_VOID(packetBESTNAV, initBestnav); // Check that RAM has been allocated

        lastUpdateGeodetic = millis(); // Update stale marker

        uint8_t *data = &response[um980HeaderLength]; // Point at the start of the data fields

        // Move data into given containers

        // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace
        memcpy(&packetBESTNAV->data.solutionStatus, &data[offsetBestnavPsolStatus], sizeof(uint8_t));

        // // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
        memcpy(&packetBESTNAV->data.positionType, &data[offsetBestnavPosType], sizeof(uint8_t));

        memcpy(&packetBESTNAV->data.latitude, &data[offsetBestnavLat], sizeof(double));
        memcpy(&packetBESTNAV->data.longitude, &data[offsetBestnavLon], sizeof(double));
        memcpy(&packetBESTNAV->data.altitude, &data[offsetBestnavHgt], sizeof(double));
        memcpy(&packetBESTNAV->data.latitudeDeviation, &data[offsetBestnavLatDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.longitudeDeviation, &data[offsetBestnavLonDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.heightDeviation, &data[offsetBestnavHgtDeviation], sizeof(float));
        memcpy(&packetBESTNAV->data.satellitesTracked, &data[offsetBestnavSatsTracked], sizeof(uint8_t));
        memcpy(&packetBESTNAV->data.satellitesUsed, &data[offsetBestnavSatsUsed], sizeof(uint8_t));

        uint8_t extSolStat;
        memcpy(&extSolStat, &data[offsetBestnavExtSolStat], sizeof(uint8_t));
        packetBESTNAV->data.rtkSolution = extSolStat & 0x01; // 0 = checked, 1 = unchecked
        packetBESTNAV->data.pseudorangeCorrection = extSolStat >> 1;
    }
    else if (messageID == messageIdRectime)
    {
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
}

// Allocate RAM for packetBESTNAV and initialize it
bool UM980::initBestnav(uint8_t rate)
{
    packetBESTNAV = new UNICORE_BESTNAV_t; // Allocate RAM for the main struct
    if (packetBESTNAV == nullptr)
    {
        Serial.println("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAV->callbackPointerPtr = nullptr;
    //   packetBESTNAV->callbackData = nullptr;

    // Start outputting BESTNAV in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVB %d", rate);
    if (sendCommand(command) == false)
        return (false);

    // Wait until first report is available
    lastUpdateGeodetic = 0;
    while (1)
    {
        update(); // Call parser
        if (lastUpdateGeodetic > 0)
            break;
    }
    return (true);
}

// Allocate RAM for packetBESTNAVXYZ and initialize it
bool UM980::initBestnavXyz(uint8_t rate)
{
    packetBESTNAVXYZ = new UNICORE_BESTNAVXYZ_t; // Allocate RAM for the main struct
    if (packetBESTNAVXYZ == nullptr)
    {
        Serial.println("Pointer alloc fail");
        return (false);
    }
    //   packetBESTNAVXYZ->callbackPointerPtr = nullptr;
    //   packetBESTNAVXYZ->callbackData = nullptr;

    // Start outputting BESTNAVXYZ in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "BESTNAVXYZB %d", rate);
    if (sendCommand(command) == false)
        return (false);

    // Wait until first report is available
    lastUpdateEcef = 0;
    while (1)
    {
        update(); // Call parser
        if (lastUpdateEcef > 0)
            break;
    }
    return (true);
}

// Allocate RAM for packetRECTIME and initialize it
bool UM980::initRectime(uint8_t rate)
{
    packetRECTIME = new UNICORE_RECTIME_t; // Allocate RAM for the main struct
    if (packetRECTIME == nullptr)
    {
        Serial.println("Pointer alloc fail");
        return (false);
    }
    //   packetRECTIME->callbackPointerPtr = nullptr;
    //   packetRECTIME->callbackData = nullptr;

    // Start outputting RECTIME in Binary on this COM port
    char command[50];
    snprintf(command, sizeof(command), "RECTIMEB %d", rate);
    if (sendCommand(command) == false)
        return (false);

    // Wait until first report is available
    lastUpdateDateTime = 0;
    while (1)
    {
        update(); // Call parser
        if (lastUpdateDateTime > 0)
            break;
    }
    return (true);
}

//All the general gets and sets
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

uint8_t UM980::getSIV()
{
    return (getSatellitesTracked());
}
uint8_t UM980::getSatellitesUsed()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.satellitesUsed);
}
uint8_t UM980::getSatellitesTracked()
{
    CHECK_POINTER_BOOL(packetBESTNAV, initBestnav); // Check that RAM has been allocated
    return (packetBESTNAV->data.satellitesTracked);
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
