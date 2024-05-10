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

#ifndef _SPARKFUN_UNICORE_GNSS_ARDUINO_LIBRARY_H
#define _SPARKFUN_UNICORE_GNSS_ARDUINO_LIBRARY_H

#include "Arduino.h"

#if __has_include("SoftwareSerial.h")
#include <SoftwareSerial.h>
#endif

#include "unicore_structs.h"
#include <SparkFun_Extensible_Message_Parser.h> //http://librarymanager/All#SparkFun_Extensible_Message_Parser

typedef enum
{
    UM980_RESULT_OK = 0,
    UM980_RESULT_TIMEOUT_START_BYTE,
    UM980_RESULT_TIMEOUT_DATA_BYTE,
    UM980_RESULT_TIMEOUT_END_BYTE,
    UM980_RESULT_TIMEOUT_RESPONSE,
    UM980_RESULT_WRONG_COMMAND,
    UM980_RESULT_WRONG_MESSAGE_ID,
    UM980_RESULT_BAD_START_BYTE,
    UM980_RESULT_BAD_CHECKSUM,
    UM980_RESULT_BAD_CRC,
    UM980_RESULT_MISSING_CRC,
    UM980_RESULT_TIMEOUT,
    UM980_RESULT_RESPONSE_OVERFLOW,
    UM980_RESULT_RESPONSE_COMMAND_OK,
    UM980_RESULT_RESPONSE_COMMAND_ERROR,
    UM980_RESULT_RESPONSE_COMMAND_WAITING,
    UM980_RESULT_RESPONSE_COMMAND_CONFIG,
    UM980_RESULT_CONFIG_PRESENT,
} Um980Result;

#define um980BinarySyncA ((uint8_t)0xAA)
#define um980BinarySyncB ((uint8_t)0x44)
#define um980BinarySyncC ((uint8_t)0xB5)
#define um980ASCIISyncEnd ((uint8_t)'\n')

#define um980HeaderLength ((uint16_t)24)
#define offsetHeaderSyncA ((uint16_t)0)
#define offsetHeaderSyncB ((uint16_t)1)
#define offsetHeaderSyncC ((uint16_t)2)
#define offsetHeaderCpuIdle ((uint16_t)3)
#define offsetHeaderMessageId ((uint16_t)4)
#define offsetHeaderMessageLength ((uint16_t)6)
#define offsetHeaderReferenceTime ((uint16_t)8)
#define offsetHeaderTimeStatus ((uint16_t)9)
#define offsetHeaderWeekNumber ((uint16_t)10)
#define offsetHeaderSecondsOfWeek ((uint16_t)12)
#define offsetHeaderReleaseVersion ((uint16_t)20)
#define offsetHeaderLeapSecond ((uint16_t)21)
#define offsetHeaderOutputDelay ((uint16_t)22)

// VERSIONB
#define messageIdVersion ((uint16_t)37)
#define offsetVersionModuleType ((uint16_t)0)
#define offsetVersionFirmwareVersion ((uint16_t)4)
#define offsetVersionAuth ((uint16_t)37)
#define offsetVersionPsn ((uint16_t)166)
#define offsetVersionEfuseID ((uint16_t)232)
#define offsetVersionCompTime ((uint16_t)265)

// BESTNAVB contains HPA, sats tracked/used, lat/long, RTK status, fix status
#define messageIdBestnav ((uint16_t)2118)
#define offsetBestnavPsolStatus ((uint16_t)0)
#define offsetBestnavPosType ((uint16_t)4)
#define offsetBestnavLat ((uint16_t)8)
#define offsetBestnavLon ((uint16_t)16)
#define offsetBestnavHgt ((uint16_t)24)
#define offsetBestnavLatDeviation ((uint16_t)40)
#define offsetBestnavLonDeviation ((uint16_t)44)
#define offsetBestnavHgtDeviation ((uint16_t)48)
#define offsetBestnavSatsTracked ((uint16_t)64)
#define offsetBestnavSatsUsed ((uint16_t)65)
#define offsetBestnavExtSolStat ((uint16_t)69)
#define offsetBestnavVelType ((uint16_t)76)
#define offsetBestnavHorSpd ((uint16_t)88)
#define offsetBestnavTrkGnd ((uint16_t)96)
#define offsetBestnavVertSpd ((uint16_t)104)
#define offsetBestnavVerspdStd ((uint16_t)112)
#define offsetBestnavHorspdStd ((uint16_t)116)

// BESTNAVXYZB
#define messageIdBestnavXyz ((uint16_t)240)
#define offsetBestnavXyzPsolStatus ((uint16_t)0)
#define offsetBestnavXyzPosType ((uint16_t)4)
#define offsetBestnavXyzPX ((uint16_t)8)
#define offsetBestnavXyzPY ((uint16_t)16)
#define offsetBestnavXyzPZ ((uint16_t)24)
#define offsetBestnavXyzPXDeviation ((uint16_t)32)
#define offsetBestnavXyzPYDeviation ((uint16_t)36)
#define offsetBestnavXyzPZDeviation ((uint16_t)40)
#define offsetBestnavXyzSatsTracked ((uint16_t)104)
#define offsetBestnavXyzSatsUsed ((uint16_t)105)
#define offsetBestnavXyzExtSolStat ((uint16_t)109)

// RECTIMEB for time/date
#define messageIdRectime ((uint16_t)102)
#define offsetRectimeClockStatus ((uint16_t)0)
#define offsetRectimeOffset ((uint16_t)4)
#define offsetRectimeOffsetStd ((uint16_t)12)
#define offsetRectimeUtcYear ((uint16_t)28)
#define offsetRectimeUtcMonth ((uint16_t)32)
#define offsetRectimeUtcDay ((uint16_t)33)
#define offsetRectimeUtcHour ((uint16_t)34)
#define offsetRectimeUtcMinute ((uint16_t)35)
#define offsetRectimeUtcMillisecond ((uint16_t)36)
#define offsetRectimeUtcStatus ((uint16_t)40)

// HWSTATUS has temperature info, and voltage info

void um980ProcessMessage(SEMP_PARSE_STATE *parse, uint16_t type);

class UM980
{
  private:
    const uint16_t dataFreshLimit_ms = 2000;
    unsigned long lastUpdateGeodetic = 0;
    unsigned long lastUpdateEcef = 0;
    unsigned long lastUpdateDateTime = 0;
    unsigned long lastUpdateVersion = 0;

    bool isNmeaFixed(); // Returns true when GNGGA NMEA reports position status >= 1

    void stopAutoReports(); // Delete all pointers to force reinit next time a helper function is called

    Um980Result getGeodetic(uint16_t maxWaitMs = 1500);
    Um980Result updateEcef(uint16_t maxWaitMs = 1500);
    Um980Result updateDateTime(uint16_t maxWaitMs = 1500);

    Print *_debugPort = nullptr; // The stream to send debug messages to if enabled. Usually Serial.

    SEMP_PARSE_STATE *_sempParse; // State of the SparkFun Extensible Message Parser

    bool unicoreLibrarySemaphoreBlock = false; // Gets set to true when the Unicore library needs to interact directly
                                               // with the serial hardware
    char configStringToFind[100] = {'\0'};
    bool configStringFound = false; // configHandler() sets true if we find the intended string

  protected:
    HardwareSerial *_hwSerialPort = nullptr;

  public:
    bool _printBadChecksum = false;       // Display bad checksum message from the parser
    bool _printParserTransitions = false; // Display the parser transitions
    bool _printRxMessages = false;        // Display the received message summary
    bool _dumpRxMessages = false;         // Display the received message hex dump

    uint8_t nmeaPositionStatus = 0; // Position psition status obtained from GNGGA NMEA

    // By default, library will attempt to start RECTIME and BESTNAV regardless of GNSS fix.
    // This may lead to command timeouts as the UM980 does not appear to respond to BESTNAVB commands if 3D fix is not
    // achieved. Set startBinartBeforeFix = false via disableBinaryBeforeFix() to block binary commands before a fix is
    // achieved
    bool startBinaryBeforeFix = true;

    bool begin(HardwareSerial &serialPort, Print *parserDebug = nullptr, Print *parserError = &Serial);
    bool isConnected();
    bool isBlocking();
    bool update();
    bool updateOnce();

    void debugPrintf(const char *format, ...);
    void enableDebugging(Print &debugPort = Serial);
    void disableDebugging();

    void enableParserDebug(Print *print = &Serial);
    void disableParserDebug();
    void enableParserErrors(Print *print = &Serial);
    void disableParserErrors();

    void enableBinaryBeforeFix();
    void disableBinaryBeforeFix();

    void enablePrintBadChecksums();
    void disablePrintBadChecksums();
    void enablePrintParserTransitions();
    void disablePrintParserTransitions();
    void enablePrintRxMessages();
    void disablePrintRxMessages();
    void enableRxMessageDump();
    void disableRxMessageDump();
    void printParserConfiguration(Print *print = &Serial);

    void dumpBuffer(const uint8_t *buffer, uint16_t length);

    char commandName[50] = ""; // Passes the command type into parser - CONFIG PPS ENABLE GPS POSITIVE 200000 1000 0 0
    uint8_t commandResponse = UM980_RESULT_OK; // Gets EOM result from parser

    // Mode
    bool setMode(const char *modeType);
    bool setModeBase(const char *baseType);
    bool setModeBaseGeodetic(double latitude, double longitude, double altitude);
    bool setModeBaseECEF(double coordinateX, double coordinateY, double coordinateZ);
    bool setModeBaseAverage();
    bool setModeBaseAverage(uint16_t averageTime);
    bool setModeRover(const char *roverType);
    bool setModeRoverSurvey();
    bool setModeRoverUAV();
    bool setModeRoverAutomotive();
    bool setModeRoverMow();

    // Config
    bool setPortBaudrate(const char *comName, unsigned long newBaud);
    bool setBaudrate(unsigned long newBaud);
    bool enablePPS(uint32_t widthMicroseconds, uint16_t periodMilliseconds, bool positivePolarity = true,
                   int16_t rfDelay = 0, int16_t userDelay = 0);
    bool disablePPS();
    bool configurePPS(const char *configString);

    // Mask
    bool enableConstellation(const char *constellationName);
    bool disableConstellation(const char *constellationName);
    bool setElevationAngle(int16_t elevationDegrees, const char *constellationName);
    bool setElevationAngle(int16_t elevationDegrees);
    bool setMinCNO(uint8_t dBHz);
    bool enableFrequency(const char *frequencyName);
    bool disableFrequency(const char *frequencyName);
    bool enableSystem(const char *systemName);
    bool disableSystem(const char *systemName);

    // Data output
    bool setNMEAPortMessage(const char *sentenceType, const char *comName, float outputRate);
    bool setNMEAMessage(const char *sentenceType, float outputRate);
    bool setRTCMPortMessage(const char *sentenceType, const char *comName, float outputRate);
    bool setRTCMMessage(const char *sentenceType, float outputRate);

    // Other
    bool disableOutput();
    bool disableOutputPort(const char *comName);
    bool factoryReset();
    bool reset();
    bool saveConfiguration(uint16_t maxWaitMs = 1500);

    uint16_t serialAvailable();
    uint8_t serialRead();
    void serialPrintln(const char *command);
    void clearBuffer();

    bool sendCommand(const char *command, uint16_t maxWaitMs = 1500);
    Um980Result sendQuery(const char *command, uint16_t maxWaitMs = 1500);
    Um980Result sendString(const char *command, uint16_t maxWaitMs = 1500);

    // Main helper functions
    double getLatitude(uint16_t maxWaitMs = 1500);
    double getLongitude();
    double getAltitude();
    double getHorizontalSpeed();
    double getVerticalSpeed();
    double getTrackGround();

    float getLatitudeDeviation();
    float getLongitudeDeviation();
    float getAltitudeDeviation();
    float getHorizontalSpeedDeviation();
    float getVerticalSpeedDeviation();

    double getEcefX();
    double getEcefY();
    double getEcefZ();
    float getEcefXDeviation();
    float getEcefYDeviation();
    float getEcefZDeviation();

    uint8_t getSIV();
    uint8_t getSatellitesTracked();
    uint8_t getSatellitesUsed();
    uint8_t getSolutionStatus();
    uint8_t getPositionType();
    uint8_t getVelocityType();

    uint8_t getRTKSolution();
    uint8_t getPseudorangeCorrection();

    uint16_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    uint16_t getMillisecond();
    uint8_t getTimeStatus();
    uint8_t getDateStatus();
    double getTimeOffset();
    double getTimeOffsetDeviation();

    uint32_t getFixAgeMilliseconds(); // Based on Geodetic report

    uint8_t getModelType();
    char *getVersion();
    char *getID();
    char *getCompileTime();

    char *getVersionFull(uint16_t maxWaitMs = 1500);

    // Limit maxWaitMs for CONFIG interactions. 800ms good. 500ms too short.
    // because we rely on response timeout - there is no known end to the CONFIG response
    bool isConfigurationPresent(const char *configStringToFind, uint16_t maxWaitMs = 800);

    void unicoreHandler(uint8_t *data, uint16_t length);
    void configHandler(uint8_t *response, uint16_t length);

    bool initBestnav(uint8_t rate = 1);
    UNICORE_BESTNAV_t *packetBESTNAV = nullptr;

    bool initBestnavXyz(uint8_t rate = 1);
    UNICORE_BESTNAVXYZ_t *packetBESTNAVXYZ = nullptr;

    bool initRectime(uint8_t rate = 1);
    UNICORE_RECTIME_t *packetRECTIME = nullptr;

    bool initVersion();
    UNICORE_VERSION_t *packetVERSION = nullptr;
};

#endif //_SPARKFUN_UNICORE_GNSS_ARDUINO_LIBRARY_H
