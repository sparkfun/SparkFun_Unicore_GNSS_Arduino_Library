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

#include "parser.h"
#include "unicore_structs.h"

// Default maximum NMEA byte count
// maxNMEAByteCount was set to 82: https://en.wikipedia.org/wiki/NMEA_0183#Message_structure
// but this is often violated.
// The user can adjust maxNMEAByteCount by calling setMaxNMEAByteCount
// To be safe, use 100
#define SFE_UM980_MAX_NMEA_BYTE_COUNT 100

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
} Um980Result;

const uint8_t um980BinarySyncA = 0xAA;
const uint8_t um980BinarySyncB = 0x44;
const uint8_t um980BinarySyncC = 0xB5;
const uint8_t um980ASCIISyncEnd = '\n';
const uint16_t um980HeaderLength = 24;

const uint16_t offsetHeaderSyncA = 0;
const uint16_t offsetHeaderSyncB = 1;
const uint16_t offsetHeaderSyncC = 2;
const uint16_t offsetHeaderCpuIdle = 3;
const uint16_t offsetHeaderMessageId = 4;
const uint16_t offsetHeaderMessageLength = 6;
const uint16_t offsetHeaderReferenceTime = 8;
const uint16_t offsetHeaderTimeStatus = 9;
const uint16_t offsetHeaderWeekNumber = 10;
const uint16_t offsetHeaderSecondsOfWeek = 12;
const uint16_t offsetHeaderReleaseVersion = 20;
const uint16_t offsetHeaderLeapSecond = 21;
const uint16_t offsetHeaderOutputDelay = 22;

// VERSIONB
const uint16_t messageIdVersion = 37;
const uint16_t offsetVersionModuleType = 0;
const uint16_t offsetVersionFirmwareVersion = 4;
const uint16_t offsetVersionAuth = 37;
const uint16_t offsetVersionPsn = 166;
const uint16_t offsetVersionEfuseID = 232;
const uint16_t offsetVersionCompTime = 265;

// BESTNAVB contains HPA, sats tracked/used, lat/long, RTK status, fix status
const uint16_t messageIdBestnav = 2118;
const uint16_t offsetBestnavPsolStatus = 0;
const uint16_t offsetBestnavPosType = 4;
const uint16_t offsetBestnavLat = 8;
const uint16_t offsetBestnavLon = 16;
const uint16_t offsetBestnavHgt = 24;
const uint16_t offsetBestnavLatDeviation = 40;
const uint16_t offsetBestnavLonDeviation = 44;
const uint16_t offsetBestnavHgtDeviation = 48;
const uint16_t offsetBestnavSatsTracked = 64;
const uint16_t offsetBestnavSatsUsed = 65;
const uint16_t offsetBestnavExtSolStat = 69;
const uint16_t offsetBestnavVelType = 76;
const uint16_t offsetBestnavHorSpd = 88;
const uint16_t offsetBestnavTrkGnd = 96;
const uint16_t offsetBestnavVertSpd = 104;
const uint16_t offsetBestnavVerspdStd = 112;
const uint16_t offsetBestnavHorspdStd = 116;

// BESTNAVXYZB
const uint16_t messageIdBestnavXyz = 240;
const uint16_t offsetBestnavXyzPsolStatus = 0;
const uint16_t offsetBestnavXyzPosType = 4;
const uint16_t offsetBestnavXyzPX = 8;
const uint16_t offsetBestnavXyzPY = 16;
const uint16_t offsetBestnavXyzPZ = 24;
const uint16_t offsetBestnavXyzPXDeviation = 32;
const uint16_t offsetBestnavXyzPYDeviation = 36;
const uint16_t offsetBestnavXyzPZDeviation = 40;
const uint16_t offsetBestnavXyzSatsTracked = 104;
const uint16_t offsetBestnavXyzSatsUsed = 105;
const uint16_t offsetBestnavXyzExtSolStat = 109;

// RECTIMEB for time/date
const uint16_t messageIdRectime = 102;
const uint16_t offsetRectimeClockStatus = 0;
const uint16_t offsetRectimeOffset = 4;
const uint16_t offsetRectimeOffsetStd = 12;
const uint16_t offsetRectimeUtcYear = 28;
const uint16_t offsetRectimeUtcMonth = 32;
const uint16_t offsetRectimeUtcDay = 33;
const uint16_t offsetRectimeUtcHour = 34;
const uint16_t offsetRectimeUtcMinute = 35;
const uint16_t offsetRectimeUtcMillisecond = 36;
const uint16_t offsetRectimeUtcStatus = 40;

// HWSTATUS has temperature info, and voltage info

class UM980
{
  private:
    const uint16_t dataFreshLimit_ms = 2000;
    unsigned long lastUpdateGeodetic = 0;
    unsigned long lastUpdateEcef = 0;
    unsigned long lastUpdateDateTime = 0;
    unsigned long lastUpdateVersion = 0;

    bool staleDateTime();
    bool staleEcef();
    void stopAutoReports(); // Delete all pointers to force reinit next time a helper function is called

    Um980Result getGeodetic(uint16_t maxWaitMs = 1500);
    Um980Result updateEcef(uint16_t maxWaitMs = 1500);
    Um980Result updateDateTime(uint16_t maxWaitMs = 1500);

    Print *_debugPort = nullptr; // The stream to send debug messages to if enabled. Usually Serial.

  protected:
    HardwareSerial *_hwSerialPort = nullptr;

  public:
    bool begin(HardwareSerial &serialPort);
    bool isConnected();
    bool update();
    bool updateOnce();

    void debugPrintf(const char *format, ...);
    void enableDebugging(Print &debugPort = Serial);
    void disableDebugging();

    char commandName[20] = "";                 // Passes the command type into parser
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
    bool saveConfiguration();

    uint16_t serialAvailable();
    uint8_t serialRead();
    void serialPrintln(const char *command);
    void clearBuffer();

    bool sendCommand(const char *command, uint16_t maxWaitMs = 1500);
    Um980Result sendQuery(const char *command, uint16_t maxWaitMs = 1500);
    Um980Result sendString(const char *command, uint16_t maxWaitMs = 1500);
    Um980Result checkCRC(char *response);

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

    void unicoreHandler(uint8_t *data, uint16_t length);

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
