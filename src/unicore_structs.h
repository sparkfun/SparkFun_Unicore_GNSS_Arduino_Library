#ifndef _SPARKFUN_UNICORE_STRUCTS_H
#define _SPARKFUN_UNICORE_STRUCTS_H

typedef struct
{

    double latitude;
    double longitude;
    double altitude;
    double horizontalSpeed;
    double verticalSpeed;
    double trackGround;

    float latitudeDeviation;
    float longitudeDeviation;
    float heightDeviation;
    float horizontalSpeedDeviation;
    float verticalSpeedDeviation;

    uint8_t positionType; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...
    uint8_t velocityType; // 0 = None, 1 = FixedPos, 8 = DopplerVelocity, 16 = Single, ...

    uint8_t
        solutionStatus; // 0 = Solution computed, 1 = Insufficient observation, 3 = No convergence, 4 = Covariance trace

    uint8_t satellitesTracked;
    uint8_t satellitesUsed;

    uint8_t rtkSolution = 0;
    uint8_t pseudorangeCorrection = 0;
} UNICORE_BESTNAV_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_BESTNAV_data_t data;
    void (*callbackPointerPtr)(UNICORE_BESTNAV_data_t *);
    UNICORE_BESTNAV_data_t *callbackData;
} UNICORE_BESTNAV_t;

typedef struct
{
    double ecefX = 0;
    double ecefY = 0;
    double ecefZ = 0;
    float ecefXDeviation = 0;
    float ecefYDeviation = 0;
    float ecefZDeviation = 0;
} UNICORE_BESTNAVXYZ_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_BESTNAVXYZ_data_t data;
    void (*callbackPointerPtr)(UNICORE_BESTNAVXYZ_data_t *);
    UNICORE_BESTNAVXYZ_data_t *callbackData;
} UNICORE_BESTNAVXYZ_t;

typedef struct
{
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    uint8_t timeStatus = 3; // 0 = valid, 3 = invalid
    uint8_t dateStatus = 0; // 0 = Invalid, 1 = valid, 2 = leap second warning
    double timeOffset = 0;
    double timeDeviation = 0;
} UNICORE_RECTIME_data_t;

typedef struct
{
    // ubxAutomaticFlags automaticFlags;
    UNICORE_RECTIME_data_t data;
    void (*callbackPointerPtr)(UNICORE_RECTIME_data_t *);
    UNICORE_RECTIME_data_t *callbackData;
} UNICORE_RECTIME_t;

#endif // _SPARKFUN_UNICORE_STRUCTS_H
