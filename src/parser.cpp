#include "parser.h"
#include "Arduino.h"

extern UM980 *ptrUM980; // Global pointer for external parser access into library class

// End of message handler
// If it's a response to a command, is it OK or BAD? 
// Ex: $command,badResponse,response: PARSING FAILD NO MATCHING FUNC BADRESPONSE*40
// If it's Unicore binary, load into target variables 
// If it's NMEA or RTCM, discard
void um980EomHandler(UNICORE_PARSE_STATE *parse)
{
    // Switch on message type (NMEA, RTCM, Unicore Binary, etc)

    if (parse->messageType == SFE_SENTENCE_TYPE_UNICORE_COMMAND_RESPONSE)
    {
        // Does this response contain the command we are looking for?
        // It may be anywhere in the response:
        // $command,MODE,response: OK*5D
        char *responsePointer = strcasestr((char *)parse->buffer, ptrUM980->commandName);
        if (responsePointer != nullptr) // Found
        {
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
                ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_ERROR;
                return;
            }
        }
    }
    else if (parse->messageType == SFE_SENTENCE_TYPE_UNICORE_POUND_RESPONSE)
    {
        // Does this response contain the command we are looking for?
        if (strcasecmp((char *)parse->nmeaMessageName, ptrUM980->commandName) == 0) // Found
            ptrUM980->commandResponse = UM980_RESULT_RESPONSE_COMMAND_OK;
    }
    else if (parse->messageType == SFE_SENTENCE_TYPE_NMEA)
    {
        // NMEA Handler
    }
    else if (parse->messageType == SFE_SENTENCE_TYPE_UNICORE_BINARY)
    {
        ptrUM980->unicoreHandler(parse->buffer, parse->length);
    }
    else if (parse->messageType == SFE_SENTENCE_TYPE_RTCM)
    {
        // RTCM handler
    }
}

void um980WaitForPreamble(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Verify that this is the preamble byte
    switch (data)
    {
    case '$':

        //
        //    NMEA Message
        //
        //    +----------+---------+--------+---------+----------+----------+
        //    | Preamble |  Name   | Comma  |  Data   | Asterisk | Checksum |
        //    |  8 bits  | n bytes | 8 bits | n bytes |  8 bits  | 2 bytes  |
        //    |     $    |         |    ,   |         |          |          |
        //    +----------+---------+--------+---------+----------+----------+
        //               |                            |
        //               |<-------- Checksum -------->|
        //

    case '#':

        //
        //    Unicore response to a query command (MODE, etc)
        //    #MODE,97,GPS,FINE,2283,499142000,0,0,18,22;MODE BASE -1280206.5680 -4716804.4030 4086665.4840,*60
        //    Use the same NMEA parser

        parse->check = 0;
        parse->nmeaMessageNameLength = 0;
        parse->state = UNICORE_PARSE_STATE_NMEA_FIRST_COMMA;
        return;

    case 0xD3:

        //
        //    RTCM Standard 10403.2 - Chapter 4, Transport Layer
        //
        //    |<------------- 3 bytes ------------>|<----- length ----->|<- 3 bytes ->|
        //    |                                    |                    |             |
        //    +----------+--------+----------------+---------+----------+-------------+
        //    | Preamble |  Fill  | Message Length | Message |   Fill   |   CRC-24Q   |
        //    |  8 bits  | 6 bits |    10 bits     |  n-bits | 0-7 bits |   24 bits   |
        //    |   0xd3   | 000000 |   (in bytes)   |         |   zeros  |             |
        //    +----------+--------+----------------+---------+----------+-------------+
        //    |                                                         |
        //    |<------------------------ CRC -------------------------->|
        //

        // Start the CRC with this byte
        parse->state = UNICORE_PARSE_STATE_RTCM_LENGTH1;
        return;

    case 0xAA:

        //
        //    Unicore Binary Response
        //
        //    |<----- 24 byte header ------>|<--- length --->|<- 4 bytes ->|
        //    |                             |                |             |
        //    +------------+----------------+----------------+-------------+
        //    |  Preamble  | See table 7-48 |      Data      |    CRC      |
        //    |  3 bytes   |   21 bytes     |    n bytes     |   32 bits   |
        //    | 0xAA 44 B5 |                |                |             |
        //    +------------+----------------+----------------+-------------+
        //    |                                              |
        //    |<------------------------ CRC --------------->|
        //

        parse->state = UNICORE_PARSE_STATE_UNICORE_SYNC2;
        return;
    }

    // Preamble byte not found
    parse->length = 0;
    parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE;
}

// Read the message name
void um980NmeaFindFirstComma(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->check ^= data;
    if ((data != ',') || (parse->nmeaMessageNameLength == 0))
    {
        // UM980 respondes ("$command,CONFIG,response: OK*54") are very similar to NMEA
        // responses.

        // Save the message name
        parse->nmeaMessageName[parse->nmeaMessageNameLength++] = data;
    }
    else
    {
        // Zero terminate the message name
        parse->nmeaMessageName[parse->nmeaMessageNameLength++] = 0;
        parse->state = UNICORE_PARSE_STATE_NMEA_FIND_ASTERISK; // Move to next state
    }
}

// Read the message data
void um980NmeaFindAsterisk(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    if (data != '*')
        parse->check ^= data;
    else
    {
        // Determine if we need to capture 2 bytes (checksum) or 8 bytes (CRC)
        char *responseType = strcasestr((char *)parse->nmeaMessageName, "VERSION");
        if (responseType != nullptr) // Found
        {
            parse->state = UNICORE_PARSE_STATE_UNICORE_CRC; // VERSION uses 8 byte CRC
            parse->bytesRemaining = 8;
        }
        else
            parse->state = UNICORE_PARSE_STATE_NMEA_CHECKSUM1; // NMEA and MODE use 2 bytes
    }
}

// Read the first checksum byte
void um980UnicoreCRC(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->bytesRemaining -= 1; // Account for a byte received

    if (parse->bytesRemaining == 0)
    {
        parse->bytesRemaining = 2;
        parse->state = UNICORE_PARSE_STATE_NMEA_TERMINATION; // Move to next state
    }
}

// Read the first checksum byte
void um980NmeaChecksumByte1(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->state = UNICORE_PARSE_STATE_NMEA_CHECKSUM2; // Move to next state
}

// Read the second checksum byte
void um980NmeaChecksumByte2(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->nmeaLength = parse->length;

    parse->bytesRemaining = 2;

    parse->state = UNICORE_PARSE_STATE_NMEA_TERMINATION; // Move to next state
}

// Read the line termination
void um980NmeaLineTermination(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // We expect a \r\n termination, but may vary between receiver types
    if (data == '\r' || data == '\n')
    {
        parse->bytesRemaining -= 1; // Account for a byte received
    }

    // If we receive something other than a terminator, or if we've hit 0 bytes remaining, process sentence
    if (((data != '\r') && (data != '\n')) || (parse->bytesRemaining == 0))
    {
        // If it's not a terminator, remove this character from the buffer
        if ((data != '\r') && (data != '\n'))
            parse->length--;

        // Convert the checksum characters into binary
        int checksum = um980AsciiToNibble(parse->buffer[parse->nmeaLength - 2]) << 4;
        checksum |= um980AsciiToNibble(parse->buffer[parse->nmeaLength - 1]);

        parse->messageType = SFE_SENTENCE_TYPE_NMEA;

        // $GPGGA - NMEA prefixed with $ and the checksum is complete

        //$command,mode,response: OK*5D
        //$CONFIG,MASK,MASK 5.0*25
        //$CONFIG,ANTENNA,CONFIG ANTENNA POWERON*7A
        // For command, CONFIG, and MASK, the data will be prefixed with a $, and needs $ added to checksum

        // #MODE,97,GPS,FINE,2283,499142000,0,0,18,22;MODE BASE -1280206.5680 -4716804.4030 4086665.4840,*60
        // For MODE and any Unicore command, the data will be prefixed with a #, and needs # added to checksum

        // #VERSION,97,GPS,FINE,2282,248561000,0,0,18,676;UM980,R4.10Build7923,HRPT00-S10C-P,2310415000001-MD22B1224962616,ff3bac96f31f9bdd,2022/09/28*7432d4ed
        // For VERSION, the data will be prefixed with a #, uses a 32-bit CRC

        // Serial.println("Buffer:");
        // for (int x = 0; x < parse->length; x++)
        //     Serial.printf("%c", parse->buffer[x]);
        // Serial.println();

        // Handle CRC for command response with a leading #
        if (parse->buffer[0] == '#')
        {
            parse->messageType = SFE_SENTENCE_TYPE_UNICORE_POUND_RESPONSE;

            // Determine if this response type uses checksum or CRC
            char *responseType = strcasestr((char *)parse->nmeaMessageName, "VERSION");
            if (responseType != nullptr) // Found
            {
                if (ptrUM980->checkCRC((char *)parse->buffer) == UM980_RESULT_OK)
                {
                    checksum = parse->check; // Mark message as valid
                }
            }
            else
            {
                // Must be MODE response
                // #MODE,97,GPS,FINE,2283,499142000,0,0,18,22;MODE BASE -1280206.5680 -4716804.4030 4086665.4840,*60
                parse->check ^= '#';
            }
        }
        else
        {
            // Sentence starts with '$'

            // NMEA checksum excludes the $ and *
            // Unicore includes the $, excludes the *
            // If this is a Unicore response, we need to add a $ to the checksum
            //$command,MODE,response: OK*5D
            if (strcasecmp((char *)parse->nmeaMessageName, "command") == 0 ||
                strcasecmp((char *)parse->nmeaMessageName, "MASK") == 0 ||
                strcasecmp((char *)parse->nmeaMessageName, "CONFIG") == 0) // Found
            {
                parse->check ^= '$';
                parse->messageType = SFE_SENTENCE_TYPE_UNICORE_COMMAND_RESPONSE;
            }
        }

        // Validate the checksum
        if (checksum == parse->check)
            parse->check = 0;

        // Process this message if CRC is valid
        if (parse->check == 0)
        {
            // CRC is valid

            // Return immediately if we are expecting a command
            if (ptrUM980->commandResponse == UM980_RESULT_RESPONSE_COMMAND_WAITING)
            {
                um980EomHandler(parse);

                parse->length = 0;
                parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE; // Move to next state
                return;
            }

            // Return immediately if we want the full buffer
            if (ptrUM980->commandResponse == UM980_RESULT_OK)
            {
                um980EomHandler(parse);
                return; //Do not erase the buffer length
            }

            // Otherwise, continue parsing after handler
            um980EomHandler(parse);
        }
        else
        {
            if (parse->messageType == SFE_SENTENCE_TYPE_NMEA)
            {
                ptrUM980->debugPrintf("Bad CRC on NMEA.");
                ptrUM980->debugPrintf("Expecting: 0x%02X Found: 0x%02X.", checksum, parse->check);
            }
            else if (parse->messageType == SFE_SENTENCE_TYPE_UNICORE_POUND_RESPONSE)
            {
                ptrUM980->debugPrintf("Bad CRC on Unicore # Response.");
                ptrUM980->debugPrintf("Expecting: 0x%02X Found: 0x%02X.", checksum, parse->check);
            }
            else if (parse->messageType == SFE_SENTENCE_TYPE_UNICORE_COMMAND_RESPONSE)
            {
                ptrUM980->debugPrintf("Bad CRC on Unicore Command.");
                ptrUM980->debugPrintf("Expecting: 0x%08X Found: 0x%08X.", checksum, parse->check);
            }

            parse->messageType = SFE_SENTENCE_TYPE_NONE;
        }

        if ((data != '\r') && (data != '\n'))
        {
            // We already have new data, process it immediately
            parse->length = 0;
            parse->buffer[parse->length++] = data;
            parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE; // Move to next state
            return (um980WaitForPreamble(parse, data));
        }
        else
        {
            // We're done. Move on to waiting.
            parse->length = 0;
            parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE; // Move to next state
        }
    }
}

// Convert nibble to ASCII
int um980AsciiToNibble(int data)
{
    // Convert the value to lower case
    data |= 0x20;
    if ((data >= 'a') && (data <= 'f'))
        return data - 'a' + 10;
    if ((data >= '0') && (data <= '9'))
        return data - '0';
    return -1;
}

// Read the second sync byte
void um980UnicoreBinarySync2(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 2
    if (data != 0x44)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (um980WaitForPreamble(parse, data)); // Start searching for a preamble byte
    }

    parse->state = UNICORE_PARSE_STATE_UNICORE_SYNC3; // Move on
}

// Read the third sync byte
void um980UnicoreBinarySync3(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Verify sync byte 3
    if (data != 0xB5)
    {
        // Invalid sync byte, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;
        return (um980WaitForPreamble(parse, data)); // Start searching for a preamble byte
    }

    parse->state = UNICORE_PARSE_STATE_UNICORE_READ_LENGTH; // Move on
}

// Read the message length
void um980UnicoreBinaryReadLength(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    if (parse->length == offsetHeaderMessageLength + 2)
    {
        // Get the length
        uint16_t expectedLength =
            (parse->buffer[offsetHeaderMessageLength + 1] << 8) | parse->buffer[offsetHeaderMessageLength];

        // The overall message length is header (24) + data (expectedLength) + CRC (4)
        parse->bytesRemaining = um980HeaderLength + expectedLength + 4;

        if (parse->bytesRemaining > UNICORE_PARSE_BUFFER_LENGTH)
        {
            ptrUM980->debugPrintf("Length overflow");

            // Invalid length, place this byte at the beginning of the buffer
            parse->length = 0;
            parse->buffer[parse->length++] = data;

            // Start searching for a preamble byte
            return um980WaitForPreamble(parse, data);
        }

        // Account for the bytes already received
        parse->bytesRemaining -= parse->length;
        parse->state = UNICORE_PARSE_STATE_UNICORE_READ_DATA; // Move on
    }
}

// Read the message content until we reach the end, then check CRC
void um980UnicoreReadData(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Account for this data byte
    parse->bytesRemaining -= 1;

    // Wait until all the data is received, including the 4 byte CRC
    if (parse->bytesRemaining > 0)
        return;

    // We have all the data including the CRC

    // Check the CRC
    uint32_t sentenceCRC = ((uint32_t)parse->buffer[parse->length - 4] << (8 * 0)) |
                           ((uint32_t)parse->buffer[parse->length - 3] << (8 * 1)) |
                           ((uint32_t)parse->buffer[parse->length - 2] << (8 * 2)) |
                           ((uint32_t)parse->buffer[parse->length - 1] << (8 * 3));
    uint32_t calculatedCRC =
        um980CalculateCRC32(parse->buffer, parse->length - 4); // CRC is calculated on entire messsage, sans CRC

    // Process this message if CRC is valid
    if (sentenceCRC == calculatedCRC)
    {
        // Message complete, CRC is valid
        parse->messageType = SFE_SENTENCE_TYPE_UNICORE_BINARY;
        um980EomHandler(parse);
    }
    else
    {
        ptrUM980->debugPrintf("Unicore CRC failed length: %d Sentence CRC: 0x%02X Calculated CRC: 0x%02X\r\n",
                              parse->length, sentenceCRC, calculatedCRC);
    }

    // Search for another preamble byte
    parse->length = 0;
    parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE; // Move on
}

// Calculate and return the CRC of the given buffer
uint32_t um980CalculateCRC32(uint8_t *charBuffer, uint16_t bufferSize)
{
    uint32_t crc = 0;
    for (uint16_t x = 0; x < bufferSize; x++)
        crc = crc32Table[(crc ^ charBuffer[x]) & 0xFF] ^ (crc >> 8);
    return crc;
}

// Read the upper two bits of the length
void um980RtcmReadLength1(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Verify the length byte - check the 6 MS bits are all zero
    if (data & (~3))
    {
        // Invalid length, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;

        // Start searching for a preamble byte
        return um980WaitForPreamble(parse, data);
    }

    // Save the upper 2 bits of the length
    parse->bytesRemaining = data << 8;
    parse->state = UNICORE_PARSE_STATE_RTCM_LENGTH2;
}

// Read the lower 8 bits of the length
void um980RtcmReadLength2(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->bytesRemaining |= data;

    // Check the length
    if (parse->bytesRemaining > UNICORE_PARSE_BUFFER_LENGTH)
    {
        ptrUM980->debugPrintf("RTCM length overflow");

        // Invalid length, place this byte at the beginning of the buffer
        parse->length = 0;
        parse->buffer[parse->length++] = data;

        // Start searching for a preamble byte
        return um980WaitForPreamble(parse, data);
    }

    parse->state = UNICORE_PARSE_STATE_RTCM_MESSAGE1;
}

// Read the upper 8 bits of the message number
void um980RtcmReadMessage1(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->bytesRemaining -= 1;
    parse->state = UNICORE_PARSE_STATE_RTCM_MESSAGE2;
}

// Read the lower 4 bits of the message number
void um980RtcmReadMessage2(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    parse->bytesRemaining -= 1;
    parse->state = UNICORE_PARSE_STATE_RTCM_DATA;
}

// Read the rest of the message
void um980RtcmReadData(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Account for this data byte
    parse->bytesRemaining -= 1;

    // Wait until all the data is received
    if (parse->bytesRemaining <= 0)
    {
        parse->bytesRemaining = 3;
        parse->state = UNICORE_PARSE_STATE_RTCM_CRC;
    }
}

// Read the CRC
void um980RtcmReadCrc(UNICORE_PARSE_STATE *parse, uint8_t data)
{
    // Account for this data byte
    parse->bytesRemaining -= 1;

    // Wait until all the data is received
    if (parse->bytesRemaining > 0)
        return;

    // Get CRC
    uint32_t sentenceCRC = (parse->buffer[parse->length - 3] << 16) | (parse->buffer[parse->length - 2] << 8) |
                           (parse->buffer[parse->length - 1] << 0);

    // Calculate CRC
    parse->check = 0;
    for (int x = 0; x < parse->length - 3; x++) // Exclude CRC
    {
        parse->check = ((parse)->check << 8) ^ crc24qTable[parse->buffer[x] ^ (((parse)->check >> 16) & 0xff)];
    }
    parse->check &= 0x00ffffff;

    // Process the message if CRC is valid
    if (parse->check == sentenceCRC)
    {
        parse->messageType = SFE_SENTENCE_TYPE_RTCM;
        um980EomHandler(parse);
    }
    else
    {
        ptrUM980->debugPrintf("RTCM CRC failed length: %d sentence CRC: 0x%02X calc CRC: 0x%02X\r\n", parse->length,
                              sentenceCRC, parse->check);
    }

    // Search for another preamble byte
    parse->length = 0;
    parse->state = UNICORE_PARSE_STATE_WAITING_FOR_PREAMBLE;
}