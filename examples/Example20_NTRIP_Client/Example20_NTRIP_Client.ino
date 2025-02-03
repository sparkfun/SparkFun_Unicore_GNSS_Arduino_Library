/*
  Connect to NTRIP Caster to obtain corrections
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 31, 2025
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to connect to an NTRIP Caster and push RTCM to the UM980 to
  obtain an RTK Fix.

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

#include <WiFi.h>
#include "secrets.h"

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS
UM980 myGNSS;

#define pin_UART_TX     4
#define pin_UART_RX     13

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0;       //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

bool transmitLocation = true;        //By default we will transmit the units location via GGA sentence.
int timeBetweenGGAUpdate_ms = 10000; //GGA is required for Rev2 NTRIP casters. Don't transmit but once every 10 seconds
long lastTransmittedGGA_ms = 0;

//Used for GGA sentence parsing from incoming NMEA
bool ggaSentenceStarted = false;
bool ggaSentenceComplete = false;
bool ggaTransmitComplete = false; //Goes true once we transmit GGA to the caster

char ggaSentence[128] = {0};
byte ggaSentenceSpot = 0;
int ggaSentenceEndSpot = 0;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example");

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART_RX, pin_UART_TX);

  //myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates.");
    while (1);
  }
  Serial.println("UM980 detected!");

  myGNSS.disableOutput(); // Disables all messages on this port

  myGNSS.setModeRoverSurvey();

  //Enable the basic 5 NMEA sentences including GGA for the NTRIP Caster at 1Hz
  myGNSS.setNMEAPortMessage("GPGGA", 1);
  myGNSS.setNMEAPortMessage("GPGSA", 1);
  myGNSS.setNMEAPortMessage("GPGST", 1);
  myGNSS.setNMEAPortMessage("GPGSV", 1);
  myGNSS.setNMEAPortMessage("GPRMC", 1);

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  Serial.println("GNSS Configuration complete");

  Serial.print("Connecting to local WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

  //Clear any serial characters from the buffer
  while (Serial.available())
    Serial.read();
}

void loop()
{
  if (Serial.available())
  {
    beginClient();
    while (Serial.available())
      Serial.read(); //Empty buffer of any newline chars
  }

  Serial.println("Press any key to start NTRIP Client.");

  delay(1000);
}

//Connect to the NTRIP Caster, receive RTCM, and push it to the GNSS module
void beginClient()
{
  WiFiClient ntripClient;
  long rtcmCount = 0;

  Serial.println("Subscribing to Caster. Press key to stop");
  delay(10); //Wait for any serial to arrive
  while (Serial.available())
    Serial.read(); //Flush

  // Break if we receive a character from the user
  while (Serial.available() == 0)
  {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false)
    {
      Serial.print("Opening socket to ");
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
      {
        Serial.println("Connection to caster failed");
        return;
      }
      else
      {
        Serial.print("Connected to ");
        Serial.print(casterHost);
        Serial.print(": ");
        Serial.println(casterPort);

        Serial.print("Requesting NTRIP Data from mount point ");
        Serial.println(mountPoint);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE + 1];

        snprintf(serverRequest,
                 SERVER_BUFFER_SIZE,
                 "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun UM980 Client v1.0\r\n",
                 mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0)
        {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        }
        else
        {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print("Sending credentials: ");
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen];                                         //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif

          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print("serverRequest size: ");
        Serial.print(strlen(serverRequest));
        Serial.print(" of ");
        Serial.print(sizeof(serverRequest));
        Serial.println(" bytes available");

        Serial.println("Sending server request:");
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println("Caster timed out!");
            ntripClient.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available())
        {
          if (responseSpot == sizeof(response) - 1)
            break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") != nullptr) //Look for '200 OK'
            connectionSuccess = true;
          if (strstr(response, "401") != nullptr) //Look for '401 Unauthorized'
          {
            Serial.println("Hey - your credentials look bad! Check you caster username and password.");
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print("Caster responded with: ");
        Serial.println(response);

        if (connectionSuccess == false)
        {
          Serial.print("Failed to connect to ");
          Serial.println(casterHost);
          return;
        }
        else
        {
          Serial.print("Connected to ");
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis(); //Reset timeout
          ggaTransmitComplete = true;     //Reset to start polling for new GGA data
        }
      } //End attempt to connect
    }   //End connected == false

    if (ntripClient.connected() == true)
    {
      uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available())
      {
        //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData))
          break;
      }

      if (rtcmCount > 0)
      {
        lastReceivedRTCM_ms = millis();

        //Write incoming RTCM directly to UM980
        SerialGNSS.write(rtcmData, rtcmCount);
        Serial.print("RTCM pushed to GNSS: ");
        Serial.println(rtcmCount);
      }
    }

    //Write incoming NMEA back out to serial port and check for incoming GGA sentence
    while (SerialGNSS.available())
    {
      byte incoming = SerialGNSS.read();
      processNMEA(incoming);
      Serial.write(incoming);
    }

    //Provide the caster with our current position as needed
    if (ntripClient.connected() == true
        && transmitLocation == true
        && (millis() - lastTransmittedGGA_ms) > timeBetweenGGAUpdate_ms
        && ggaSentenceComplete == true
        && ggaTransmitComplete == false)
    {
      Serial.print("Pushing GGA to server: ");
      Serial.println(ggaSentence);

      lastTransmittedGGA_ms = millis();

      //Push our current GGA sentence to caster
      ntripClient.print(ggaSentence);
      ntripClient.print("\r\n");

      ggaTransmitComplete = true;
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println("RTCM timeout. Disconnecting...");
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println("User pressed a key");
  Serial.println("Disconnecting...");
  ntripClient.stop();
}

//As each NMEA character comes in you can specify what to do with it
//We will look for and copy the GGA sentence
void processNMEA(char incoming)
{
  //Take the incoming char from the GNSS and check to see if we should record it or not
  if (incoming == '$' && ggaTransmitComplete == true)
  {
    ggaSentenceStarted = true;
    ggaSentenceSpot = 0;
    ggaSentenceEndSpot = sizeof(ggaSentence);
    ggaSentenceComplete = false;
  }

  if (ggaSentenceStarted == true)
  {
    ggaSentence[ggaSentenceSpot++] = incoming;

    //Make sure we don't go out of bounds
    if (ggaSentenceSpot == sizeof(ggaSentence))
    {
      //Start over
      ggaSentenceStarted = false;
    }
    //Verify this is the GGA setence
    else if (ggaSentenceSpot == 5 && incoming != 'G')
    {
      //Ignore this sentence, start over
      ggaSentenceStarted = false;
    }
    else if (incoming == '*')
    {
      //We're near the end. Keep listening for two more bytes to complete the CRC
      ggaSentenceEndSpot = ggaSentenceSpot + 2;
    }
    else if (ggaSentenceSpot == ggaSentenceEndSpot)
    {
      ggaSentence[ggaSentenceSpot] = '\0'; //Terminate this string
      ggaSentenceComplete = true;
      ggaTransmitComplete = false; //We are ready for transmission

      //Serial.print("GGA Parsed - ");
      //Serial.println(ggaSentence);

      //Start over
      ggaSentenceStarted = false;
    }
  }
}