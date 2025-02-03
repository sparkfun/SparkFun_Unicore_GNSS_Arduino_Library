/*
  Connect to NTRIP Caster and send RTCM as an NTRIP Server
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 31, 2025
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to gather RTCM data from the GNSS receiver and push it to a casting service over WiFi.
  It's confusing, but the Arduino is acting as a 'server' to a 'caster'. In this case we will
  use RTK2Go.com as our caster because it is free. A rover (car, surveyor stick, etc) can
  then connect to RTK2Go as a 'client' and get the RTCM data it needs to achieve an RTK Fix.

  You will need to register your mountpoint here: http://www.rtk2go.com/new-reservation/
  (They'll probably block the credentials we include in this example)

  To see if your mountpoint is active go here: http://rtk2go.com:2101/

  This is a proof of concept. Serving RTCM to a caster over WiFi is useful when you need to
  set up a high-precision base station.

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
WiFiClient ntripCaster;

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS
UM980 myGNSS;

#define pin_UART_TX     4
#define pin_UART_RX     13

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

//Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSentRTCM_ms = 0;           //Time of last data pushed to socket
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

uint32_t serverBytesSent = 0; //Just a running total
long lastReport_ms = 0;       //Time of last report of bytes sent
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

  //We are not going to enable any NMEA so that the only thing pushed to the caster is RTCM

  //When the coordinates of the base station are known, users can set the position
  //using Geodetic or ECEF coordinates.
  //myGNSS.setModeBaseGeodetic(40.09029479, -105.18505761, 1560.089); //SparkFun HQ
  myGNSS.setModeBaseECEF(-1280206.568, -4716804.403, 4086665.484); //SparkFun HQ

  //Set the output rate of RTCM correction messages.
  //Unicore supports a *large* number of RTCM messages. We recommend the following for most applications:
  //RTCM1006, 1074, 1084, 1094, 1124, 1033
  myGNSS.setRTCMMessage("RTCM1006", 10);
  myGNSS.setRTCMMessage("RTCM1074", 1);
  myGNSS.setRTCMMessage("RTCM1084", 1);
  myGNSS.setRTCMMessage("RTCM1094", 1);
  myGNSS.setRTCMMessage("RTCM1124", 1);
  myGNSS.setRTCMMessage("RTCM1033", 10);

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
    beginServer();
    while (Serial.available())
      Serial.read(); //Empty buffer of any newline chars
  }

  Serial.println("Press any key to start NTRIP Server.");

  delay(1000);
}

//Connect to the NTRIP Caster and push RTCM to it
void beginServer()
{
  Serial.println("Begin transmitting to caster. Press any key to stop");
  delay(10); //Wait for any serial to arrive
  while (Serial.available())
    Serial.read(); //Flush

  while (Serial.available() == 0)
  {
    //Connect if we are not already
    if (ntripCaster.connected() == false)
    {
      Serial.printf("Opening socket to %s\n", casterHost);

      if (ntripCaster.connect(casterHost, casterPort) == true) //Attempt connection
      {
        Serial.printf("Connected to %s:%d\n", casterHost, casterPort);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest,
                 SERVER_BUFFER_SIZE,
                 "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun UM980 Server v1.0\r\n\r\n",
                 mountPointPW, mountPoint);

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripCaster.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripCaster.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println("Caster timed out!");
            ntripCaster.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripCaster.available())
        {
          response[responseSpot++] = ntripCaster.read();
          if (strstr(response, "200") != nullptr) //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (responseSpot == 512 - 1)
            break;
        }
        response[responseSpot] = '\0';

        if (connectionSuccess == false)
        {
          Serial.printf("Failed to connect to Caster: %s", response);
          return;
        }
      } //End attempt to connect
      else
      {
        Serial.println("Connection to host failed");
        return;
      }
    } //End connected == false

    if (ntripCaster.connected() == true)
    {
      delay(10);
      while (Serial.available())
        Serial.read(); //Flush any endlines or carriage returns

      lastReport_ms = millis();
      lastSentRTCM_ms = millis();

      //This is the main sending loop. We scan for new data but processRTCM() is where the data actually gets sent out.
      while (1)
      {
        if (Serial.available())
          break;

        //Write incoming RTCM to the NTRIP Caster
        while (SerialGNSS.available())
        {
          ntripCaster.write(SerialGNSS.read()); //Send this byte to socket
          serverBytesSent++;
          lastSentRTCM_ms = millis();
        }

        //Close socket if we don't have new data for 10s
        //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
        //So let's not leave the socket open/hanging without data
        if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms)
        {
          Serial.println("RTCM timeout. Disconnecting...");
          ntripCaster.stop();
          return;
        }

        delay(10);

        //Report some statistics every 250
        if (millis() - lastReport_ms > 250)
        {
          lastReport_ms += 250;
          Serial.printf("Total sent: %d\n", serverBytesSent);
        }
      }
    }

    delay(10);
  }

  Serial.println("User pressed a key");
  Serial.println("Disconnecting...");
  ntripCaster.stop();

  delay(10);
  while (Serial.available())
    Serial.read(); //Flush any endlines or carriage returns
}
