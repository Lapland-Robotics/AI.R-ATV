/*
  Use ESP32 WiFi to get RTCM data from RTK2Go (caster) as a Client
  By: SparkFun Electronics / Nathan Seidle
  Date: November 18th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain RTCM data from a NTRIP Caster over WiFi and push it over I2C to a ZED-F9x.
  The Arduino is acting as a 'client' to a 'caster'. In this case we will use RTK2Go.com as our caster because it is free. 
  See the NTRIPServer example to see how to push RTCM data to the caster.

  You will need to have a valid mountpoint available. To see available mountpoints go here: http://rtk2go.com:2101/

  For more information about NTRIP Clients and the differences between Rev1 and Rev2 of the protocol
  please see: https://www.use-snip.com/kb/knowledge-base/ntrip-rev1-versus-rev2-formats/

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  Open the serial monitor at 115200 baud to see the output
*/

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"  //Built-in ESP32 library
#else
#include <Base64.h>  //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

#include <WiFi.h>
#include <stdio.h>
#include <math.h>
#include "secrets.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS

#define EARTH_RADIUS_CM 637100000.0

long lastReceivedRTCM_ms = 0;        //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000;  //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
SFE_UBLOX_GNSS myGNSS;

// GPS test points coordinates in lapinAMK
double gpsTestPoint[3][2] = {
  { 66.480572358, 25.722133784 }, // apk3
  { 66.481161444, 25.722809583 }, // 50502
  { 66.480765600, 25.720936400 }  // IOT lab
};


// Convert degrees to radians
double deg_to_rad(double deg) {
  return deg * (M_PI / 180.0);
}

// Haversine formula to calculate distance between two GPS coordinates in cm
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitudes and longitudes from degrees to radians
  lat1 = deg_to_rad(lat1);
  lon1 = deg_to_rad(lon1);
  lat2 = deg_to_rad(lat2);
  lon2 = deg_to_rad(lon2);

  // Differences between the points
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;

  // Haversine formula
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Distance in centimeters
  double distance = EARTH_RADIUS_CM * c;

  return distance;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("NTRIP testing"));
  Wire.begin();  //Start I2C

  //Connect to the Ublox module using Wire port
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("u-blox module connected"));

  myGNSS.setI2COutput(COM_TYPE_UBX);                                                 //Turn off NMEA noise
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);  //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
  myGNSS.setNavigationFrequency(1);  //Set output in Hz.

  Serial.print(F("Connecting to local WiFi"));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.print(F("WiFi connected with IP: "));
  Serial.println(WiFi.localIP());

  while (Serial.available()) Serial.read();
}

void loop() {
  if (true) {
    beginClient();
    while (Serial.available()) Serial.read();  //Empty buffer of any newline chars
  }

  delay(1000);
}

//Connect to NTRIP Caster, receive RTCM, and push to ZED module over I2C
void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;

  Serial.println(F("Subscribing to Caster. Press key to stop"));
  delay(10);                                 //Wait for any serial to arrive
  while (Serial.available()) Serial.read();  //Flush

  while (Serial.available() == 0) {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false) {
      Serial.print(F("Opening socket to "));
      Serial.println(casterHost);

      if (ntripClient.connect(casterHost, casterPort) == false)  //Attempt connection
      {
        Serial.println(F("Connection to caster failed"));
        return;
      } else {
        Serial.print(F("Connected to "));
        Serial.print(casterHost);
        Serial.print(F(": "));
        Serial.println(casterPort);

        Serial.print(F("Requesting NTRIP Data from mount point "));
        Serial.println(mountPoint);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest, SERVER_BUFFER_SIZE, "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
                 mountPoint);

        char credentials[512];
        if (strlen(casterUser) == 0) {
          strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
        } else {
          //Pass base64 encoded user:pw
          char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1];  //The ':' takes up a spot
          snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

          Serial.print(F("Sending credentials: "));
          Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
          //Encode with ESP32 built-in library
          base64 b;
          String strEncodedCredentials = b.encode(userCredentials);
          char encodedCredentials[strEncodedCredentials.length() + 1];
          strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials));  //Convert String to char array
          snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
#else
          //Encode with nfriendly library
          int encodedLen = base64_enc_len(strlen(userCredentials));
          char encodedCredentials[encodedLen];                                          //Create array large enough to house encoded data
          base64_encode(encodedCredentials, userCredentials, strlen(userCredentials));  //Note: Input array is consumed
#endif
        }
        strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
        strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

        Serial.print(F("serverRequest size: "));
        Serial.print(strlen(serverRequest));
        Serial.print(F(" of "));
        Serial.print(sizeof(serverRequest));
        Serial.println(F(" bytes available"));
        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0) {
          if (millis() - timeout > 5000) {
            Serial.println(F("Caster timed out!"));
            ntripClient.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripClient.available()) {
          if (responseSpot == sizeof(response) - 1) break;

          response[responseSpot++] = ntripClient.read();
          if (strstr(response, "200") > 0)  //Look for 'ICY 200 OK'
            connectionSuccess = true;

          if (strstr(response, "401") > 0){
            Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        Serial.print(F("Caster responded with: "));
        Serial.println(response);

        if (connectionSuccess == false) {
          Serial.print(F("Failed to connect to "));
          Serial.print(casterHost);
          Serial.print(F(": "));
          Serial.println(response);
          return;
        } else {
          Serial.print(F("Connected to "));
          Serial.println(casterHost);
          lastReceivedRTCM_ms = millis();  //Reset timeout
        }
      }  //End attempt to connect
    }    //End connected == false

    if (ntripClient.connected() == true) {
      uint8_t rtcmData[512 * 4];  //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available()) {
        //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        Serial.print(F("RTCM pushed to ZED: "));
        Serial.println(rtcmCount);
      }

      if (myGNSS.getGnssFixOk())  // Check if GNSS fix is available
      {
        double latitude = myGNSS.getLatitude() / 10000000.00;    // Get latitude and convert to degrees
        double longitude = myGNSS.getLongitude() / 10000000.00;  // Get longitude and convert to degrees
        double altitude = myGNSS.getAltitude() / 1000.00;        // Get altitude in meters
        double horizontal_accuracy = myGNSS.getHorizontalAccuracy() / 100.00;
        double vertical_accuracy = myGNSS.getVerticalAccuracy() / 100.00;
        double real_distance = haversine_distance(latitude, longitude, gpsTestPoint[2][0], gpsTestPoint[2][1]);

        Serial.print("Latitude: ");
        Serial.print(latitude, 9);  // Print latitude with 7 decimal places
        Serial.print(", Longitude: ");
        Serial.print(longitude, 9);  // Print longitude with 7 decimal places
        Serial.print(", Altitude: ");
        Serial.print(altitude);
        Serial.print(", horizontal_accuracy (cm): ");
        Serial.print(horizontal_accuracy, 4);
        Serial.print(", vertical_accuracy (cm): ");
        Serial.print(vertical_accuracy, 4);
        Serial.print(", real_distance (cm): ");
        Serial.print(real_distance, 4);
        Serial.println();
      } else {
        Serial.println(F("Waiting for GNSS fix..."));
      }
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      Serial.println(F("RTCM timeout. Disconnecting..."));
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  ntripClient.stop();
}
