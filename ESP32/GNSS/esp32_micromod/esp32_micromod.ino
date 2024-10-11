/*
  GNSS module obtain RTCM data from a NTRIP Caster over WiFi and push it over I2C to a ZED-F9x.
  The module is acting as a 'client' to a 'caster'.
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
#include <string>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/string.h>

long lastReceivedRTCM_ms = 0;        //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000;  //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
double latitude;    // Get latitude and convert to degrees
double longitude;  // Get longitude and convert to degrees
double altitude;        // Get altitude in meters
double horizontal_accuracy;
double vertical_accuracy;
double distance;
rcl_publisher_t gpsMsgPublisher;
rcl_publisher_t debugMsgPublisher; // Define the new publisher
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
sensor_msgs__msg__NavSatFix navSatMsg;
std_msgs__msg__String debugMsg;
SFE_UBLOX_GNSS myGNSS;

#define EARTH_RADIUS_CM 637100000.0
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

void microros_init(){
  
  set_microros_transports(); // microros over serial
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_gnss_node", "", &support));

  // publisher for /snower/gps
  RCCHECK(rclc_publisher_init_best_effort(&gpsMsgPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/snower/gps"));
  // publisher for /snower/debug
  RCCHECK(rclc_publisher_init_best_effort(&debugMsgPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/snower/debug"));

  // Initialize the navSatMsg message
  navSatMsg.latitude = 0.0;
  navSatMsg.longitude = 0.0;
  navSatMsg.altitude = 0.0;
  navSatMsg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;

  // Initialize the navSatMsg message
  debugMsg.data.data = (char *)malloc(200 * sizeof(char)); // Allocate memory for the string
  debugMsg.data.size = 0;
  debugMsg.data.capacity = 200;
}

void nTrip_init(){

  // SparkFun_u-blox_GNSS
  debug("NTRIP testing");
  Wire.begin();  //Start I2C

  //Connect to the Ublox module using Wire port
  if (myGNSS.begin() == false) {
    debug("u-blox GPS not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  debug("u-blox module connected");

  myGNSS.setI2COutput(COM_TYPE_UBX);  //Turn off NMEA noise
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);  //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.
  myGNSS.setNavigationFrequency(1);  //Set output in Hz.

  debug("Connecting to local WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug(".");
  }
  debug("WiFi connected with IP: %s", WiFi.localIP().toString().c_str());

}

void setup() {
  // Serial.begin(115200);
  microros_init();
  nTrip_init();
}

void loop() {
  beginClient();
  delay(500);
}

//Connect to NTRIP Caster, receive RTCM, and push to ZED module over I2C
void beginClient() {
  WiFiClient ntripClient;
  long rtcmCount = 0;

  while (true) {
    //Connect if we are not already. Limit to 5s between attempts.
    if (ntripClient.connected() == false) {
      debug("Opening socket to %s", casterHost);

      //Attempt connection
      if (ntripClient.connect(casterHost, casterPort) == false) {
        debug("Connection to caster failed");
        return;
      } else {
        debug("Connected to %s:%i ", casterHost, casterPort);
        debug("Requesting NTRIP Data from mount point %s", mountPoint);

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

          debug("Sending credentials...");

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

        debug("serverRequest size: %d of %d bytes available", strlen(serverRequest), sizeof(serverRequest));
        debug("Sending server request: %s", serverRequest);
        ntripClient.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripClient.available() == 0) {
          if (millis() - timeout > 5000) {
            debug("Caster timed out!");
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
            debug("Hey - your credentials look bad! Check you caster username and password.");
            connectionSuccess = false;
          }
        }
        response[responseSpot] = '\0';

        debug("Caster responded with: %s", response);

        if (connectionSuccess == false) {
          debug("Failed to connect to %s : %s", casterHost, response);
          return;
        } else {
          debug("Connected to %s", casterHost);
          lastReceivedRTCM_ms = millis();  //Reset timeout
        }
      }  //End attempt to connect
    }    //End connected == false

    if (ntripClient.connected() == true) {
      uint8_t rtcmData[512 * 4];  //Most incoming data is around 500 bytes but may be larger
      rtcmCount = 0;

      //Print any available RTCM data
      while (ntripClient.available()) {
        //debug(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
        rtcmData[rtcmCount++] = ntripClient.read();
        if (rtcmCount == sizeof(rtcmData)) break;
      }

      if (rtcmCount > 0) {
        lastReceivedRTCM_ms = millis();
        //Push RTCM to GNSS module over I2C
        myGNSS.pushRawData(rtcmData, rtcmCount, false);
        debug("RTCM pushed to ZED: %li", rtcmCount);
      }
      
      // Check if GNSS fix is available
      if (myGNSS.getGnssFixOk()) {

        navSatMsg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_GBAS_FIX;
        navSatMsg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
        navSatMsg.header.stamp.sec = myGNSS.getUnixEpoch(); // Get epoch time in seconds
        navSatMsg.header.stamp.nanosec = (myGNSS.getMillisecond() * 1000000) + myGNSS.getNanosecond(); // Convert to nanoseconds

        latitude = myGNSS.getLatitude() / 10000000.00;    // Get latitude and convert to degrees
        longitude = myGNSS.getLongitude() / 10000000.00;  // Get longitude and convert to degrees
        altitude = myGNSS.getAltitude() / 1000.00;        // Get altitude in meters
        horizontal_accuracy = myGNSS.getHorizontalAccuracy() / 10000.00;  // Convert to meters
        vertical_accuracy = myGNSS.getVerticalAccuracy() / 10000.00;  // Convert to meters

      } else {
        
        navSatMsg.header.stamp.sec = myGNSS.getUnixEpoch(); // Get epoch time in seconds
        navSatMsg.header.stamp.nanosec = (myGNSS.getMillisecond() * 1000000) + myGNSS.getNanosecond(); // Convert to nanoseconds
        navSatMsg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;

        latitude = myGNSS.getLatitude() / 10000000.00;    // Get latitude and convert to degrees
        longitude = myGNSS.getLongitude() / 10000000.00;  // Get longitude and convert to degrees
        altitude = myGNSS.getAltitude() / 1000.00;        // Get altitude in meters
        horizontal_accuracy = myGNSS.getHorizontalAccuracy() / 10000.00;  // Convert to meters
        vertical_accuracy = myGNSS.getVerticalAccuracy() / 10000.00;  // Convert to meters

      }

      navSatMsg.latitude = latitude;
      navSatMsg.longitude = longitude;
      navSatMsg.altitude = altitude;
      navSatMsg.position_covariance[0] = horizontal_accuracy * horizontal_accuracy;  // Horizontal accuracy
      navSatMsg.position_covariance[4] = horizontal_accuracy * horizontal_accuracy;  // Same for covariance
      navSatMsg.position_covariance[8] = vertical_accuracy * vertical_accuracy;    // Vertical accuracy
      navSatMsg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_DIAGONAL_KNOWN;

      distance = haversine_distance(latitude, longitude, gpsTestPoint[2][0], gpsTestPoint[2][1]);
      debug("latitude- %lf, longitude- %lf, horizontal_accuracy- %lf m, distance to point- %lf cm", latitude, longitude, horizontal_accuracy, distance);

      RCSOFTCHECK(rcl_publish(&gpsMsgPublisher, &navSatMsg, NULL));
    }

    //Close socket if we don't have new data for 10s
    if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms) {
      debug("RTCM timeout. Disconnecting...");
      if (ntripClient.connected() == true)
        ntripClient.stop();
      return;
    }

    delay(10);
  }
}

void debug(const char* format, ...) {
  char buffer[512];  // Buffer to store the formatted message
  va_list args;

  va_start(args, format);  // Initialize the variable argument list
  vsprintf(buffer, format, args);  // Format the string with the variable arguments
  va_end(args);  // Clean up the variable argument list
  // Serial.println(buffer);

  snprintf(debugMsg.data.data, debugMsg.data.capacity, "[GNSS]: %s", buffer);
  debugMsg.data.size = strlen(debugMsg.data.data);
  RCSOFTCHECK(rcl_publish(&debugMsgPublisher, &debugMsg, NULL));

  delay(50); 
}
