// CRSF Telemetry to MAVLink Telemetry
//
// (c) 2019 David Wyss
// (c) Rob Thomson
// (c) Eric Flynn
//
// Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

#include "Arduino.h"
#include <stdint.h> 
#include "xfire.h"
#include <limits.h>
//#include "RTClib.h"
#include <Time.h>
#include <inttypes.h>

#include <common/mavlink.h>
#include <SoftwareSerial.h>

#define REFRESH_INTERVAL 26  //<<<<< ----- Change to 4 for Normal CRSF/ELRS
#define CROSSFIRE_BAUD_RATE 115200 // <<<<<---- Change to 400000 for Normal CRSF/ELRS rate

//Bring in all externals for crossfire
extern uint32_t crossfireChannels[CROSSFIRE_CHANNELS_COUNT];  //RC channel pulse data
extern float sensorVario;
extern double sensorGPSLat;
extern double sensorGPSLong;
extern float sensorAltitude;
extern float sensorHeading;
extern float sensorSpeed;
extern uint32_t sensorSats;
extern float sensorPitch;
extern float sensorRoll;
extern float sensorYaw;
extern double sensorVoltage;
extern double sensorCurrent;
extern double sensorFuel;
extern uint32_t  sensor1RSS;
extern uint32_t sensor2RSS;
extern uint32_t sensorRXQly;
extern uint32_t sensorRXSNR; 
extern uint32_t sensorAntenna; 
extern uint32_t sensorRFMode;
extern uint32_t sensorTXPWR;
extern uint32_t sensorTXRSSI; 
extern uint32_t sensorTXQly;
extern uint32_t sensorTXSNR;
extern uint32_t sensorCapacity;



// Mavlink Parameter setup
// These parameters may be entered manually, parsed from another protocol or simulated locally.
// CRSF Does not provide input data for all Parameters. Enter defaults that make sense to you. These defaults will be passed to Mavlink as STATIC values.
// All parameters set here are passed through (Serial) output in MAVLink format.

//Mavlink Basic UAV Parameters
uint8_t system_id = 1;        // MAVLink system ID. Leave at 0 unless you need a specific ID.
uint8_t component_id = 0;     // Should be left at 0. Set to 190 to simulate mission planner sending a command
uint8_t system_type = 2;      // UAV type. 0 = generic, 1 = fixed wing, 2 = quadcopter, 3 = helicopter
uint8_t autopilot_type = 0;   // Autopilot type. Usually set to 0 for generic autopilot with all capabilities
uint8_t system_mode = 16;     // Flight mode. 4 = auto mode, 8 = guided mode, 16 = stabilize mode, 64 = manual mode
uint32_t custom_mode = 0;     // Usually set to 0          
uint8_t system_state = 4;     // 0 = unknown, 3 = standby, 4 = active
uint32_t upTime = 0;          // System uptime, usually set to 0 for cases where it doesn't matter

//Mavlink Flight parameters
float roll = 0;         // Roll angle in degrees
float pitch = 0;        // Pitch angle in degrees
float yaw = 0;          // Yaw angle in degrees
int16_t heading = 0;      // Geographical heading angle in degrees
float lat = 47.379945;    // GPS latitude in degrees (example: 47.123456)
float lon = 8.539970;     // GPS longitude in degrees
float alt = 0.0;        // Relative flight altitude in m
float groundspeed = 0.0; // Groundspeed in m/s
float airspeed = 0.0;    // Airspeed in m/s
float climbrate = 0.0;    // Climb rate in m/s, currently not working
float throttle = 0.0;     // Throttle percentage

//Mavlink GPS parameters
int16_t gps_sats = 0;    // Number of visible GPS satellites
int32_t gps_alt = 0.0;  // GPS altitude (Altitude above MSL)
float gps_hdop = 100.0;     // GPS HDOP
uint8_t fixType = 3;      // GPS fix type. 0-1: no fix, 2: 2D fix, 3: 3D fix




//Mavlink Battery parameters
float battery_remaining = 0.0;  // Remaining battery percentage
float voltage_battery = 0.0;    // Battery voltage in V
float current_battery = 0.0;    // Battery current in A
#define SOFTRX 12
#define SOFTTX 18

#define DEBUG

EspSoftwareSerial::UART SerialX;

void setup() {
  
   // Enable serial output. Default: 57600 baud, can be set higher if needed
   SerialX.begin(115200, SWSERIAL_8N1, SOFTRX, SOFTTX);// SerialX Debug Output via USB/serial
   
   startCrossfire();  //this invokes the crossfire system on hardware Serial1
   Serial.begin(57600, SERIAL_8N1, 1, 0);//Serial MAVLINK OUTPUT
}



/************************************************************
* @brief Sends a MAVLink heartbeat message, needed for the system to be recognized
* @param Basic UAV parameters, as defined above
* @return void
*************************************************************/

void command_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t system_type, uint8_t autopilot_type, uint8_t system_mode, uint32_t custom_mode, uint8_t system_state) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(system_id,component_id, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message
  Serial.write(buf, len);
}


/************************************************************
* @brief Send some system data parameters (battery, etc)
* @param 
* @return void
*************************************************************/
       
void command_status(uint8_t system_id, uint8_t component_id, float sensorFuel, float sensorVoltage, float sensorCurrent) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
    mavlink_msg_sys_status_pack(system_id, component_id, &msg, 32767, 32767, 32767, 500, sensorVoltage * 1000.0, sensorCurrent * 10.0, sensorFuel, 0, 0, 0, 0, 0, 0);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float sensorGPSLat, float sensorGPSLong, float sensorAltitude, float gps_alt, uint16_t sensorHeading);

/************************************************************
* @brief Sends current geographical location (GPS position), altitude and heading
* @param lat: latitude in degrees, lon: longitude in degrees, alt: altitude, heading: heading
* @return void
*************************************************************/

void command_gps(int8_t system_id, int8_t component_id, int32_t upTime, int8_t fixType, float sensorGPSLat, float sensorGPSLong, float sensorAltitude, float gps_alt, int16_t sensorHeading, float sensorSpeed, float gps_hdop, int16_t sensorSats) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Setiing fixtype based on sensorSats
  if (sensorSats >= 6){fixType = 3;
      }
  if (sensorSats < 6){fixType = 1;
      }

  // Pack the message
  mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg, upTime, fixType, sensorGPSLat * 10000000.0, sensorGPSLong * 10000000.0, sensorAltitude * 1000.0, gps_hdop * 100.0, 65535, sensorSpeed / 3.6, 65535, sensorSats);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send globalgps command
  command_globalgps(system_id, component_id, upTime, sensorGPSLat, sensorGPSLong, sensorAltitude, gps_alt, sensorHeading);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

/************************************************************
* @brief Send some core parameters such as speed to the MAVLink ground station HUD
* @param 
* @return void
*************************************************************/
       
void command_hud(int8_t system_id, int8_t component_id, float airspeed, float sensorSpeed, int16_t sensorHeading, float throttle, float sensorAltitude, float climbrate) {

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_vfr_hud_pack(system_id, component_id, &msg, airspeed, sensorSpeed, sensorHeading, throttle, sensorAltitude * 1000.0, climbrate);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}

/************************************************************
* @brief Send attitude and heading data to the primary flight display (artificial horizon)
* @param roll: roll in degrees, pitch: pitch in degrees, yaw: yaw in degrees, heading: heading in degrees
* @return void
*************************************************************/
       
void command_attitude(int8_t system_id, int8_t component_id, int32_t upTime, float sensorRoll, float sensorPitch, float sensorYaw) {

  //Radian -> degree conversion rate
  float radian = 57.2958;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_attitude_pack(system_id, component_id, &msg, upTime, sensorRoll/radian, sensorPitch/radian, sensorYaw/radian, 0, 0, 0); 

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}



/************************************************************
* @brief Sends Integer representation of location, only for ijnternal use (do not call directly)
* @param lat: latitude, lon: longitude, alt: altitude, gps_alt: altitude above MSL, heading: heading
* @return void
*************************************************************/
       
void command_globalgps(int8_t system_id, int8_t component_id, int32_t upTime, float sensorGPSLat, float sensorGPSLong, float sensorAltitude, float gps_alt, uint16_t sensorHeading) {

  int16_t velx = 0; //x speed
  int16_t vely = 0; //y speed
  int16_t velz = 0; //z speed


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_global_position_int_pack(system_id, component_id, &msg, upTime, sensorGPSLat * 10000000.0, sensorGPSLong * 10000000.0, gps_alt * 1000.0, sensorAltitude * 1000.0, velx, vely, velz, sensorHeading);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes)
  Serial.write(buf, len);
}


//Main loop: Send generated MAVLink data via serial output
void loop() {

#if defined(DEBUG)
  //Print CRSF Telem to USB/serial using variables as defined above. Add others as you wish
     SerialX.print("VOLTAGE: ");
     SerialX.println(sensorVoltage);

     SerialX.print("SPEED: ");     
     SerialX.println(sensorSpeed / 3.6);

     SerialX.print("ALT: ");     
     SerialX.println(sensorAltitude);

     SerialX.print("HEADING: ");     
     SerialX.println(sensorHeading);
     
     SerialX.print("SATS: ");     
     SerialX.println(sensorSats);

     SerialX.print("LAT: ");     
     SerialX.println(sensorGPSLat,7);

     SerialX.print("LON: ");     
     SerialX.println(sensorGPSLong,7);
  //etc..
#endif
  
  // Send MAVLink messages
  command_heartbeat(system_id, component_id, system_type, autopilot_type, system_mode, custom_mode, system_state);
  command_status(system_id, component_id, sensorFuel, sensorVoltage, sensorCurrent);
  command_gps(system_id, component_id, upTime, fixType, sensorGPSLat, sensorGPSLong, sensorAltitude, sensorAltitude, sensorHeading, sensorSpeed, gps_hdop, sensorSats);
  command_hud(system_id, component_id, airspeed, sensorSpeed, sensorHeading, throttle, sensorAltitude, climbrate);
  command_attitude(system_id, component_id, upTime, sensorRoll, sensorPitch, sensorYaw);


  //Send messages at about 10Hz
  delay(100);
}

