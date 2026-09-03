#pragma once
// MESSAGE HR_TM PACKING

#define MAVLINK_MSG_ID_HR_TM 180

MAVPACKED(
typedef struct __mavlink_hr_tm_t {
 uint64_t timestamp; /*< [ms] Timestamp in milliseconds*/
 float pressure_ada; /*< [Pa] Atmospheric pressure estimated by ADA*/
 float pressure_digi; /*< [Pa] Pressure from digital sensor*/
 float pressure_static; /*< [Pa] Pressure from static port*/
 float pressure_dpl; /*< [Pa] Pressure from deployment vane sensor*/
 float airspeed_pitot; /*< [m/s] Pitot airspeed*/
 float msl_altitude; /*< [m] Altitude above mean sea level*/
 float ada_vert_speed; /*< [m/s] Vertical speed estimated by ADA*/
 float ada_vert_accel; /*< [m/s^2] Vertical acceleration estimated by ADA*/
 float acc_x; /*< [m/s^2] Acceleration on X axis (body)*/
 float acc_y; /*< [m/s^2] Acceleration on Y axis (body)*/
 float acc_z; /*< [m/s^2] Acceleration on Z axis (body)*/
 float gyro_x; /*< [rad/s] Angular speed on X axis (body)*/
 float gyro_y; /*< [rad/s] Angular speed on Y axis (body)*/
 float gyro_z; /*< [rad/s] Angular speed on Z axis (body)*/
 float mag_x; /*< [uT] Magnetic field on X axis (body)*/
 float mag_y; /*< [uT] Magnetic field on Y axis (body)*/
 float mag_z; /*< [uT] Magnetic field on Z axis (body)*/
 float gps_lat; /*< [deg] Latitude*/
 float gps_lon; /*< [deg] Longitude*/
 float gps_alt; /*< [m] GPS Altitude*/
 float vbat; /*< [V] Battery voltage*/
 float vsupply_5v; /*< [V] Power supply 5V*/
 float temperature; /*< [degC] Temperature*/
 float ab_angle; /*< [deg] Aerobrakes angle*/
 float ab_estimated_cd; /*<  Estimated drag coefficient*/
 float nas_x; /*< [deg] Navigation system estimated X position (longitude)*/
 float nas_y; /*< [deg] Navigation system estimated Y position (latitude)*/
 float nas_z; /*< [m] Navigation system estimated Z position*/
 float nas_vx; /*< [m/s] Navigation system estimated north velocity*/
 float nas_vy; /*< [m/s] Navigation system estimated east velocity*/
 float nas_vz; /*< [m/s] Navigation system estimated down velocity*/
 float nas_roll; /*< [deg] Navigation system estimated attitude (roll)*/
 float nas_pitch; /*< [deg] Navigation system estimated attitude (pitch)*/
 float nas_yaw; /*< [deg] Navigation system estimated attitude (yaw)*/
 float nas_bias0; /*<  Navigation system bias*/
 float nas_bias1; /*<  Navigation system bias*/
 float nas_bias2; /*<  Navigation system bias*/
 uint8_t ada_state; /*<  ADA Controller State*/
 uint8_t fmm_state; /*<  Flight Mode Manager State*/
 uint8_t dpl_state; /*<  Deployment Controller State*/
 uint8_t ab_state; /*<  Aerobrake FSM state*/
 uint8_t nas_state; /*<  Navigation System FSM State*/
 uint8_t gps_fix; /*<  GPS fix (1 = fix, 0 = no fix)*/
 uint8_t pin_launch; /*<  Launch pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_nosecone; /*<  Nosecone pin status (1 = connected, 0 = disconnected)*/
 uint8_t servo_sensor; /*<  Servo sensor status (1 = actuated, 0 = idle)*/
 int8_t logger_error; /*<  Logger error (0 = no error, -1 otherwise)*/
}) mavlink_hr_tm_t;

#define MAVLINK_MSG_ID_HR_TM_LEN 166
#define MAVLINK_MSG_ID_HR_TM_MIN_LEN 166
#define MAVLINK_MSG_ID_180_LEN 166
#define MAVLINK_MSG_ID_180_MIN_LEN 166

#define MAVLINK_MSG_ID_HR_TM_CRC 67
#define MAVLINK_MSG_ID_180_CRC 67



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HR_TM { \
    180, \
    "HR_TM", \
    48, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hr_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_hr_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 157, offsetof(mavlink_hr_tm_t, fmm_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_hr_tm_t, dpl_state) }, \
         { "ab_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_hr_tm_t, ab_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_hr_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hr_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hr_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hr_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hr_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hr_tm_t, airspeed_pitot) }, \
         { "msl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hr_tm_t, msl_altitude) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hr_tm_t, ada_vert_speed) }, \
         { "ada_vert_accel", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hr_tm_t, ada_vert_accel) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hr_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hr_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hr_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hr_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hr_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_hr_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_hr_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_hr_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_hr_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_hr_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_hr_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_hr_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_hr_tm_t, gps_alt) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_hr_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_hr_tm_t, vsupply_5v) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_hr_tm_t, temperature) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_hr_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_hr_tm_t, pin_nosecone) }, \
         { "servo_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_hr_tm_t, servo_sensor) }, \
         { "ab_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_hr_tm_t, ab_angle) }, \
         { "ab_estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_hr_tm_t, ab_estimated_cd) }, \
         { "nas_x", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_hr_tm_t, nas_x) }, \
         { "nas_y", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_hr_tm_t, nas_y) }, \
         { "nas_z", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_hr_tm_t, nas_z) }, \
         { "nas_vx", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_hr_tm_t, nas_vx) }, \
         { "nas_vy", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_hr_tm_t, nas_vy) }, \
         { "nas_vz", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_hr_tm_t, nas_vz) }, \
         { "nas_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_hr_tm_t, nas_roll) }, \
         { "nas_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_hr_tm_t, nas_pitch) }, \
         { "nas_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_hr_tm_t, nas_yaw) }, \
         { "nas_bias0", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_hr_tm_t, nas_bias0) }, \
         { "nas_bias1", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_hr_tm_t, nas_bias1) }, \
         { "nas_bias2", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_hr_tm_t, nas_bias2) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 165, offsetof(mavlink_hr_tm_t, logger_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HR_TM { \
    "HR_TM", \
    48, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_hr_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_hr_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 157, offsetof(mavlink_hr_tm_t, fmm_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_hr_tm_t, dpl_state) }, \
         { "ab_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_hr_tm_t, ab_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_hr_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_hr_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_hr_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_hr_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_hr_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_hr_tm_t, airspeed_pitot) }, \
         { "msl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_hr_tm_t, msl_altitude) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_hr_tm_t, ada_vert_speed) }, \
         { "ada_vert_accel", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_hr_tm_t, ada_vert_accel) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_hr_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_hr_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_hr_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_hr_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_hr_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_hr_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_hr_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_hr_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_hr_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_hr_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_hr_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_hr_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_hr_tm_t, gps_alt) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_hr_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_hr_tm_t, vsupply_5v) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_hr_tm_t, temperature) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_hr_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_hr_tm_t, pin_nosecone) }, \
         { "servo_sensor", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_hr_tm_t, servo_sensor) }, \
         { "ab_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_hr_tm_t, ab_angle) }, \
         { "ab_estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_hr_tm_t, ab_estimated_cd) }, \
         { "nas_x", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_hr_tm_t, nas_x) }, \
         { "nas_y", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_hr_tm_t, nas_y) }, \
         { "nas_z", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_hr_tm_t, nas_z) }, \
         { "nas_vx", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_hr_tm_t, nas_vx) }, \
         { "nas_vy", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_hr_tm_t, nas_vy) }, \
         { "nas_vz", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_hr_tm_t, nas_vz) }, \
         { "nas_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_hr_tm_t, nas_roll) }, \
         { "nas_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_hr_tm_t, nas_pitch) }, \
         { "nas_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_hr_tm_t, nas_yaw) }, \
         { "nas_bias0", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_hr_tm_t, nas_bias0) }, \
         { "nas_bias1", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_hr_tm_t, nas_bias1) }, \
         { "nas_bias2", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_hr_tm_t, nas_bias2) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 165, offsetof(mavlink_hr_tm_t, logger_error) }, \
         } \
}
#endif

/**
 * @brief Pack a hr_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param ab_state  Aerobrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param msl_altitude [m] Altitude above mean sea level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param ada_vert_accel [m/s^2] Vertical acceleration estimated by ADA
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param gps_fix  GPS fix (1 = fix, 0 = no fix)
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param servo_sensor  Servo sensor status (1 = actuated, 0 = idle)
 * @param ab_angle [deg] Aerobrakes angle
 * @param ab_estimated_cd  Estimated drag coefficient
 * @param nas_x [deg] Navigation system estimated X position (longitude)
 * @param nas_y [deg] Navigation system estimated Y position (latitude)
 * @param nas_z [m] Navigation system estimated Z position
 * @param nas_vx [m/s] Navigation system estimated north velocity
 * @param nas_vy [m/s] Navigation system estimated east velocity
 * @param nas_vz [m/s] Navigation system estimated down velocity
 * @param nas_roll [deg] Navigation system estimated attitude (roll)
 * @param nas_pitch [deg] Navigation system estimated attitude (pitch)
 * @param nas_yaw [deg] Navigation system estimated attitude (yaw)
 * @param nas_bias0  Navigation system bias
 * @param nas_bias1  Navigation system bias
 * @param nas_bias2  Navigation system bias
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t ab_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float msl_altitude, float ada_vert_speed, float ada_vert_accel, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float vbat, float vsupply_5v, float temperature, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t servo_sensor, float ab_angle, float ab_estimated_cd, float nas_x, float nas_y, float nas_z, float nas_vx, float nas_vy, float nas_vz, float nas_roll, float nas_pitch, float nas_yaw, float nas_bias0, float nas_bias1, float nas_bias2, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, msl_altitude);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, ada_vert_accel);
    _mav_put_float(buf, 40, acc_x);
    _mav_put_float(buf, 44, acc_y);
    _mav_put_float(buf, 48, acc_z);
    _mav_put_float(buf, 52, gyro_x);
    _mav_put_float(buf, 56, gyro_y);
    _mav_put_float(buf, 60, gyro_z);
    _mav_put_float(buf, 64, mag_x);
    _mav_put_float(buf, 68, mag_y);
    _mav_put_float(buf, 72, mag_z);
    _mav_put_float(buf, 76, gps_lat);
    _mav_put_float(buf, 80, gps_lon);
    _mav_put_float(buf, 84, gps_alt);
    _mav_put_float(buf, 88, vbat);
    _mav_put_float(buf, 92, vsupply_5v);
    _mav_put_float(buf, 96, temperature);
    _mav_put_float(buf, 100, ab_angle);
    _mav_put_float(buf, 104, ab_estimated_cd);
    _mav_put_float(buf, 108, nas_x);
    _mav_put_float(buf, 112, nas_y);
    _mav_put_float(buf, 116, nas_z);
    _mav_put_float(buf, 120, nas_vx);
    _mav_put_float(buf, 124, nas_vy);
    _mav_put_float(buf, 128, nas_vz);
    _mav_put_float(buf, 132, nas_roll);
    _mav_put_float(buf, 136, nas_pitch);
    _mav_put_float(buf, 140, nas_yaw);
    _mav_put_float(buf, 144, nas_bias0);
    _mav_put_float(buf, 148, nas_bias1);
    _mav_put_float(buf, 152, nas_bias2);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, ab_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, servo_sensor);
    _mav_put_int8_t(buf, 165, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_TM_LEN);
#else
    mavlink_hr_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.msl_altitude = msl_altitude;
    packet.ada_vert_speed = ada_vert_speed;
    packet.ada_vert_accel = ada_vert_accel;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ab_angle = ab_angle;
    packet.ab_estimated_cd = ab_estimated_cd;
    packet.nas_x = nas_x;
    packet.nas_y = nas_y;
    packet.nas_z = nas_z;
    packet.nas_vx = nas_vx;
    packet.nas_vy = nas_vy;
    packet.nas_vz = nas_vz;
    packet.nas_roll = nas_roll;
    packet.nas_pitch = nas_pitch;
    packet.nas_yaw = nas_yaw;
    packet.nas_bias0 = nas_bias0;
    packet.nas_bias1 = nas_bias1;
    packet.nas_bias2 = nas_bias2;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.ab_state = ab_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.servo_sensor = servo_sensor;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
}

/**
 * @brief Pack a hr_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param ab_state  Aerobrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param msl_altitude [m] Altitude above mean sea level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param ada_vert_accel [m/s^2] Vertical acceleration estimated by ADA
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param gps_fix  GPS fix (1 = fix, 0 = no fix)
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param servo_sensor  Servo sensor status (1 = actuated, 0 = idle)
 * @param ab_angle [deg] Aerobrakes angle
 * @param ab_estimated_cd  Estimated drag coefficient
 * @param nas_x [deg] Navigation system estimated X position (longitude)
 * @param nas_y [deg] Navigation system estimated Y position (latitude)
 * @param nas_z [m] Navigation system estimated Z position
 * @param nas_vx [m/s] Navigation system estimated north velocity
 * @param nas_vy [m/s] Navigation system estimated east velocity
 * @param nas_vz [m/s] Navigation system estimated down velocity
 * @param nas_roll [deg] Navigation system estimated attitude (roll)
 * @param nas_pitch [deg] Navigation system estimated attitude (pitch)
 * @param nas_yaw [deg] Navigation system estimated attitude (yaw)
 * @param nas_bias0  Navigation system bias
 * @param nas_bias1  Navigation system bias
 * @param nas_bias2  Navigation system bias
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t ada_state,uint8_t fmm_state,uint8_t dpl_state,uint8_t ab_state,uint8_t nas_state,float pressure_ada,float pressure_digi,float pressure_static,float pressure_dpl,float airspeed_pitot,float msl_altitude,float ada_vert_speed,float ada_vert_accel,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z,uint8_t gps_fix,float gps_lat,float gps_lon,float gps_alt,float vbat,float vsupply_5v,float temperature,uint8_t pin_launch,uint8_t pin_nosecone,uint8_t servo_sensor,float ab_angle,float ab_estimated_cd,float nas_x,float nas_y,float nas_z,float nas_vx,float nas_vy,float nas_vz,float nas_roll,float nas_pitch,float nas_yaw,float nas_bias0,float nas_bias1,float nas_bias2,int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, msl_altitude);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, ada_vert_accel);
    _mav_put_float(buf, 40, acc_x);
    _mav_put_float(buf, 44, acc_y);
    _mav_put_float(buf, 48, acc_z);
    _mav_put_float(buf, 52, gyro_x);
    _mav_put_float(buf, 56, gyro_y);
    _mav_put_float(buf, 60, gyro_z);
    _mav_put_float(buf, 64, mag_x);
    _mav_put_float(buf, 68, mag_y);
    _mav_put_float(buf, 72, mag_z);
    _mav_put_float(buf, 76, gps_lat);
    _mav_put_float(buf, 80, gps_lon);
    _mav_put_float(buf, 84, gps_alt);
    _mav_put_float(buf, 88, vbat);
    _mav_put_float(buf, 92, vsupply_5v);
    _mav_put_float(buf, 96, temperature);
    _mav_put_float(buf, 100, ab_angle);
    _mav_put_float(buf, 104, ab_estimated_cd);
    _mav_put_float(buf, 108, nas_x);
    _mav_put_float(buf, 112, nas_y);
    _mav_put_float(buf, 116, nas_z);
    _mav_put_float(buf, 120, nas_vx);
    _mav_put_float(buf, 124, nas_vy);
    _mav_put_float(buf, 128, nas_vz);
    _mav_put_float(buf, 132, nas_roll);
    _mav_put_float(buf, 136, nas_pitch);
    _mav_put_float(buf, 140, nas_yaw);
    _mav_put_float(buf, 144, nas_bias0);
    _mav_put_float(buf, 148, nas_bias1);
    _mav_put_float(buf, 152, nas_bias2);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, ab_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, servo_sensor);
    _mav_put_int8_t(buf, 165, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_TM_LEN);
#else
    mavlink_hr_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.msl_altitude = msl_altitude;
    packet.ada_vert_speed = ada_vert_speed;
    packet.ada_vert_accel = ada_vert_accel;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ab_angle = ab_angle;
    packet.ab_estimated_cd = ab_estimated_cd;
    packet.nas_x = nas_x;
    packet.nas_y = nas_y;
    packet.nas_z = nas_z;
    packet.nas_vx = nas_vx;
    packet.nas_vy = nas_vy;
    packet.nas_vz = nas_vz;
    packet.nas_roll = nas_roll;
    packet.nas_pitch = nas_pitch;
    packet.nas_yaw = nas_yaw;
    packet.nas_bias0 = nas_bias0;
    packet.nas_bias1 = nas_bias1;
    packet.nas_bias2 = nas_bias2;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.ab_state = ab_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.servo_sensor = servo_sensor;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
}

/**
 * @brief Encode a hr_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hr_tm_t* hr_tm)
{
    return mavlink_msg_hr_tm_pack(system_id, component_id, msg, hr_tm->timestamp, hr_tm->ada_state, hr_tm->fmm_state, hr_tm->dpl_state, hr_tm->ab_state, hr_tm->nas_state, hr_tm->pressure_ada, hr_tm->pressure_digi, hr_tm->pressure_static, hr_tm->pressure_dpl, hr_tm->airspeed_pitot, hr_tm->msl_altitude, hr_tm->ada_vert_speed, hr_tm->ada_vert_accel, hr_tm->acc_x, hr_tm->acc_y, hr_tm->acc_z, hr_tm->gyro_x, hr_tm->gyro_y, hr_tm->gyro_z, hr_tm->mag_x, hr_tm->mag_y, hr_tm->mag_z, hr_tm->gps_fix, hr_tm->gps_lat, hr_tm->gps_lon, hr_tm->gps_alt, hr_tm->vbat, hr_tm->vsupply_5v, hr_tm->temperature, hr_tm->pin_launch, hr_tm->pin_nosecone, hr_tm->servo_sensor, hr_tm->ab_angle, hr_tm->ab_estimated_cd, hr_tm->nas_x, hr_tm->nas_y, hr_tm->nas_z, hr_tm->nas_vx, hr_tm->nas_vy, hr_tm->nas_vz, hr_tm->nas_roll, hr_tm->nas_pitch, hr_tm->nas_yaw, hr_tm->nas_bias0, hr_tm->nas_bias1, hr_tm->nas_bias2, hr_tm->logger_error);
}

/**
 * @brief Encode a hr_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hr_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hr_tm_t* hr_tm)
{
    return mavlink_msg_hr_tm_pack_chan(system_id, component_id, chan, msg, hr_tm->timestamp, hr_tm->ada_state, hr_tm->fmm_state, hr_tm->dpl_state, hr_tm->ab_state, hr_tm->nas_state, hr_tm->pressure_ada, hr_tm->pressure_digi, hr_tm->pressure_static, hr_tm->pressure_dpl, hr_tm->airspeed_pitot, hr_tm->msl_altitude, hr_tm->ada_vert_speed, hr_tm->ada_vert_accel, hr_tm->acc_x, hr_tm->acc_y, hr_tm->acc_z, hr_tm->gyro_x, hr_tm->gyro_y, hr_tm->gyro_z, hr_tm->mag_x, hr_tm->mag_y, hr_tm->mag_z, hr_tm->gps_fix, hr_tm->gps_lat, hr_tm->gps_lon, hr_tm->gps_alt, hr_tm->vbat, hr_tm->vsupply_5v, hr_tm->temperature, hr_tm->pin_launch, hr_tm->pin_nosecone, hr_tm->servo_sensor, hr_tm->ab_angle, hr_tm->ab_estimated_cd, hr_tm->nas_x, hr_tm->nas_y, hr_tm->nas_z, hr_tm->nas_vx, hr_tm->nas_vy, hr_tm->nas_vz, hr_tm->nas_roll, hr_tm->nas_pitch, hr_tm->nas_yaw, hr_tm->nas_bias0, hr_tm->nas_bias1, hr_tm->nas_bias2, hr_tm->logger_error);
}

/**
 * @brief Send a hr_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param ab_state  Aerobrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param msl_altitude [m] Altitude above mean sea level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param ada_vert_accel [m/s^2] Vertical acceleration estimated by ADA
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param gps_fix  GPS fix (1 = fix, 0 = no fix)
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param servo_sensor  Servo sensor status (1 = actuated, 0 = idle)
 * @param ab_angle [deg] Aerobrakes angle
 * @param ab_estimated_cd  Estimated drag coefficient
 * @param nas_x [deg] Navigation system estimated X position (longitude)
 * @param nas_y [deg] Navigation system estimated Y position (latitude)
 * @param nas_z [m] Navigation system estimated Z position
 * @param nas_vx [m/s] Navigation system estimated north velocity
 * @param nas_vy [m/s] Navigation system estimated east velocity
 * @param nas_vz [m/s] Navigation system estimated down velocity
 * @param nas_roll [deg] Navigation system estimated attitude (roll)
 * @param nas_pitch [deg] Navigation system estimated attitude (pitch)
 * @param nas_yaw [deg] Navigation system estimated attitude (yaw)
 * @param nas_bias0  Navigation system bias
 * @param nas_bias1  Navigation system bias
 * @param nas_bias2  Navigation system bias
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hr_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t ab_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float msl_altitude, float ada_vert_speed, float ada_vert_accel, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float vbat, float vsupply_5v, float temperature, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t servo_sensor, float ab_angle, float ab_estimated_cd, float nas_x, float nas_y, float nas_z, float nas_vx, float nas_vy, float nas_vz, float nas_roll, float nas_pitch, float nas_yaw, float nas_bias0, float nas_bias1, float nas_bias2, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, msl_altitude);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, ada_vert_accel);
    _mav_put_float(buf, 40, acc_x);
    _mav_put_float(buf, 44, acc_y);
    _mav_put_float(buf, 48, acc_z);
    _mav_put_float(buf, 52, gyro_x);
    _mav_put_float(buf, 56, gyro_y);
    _mav_put_float(buf, 60, gyro_z);
    _mav_put_float(buf, 64, mag_x);
    _mav_put_float(buf, 68, mag_y);
    _mav_put_float(buf, 72, mag_z);
    _mav_put_float(buf, 76, gps_lat);
    _mav_put_float(buf, 80, gps_lon);
    _mav_put_float(buf, 84, gps_alt);
    _mav_put_float(buf, 88, vbat);
    _mav_put_float(buf, 92, vsupply_5v);
    _mav_put_float(buf, 96, temperature);
    _mav_put_float(buf, 100, ab_angle);
    _mav_put_float(buf, 104, ab_estimated_cd);
    _mav_put_float(buf, 108, nas_x);
    _mav_put_float(buf, 112, nas_y);
    _mav_put_float(buf, 116, nas_z);
    _mav_put_float(buf, 120, nas_vx);
    _mav_put_float(buf, 124, nas_vy);
    _mav_put_float(buf, 128, nas_vz);
    _mav_put_float(buf, 132, nas_roll);
    _mav_put_float(buf, 136, nas_pitch);
    _mav_put_float(buf, 140, nas_yaw);
    _mav_put_float(buf, 144, nas_bias0);
    _mav_put_float(buf, 148, nas_bias1);
    _mav_put_float(buf, 152, nas_bias2);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, ab_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, servo_sensor);
    _mav_put_int8_t(buf, 165, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, buf, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#else
    mavlink_hr_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.msl_altitude = msl_altitude;
    packet.ada_vert_speed = ada_vert_speed;
    packet.ada_vert_accel = ada_vert_accel;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.vbat = vbat;
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ab_angle = ab_angle;
    packet.ab_estimated_cd = ab_estimated_cd;
    packet.nas_x = nas_x;
    packet.nas_y = nas_y;
    packet.nas_z = nas_z;
    packet.nas_vx = nas_vx;
    packet.nas_vy = nas_vy;
    packet.nas_vz = nas_vz;
    packet.nas_roll = nas_roll;
    packet.nas_pitch = nas_pitch;
    packet.nas_yaw = nas_yaw;
    packet.nas_bias0 = nas_bias0;
    packet.nas_bias1 = nas_bias1;
    packet.nas_bias2 = nas_bias2;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.ab_state = ab_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.servo_sensor = servo_sensor;
    packet.logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)&packet, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}

/**
 * @brief Send a hr_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hr_tm_send_struct(mavlink_channel_t chan, const mavlink_hr_tm_t* hr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hr_tm_send(chan, hr_tm->timestamp, hr_tm->ada_state, hr_tm->fmm_state, hr_tm->dpl_state, hr_tm->ab_state, hr_tm->nas_state, hr_tm->pressure_ada, hr_tm->pressure_digi, hr_tm->pressure_static, hr_tm->pressure_dpl, hr_tm->airspeed_pitot, hr_tm->msl_altitude, hr_tm->ada_vert_speed, hr_tm->ada_vert_accel, hr_tm->acc_x, hr_tm->acc_y, hr_tm->acc_z, hr_tm->gyro_x, hr_tm->gyro_y, hr_tm->gyro_z, hr_tm->mag_x, hr_tm->mag_y, hr_tm->mag_z, hr_tm->gps_fix, hr_tm->gps_lat, hr_tm->gps_lon, hr_tm->gps_alt, hr_tm->vbat, hr_tm->vsupply_5v, hr_tm->temperature, hr_tm->pin_launch, hr_tm->pin_nosecone, hr_tm->servo_sensor, hr_tm->ab_angle, hr_tm->ab_estimated_cd, hr_tm->nas_x, hr_tm->nas_y, hr_tm->nas_z, hr_tm->nas_vx, hr_tm->nas_vy, hr_tm->nas_vz, hr_tm->nas_roll, hr_tm->nas_pitch, hr_tm->nas_yaw, hr_tm->nas_bias0, hr_tm->nas_bias1, hr_tm->nas_bias2, hr_tm->logger_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)hr_tm, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_HR_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hr_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t ab_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float msl_altitude, float ada_vert_speed, float ada_vert_accel, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float vbat, float vsupply_5v, float temperature, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t servo_sensor, float ab_angle, float ab_estimated_cd, float nas_x, float nas_y, float nas_z, float nas_vx, float nas_vy, float nas_vz, float nas_roll, float nas_pitch, float nas_yaw, float nas_bias0, float nas_bias1, float nas_bias2, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, msl_altitude);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, ada_vert_accel);
    _mav_put_float(buf, 40, acc_x);
    _mav_put_float(buf, 44, acc_y);
    _mav_put_float(buf, 48, acc_z);
    _mav_put_float(buf, 52, gyro_x);
    _mav_put_float(buf, 56, gyro_y);
    _mav_put_float(buf, 60, gyro_z);
    _mav_put_float(buf, 64, mag_x);
    _mav_put_float(buf, 68, mag_y);
    _mav_put_float(buf, 72, mag_z);
    _mav_put_float(buf, 76, gps_lat);
    _mav_put_float(buf, 80, gps_lon);
    _mav_put_float(buf, 84, gps_alt);
    _mav_put_float(buf, 88, vbat);
    _mav_put_float(buf, 92, vsupply_5v);
    _mav_put_float(buf, 96, temperature);
    _mav_put_float(buf, 100, ab_angle);
    _mav_put_float(buf, 104, ab_estimated_cd);
    _mav_put_float(buf, 108, nas_x);
    _mav_put_float(buf, 112, nas_y);
    _mav_put_float(buf, 116, nas_z);
    _mav_put_float(buf, 120, nas_vx);
    _mav_put_float(buf, 124, nas_vy);
    _mav_put_float(buf, 128, nas_vz);
    _mav_put_float(buf, 132, nas_roll);
    _mav_put_float(buf, 136, nas_pitch);
    _mav_put_float(buf, 140, nas_yaw);
    _mav_put_float(buf, 144, nas_bias0);
    _mav_put_float(buf, 148, nas_bias1);
    _mav_put_float(buf, 152, nas_bias2);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, ab_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, servo_sensor);
    _mav_put_int8_t(buf, 165, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, buf, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#else
    mavlink_hr_tm_t *packet = (mavlink_hr_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure_ada = pressure_ada;
    packet->pressure_digi = pressure_digi;
    packet->pressure_static = pressure_static;
    packet->pressure_dpl = pressure_dpl;
    packet->airspeed_pitot = airspeed_pitot;
    packet->msl_altitude = msl_altitude;
    packet->ada_vert_speed = ada_vert_speed;
    packet->ada_vert_accel = ada_vert_accel;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    packet->gps_lat = gps_lat;
    packet->gps_lon = gps_lon;
    packet->gps_alt = gps_alt;
    packet->vbat = vbat;
    packet->vsupply_5v = vsupply_5v;
    packet->temperature = temperature;
    packet->ab_angle = ab_angle;
    packet->ab_estimated_cd = ab_estimated_cd;
    packet->nas_x = nas_x;
    packet->nas_y = nas_y;
    packet->nas_z = nas_z;
    packet->nas_vx = nas_vx;
    packet->nas_vy = nas_vy;
    packet->nas_vz = nas_vz;
    packet->nas_roll = nas_roll;
    packet->nas_pitch = nas_pitch;
    packet->nas_yaw = nas_yaw;
    packet->nas_bias0 = nas_bias0;
    packet->nas_bias1 = nas_bias1;
    packet->nas_bias2 = nas_bias2;
    packet->ada_state = ada_state;
    packet->fmm_state = fmm_state;
    packet->dpl_state = dpl_state;
    packet->ab_state = ab_state;
    packet->nas_state = nas_state;
    packet->gps_fix = gps_fix;
    packet->pin_launch = pin_launch;
    packet->pin_nosecone = pin_nosecone;
    packet->servo_sensor = servo_sensor;
    packet->logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_TM, (const char *)packet, MAVLINK_MSG_ID_HR_TM_MIN_LEN, MAVLINK_MSG_ID_HR_TM_LEN, MAVLINK_MSG_ID_HR_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE HR_TM UNPACKING


/**
 * @brief Get field timestamp from hr_tm message
 *
 * @return [ms] Timestamp in milliseconds
 */
static inline uint64_t mavlink_msg_hr_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ada_state from hr_tm message
 *
 * @return  ADA Controller State
 */
static inline uint8_t mavlink_msg_hr_tm_get_ada_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  156);
}

/**
 * @brief Get field fmm_state from hr_tm message
 *
 * @return  Flight Mode Manager State
 */
static inline uint8_t mavlink_msg_hr_tm_get_fmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  157);
}

/**
 * @brief Get field dpl_state from hr_tm message
 *
 * @return  Deployment Controller State
 */
static inline uint8_t mavlink_msg_hr_tm_get_dpl_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  158);
}

/**
 * @brief Get field ab_state from hr_tm message
 *
 * @return  Aerobrake FSM state
 */
static inline uint8_t mavlink_msg_hr_tm_get_ab_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  159);
}

/**
 * @brief Get field nas_state from hr_tm message
 *
 * @return  Navigation System FSM State
 */
static inline uint8_t mavlink_msg_hr_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  160);
}

/**
 * @brief Get field pressure_ada from hr_tm message
 *
 * @return [Pa] Atmospheric pressure estimated by ADA
 */
static inline float mavlink_msg_hr_tm_get_pressure_ada(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure_digi from hr_tm message
 *
 * @return [Pa] Pressure from digital sensor
 */
static inline float mavlink_msg_hr_tm_get_pressure_digi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pressure_static from hr_tm message
 *
 * @return [Pa] Pressure from static port
 */
static inline float mavlink_msg_hr_tm_get_pressure_static(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure_dpl from hr_tm message
 *
 * @return [Pa] Pressure from deployment vane sensor
 */
static inline float mavlink_msg_hr_tm_get_pressure_dpl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field airspeed_pitot from hr_tm message
 *
 * @return [m/s] Pitot airspeed
 */
static inline float mavlink_msg_hr_tm_get_airspeed_pitot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field msl_altitude from hr_tm message
 *
 * @return [m] Altitude above mean sea level
 */
static inline float mavlink_msg_hr_tm_get_msl_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ada_vert_speed from hr_tm message
 *
 * @return [m/s] Vertical speed estimated by ADA
 */
static inline float mavlink_msg_hr_tm_get_ada_vert_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ada_vert_accel from hr_tm message
 *
 * @return [m/s^2] Vertical acceleration estimated by ADA
 */
static inline float mavlink_msg_hr_tm_get_ada_vert_accel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field acc_x from hr_tm message
 *
 * @return [m/s^2] Acceleration on X axis (body)
 */
static inline float mavlink_msg_hr_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field acc_y from hr_tm message
 *
 * @return [m/s^2] Acceleration on Y axis (body)
 */
static inline float mavlink_msg_hr_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field acc_z from hr_tm message
 *
 * @return [m/s^2] Acceleration on Z axis (body)
 */
static inline float mavlink_msg_hr_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gyro_x from hr_tm message
 *
 * @return [rad/s] Angular speed on X axis (body)
 */
static inline float mavlink_msg_hr_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field gyro_y from hr_tm message
 *
 * @return [rad/s] Angular speed on Y axis (body)
 */
static inline float mavlink_msg_hr_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field gyro_z from hr_tm message
 *
 * @return [rad/s] Angular speed on Z axis (body)
 */
static inline float mavlink_msg_hr_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mag_x from hr_tm message
 *
 * @return [uT] Magnetic field on X axis (body)
 */
static inline float mavlink_msg_hr_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field mag_y from hr_tm message
 *
 * @return [uT] Magnetic field on Y axis (body)
 */
static inline float mavlink_msg_hr_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field mag_z from hr_tm message
 *
 * @return [uT] Magnetic field on Z axis (body)
 */
static inline float mavlink_msg_hr_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field gps_fix from hr_tm message
 *
 * @return  GPS fix (1 = fix, 0 = no fix)
 */
static inline uint8_t mavlink_msg_hr_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  161);
}

/**
 * @brief Get field gps_lat from hr_tm message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_hr_tm_get_gps_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field gps_lon from hr_tm message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_hr_tm_get_gps_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field gps_alt from hr_tm message
 *
 * @return [m] GPS Altitude
 */
static inline float mavlink_msg_hr_tm_get_gps_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field vbat from hr_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_hr_tm_get_vbat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field vsupply_5v from hr_tm message
 *
 * @return [V] Power supply 5V
 */
static inline float mavlink_msg_hr_tm_get_vsupply_5v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field temperature from hr_tm message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_hr_tm_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field pin_launch from hr_tm message
 *
 * @return  Launch pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_hr_tm_get_pin_launch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  162);
}

/**
 * @brief Get field pin_nosecone from hr_tm message
 *
 * @return  Nosecone pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_hr_tm_get_pin_nosecone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  163);
}

/**
 * @brief Get field servo_sensor from hr_tm message
 *
 * @return  Servo sensor status (1 = actuated, 0 = idle)
 */
static inline uint8_t mavlink_msg_hr_tm_get_servo_sensor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  164);
}

/**
 * @brief Get field ab_angle from hr_tm message
 *
 * @return [deg] Aerobrakes angle
 */
static inline float mavlink_msg_hr_tm_get_ab_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field ab_estimated_cd from hr_tm message
 *
 * @return  Estimated drag coefficient
 */
static inline float mavlink_msg_hr_tm_get_ab_estimated_cd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field nas_x from hr_tm message
 *
 * @return [deg] Navigation system estimated X position (longitude)
 */
static inline float mavlink_msg_hr_tm_get_nas_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field nas_y from hr_tm message
 *
 * @return [deg] Navigation system estimated Y position (latitude)
 */
static inline float mavlink_msg_hr_tm_get_nas_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field nas_z from hr_tm message
 *
 * @return [m] Navigation system estimated Z position
 */
static inline float mavlink_msg_hr_tm_get_nas_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field nas_vx from hr_tm message
 *
 * @return [m/s] Navigation system estimated north velocity
 */
static inline float mavlink_msg_hr_tm_get_nas_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field nas_vy from hr_tm message
 *
 * @return [m/s] Navigation system estimated east velocity
 */
static inline float mavlink_msg_hr_tm_get_nas_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field nas_vz from hr_tm message
 *
 * @return [m/s] Navigation system estimated down velocity
 */
static inline float mavlink_msg_hr_tm_get_nas_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field nas_roll from hr_tm message
 *
 * @return [deg] Navigation system estimated attitude (roll)
 */
static inline float mavlink_msg_hr_tm_get_nas_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field nas_pitch from hr_tm message
 *
 * @return [deg] Navigation system estimated attitude (pitch)
 */
static inline float mavlink_msg_hr_tm_get_nas_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field nas_yaw from hr_tm message
 *
 * @return [deg] Navigation system estimated attitude (yaw)
 */
static inline float mavlink_msg_hr_tm_get_nas_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field nas_bias0 from hr_tm message
 *
 * @return  Navigation system bias
 */
static inline float mavlink_msg_hr_tm_get_nas_bias0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field nas_bias1 from hr_tm message
 *
 * @return  Navigation system bias
 */
static inline float mavlink_msg_hr_tm_get_nas_bias1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field nas_bias2 from hr_tm message
 *
 * @return  Navigation system bias
 */
static inline float mavlink_msg_hr_tm_get_nas_bias2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  152);
}

/**
 * @brief Get field logger_error from hr_tm message
 *
 * @return  Logger error (0 = no error, -1 otherwise)
 */
static inline int8_t mavlink_msg_hr_tm_get_logger_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  165);
}

/**
 * @brief Decode a hr_tm message into a struct
 *
 * @param msg The message to decode
 * @param hr_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_hr_tm_decode(const mavlink_message_t* msg, mavlink_hr_tm_t* hr_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hr_tm->timestamp = mavlink_msg_hr_tm_get_timestamp(msg);
    hr_tm->pressure_ada = mavlink_msg_hr_tm_get_pressure_ada(msg);
    hr_tm->pressure_digi = mavlink_msg_hr_tm_get_pressure_digi(msg);
    hr_tm->pressure_static = mavlink_msg_hr_tm_get_pressure_static(msg);
    hr_tm->pressure_dpl = mavlink_msg_hr_tm_get_pressure_dpl(msg);
    hr_tm->airspeed_pitot = mavlink_msg_hr_tm_get_airspeed_pitot(msg);
    hr_tm->msl_altitude = mavlink_msg_hr_tm_get_msl_altitude(msg);
    hr_tm->ada_vert_speed = mavlink_msg_hr_tm_get_ada_vert_speed(msg);
    hr_tm->ada_vert_accel = mavlink_msg_hr_tm_get_ada_vert_accel(msg);
    hr_tm->acc_x = mavlink_msg_hr_tm_get_acc_x(msg);
    hr_tm->acc_y = mavlink_msg_hr_tm_get_acc_y(msg);
    hr_tm->acc_z = mavlink_msg_hr_tm_get_acc_z(msg);
    hr_tm->gyro_x = mavlink_msg_hr_tm_get_gyro_x(msg);
    hr_tm->gyro_y = mavlink_msg_hr_tm_get_gyro_y(msg);
    hr_tm->gyro_z = mavlink_msg_hr_tm_get_gyro_z(msg);
    hr_tm->mag_x = mavlink_msg_hr_tm_get_mag_x(msg);
    hr_tm->mag_y = mavlink_msg_hr_tm_get_mag_y(msg);
    hr_tm->mag_z = mavlink_msg_hr_tm_get_mag_z(msg);
    hr_tm->gps_lat = mavlink_msg_hr_tm_get_gps_lat(msg);
    hr_tm->gps_lon = mavlink_msg_hr_tm_get_gps_lon(msg);
    hr_tm->gps_alt = mavlink_msg_hr_tm_get_gps_alt(msg);
    hr_tm->vbat = mavlink_msg_hr_tm_get_vbat(msg);
    hr_tm->vsupply_5v = mavlink_msg_hr_tm_get_vsupply_5v(msg);
    hr_tm->temperature = mavlink_msg_hr_tm_get_temperature(msg);
    hr_tm->ab_angle = mavlink_msg_hr_tm_get_ab_angle(msg);
    hr_tm->ab_estimated_cd = mavlink_msg_hr_tm_get_ab_estimated_cd(msg);
    hr_tm->nas_x = mavlink_msg_hr_tm_get_nas_x(msg);
    hr_tm->nas_y = mavlink_msg_hr_tm_get_nas_y(msg);
    hr_tm->nas_z = mavlink_msg_hr_tm_get_nas_z(msg);
    hr_tm->nas_vx = mavlink_msg_hr_tm_get_nas_vx(msg);
    hr_tm->nas_vy = mavlink_msg_hr_tm_get_nas_vy(msg);
    hr_tm->nas_vz = mavlink_msg_hr_tm_get_nas_vz(msg);
    hr_tm->nas_roll = mavlink_msg_hr_tm_get_nas_roll(msg);
    hr_tm->nas_pitch = mavlink_msg_hr_tm_get_nas_pitch(msg);
    hr_tm->nas_yaw = mavlink_msg_hr_tm_get_nas_yaw(msg);
    hr_tm->nas_bias0 = mavlink_msg_hr_tm_get_nas_bias0(msg);
    hr_tm->nas_bias1 = mavlink_msg_hr_tm_get_nas_bias1(msg);
    hr_tm->nas_bias2 = mavlink_msg_hr_tm_get_nas_bias2(msg);
    hr_tm->ada_state = mavlink_msg_hr_tm_get_ada_state(msg);
    hr_tm->fmm_state = mavlink_msg_hr_tm_get_fmm_state(msg);
    hr_tm->dpl_state = mavlink_msg_hr_tm_get_dpl_state(msg);
    hr_tm->ab_state = mavlink_msg_hr_tm_get_ab_state(msg);
    hr_tm->nas_state = mavlink_msg_hr_tm_get_nas_state(msg);
    hr_tm->gps_fix = mavlink_msg_hr_tm_get_gps_fix(msg);
    hr_tm->pin_launch = mavlink_msg_hr_tm_get_pin_launch(msg);
    hr_tm->pin_nosecone = mavlink_msg_hr_tm_get_pin_nosecone(msg);
    hr_tm->servo_sensor = mavlink_msg_hr_tm_get_servo_sensor(msg);
    hr_tm->logger_error = mavlink_msg_hr_tm_get_logger_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HR_TM_LEN? msg->len : MAVLINK_MSG_ID_HR_TM_LEN;
        memset(hr_tm, 0, MAVLINK_MSG_ID_HR_TM_LEN);
    memcpy(hr_tm, _MAV_PAYLOAD(msg), len);
#endif
}
