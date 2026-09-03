#pragma once
// MESSAGE ROCKET_FLIGHT_TM PACKING

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM 207


typedef struct __mavlink_rocket_flight_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in milliseconds*/
 float pressure_ada; /*< [Pa] Atmospheric pressure estimated by ADA*/
 float pressure_digi; /*< [Pa] Pressure from digital sensor*/
 float pressure_static; /*< [Pa] Pressure from static port*/
 float pressure_dpl; /*< [Pa] Pressure from deployment vane sensor*/
 float airspeed_pitot; /*< [m/s] Pitot airspeed*/
 float altitude_agl; /*< [m] Altitude above ground level*/
 float ada_vert_speed; /*< [m/s] Vertical speed estimated by ADA*/
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
 float abk_angle; /*< [deg] Air Brakes angle*/
 float abk_estimated_cd; /*<  Estimated drag coefficient*/
 float parachute_load; /*<  Parachute load cell value*/
 float nas_n; /*< [deg] Navigation system estimated noth position*/
 float nas_e; /*< [deg] Navigation system estimated east position*/
 float nas_d; /*< [m] Navigation system estimated down position*/
 float nas_vn; /*< [m/s] Navigation system estimated north velocity*/
 float nas_ve; /*< [m/s] Navigation system estimated east velocity*/
 float nas_vd; /*< [m/s] Navigation system estimated down velocity*/
 float nas_qx; /*< [deg] Navigation system estimated attitude (qx)*/
 float nas_qy; /*< [deg] Navigation system estimated attitude (qy)*/
 float nas_qz; /*< [deg] Navigation system estimated attitude (qz)*/
 float nas_qw; /*< [deg] Navigation system estimated attitude (qw)*/
 float nas_bias_x; /*<  Navigation system gyroscope bias on x axis*/
 float nas_bias_y; /*<  Navigation system gyroscope bias on x axis*/
 float nas_bias_z; /*<  Navigation system gyroscope bias on x axis*/
 float vbat; /*< [V] Battery voltage*/
 float temperature; /*< [degC] Temperature*/
 uint8_t ada_state; /*<  ADA Controller State*/
 uint8_t fmm_state; /*<  Flight Mode Manager State*/
 uint8_t dpl_state; /*<  Deployment Controller State*/
 uint8_t abk_state; /*<  Airbrake FSM state*/
 uint8_t nas_state; /*<  Navigation System FSM State*/
 uint8_t gps_fix; /*<  GPS fix (1 = fix, 0 = no fix)*/
 uint8_t pin_launch; /*<  Launch pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_nosecone; /*<  Nosecone pin status (1 = connected, 0 = disconnected)*/
 uint8_t pin_expulsion; /*<  Servo sensor status (1 = actuated, 0 = idle)*/
 uint8_t cutter_presence; /*<  Cutter presence status (1 = present, 0 = missing)*/
 int8_t logger_error; /*<  Logger error (0 = no error, -1 otherwise)*/
} mavlink_rocket_flight_tm_t;

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN 167
#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN 167
#define MAVLINK_MSG_ID_207_LEN 167
#define MAVLINK_MSG_ID_207_MIN_LEN 167

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC 184
#define MAVLINK_MSG_ID_207_CRC 184



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM { \
    207, \
    "ROCKET_FLIGHT_TM", \
    49, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_flight_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_rocket_flight_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 157, offsetof(mavlink_rocket_flight_tm_t, fmm_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_rocket_flight_tm_t, dpl_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_rocket_flight_tm_t, abk_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_rocket_flight_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rocket_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rocket_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rocket_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rocket_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rocket_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rocket_flight_tm_t, altitude_agl) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rocket_flight_tm_t, ada_vert_speed) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rocket_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_flight_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_rocket_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_flight_tm_t, gps_alt) }, \
         { "abk_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_flight_tm_t, abk_angle) }, \
         { "abk_estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_flight_tm_t, abk_estimated_cd) }, \
         { "parachute_load", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_flight_tm_t, parachute_load) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_flight_tm_t, nas_bias_z) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_rocket_flight_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_rocket_flight_tm_t, pin_nosecone) }, \
         { "pin_expulsion", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_rocket_flight_tm_t, pin_expulsion) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 165, offsetof(mavlink_rocket_flight_tm_t, cutter_presence) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_flight_tm_t, vbat) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_rocket_flight_tm_t, temperature) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 166, offsetof(mavlink_rocket_flight_tm_t, logger_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM { \
    "ROCKET_FLIGHT_TM", \
    49, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_flight_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_rocket_flight_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 157, offsetof(mavlink_rocket_flight_tm_t, fmm_state) }, \
         { "dpl_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 158, offsetof(mavlink_rocket_flight_tm_t, dpl_state) }, \
         { "abk_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 159, offsetof(mavlink_rocket_flight_tm_t, abk_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 160, offsetof(mavlink_rocket_flight_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rocket_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rocket_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rocket_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rocket_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rocket_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rocket_flight_tm_t, altitude_agl) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rocket_flight_tm_t, ada_vert_speed) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rocket_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_flight_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 161, offsetof(mavlink_rocket_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_flight_tm_t, gps_alt) }, \
         { "abk_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_flight_tm_t, abk_angle) }, \
         { "abk_estimated_cd", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_flight_tm_t, abk_estimated_cd) }, \
         { "parachute_load", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_flight_tm_t, parachute_load) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_flight_tm_t, nas_bias_z) }, \
         { "pin_launch", NULL, MAVLINK_TYPE_UINT8_T, 0, 162, offsetof(mavlink_rocket_flight_tm_t, pin_launch) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 163, offsetof(mavlink_rocket_flight_tm_t, pin_nosecone) }, \
         { "pin_expulsion", NULL, MAVLINK_TYPE_UINT8_T, 0, 164, offsetof(mavlink_rocket_flight_tm_t, pin_expulsion) }, \
         { "cutter_presence", NULL, MAVLINK_TYPE_UINT8_T, 0, 165, offsetof(mavlink_rocket_flight_tm_t, cutter_presence) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_flight_tm_t, vbat) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_rocket_flight_tm_t, temperature) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 166, offsetof(mavlink_rocket_flight_tm_t, logger_error) }, \
         } \
}
#endif

/**
 * @brief Pack a rocket_flight_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
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
 * @param abk_angle [deg] Air Brakes angle
 * @param abk_estimated_cd  Estimated drag coefficient
 * @param parachute_load  Parachute load cell value
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param vbat [V] Battery voltage
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t abk_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float abk_angle, float abk_estimated_cd, float parachute_load, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, float vbat, float temperature, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, acc_x);
    _mav_put_float(buf, 40, acc_y);
    _mav_put_float(buf, 44, acc_z);
    _mav_put_float(buf, 48, gyro_x);
    _mav_put_float(buf, 52, gyro_y);
    _mav_put_float(buf, 56, gyro_z);
    _mav_put_float(buf, 60, mag_x);
    _mav_put_float(buf, 64, mag_y);
    _mav_put_float(buf, 68, mag_z);
    _mav_put_float(buf, 72, gps_lat);
    _mav_put_float(buf, 76, gps_lon);
    _mav_put_float(buf, 80, gps_alt);
    _mav_put_float(buf, 84, abk_angle);
    _mav_put_float(buf, 88, abk_estimated_cd);
    _mav_put_float(buf, 92, parachute_load);
    _mav_put_float(buf, 96, nas_n);
    _mav_put_float(buf, 100, nas_e);
    _mav_put_float(buf, 104, nas_d);
    _mav_put_float(buf, 108, nas_vn);
    _mav_put_float(buf, 112, nas_ve);
    _mav_put_float(buf, 116, nas_vd);
    _mav_put_float(buf, 120, nas_qx);
    _mav_put_float(buf, 124, nas_qy);
    _mav_put_float(buf, 128, nas_qz);
    _mav_put_float(buf, 132, nas_qw);
    _mav_put_float(buf, 136, nas_bias_x);
    _mav_put_float(buf, 140, nas_bias_y);
    _mav_put_float(buf, 144, nas_bias_z);
    _mav_put_float(buf, 148, vbat);
    _mav_put_float(buf, 152, temperature);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, abk_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, pin_expulsion);
    _mav_put_uint8_t(buf, 165, cutter_presence);
    _mav_put_int8_t(buf, 166, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
#else
    mavlink_rocket_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
    packet.ada_vert_speed = ada_vert_speed;
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
    packet.abk_angle = abk_angle;
    packet.abk_estimated_cd = abk_estimated_cd;
    packet.parachute_load = parachute_load;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.vbat = vbat;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_FLIGHT_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
}

/**
 * @brief Pack a rocket_flight_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
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
 * @param abk_angle [deg] Air Brakes angle
 * @param abk_estimated_cd  Estimated drag coefficient
 * @param parachute_load  Parachute load cell value
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param vbat [V] Battery voltage
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t ada_state,uint8_t fmm_state,uint8_t dpl_state,uint8_t abk_state,uint8_t nas_state,float pressure_ada,float pressure_digi,float pressure_static,float pressure_dpl,float airspeed_pitot,float altitude_agl,float ada_vert_speed,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z,uint8_t gps_fix,float gps_lat,float gps_lon,float gps_alt,float abk_angle,float abk_estimated_cd,float parachute_load,float nas_n,float nas_e,float nas_d,float nas_vn,float nas_ve,float nas_vd,float nas_qx,float nas_qy,float nas_qz,float nas_qw,float nas_bias_x,float nas_bias_y,float nas_bias_z,uint8_t pin_launch,uint8_t pin_nosecone,uint8_t pin_expulsion,uint8_t cutter_presence,float vbat,float temperature,int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, acc_x);
    _mav_put_float(buf, 40, acc_y);
    _mav_put_float(buf, 44, acc_z);
    _mav_put_float(buf, 48, gyro_x);
    _mav_put_float(buf, 52, gyro_y);
    _mav_put_float(buf, 56, gyro_z);
    _mav_put_float(buf, 60, mag_x);
    _mav_put_float(buf, 64, mag_y);
    _mav_put_float(buf, 68, mag_z);
    _mav_put_float(buf, 72, gps_lat);
    _mav_put_float(buf, 76, gps_lon);
    _mav_put_float(buf, 80, gps_alt);
    _mav_put_float(buf, 84, abk_angle);
    _mav_put_float(buf, 88, abk_estimated_cd);
    _mav_put_float(buf, 92, parachute_load);
    _mav_put_float(buf, 96, nas_n);
    _mav_put_float(buf, 100, nas_e);
    _mav_put_float(buf, 104, nas_d);
    _mav_put_float(buf, 108, nas_vn);
    _mav_put_float(buf, 112, nas_ve);
    _mav_put_float(buf, 116, nas_vd);
    _mav_put_float(buf, 120, nas_qx);
    _mav_put_float(buf, 124, nas_qy);
    _mav_put_float(buf, 128, nas_qz);
    _mav_put_float(buf, 132, nas_qw);
    _mav_put_float(buf, 136, nas_bias_x);
    _mav_put_float(buf, 140, nas_bias_y);
    _mav_put_float(buf, 144, nas_bias_z);
    _mav_put_float(buf, 148, vbat);
    _mav_put_float(buf, 152, temperature);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, abk_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, pin_expulsion);
    _mav_put_uint8_t(buf, 165, cutter_presence);
    _mav_put_int8_t(buf, 166, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
#else
    mavlink_rocket_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
    packet.ada_vert_speed = ada_vert_speed;
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
    packet.abk_angle = abk_angle;
    packet.abk_estimated_cd = abk_estimated_cd;
    packet.parachute_load = parachute_load;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.vbat = vbat;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROCKET_FLIGHT_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
}

/**
 * @brief Encode a rocket_flight_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rocket_flight_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rocket_flight_tm_t* rocket_flight_tm)
{
    return mavlink_msg_rocket_flight_tm_pack(system_id, component_id, msg, rocket_flight_tm->timestamp, rocket_flight_tm->ada_state, rocket_flight_tm->fmm_state, rocket_flight_tm->dpl_state, rocket_flight_tm->abk_state, rocket_flight_tm->nas_state, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->abk_angle, rocket_flight_tm->abk_estimated_cd, rocket_flight_tm->parachute_load, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->pin_launch, rocket_flight_tm->pin_nosecone, rocket_flight_tm->pin_expulsion, rocket_flight_tm->cutter_presence, rocket_flight_tm->vbat, rocket_flight_tm->temperature, rocket_flight_tm->logger_error);
}

/**
 * @brief Encode a rocket_flight_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rocket_flight_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rocket_flight_tm_t* rocket_flight_tm)
{
    return mavlink_msg_rocket_flight_tm_pack_chan(system_id, component_id, chan, msg, rocket_flight_tm->timestamp, rocket_flight_tm->ada_state, rocket_flight_tm->fmm_state, rocket_flight_tm->dpl_state, rocket_flight_tm->abk_state, rocket_flight_tm->nas_state, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->abk_angle, rocket_flight_tm->abk_estimated_cd, rocket_flight_tm->parachute_load, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->pin_launch, rocket_flight_tm->pin_nosecone, rocket_flight_tm->pin_expulsion, rocket_flight_tm->cutter_presence, rocket_flight_tm->vbat, rocket_flight_tm->temperature, rocket_flight_tm->logger_error);
}

/**
 * @brief Send a rocket_flight_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param dpl_state  Deployment Controller State
 * @param abk_state  Airbrake FSM state
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
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
 * @param abk_angle [deg] Air Brakes angle
 * @param abk_estimated_cd  Estimated drag coefficient
 * @param parachute_load  Parachute load cell value
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param pin_launch  Launch pin status (1 = connected, 0 = disconnected)
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param pin_expulsion  Servo sensor status (1 = actuated, 0 = idle)
 * @param cutter_presence  Cutter presence status (1 = present, 0 = missing)
 * @param vbat [V] Battery voltage
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error, -1 otherwise)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rocket_flight_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t abk_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float abk_angle, float abk_estimated_cd, float parachute_load, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, float vbat, float temperature, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, acc_x);
    _mav_put_float(buf, 40, acc_y);
    _mav_put_float(buf, 44, acc_z);
    _mav_put_float(buf, 48, gyro_x);
    _mav_put_float(buf, 52, gyro_y);
    _mav_put_float(buf, 56, gyro_z);
    _mav_put_float(buf, 60, mag_x);
    _mav_put_float(buf, 64, mag_y);
    _mav_put_float(buf, 68, mag_z);
    _mav_put_float(buf, 72, gps_lat);
    _mav_put_float(buf, 76, gps_lon);
    _mav_put_float(buf, 80, gps_alt);
    _mav_put_float(buf, 84, abk_angle);
    _mav_put_float(buf, 88, abk_estimated_cd);
    _mav_put_float(buf, 92, parachute_load);
    _mav_put_float(buf, 96, nas_n);
    _mav_put_float(buf, 100, nas_e);
    _mav_put_float(buf, 104, nas_d);
    _mav_put_float(buf, 108, nas_vn);
    _mav_put_float(buf, 112, nas_ve);
    _mav_put_float(buf, 116, nas_vd);
    _mav_put_float(buf, 120, nas_qx);
    _mav_put_float(buf, 124, nas_qy);
    _mav_put_float(buf, 128, nas_qz);
    _mav_put_float(buf, 132, nas_qw);
    _mav_put_float(buf, 136, nas_bias_x);
    _mav_put_float(buf, 140, nas_bias_y);
    _mav_put_float(buf, 144, nas_bias_z);
    _mav_put_float(buf, 148, vbat);
    _mav_put_float(buf, 152, temperature);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, abk_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, pin_expulsion);
    _mav_put_uint8_t(buf, 165, cutter_presence);
    _mav_put_int8_t(buf, 166, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, buf, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#else
    mavlink_rocket_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
    packet.ada_vert_speed = ada_vert_speed;
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
    packet.abk_angle = abk_angle;
    packet.abk_estimated_cd = abk_estimated_cd;
    packet.parachute_load = parachute_load;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.vbat = vbat;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.dpl_state = dpl_state;
    packet.abk_state = abk_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_launch = pin_launch;
    packet.pin_nosecone = pin_nosecone;
    packet.pin_expulsion = pin_expulsion;
    packet.cutter_presence = cutter_presence;
    packet.logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, (const char *)&packet, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#endif
}

/**
 * @brief Send a rocket_flight_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rocket_flight_tm_send_struct(mavlink_channel_t chan, const mavlink_rocket_flight_tm_t* rocket_flight_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rocket_flight_tm_send(chan, rocket_flight_tm->timestamp, rocket_flight_tm->ada_state, rocket_flight_tm->fmm_state, rocket_flight_tm->dpl_state, rocket_flight_tm->abk_state, rocket_flight_tm->nas_state, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->abk_angle, rocket_flight_tm->abk_estimated_cd, rocket_flight_tm->parachute_load, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->pin_launch, rocket_flight_tm->pin_nosecone, rocket_flight_tm->pin_expulsion, rocket_flight_tm->cutter_presence, rocket_flight_tm->vbat, rocket_flight_tm->temperature, rocket_flight_tm->logger_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, (const char *)rocket_flight_tm, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rocket_flight_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t dpl_state, uint8_t abk_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float abk_angle, float abk_estimated_cd, float parachute_load, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_launch, uint8_t pin_nosecone, uint8_t pin_expulsion, uint8_t cutter_presence, float vbat, float temperature, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, ada_vert_speed);
    _mav_put_float(buf, 36, acc_x);
    _mav_put_float(buf, 40, acc_y);
    _mav_put_float(buf, 44, acc_z);
    _mav_put_float(buf, 48, gyro_x);
    _mav_put_float(buf, 52, gyro_y);
    _mav_put_float(buf, 56, gyro_z);
    _mav_put_float(buf, 60, mag_x);
    _mav_put_float(buf, 64, mag_y);
    _mav_put_float(buf, 68, mag_z);
    _mav_put_float(buf, 72, gps_lat);
    _mav_put_float(buf, 76, gps_lon);
    _mav_put_float(buf, 80, gps_alt);
    _mav_put_float(buf, 84, abk_angle);
    _mav_put_float(buf, 88, abk_estimated_cd);
    _mav_put_float(buf, 92, parachute_load);
    _mav_put_float(buf, 96, nas_n);
    _mav_put_float(buf, 100, nas_e);
    _mav_put_float(buf, 104, nas_d);
    _mav_put_float(buf, 108, nas_vn);
    _mav_put_float(buf, 112, nas_ve);
    _mav_put_float(buf, 116, nas_vd);
    _mav_put_float(buf, 120, nas_qx);
    _mav_put_float(buf, 124, nas_qy);
    _mav_put_float(buf, 128, nas_qz);
    _mav_put_float(buf, 132, nas_qw);
    _mav_put_float(buf, 136, nas_bias_x);
    _mav_put_float(buf, 140, nas_bias_y);
    _mav_put_float(buf, 144, nas_bias_z);
    _mav_put_float(buf, 148, vbat);
    _mav_put_float(buf, 152, temperature);
    _mav_put_uint8_t(buf, 156, ada_state);
    _mav_put_uint8_t(buf, 157, fmm_state);
    _mav_put_uint8_t(buf, 158, dpl_state);
    _mav_put_uint8_t(buf, 159, abk_state);
    _mav_put_uint8_t(buf, 160, nas_state);
    _mav_put_uint8_t(buf, 161, gps_fix);
    _mav_put_uint8_t(buf, 162, pin_launch);
    _mav_put_uint8_t(buf, 163, pin_nosecone);
    _mav_put_uint8_t(buf, 164, pin_expulsion);
    _mav_put_uint8_t(buf, 165, cutter_presence);
    _mav_put_int8_t(buf, 166, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, buf, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#else
    mavlink_rocket_flight_tm_t *packet = (mavlink_rocket_flight_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure_ada = pressure_ada;
    packet->pressure_digi = pressure_digi;
    packet->pressure_static = pressure_static;
    packet->pressure_dpl = pressure_dpl;
    packet->airspeed_pitot = airspeed_pitot;
    packet->altitude_agl = altitude_agl;
    packet->ada_vert_speed = ada_vert_speed;
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
    packet->abk_angle = abk_angle;
    packet->abk_estimated_cd = abk_estimated_cd;
    packet->parachute_load = parachute_load;
    packet->nas_n = nas_n;
    packet->nas_e = nas_e;
    packet->nas_d = nas_d;
    packet->nas_vn = nas_vn;
    packet->nas_ve = nas_ve;
    packet->nas_vd = nas_vd;
    packet->nas_qx = nas_qx;
    packet->nas_qy = nas_qy;
    packet->nas_qz = nas_qz;
    packet->nas_qw = nas_qw;
    packet->nas_bias_x = nas_bias_x;
    packet->nas_bias_y = nas_bias_y;
    packet->nas_bias_z = nas_bias_z;
    packet->vbat = vbat;
    packet->temperature = temperature;
    packet->ada_state = ada_state;
    packet->fmm_state = fmm_state;
    packet->dpl_state = dpl_state;
    packet->abk_state = abk_state;
    packet->nas_state = nas_state;
    packet->gps_fix = gps_fix;
    packet->pin_launch = pin_launch;
    packet->pin_nosecone = pin_nosecone;
    packet->pin_expulsion = pin_expulsion;
    packet->cutter_presence = cutter_presence;
    packet->logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, (const char *)packet, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ROCKET_FLIGHT_TM UNPACKING


/**
 * @brief Get field timestamp from rocket_flight_tm message
 *
 * @return [us] Timestamp in milliseconds
 */
static inline uint64_t mavlink_msg_rocket_flight_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ada_state from rocket_flight_tm message
 *
 * @return  ADA Controller State
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_ada_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  156);
}

/**
 * @brief Get field fmm_state from rocket_flight_tm message
 *
 * @return  Flight Mode Manager State
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_fmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  157);
}

/**
 * @brief Get field dpl_state from rocket_flight_tm message
 *
 * @return  Deployment Controller State
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_dpl_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  158);
}

/**
 * @brief Get field abk_state from rocket_flight_tm message
 *
 * @return  Airbrake FSM state
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_abk_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  159);
}

/**
 * @brief Get field nas_state from rocket_flight_tm message
 *
 * @return  Navigation System FSM State
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  160);
}

/**
 * @brief Get field pressure_ada from rocket_flight_tm message
 *
 * @return [Pa] Atmospheric pressure estimated by ADA
 */
static inline float mavlink_msg_rocket_flight_tm_get_pressure_ada(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure_digi from rocket_flight_tm message
 *
 * @return [Pa] Pressure from digital sensor
 */
static inline float mavlink_msg_rocket_flight_tm_get_pressure_digi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pressure_static from rocket_flight_tm message
 *
 * @return [Pa] Pressure from static port
 */
static inline float mavlink_msg_rocket_flight_tm_get_pressure_static(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure_dpl from rocket_flight_tm message
 *
 * @return [Pa] Pressure from deployment vane sensor
 */
static inline float mavlink_msg_rocket_flight_tm_get_pressure_dpl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field airspeed_pitot from rocket_flight_tm message
 *
 * @return [m/s] Pitot airspeed
 */
static inline float mavlink_msg_rocket_flight_tm_get_airspeed_pitot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field altitude_agl from rocket_flight_tm message
 *
 * @return [m] Altitude above ground level
 */
static inline float mavlink_msg_rocket_flight_tm_get_altitude_agl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ada_vert_speed from rocket_flight_tm message
 *
 * @return [m/s] Vertical speed estimated by ADA
 */
static inline float mavlink_msg_rocket_flight_tm_get_ada_vert_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acc_x from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field acc_y from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field acc_z from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field gyro_x from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gyro_y from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field gyro_z from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field mag_x from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mag_y from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field mag_z from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field gps_fix from rocket_flight_tm message
 *
 * @return  GPS fix (1 = fix, 0 = no fix)
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  161);
}

/**
 * @brief Get field gps_lat from rocket_flight_tm message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field gps_lon from rocket_flight_tm message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field gps_alt from rocket_flight_tm message
 *
 * @return [m] GPS Altitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field abk_angle from rocket_flight_tm message
 *
 * @return [deg] Air Brakes angle
 */
static inline float mavlink_msg_rocket_flight_tm_get_abk_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field abk_estimated_cd from rocket_flight_tm message
 *
 * @return  Estimated drag coefficient
 */
static inline float mavlink_msg_rocket_flight_tm_get_abk_estimated_cd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field parachute_load from rocket_flight_tm message
 *
 * @return  Parachute load cell value
 */
static inline float mavlink_msg_rocket_flight_tm_get_parachute_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field nas_n from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated noth position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field nas_e from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated east position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field nas_d from rocket_flight_tm message
 *
 * @return [m] Navigation system estimated down position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field nas_vn from rocket_flight_tm message
 *
 * @return [m/s] Navigation system estimated north velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field nas_ve from rocket_flight_tm message
 *
 * @return [m/s] Navigation system estimated east velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field nas_vd from rocket_flight_tm message
 *
 * @return [m/s] Navigation system estimated down velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field nas_qx from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qx)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field nas_qy from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qy)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field nas_qz from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qz)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field nas_qw from rocket_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qw)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field nas_bias_x from rocket_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field nas_bias_y from rocket_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field nas_bias_z from rocket_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field pin_launch from rocket_flight_tm message
 *
 * @return  Launch pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_pin_launch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  162);
}

/**
 * @brief Get field pin_nosecone from rocket_flight_tm message
 *
 * @return  Nosecone pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_pin_nosecone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  163);
}

/**
 * @brief Get field pin_expulsion from rocket_flight_tm message
 *
 * @return  Servo sensor status (1 = actuated, 0 = idle)
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_pin_expulsion(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  164);
}

/**
 * @brief Get field cutter_presence from rocket_flight_tm message
 *
 * @return  Cutter presence status (1 = present, 0 = missing)
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_cutter_presence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  165);
}

/**
 * @brief Get field vbat from rocket_flight_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_rocket_flight_tm_get_vbat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field temperature from rocket_flight_tm message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_rocket_flight_tm_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  152);
}

/**
 * @brief Get field logger_error from rocket_flight_tm message
 *
 * @return  Logger error (0 = no error, -1 otherwise)
 */
static inline int8_t mavlink_msg_rocket_flight_tm_get_logger_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  166);
}

/**
 * @brief Decode a rocket_flight_tm message into a struct
 *
 * @param msg The message to decode
 * @param rocket_flight_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_rocket_flight_tm_decode(const mavlink_message_t* msg, mavlink_rocket_flight_tm_t* rocket_flight_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rocket_flight_tm->timestamp = mavlink_msg_rocket_flight_tm_get_timestamp(msg);
    rocket_flight_tm->pressure_ada = mavlink_msg_rocket_flight_tm_get_pressure_ada(msg);
    rocket_flight_tm->pressure_digi = mavlink_msg_rocket_flight_tm_get_pressure_digi(msg);
    rocket_flight_tm->pressure_static = mavlink_msg_rocket_flight_tm_get_pressure_static(msg);
    rocket_flight_tm->pressure_dpl = mavlink_msg_rocket_flight_tm_get_pressure_dpl(msg);
    rocket_flight_tm->airspeed_pitot = mavlink_msg_rocket_flight_tm_get_airspeed_pitot(msg);
    rocket_flight_tm->altitude_agl = mavlink_msg_rocket_flight_tm_get_altitude_agl(msg);
    rocket_flight_tm->ada_vert_speed = mavlink_msg_rocket_flight_tm_get_ada_vert_speed(msg);
    rocket_flight_tm->acc_x = mavlink_msg_rocket_flight_tm_get_acc_x(msg);
    rocket_flight_tm->acc_y = mavlink_msg_rocket_flight_tm_get_acc_y(msg);
    rocket_flight_tm->acc_z = mavlink_msg_rocket_flight_tm_get_acc_z(msg);
    rocket_flight_tm->gyro_x = mavlink_msg_rocket_flight_tm_get_gyro_x(msg);
    rocket_flight_tm->gyro_y = mavlink_msg_rocket_flight_tm_get_gyro_y(msg);
    rocket_flight_tm->gyro_z = mavlink_msg_rocket_flight_tm_get_gyro_z(msg);
    rocket_flight_tm->mag_x = mavlink_msg_rocket_flight_tm_get_mag_x(msg);
    rocket_flight_tm->mag_y = mavlink_msg_rocket_flight_tm_get_mag_y(msg);
    rocket_flight_tm->mag_z = mavlink_msg_rocket_flight_tm_get_mag_z(msg);
    rocket_flight_tm->gps_lat = mavlink_msg_rocket_flight_tm_get_gps_lat(msg);
    rocket_flight_tm->gps_lon = mavlink_msg_rocket_flight_tm_get_gps_lon(msg);
    rocket_flight_tm->gps_alt = mavlink_msg_rocket_flight_tm_get_gps_alt(msg);
    rocket_flight_tm->abk_angle = mavlink_msg_rocket_flight_tm_get_abk_angle(msg);
    rocket_flight_tm->abk_estimated_cd = mavlink_msg_rocket_flight_tm_get_abk_estimated_cd(msg);
    rocket_flight_tm->parachute_load = mavlink_msg_rocket_flight_tm_get_parachute_load(msg);
    rocket_flight_tm->nas_n = mavlink_msg_rocket_flight_tm_get_nas_n(msg);
    rocket_flight_tm->nas_e = mavlink_msg_rocket_flight_tm_get_nas_e(msg);
    rocket_flight_tm->nas_d = mavlink_msg_rocket_flight_tm_get_nas_d(msg);
    rocket_flight_tm->nas_vn = mavlink_msg_rocket_flight_tm_get_nas_vn(msg);
    rocket_flight_tm->nas_ve = mavlink_msg_rocket_flight_tm_get_nas_ve(msg);
    rocket_flight_tm->nas_vd = mavlink_msg_rocket_flight_tm_get_nas_vd(msg);
    rocket_flight_tm->nas_qx = mavlink_msg_rocket_flight_tm_get_nas_qx(msg);
    rocket_flight_tm->nas_qy = mavlink_msg_rocket_flight_tm_get_nas_qy(msg);
    rocket_flight_tm->nas_qz = mavlink_msg_rocket_flight_tm_get_nas_qz(msg);
    rocket_flight_tm->nas_qw = mavlink_msg_rocket_flight_tm_get_nas_qw(msg);
    rocket_flight_tm->nas_bias_x = mavlink_msg_rocket_flight_tm_get_nas_bias_x(msg);
    rocket_flight_tm->nas_bias_y = mavlink_msg_rocket_flight_tm_get_nas_bias_y(msg);
    rocket_flight_tm->nas_bias_z = mavlink_msg_rocket_flight_tm_get_nas_bias_z(msg);
    rocket_flight_tm->vbat = mavlink_msg_rocket_flight_tm_get_vbat(msg);
    rocket_flight_tm->temperature = mavlink_msg_rocket_flight_tm_get_temperature(msg);
    rocket_flight_tm->ada_state = mavlink_msg_rocket_flight_tm_get_ada_state(msg);
    rocket_flight_tm->fmm_state = mavlink_msg_rocket_flight_tm_get_fmm_state(msg);
    rocket_flight_tm->dpl_state = mavlink_msg_rocket_flight_tm_get_dpl_state(msg);
    rocket_flight_tm->abk_state = mavlink_msg_rocket_flight_tm_get_abk_state(msg);
    rocket_flight_tm->nas_state = mavlink_msg_rocket_flight_tm_get_nas_state(msg);
    rocket_flight_tm->gps_fix = mavlink_msg_rocket_flight_tm_get_gps_fix(msg);
    rocket_flight_tm->pin_launch = mavlink_msg_rocket_flight_tm_get_pin_launch(msg);
    rocket_flight_tm->pin_nosecone = mavlink_msg_rocket_flight_tm_get_pin_nosecone(msg);
    rocket_flight_tm->pin_expulsion = mavlink_msg_rocket_flight_tm_get_pin_expulsion(msg);
    rocket_flight_tm->cutter_presence = mavlink_msg_rocket_flight_tm_get_cutter_presence(msg);
    rocket_flight_tm->logger_error = mavlink_msg_rocket_flight_tm_get_logger_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN? msg->len : MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN;
        memset(rocket_flight_tm, 0, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
    memcpy(rocket_flight_tm, _MAV_PAYLOAD(msg), len);
#endif
}
