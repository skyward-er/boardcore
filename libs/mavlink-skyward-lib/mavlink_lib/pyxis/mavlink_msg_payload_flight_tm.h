#pragma once
// MESSAGE PAYLOAD_FLIGHT_TM PACKING

#define MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM 208


typedef struct __mavlink_payload_flight_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in milliseconds*/
 float pressure_ada; /*< [Pa] Atmospheric pressure estimated by ADA*/
 float pressure_digi; /*< [Pa] Pressure from digital sensor*/
 float pressure_static; /*< [Pa] Pressure from static port*/
 float pressure_dpl; /*< [Pa] Pressure from deployment vane sensor*/
 float airspeed_pitot; /*< [m/s] Pitot airspeed*/
 float altitude_agl; /*< [m] Altitude above ground level*/
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
 float left_servo_angle; /*< [deg] Left servo motor angle*/
 float right_servo_angle; /*< [deg] Right servo motor angle*/
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
 float vsupply_5v; /*< [V] Power supply 5V*/
 float temperature; /*< [degC] Temperature*/
 uint8_t ada_state; /*<  ADA Controller State*/
 uint8_t fmm_state; /*<  Flight Mode Manager State*/
 uint8_t nas_state; /*<  Navigation System FSM State*/
 uint8_t gps_fix; /*<  GPS fix (1 = fix, 0 = no fix)*/
 uint8_t pin_nosecone; /*<  Nosecone pin status (1 = connected, 0 = disconnected)*/
 int8_t logger_error; /*<  Logger error (0 = no error)*/
} mavlink_payload_flight_tm_t;

#define MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN 158
#define MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN 158
#define MAVLINK_MSG_ID_208_LEN 158
#define MAVLINK_MSG_ID_208_MIN_LEN 158

#define MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC 193
#define MAVLINK_MSG_ID_208_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PAYLOAD_FLIGHT_TM { \
    208, \
    "PAYLOAD_FLIGHT_TM", \
    43, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_flight_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 152, offsetof(mavlink_payload_flight_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 153, offsetof(mavlink_payload_flight_tm_t, fmm_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 154, offsetof(mavlink_payload_flight_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_payload_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_payload_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_payload_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_payload_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_payload_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_payload_flight_tm_t, altitude_agl) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_payload_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_payload_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_payload_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_payload_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_payload_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_payload_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_payload_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_payload_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_payload_flight_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 155, offsetof(mavlink_payload_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_payload_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_payload_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_payload_flight_tm_t, gps_alt) }, \
         { "left_servo_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_payload_flight_tm_t, left_servo_angle) }, \
         { "right_servo_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_payload_flight_tm_t, right_servo_angle) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_payload_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_payload_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_payload_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_payload_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_payload_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_payload_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_payload_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_payload_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_payload_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_payload_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_payload_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_payload_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_payload_flight_tm_t, nas_bias_z) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_payload_flight_tm_t, pin_nosecone) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_payload_flight_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_payload_flight_tm_t, vsupply_5v) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_payload_flight_tm_t, temperature) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 157, offsetof(mavlink_payload_flight_tm_t, logger_error) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PAYLOAD_FLIGHT_TM { \
    "PAYLOAD_FLIGHT_TM", \
    43, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_payload_flight_tm_t, timestamp) }, \
         { "ada_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 152, offsetof(mavlink_payload_flight_tm_t, ada_state) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 153, offsetof(mavlink_payload_flight_tm_t, fmm_state) }, \
         { "nas_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 154, offsetof(mavlink_payload_flight_tm_t, nas_state) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_payload_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_payload_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_payload_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_payload_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_payload_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_payload_flight_tm_t, altitude_agl) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_payload_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_payload_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_payload_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_payload_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_payload_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_payload_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_payload_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_payload_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_payload_flight_tm_t, mag_z) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 155, offsetof(mavlink_payload_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_payload_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_payload_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_payload_flight_tm_t, gps_alt) }, \
         { "left_servo_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_payload_flight_tm_t, left_servo_angle) }, \
         { "right_servo_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_payload_flight_tm_t, right_servo_angle) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_payload_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_payload_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_payload_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_payload_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_payload_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_payload_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_payload_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_payload_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_payload_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_payload_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_payload_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_payload_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_payload_flight_tm_t, nas_bias_z) }, \
         { "pin_nosecone", NULL, MAVLINK_TYPE_UINT8_T, 0, 156, offsetof(mavlink_payload_flight_tm_t, pin_nosecone) }, \
         { "vbat", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_payload_flight_tm_t, vbat) }, \
         { "vsupply_5v", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_payload_flight_tm_t, vsupply_5v) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_payload_flight_tm_t, temperature) }, \
         { "logger_error", NULL, MAVLINK_TYPE_INT8_T, 0, 157, offsetof(mavlink_payload_flight_tm_t, logger_error) }, \
         } \
}
#endif

/**
 * @brief Pack a payload_flight_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
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
 * @param left_servo_angle [deg] Left servo motor angle
 * @param right_servo_angle [deg] Right servo motor angle
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
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_flight_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float left_servo_angle, float right_servo_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_nosecone, float vbat, float vsupply_5v, float temperature, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, acc_x);
    _mav_put_float(buf, 36, acc_y);
    _mav_put_float(buf, 40, acc_z);
    _mav_put_float(buf, 44, gyro_x);
    _mav_put_float(buf, 48, gyro_y);
    _mav_put_float(buf, 52, gyro_z);
    _mav_put_float(buf, 56, mag_x);
    _mav_put_float(buf, 60, mag_y);
    _mav_put_float(buf, 64, mag_z);
    _mav_put_float(buf, 68, gps_lat);
    _mav_put_float(buf, 72, gps_lon);
    _mav_put_float(buf, 76, gps_alt);
    _mav_put_float(buf, 80, left_servo_angle);
    _mav_put_float(buf, 84, right_servo_angle);
    _mav_put_float(buf, 88, nas_n);
    _mav_put_float(buf, 92, nas_e);
    _mav_put_float(buf, 96, nas_d);
    _mav_put_float(buf, 100, nas_vn);
    _mav_put_float(buf, 104, nas_ve);
    _mav_put_float(buf, 108, nas_vd);
    _mav_put_float(buf, 112, nas_qx);
    _mav_put_float(buf, 116, nas_qy);
    _mav_put_float(buf, 120, nas_qz);
    _mav_put_float(buf, 124, nas_qw);
    _mav_put_float(buf, 128, nas_bias_x);
    _mav_put_float(buf, 132, nas_bias_y);
    _mav_put_float(buf, 136, nas_bias_z);
    _mav_put_float(buf, 140, vbat);
    _mav_put_float(buf, 144, vsupply_5v);
    _mav_put_float(buf, 148, temperature);
    _mav_put_uint8_t(buf, 152, ada_state);
    _mav_put_uint8_t(buf, 153, fmm_state);
    _mav_put_uint8_t(buf, 154, nas_state);
    _mav_put_uint8_t(buf, 155, gps_fix);
    _mav_put_uint8_t(buf, 156, pin_nosecone);
    _mav_put_int8_t(buf, 157, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN);
#else
    mavlink_payload_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
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
    packet.left_servo_angle = left_servo_angle;
    packet.right_servo_angle = right_servo_angle;
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
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_nosecone = pin_nosecone;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
}

/**
 * @brief Pack a payload_flight_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
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
 * @param left_servo_angle [deg] Left servo motor angle
 * @param right_servo_angle [deg] Right servo motor angle
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
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_payload_flight_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t ada_state,uint8_t fmm_state,uint8_t nas_state,float pressure_ada,float pressure_digi,float pressure_static,float pressure_dpl,float airspeed_pitot,float altitude_agl,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z,uint8_t gps_fix,float gps_lat,float gps_lon,float gps_alt,float left_servo_angle,float right_servo_angle,float nas_n,float nas_e,float nas_d,float nas_vn,float nas_ve,float nas_vd,float nas_qx,float nas_qy,float nas_qz,float nas_qw,float nas_bias_x,float nas_bias_y,float nas_bias_z,uint8_t pin_nosecone,float vbat,float vsupply_5v,float temperature,int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, acc_x);
    _mav_put_float(buf, 36, acc_y);
    _mav_put_float(buf, 40, acc_z);
    _mav_put_float(buf, 44, gyro_x);
    _mav_put_float(buf, 48, gyro_y);
    _mav_put_float(buf, 52, gyro_z);
    _mav_put_float(buf, 56, mag_x);
    _mav_put_float(buf, 60, mag_y);
    _mav_put_float(buf, 64, mag_z);
    _mav_put_float(buf, 68, gps_lat);
    _mav_put_float(buf, 72, gps_lon);
    _mav_put_float(buf, 76, gps_alt);
    _mav_put_float(buf, 80, left_servo_angle);
    _mav_put_float(buf, 84, right_servo_angle);
    _mav_put_float(buf, 88, nas_n);
    _mav_put_float(buf, 92, nas_e);
    _mav_put_float(buf, 96, nas_d);
    _mav_put_float(buf, 100, nas_vn);
    _mav_put_float(buf, 104, nas_ve);
    _mav_put_float(buf, 108, nas_vd);
    _mav_put_float(buf, 112, nas_qx);
    _mav_put_float(buf, 116, nas_qy);
    _mav_put_float(buf, 120, nas_qz);
    _mav_put_float(buf, 124, nas_qw);
    _mav_put_float(buf, 128, nas_bias_x);
    _mav_put_float(buf, 132, nas_bias_y);
    _mav_put_float(buf, 136, nas_bias_z);
    _mav_put_float(buf, 140, vbat);
    _mav_put_float(buf, 144, vsupply_5v);
    _mav_put_float(buf, 148, temperature);
    _mav_put_uint8_t(buf, 152, ada_state);
    _mav_put_uint8_t(buf, 153, fmm_state);
    _mav_put_uint8_t(buf, 154, nas_state);
    _mav_put_uint8_t(buf, 155, gps_fix);
    _mav_put_uint8_t(buf, 156, pin_nosecone);
    _mav_put_int8_t(buf, 157, logger_error);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN);
#else
    mavlink_payload_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
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
    packet.left_servo_angle = left_servo_angle;
    packet.right_servo_angle = right_servo_angle;
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
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_nosecone = pin_nosecone;
    packet.logger_error = logger_error;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
}

/**
 * @brief Encode a payload_flight_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param payload_flight_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_flight_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_payload_flight_tm_t* payload_flight_tm)
{
    return mavlink_msg_payload_flight_tm_pack(system_id, component_id, msg, payload_flight_tm->timestamp, payload_flight_tm->ada_state, payload_flight_tm->fmm_state, payload_flight_tm->nas_state, payload_flight_tm->pressure_ada, payload_flight_tm->pressure_digi, payload_flight_tm->pressure_static, payload_flight_tm->pressure_dpl, payload_flight_tm->airspeed_pitot, payload_flight_tm->altitude_agl, payload_flight_tm->acc_x, payload_flight_tm->acc_y, payload_flight_tm->acc_z, payload_flight_tm->gyro_x, payload_flight_tm->gyro_y, payload_flight_tm->gyro_z, payload_flight_tm->mag_x, payload_flight_tm->mag_y, payload_flight_tm->mag_z, payload_flight_tm->gps_fix, payload_flight_tm->gps_lat, payload_flight_tm->gps_lon, payload_flight_tm->gps_alt, payload_flight_tm->left_servo_angle, payload_flight_tm->right_servo_angle, payload_flight_tm->nas_n, payload_flight_tm->nas_e, payload_flight_tm->nas_d, payload_flight_tm->nas_vn, payload_flight_tm->nas_ve, payload_flight_tm->nas_vd, payload_flight_tm->nas_qx, payload_flight_tm->nas_qy, payload_flight_tm->nas_qz, payload_flight_tm->nas_qw, payload_flight_tm->nas_bias_x, payload_flight_tm->nas_bias_y, payload_flight_tm->nas_bias_z, payload_flight_tm->pin_nosecone, payload_flight_tm->vbat, payload_flight_tm->vsupply_5v, payload_flight_tm->temperature, payload_flight_tm->logger_error);
}

/**
 * @brief Encode a payload_flight_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param payload_flight_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_payload_flight_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_payload_flight_tm_t* payload_flight_tm)
{
    return mavlink_msg_payload_flight_tm_pack_chan(system_id, component_id, chan, msg, payload_flight_tm->timestamp, payload_flight_tm->ada_state, payload_flight_tm->fmm_state, payload_flight_tm->nas_state, payload_flight_tm->pressure_ada, payload_flight_tm->pressure_digi, payload_flight_tm->pressure_static, payload_flight_tm->pressure_dpl, payload_flight_tm->airspeed_pitot, payload_flight_tm->altitude_agl, payload_flight_tm->acc_x, payload_flight_tm->acc_y, payload_flight_tm->acc_z, payload_flight_tm->gyro_x, payload_flight_tm->gyro_y, payload_flight_tm->gyro_z, payload_flight_tm->mag_x, payload_flight_tm->mag_y, payload_flight_tm->mag_z, payload_flight_tm->gps_fix, payload_flight_tm->gps_lat, payload_flight_tm->gps_lon, payload_flight_tm->gps_alt, payload_flight_tm->left_servo_angle, payload_flight_tm->right_servo_angle, payload_flight_tm->nas_n, payload_flight_tm->nas_e, payload_flight_tm->nas_d, payload_flight_tm->nas_vn, payload_flight_tm->nas_ve, payload_flight_tm->nas_vd, payload_flight_tm->nas_qx, payload_flight_tm->nas_qy, payload_flight_tm->nas_qz, payload_flight_tm->nas_qw, payload_flight_tm->nas_bias_x, payload_flight_tm->nas_bias_y, payload_flight_tm->nas_bias_z, payload_flight_tm->pin_nosecone, payload_flight_tm->vbat, payload_flight_tm->vsupply_5v, payload_flight_tm->temperature, payload_flight_tm->logger_error);
}

/**
 * @brief Send a payload_flight_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in milliseconds
 * @param ada_state  ADA Controller State
 * @param fmm_state  Flight Mode Manager State
 * @param nas_state  Navigation System FSM State
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
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
 * @param left_servo_angle [deg] Left servo motor angle
 * @param right_servo_angle [deg] Right servo motor angle
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
 * @param pin_nosecone  Nosecone pin status (1 = connected, 0 = disconnected)
 * @param vbat [V] Battery voltage
 * @param vsupply_5v [V] Power supply 5V
 * @param temperature [degC] Temperature
 * @param logger_error  Logger error (0 = no error)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_payload_flight_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float left_servo_angle, float right_servo_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_nosecone, float vbat, float vsupply_5v, float temperature, int8_t logger_error)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, pressure_ada);
    _mav_put_float(buf, 12, pressure_digi);
    _mav_put_float(buf, 16, pressure_static);
    _mav_put_float(buf, 20, pressure_dpl);
    _mav_put_float(buf, 24, airspeed_pitot);
    _mav_put_float(buf, 28, altitude_agl);
    _mav_put_float(buf, 32, acc_x);
    _mav_put_float(buf, 36, acc_y);
    _mav_put_float(buf, 40, acc_z);
    _mav_put_float(buf, 44, gyro_x);
    _mav_put_float(buf, 48, gyro_y);
    _mav_put_float(buf, 52, gyro_z);
    _mav_put_float(buf, 56, mag_x);
    _mav_put_float(buf, 60, mag_y);
    _mav_put_float(buf, 64, mag_z);
    _mav_put_float(buf, 68, gps_lat);
    _mav_put_float(buf, 72, gps_lon);
    _mav_put_float(buf, 76, gps_alt);
    _mav_put_float(buf, 80, left_servo_angle);
    _mav_put_float(buf, 84, right_servo_angle);
    _mav_put_float(buf, 88, nas_n);
    _mav_put_float(buf, 92, nas_e);
    _mav_put_float(buf, 96, nas_d);
    _mav_put_float(buf, 100, nas_vn);
    _mav_put_float(buf, 104, nas_ve);
    _mav_put_float(buf, 108, nas_vd);
    _mav_put_float(buf, 112, nas_qx);
    _mav_put_float(buf, 116, nas_qy);
    _mav_put_float(buf, 120, nas_qz);
    _mav_put_float(buf, 124, nas_qw);
    _mav_put_float(buf, 128, nas_bias_x);
    _mav_put_float(buf, 132, nas_bias_y);
    _mav_put_float(buf, 136, nas_bias_z);
    _mav_put_float(buf, 140, vbat);
    _mav_put_float(buf, 144, vsupply_5v);
    _mav_put_float(buf, 148, temperature);
    _mav_put_uint8_t(buf, 152, ada_state);
    _mav_put_uint8_t(buf, 153, fmm_state);
    _mav_put_uint8_t(buf, 154, nas_state);
    _mav_put_uint8_t(buf, 155, gps_fix);
    _mav_put_uint8_t(buf, 156, pin_nosecone);
    _mav_put_int8_t(buf, 157, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM, buf, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
#else
    mavlink_payload_flight_tm_t packet;
    packet.timestamp = timestamp;
    packet.pressure_ada = pressure_ada;
    packet.pressure_digi = pressure_digi;
    packet.pressure_static = pressure_static;
    packet.pressure_dpl = pressure_dpl;
    packet.airspeed_pitot = airspeed_pitot;
    packet.altitude_agl = altitude_agl;
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
    packet.left_servo_angle = left_servo_angle;
    packet.right_servo_angle = right_servo_angle;
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
    packet.vsupply_5v = vsupply_5v;
    packet.temperature = temperature;
    packet.ada_state = ada_state;
    packet.fmm_state = fmm_state;
    packet.nas_state = nas_state;
    packet.gps_fix = gps_fix;
    packet.pin_nosecone = pin_nosecone;
    packet.logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM, (const char *)&packet, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
#endif
}

/**
 * @brief Send a payload_flight_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_payload_flight_tm_send_struct(mavlink_channel_t chan, const mavlink_payload_flight_tm_t* payload_flight_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_payload_flight_tm_send(chan, payload_flight_tm->timestamp, payload_flight_tm->ada_state, payload_flight_tm->fmm_state, payload_flight_tm->nas_state, payload_flight_tm->pressure_ada, payload_flight_tm->pressure_digi, payload_flight_tm->pressure_static, payload_flight_tm->pressure_dpl, payload_flight_tm->airspeed_pitot, payload_flight_tm->altitude_agl, payload_flight_tm->acc_x, payload_flight_tm->acc_y, payload_flight_tm->acc_z, payload_flight_tm->gyro_x, payload_flight_tm->gyro_y, payload_flight_tm->gyro_z, payload_flight_tm->mag_x, payload_flight_tm->mag_y, payload_flight_tm->mag_z, payload_flight_tm->gps_fix, payload_flight_tm->gps_lat, payload_flight_tm->gps_lon, payload_flight_tm->gps_alt, payload_flight_tm->left_servo_angle, payload_flight_tm->right_servo_angle, payload_flight_tm->nas_n, payload_flight_tm->nas_e, payload_flight_tm->nas_d, payload_flight_tm->nas_vn, payload_flight_tm->nas_ve, payload_flight_tm->nas_vd, payload_flight_tm->nas_qx, payload_flight_tm->nas_qy, payload_flight_tm->nas_qz, payload_flight_tm->nas_qw, payload_flight_tm->nas_bias_x, payload_flight_tm->nas_bias_y, payload_flight_tm->nas_bias_z, payload_flight_tm->pin_nosecone, payload_flight_tm->vbat, payload_flight_tm->vsupply_5v, payload_flight_tm->temperature, payload_flight_tm->logger_error);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM, (const char *)payload_flight_tm, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_payload_flight_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t ada_state, uint8_t fmm_state, uint8_t nas_state, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, float left_servo_angle, float right_servo_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, uint8_t pin_nosecone, float vbat, float vsupply_5v, float temperature, int8_t logger_error)
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
    _mav_put_float(buf, 32, acc_x);
    _mav_put_float(buf, 36, acc_y);
    _mav_put_float(buf, 40, acc_z);
    _mav_put_float(buf, 44, gyro_x);
    _mav_put_float(buf, 48, gyro_y);
    _mav_put_float(buf, 52, gyro_z);
    _mav_put_float(buf, 56, mag_x);
    _mav_put_float(buf, 60, mag_y);
    _mav_put_float(buf, 64, mag_z);
    _mav_put_float(buf, 68, gps_lat);
    _mav_put_float(buf, 72, gps_lon);
    _mav_put_float(buf, 76, gps_alt);
    _mav_put_float(buf, 80, left_servo_angle);
    _mav_put_float(buf, 84, right_servo_angle);
    _mav_put_float(buf, 88, nas_n);
    _mav_put_float(buf, 92, nas_e);
    _mav_put_float(buf, 96, nas_d);
    _mav_put_float(buf, 100, nas_vn);
    _mav_put_float(buf, 104, nas_ve);
    _mav_put_float(buf, 108, nas_vd);
    _mav_put_float(buf, 112, nas_qx);
    _mav_put_float(buf, 116, nas_qy);
    _mav_put_float(buf, 120, nas_qz);
    _mav_put_float(buf, 124, nas_qw);
    _mav_put_float(buf, 128, nas_bias_x);
    _mav_put_float(buf, 132, nas_bias_y);
    _mav_put_float(buf, 136, nas_bias_z);
    _mav_put_float(buf, 140, vbat);
    _mav_put_float(buf, 144, vsupply_5v);
    _mav_put_float(buf, 148, temperature);
    _mav_put_uint8_t(buf, 152, ada_state);
    _mav_put_uint8_t(buf, 153, fmm_state);
    _mav_put_uint8_t(buf, 154, nas_state);
    _mav_put_uint8_t(buf, 155, gps_fix);
    _mav_put_uint8_t(buf, 156, pin_nosecone);
    _mav_put_int8_t(buf, 157, logger_error);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM, buf, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
#else
    mavlink_payload_flight_tm_t *packet = (mavlink_payload_flight_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->pressure_ada = pressure_ada;
    packet->pressure_digi = pressure_digi;
    packet->pressure_static = pressure_static;
    packet->pressure_dpl = pressure_dpl;
    packet->airspeed_pitot = airspeed_pitot;
    packet->altitude_agl = altitude_agl;
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
    packet->left_servo_angle = left_servo_angle;
    packet->right_servo_angle = right_servo_angle;
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
    packet->vsupply_5v = vsupply_5v;
    packet->temperature = temperature;
    packet->ada_state = ada_state;
    packet->fmm_state = fmm_state;
    packet->nas_state = nas_state;
    packet->gps_fix = gps_fix;
    packet->pin_nosecone = pin_nosecone;
    packet->logger_error = logger_error;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM, (const char *)packet, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE PAYLOAD_FLIGHT_TM UNPACKING


/**
 * @brief Get field timestamp from payload_flight_tm message
 *
 * @return [us] Timestamp in milliseconds
 */
static inline uint64_t mavlink_msg_payload_flight_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field ada_state from payload_flight_tm message
 *
 * @return  ADA Controller State
 */
static inline uint8_t mavlink_msg_payload_flight_tm_get_ada_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  152);
}

/**
 * @brief Get field fmm_state from payload_flight_tm message
 *
 * @return  Flight Mode Manager State
 */
static inline uint8_t mavlink_msg_payload_flight_tm_get_fmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  153);
}

/**
 * @brief Get field nas_state from payload_flight_tm message
 *
 * @return  Navigation System FSM State
 */
static inline uint8_t mavlink_msg_payload_flight_tm_get_nas_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  154);
}

/**
 * @brief Get field pressure_ada from payload_flight_tm message
 *
 * @return [Pa] Atmospheric pressure estimated by ADA
 */
static inline float mavlink_msg_payload_flight_tm_get_pressure_ada(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure_digi from payload_flight_tm message
 *
 * @return [Pa] Pressure from digital sensor
 */
static inline float mavlink_msg_payload_flight_tm_get_pressure_digi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pressure_static from payload_flight_tm message
 *
 * @return [Pa] Pressure from static port
 */
static inline float mavlink_msg_payload_flight_tm_get_pressure_static(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure_dpl from payload_flight_tm message
 *
 * @return [Pa] Pressure from deployment vane sensor
 */
static inline float mavlink_msg_payload_flight_tm_get_pressure_dpl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field airspeed_pitot from payload_flight_tm message
 *
 * @return [m/s] Pitot airspeed
 */
static inline float mavlink_msg_payload_flight_tm_get_airspeed_pitot(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field altitude_agl from payload_flight_tm message
 *
 * @return [m] Altitude above ground level
 */
static inline float mavlink_msg_payload_flight_tm_get_altitude_agl(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field acc_x from payload_flight_tm message
 *
 * @return [m/s^2] Acceleration on X axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acc_y from payload_flight_tm message
 *
 * @return [m/s^2] Acceleration on Y axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field acc_z from payload_flight_tm message
 *
 * @return [m/s^2] Acceleration on Z axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field gyro_x from payload_flight_tm message
 *
 * @return [rad/s] Angular speed on X axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field gyro_y from payload_flight_tm message
 *
 * @return [rad/s] Angular speed on Y axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gyro_z from payload_flight_tm message
 *
 * @return [rad/s] Angular speed on Z axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field mag_x from payload_flight_tm message
 *
 * @return [uT] Magnetic field on X axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field mag_y from payload_flight_tm message
 *
 * @return [uT] Magnetic field on Y axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mag_z from payload_flight_tm message
 *
 * @return [uT] Magnetic field on Z axis (body)
 */
static inline float mavlink_msg_payload_flight_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field gps_fix from payload_flight_tm message
 *
 * @return  GPS fix (1 = fix, 0 = no fix)
 */
static inline uint8_t mavlink_msg_payload_flight_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  155);
}

/**
 * @brief Get field gps_lat from payload_flight_tm message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_payload_flight_tm_get_gps_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field gps_lon from payload_flight_tm message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_payload_flight_tm_get_gps_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field gps_alt from payload_flight_tm message
 *
 * @return [m] GPS Altitude
 */
static inline float mavlink_msg_payload_flight_tm_get_gps_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field left_servo_angle from payload_flight_tm message
 *
 * @return [deg] Left servo motor angle
 */
static inline float mavlink_msg_payload_flight_tm_get_left_servo_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field right_servo_angle from payload_flight_tm message
 *
 * @return [deg] Right servo motor angle
 */
static inline float mavlink_msg_payload_flight_tm_get_right_servo_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field nas_n from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated noth position
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field nas_e from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated east position
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field nas_d from payload_flight_tm message
 *
 * @return [m] Navigation system estimated down position
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field nas_vn from payload_flight_tm message
 *
 * @return [m/s] Navigation system estimated north velocity
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field nas_ve from payload_flight_tm message
 *
 * @return [m/s] Navigation system estimated east velocity
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field nas_vd from payload_flight_tm message
 *
 * @return [m/s] Navigation system estimated down velocity
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field nas_qx from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qx)
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field nas_qy from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qy)
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field nas_qz from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qz)
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field nas_qw from payload_flight_tm message
 *
 * @return [deg] Navigation system estimated attitude (qw)
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field nas_bias_x from payload_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field nas_bias_y from payload_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field nas_bias_z from payload_flight_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_payload_flight_tm_get_nas_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field pin_nosecone from payload_flight_tm message
 *
 * @return  Nosecone pin status (1 = connected, 0 = disconnected)
 */
static inline uint8_t mavlink_msg_payload_flight_tm_get_pin_nosecone(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  156);
}

/**
 * @brief Get field vbat from payload_flight_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_payload_flight_tm_get_vbat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field vsupply_5v from payload_flight_tm message
 *
 * @return [V] Power supply 5V
 */
static inline float mavlink_msg_payload_flight_tm_get_vsupply_5v(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field temperature from payload_flight_tm message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_payload_flight_tm_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field logger_error from payload_flight_tm message
 *
 * @return  Logger error (0 = no error)
 */
static inline int8_t mavlink_msg_payload_flight_tm_get_logger_error(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  157);
}

/**
 * @brief Decode a payload_flight_tm message into a struct
 *
 * @param msg The message to decode
 * @param payload_flight_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_payload_flight_tm_decode(const mavlink_message_t* msg, mavlink_payload_flight_tm_t* payload_flight_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    payload_flight_tm->timestamp = mavlink_msg_payload_flight_tm_get_timestamp(msg);
    payload_flight_tm->pressure_ada = mavlink_msg_payload_flight_tm_get_pressure_ada(msg);
    payload_flight_tm->pressure_digi = mavlink_msg_payload_flight_tm_get_pressure_digi(msg);
    payload_flight_tm->pressure_static = mavlink_msg_payload_flight_tm_get_pressure_static(msg);
    payload_flight_tm->pressure_dpl = mavlink_msg_payload_flight_tm_get_pressure_dpl(msg);
    payload_flight_tm->airspeed_pitot = mavlink_msg_payload_flight_tm_get_airspeed_pitot(msg);
    payload_flight_tm->altitude_agl = mavlink_msg_payload_flight_tm_get_altitude_agl(msg);
    payload_flight_tm->acc_x = mavlink_msg_payload_flight_tm_get_acc_x(msg);
    payload_flight_tm->acc_y = mavlink_msg_payload_flight_tm_get_acc_y(msg);
    payload_flight_tm->acc_z = mavlink_msg_payload_flight_tm_get_acc_z(msg);
    payload_flight_tm->gyro_x = mavlink_msg_payload_flight_tm_get_gyro_x(msg);
    payload_flight_tm->gyro_y = mavlink_msg_payload_flight_tm_get_gyro_y(msg);
    payload_flight_tm->gyro_z = mavlink_msg_payload_flight_tm_get_gyro_z(msg);
    payload_flight_tm->mag_x = mavlink_msg_payload_flight_tm_get_mag_x(msg);
    payload_flight_tm->mag_y = mavlink_msg_payload_flight_tm_get_mag_y(msg);
    payload_flight_tm->mag_z = mavlink_msg_payload_flight_tm_get_mag_z(msg);
    payload_flight_tm->gps_lat = mavlink_msg_payload_flight_tm_get_gps_lat(msg);
    payload_flight_tm->gps_lon = mavlink_msg_payload_flight_tm_get_gps_lon(msg);
    payload_flight_tm->gps_alt = mavlink_msg_payload_flight_tm_get_gps_alt(msg);
    payload_flight_tm->left_servo_angle = mavlink_msg_payload_flight_tm_get_left_servo_angle(msg);
    payload_flight_tm->right_servo_angle = mavlink_msg_payload_flight_tm_get_right_servo_angle(msg);
    payload_flight_tm->nas_n = mavlink_msg_payload_flight_tm_get_nas_n(msg);
    payload_flight_tm->nas_e = mavlink_msg_payload_flight_tm_get_nas_e(msg);
    payload_flight_tm->nas_d = mavlink_msg_payload_flight_tm_get_nas_d(msg);
    payload_flight_tm->nas_vn = mavlink_msg_payload_flight_tm_get_nas_vn(msg);
    payload_flight_tm->nas_ve = mavlink_msg_payload_flight_tm_get_nas_ve(msg);
    payload_flight_tm->nas_vd = mavlink_msg_payload_flight_tm_get_nas_vd(msg);
    payload_flight_tm->nas_qx = mavlink_msg_payload_flight_tm_get_nas_qx(msg);
    payload_flight_tm->nas_qy = mavlink_msg_payload_flight_tm_get_nas_qy(msg);
    payload_flight_tm->nas_qz = mavlink_msg_payload_flight_tm_get_nas_qz(msg);
    payload_flight_tm->nas_qw = mavlink_msg_payload_flight_tm_get_nas_qw(msg);
    payload_flight_tm->nas_bias_x = mavlink_msg_payload_flight_tm_get_nas_bias_x(msg);
    payload_flight_tm->nas_bias_y = mavlink_msg_payload_flight_tm_get_nas_bias_y(msg);
    payload_flight_tm->nas_bias_z = mavlink_msg_payload_flight_tm_get_nas_bias_z(msg);
    payload_flight_tm->vbat = mavlink_msg_payload_flight_tm_get_vbat(msg);
    payload_flight_tm->vsupply_5v = mavlink_msg_payload_flight_tm_get_vsupply_5v(msg);
    payload_flight_tm->temperature = mavlink_msg_payload_flight_tm_get_temperature(msg);
    payload_flight_tm->ada_state = mavlink_msg_payload_flight_tm_get_ada_state(msg);
    payload_flight_tm->fmm_state = mavlink_msg_payload_flight_tm_get_fmm_state(msg);
    payload_flight_tm->nas_state = mavlink_msg_payload_flight_tm_get_nas_state(msg);
    payload_flight_tm->gps_fix = mavlink_msg_payload_flight_tm_get_gps_fix(msg);
    payload_flight_tm->pin_nosecone = mavlink_msg_payload_flight_tm_get_pin_nosecone(msg);
    payload_flight_tm->logger_error = mavlink_msg_payload_flight_tm_get_logger_error(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN? msg->len : MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN;
        memset(payload_flight_tm, 0, MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_LEN);
    memcpy(payload_flight_tm, _MAV_PAYLOAD(msg), len);
#endif
}
