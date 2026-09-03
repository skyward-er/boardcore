#pragma once
// MESSAGE ROCKET_FLIGHT_TM PACKING

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM 208


typedef struct __mavlink_rocket_flight_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float pressure_ada; /*< [Pa] Atmospheric pressure estimated by ADA*/
 float pressure_digi; /*< [Pa] Pressure from digital sensor*/
 float pressure_static; /*< [Pa] Pressure from static port*/
 float pressure_dpl; /*< [Pa] Pressure from deployment vane sensor*/
 float airspeed_pitot; /*< [m/s] Pitot airspeed*/
 float altitude_agl; /*< [m] Altitude above ground level*/
 float ada_vert_speed; /*< [m/s] Vertical speed estimated by ADA*/
 float mea_mass; /*< [kg] Mass estimated by MEA*/
 float mea_apogee; /*< [m] MEA estimated apogee*/
 float acc_x; /*< [m/s^2] Acceleration on X axis (body)*/
 float acc_y; /*< [m/s^2] Acceleration on Y axis (body)*/
 float acc_z; /*< [m/s^2] Acceleration on Z axis (body)*/
 float gyro_x; /*< [rad/s] Angular speed on X axis (body)*/
 float gyro_y; /*< [rad/s] Angular speed on Y axis (body)*/
 float gyro_z; /*< [rad/s] Angular speed on Z axis (body)*/
 float mag_x; /*< [uT] Magnetic field on X axis (body)*/
 float mag_y; /*< [uT] Magnetic field on Y axis (body)*/
 float mag_z; /*< [uT] Magnetic field on Z axis (body)*/
 float vn100_qx; /*<  VN100 estimated attitude (qx)*/
 float vn100_qy; /*<  VN100 estimated attitude (qy)*/
 float vn100_qz; /*<  VN100 estimated attitude (qz)*/
 float vn100_qw; /*<  VN100 estimated attitude (qw)*/
 float gps_lat; /*< [deg] Latitude*/
 float gps_lon; /*< [deg] Longitude*/
 float gps_alt; /*< [m] GPS Altitude*/
 float abk_angle; /*<  Air Brakes angle*/
 float nas_n; /*< [deg] NAS estimated noth position*/
 float nas_e; /*< [deg] NAS estimated east position*/
 float nas_d; /*< [m] NAS estimated down position*/
 float nas_vn; /*< [m/s] NAS estimated north velocity*/
 float nas_ve; /*< [m/s] NAS estimated east velocity*/
 float nas_vd; /*< [m/s] NAS estimated down velocity*/
 float nas_qx; /*<  NAS estimated attitude (qx)*/
 float nas_qy; /*<  NAS estimated attitude (qy)*/
 float nas_qz; /*<  NAS estimated attitude (qz)*/
 float nas_qw; /*<  NAS estimated attitude (qw)*/
 float nas_bias_x; /*<  NAS gyroscope bias on x axis*/
 float nas_bias_y; /*<  NAS gyroscope bias on y axis*/
 float nas_bias_z; /*<  NAS gyroscope bias on z axis*/
 float battery_voltage; /*< [V] Battery voltage*/
 float cam_battery_voltage; /*< [V] Camera battery voltage*/
 float temperature; /*< [degC] Temperature*/
 uint8_t gps_fix; /*<  Wether the GPS has a FIX*/
 uint8_t fmm_state; /*<  Flight Mode Manager State*/
} mavlink_rocket_flight_tm_t;

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN 178
#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN 178
#define MAVLINK_MSG_ID_208_LEN 178
#define MAVLINK_MSG_ID_208_MIN_LEN 178

#define MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC 235
#define MAVLINK_MSG_ID_208_CRC 235



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM { \
    208, \
    "ROCKET_FLIGHT_TM", \
    45, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_flight_tm_t, timestamp) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rocket_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rocket_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rocket_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rocket_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rocket_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rocket_flight_tm_t, altitude_agl) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rocket_flight_tm_t, ada_vert_speed) }, \
         { "mea_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rocket_flight_tm_t, mea_mass) }, \
         { "mea_apogee", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_flight_tm_t, mea_apogee) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_flight_tm_t, mag_z) }, \
         { "vn100_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_flight_tm_t, vn100_qx) }, \
         { "vn100_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_flight_tm_t, vn100_qy) }, \
         { "vn100_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_flight_tm_t, vn100_qz) }, \
         { "vn100_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_flight_tm_t, vn100_qw) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_rocket_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_flight_tm_t, gps_alt) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_rocket_flight_tm_t, fmm_state) }, \
         { "abk_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_flight_tm_t, abk_angle) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_rocket_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 156, offsetof(mavlink_rocket_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 160, offsetof(mavlink_rocket_flight_tm_t, nas_bias_z) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 164, offsetof(mavlink_rocket_flight_tm_t, battery_voltage) }, \
         { "cam_battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 168, offsetof(mavlink_rocket_flight_tm_t, cam_battery_voltage) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 172, offsetof(mavlink_rocket_flight_tm_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROCKET_FLIGHT_TM { \
    "ROCKET_FLIGHT_TM", \
    45, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rocket_flight_tm_t, timestamp) }, \
         { "pressure_ada", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rocket_flight_tm_t, pressure_ada) }, \
         { "pressure_digi", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rocket_flight_tm_t, pressure_digi) }, \
         { "pressure_static", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rocket_flight_tm_t, pressure_static) }, \
         { "pressure_dpl", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rocket_flight_tm_t, pressure_dpl) }, \
         { "airspeed_pitot", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rocket_flight_tm_t, airspeed_pitot) }, \
         { "altitude_agl", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rocket_flight_tm_t, altitude_agl) }, \
         { "ada_vert_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_rocket_flight_tm_t, ada_vert_speed) }, \
         { "mea_mass", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_rocket_flight_tm_t, mea_mass) }, \
         { "mea_apogee", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_rocket_flight_tm_t, mea_apogee) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_rocket_flight_tm_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_rocket_flight_tm_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_rocket_flight_tm_t, acc_z) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_rocket_flight_tm_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_rocket_flight_tm_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_rocket_flight_tm_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_rocket_flight_tm_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_rocket_flight_tm_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_rocket_flight_tm_t, mag_z) }, \
         { "vn100_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_rocket_flight_tm_t, vn100_qx) }, \
         { "vn100_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_rocket_flight_tm_t, vn100_qy) }, \
         { "vn100_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_rocket_flight_tm_t, vn100_qz) }, \
         { "vn100_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_rocket_flight_tm_t, vn100_qw) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_rocket_flight_tm_t, gps_fix) }, \
         { "gps_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_rocket_flight_tm_t, gps_lat) }, \
         { "gps_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_rocket_flight_tm_t, gps_lon) }, \
         { "gps_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_rocket_flight_tm_t, gps_alt) }, \
         { "fmm_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_rocket_flight_tm_t, fmm_state) }, \
         { "abk_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_rocket_flight_tm_t, abk_angle) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_rocket_flight_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_rocket_flight_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_rocket_flight_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_rocket_flight_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_rocket_flight_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_rocket_flight_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_rocket_flight_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_rocket_flight_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_rocket_flight_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_rocket_flight_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 152, offsetof(mavlink_rocket_flight_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 156, offsetof(mavlink_rocket_flight_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 160, offsetof(mavlink_rocket_flight_tm_t, nas_bias_z) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 164, offsetof(mavlink_rocket_flight_tm_t, battery_voltage) }, \
         { "cam_battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 168, offsetof(mavlink_rocket_flight_tm_t, cam_battery_voltage) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 172, offsetof(mavlink_rocket_flight_tm_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a rocket_flight_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param mea_mass [kg] Mass estimated by MEA
 * @param mea_apogee [m] MEA estimated apogee
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param vn100_qx  VN100 estimated attitude (qx)
 * @param vn100_qy  VN100 estimated attitude (qy)
 * @param vn100_qz  VN100 estimated attitude (qz)
 * @param vn100_qw  VN100 estimated attitude (qw)
 * @param gps_fix  Wether the GPS has a FIX
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param fmm_state  Flight Mode Manager State
 * @param abk_angle  Air Brakes angle
 * @param nas_n [deg] NAS estimated noth position
 * @param nas_e [deg] NAS estimated east position
 * @param nas_d [m] NAS estimated down position
 * @param nas_vn [m/s] NAS estimated north velocity
 * @param nas_ve [m/s] NAS estimated east velocity
 * @param nas_vd [m/s] NAS estimated down velocity
 * @param nas_qx  NAS estimated attitude (qx)
 * @param nas_qy  NAS estimated attitude (qy)
 * @param nas_qz  NAS estimated attitude (qz)
 * @param nas_qw  NAS estimated attitude (qw)
 * @param nas_bias_x  NAS gyroscope bias on x axis
 * @param nas_bias_y  NAS gyroscope bias on y axis
 * @param nas_bias_z  NAS gyroscope bias on z axis
 * @param battery_voltage [V] Battery voltage
 * @param cam_battery_voltage [V] Camera battery voltage
 * @param temperature [degC] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float mea_mass, float mea_apogee, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, float vn100_qx, float vn100_qy, float vn100_qz, float vn100_qw, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, uint8_t fmm_state, float abk_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float battery_voltage, float cam_battery_voltage, float temperature)
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
    _mav_put_float(buf, 36, mea_mass);
    _mav_put_float(buf, 40, mea_apogee);
    _mav_put_float(buf, 44, acc_x);
    _mav_put_float(buf, 48, acc_y);
    _mav_put_float(buf, 52, acc_z);
    _mav_put_float(buf, 56, gyro_x);
    _mav_put_float(buf, 60, gyro_y);
    _mav_put_float(buf, 64, gyro_z);
    _mav_put_float(buf, 68, mag_x);
    _mav_put_float(buf, 72, mag_y);
    _mav_put_float(buf, 76, mag_z);
    _mav_put_float(buf, 80, vn100_qx);
    _mav_put_float(buf, 84, vn100_qy);
    _mav_put_float(buf, 88, vn100_qz);
    _mav_put_float(buf, 92, vn100_qw);
    _mav_put_float(buf, 96, gps_lat);
    _mav_put_float(buf, 100, gps_lon);
    _mav_put_float(buf, 104, gps_alt);
    _mav_put_float(buf, 108, abk_angle);
    _mav_put_float(buf, 112, nas_n);
    _mav_put_float(buf, 116, nas_e);
    _mav_put_float(buf, 120, nas_d);
    _mav_put_float(buf, 124, nas_vn);
    _mav_put_float(buf, 128, nas_ve);
    _mav_put_float(buf, 132, nas_vd);
    _mav_put_float(buf, 136, nas_qx);
    _mav_put_float(buf, 140, nas_qy);
    _mav_put_float(buf, 144, nas_qz);
    _mav_put_float(buf, 148, nas_qw);
    _mav_put_float(buf, 152, nas_bias_x);
    _mav_put_float(buf, 156, nas_bias_y);
    _mav_put_float(buf, 160, nas_bias_z);
    _mav_put_float(buf, 164, battery_voltage);
    _mav_put_float(buf, 168, cam_battery_voltage);
    _mav_put_float(buf, 172, temperature);
    _mav_put_uint8_t(buf, 176, gps_fix);
    _mav_put_uint8_t(buf, 177, fmm_state);

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
    packet.mea_mass = mea_mass;
    packet.mea_apogee = mea_apogee;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.vn100_qx = vn100_qx;
    packet.vn100_qy = vn100_qy;
    packet.vn100_qz = vn100_qz;
    packet.vn100_qw = vn100_qw;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.abk_angle = abk_angle;
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
    packet.battery_voltage = battery_voltage;
    packet.cam_battery_voltage = cam_battery_voltage;
    packet.temperature = temperature;
    packet.gps_fix = gps_fix;
    packet.fmm_state = fmm_state;

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
 * @param timestamp [us] Timestamp in microseconds
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param mea_mass [kg] Mass estimated by MEA
 * @param mea_apogee [m] MEA estimated apogee
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param vn100_qx  VN100 estimated attitude (qx)
 * @param vn100_qy  VN100 estimated attitude (qy)
 * @param vn100_qz  VN100 estimated attitude (qz)
 * @param vn100_qw  VN100 estimated attitude (qw)
 * @param gps_fix  Wether the GPS has a FIX
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param fmm_state  Flight Mode Manager State
 * @param abk_angle  Air Brakes angle
 * @param nas_n [deg] NAS estimated noth position
 * @param nas_e [deg] NAS estimated east position
 * @param nas_d [m] NAS estimated down position
 * @param nas_vn [m/s] NAS estimated north velocity
 * @param nas_ve [m/s] NAS estimated east velocity
 * @param nas_vd [m/s] NAS estimated down velocity
 * @param nas_qx  NAS estimated attitude (qx)
 * @param nas_qy  NAS estimated attitude (qy)
 * @param nas_qz  NAS estimated attitude (qz)
 * @param nas_qw  NAS estimated attitude (qw)
 * @param nas_bias_x  NAS gyroscope bias on x axis
 * @param nas_bias_y  NAS gyroscope bias on y axis
 * @param nas_bias_z  NAS gyroscope bias on z axis
 * @param battery_voltage [V] Battery voltage
 * @param cam_battery_voltage [V] Camera battery voltage
 * @param temperature [degC] Temperature
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rocket_flight_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float pressure_ada,float pressure_digi,float pressure_static,float pressure_dpl,float airspeed_pitot,float altitude_agl,float ada_vert_speed,float mea_mass,float mea_apogee,float acc_x,float acc_y,float acc_z,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z,float vn100_qx,float vn100_qy,float vn100_qz,float vn100_qw,uint8_t gps_fix,float gps_lat,float gps_lon,float gps_alt,uint8_t fmm_state,float abk_angle,float nas_n,float nas_e,float nas_d,float nas_vn,float nas_ve,float nas_vd,float nas_qx,float nas_qy,float nas_qz,float nas_qw,float nas_bias_x,float nas_bias_y,float nas_bias_z,float battery_voltage,float cam_battery_voltage,float temperature)
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
    _mav_put_float(buf, 36, mea_mass);
    _mav_put_float(buf, 40, mea_apogee);
    _mav_put_float(buf, 44, acc_x);
    _mav_put_float(buf, 48, acc_y);
    _mav_put_float(buf, 52, acc_z);
    _mav_put_float(buf, 56, gyro_x);
    _mav_put_float(buf, 60, gyro_y);
    _mav_put_float(buf, 64, gyro_z);
    _mav_put_float(buf, 68, mag_x);
    _mav_put_float(buf, 72, mag_y);
    _mav_put_float(buf, 76, mag_z);
    _mav_put_float(buf, 80, vn100_qx);
    _mav_put_float(buf, 84, vn100_qy);
    _mav_put_float(buf, 88, vn100_qz);
    _mav_put_float(buf, 92, vn100_qw);
    _mav_put_float(buf, 96, gps_lat);
    _mav_put_float(buf, 100, gps_lon);
    _mav_put_float(buf, 104, gps_alt);
    _mav_put_float(buf, 108, abk_angle);
    _mav_put_float(buf, 112, nas_n);
    _mav_put_float(buf, 116, nas_e);
    _mav_put_float(buf, 120, nas_d);
    _mav_put_float(buf, 124, nas_vn);
    _mav_put_float(buf, 128, nas_ve);
    _mav_put_float(buf, 132, nas_vd);
    _mav_put_float(buf, 136, nas_qx);
    _mav_put_float(buf, 140, nas_qy);
    _mav_put_float(buf, 144, nas_qz);
    _mav_put_float(buf, 148, nas_qw);
    _mav_put_float(buf, 152, nas_bias_x);
    _mav_put_float(buf, 156, nas_bias_y);
    _mav_put_float(buf, 160, nas_bias_z);
    _mav_put_float(buf, 164, battery_voltage);
    _mav_put_float(buf, 168, cam_battery_voltage);
    _mav_put_float(buf, 172, temperature);
    _mav_put_uint8_t(buf, 176, gps_fix);
    _mav_put_uint8_t(buf, 177, fmm_state);

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
    packet.mea_mass = mea_mass;
    packet.mea_apogee = mea_apogee;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.vn100_qx = vn100_qx;
    packet.vn100_qy = vn100_qy;
    packet.vn100_qz = vn100_qz;
    packet.vn100_qw = vn100_qw;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.abk_angle = abk_angle;
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
    packet.battery_voltage = battery_voltage;
    packet.cam_battery_voltage = cam_battery_voltage;
    packet.temperature = temperature;
    packet.gps_fix = gps_fix;
    packet.fmm_state = fmm_state;

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
    return mavlink_msg_rocket_flight_tm_pack(system_id, component_id, msg, rocket_flight_tm->timestamp, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->mea_mass, rocket_flight_tm->mea_apogee, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->vn100_qx, rocket_flight_tm->vn100_qy, rocket_flight_tm->vn100_qz, rocket_flight_tm->vn100_qw, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->fmm_state, rocket_flight_tm->abk_angle, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->battery_voltage, rocket_flight_tm->cam_battery_voltage, rocket_flight_tm->temperature);
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
    return mavlink_msg_rocket_flight_tm_pack_chan(system_id, component_id, chan, msg, rocket_flight_tm->timestamp, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->mea_mass, rocket_flight_tm->mea_apogee, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->vn100_qx, rocket_flight_tm->vn100_qy, rocket_flight_tm->vn100_qz, rocket_flight_tm->vn100_qw, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->fmm_state, rocket_flight_tm->abk_angle, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->battery_voltage, rocket_flight_tm->cam_battery_voltage, rocket_flight_tm->temperature);
}

/**
 * @brief Send a rocket_flight_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param pressure_ada [Pa] Atmospheric pressure estimated by ADA
 * @param pressure_digi [Pa] Pressure from digital sensor
 * @param pressure_static [Pa] Pressure from static port
 * @param pressure_dpl [Pa] Pressure from deployment vane sensor
 * @param airspeed_pitot [m/s] Pitot airspeed
 * @param altitude_agl [m] Altitude above ground level
 * @param ada_vert_speed [m/s] Vertical speed estimated by ADA
 * @param mea_mass [kg] Mass estimated by MEA
 * @param mea_apogee [m] MEA estimated apogee
 * @param acc_x [m/s^2] Acceleration on X axis (body)
 * @param acc_y [m/s^2] Acceleration on Y axis (body)
 * @param acc_z [m/s^2] Acceleration on Z axis (body)
 * @param gyro_x [rad/s] Angular speed on X axis (body)
 * @param gyro_y [rad/s] Angular speed on Y axis (body)
 * @param gyro_z [rad/s] Angular speed on Z axis (body)
 * @param mag_x [uT] Magnetic field on X axis (body)
 * @param mag_y [uT] Magnetic field on Y axis (body)
 * @param mag_z [uT] Magnetic field on Z axis (body)
 * @param vn100_qx  VN100 estimated attitude (qx)
 * @param vn100_qy  VN100 estimated attitude (qy)
 * @param vn100_qz  VN100 estimated attitude (qz)
 * @param vn100_qw  VN100 estimated attitude (qw)
 * @param gps_fix  Wether the GPS has a FIX
 * @param gps_lat [deg] Latitude
 * @param gps_lon [deg] Longitude
 * @param gps_alt [m] GPS Altitude
 * @param fmm_state  Flight Mode Manager State
 * @param abk_angle  Air Brakes angle
 * @param nas_n [deg] NAS estimated noth position
 * @param nas_e [deg] NAS estimated east position
 * @param nas_d [m] NAS estimated down position
 * @param nas_vn [m/s] NAS estimated north velocity
 * @param nas_ve [m/s] NAS estimated east velocity
 * @param nas_vd [m/s] NAS estimated down velocity
 * @param nas_qx  NAS estimated attitude (qx)
 * @param nas_qy  NAS estimated attitude (qy)
 * @param nas_qz  NAS estimated attitude (qz)
 * @param nas_qw  NAS estimated attitude (qw)
 * @param nas_bias_x  NAS gyroscope bias on x axis
 * @param nas_bias_y  NAS gyroscope bias on y axis
 * @param nas_bias_z  NAS gyroscope bias on z axis
 * @param battery_voltage [V] Battery voltage
 * @param cam_battery_voltage [V] Camera battery voltage
 * @param temperature [degC] Temperature
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rocket_flight_tm_send(mavlink_channel_t chan, uint64_t timestamp, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float mea_mass, float mea_apogee, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, float vn100_qx, float vn100_qy, float vn100_qz, float vn100_qw, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, uint8_t fmm_state, float abk_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float battery_voltage, float cam_battery_voltage, float temperature)
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
    _mav_put_float(buf, 36, mea_mass);
    _mav_put_float(buf, 40, mea_apogee);
    _mav_put_float(buf, 44, acc_x);
    _mav_put_float(buf, 48, acc_y);
    _mav_put_float(buf, 52, acc_z);
    _mav_put_float(buf, 56, gyro_x);
    _mav_put_float(buf, 60, gyro_y);
    _mav_put_float(buf, 64, gyro_z);
    _mav_put_float(buf, 68, mag_x);
    _mav_put_float(buf, 72, mag_y);
    _mav_put_float(buf, 76, mag_z);
    _mav_put_float(buf, 80, vn100_qx);
    _mav_put_float(buf, 84, vn100_qy);
    _mav_put_float(buf, 88, vn100_qz);
    _mav_put_float(buf, 92, vn100_qw);
    _mav_put_float(buf, 96, gps_lat);
    _mav_put_float(buf, 100, gps_lon);
    _mav_put_float(buf, 104, gps_alt);
    _mav_put_float(buf, 108, abk_angle);
    _mav_put_float(buf, 112, nas_n);
    _mav_put_float(buf, 116, nas_e);
    _mav_put_float(buf, 120, nas_d);
    _mav_put_float(buf, 124, nas_vn);
    _mav_put_float(buf, 128, nas_ve);
    _mav_put_float(buf, 132, nas_vd);
    _mav_put_float(buf, 136, nas_qx);
    _mav_put_float(buf, 140, nas_qy);
    _mav_put_float(buf, 144, nas_qz);
    _mav_put_float(buf, 148, nas_qw);
    _mav_put_float(buf, 152, nas_bias_x);
    _mav_put_float(buf, 156, nas_bias_y);
    _mav_put_float(buf, 160, nas_bias_z);
    _mav_put_float(buf, 164, battery_voltage);
    _mav_put_float(buf, 168, cam_battery_voltage);
    _mav_put_float(buf, 172, temperature);
    _mav_put_uint8_t(buf, 176, gps_fix);
    _mav_put_uint8_t(buf, 177, fmm_state);

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
    packet.mea_mass = mea_mass;
    packet.mea_apogee = mea_apogee;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.vn100_qx = vn100_qx;
    packet.vn100_qy = vn100_qy;
    packet.vn100_qz = vn100_qz;
    packet.vn100_qw = vn100_qw;
    packet.gps_lat = gps_lat;
    packet.gps_lon = gps_lon;
    packet.gps_alt = gps_alt;
    packet.abk_angle = abk_angle;
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
    packet.battery_voltage = battery_voltage;
    packet.cam_battery_voltage = cam_battery_voltage;
    packet.temperature = temperature;
    packet.gps_fix = gps_fix;
    packet.fmm_state = fmm_state;

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
    mavlink_msg_rocket_flight_tm_send(chan, rocket_flight_tm->timestamp, rocket_flight_tm->pressure_ada, rocket_flight_tm->pressure_digi, rocket_flight_tm->pressure_static, rocket_flight_tm->pressure_dpl, rocket_flight_tm->airspeed_pitot, rocket_flight_tm->altitude_agl, rocket_flight_tm->ada_vert_speed, rocket_flight_tm->mea_mass, rocket_flight_tm->mea_apogee, rocket_flight_tm->acc_x, rocket_flight_tm->acc_y, rocket_flight_tm->acc_z, rocket_flight_tm->gyro_x, rocket_flight_tm->gyro_y, rocket_flight_tm->gyro_z, rocket_flight_tm->mag_x, rocket_flight_tm->mag_y, rocket_flight_tm->mag_z, rocket_flight_tm->vn100_qx, rocket_flight_tm->vn100_qy, rocket_flight_tm->vn100_qz, rocket_flight_tm->vn100_qw, rocket_flight_tm->gps_fix, rocket_flight_tm->gps_lat, rocket_flight_tm->gps_lon, rocket_flight_tm->gps_alt, rocket_flight_tm->fmm_state, rocket_flight_tm->abk_angle, rocket_flight_tm->nas_n, rocket_flight_tm->nas_e, rocket_flight_tm->nas_d, rocket_flight_tm->nas_vn, rocket_flight_tm->nas_ve, rocket_flight_tm->nas_vd, rocket_flight_tm->nas_qx, rocket_flight_tm->nas_qy, rocket_flight_tm->nas_qz, rocket_flight_tm->nas_qw, rocket_flight_tm->nas_bias_x, rocket_flight_tm->nas_bias_y, rocket_flight_tm->nas_bias_z, rocket_flight_tm->battery_voltage, rocket_flight_tm->cam_battery_voltage, rocket_flight_tm->temperature);
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
static inline void mavlink_msg_rocket_flight_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float pressure_ada, float pressure_digi, float pressure_static, float pressure_dpl, float airspeed_pitot, float altitude_agl, float ada_vert_speed, float mea_mass, float mea_apogee, float acc_x, float acc_y, float acc_z, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z, float vn100_qx, float vn100_qy, float vn100_qz, float vn100_qw, uint8_t gps_fix, float gps_lat, float gps_lon, float gps_alt, uint8_t fmm_state, float abk_angle, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float battery_voltage, float cam_battery_voltage, float temperature)
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
    _mav_put_float(buf, 36, mea_mass);
    _mav_put_float(buf, 40, mea_apogee);
    _mav_put_float(buf, 44, acc_x);
    _mav_put_float(buf, 48, acc_y);
    _mav_put_float(buf, 52, acc_z);
    _mav_put_float(buf, 56, gyro_x);
    _mav_put_float(buf, 60, gyro_y);
    _mav_put_float(buf, 64, gyro_z);
    _mav_put_float(buf, 68, mag_x);
    _mav_put_float(buf, 72, mag_y);
    _mav_put_float(buf, 76, mag_z);
    _mav_put_float(buf, 80, vn100_qx);
    _mav_put_float(buf, 84, vn100_qy);
    _mav_put_float(buf, 88, vn100_qz);
    _mav_put_float(buf, 92, vn100_qw);
    _mav_put_float(buf, 96, gps_lat);
    _mav_put_float(buf, 100, gps_lon);
    _mav_put_float(buf, 104, gps_alt);
    _mav_put_float(buf, 108, abk_angle);
    _mav_put_float(buf, 112, nas_n);
    _mav_put_float(buf, 116, nas_e);
    _mav_put_float(buf, 120, nas_d);
    _mav_put_float(buf, 124, nas_vn);
    _mav_put_float(buf, 128, nas_ve);
    _mav_put_float(buf, 132, nas_vd);
    _mav_put_float(buf, 136, nas_qx);
    _mav_put_float(buf, 140, nas_qy);
    _mav_put_float(buf, 144, nas_qz);
    _mav_put_float(buf, 148, nas_qw);
    _mav_put_float(buf, 152, nas_bias_x);
    _mav_put_float(buf, 156, nas_bias_y);
    _mav_put_float(buf, 160, nas_bias_z);
    _mav_put_float(buf, 164, battery_voltage);
    _mav_put_float(buf, 168, cam_battery_voltage);
    _mav_put_float(buf, 172, temperature);
    _mav_put_uint8_t(buf, 176, gps_fix);
    _mav_put_uint8_t(buf, 177, fmm_state);

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
    packet->mea_mass = mea_mass;
    packet->mea_apogee = mea_apogee;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    packet->vn100_qx = vn100_qx;
    packet->vn100_qy = vn100_qy;
    packet->vn100_qz = vn100_qz;
    packet->vn100_qw = vn100_qw;
    packet->gps_lat = gps_lat;
    packet->gps_lon = gps_lon;
    packet->gps_alt = gps_alt;
    packet->abk_angle = abk_angle;
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
    packet->battery_voltage = battery_voltage;
    packet->cam_battery_voltage = cam_battery_voltage;
    packet->temperature = temperature;
    packet->gps_fix = gps_fix;
    packet->fmm_state = fmm_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM, (const char *)packet, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ROCKET_FLIGHT_TM UNPACKING


/**
 * @brief Get field timestamp from rocket_flight_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_rocket_flight_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
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
 * @brief Get field mea_mass from rocket_flight_tm message
 *
 * @return [kg] Mass estimated by MEA
 */
static inline float mavlink_msg_rocket_flight_tm_get_mea_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field mea_apogee from rocket_flight_tm message
 *
 * @return [m] MEA estimated apogee
 */
static inline float mavlink_msg_rocket_flight_tm_get_mea_apogee(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field acc_x from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field acc_y from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field acc_z from rocket_flight_tm message
 *
 * @return [m/s^2] Acceleration on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field gyro_x from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field gyro_y from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field gyro_z from rocket_flight_tm message
 *
 * @return [rad/s] Angular speed on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field mag_x from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on X axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field mag_y from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on Y axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field mag_z from rocket_flight_tm message
 *
 * @return [uT] Magnetic field on Z axis (body)
 */
static inline float mavlink_msg_rocket_flight_tm_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field vn100_qx from rocket_flight_tm message
 *
 * @return  VN100 estimated attitude (qx)
 */
static inline float mavlink_msg_rocket_flight_tm_get_vn100_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field vn100_qy from rocket_flight_tm message
 *
 * @return  VN100 estimated attitude (qy)
 */
static inline float mavlink_msg_rocket_flight_tm_get_vn100_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field vn100_qz from rocket_flight_tm message
 *
 * @return  VN100 estimated attitude (qz)
 */
static inline float mavlink_msg_rocket_flight_tm_get_vn100_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field vn100_qw from rocket_flight_tm message
 *
 * @return  VN100 estimated attitude (qw)
 */
static inline float mavlink_msg_rocket_flight_tm_get_vn100_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field gps_fix from rocket_flight_tm message
 *
 * @return  Wether the GPS has a FIX
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  176);
}

/**
 * @brief Get field gps_lat from rocket_flight_tm message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field gps_lon from rocket_flight_tm message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field gps_alt from rocket_flight_tm message
 *
 * @return [m] GPS Altitude
 */
static inline float mavlink_msg_rocket_flight_tm_get_gps_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field fmm_state from rocket_flight_tm message
 *
 * @return  Flight Mode Manager State
 */
static inline uint8_t mavlink_msg_rocket_flight_tm_get_fmm_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  177);
}

/**
 * @brief Get field abk_angle from rocket_flight_tm message
 *
 * @return  Air Brakes angle
 */
static inline float mavlink_msg_rocket_flight_tm_get_abk_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field nas_n from rocket_flight_tm message
 *
 * @return [deg] NAS estimated noth position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field nas_e from rocket_flight_tm message
 *
 * @return [deg] NAS estimated east position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field nas_d from rocket_flight_tm message
 *
 * @return [m] NAS estimated down position
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field nas_vn from rocket_flight_tm message
 *
 * @return [m/s] NAS estimated north velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field nas_ve from rocket_flight_tm message
 *
 * @return [m/s] NAS estimated east velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field nas_vd from rocket_flight_tm message
 *
 * @return [m/s] NAS estimated down velocity
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field nas_qx from rocket_flight_tm message
 *
 * @return  NAS estimated attitude (qx)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field nas_qy from rocket_flight_tm message
 *
 * @return  NAS estimated attitude (qy)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field nas_qz from rocket_flight_tm message
 *
 * @return  NAS estimated attitude (qz)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field nas_qw from rocket_flight_tm message
 *
 * @return  NAS estimated attitude (qw)
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field nas_bias_x from rocket_flight_tm message
 *
 * @return  NAS gyroscope bias on x axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  152);
}

/**
 * @brief Get field nas_bias_y from rocket_flight_tm message
 *
 * @return  NAS gyroscope bias on y axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  156);
}

/**
 * @brief Get field nas_bias_z from rocket_flight_tm message
 *
 * @return  NAS gyroscope bias on z axis
 */
static inline float mavlink_msg_rocket_flight_tm_get_nas_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  160);
}

/**
 * @brief Get field battery_voltage from rocket_flight_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_rocket_flight_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  164);
}

/**
 * @brief Get field cam_battery_voltage from rocket_flight_tm message
 *
 * @return [V] Camera battery voltage
 */
static inline float mavlink_msg_rocket_flight_tm_get_cam_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  168);
}

/**
 * @brief Get field temperature from rocket_flight_tm message
 *
 * @return [degC] Temperature
 */
static inline float mavlink_msg_rocket_flight_tm_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  172);
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
    rocket_flight_tm->mea_mass = mavlink_msg_rocket_flight_tm_get_mea_mass(msg);
    rocket_flight_tm->mea_apogee = mavlink_msg_rocket_flight_tm_get_mea_apogee(msg);
    rocket_flight_tm->acc_x = mavlink_msg_rocket_flight_tm_get_acc_x(msg);
    rocket_flight_tm->acc_y = mavlink_msg_rocket_flight_tm_get_acc_y(msg);
    rocket_flight_tm->acc_z = mavlink_msg_rocket_flight_tm_get_acc_z(msg);
    rocket_flight_tm->gyro_x = mavlink_msg_rocket_flight_tm_get_gyro_x(msg);
    rocket_flight_tm->gyro_y = mavlink_msg_rocket_flight_tm_get_gyro_y(msg);
    rocket_flight_tm->gyro_z = mavlink_msg_rocket_flight_tm_get_gyro_z(msg);
    rocket_flight_tm->mag_x = mavlink_msg_rocket_flight_tm_get_mag_x(msg);
    rocket_flight_tm->mag_y = mavlink_msg_rocket_flight_tm_get_mag_y(msg);
    rocket_flight_tm->mag_z = mavlink_msg_rocket_flight_tm_get_mag_z(msg);
    rocket_flight_tm->vn100_qx = mavlink_msg_rocket_flight_tm_get_vn100_qx(msg);
    rocket_flight_tm->vn100_qy = mavlink_msg_rocket_flight_tm_get_vn100_qy(msg);
    rocket_flight_tm->vn100_qz = mavlink_msg_rocket_flight_tm_get_vn100_qz(msg);
    rocket_flight_tm->vn100_qw = mavlink_msg_rocket_flight_tm_get_vn100_qw(msg);
    rocket_flight_tm->gps_lat = mavlink_msg_rocket_flight_tm_get_gps_lat(msg);
    rocket_flight_tm->gps_lon = mavlink_msg_rocket_flight_tm_get_gps_lon(msg);
    rocket_flight_tm->gps_alt = mavlink_msg_rocket_flight_tm_get_gps_alt(msg);
    rocket_flight_tm->abk_angle = mavlink_msg_rocket_flight_tm_get_abk_angle(msg);
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
    rocket_flight_tm->battery_voltage = mavlink_msg_rocket_flight_tm_get_battery_voltage(msg);
    rocket_flight_tm->cam_battery_voltage = mavlink_msg_rocket_flight_tm_get_cam_battery_voltage(msg);
    rocket_flight_tm->temperature = mavlink_msg_rocket_flight_tm_get_temperature(msg);
    rocket_flight_tm->gps_fix = mavlink_msg_rocket_flight_tm_get_gps_fix(msg);
    rocket_flight_tm->fmm_state = mavlink_msg_rocket_flight_tm_get_fmm_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN? msg->len : MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN;
        memset(rocket_flight_tm, 0, MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_LEN);
    memcpy(rocket_flight_tm, _MAV_PAYLOAD(msg), len);
#endif
}
