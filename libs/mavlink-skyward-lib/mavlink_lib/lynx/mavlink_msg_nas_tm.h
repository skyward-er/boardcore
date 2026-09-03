#pragma once
// MESSAGE NAS_TM PACKING

#define MAVLINK_MSG_ID_NAS_TM 169

MAVPACKED(
typedef struct __mavlink_nas_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged*/
 float x0; /*< [m] Kalman state variable 0 (position x)*/
 float x1; /*< [m] Kalman state variable 1 (position y)*/
 float x2; /*< [m] >Kalman state variable 2 (position z)*/
 float x3; /*< [m/s] >Kalman state variable 3 (velocity x)*/
 float x4; /*< [m/s] >Kalman state variable 4 (velocity y)*/
 float x5; /*< [m/s] >Kalman state variable 5 (velocity z)*/
 float x6; /*<  Kalman state variable 6 (q0)*/
 float x7; /*<  Kalman state variable 7 (q1)*/
 float x8; /*<  Kalman state variable 8 (q2)*/
 float x9; /*<  Kalman state variable 9 (q3)*/
 float x10; /*<  Kalman state variable 10 (bias)*/
 float x11; /*<  Kalman state variable 11 (bias)*/
 float x12; /*<  Kalman state variable 12 (bias)*/
 float ref_pressure; /*< [Pa] Calibration pressure*/
 float ref_temperature; /*< [degC] Calibration temperature*/
 float ref_latitude; /*< [deg] Calibration latitude*/
 float ref_longitude; /*< [deg] Calibration longitude*/
 float ref_accel_x; /*< [m/s2] Calibration acceleration on x axis*/
 float ref_accel_y; /*< [m/s2] Calibration acceleration on y axis*/
 float ref_accel_z; /*< [m/s2] Calibration acceleration on z axis*/
 float ref_mag_x; /*< [uT] Calibration compass on x axis*/
 float ref_mag_y; /*< [uT] Calibration compass on y axis*/
 float ref_mag_z; /*< [uT] Calibration compass on z axis*/
 float triad_roll; /*<  Orientation on roll estimated by TRIAD initialization*/
 float triad_pitch; /*<  Orientation on pitch estimated by TRIAD initialization*/
 float triad_yaw; /*<  Orientation on yaw estimated by TRIAD initialization*/
 float roll; /*< [deg] Orientation on roll*/
 float pitch; /*< [deg] Orientation on pitch*/
 float yaw; /*< [deg] Orientation on yaw*/
 uint8_t state; /*<  NAS current state*/
}) mavlink_nas_tm_t;

#define MAVLINK_MSG_ID_NAS_TM_LEN 125
#define MAVLINK_MSG_ID_NAS_TM_MIN_LEN 125
#define MAVLINK_MSG_ID_169_LEN 125
#define MAVLINK_MSG_ID_169_MIN_LEN 125

#define MAVLINK_MSG_ID_NAS_TM_CRC 14
#define MAVLINK_MSG_ID_169_CRC 14



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAS_TM { \
    169, \
    "NAS_TM", \
    31, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nas_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 124, offsetof(mavlink_nas_tm_t, state) }, \
         { "x0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nas_tm_t, x0) }, \
         { "x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nas_tm_t, x1) }, \
         { "x2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nas_tm_t, x2) }, \
         { "x3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nas_tm_t, x3) }, \
         { "x4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nas_tm_t, x4) }, \
         { "x5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nas_tm_t, x5) }, \
         { "x6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_nas_tm_t, x6) }, \
         { "x7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_nas_tm_t, x7) }, \
         { "x8", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_nas_tm_t, x8) }, \
         { "x9", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_nas_tm_t, x9) }, \
         { "x10", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_nas_tm_t, x10) }, \
         { "x11", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_nas_tm_t, x11) }, \
         { "x12", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_nas_tm_t, x12) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_nas_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_nas_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_nas_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_nas_tm_t, ref_longitude) }, \
         { "ref_accel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_nas_tm_t, ref_accel_x) }, \
         { "ref_accel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_nas_tm_t, ref_accel_y) }, \
         { "ref_accel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_nas_tm_t, ref_accel_z) }, \
         { "ref_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_nas_tm_t, ref_mag_x) }, \
         { "ref_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_nas_tm_t, ref_mag_y) }, \
         { "ref_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_nas_tm_t, ref_mag_z) }, \
         { "triad_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_nas_tm_t, triad_roll) }, \
         { "triad_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_nas_tm_t, triad_pitch) }, \
         { "triad_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_nas_tm_t, triad_yaw) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_nas_tm_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_nas_tm_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_nas_tm_t, yaw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAS_TM { \
    "NAS_TM", \
    31, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nas_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 124, offsetof(mavlink_nas_tm_t, state) }, \
         { "x0", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nas_tm_t, x0) }, \
         { "x1", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nas_tm_t, x1) }, \
         { "x2", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nas_tm_t, x2) }, \
         { "x3", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nas_tm_t, x3) }, \
         { "x4", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nas_tm_t, x4) }, \
         { "x5", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nas_tm_t, x5) }, \
         { "x6", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_nas_tm_t, x6) }, \
         { "x7", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_nas_tm_t, x7) }, \
         { "x8", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_nas_tm_t, x8) }, \
         { "x9", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_nas_tm_t, x9) }, \
         { "x10", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_nas_tm_t, x10) }, \
         { "x11", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_nas_tm_t, x11) }, \
         { "x12", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_nas_tm_t, x12) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_nas_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_nas_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_nas_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_nas_tm_t, ref_longitude) }, \
         { "ref_accel_x", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_nas_tm_t, ref_accel_x) }, \
         { "ref_accel_y", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_nas_tm_t, ref_accel_y) }, \
         { "ref_accel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_nas_tm_t, ref_accel_z) }, \
         { "ref_mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_nas_tm_t, ref_mag_x) }, \
         { "ref_mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_nas_tm_t, ref_mag_y) }, \
         { "ref_mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_nas_tm_t, ref_mag_z) }, \
         { "triad_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_nas_tm_t, triad_roll) }, \
         { "triad_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_nas_tm_t, triad_pitch) }, \
         { "triad_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_nas_tm_t, triad_yaw) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_nas_tm_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_nas_tm_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_nas_tm_t, yaw) }, \
         } \
}
#endif

/**
 * @brief Pack a nas_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged
 * @param state  NAS current state
 * @param x0 [m] Kalman state variable 0 (position x)
 * @param x1 [m] Kalman state variable 1 (position y)
 * @param x2 [m] >Kalman state variable 2 (position z)
 * @param x3 [m/s] >Kalman state variable 3 (velocity x)
 * @param x4 [m/s] >Kalman state variable 4 (velocity y)
 * @param x5 [m/s] >Kalman state variable 5 (velocity z)
 * @param x6  Kalman state variable 6 (q0)
 * @param x7  Kalman state variable 7 (q1)
 * @param x8  Kalman state variable 8 (q2)
 * @param x9  Kalman state variable 9 (q3)
 * @param x10  Kalman state variable 10 (bias)
 * @param x11  Kalman state variable 11 (bias)
 * @param x12  Kalman state variable 12 (bias)
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 * @param ref_accel_x [m/s2] Calibration acceleration on x axis
 * @param ref_accel_y [m/s2] Calibration acceleration on y axis
 * @param ref_accel_z [m/s2] Calibration acceleration on z axis
 * @param ref_mag_x [uT] Calibration compass on x axis
 * @param ref_mag_y [uT] Calibration compass on y axis
 * @param ref_mag_z [uT] Calibration compass on z axis
 * @param triad_roll  Orientation on roll estimated by TRIAD initialization
 * @param triad_pitch  Orientation on pitch estimated by TRIAD initialization
 * @param triad_yaw  Orientation on yaw estimated by TRIAD initialization
 * @param roll [deg] Orientation on roll
 * @param pitch [deg] Orientation on pitch
 * @param yaw [deg] Orientation on yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nas_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float x0, float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9, float x10, float x11, float x12, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float ref_accel_x, float ref_accel_y, float ref_accel_z, float ref_mag_x, float ref_mag_y, float ref_mag_z, float triad_roll, float triad_pitch, float triad_yaw, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, x2);
    _mav_put_float(buf, 20, x3);
    _mav_put_float(buf, 24, x4);
    _mav_put_float(buf, 28, x5);
    _mav_put_float(buf, 32, x6);
    _mav_put_float(buf, 36, x7);
    _mav_put_float(buf, 40, x8);
    _mav_put_float(buf, 44, x9);
    _mav_put_float(buf, 48, x10);
    _mav_put_float(buf, 52, x11);
    _mav_put_float(buf, 56, x12);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_float(buf, 76, ref_accel_x);
    _mav_put_float(buf, 80, ref_accel_y);
    _mav_put_float(buf, 84, ref_accel_z);
    _mav_put_float(buf, 88, ref_mag_x);
    _mav_put_float(buf, 92, ref_mag_y);
    _mav_put_float(buf, 96, ref_mag_z);
    _mav_put_float(buf, 100, triad_roll);
    _mav_put_float(buf, 104, triad_pitch);
    _mav_put_float(buf, 108, triad_yaw);
    _mav_put_float(buf, 112, roll);
    _mav_put_float(buf, 116, pitch);
    _mav_put_float(buf, 120, yaw);
    _mav_put_uint8_t(buf, 124, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAS_TM_LEN);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.x0 = x0;
    packet.x1 = x1;
    packet.x2 = x2;
    packet.x3 = x3;
    packet.x4 = x4;
    packet.x5 = x5;
    packet.x6 = x6;
    packet.x7 = x7;
    packet.x8 = x8;
    packet.x9 = x9;
    packet.x10 = x10;
    packet.x11 = x11;
    packet.x12 = x12;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.ref_accel_x = ref_accel_x;
    packet.ref_accel_y = ref_accel_y;
    packet.ref_accel_z = ref_accel_z;
    packet.ref_mag_x = ref_mag_x;
    packet.ref_mag_y = ref_mag_y;
    packet.ref_mag_z = ref_mag_z;
    packet.triad_roll = triad_roll;
    packet.triad_pitch = triad_pitch;
    packet.triad_yaw = triad_yaw;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
}

/**
 * @brief Pack a nas_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged
 * @param state  NAS current state
 * @param x0 [m] Kalman state variable 0 (position x)
 * @param x1 [m] Kalman state variable 1 (position y)
 * @param x2 [m] >Kalman state variable 2 (position z)
 * @param x3 [m/s] >Kalman state variable 3 (velocity x)
 * @param x4 [m/s] >Kalman state variable 4 (velocity y)
 * @param x5 [m/s] >Kalman state variable 5 (velocity z)
 * @param x6  Kalman state variable 6 (q0)
 * @param x7  Kalman state variable 7 (q1)
 * @param x8  Kalman state variable 8 (q2)
 * @param x9  Kalman state variable 9 (q3)
 * @param x10  Kalman state variable 10 (bias)
 * @param x11  Kalman state variable 11 (bias)
 * @param x12  Kalman state variable 12 (bias)
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 * @param ref_accel_x [m/s2] Calibration acceleration on x axis
 * @param ref_accel_y [m/s2] Calibration acceleration on y axis
 * @param ref_accel_z [m/s2] Calibration acceleration on z axis
 * @param ref_mag_x [uT] Calibration compass on x axis
 * @param ref_mag_y [uT] Calibration compass on y axis
 * @param ref_mag_z [uT] Calibration compass on z axis
 * @param triad_roll  Orientation on roll estimated by TRIAD initialization
 * @param triad_pitch  Orientation on pitch estimated by TRIAD initialization
 * @param triad_yaw  Orientation on yaw estimated by TRIAD initialization
 * @param roll [deg] Orientation on roll
 * @param pitch [deg] Orientation on pitch
 * @param yaw [deg] Orientation on yaw
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nas_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float x0,float x1,float x2,float x3,float x4,float x5,float x6,float x7,float x8,float x9,float x10,float x11,float x12,float ref_pressure,float ref_temperature,float ref_latitude,float ref_longitude,float ref_accel_x,float ref_accel_y,float ref_accel_z,float ref_mag_x,float ref_mag_y,float ref_mag_z,float triad_roll,float triad_pitch,float triad_yaw,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, x2);
    _mav_put_float(buf, 20, x3);
    _mav_put_float(buf, 24, x4);
    _mav_put_float(buf, 28, x5);
    _mav_put_float(buf, 32, x6);
    _mav_put_float(buf, 36, x7);
    _mav_put_float(buf, 40, x8);
    _mav_put_float(buf, 44, x9);
    _mav_put_float(buf, 48, x10);
    _mav_put_float(buf, 52, x11);
    _mav_put_float(buf, 56, x12);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_float(buf, 76, ref_accel_x);
    _mav_put_float(buf, 80, ref_accel_y);
    _mav_put_float(buf, 84, ref_accel_z);
    _mav_put_float(buf, 88, ref_mag_x);
    _mav_put_float(buf, 92, ref_mag_y);
    _mav_put_float(buf, 96, ref_mag_z);
    _mav_put_float(buf, 100, triad_roll);
    _mav_put_float(buf, 104, triad_pitch);
    _mav_put_float(buf, 108, triad_yaw);
    _mav_put_float(buf, 112, roll);
    _mav_put_float(buf, 116, pitch);
    _mav_put_float(buf, 120, yaw);
    _mav_put_uint8_t(buf, 124, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAS_TM_LEN);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.x0 = x0;
    packet.x1 = x1;
    packet.x2 = x2;
    packet.x3 = x3;
    packet.x4 = x4;
    packet.x5 = x5;
    packet.x6 = x6;
    packet.x7 = x7;
    packet.x8 = x8;
    packet.x9 = x9;
    packet.x10 = x10;
    packet.x11 = x11;
    packet.x12 = x12;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.ref_accel_x = ref_accel_x;
    packet.ref_accel_y = ref_accel_y;
    packet.ref_accel_z = ref_accel_z;
    packet.ref_mag_x = ref_mag_x;
    packet.ref_mag_y = ref_mag_y;
    packet.ref_mag_z = ref_mag_z;
    packet.triad_roll = triad_roll;
    packet.triad_pitch = triad_pitch;
    packet.triad_yaw = triad_yaw;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
}

/**
 * @brief Encode a nas_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nas_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nas_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nas_tm_t* nas_tm)
{
    return mavlink_msg_nas_tm_pack(system_id, component_id, msg, nas_tm->timestamp, nas_tm->state, nas_tm->x0, nas_tm->x1, nas_tm->x2, nas_tm->x3, nas_tm->x4, nas_tm->x5, nas_tm->x6, nas_tm->x7, nas_tm->x8, nas_tm->x9, nas_tm->x10, nas_tm->x11, nas_tm->x12, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude, nas_tm->ref_accel_x, nas_tm->ref_accel_y, nas_tm->ref_accel_z, nas_tm->ref_mag_x, nas_tm->ref_mag_y, nas_tm->ref_mag_z, nas_tm->triad_roll, nas_tm->triad_pitch, nas_tm->triad_yaw, nas_tm->roll, nas_tm->pitch, nas_tm->yaw);
}

/**
 * @brief Encode a nas_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nas_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nas_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nas_tm_t* nas_tm)
{
    return mavlink_msg_nas_tm_pack_chan(system_id, component_id, chan, msg, nas_tm->timestamp, nas_tm->state, nas_tm->x0, nas_tm->x1, nas_tm->x2, nas_tm->x3, nas_tm->x4, nas_tm->x5, nas_tm->x6, nas_tm->x7, nas_tm->x8, nas_tm->x9, nas_tm->x10, nas_tm->x11, nas_tm->x12, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude, nas_tm->ref_accel_x, nas_tm->ref_accel_y, nas_tm->ref_accel_z, nas_tm->ref_mag_x, nas_tm->ref_mag_y, nas_tm->ref_mag_z, nas_tm->triad_roll, nas_tm->triad_pitch, nas_tm->triad_yaw, nas_tm->roll, nas_tm->pitch, nas_tm->yaw);
}

/**
 * @brief Send a nas_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged
 * @param state  NAS current state
 * @param x0 [m] Kalman state variable 0 (position x)
 * @param x1 [m] Kalman state variable 1 (position y)
 * @param x2 [m] >Kalman state variable 2 (position z)
 * @param x3 [m/s] >Kalman state variable 3 (velocity x)
 * @param x4 [m/s] >Kalman state variable 4 (velocity y)
 * @param x5 [m/s] >Kalman state variable 5 (velocity z)
 * @param x6  Kalman state variable 6 (q0)
 * @param x7  Kalman state variable 7 (q1)
 * @param x8  Kalman state variable 8 (q2)
 * @param x9  Kalman state variable 9 (q3)
 * @param x10  Kalman state variable 10 (bias)
 * @param x11  Kalman state variable 11 (bias)
 * @param x12  Kalman state variable 12 (bias)
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 * @param ref_accel_x [m/s2] Calibration acceleration on x axis
 * @param ref_accel_y [m/s2] Calibration acceleration on y axis
 * @param ref_accel_z [m/s2] Calibration acceleration on z axis
 * @param ref_mag_x [uT] Calibration compass on x axis
 * @param ref_mag_y [uT] Calibration compass on y axis
 * @param ref_mag_z [uT] Calibration compass on z axis
 * @param triad_roll  Orientation on roll estimated by TRIAD initialization
 * @param triad_pitch  Orientation on pitch estimated by TRIAD initialization
 * @param triad_yaw  Orientation on yaw estimated by TRIAD initialization
 * @param roll [deg] Orientation on roll
 * @param pitch [deg] Orientation on pitch
 * @param yaw [deg] Orientation on yaw
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nas_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float x0, float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9, float x10, float x11, float x12, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float ref_accel_x, float ref_accel_y, float ref_accel_z, float ref_mag_x, float ref_mag_y, float ref_mag_z, float triad_roll, float triad_pitch, float triad_yaw, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, x2);
    _mav_put_float(buf, 20, x3);
    _mav_put_float(buf, 24, x4);
    _mav_put_float(buf, 28, x5);
    _mav_put_float(buf, 32, x6);
    _mav_put_float(buf, 36, x7);
    _mav_put_float(buf, 40, x8);
    _mav_put_float(buf, 44, x9);
    _mav_put_float(buf, 48, x10);
    _mav_put_float(buf, 52, x11);
    _mav_put_float(buf, 56, x12);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_float(buf, 76, ref_accel_x);
    _mav_put_float(buf, 80, ref_accel_y);
    _mav_put_float(buf, 84, ref_accel_z);
    _mav_put_float(buf, 88, ref_mag_x);
    _mav_put_float(buf, 92, ref_mag_y);
    _mav_put_float(buf, 96, ref_mag_z);
    _mav_put_float(buf, 100, triad_roll);
    _mav_put_float(buf, 104, triad_pitch);
    _mav_put_float(buf, 108, triad_yaw);
    _mav_put_float(buf, 112, roll);
    _mav_put_float(buf, 116, pitch);
    _mav_put_float(buf, 120, yaw);
    _mav_put_uint8_t(buf, 124, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, buf, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.x0 = x0;
    packet.x1 = x1;
    packet.x2 = x2;
    packet.x3 = x3;
    packet.x4 = x4;
    packet.x5 = x5;
    packet.x6 = x6;
    packet.x7 = x7;
    packet.x8 = x8;
    packet.x9 = x9;
    packet.x10 = x10;
    packet.x11 = x11;
    packet.x12 = x12;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.ref_accel_x = ref_accel_x;
    packet.ref_accel_y = ref_accel_y;
    packet.ref_accel_z = ref_accel_z;
    packet.ref_mag_x = ref_mag_x;
    packet.ref_mag_y = ref_mag_y;
    packet.ref_mag_z = ref_mag_z;
    packet.triad_roll = triad_roll;
    packet.triad_pitch = triad_pitch;
    packet.triad_yaw = triad_yaw;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)&packet, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}

/**
 * @brief Send a nas_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nas_tm_send_struct(mavlink_channel_t chan, const mavlink_nas_tm_t* nas_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nas_tm_send(chan, nas_tm->timestamp, nas_tm->state, nas_tm->x0, nas_tm->x1, nas_tm->x2, nas_tm->x3, nas_tm->x4, nas_tm->x5, nas_tm->x6, nas_tm->x7, nas_tm->x8, nas_tm->x9, nas_tm->x10, nas_tm->x11, nas_tm->x12, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude, nas_tm->ref_accel_x, nas_tm->ref_accel_y, nas_tm->ref_accel_z, nas_tm->ref_mag_x, nas_tm->ref_mag_y, nas_tm->ref_mag_z, nas_tm->triad_roll, nas_tm->triad_pitch, nas_tm->triad_yaw, nas_tm->roll, nas_tm->pitch, nas_tm->yaw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)nas_tm, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nas_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float x0, float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9, float x10, float x11, float x12, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude, float ref_accel_x, float ref_accel_y, float ref_accel_z, float ref_mag_x, float ref_mag_y, float ref_mag_z, float triad_roll, float triad_pitch, float triad_yaw, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, x0);
    _mav_put_float(buf, 12, x1);
    _mav_put_float(buf, 16, x2);
    _mav_put_float(buf, 20, x3);
    _mav_put_float(buf, 24, x4);
    _mav_put_float(buf, 28, x5);
    _mav_put_float(buf, 32, x6);
    _mav_put_float(buf, 36, x7);
    _mav_put_float(buf, 40, x8);
    _mav_put_float(buf, 44, x9);
    _mav_put_float(buf, 48, x10);
    _mav_put_float(buf, 52, x11);
    _mav_put_float(buf, 56, x12);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_float(buf, 76, ref_accel_x);
    _mav_put_float(buf, 80, ref_accel_y);
    _mav_put_float(buf, 84, ref_accel_z);
    _mav_put_float(buf, 88, ref_mag_x);
    _mav_put_float(buf, 92, ref_mag_y);
    _mav_put_float(buf, 96, ref_mag_z);
    _mav_put_float(buf, 100, triad_roll);
    _mav_put_float(buf, 104, triad_pitch);
    _mav_put_float(buf, 108, triad_yaw);
    _mav_put_float(buf, 112, roll);
    _mav_put_float(buf, 116, pitch);
    _mav_put_float(buf, 120, yaw);
    _mav_put_uint8_t(buf, 124, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, buf, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#else
    mavlink_nas_tm_t *packet = (mavlink_nas_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->x0 = x0;
    packet->x1 = x1;
    packet->x2 = x2;
    packet->x3 = x3;
    packet->x4 = x4;
    packet->x5 = x5;
    packet->x6 = x6;
    packet->x7 = x7;
    packet->x8 = x8;
    packet->x9 = x9;
    packet->x10 = x10;
    packet->x11 = x11;
    packet->x12 = x12;
    packet->ref_pressure = ref_pressure;
    packet->ref_temperature = ref_temperature;
    packet->ref_latitude = ref_latitude;
    packet->ref_longitude = ref_longitude;
    packet->ref_accel_x = ref_accel_x;
    packet->ref_accel_y = ref_accel_y;
    packet->ref_accel_z = ref_accel_z;
    packet->ref_mag_x = ref_mag_x;
    packet->ref_mag_y = ref_mag_y;
    packet->ref_mag_z = ref_mag_z;
    packet->triad_roll = triad_roll;
    packet->triad_pitch = triad_pitch;
    packet->triad_yaw = triad_yaw;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)packet, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE NAS_TM UNPACKING


/**
 * @brief Get field timestamp from nas_tm message
 *
 * @return [ms] When was this logged
 */
static inline uint64_t mavlink_msg_nas_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from nas_tm message
 *
 * @return  NAS current state
 */
static inline uint8_t mavlink_msg_nas_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  124);
}

/**
 * @brief Get field x0 from nas_tm message
 *
 * @return [m] Kalman state variable 0 (position x)
 */
static inline float mavlink_msg_nas_tm_get_x0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x1 from nas_tm message
 *
 * @return [m] Kalman state variable 1 (position y)
 */
static inline float mavlink_msg_nas_tm_get_x1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field x2 from nas_tm message
 *
 * @return [m] >Kalman state variable 2 (position z)
 */
static inline float mavlink_msg_nas_tm_get_x2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field x3 from nas_tm message
 *
 * @return [m/s] >Kalman state variable 3 (velocity x)
 */
static inline float mavlink_msg_nas_tm_get_x3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field x4 from nas_tm message
 *
 * @return [m/s] >Kalman state variable 4 (velocity y)
 */
static inline float mavlink_msg_nas_tm_get_x4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field x5 from nas_tm message
 *
 * @return [m/s] >Kalman state variable 5 (velocity z)
 */
static inline float mavlink_msg_nas_tm_get_x5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field x6 from nas_tm message
 *
 * @return  Kalman state variable 6 (q0)
 */
static inline float mavlink_msg_nas_tm_get_x6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field x7 from nas_tm message
 *
 * @return  Kalman state variable 7 (q1)
 */
static inline float mavlink_msg_nas_tm_get_x7(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field x8 from nas_tm message
 *
 * @return  Kalman state variable 8 (q2)
 */
static inline float mavlink_msg_nas_tm_get_x8(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field x9 from nas_tm message
 *
 * @return  Kalman state variable 9 (q3)
 */
static inline float mavlink_msg_nas_tm_get_x9(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field x10 from nas_tm message
 *
 * @return  Kalman state variable 10 (bias)
 */
static inline float mavlink_msg_nas_tm_get_x10(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field x11 from nas_tm message
 *
 * @return  Kalman state variable 11 (bias)
 */
static inline float mavlink_msg_nas_tm_get_x11(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field x12 from nas_tm message
 *
 * @return  Kalman state variable 12 (bias)
 */
static inline float mavlink_msg_nas_tm_get_x12(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field ref_pressure from nas_tm message
 *
 * @return [Pa] Calibration pressure
 */
static inline float mavlink_msg_nas_tm_get_ref_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field ref_temperature from nas_tm message
 *
 * @return [degC] Calibration temperature
 */
static inline float mavlink_msg_nas_tm_get_ref_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field ref_latitude from nas_tm message
 *
 * @return [deg] Calibration latitude
 */
static inline float mavlink_msg_nas_tm_get_ref_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field ref_longitude from nas_tm message
 *
 * @return [deg] Calibration longitude
 */
static inline float mavlink_msg_nas_tm_get_ref_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field ref_accel_x from nas_tm message
 *
 * @return [m/s2] Calibration acceleration on x axis
 */
static inline float mavlink_msg_nas_tm_get_ref_accel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field ref_accel_y from nas_tm message
 *
 * @return [m/s2] Calibration acceleration on y axis
 */
static inline float mavlink_msg_nas_tm_get_ref_accel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field ref_accel_z from nas_tm message
 *
 * @return [m/s2] Calibration acceleration on z axis
 */
static inline float mavlink_msg_nas_tm_get_ref_accel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field ref_mag_x from nas_tm message
 *
 * @return [uT] Calibration compass on x axis
 */
static inline float mavlink_msg_nas_tm_get_ref_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field ref_mag_y from nas_tm message
 *
 * @return [uT] Calibration compass on y axis
 */
static inline float mavlink_msg_nas_tm_get_ref_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field ref_mag_z from nas_tm message
 *
 * @return [uT] Calibration compass on z axis
 */
static inline float mavlink_msg_nas_tm_get_ref_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field triad_roll from nas_tm message
 *
 * @return  Orientation on roll estimated by TRIAD initialization
 */
static inline float mavlink_msg_nas_tm_get_triad_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field triad_pitch from nas_tm message
 *
 * @return  Orientation on pitch estimated by TRIAD initialization
 */
static inline float mavlink_msg_nas_tm_get_triad_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field triad_yaw from nas_tm message
 *
 * @return  Orientation on yaw estimated by TRIAD initialization
 */
static inline float mavlink_msg_nas_tm_get_triad_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field roll from nas_tm message
 *
 * @return [deg] Orientation on roll
 */
static inline float mavlink_msg_nas_tm_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field pitch from nas_tm message
 *
 * @return [deg] Orientation on pitch
 */
static inline float mavlink_msg_nas_tm_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field yaw from nas_tm message
 *
 * @return [deg] Orientation on yaw
 */
static inline float mavlink_msg_nas_tm_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Decode a nas_tm message into a struct
 *
 * @param msg The message to decode
 * @param nas_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_nas_tm_decode(const mavlink_message_t* msg, mavlink_nas_tm_t* nas_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nas_tm->timestamp = mavlink_msg_nas_tm_get_timestamp(msg);
    nas_tm->x0 = mavlink_msg_nas_tm_get_x0(msg);
    nas_tm->x1 = mavlink_msg_nas_tm_get_x1(msg);
    nas_tm->x2 = mavlink_msg_nas_tm_get_x2(msg);
    nas_tm->x3 = mavlink_msg_nas_tm_get_x3(msg);
    nas_tm->x4 = mavlink_msg_nas_tm_get_x4(msg);
    nas_tm->x5 = mavlink_msg_nas_tm_get_x5(msg);
    nas_tm->x6 = mavlink_msg_nas_tm_get_x6(msg);
    nas_tm->x7 = mavlink_msg_nas_tm_get_x7(msg);
    nas_tm->x8 = mavlink_msg_nas_tm_get_x8(msg);
    nas_tm->x9 = mavlink_msg_nas_tm_get_x9(msg);
    nas_tm->x10 = mavlink_msg_nas_tm_get_x10(msg);
    nas_tm->x11 = mavlink_msg_nas_tm_get_x11(msg);
    nas_tm->x12 = mavlink_msg_nas_tm_get_x12(msg);
    nas_tm->ref_pressure = mavlink_msg_nas_tm_get_ref_pressure(msg);
    nas_tm->ref_temperature = mavlink_msg_nas_tm_get_ref_temperature(msg);
    nas_tm->ref_latitude = mavlink_msg_nas_tm_get_ref_latitude(msg);
    nas_tm->ref_longitude = mavlink_msg_nas_tm_get_ref_longitude(msg);
    nas_tm->ref_accel_x = mavlink_msg_nas_tm_get_ref_accel_x(msg);
    nas_tm->ref_accel_y = mavlink_msg_nas_tm_get_ref_accel_y(msg);
    nas_tm->ref_accel_z = mavlink_msg_nas_tm_get_ref_accel_z(msg);
    nas_tm->ref_mag_x = mavlink_msg_nas_tm_get_ref_mag_x(msg);
    nas_tm->ref_mag_y = mavlink_msg_nas_tm_get_ref_mag_y(msg);
    nas_tm->ref_mag_z = mavlink_msg_nas_tm_get_ref_mag_z(msg);
    nas_tm->triad_roll = mavlink_msg_nas_tm_get_triad_roll(msg);
    nas_tm->triad_pitch = mavlink_msg_nas_tm_get_triad_pitch(msg);
    nas_tm->triad_yaw = mavlink_msg_nas_tm_get_triad_yaw(msg);
    nas_tm->roll = mavlink_msg_nas_tm_get_roll(msg);
    nas_tm->pitch = mavlink_msg_nas_tm_get_pitch(msg);
    nas_tm->yaw = mavlink_msg_nas_tm_get_yaw(msg);
    nas_tm->state = mavlink_msg_nas_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAS_TM_LEN? msg->len : MAVLINK_MSG_ID_NAS_TM_LEN;
        memset(nas_tm, 0, MAVLINK_MSG_ID_NAS_TM_LEN);
    memcpy(nas_tm, _MAV_PAYLOAD(msg), len);
#endif
}
