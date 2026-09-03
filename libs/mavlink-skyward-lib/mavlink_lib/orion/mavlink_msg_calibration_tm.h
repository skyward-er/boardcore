#pragma once
// MESSAGE CALIBRATION_TM PACKING

#define MAVLINK_MSG_ID_CALIBRATION_TM 214


typedef struct __mavlink_calibration_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float acc0_bias_x; /*< [m/s^2] Accelerometer 0 X bias*/
 float acc0_bias_y; /*< [m/s^2] Accelerometer 0 Y bias*/
 float acc0_bias_z; /*< [m/s^2] Accelerometer 0 Z bias*/
 float gyro0_bias_x; /*< [rad/s] Gyroscope 0 X bias*/
 float gyro0_bias_y; /*< [rad/s] Gyroscope 0 Y bias*/
 float gyro0_bias_z; /*< [rad/s] Gyroscope 0 Z bias*/
 float acc1_bias_x; /*< [m/s^2] Accelerometer 1 X bias*/
 float acc1_bias_y; /*< [m/s^2] Accelerometer 1 Y bias*/
 float acc1_bias_z; /*< [m/s^2] Accelerometer 1 Z bias*/
 float gyro1_bias_x; /*< [rad/s] Gyroscope 1 X bias*/
 float gyro1_bias_y; /*< [rad/s] Gyroscope 1 Y bias*/
 float gyro1_bias_z; /*< [rad/s] Gyroscope 1 Z bias*/
 float mag_bias_x; /*< [uT] Magnetometer X bias*/
 float mag_bias_y; /*< [uT] Magnetometer Y bias*/
 float mag_bias_z; /*< [uT] Magnetometer Z bias*/
 float mag_scale_x; /*<  Magnetometer X scale*/
 float mag_scale_y; /*<  Magnetometer Y scale*/
 float mag_scale_z; /*<  Magnetometer Z scale*/
 float pitot_dynamic_bias; /*< [Pa] Pitot dynamic pressure bias*/
} mavlink_calibration_tm_t;

#define MAVLINK_MSG_ID_CALIBRATION_TM_LEN 84
#define MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN 84
#define MAVLINK_MSG_ID_214_LEN 84
#define MAVLINK_MSG_ID_214_MIN_LEN 84

#define MAVLINK_MSG_ID_CALIBRATION_TM_CRC 116
#define MAVLINK_MSG_ID_214_CRC 116



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CALIBRATION_TM { \
    214, \
    "CALIBRATION_TM", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_calibration_tm_t, timestamp) }, \
         { "acc0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_calibration_tm_t, acc0_bias_x) }, \
         { "acc0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_calibration_tm_t, acc0_bias_y) }, \
         { "acc0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_calibration_tm_t, acc0_bias_z) }, \
         { "gyro0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_calibration_tm_t, gyro0_bias_x) }, \
         { "gyro0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_calibration_tm_t, gyro0_bias_y) }, \
         { "gyro0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_calibration_tm_t, gyro0_bias_z) }, \
         { "acc1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_calibration_tm_t, acc1_bias_x) }, \
         { "acc1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_calibration_tm_t, acc1_bias_y) }, \
         { "acc1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_calibration_tm_t, acc1_bias_z) }, \
         { "gyro1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_calibration_tm_t, gyro1_bias_x) }, \
         { "gyro1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_calibration_tm_t, gyro1_bias_y) }, \
         { "gyro1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_calibration_tm_t, gyro1_bias_z) }, \
         { "mag_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_calibration_tm_t, mag_bias_x) }, \
         { "mag_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_calibration_tm_t, mag_bias_y) }, \
         { "mag_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_calibration_tm_t, mag_bias_z) }, \
         { "mag_scale_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_calibration_tm_t, mag_scale_x) }, \
         { "mag_scale_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_calibration_tm_t, mag_scale_y) }, \
         { "mag_scale_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_calibration_tm_t, mag_scale_z) }, \
         { "pitot_dynamic_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_calibration_tm_t, pitot_dynamic_bias) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CALIBRATION_TM { \
    "CALIBRATION_TM", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_calibration_tm_t, timestamp) }, \
         { "acc0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_calibration_tm_t, acc0_bias_x) }, \
         { "acc0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_calibration_tm_t, acc0_bias_y) }, \
         { "acc0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_calibration_tm_t, acc0_bias_z) }, \
         { "gyro0_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_calibration_tm_t, gyro0_bias_x) }, \
         { "gyro0_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_calibration_tm_t, gyro0_bias_y) }, \
         { "gyro0_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_calibration_tm_t, gyro0_bias_z) }, \
         { "acc1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_calibration_tm_t, acc1_bias_x) }, \
         { "acc1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_calibration_tm_t, acc1_bias_y) }, \
         { "acc1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_calibration_tm_t, acc1_bias_z) }, \
         { "gyro1_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_calibration_tm_t, gyro1_bias_x) }, \
         { "gyro1_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_calibration_tm_t, gyro1_bias_y) }, \
         { "gyro1_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_calibration_tm_t, gyro1_bias_z) }, \
         { "mag_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_calibration_tm_t, mag_bias_x) }, \
         { "mag_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_calibration_tm_t, mag_bias_y) }, \
         { "mag_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_calibration_tm_t, mag_bias_z) }, \
         { "mag_scale_x", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_calibration_tm_t, mag_scale_x) }, \
         { "mag_scale_y", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_calibration_tm_t, mag_scale_y) }, \
         { "mag_scale_z", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_calibration_tm_t, mag_scale_z) }, \
         { "pitot_dynamic_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_calibration_tm_t, pitot_dynamic_bias) }, \
         } \
}
#endif

/**
 * @brief Pack a calibration_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param pitot_dynamic_bias [Pa] Pitot dynamic pressure bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_calibration_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float pitot_dynamic_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);
    _mav_put_float(buf, 56, mag_bias_x);
    _mav_put_float(buf, 60, mag_bias_y);
    _mav_put_float(buf, 64, mag_bias_z);
    _mav_put_float(buf, 68, mag_scale_x);
    _mav_put_float(buf, 72, mag_scale_y);
    _mav_put_float(buf, 76, mag_scale_z);
    _mav_put_float(buf, 80, pitot_dynamic_bias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.pitot_dynamic_bias = pitot_dynamic_bias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CALIBRATION_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
}

/**
 * @brief Pack a calibration_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param pitot_dynamic_bias [Pa] Pitot dynamic pressure bias
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_calibration_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float acc0_bias_x,float acc0_bias_y,float acc0_bias_z,float gyro0_bias_x,float gyro0_bias_y,float gyro0_bias_z,float acc1_bias_x,float acc1_bias_y,float acc1_bias_z,float gyro1_bias_x,float gyro1_bias_y,float gyro1_bias_z,float mag_bias_x,float mag_bias_y,float mag_bias_z,float mag_scale_x,float mag_scale_y,float mag_scale_z,float pitot_dynamic_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);
    _mav_put_float(buf, 56, mag_bias_x);
    _mav_put_float(buf, 60, mag_bias_y);
    _mav_put_float(buf, 64, mag_bias_z);
    _mav_put_float(buf, 68, mag_scale_x);
    _mav_put_float(buf, 72, mag_scale_y);
    _mav_put_float(buf, 76, mag_scale_z);
    _mav_put_float(buf, 80, pitot_dynamic_bias);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.pitot_dynamic_bias = pitot_dynamic_bias;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CALIBRATION_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
}

/**
 * @brief Encode a calibration_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param calibration_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_calibration_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_calibration_tm_t* calibration_tm)
{
    return mavlink_msg_calibration_tm_pack(system_id, component_id, msg, calibration_tm->timestamp, calibration_tm->acc0_bias_x, calibration_tm->acc0_bias_y, calibration_tm->acc0_bias_z, calibration_tm->gyro0_bias_x, calibration_tm->gyro0_bias_y, calibration_tm->gyro0_bias_z, calibration_tm->acc1_bias_x, calibration_tm->acc1_bias_y, calibration_tm->acc1_bias_z, calibration_tm->gyro1_bias_x, calibration_tm->gyro1_bias_y, calibration_tm->gyro1_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->pitot_dynamic_bias);
}

/**
 * @brief Encode a calibration_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param calibration_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_calibration_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_calibration_tm_t* calibration_tm)
{
    return mavlink_msg_calibration_tm_pack_chan(system_id, component_id, chan, msg, calibration_tm->timestamp, calibration_tm->acc0_bias_x, calibration_tm->acc0_bias_y, calibration_tm->acc0_bias_z, calibration_tm->gyro0_bias_x, calibration_tm->gyro0_bias_y, calibration_tm->gyro0_bias_z, calibration_tm->acc1_bias_x, calibration_tm->acc1_bias_y, calibration_tm->acc1_bias_z, calibration_tm->gyro1_bias_x, calibration_tm->gyro1_bias_y, calibration_tm->gyro1_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->pitot_dynamic_bias);
}

/**
 * @brief Send a calibration_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param acc0_bias_x [m/s^2] Accelerometer 0 X bias
 * @param acc0_bias_y [m/s^2] Accelerometer 0 Y bias
 * @param acc0_bias_z [m/s^2] Accelerometer 0 Z bias
 * @param gyro0_bias_x [rad/s] Gyroscope 0 X bias
 * @param gyro0_bias_y [rad/s] Gyroscope 0 Y bias
 * @param gyro0_bias_z [rad/s] Gyroscope 0 Z bias
 * @param acc1_bias_x [m/s^2] Accelerometer 1 X bias
 * @param acc1_bias_y [m/s^2] Accelerometer 1 Y bias
 * @param acc1_bias_z [m/s^2] Accelerometer 1 Z bias
 * @param gyro1_bias_x [rad/s] Gyroscope 1 X bias
 * @param gyro1_bias_y [rad/s] Gyroscope 1 Y bias
 * @param gyro1_bias_z [rad/s] Gyroscope 1 Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param pitot_dynamic_bias [Pa] Pitot dynamic pressure bias
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_calibration_tm_send(mavlink_channel_t chan, uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float pitot_dynamic_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);
    _mav_put_float(buf, 56, mag_bias_x);
    _mav_put_float(buf, 60, mag_bias_y);
    _mav_put_float(buf, 64, mag_bias_z);
    _mav_put_float(buf, 68, mag_scale_x);
    _mav_put_float(buf, 72, mag_scale_y);
    _mav_put_float(buf, 76, mag_scale_z);
    _mav_put_float(buf, 80, pitot_dynamic_bias);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, buf, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.acc0_bias_x = acc0_bias_x;
    packet.acc0_bias_y = acc0_bias_y;
    packet.acc0_bias_z = acc0_bias_z;
    packet.gyro0_bias_x = gyro0_bias_x;
    packet.gyro0_bias_y = gyro0_bias_y;
    packet.gyro0_bias_z = gyro0_bias_z;
    packet.acc1_bias_x = acc1_bias_x;
    packet.acc1_bias_y = acc1_bias_y;
    packet.acc1_bias_z = acc1_bias_z;
    packet.gyro1_bias_x = gyro1_bias_x;
    packet.gyro1_bias_y = gyro1_bias_y;
    packet.gyro1_bias_z = gyro1_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.pitot_dynamic_bias = pitot_dynamic_bias;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, (const char *)&packet, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#endif
}

/**
 * @brief Send a calibration_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_calibration_tm_send_struct(mavlink_channel_t chan, const mavlink_calibration_tm_t* calibration_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_calibration_tm_send(chan, calibration_tm->timestamp, calibration_tm->acc0_bias_x, calibration_tm->acc0_bias_y, calibration_tm->acc0_bias_z, calibration_tm->gyro0_bias_x, calibration_tm->gyro0_bias_y, calibration_tm->gyro0_bias_z, calibration_tm->acc1_bias_x, calibration_tm->acc1_bias_y, calibration_tm->acc1_bias_z, calibration_tm->gyro1_bias_x, calibration_tm->gyro1_bias_y, calibration_tm->gyro1_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->pitot_dynamic_bias);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, (const char *)calibration_tm, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_CALIBRATION_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_calibration_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float acc0_bias_x, float acc0_bias_y, float acc0_bias_z, float gyro0_bias_x, float gyro0_bias_y, float gyro0_bias_z, float acc1_bias_x, float acc1_bias_y, float acc1_bias_z, float gyro1_bias_x, float gyro1_bias_y, float gyro1_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float pitot_dynamic_bias)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, acc0_bias_x);
    _mav_put_float(buf, 12, acc0_bias_y);
    _mav_put_float(buf, 16, acc0_bias_z);
    _mav_put_float(buf, 20, gyro0_bias_x);
    _mav_put_float(buf, 24, gyro0_bias_y);
    _mav_put_float(buf, 28, gyro0_bias_z);
    _mav_put_float(buf, 32, acc1_bias_x);
    _mav_put_float(buf, 36, acc1_bias_y);
    _mav_put_float(buf, 40, acc1_bias_z);
    _mav_put_float(buf, 44, gyro1_bias_x);
    _mav_put_float(buf, 48, gyro1_bias_y);
    _mav_put_float(buf, 52, gyro1_bias_z);
    _mav_put_float(buf, 56, mag_bias_x);
    _mav_put_float(buf, 60, mag_bias_y);
    _mav_put_float(buf, 64, mag_bias_z);
    _mav_put_float(buf, 68, mag_scale_x);
    _mav_put_float(buf, 72, mag_scale_y);
    _mav_put_float(buf, 76, mag_scale_z);
    _mav_put_float(buf, 80, pitot_dynamic_bias);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, buf, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#else
    mavlink_calibration_tm_t *packet = (mavlink_calibration_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->acc0_bias_x = acc0_bias_x;
    packet->acc0_bias_y = acc0_bias_y;
    packet->acc0_bias_z = acc0_bias_z;
    packet->gyro0_bias_x = gyro0_bias_x;
    packet->gyro0_bias_y = gyro0_bias_y;
    packet->gyro0_bias_z = gyro0_bias_z;
    packet->acc1_bias_x = acc1_bias_x;
    packet->acc1_bias_y = acc1_bias_y;
    packet->acc1_bias_z = acc1_bias_z;
    packet->gyro1_bias_x = gyro1_bias_x;
    packet->gyro1_bias_y = gyro1_bias_y;
    packet->gyro1_bias_z = gyro1_bias_z;
    packet->mag_bias_x = mag_bias_x;
    packet->mag_bias_y = mag_bias_y;
    packet->mag_bias_z = mag_bias_z;
    packet->mag_scale_x = mag_scale_x;
    packet->mag_scale_y = mag_scale_y;
    packet->mag_scale_z = mag_scale_z;
    packet->pitot_dynamic_bias = pitot_dynamic_bias;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, (const char *)packet, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE CALIBRATION_TM UNPACKING


/**
 * @brief Get field timestamp from calibration_tm message
 *
 * @return [us] Timestamp in microseconds
 */
static inline uint64_t mavlink_msg_calibration_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field acc0_bias_x from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 0 X bias
 */
static inline float mavlink_msg_calibration_tm_get_acc0_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acc0_bias_y from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 0 Y bias
 */
static inline float mavlink_msg_calibration_tm_get_acc0_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field acc0_bias_z from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 0 Z bias
 */
static inline float mavlink_msg_calibration_tm_get_acc0_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro0_bias_x from calibration_tm message
 *
 * @return [rad/s] Gyroscope 0 X bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro0_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro0_bias_y from calibration_tm message
 *
 * @return [rad/s] Gyroscope 0 Y bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro0_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gyro0_bias_z from calibration_tm message
 *
 * @return [rad/s] Gyroscope 0 Z bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro0_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field acc1_bias_x from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 1 X bias
 */
static inline float mavlink_msg_calibration_tm_get_acc1_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field acc1_bias_y from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 1 Y bias
 */
static inline float mavlink_msg_calibration_tm_get_acc1_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field acc1_bias_z from calibration_tm message
 *
 * @return [m/s^2] Accelerometer 1 Z bias
 */
static inline float mavlink_msg_calibration_tm_get_acc1_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field gyro1_bias_x from calibration_tm message
 *
 * @return [rad/s] Gyroscope 1 X bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro1_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field gyro1_bias_y from calibration_tm message
 *
 * @return [rad/s] Gyroscope 1 Y bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro1_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gyro1_bias_z from calibration_tm message
 *
 * @return [rad/s] Gyroscope 1 Z bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro1_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field mag_bias_x from calibration_tm message
 *
 * @return [uT] Magnetometer X bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field mag_bias_y from calibration_tm message
 *
 * @return [uT] Magnetometer Y bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field mag_bias_z from calibration_tm message
 *
 * @return [uT] Magnetometer Z bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field mag_scale_x from calibration_tm message
 *
 * @return  Magnetometer X scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field mag_scale_y from calibration_tm message
 *
 * @return  Magnetometer Y scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field mag_scale_z from calibration_tm message
 *
 * @return  Magnetometer Z scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field pitot_dynamic_bias from calibration_tm message
 *
 * @return [Pa] Pitot dynamic pressure bias
 */
static inline float mavlink_msg_calibration_tm_get_pitot_dynamic_bias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Decode a calibration_tm message into a struct
 *
 * @param msg The message to decode
 * @param calibration_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_calibration_tm_decode(const mavlink_message_t* msg, mavlink_calibration_tm_t* calibration_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    calibration_tm->timestamp = mavlink_msg_calibration_tm_get_timestamp(msg);
    calibration_tm->acc0_bias_x = mavlink_msg_calibration_tm_get_acc0_bias_x(msg);
    calibration_tm->acc0_bias_y = mavlink_msg_calibration_tm_get_acc0_bias_y(msg);
    calibration_tm->acc0_bias_z = mavlink_msg_calibration_tm_get_acc0_bias_z(msg);
    calibration_tm->gyro0_bias_x = mavlink_msg_calibration_tm_get_gyro0_bias_x(msg);
    calibration_tm->gyro0_bias_y = mavlink_msg_calibration_tm_get_gyro0_bias_y(msg);
    calibration_tm->gyro0_bias_z = mavlink_msg_calibration_tm_get_gyro0_bias_z(msg);
    calibration_tm->acc1_bias_x = mavlink_msg_calibration_tm_get_acc1_bias_x(msg);
    calibration_tm->acc1_bias_y = mavlink_msg_calibration_tm_get_acc1_bias_y(msg);
    calibration_tm->acc1_bias_z = mavlink_msg_calibration_tm_get_acc1_bias_z(msg);
    calibration_tm->gyro1_bias_x = mavlink_msg_calibration_tm_get_gyro1_bias_x(msg);
    calibration_tm->gyro1_bias_y = mavlink_msg_calibration_tm_get_gyro1_bias_y(msg);
    calibration_tm->gyro1_bias_z = mavlink_msg_calibration_tm_get_gyro1_bias_z(msg);
    calibration_tm->mag_bias_x = mavlink_msg_calibration_tm_get_mag_bias_x(msg);
    calibration_tm->mag_bias_y = mavlink_msg_calibration_tm_get_mag_bias_y(msg);
    calibration_tm->mag_bias_z = mavlink_msg_calibration_tm_get_mag_bias_z(msg);
    calibration_tm->mag_scale_x = mavlink_msg_calibration_tm_get_mag_scale_x(msg);
    calibration_tm->mag_scale_y = mavlink_msg_calibration_tm_get_mag_scale_y(msg);
    calibration_tm->mag_scale_z = mavlink_msg_calibration_tm_get_mag_scale_z(msg);
    calibration_tm->pitot_dynamic_bias = mavlink_msg_calibration_tm_get_pitot_dynamic_bias(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CALIBRATION_TM_LEN? msg->len : MAVLINK_MSG_ID_CALIBRATION_TM_LEN;
        memset(calibration_tm, 0, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
    memcpy(calibration_tm, _MAV_PAYLOAD(msg), len);
#endif
}
