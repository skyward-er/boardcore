#pragma once
// MESSAGE CALIBRATION_TM PACKING

#define MAVLINK_MSG_ID_CALIBRATION_TM 214


typedef struct __mavlink_calibration_tm_t {
 uint64_t timestamp; /*< [us] Timestamp in microseconds*/
 float gyro_bias_x; /*< [deg/s] Gyroscope X bias*/
 float gyro_bias_y; /*< [deg/s] Gyroscope Y bias*/
 float gyro_bias_z; /*< [deg/s] Gyroscope Z bias*/
 float mag_bias_x; /*< [uT] Magnetometer X bias*/
 float mag_bias_y; /*< [uT] Magnetometer Y bias*/
 float mag_bias_z; /*< [uT] Magnetometer Z bias*/
 float mag_scale_x; /*<  Magnetometer X scale*/
 float mag_scale_y; /*<  Magnetometer Y scale*/
 float mag_scale_z; /*<  Magnetometer Z scale*/
 float static_press_1_bias; /*< [Pa] Static pressure 1 bias*/
 float static_press_1_scale; /*<  Static pressure 1 scale*/
 float static_press_2_bias; /*< [Pa] Static pressure 2 bias*/
 float static_press_2_scale; /*<  Static pressure 2 scale*/
 float dpl_bay_press_bias; /*< [Pa] Deployment bay pressure bias*/
 float dpl_bay_press_scale; /*<  Deployment bay pressure scale*/
 float dynamic_press_bias; /*< [Pa] Dynamic pressure bias*/
 float dynamic_press_scale; /*<  Dynamic pressure scale*/
} mavlink_calibration_tm_t;

#define MAVLINK_MSG_ID_CALIBRATION_TM_LEN 76
#define MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN 76
#define MAVLINK_MSG_ID_214_LEN 76
#define MAVLINK_MSG_ID_214_MIN_LEN 76

#define MAVLINK_MSG_ID_CALIBRATION_TM_CRC 210
#define MAVLINK_MSG_ID_214_CRC 210



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CALIBRATION_TM { \
    214, \
    "CALIBRATION_TM", \
    18, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_calibration_tm_t, timestamp) }, \
         { "gyro_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_calibration_tm_t, gyro_bias_x) }, \
         { "gyro_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_calibration_tm_t, gyro_bias_y) }, \
         { "gyro_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_calibration_tm_t, gyro_bias_z) }, \
         { "mag_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_calibration_tm_t, mag_bias_x) }, \
         { "mag_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_calibration_tm_t, mag_bias_y) }, \
         { "mag_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_calibration_tm_t, mag_bias_z) }, \
         { "mag_scale_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_calibration_tm_t, mag_scale_x) }, \
         { "mag_scale_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_calibration_tm_t, mag_scale_y) }, \
         { "mag_scale_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_calibration_tm_t, mag_scale_z) }, \
         { "static_press_1_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_calibration_tm_t, static_press_1_bias) }, \
         { "static_press_1_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_calibration_tm_t, static_press_1_scale) }, \
         { "static_press_2_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_calibration_tm_t, static_press_2_bias) }, \
         { "static_press_2_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_calibration_tm_t, static_press_2_scale) }, \
         { "dpl_bay_press_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_calibration_tm_t, dpl_bay_press_bias) }, \
         { "dpl_bay_press_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_calibration_tm_t, dpl_bay_press_scale) }, \
         { "dynamic_press_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_calibration_tm_t, dynamic_press_bias) }, \
         { "dynamic_press_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_calibration_tm_t, dynamic_press_scale) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CALIBRATION_TM { \
    "CALIBRATION_TM", \
    18, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_calibration_tm_t, timestamp) }, \
         { "gyro_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_calibration_tm_t, gyro_bias_x) }, \
         { "gyro_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_calibration_tm_t, gyro_bias_y) }, \
         { "gyro_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_calibration_tm_t, gyro_bias_z) }, \
         { "mag_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_calibration_tm_t, mag_bias_x) }, \
         { "mag_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_calibration_tm_t, mag_bias_y) }, \
         { "mag_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_calibration_tm_t, mag_bias_z) }, \
         { "mag_scale_x", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_calibration_tm_t, mag_scale_x) }, \
         { "mag_scale_y", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_calibration_tm_t, mag_scale_y) }, \
         { "mag_scale_z", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_calibration_tm_t, mag_scale_z) }, \
         { "static_press_1_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_calibration_tm_t, static_press_1_bias) }, \
         { "static_press_1_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_calibration_tm_t, static_press_1_scale) }, \
         { "static_press_2_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_calibration_tm_t, static_press_2_bias) }, \
         { "static_press_2_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_calibration_tm_t, static_press_2_scale) }, \
         { "dpl_bay_press_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_calibration_tm_t, dpl_bay_press_bias) }, \
         { "dpl_bay_press_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_calibration_tm_t, dpl_bay_press_scale) }, \
         { "dynamic_press_bias", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_calibration_tm_t, dynamic_press_bias) }, \
         { "dynamic_press_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_calibration_tm_t, dynamic_press_scale) }, \
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
 * @param gyro_bias_x [deg/s] Gyroscope X bias
 * @param gyro_bias_y [deg/s] Gyroscope Y bias
 * @param gyro_bias_z [deg/s] Gyroscope Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param static_press_1_bias [Pa] Static pressure 1 bias
 * @param static_press_1_scale  Static pressure 1 scale
 * @param static_press_2_bias [Pa] Static pressure 2 bias
 * @param static_press_2_scale  Static pressure 2 scale
 * @param dpl_bay_press_bias [Pa] Deployment bay pressure bias
 * @param dpl_bay_press_scale  Deployment bay pressure scale
 * @param dynamic_press_bias [Pa] Dynamic pressure bias
 * @param dynamic_press_scale  Dynamic pressure scale
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_calibration_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float static_press_1_bias, float static_press_1_scale, float static_press_2_bias, float static_press_2_scale, float dpl_bay_press_bias, float dpl_bay_press_scale, float dynamic_press_bias, float dynamic_press_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_bias_x);
    _mav_put_float(buf, 12, gyro_bias_y);
    _mav_put_float(buf, 16, gyro_bias_z);
    _mav_put_float(buf, 20, mag_bias_x);
    _mav_put_float(buf, 24, mag_bias_y);
    _mav_put_float(buf, 28, mag_bias_z);
    _mav_put_float(buf, 32, mag_scale_x);
    _mav_put_float(buf, 36, mag_scale_y);
    _mav_put_float(buf, 40, mag_scale_z);
    _mav_put_float(buf, 44, static_press_1_bias);
    _mav_put_float(buf, 48, static_press_1_scale);
    _mav_put_float(buf, 52, static_press_2_bias);
    _mav_put_float(buf, 56, static_press_2_scale);
    _mav_put_float(buf, 60, dpl_bay_press_bias);
    _mav_put_float(buf, 64, dpl_bay_press_scale);
    _mav_put_float(buf, 68, dynamic_press_bias);
    _mav_put_float(buf, 72, dynamic_press_scale);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.gyro_bias_x = gyro_bias_x;
    packet.gyro_bias_y = gyro_bias_y;
    packet.gyro_bias_z = gyro_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.static_press_1_bias = static_press_1_bias;
    packet.static_press_1_scale = static_press_1_scale;
    packet.static_press_2_bias = static_press_2_bias;
    packet.static_press_2_scale = static_press_2_scale;
    packet.dpl_bay_press_bias = dpl_bay_press_bias;
    packet.dpl_bay_press_scale = dpl_bay_press_scale;
    packet.dynamic_press_bias = dynamic_press_bias;
    packet.dynamic_press_scale = dynamic_press_scale;

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
 * @param gyro_bias_x [deg/s] Gyroscope X bias
 * @param gyro_bias_y [deg/s] Gyroscope Y bias
 * @param gyro_bias_z [deg/s] Gyroscope Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param static_press_1_bias [Pa] Static pressure 1 bias
 * @param static_press_1_scale  Static pressure 1 scale
 * @param static_press_2_bias [Pa] Static pressure 2 bias
 * @param static_press_2_scale  Static pressure 2 scale
 * @param dpl_bay_press_bias [Pa] Deployment bay pressure bias
 * @param dpl_bay_press_scale  Deployment bay pressure scale
 * @param dynamic_press_bias [Pa] Dynamic pressure bias
 * @param dynamic_press_scale  Dynamic pressure scale
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_calibration_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float gyro_bias_x,float gyro_bias_y,float gyro_bias_z,float mag_bias_x,float mag_bias_y,float mag_bias_z,float mag_scale_x,float mag_scale_y,float mag_scale_z,float static_press_1_bias,float static_press_1_scale,float static_press_2_bias,float static_press_2_scale,float dpl_bay_press_bias,float dpl_bay_press_scale,float dynamic_press_bias,float dynamic_press_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_bias_x);
    _mav_put_float(buf, 12, gyro_bias_y);
    _mav_put_float(buf, 16, gyro_bias_z);
    _mav_put_float(buf, 20, mag_bias_x);
    _mav_put_float(buf, 24, mag_bias_y);
    _mav_put_float(buf, 28, mag_bias_z);
    _mav_put_float(buf, 32, mag_scale_x);
    _mav_put_float(buf, 36, mag_scale_y);
    _mav_put_float(buf, 40, mag_scale_z);
    _mav_put_float(buf, 44, static_press_1_bias);
    _mav_put_float(buf, 48, static_press_1_scale);
    _mav_put_float(buf, 52, static_press_2_bias);
    _mav_put_float(buf, 56, static_press_2_scale);
    _mav_put_float(buf, 60, dpl_bay_press_bias);
    _mav_put_float(buf, 64, dpl_bay_press_scale);
    _mav_put_float(buf, 68, dynamic_press_bias);
    _mav_put_float(buf, 72, dynamic_press_scale);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.gyro_bias_x = gyro_bias_x;
    packet.gyro_bias_y = gyro_bias_y;
    packet.gyro_bias_z = gyro_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.static_press_1_bias = static_press_1_bias;
    packet.static_press_1_scale = static_press_1_scale;
    packet.static_press_2_bias = static_press_2_bias;
    packet.static_press_2_scale = static_press_2_scale;
    packet.dpl_bay_press_bias = dpl_bay_press_bias;
    packet.dpl_bay_press_scale = dpl_bay_press_scale;
    packet.dynamic_press_bias = dynamic_press_bias;
    packet.dynamic_press_scale = dynamic_press_scale;

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
    return mavlink_msg_calibration_tm_pack(system_id, component_id, msg, calibration_tm->timestamp, calibration_tm->gyro_bias_x, calibration_tm->gyro_bias_y, calibration_tm->gyro_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->static_press_1_bias, calibration_tm->static_press_1_scale, calibration_tm->static_press_2_bias, calibration_tm->static_press_2_scale, calibration_tm->dpl_bay_press_bias, calibration_tm->dpl_bay_press_scale, calibration_tm->dynamic_press_bias, calibration_tm->dynamic_press_scale);
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
    return mavlink_msg_calibration_tm_pack_chan(system_id, component_id, chan, msg, calibration_tm->timestamp, calibration_tm->gyro_bias_x, calibration_tm->gyro_bias_y, calibration_tm->gyro_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->static_press_1_bias, calibration_tm->static_press_1_scale, calibration_tm->static_press_2_bias, calibration_tm->static_press_2_scale, calibration_tm->dpl_bay_press_bias, calibration_tm->dpl_bay_press_scale, calibration_tm->dynamic_press_bias, calibration_tm->dynamic_press_scale);
}

/**
 * @brief Send a calibration_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp in microseconds
 * @param gyro_bias_x [deg/s] Gyroscope X bias
 * @param gyro_bias_y [deg/s] Gyroscope Y bias
 * @param gyro_bias_z [deg/s] Gyroscope Z bias
 * @param mag_bias_x [uT] Magnetometer X bias
 * @param mag_bias_y [uT] Magnetometer Y bias
 * @param mag_bias_z [uT] Magnetometer Z bias
 * @param mag_scale_x  Magnetometer X scale
 * @param mag_scale_y  Magnetometer Y scale
 * @param mag_scale_z  Magnetometer Z scale
 * @param static_press_1_bias [Pa] Static pressure 1 bias
 * @param static_press_1_scale  Static pressure 1 scale
 * @param static_press_2_bias [Pa] Static pressure 2 bias
 * @param static_press_2_scale  Static pressure 2 scale
 * @param dpl_bay_press_bias [Pa] Deployment bay pressure bias
 * @param dpl_bay_press_scale  Deployment bay pressure scale
 * @param dynamic_press_bias [Pa] Dynamic pressure bias
 * @param dynamic_press_scale  Dynamic pressure scale
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_calibration_tm_send(mavlink_channel_t chan, uint64_t timestamp, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float static_press_1_bias, float static_press_1_scale, float static_press_2_bias, float static_press_2_scale, float dpl_bay_press_bias, float dpl_bay_press_scale, float dynamic_press_bias, float dynamic_press_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CALIBRATION_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_bias_x);
    _mav_put_float(buf, 12, gyro_bias_y);
    _mav_put_float(buf, 16, gyro_bias_z);
    _mav_put_float(buf, 20, mag_bias_x);
    _mav_put_float(buf, 24, mag_bias_y);
    _mav_put_float(buf, 28, mag_bias_z);
    _mav_put_float(buf, 32, mag_scale_x);
    _mav_put_float(buf, 36, mag_scale_y);
    _mav_put_float(buf, 40, mag_scale_z);
    _mav_put_float(buf, 44, static_press_1_bias);
    _mav_put_float(buf, 48, static_press_1_scale);
    _mav_put_float(buf, 52, static_press_2_bias);
    _mav_put_float(buf, 56, static_press_2_scale);
    _mav_put_float(buf, 60, dpl_bay_press_bias);
    _mav_put_float(buf, 64, dpl_bay_press_scale);
    _mav_put_float(buf, 68, dynamic_press_bias);
    _mav_put_float(buf, 72, dynamic_press_scale);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, buf, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#else
    mavlink_calibration_tm_t packet;
    packet.timestamp = timestamp;
    packet.gyro_bias_x = gyro_bias_x;
    packet.gyro_bias_y = gyro_bias_y;
    packet.gyro_bias_z = gyro_bias_z;
    packet.mag_bias_x = mag_bias_x;
    packet.mag_bias_y = mag_bias_y;
    packet.mag_bias_z = mag_bias_z;
    packet.mag_scale_x = mag_scale_x;
    packet.mag_scale_y = mag_scale_y;
    packet.mag_scale_z = mag_scale_z;
    packet.static_press_1_bias = static_press_1_bias;
    packet.static_press_1_scale = static_press_1_scale;
    packet.static_press_2_bias = static_press_2_bias;
    packet.static_press_2_scale = static_press_2_scale;
    packet.dpl_bay_press_bias = dpl_bay_press_bias;
    packet.dpl_bay_press_scale = dpl_bay_press_scale;
    packet.dynamic_press_bias = dynamic_press_bias;
    packet.dynamic_press_scale = dynamic_press_scale;

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
    mavlink_msg_calibration_tm_send(chan, calibration_tm->timestamp, calibration_tm->gyro_bias_x, calibration_tm->gyro_bias_y, calibration_tm->gyro_bias_z, calibration_tm->mag_bias_x, calibration_tm->mag_bias_y, calibration_tm->mag_bias_z, calibration_tm->mag_scale_x, calibration_tm->mag_scale_y, calibration_tm->mag_scale_z, calibration_tm->static_press_1_bias, calibration_tm->static_press_1_scale, calibration_tm->static_press_2_bias, calibration_tm->static_press_2_scale, calibration_tm->dpl_bay_press_bias, calibration_tm->dpl_bay_press_scale, calibration_tm->dynamic_press_bias, calibration_tm->dynamic_press_scale);
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
static inline void mavlink_msg_calibration_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z, float mag_bias_x, float mag_bias_y, float mag_bias_z, float mag_scale_x, float mag_scale_y, float mag_scale_z, float static_press_1_bias, float static_press_1_scale, float static_press_2_bias, float static_press_2_scale, float dpl_bay_press_bias, float dpl_bay_press_scale, float dynamic_press_bias, float dynamic_press_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, gyro_bias_x);
    _mav_put_float(buf, 12, gyro_bias_y);
    _mav_put_float(buf, 16, gyro_bias_z);
    _mav_put_float(buf, 20, mag_bias_x);
    _mav_put_float(buf, 24, mag_bias_y);
    _mav_put_float(buf, 28, mag_bias_z);
    _mav_put_float(buf, 32, mag_scale_x);
    _mav_put_float(buf, 36, mag_scale_y);
    _mav_put_float(buf, 40, mag_scale_z);
    _mav_put_float(buf, 44, static_press_1_bias);
    _mav_put_float(buf, 48, static_press_1_scale);
    _mav_put_float(buf, 52, static_press_2_bias);
    _mav_put_float(buf, 56, static_press_2_scale);
    _mav_put_float(buf, 60, dpl_bay_press_bias);
    _mav_put_float(buf, 64, dpl_bay_press_scale);
    _mav_put_float(buf, 68, dynamic_press_bias);
    _mav_put_float(buf, 72, dynamic_press_scale);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CALIBRATION_TM, buf, MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_LEN, MAVLINK_MSG_ID_CALIBRATION_TM_CRC);
#else
    mavlink_calibration_tm_t *packet = (mavlink_calibration_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->gyro_bias_x = gyro_bias_x;
    packet->gyro_bias_y = gyro_bias_y;
    packet->gyro_bias_z = gyro_bias_z;
    packet->mag_bias_x = mag_bias_x;
    packet->mag_bias_y = mag_bias_y;
    packet->mag_bias_z = mag_bias_z;
    packet->mag_scale_x = mag_scale_x;
    packet->mag_scale_y = mag_scale_y;
    packet->mag_scale_z = mag_scale_z;
    packet->static_press_1_bias = static_press_1_bias;
    packet->static_press_1_scale = static_press_1_scale;
    packet->static_press_2_bias = static_press_2_bias;
    packet->static_press_2_scale = static_press_2_scale;
    packet->dpl_bay_press_bias = dpl_bay_press_bias;
    packet->dpl_bay_press_scale = dpl_bay_press_scale;
    packet->dynamic_press_bias = dynamic_press_bias;
    packet->dynamic_press_scale = dynamic_press_scale;

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
 * @brief Get field gyro_bias_x from calibration_tm message
 *
 * @return [deg/s] Gyroscope X bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field gyro_bias_y from calibration_tm message
 *
 * @return [deg/s] Gyroscope Y bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_bias_z from calibration_tm message
 *
 * @return [deg/s] Gyroscope Z bias
 */
static inline float mavlink_msg_calibration_tm_get_gyro_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field mag_bias_x from calibration_tm message
 *
 * @return [uT] Magnetometer X bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field mag_bias_y from calibration_tm message
 *
 * @return [uT] Magnetometer Y bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field mag_bias_z from calibration_tm message
 *
 * @return [uT] Magnetometer Z bias
 */
static inline float mavlink_msg_calibration_tm_get_mag_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field mag_scale_x from calibration_tm message
 *
 * @return  Magnetometer X scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mag_scale_y from calibration_tm message
 *
 * @return  Magnetometer Y scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field mag_scale_z from calibration_tm message
 *
 * @return  Magnetometer Z scale
 */
static inline float mavlink_msg_calibration_tm_get_mag_scale_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field static_press_1_bias from calibration_tm message
 *
 * @return [Pa] Static pressure 1 bias
 */
static inline float mavlink_msg_calibration_tm_get_static_press_1_bias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field static_press_1_scale from calibration_tm message
 *
 * @return  Static pressure 1 scale
 */
static inline float mavlink_msg_calibration_tm_get_static_press_1_scale(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field static_press_2_bias from calibration_tm message
 *
 * @return [Pa] Static pressure 2 bias
 */
static inline float mavlink_msg_calibration_tm_get_static_press_2_bias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field static_press_2_scale from calibration_tm message
 *
 * @return  Static pressure 2 scale
 */
static inline float mavlink_msg_calibration_tm_get_static_press_2_scale(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field dpl_bay_press_bias from calibration_tm message
 *
 * @return [Pa] Deployment bay pressure bias
 */
static inline float mavlink_msg_calibration_tm_get_dpl_bay_press_bias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field dpl_bay_press_scale from calibration_tm message
 *
 * @return  Deployment bay pressure scale
 */
static inline float mavlink_msg_calibration_tm_get_dpl_bay_press_scale(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field dynamic_press_bias from calibration_tm message
 *
 * @return [Pa] Dynamic pressure bias
 */
static inline float mavlink_msg_calibration_tm_get_dynamic_press_bias(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field dynamic_press_scale from calibration_tm message
 *
 * @return  Dynamic pressure scale
 */
static inline float mavlink_msg_calibration_tm_get_dynamic_press_scale(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
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
    calibration_tm->gyro_bias_x = mavlink_msg_calibration_tm_get_gyro_bias_x(msg);
    calibration_tm->gyro_bias_y = mavlink_msg_calibration_tm_get_gyro_bias_y(msg);
    calibration_tm->gyro_bias_z = mavlink_msg_calibration_tm_get_gyro_bias_z(msg);
    calibration_tm->mag_bias_x = mavlink_msg_calibration_tm_get_mag_bias_x(msg);
    calibration_tm->mag_bias_y = mavlink_msg_calibration_tm_get_mag_bias_y(msg);
    calibration_tm->mag_bias_z = mavlink_msg_calibration_tm_get_mag_bias_z(msg);
    calibration_tm->mag_scale_x = mavlink_msg_calibration_tm_get_mag_scale_x(msg);
    calibration_tm->mag_scale_y = mavlink_msg_calibration_tm_get_mag_scale_y(msg);
    calibration_tm->mag_scale_z = mavlink_msg_calibration_tm_get_mag_scale_z(msg);
    calibration_tm->static_press_1_bias = mavlink_msg_calibration_tm_get_static_press_1_bias(msg);
    calibration_tm->static_press_1_scale = mavlink_msg_calibration_tm_get_static_press_1_scale(msg);
    calibration_tm->static_press_2_bias = mavlink_msg_calibration_tm_get_static_press_2_bias(msg);
    calibration_tm->static_press_2_scale = mavlink_msg_calibration_tm_get_static_press_2_scale(msg);
    calibration_tm->dpl_bay_press_bias = mavlink_msg_calibration_tm_get_dpl_bay_press_bias(msg);
    calibration_tm->dpl_bay_press_scale = mavlink_msg_calibration_tm_get_dpl_bay_press_scale(msg);
    calibration_tm->dynamic_press_bias = mavlink_msg_calibration_tm_get_dynamic_press_bias(msg);
    calibration_tm->dynamic_press_scale = mavlink_msg_calibration_tm_get_dynamic_press_scale(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CALIBRATION_TM_LEN? msg->len : MAVLINK_MSG_ID_CALIBRATION_TM_LEN;
        memset(calibration_tm, 0, MAVLINK_MSG_ID_CALIBRATION_TM_LEN);
    memcpy(calibration_tm, _MAV_PAYLOAD(msg), len);
#endif
}
