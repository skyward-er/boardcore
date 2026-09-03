#pragma once
// MESSAGE LR_SAMPLE_DATA_TM PACKING

#define MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM 150

MAVPACKED(
typedef struct __mavlink_lr_sample_data_tm_t {
 uint16_t pressure; /*<  Pressure value in raw value //TODO: add calibration curve*/
 uint16_t imu1_acc_x; /*<  Raw acceleration value of the x axis of the ADIS imu*/
 uint16_t imu1_acc_y; /*<  Raw acceleration value of the y axis of the ADIS imu*/
 uint16_t imu1_acc_z; /*<  Raw acceleration value of the z axis of the ADIS imu*/
 uint16_t imu1_gyr_x; /*<  Raw gyro value of the x axis of the ADIS imu*/
 uint16_t imu1_gyr_y; /*<  Raw gyro value of the y axis of the ADIS imu*/
 uint16_t imu1_gyr_z; /*<  Raw gyro value of the z axis of the ADIS imu*/
}) mavlink_lr_sample_data_tm_t;

#define MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN 14
#define MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN 14
#define MAVLINK_MSG_ID_150_LEN 14
#define MAVLINK_MSG_ID_150_MIN_LEN 14

#define MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC 244
#define MAVLINK_MSG_ID_150_CRC 244



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LR_SAMPLE_DATA_TM { \
    150, \
    "LR_SAMPLE_DATA_TM", \
    7, \
    {  { "pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lr_sample_data_tm_t, pressure) }, \
         { "imu1_acc_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_x) }, \
         { "imu1_acc_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_y) }, \
         { "imu1_acc_z", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_z) }, \
         { "imu1_gyr_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_x) }, \
         { "imu1_gyr_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_y) }, \
         { "imu1_gyr_z", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LR_SAMPLE_DATA_TM { \
    "LR_SAMPLE_DATA_TM", \
    7, \
    {  { "pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lr_sample_data_tm_t, pressure) }, \
         { "imu1_acc_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_x) }, \
         { "imu1_acc_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_y) }, \
         { "imu1_acc_z", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_lr_sample_data_tm_t, imu1_acc_z) }, \
         { "imu1_gyr_x", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_x) }, \
         { "imu1_gyr_y", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_y) }, \
         { "imu1_gyr_z", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_lr_sample_data_tm_t, imu1_gyr_z) }, \
         } \
}
#endif

/**
 * @brief Pack a lr_sample_data_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 * @param imu1_acc_x  Raw acceleration value of the x axis of the ADIS imu
 * @param imu1_acc_y  Raw acceleration value of the y axis of the ADIS imu
 * @param imu1_acc_z  Raw acceleration value of the z axis of the ADIS imu
 * @param imu1_gyr_x  Raw gyro value of the x axis of the ADIS imu
 * @param imu1_gyr_y  Raw gyro value of the y axis of the ADIS imu
 * @param imu1_gyr_z  Raw gyro value of the z axis of the ADIS imu
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t pressure, uint16_t imu1_acc_x, uint16_t imu1_acc_y, uint16_t imu1_acc_z, uint16_t imu1_gyr_x, uint16_t imu1_gyr_y, uint16_t imu1_gyr_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);
    _mav_put_uint16_t(buf, 2, imu1_acc_x);
    _mav_put_uint16_t(buf, 4, imu1_acc_y);
    _mav_put_uint16_t(buf, 6, imu1_acc_z);
    _mav_put_uint16_t(buf, 8, imu1_gyr_x);
    _mav_put_uint16_t(buf, 10, imu1_gyr_y);
    _mav_put_uint16_t(buf, 12, imu1_gyr_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN);
#else
    mavlink_lr_sample_data_tm_t packet;
    packet.pressure = pressure;
    packet.imu1_acc_x = imu1_acc_x;
    packet.imu1_acc_y = imu1_acc_y;
    packet.imu1_acc_z = imu1_acc_z;
    packet.imu1_gyr_x = imu1_gyr_x;
    packet.imu1_gyr_y = imu1_gyr_y;
    packet.imu1_gyr_z = imu1_gyr_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
}

/**
 * @brief Pack a lr_sample_data_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 * @param imu1_acc_x  Raw acceleration value of the x axis of the ADIS imu
 * @param imu1_acc_y  Raw acceleration value of the y axis of the ADIS imu
 * @param imu1_acc_z  Raw acceleration value of the z axis of the ADIS imu
 * @param imu1_gyr_x  Raw gyro value of the x axis of the ADIS imu
 * @param imu1_gyr_y  Raw gyro value of the y axis of the ADIS imu
 * @param imu1_gyr_z  Raw gyro value of the z axis of the ADIS imu
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t pressure,uint16_t imu1_acc_x,uint16_t imu1_acc_y,uint16_t imu1_acc_z,uint16_t imu1_gyr_x,uint16_t imu1_gyr_y,uint16_t imu1_gyr_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);
    _mav_put_uint16_t(buf, 2, imu1_acc_x);
    _mav_put_uint16_t(buf, 4, imu1_acc_y);
    _mav_put_uint16_t(buf, 6, imu1_acc_z);
    _mav_put_uint16_t(buf, 8, imu1_gyr_x);
    _mav_put_uint16_t(buf, 10, imu1_gyr_y);
    _mav_put_uint16_t(buf, 12, imu1_gyr_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN);
#else
    mavlink_lr_sample_data_tm_t packet;
    packet.pressure = pressure;
    packet.imu1_acc_x = imu1_acc_x;
    packet.imu1_acc_y = imu1_acc_y;
    packet.imu1_acc_z = imu1_acc_z;
    packet.imu1_gyr_x = imu1_gyr_x;
    packet.imu1_gyr_y = imu1_gyr_y;
    packet.imu1_gyr_z = imu1_gyr_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
}

/**
 * @brief Encode a lr_sample_data_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lr_sample_data_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lr_sample_data_tm_t* lr_sample_data_tm)
{
    return mavlink_msg_lr_sample_data_tm_pack(system_id, component_id, msg, lr_sample_data_tm->pressure, lr_sample_data_tm->imu1_acc_x, lr_sample_data_tm->imu1_acc_y, lr_sample_data_tm->imu1_acc_z, lr_sample_data_tm->imu1_gyr_x, lr_sample_data_tm->imu1_gyr_y, lr_sample_data_tm->imu1_gyr_z);
}

/**
 * @brief Encode a lr_sample_data_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lr_sample_data_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lr_sample_data_tm_t* lr_sample_data_tm)
{
    return mavlink_msg_lr_sample_data_tm_pack_chan(system_id, component_id, chan, msg, lr_sample_data_tm->pressure, lr_sample_data_tm->imu1_acc_x, lr_sample_data_tm->imu1_acc_y, lr_sample_data_tm->imu1_acc_z, lr_sample_data_tm->imu1_gyr_x, lr_sample_data_tm->imu1_gyr_y, lr_sample_data_tm->imu1_gyr_z);
}

/**
 * @brief Send a lr_sample_data_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 * @param imu1_acc_x  Raw acceleration value of the x axis of the ADIS imu
 * @param imu1_acc_y  Raw acceleration value of the y axis of the ADIS imu
 * @param imu1_acc_z  Raw acceleration value of the z axis of the ADIS imu
 * @param imu1_gyr_x  Raw gyro value of the x axis of the ADIS imu
 * @param imu1_gyr_y  Raw gyro value of the y axis of the ADIS imu
 * @param imu1_gyr_z  Raw gyro value of the z axis of the ADIS imu
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lr_sample_data_tm_send(mavlink_channel_t chan, uint16_t pressure, uint16_t imu1_acc_x, uint16_t imu1_acc_y, uint16_t imu1_acc_z, uint16_t imu1_gyr_x, uint16_t imu1_gyr_y, uint16_t imu1_gyr_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);
    _mav_put_uint16_t(buf, 2, imu1_acc_x);
    _mav_put_uint16_t(buf, 4, imu1_acc_y);
    _mav_put_uint16_t(buf, 6, imu1_acc_z);
    _mav_put_uint16_t(buf, 8, imu1_gyr_x);
    _mav_put_uint16_t(buf, 10, imu1_gyr_y);
    _mav_put_uint16_t(buf, 12, imu1_gyr_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM, buf, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
#else
    mavlink_lr_sample_data_tm_t packet;
    packet.pressure = pressure;
    packet.imu1_acc_x = imu1_acc_x;
    packet.imu1_acc_y = imu1_acc_y;
    packet.imu1_acc_z = imu1_acc_z;
    packet.imu1_gyr_x = imu1_gyr_x;
    packet.imu1_gyr_y = imu1_gyr_y;
    packet.imu1_gyr_z = imu1_gyr_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM, (const char *)&packet, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
#endif
}

/**
 * @brief Send a lr_sample_data_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lr_sample_data_tm_send_struct(mavlink_channel_t chan, const mavlink_lr_sample_data_tm_t* lr_sample_data_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lr_sample_data_tm_send(chan, lr_sample_data_tm->pressure, lr_sample_data_tm->imu1_acc_x, lr_sample_data_tm->imu1_acc_y, lr_sample_data_tm->imu1_acc_z, lr_sample_data_tm->imu1_gyr_x, lr_sample_data_tm->imu1_gyr_y, lr_sample_data_tm->imu1_gyr_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM, (const char *)lr_sample_data_tm, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lr_sample_data_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t pressure, uint16_t imu1_acc_x, uint16_t imu1_acc_y, uint16_t imu1_acc_z, uint16_t imu1_gyr_x, uint16_t imu1_gyr_y, uint16_t imu1_gyr_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, pressure);
    _mav_put_uint16_t(buf, 2, imu1_acc_x);
    _mav_put_uint16_t(buf, 4, imu1_acc_y);
    _mav_put_uint16_t(buf, 6, imu1_acc_z);
    _mav_put_uint16_t(buf, 8, imu1_gyr_x);
    _mav_put_uint16_t(buf, 10, imu1_gyr_y);
    _mav_put_uint16_t(buf, 12, imu1_gyr_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM, buf, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
#else
    mavlink_lr_sample_data_tm_t *packet = (mavlink_lr_sample_data_tm_t *)msgbuf;
    packet->pressure = pressure;
    packet->imu1_acc_x = imu1_acc_x;
    packet->imu1_acc_y = imu1_acc_y;
    packet->imu1_acc_z = imu1_acc_z;
    packet->imu1_gyr_x = imu1_gyr_x;
    packet->imu1_gyr_y = imu1_gyr_y;
    packet->imu1_gyr_z = imu1_gyr_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM, (const char *)packet, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE LR_SAMPLE_DATA_TM UNPACKING


/**
 * @brief Get field pressure from lr_sample_data_tm message
 *
 * @return  Pressure value in raw value //TODO: add calibration curve
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field imu1_acc_x from lr_sample_data_tm message
 *
 * @return  Raw acceleration value of the x axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field imu1_acc_y from lr_sample_data_tm message
 *
 * @return  Raw acceleration value of the y axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field imu1_acc_z from lr_sample_data_tm message
 *
 * @return  Raw acceleration value of the z axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field imu1_gyr_x from lr_sample_data_tm message
 *
 * @return  Raw gyro value of the x axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_gyr_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field imu1_gyr_y from lr_sample_data_tm message
 *
 * @return  Raw gyro value of the y axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_gyr_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field imu1_gyr_z from lr_sample_data_tm message
 *
 * @return  Raw gyro value of the z axis of the ADIS imu
 */
static inline uint16_t mavlink_msg_lr_sample_data_tm_get_imu1_gyr_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a lr_sample_data_tm message into a struct
 *
 * @param msg The message to decode
 * @param lr_sample_data_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_lr_sample_data_tm_decode(const mavlink_message_t* msg, mavlink_lr_sample_data_tm_t* lr_sample_data_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    lr_sample_data_tm->pressure = mavlink_msg_lr_sample_data_tm_get_pressure(msg);
    lr_sample_data_tm->imu1_acc_x = mavlink_msg_lr_sample_data_tm_get_imu1_acc_x(msg);
    lr_sample_data_tm->imu1_acc_y = mavlink_msg_lr_sample_data_tm_get_imu1_acc_y(msg);
    lr_sample_data_tm->imu1_acc_z = mavlink_msg_lr_sample_data_tm_get_imu1_acc_z(msg);
    lr_sample_data_tm->imu1_gyr_x = mavlink_msg_lr_sample_data_tm_get_imu1_gyr_x(msg);
    lr_sample_data_tm->imu1_gyr_y = mavlink_msg_lr_sample_data_tm_get_imu1_gyr_y(msg);
    lr_sample_data_tm->imu1_gyr_z = mavlink_msg_lr_sample_data_tm_get_imu1_gyr_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN? msg->len : MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN;
        memset(lr_sample_data_tm, 0, MAVLINK_MSG_ID_LR_SAMPLE_DATA_TM_LEN);
    memcpy(lr_sample_data_tm, _MAV_PAYLOAD(msg), len);
#endif
}
