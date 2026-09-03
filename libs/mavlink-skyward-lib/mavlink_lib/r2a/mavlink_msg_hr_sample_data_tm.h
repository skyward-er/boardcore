#pragma once
// MESSAGE HR_SAMPLE_DATA_TM PACKING

#define MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM 155

MAVPACKED(
typedef struct __mavlink_hr_sample_data_tm_t {
 uint16_t pressure; /*<  Pressure value in raw value //TODO: add calibration curve*/
}) mavlink_hr_sample_data_tm_t;

#define MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN 2
#define MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN 2
#define MAVLINK_MSG_ID_155_LEN 2
#define MAVLINK_MSG_ID_155_MIN_LEN 2

#define MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC 50
#define MAVLINK_MSG_ID_155_CRC 50



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_HR_SAMPLE_DATA_TM { \
    155, \
    "HR_SAMPLE_DATA_TM", \
    1, \
    {  { "pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hr_sample_data_tm_t, pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_HR_SAMPLE_DATA_TM { \
    "HR_SAMPLE_DATA_TM", \
    1, \
    {  { "pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_hr_sample_data_tm_t, pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a hr_sample_data_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_sample_data_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN);
#else
    mavlink_hr_sample_data_tm_t packet;
    packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
}

/**
 * @brief Pack a hr_sample_data_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_hr_sample_data_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN);
#else
    mavlink_hr_sample_data_tm_t packet;
    packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
}

/**
 * @brief Encode a hr_sample_data_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param hr_sample_data_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_sample_data_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_hr_sample_data_tm_t* hr_sample_data_tm)
{
    return mavlink_msg_hr_sample_data_tm_pack(system_id, component_id, msg, hr_sample_data_tm->pressure);
}

/**
 * @brief Encode a hr_sample_data_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hr_sample_data_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_hr_sample_data_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_hr_sample_data_tm_t* hr_sample_data_tm)
{
    return mavlink_msg_hr_sample_data_tm_pack_chan(system_id, component_id, chan, msg, hr_sample_data_tm->pressure);
}

/**
 * @brief Send a hr_sample_data_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param pressure  Pressure value in raw value //TODO: add calibration curve
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_hr_sample_data_tm_send(mavlink_channel_t chan, uint16_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN];
    _mav_put_uint16_t(buf, 0, pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM, buf, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
#else
    mavlink_hr_sample_data_tm_t packet;
    packet.pressure = pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM, (const char *)&packet, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
#endif
}

/**
 * @brief Send a hr_sample_data_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_hr_sample_data_tm_send_struct(mavlink_channel_t chan, const mavlink_hr_sample_data_tm_t* hr_sample_data_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_hr_sample_data_tm_send(chan, hr_sample_data_tm->pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM, (const char *)hr_sample_data_tm, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_hr_sample_data_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM, buf, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
#else
    mavlink_hr_sample_data_tm_t *packet = (mavlink_hr_sample_data_tm_t *)msgbuf;
    packet->pressure = pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM, (const char *)packet, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_MIN_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE HR_SAMPLE_DATA_TM UNPACKING


/**
 * @brief Get field pressure from hr_sample_data_tm message
 *
 * @return  Pressure value in raw value //TODO: add calibration curve
 */
static inline uint16_t mavlink_msg_hr_sample_data_tm_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a hr_sample_data_tm message into a struct
 *
 * @param msg The message to decode
 * @param hr_sample_data_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_hr_sample_data_tm_decode(const mavlink_message_t* msg, mavlink_hr_sample_data_tm_t* hr_sample_data_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    hr_sample_data_tm->pressure = mavlink_msg_hr_sample_data_tm_get_pressure(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN? msg->len : MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN;
        memset(hr_sample_data_tm, 0, MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN);
    memcpy(hr_sample_data_tm, _MAV_PAYLOAD(msg), len);
#endif
}
