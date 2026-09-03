#pragma once
// MESSAGE SET_CALIBRATION_PRESSURE PACKING

#define MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE 18


typedef struct __mavlink_set_calibration_pressure_t {
 float pressure; /*< [Pa] Pressure*/
} mavlink_set_calibration_pressure_t;

#define MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN 4
#define MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN 4
#define MAVLINK_MSG_ID_18_LEN 4
#define MAVLINK_MSG_ID_18_MIN_LEN 4

#define MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC 31
#define MAVLINK_MSG_ID_18_CRC 31



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_CALIBRATION_PRESSURE { \
    18, \
    "SET_CALIBRATION_PRESSURE", \
    1, \
    {  { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_calibration_pressure_t, pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_CALIBRATION_PRESSURE { \
    "SET_CALIBRATION_PRESSURE", \
    1, \
    {  { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_calibration_pressure_t, pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a set_calibration_pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pressure [Pa] Pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_calibration_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN];
    _mav_put_float(buf, 0, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN);
#else
    mavlink_set_calibration_pressure_t packet;
    packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
}

/**
 * @brief Pack a set_calibration_pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure [Pa] Pressure
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_calibration_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN];
    _mav_put_float(buf, 0, pressure);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN);
#else
    mavlink_set_calibration_pressure_t packet;
    packet.pressure = pressure;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
}

/**
 * @brief Encode a set_calibration_pressure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_calibration_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_calibration_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_calibration_pressure_t* set_calibration_pressure)
{
    return mavlink_msg_set_calibration_pressure_pack(system_id, component_id, msg, set_calibration_pressure->pressure);
}

/**
 * @brief Encode a set_calibration_pressure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_calibration_pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_calibration_pressure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_calibration_pressure_t* set_calibration_pressure)
{
    return mavlink_msg_set_calibration_pressure_pack_chan(system_id, component_id, chan, msg, set_calibration_pressure->pressure);
}

/**
 * @brief Send a set_calibration_pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param pressure [Pa] Pressure
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_calibration_pressure_send(mavlink_channel_t chan, float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN];
    _mav_put_float(buf, 0, pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE, buf, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
#else
    mavlink_set_calibration_pressure_t packet;
    packet.pressure = pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE, (const char *)&packet, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
#endif
}

/**
 * @brief Send a set_calibration_pressure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_calibration_pressure_send_struct(mavlink_channel_t chan, const mavlink_set_calibration_pressure_t* set_calibration_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_calibration_pressure_send(chan, set_calibration_pressure->pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE, (const char *)set_calibration_pressure, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_calibration_pressure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pressure);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE, buf, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
#else
    mavlink_set_calibration_pressure_t *packet = (mavlink_set_calibration_pressure_t *)msgbuf;
    packet->pressure = pressure;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE, (const char *)packet, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_CALIBRATION_PRESSURE UNPACKING


/**
 * @brief Get field pressure from set_calibration_pressure message
 *
 * @return [Pa] Pressure
 */
static inline float mavlink_msg_set_calibration_pressure_get_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_calibration_pressure message into a struct
 *
 * @param msg The message to decode
 * @param set_calibration_pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_calibration_pressure_decode(const mavlink_message_t* msg, mavlink_set_calibration_pressure_t* set_calibration_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_calibration_pressure->pressure = mavlink_msg_set_calibration_pressure_get_pressure(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN? msg->len : MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN;
        memset(set_calibration_pressure, 0, MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_LEN);
    memcpy(set_calibration_pressure, _MAV_PAYLOAD(msg), len);
#endif
}
