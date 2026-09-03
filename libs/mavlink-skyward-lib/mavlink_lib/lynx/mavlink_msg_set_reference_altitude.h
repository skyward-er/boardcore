#pragma once
// MESSAGE SET_REFERENCE_ALTITUDE PACKING

#define MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE 23

MAVPACKED(
typedef struct __mavlink_set_reference_altitude_t {
 float ref_altitude; /*< [m] Reference altitude*/
}) mavlink_set_reference_altitude_t;

#define MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN 4
#define MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN 4
#define MAVLINK_MSG_ID_23_LEN 4
#define MAVLINK_MSG_ID_23_MIN_LEN 4

#define MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC 95
#define MAVLINK_MSG_ID_23_CRC 95



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_REFERENCE_ALTITUDE { \
    23, \
    "SET_REFERENCE_ALTITUDE", \
    1, \
    {  { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_reference_altitude_t, ref_altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_REFERENCE_ALTITUDE { \
    "SET_REFERENCE_ALTITUDE", \
    1, \
    {  { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_reference_altitude_t, ref_altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a set_reference_altitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ref_altitude [m] Reference altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_reference_altitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float ref_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN];
    _mav_put_float(buf, 0, ref_altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN);
#else
    mavlink_set_reference_altitude_t packet;
    packet.ref_altitude = ref_altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
}

/**
 * @brief Pack a set_reference_altitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ref_altitude [m] Reference altitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_reference_altitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float ref_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN];
    _mav_put_float(buf, 0, ref_altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN);
#else
    mavlink_set_reference_altitude_t packet;
    packet.ref_altitude = ref_altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
}

/**
 * @brief Encode a set_reference_altitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_reference_altitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_reference_altitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_reference_altitude_t* set_reference_altitude)
{
    return mavlink_msg_set_reference_altitude_pack(system_id, component_id, msg, set_reference_altitude->ref_altitude);
}

/**
 * @brief Encode a set_reference_altitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_reference_altitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_reference_altitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_reference_altitude_t* set_reference_altitude)
{
    return mavlink_msg_set_reference_altitude_pack_chan(system_id, component_id, chan, msg, set_reference_altitude->ref_altitude);
}

/**
 * @brief Send a set_reference_altitude message
 * @param chan MAVLink channel to send the message
 *
 * @param ref_altitude [m] Reference altitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_reference_altitude_send(mavlink_channel_t chan, float ref_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN];
    _mav_put_float(buf, 0, ref_altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE, buf, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
#else
    mavlink_set_reference_altitude_t packet;
    packet.ref_altitude = ref_altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE, (const char *)&packet, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
#endif
}

/**
 * @brief Send a set_reference_altitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_reference_altitude_send_struct(mavlink_channel_t chan, const mavlink_set_reference_altitude_t* set_reference_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_reference_altitude_send(chan, set_reference_altitude->ref_altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE, (const char *)set_reference_altitude, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_reference_altitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float ref_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, ref_altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE, buf, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
#else
    mavlink_set_reference_altitude_t *packet = (mavlink_set_reference_altitude_t *)msgbuf;
    packet->ref_altitude = ref_altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE, (const char *)packet, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_MIN_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_REFERENCE_ALTITUDE UNPACKING


/**
 * @brief Get field ref_altitude from set_reference_altitude message
 *
 * @return [m] Reference altitude
 */
static inline float mavlink_msg_set_reference_altitude_get_ref_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_reference_altitude message into a struct
 *
 * @param msg The message to decode
 * @param set_reference_altitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_reference_altitude_decode(const mavlink_message_t* msg, mavlink_set_reference_altitude_t* set_reference_altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_reference_altitude->ref_altitude = mavlink_msg_set_reference_altitude_get_ref_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN? msg->len : MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN;
        memset(set_reference_altitude, 0, MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_LEN);
    memcpy(set_reference_altitude, _MAV_PAYLOAD(msg), len);
#endif
}
