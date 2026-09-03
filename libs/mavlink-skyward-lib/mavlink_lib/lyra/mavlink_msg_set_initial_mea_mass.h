#pragma once
// MESSAGE SET_INITIAL_MEA_MASS PACKING

#define MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS 19


typedef struct __mavlink_set_initial_mea_mass_t {
 float mass; /*< [kg] Mass*/
} mavlink_set_initial_mea_mass_t;

#define MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN 4
#define MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN 4
#define MAVLINK_MSG_ID_19_LEN 4
#define MAVLINK_MSG_ID_19_MIN_LEN 4

#define MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC 154
#define MAVLINK_MSG_ID_19_CRC 154



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SET_INITIAL_MEA_MASS { \
    19, \
    "SET_INITIAL_MEA_MASS", \
    1, \
    {  { "mass", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_initial_mea_mass_t, mass) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SET_INITIAL_MEA_MASS { \
    "SET_INITIAL_MEA_MASS", \
    1, \
    {  { "mass", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_set_initial_mea_mass_t, mass) }, \
         } \
}
#endif

/**
 * @brief Pack a set_initial_mea_mass message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mass [kg] Mass
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_initial_mea_mass_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN];
    _mav_put_float(buf, 0, mass);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN);
#else
    mavlink_set_initial_mea_mass_t packet;
    packet.mass = mass;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
}

/**
 * @brief Pack a set_initial_mea_mass message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mass [kg] Mass
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_initial_mea_mass_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN];
    _mav_put_float(buf, 0, mass);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN);
#else
    mavlink_set_initial_mea_mass_t packet;
    packet.mass = mass;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
}

/**
 * @brief Encode a set_initial_mea_mass struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_initial_mea_mass C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_initial_mea_mass_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_initial_mea_mass_t* set_initial_mea_mass)
{
    return mavlink_msg_set_initial_mea_mass_pack(system_id, component_id, msg, set_initial_mea_mass->mass);
}

/**
 * @brief Encode a set_initial_mea_mass struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_initial_mea_mass C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_initial_mea_mass_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_initial_mea_mass_t* set_initial_mea_mass)
{
    return mavlink_msg_set_initial_mea_mass_pack_chan(system_id, component_id, chan, msg, set_initial_mea_mass->mass);
}

/**
 * @brief Send a set_initial_mea_mass message
 * @param chan MAVLink channel to send the message
 *
 * @param mass [kg] Mass
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_initial_mea_mass_send(mavlink_channel_t chan, float mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN];
    _mav_put_float(buf, 0, mass);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS, buf, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
#else
    mavlink_set_initial_mea_mass_t packet;
    packet.mass = mass;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS, (const char *)&packet, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
#endif
}

/**
 * @brief Send a set_initial_mea_mass message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_set_initial_mea_mass_send_struct(mavlink_channel_t chan, const mavlink_set_initial_mea_mass_t* set_initial_mea_mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_set_initial_mea_mass_send(chan, set_initial_mea_mass->mass);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS, (const char *)set_initial_mea_mass, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_initial_mea_mass_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, mass);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS, buf, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
#else
    mavlink_set_initial_mea_mass_t *packet = (mavlink_set_initial_mea_mass_t *)msgbuf;
    packet->mass = mass;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS, (const char *)packet, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_MIN_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_CRC);
#endif
}
#endif

#endif

// MESSAGE SET_INITIAL_MEA_MASS UNPACKING


/**
 * @brief Get field mass from set_initial_mea_mass message
 *
 * @return [kg] Mass
 */
static inline float mavlink_msg_set_initial_mea_mass_get_mass(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a set_initial_mea_mass message into a struct
 *
 * @param msg The message to decode
 * @param set_initial_mea_mass C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_initial_mea_mass_decode(const mavlink_message_t* msg, mavlink_set_initial_mea_mass_t* set_initial_mea_mass)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    set_initial_mea_mass->mass = mavlink_msg_set_initial_mea_mass_get_mass(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN? msg->len : MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN;
        memset(set_initial_mea_mass, 0, MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_LEN);
    memcpy(set_initial_mea_mass, _MAV_PAYLOAD(msg), len);
#endif
}
