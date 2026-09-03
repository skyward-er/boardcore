#pragma once
// MESSAGE SERVO_TM PACKING

#define MAVLINK_MSG_ID_SERVO_TM 112


typedef struct __mavlink_servo_tm_t {
 float servo_position; /*<  Position of the servo [0-1]*/
 uint8_t servo_id; /*<  A member of the ServosList enum*/
} mavlink_servo_tm_t;

#define MAVLINK_MSG_ID_SERVO_TM_LEN 5
#define MAVLINK_MSG_ID_SERVO_TM_MIN_LEN 5
#define MAVLINK_MSG_ID_112_LEN 5
#define MAVLINK_MSG_ID_112_MIN_LEN 5

#define MAVLINK_MSG_ID_SERVO_TM_CRC 87
#define MAVLINK_MSG_ID_112_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SERVO_TM { \
    112, \
    "SERVO_TM", \
    2, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_servo_tm_t, servo_id) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_servo_tm_t, servo_position) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SERVO_TM { \
    "SERVO_TM", \
    2, \
    {  { "servo_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_servo_tm_t, servo_id) }, \
         { "servo_position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_servo_tm_t, servo_position) }, \
         } \
}
#endif

/**
 * @brief Pack a servo_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servo_id  A member of the ServosList enum
 * @param servo_position  Position of the servo [0-1]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t servo_id, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_TM_LEN];
    _mav_put_float(buf, 0, servo_position);
    _mav_put_uint8_t(buf, 4, servo_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERVO_TM_LEN);
#else
    mavlink_servo_tm_t packet;
    packet.servo_position = servo_position;
    packet.servo_id = servo_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERVO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERVO_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
}

/**
 * @brief Pack a servo_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo_id  A member of the ServosList enum
 * @param servo_position  Position of the servo [0-1]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_servo_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t servo_id,float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_TM_LEN];
    _mav_put_float(buf, 0, servo_position);
    _mav_put_uint8_t(buf, 4, servo_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERVO_TM_LEN);
#else
    mavlink_servo_tm_t packet;
    packet.servo_position = servo_position;
    packet.servo_id = servo_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERVO_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SERVO_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
}

/**
 * @brief Encode a servo_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param servo_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_servo_tm_t* servo_tm)
{
    return mavlink_msg_servo_tm_pack(system_id, component_id, msg, servo_tm->servo_id, servo_tm->servo_position);
}

/**
 * @brief Encode a servo_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param servo_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_servo_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_servo_tm_t* servo_tm)
{
    return mavlink_msg_servo_tm_pack_chan(system_id, component_id, chan, msg, servo_tm->servo_id, servo_tm->servo_position);
}

/**
 * @brief Send a servo_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param servo_id  A member of the ServosList enum
 * @param servo_position  Position of the servo [0-1]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_servo_tm_send(mavlink_channel_t chan, uint8_t servo_id, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SERVO_TM_LEN];
    _mav_put_float(buf, 0, servo_position);
    _mav_put_uint8_t(buf, 4, servo_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_TM, buf, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
#else
    mavlink_servo_tm_t packet;
    packet.servo_position = servo_position;
    packet.servo_id = servo_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_TM, (const char *)&packet, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
#endif
}

/**
 * @brief Send a servo_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_servo_tm_send_struct(mavlink_channel_t chan, const mavlink_servo_tm_t* servo_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_servo_tm_send(chan, servo_tm->servo_id, servo_tm->servo_position);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_TM, (const char *)servo_tm, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_SERVO_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_servo_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t servo_id, float servo_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, servo_position);
    _mav_put_uint8_t(buf, 4, servo_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_TM, buf, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
#else
    mavlink_servo_tm_t *packet = (mavlink_servo_tm_t *)msgbuf;
    packet->servo_position = servo_position;
    packet->servo_id = servo_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERVO_TM, (const char *)packet, MAVLINK_MSG_ID_SERVO_TM_MIN_LEN, MAVLINK_MSG_ID_SERVO_TM_LEN, MAVLINK_MSG_ID_SERVO_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE SERVO_TM UNPACKING


/**
 * @brief Get field servo_id from servo_tm message
 *
 * @return  A member of the ServosList enum
 */
static inline uint8_t mavlink_msg_servo_tm_get_servo_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field servo_position from servo_tm message
 *
 * @return  Position of the servo [0-1]
 */
static inline float mavlink_msg_servo_tm_get_servo_position(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a servo_tm message into a struct
 *
 * @param msg The message to decode
 * @param servo_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_servo_tm_decode(const mavlink_message_t* msg, mavlink_servo_tm_t* servo_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    servo_tm->servo_position = mavlink_msg_servo_tm_get_servo_position(msg);
    servo_tm->servo_id = mavlink_msg_servo_tm_get_servo_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SERVO_TM_LEN? msg->len : MAVLINK_MSG_ID_SERVO_TM_LEN;
        memset(servo_tm, 0, MAVLINK_MSG_ID_SERVO_TM_LEN);
    memcpy(servo_tm, _MAV_PAYLOAD(msg), len);
#endif
}
