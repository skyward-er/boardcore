#pragma once
// MESSAGE UPLOAD_SETTING_TC PACKING

#define MAVLINK_MSG_ID_UPLOAD_SETTING_TC 21

MAVPACKED(
typedef struct __mavlink_upload_setting_tc_t {
 float setting; /*<  The value of the setting to be uploaded*/
 uint8_t setting_id; /*<  A member of the MavSettingList enum.*/
}) mavlink_upload_setting_tc_t;

#define MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN 5
#define MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN 5
#define MAVLINK_MSG_ID_21_LEN 5
#define MAVLINK_MSG_ID_21_MIN_LEN 5

#define MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC 90
#define MAVLINK_MSG_ID_21_CRC 90



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UPLOAD_SETTING_TC { \
    21, \
    "UPLOAD_SETTING_TC", \
    2, \
    {  { "setting_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_upload_setting_tc_t, setting_id) }, \
         { "setting", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_upload_setting_tc_t, setting) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UPLOAD_SETTING_TC { \
    "UPLOAD_SETTING_TC", \
    2, \
    {  { "setting_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_upload_setting_tc_t, setting_id) }, \
         { "setting", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_upload_setting_tc_t, setting) }, \
         } \
}
#endif

/**
 * @brief Pack a upload_setting_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param setting_id  A member of the MavSettingList enum.
 * @param setting  The value of the setting to be uploaded
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_upload_setting_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t setting_id, float setting)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN];
    _mav_put_float(buf, 0, setting);
    _mav_put_uint8_t(buf, 4, setting_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN);
#else
    mavlink_upload_setting_tc_t packet;
    packet.setting = setting;
    packet.setting_id = setting_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UPLOAD_SETTING_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
}

/**
 * @brief Pack a upload_setting_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param setting_id  A member of the MavSettingList enum.
 * @param setting  The value of the setting to be uploaded
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_upload_setting_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t setting_id,float setting)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN];
    _mav_put_float(buf, 0, setting);
    _mav_put_uint8_t(buf, 4, setting_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN);
#else
    mavlink_upload_setting_tc_t packet;
    packet.setting = setting;
    packet.setting_id = setting_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UPLOAD_SETTING_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
}

/**
 * @brief Encode a upload_setting_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param upload_setting_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_upload_setting_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_upload_setting_tc_t* upload_setting_tc)
{
    return mavlink_msg_upload_setting_tc_pack(system_id, component_id, msg, upload_setting_tc->setting_id, upload_setting_tc->setting);
}

/**
 * @brief Encode a upload_setting_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param upload_setting_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_upload_setting_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_upload_setting_tc_t* upload_setting_tc)
{
    return mavlink_msg_upload_setting_tc_pack_chan(system_id, component_id, chan, msg, upload_setting_tc->setting_id, upload_setting_tc->setting);
}

/**
 * @brief Send a upload_setting_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param setting_id  A member of the MavSettingList enum.
 * @param setting  The value of the setting to be uploaded
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_upload_setting_tc_send(mavlink_channel_t chan, uint8_t setting_id, float setting)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN];
    _mav_put_float(buf, 0, setting);
    _mav_put_uint8_t(buf, 4, setting_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC, buf, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
#else
    mavlink_upload_setting_tc_t packet;
    packet.setting = setting;
    packet.setting_id = setting_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC, (const char *)&packet, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
#endif
}

/**
 * @brief Send a upload_setting_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_upload_setting_tc_send_struct(mavlink_channel_t chan, const mavlink_upload_setting_tc_t* upload_setting_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_upload_setting_tc_send(chan, upload_setting_tc->setting_id, upload_setting_tc->setting);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC, (const char *)upload_setting_tc, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_upload_setting_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t setting_id, float setting)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, setting);
    _mav_put_uint8_t(buf, 4, setting_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC, buf, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
#else
    mavlink_upload_setting_tc_t *packet = (mavlink_upload_setting_tc_t *)msgbuf;
    packet->setting = setting;
    packet->setting_id = setting_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UPLOAD_SETTING_TC, (const char *)packet, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_MIN_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE UPLOAD_SETTING_TC UNPACKING


/**
 * @brief Get field setting_id from upload_setting_tc message
 *
 * @return  A member of the MavSettingList enum.
 */
static inline uint8_t mavlink_msg_upload_setting_tc_get_setting_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field setting from upload_setting_tc message
 *
 * @return  The value of the setting to be uploaded
 */
static inline float mavlink_msg_upload_setting_tc_get_setting(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a upload_setting_tc message into a struct
 *
 * @param msg The message to decode
 * @param upload_setting_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_upload_setting_tc_decode(const mavlink_message_t* msg, mavlink_upload_setting_tc_t* upload_setting_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    upload_setting_tc->setting = mavlink_msg_upload_setting_tc_get_setting(msg);
    upload_setting_tc->setting_id = mavlink_msg_upload_setting_tc_get_setting_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN? msg->len : MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN;
        memset(upload_setting_tc, 0, MAVLINK_MSG_ID_UPLOAD_SETTING_TC_LEN);
    memcpy(upload_setting_tc, _MAV_PAYLOAD(msg), len);
#endif
}
