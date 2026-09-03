#pragma once
// MESSAGE CONRIG_STATE_TC PACKING

#define MAVLINK_MSG_ID_CONRIG_STATE_TC 32


typedef struct __mavlink_conrig_state_tc_t {
 uint8_t ox_filling_btn; /*<  OX filling valve button state*/
 uint8_t ox_release_btn; /*<  OX release line pressure valve button state*/
 uint8_t ox_detach_btn; /*<  OX quick connector detach button state*/
 uint8_t ox_venting_btn; /*<  OX venting valve button state*/
 uint8_t n2_filling_btn; /*<  N2 filling valve button state*/
 uint8_t n2_release_btn; /*<  N2 release line pressure valve button state*/
 uint8_t n2_detach_btn; /*<  N2 quick connector detach button state*/
 uint8_t n2_quenching_btn; /*<  N2 quenching valve button state*/
 uint8_t n2_3way_switch; /*<  N2 3-way valve switch state*/
 uint8_t tars_switch; /*<  TARS switch state*/
 uint8_t nitrogen_btn; /*<  Nitrogen valve button state*/
 uint8_t ignition_btn; /*<  Ignition button state*/
 uint8_t arm_switch; /*<  Arming switch state*/
 uint8_t clacson_switch; /*<  Clacson switch state*/
} mavlink_conrig_state_tc_t;

#define MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN 14
#define MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN 14
#define MAVLINK_MSG_ID_32_LEN 14
#define MAVLINK_MSG_ID_32_MIN_LEN 14

#define MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC 114
#define MAVLINK_MSG_ID_32_CRC 114



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONRIG_STATE_TC { \
    32, \
    "CONRIG_STATE_TC", \
    14, \
    {  { "ox_filling_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_conrig_state_tc_t, ox_filling_btn) }, \
         { "ox_release_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_conrig_state_tc_t, ox_release_btn) }, \
         { "ox_detach_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_conrig_state_tc_t, ox_detach_btn) }, \
         { "ox_venting_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_conrig_state_tc_t, ox_venting_btn) }, \
         { "n2_filling_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_conrig_state_tc_t, n2_filling_btn) }, \
         { "n2_release_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_conrig_state_tc_t, n2_release_btn) }, \
         { "n2_detach_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_conrig_state_tc_t, n2_detach_btn) }, \
         { "n2_quenching_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_conrig_state_tc_t, n2_quenching_btn) }, \
         { "n2_3way_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_conrig_state_tc_t, n2_3way_switch) }, \
         { "tars_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_conrig_state_tc_t, tars_switch) }, \
         { "nitrogen_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_conrig_state_tc_t, nitrogen_btn) }, \
         { "ignition_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_conrig_state_tc_t, ignition_btn) }, \
         { "arm_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_conrig_state_tc_t, arm_switch) }, \
         { "clacson_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_conrig_state_tc_t, clacson_switch) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONRIG_STATE_TC { \
    "CONRIG_STATE_TC", \
    14, \
    {  { "ox_filling_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_conrig_state_tc_t, ox_filling_btn) }, \
         { "ox_release_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_conrig_state_tc_t, ox_release_btn) }, \
         { "ox_detach_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_conrig_state_tc_t, ox_detach_btn) }, \
         { "ox_venting_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_conrig_state_tc_t, ox_venting_btn) }, \
         { "n2_filling_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_conrig_state_tc_t, n2_filling_btn) }, \
         { "n2_release_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_conrig_state_tc_t, n2_release_btn) }, \
         { "n2_detach_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_conrig_state_tc_t, n2_detach_btn) }, \
         { "n2_quenching_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_conrig_state_tc_t, n2_quenching_btn) }, \
         { "n2_3way_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_conrig_state_tc_t, n2_3way_switch) }, \
         { "tars_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_conrig_state_tc_t, tars_switch) }, \
         { "nitrogen_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_conrig_state_tc_t, nitrogen_btn) }, \
         { "ignition_btn", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_conrig_state_tc_t, ignition_btn) }, \
         { "arm_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_conrig_state_tc_t, arm_switch) }, \
         { "clacson_switch", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_conrig_state_tc_t, clacson_switch) }, \
         } \
}
#endif

/**
 * @brief Pack a conrig_state_tc message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ox_filling_btn  OX filling valve button state
 * @param ox_release_btn  OX release line pressure valve button state
 * @param ox_detach_btn  OX quick connector detach button state
 * @param ox_venting_btn  OX venting valve button state
 * @param n2_filling_btn  N2 filling valve button state
 * @param n2_release_btn  N2 release line pressure valve button state
 * @param n2_detach_btn  N2 quick connector detach button state
 * @param n2_quenching_btn  N2 quenching valve button state
 * @param n2_3way_switch  N2 3-way valve switch state
 * @param tars_switch  TARS switch state
 * @param nitrogen_btn  Nitrogen valve button state
 * @param ignition_btn  Ignition button state
 * @param arm_switch  Arming switch state
 * @param clacson_switch  Clacson switch state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_conrig_state_tc_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t ox_filling_btn, uint8_t ox_release_btn, uint8_t ox_detach_btn, uint8_t ox_venting_btn, uint8_t n2_filling_btn, uint8_t n2_release_btn, uint8_t n2_detach_btn, uint8_t n2_quenching_btn, uint8_t n2_3way_switch, uint8_t tars_switch, uint8_t nitrogen_btn, uint8_t ignition_btn, uint8_t arm_switch, uint8_t clacson_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN];
    _mav_put_uint8_t(buf, 0, ox_filling_btn);
    _mav_put_uint8_t(buf, 1, ox_release_btn);
    _mav_put_uint8_t(buf, 2, ox_detach_btn);
    _mav_put_uint8_t(buf, 3, ox_venting_btn);
    _mav_put_uint8_t(buf, 4, n2_filling_btn);
    _mav_put_uint8_t(buf, 5, n2_release_btn);
    _mav_put_uint8_t(buf, 6, n2_detach_btn);
    _mav_put_uint8_t(buf, 7, n2_quenching_btn);
    _mav_put_uint8_t(buf, 8, n2_3way_switch);
    _mav_put_uint8_t(buf, 9, tars_switch);
    _mav_put_uint8_t(buf, 10, nitrogen_btn);
    _mav_put_uint8_t(buf, 11, ignition_btn);
    _mav_put_uint8_t(buf, 12, arm_switch);
    _mav_put_uint8_t(buf, 13, clacson_switch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN);
#else
    mavlink_conrig_state_tc_t packet;
    packet.ox_filling_btn = ox_filling_btn;
    packet.ox_release_btn = ox_release_btn;
    packet.ox_detach_btn = ox_detach_btn;
    packet.ox_venting_btn = ox_venting_btn;
    packet.n2_filling_btn = n2_filling_btn;
    packet.n2_release_btn = n2_release_btn;
    packet.n2_detach_btn = n2_detach_btn;
    packet.n2_quenching_btn = n2_quenching_btn;
    packet.n2_3way_switch = n2_3way_switch;
    packet.tars_switch = tars_switch;
    packet.nitrogen_btn = nitrogen_btn;
    packet.ignition_btn = ignition_btn;
    packet.arm_switch = arm_switch;
    packet.clacson_switch = clacson_switch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONRIG_STATE_TC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
}

/**
 * @brief Pack a conrig_state_tc message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ox_filling_btn  OX filling valve button state
 * @param ox_release_btn  OX release line pressure valve button state
 * @param ox_detach_btn  OX quick connector detach button state
 * @param ox_venting_btn  OX venting valve button state
 * @param n2_filling_btn  N2 filling valve button state
 * @param n2_release_btn  N2 release line pressure valve button state
 * @param n2_detach_btn  N2 quick connector detach button state
 * @param n2_quenching_btn  N2 quenching valve button state
 * @param n2_3way_switch  N2 3-way valve switch state
 * @param tars_switch  TARS switch state
 * @param nitrogen_btn  Nitrogen valve button state
 * @param ignition_btn  Ignition button state
 * @param arm_switch  Arming switch state
 * @param clacson_switch  Clacson switch state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_conrig_state_tc_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t ox_filling_btn,uint8_t ox_release_btn,uint8_t ox_detach_btn,uint8_t ox_venting_btn,uint8_t n2_filling_btn,uint8_t n2_release_btn,uint8_t n2_detach_btn,uint8_t n2_quenching_btn,uint8_t n2_3way_switch,uint8_t tars_switch,uint8_t nitrogen_btn,uint8_t ignition_btn,uint8_t arm_switch,uint8_t clacson_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN];
    _mav_put_uint8_t(buf, 0, ox_filling_btn);
    _mav_put_uint8_t(buf, 1, ox_release_btn);
    _mav_put_uint8_t(buf, 2, ox_detach_btn);
    _mav_put_uint8_t(buf, 3, ox_venting_btn);
    _mav_put_uint8_t(buf, 4, n2_filling_btn);
    _mav_put_uint8_t(buf, 5, n2_release_btn);
    _mav_put_uint8_t(buf, 6, n2_detach_btn);
    _mav_put_uint8_t(buf, 7, n2_quenching_btn);
    _mav_put_uint8_t(buf, 8, n2_3way_switch);
    _mav_put_uint8_t(buf, 9, tars_switch);
    _mav_put_uint8_t(buf, 10, nitrogen_btn);
    _mav_put_uint8_t(buf, 11, ignition_btn);
    _mav_put_uint8_t(buf, 12, arm_switch);
    _mav_put_uint8_t(buf, 13, clacson_switch);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN);
#else
    mavlink_conrig_state_tc_t packet;
    packet.ox_filling_btn = ox_filling_btn;
    packet.ox_release_btn = ox_release_btn;
    packet.ox_detach_btn = ox_detach_btn;
    packet.ox_venting_btn = ox_venting_btn;
    packet.n2_filling_btn = n2_filling_btn;
    packet.n2_release_btn = n2_release_btn;
    packet.n2_detach_btn = n2_detach_btn;
    packet.n2_quenching_btn = n2_quenching_btn;
    packet.n2_3way_switch = n2_3way_switch;
    packet.tars_switch = tars_switch;
    packet.nitrogen_btn = nitrogen_btn;
    packet.ignition_btn = ignition_btn;
    packet.arm_switch = arm_switch;
    packet.clacson_switch = clacson_switch;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONRIG_STATE_TC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
}

/**
 * @brief Encode a conrig_state_tc struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param conrig_state_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_conrig_state_tc_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_conrig_state_tc_t* conrig_state_tc)
{
    return mavlink_msg_conrig_state_tc_pack(system_id, component_id, msg, conrig_state_tc->ox_filling_btn, conrig_state_tc->ox_release_btn, conrig_state_tc->ox_detach_btn, conrig_state_tc->ox_venting_btn, conrig_state_tc->n2_filling_btn, conrig_state_tc->n2_release_btn, conrig_state_tc->n2_detach_btn, conrig_state_tc->n2_quenching_btn, conrig_state_tc->n2_3way_switch, conrig_state_tc->tars_switch, conrig_state_tc->nitrogen_btn, conrig_state_tc->ignition_btn, conrig_state_tc->arm_switch, conrig_state_tc->clacson_switch);
}

/**
 * @brief Encode a conrig_state_tc struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param conrig_state_tc C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_conrig_state_tc_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_conrig_state_tc_t* conrig_state_tc)
{
    return mavlink_msg_conrig_state_tc_pack_chan(system_id, component_id, chan, msg, conrig_state_tc->ox_filling_btn, conrig_state_tc->ox_release_btn, conrig_state_tc->ox_detach_btn, conrig_state_tc->ox_venting_btn, conrig_state_tc->n2_filling_btn, conrig_state_tc->n2_release_btn, conrig_state_tc->n2_detach_btn, conrig_state_tc->n2_quenching_btn, conrig_state_tc->n2_3way_switch, conrig_state_tc->tars_switch, conrig_state_tc->nitrogen_btn, conrig_state_tc->ignition_btn, conrig_state_tc->arm_switch, conrig_state_tc->clacson_switch);
}

/**
 * @brief Send a conrig_state_tc message
 * @param chan MAVLink channel to send the message
 *
 * @param ox_filling_btn  OX filling valve button state
 * @param ox_release_btn  OX release line pressure valve button state
 * @param ox_detach_btn  OX quick connector detach button state
 * @param ox_venting_btn  OX venting valve button state
 * @param n2_filling_btn  N2 filling valve button state
 * @param n2_release_btn  N2 release line pressure valve button state
 * @param n2_detach_btn  N2 quick connector detach button state
 * @param n2_quenching_btn  N2 quenching valve button state
 * @param n2_3way_switch  N2 3-way valve switch state
 * @param tars_switch  TARS switch state
 * @param nitrogen_btn  Nitrogen valve button state
 * @param ignition_btn  Ignition button state
 * @param arm_switch  Arming switch state
 * @param clacson_switch  Clacson switch state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_conrig_state_tc_send(mavlink_channel_t chan, uint8_t ox_filling_btn, uint8_t ox_release_btn, uint8_t ox_detach_btn, uint8_t ox_venting_btn, uint8_t n2_filling_btn, uint8_t n2_release_btn, uint8_t n2_detach_btn, uint8_t n2_quenching_btn, uint8_t n2_3way_switch, uint8_t tars_switch, uint8_t nitrogen_btn, uint8_t ignition_btn, uint8_t arm_switch, uint8_t clacson_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN];
    _mav_put_uint8_t(buf, 0, ox_filling_btn);
    _mav_put_uint8_t(buf, 1, ox_release_btn);
    _mav_put_uint8_t(buf, 2, ox_detach_btn);
    _mav_put_uint8_t(buf, 3, ox_venting_btn);
    _mav_put_uint8_t(buf, 4, n2_filling_btn);
    _mav_put_uint8_t(buf, 5, n2_release_btn);
    _mav_put_uint8_t(buf, 6, n2_detach_btn);
    _mav_put_uint8_t(buf, 7, n2_quenching_btn);
    _mav_put_uint8_t(buf, 8, n2_3way_switch);
    _mav_put_uint8_t(buf, 9, tars_switch);
    _mav_put_uint8_t(buf, 10, nitrogen_btn);
    _mav_put_uint8_t(buf, 11, ignition_btn);
    _mav_put_uint8_t(buf, 12, arm_switch);
    _mav_put_uint8_t(buf, 13, clacson_switch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONRIG_STATE_TC, buf, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
#else
    mavlink_conrig_state_tc_t packet;
    packet.ox_filling_btn = ox_filling_btn;
    packet.ox_release_btn = ox_release_btn;
    packet.ox_detach_btn = ox_detach_btn;
    packet.ox_venting_btn = ox_venting_btn;
    packet.n2_filling_btn = n2_filling_btn;
    packet.n2_release_btn = n2_release_btn;
    packet.n2_detach_btn = n2_detach_btn;
    packet.n2_quenching_btn = n2_quenching_btn;
    packet.n2_3way_switch = n2_3way_switch;
    packet.tars_switch = tars_switch;
    packet.nitrogen_btn = nitrogen_btn;
    packet.ignition_btn = ignition_btn;
    packet.arm_switch = arm_switch;
    packet.clacson_switch = clacson_switch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONRIG_STATE_TC, (const char *)&packet, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
#endif
}

/**
 * @brief Send a conrig_state_tc message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_conrig_state_tc_send_struct(mavlink_channel_t chan, const mavlink_conrig_state_tc_t* conrig_state_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_conrig_state_tc_send(chan, conrig_state_tc->ox_filling_btn, conrig_state_tc->ox_release_btn, conrig_state_tc->ox_detach_btn, conrig_state_tc->ox_venting_btn, conrig_state_tc->n2_filling_btn, conrig_state_tc->n2_release_btn, conrig_state_tc->n2_detach_btn, conrig_state_tc->n2_quenching_btn, conrig_state_tc->n2_3way_switch, conrig_state_tc->tars_switch, conrig_state_tc->nitrogen_btn, conrig_state_tc->ignition_btn, conrig_state_tc->arm_switch, conrig_state_tc->clacson_switch);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONRIG_STATE_TC, (const char *)conrig_state_tc, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_conrig_state_tc_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t ox_filling_btn, uint8_t ox_release_btn, uint8_t ox_detach_btn, uint8_t ox_venting_btn, uint8_t n2_filling_btn, uint8_t n2_release_btn, uint8_t n2_detach_btn, uint8_t n2_quenching_btn, uint8_t n2_3way_switch, uint8_t tars_switch, uint8_t nitrogen_btn, uint8_t ignition_btn, uint8_t arm_switch, uint8_t clacson_switch)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, ox_filling_btn);
    _mav_put_uint8_t(buf, 1, ox_release_btn);
    _mav_put_uint8_t(buf, 2, ox_detach_btn);
    _mav_put_uint8_t(buf, 3, ox_venting_btn);
    _mav_put_uint8_t(buf, 4, n2_filling_btn);
    _mav_put_uint8_t(buf, 5, n2_release_btn);
    _mav_put_uint8_t(buf, 6, n2_detach_btn);
    _mav_put_uint8_t(buf, 7, n2_quenching_btn);
    _mav_put_uint8_t(buf, 8, n2_3way_switch);
    _mav_put_uint8_t(buf, 9, tars_switch);
    _mav_put_uint8_t(buf, 10, nitrogen_btn);
    _mav_put_uint8_t(buf, 11, ignition_btn);
    _mav_put_uint8_t(buf, 12, arm_switch);
    _mav_put_uint8_t(buf, 13, clacson_switch);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONRIG_STATE_TC, buf, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
#else
    mavlink_conrig_state_tc_t *packet = (mavlink_conrig_state_tc_t *)msgbuf;
    packet->ox_filling_btn = ox_filling_btn;
    packet->ox_release_btn = ox_release_btn;
    packet->ox_detach_btn = ox_detach_btn;
    packet->ox_venting_btn = ox_venting_btn;
    packet->n2_filling_btn = n2_filling_btn;
    packet->n2_release_btn = n2_release_btn;
    packet->n2_detach_btn = n2_detach_btn;
    packet->n2_quenching_btn = n2_quenching_btn;
    packet->n2_3way_switch = n2_3way_switch;
    packet->tars_switch = tars_switch;
    packet->nitrogen_btn = nitrogen_btn;
    packet->ignition_btn = ignition_btn;
    packet->arm_switch = arm_switch;
    packet->clacson_switch = clacson_switch;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONRIG_STATE_TC, (const char *)packet, MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN, MAVLINK_MSG_ID_CONRIG_STATE_TC_CRC);
#endif
}
#endif

#endif

// MESSAGE CONRIG_STATE_TC UNPACKING


/**
 * @brief Get field ox_filling_btn from conrig_state_tc message
 *
 * @return  OX filling valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_ox_filling_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field ox_release_btn from conrig_state_tc message
 *
 * @return  OX release line pressure valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_ox_release_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field ox_detach_btn from conrig_state_tc message
 *
 * @return  OX quick connector detach button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_ox_detach_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field ox_venting_btn from conrig_state_tc message
 *
 * @return  OX venting valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_ox_venting_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field n2_filling_btn from conrig_state_tc message
 *
 * @return  N2 filling valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_n2_filling_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field n2_release_btn from conrig_state_tc message
 *
 * @return  N2 release line pressure valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_n2_release_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field n2_detach_btn from conrig_state_tc message
 *
 * @return  N2 quick connector detach button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_n2_detach_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field n2_quenching_btn from conrig_state_tc message
 *
 * @return  N2 quenching valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_n2_quenching_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field n2_3way_switch from conrig_state_tc message
 *
 * @return  N2 3-way valve switch state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_n2_3way_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field tars_switch from conrig_state_tc message
 *
 * @return  TARS switch state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_tars_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field nitrogen_btn from conrig_state_tc message
 *
 * @return  Nitrogen valve button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_nitrogen_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field ignition_btn from conrig_state_tc message
 *
 * @return  Ignition button state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_ignition_btn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field arm_switch from conrig_state_tc message
 *
 * @return  Arming switch state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_arm_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field clacson_switch from conrig_state_tc message
 *
 * @return  Clacson switch state
 */
static inline uint8_t mavlink_msg_conrig_state_tc_get_clacson_switch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Decode a conrig_state_tc message into a struct
 *
 * @param msg The message to decode
 * @param conrig_state_tc C-struct to decode the message contents into
 */
static inline void mavlink_msg_conrig_state_tc_decode(const mavlink_message_t* msg, mavlink_conrig_state_tc_t* conrig_state_tc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    conrig_state_tc->ox_filling_btn = mavlink_msg_conrig_state_tc_get_ox_filling_btn(msg);
    conrig_state_tc->ox_release_btn = mavlink_msg_conrig_state_tc_get_ox_release_btn(msg);
    conrig_state_tc->ox_detach_btn = mavlink_msg_conrig_state_tc_get_ox_detach_btn(msg);
    conrig_state_tc->ox_venting_btn = mavlink_msg_conrig_state_tc_get_ox_venting_btn(msg);
    conrig_state_tc->n2_filling_btn = mavlink_msg_conrig_state_tc_get_n2_filling_btn(msg);
    conrig_state_tc->n2_release_btn = mavlink_msg_conrig_state_tc_get_n2_release_btn(msg);
    conrig_state_tc->n2_detach_btn = mavlink_msg_conrig_state_tc_get_n2_detach_btn(msg);
    conrig_state_tc->n2_quenching_btn = mavlink_msg_conrig_state_tc_get_n2_quenching_btn(msg);
    conrig_state_tc->n2_3way_switch = mavlink_msg_conrig_state_tc_get_n2_3way_switch(msg);
    conrig_state_tc->tars_switch = mavlink_msg_conrig_state_tc_get_tars_switch(msg);
    conrig_state_tc->nitrogen_btn = mavlink_msg_conrig_state_tc_get_nitrogen_btn(msg);
    conrig_state_tc->ignition_btn = mavlink_msg_conrig_state_tc_get_ignition_btn(msg);
    conrig_state_tc->arm_switch = mavlink_msg_conrig_state_tc_get_arm_switch(msg);
    conrig_state_tc->clacson_switch = mavlink_msg_conrig_state_tc_get_clacson_switch(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN? msg->len : MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN;
        memset(conrig_state_tc, 0, MAVLINK_MSG_ID_CONRIG_STATE_TC_LEN);
    memcpy(conrig_state_tc, _MAV_PAYLOAD(msg), len);
#endif
}
