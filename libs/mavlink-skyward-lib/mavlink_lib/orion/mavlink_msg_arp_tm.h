#pragma once
// MESSAGE ARP_TM PACKING

#define MAVLINK_MSG_ID_ARP_TM 150


typedef struct __mavlink_arp_tm_t {
 uint64_t timestamp; /*< [us] Timestamp*/
 float yaw; /*< [deg] Current Yaw*/
 float pitch; /*< [deg] Current Pitch*/
 float roll; /*< [deg] Current Roll*/
 float target_yaw; /*< [deg] Target Yaw*/
 float target_pitch; /*< [deg] Target Pitch*/
 float stepperX_pos; /*< [deg] StepperX current position wrt the boot position*/
 float stepperX_delta; /*< [deg] StepperX last actuated delta angle*/
 float stepperX_speed; /*< [rps] StepperX current speed*/
 float stepperY_pos; /*< [deg] StepperY current position wrt the boot position*/
 float stepperY_delta; /*< [deg] StepperY last actuated delta angle*/
 float stepperY_speed; /*< [rps] StepperY current Speed*/
 float gps_latitude; /*< [deg] Latitude*/
 float gps_longitude; /*< [deg] Longitude*/
 float gps_height; /*< [m] Altitude*/
 float battery_voltage; /*< [V] Battery voltage*/
 int16_t log_number; /*<  Currently active log file, -1 if the logger is inactive*/
 uint8_t state; /*<  State Machine Controller State*/
 uint8_t gps_fix; /*<  Wether the GPS has a FIX*/
} mavlink_arp_tm_t;

#define MAVLINK_MSG_ID_ARP_TM_LEN 72
#define MAVLINK_MSG_ID_ARP_TM_MIN_LEN 72
#define MAVLINK_MSG_ID_150_LEN 72
#define MAVLINK_MSG_ID_150_MIN_LEN 72

#define MAVLINK_MSG_ID_ARP_TM_CRC 17
#define MAVLINK_MSG_ID_150_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARP_TM { \
    150, \
    "ARP_TM", \
    19, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_arp_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_arp_tm_t, state) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arp_tm_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arp_tm_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arp_tm_t, roll) }, \
         { "target_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_arp_tm_t, target_yaw) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_arp_tm_t, target_pitch) }, \
         { "stepperX_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_arp_tm_t, stepperX_pos) }, \
         { "stepperX_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_arp_tm_t, stepperX_delta) }, \
         { "stepperX_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_arp_tm_t, stepperX_speed) }, \
         { "stepperY_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_arp_tm_t, stepperY_pos) }, \
         { "stepperY_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_arp_tm_t, stepperY_delta) }, \
         { "stepperY_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_arp_tm_t, stepperY_speed) }, \
         { "gps_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_arp_tm_t, gps_latitude) }, \
         { "gps_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_arp_tm_t, gps_longitude) }, \
         { "gps_height", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_arp_tm_t, gps_height) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_arp_tm_t, gps_fix) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_arp_tm_t, battery_voltage) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 68, offsetof(mavlink_arp_tm_t, log_number) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARP_TM { \
    "ARP_TM", \
    19, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_arp_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_arp_tm_t, state) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_arp_tm_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_arp_tm_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_arp_tm_t, roll) }, \
         { "target_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_arp_tm_t, target_yaw) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_arp_tm_t, target_pitch) }, \
         { "stepperX_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_arp_tm_t, stepperX_pos) }, \
         { "stepperX_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_arp_tm_t, stepperX_delta) }, \
         { "stepperX_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_arp_tm_t, stepperX_speed) }, \
         { "stepperY_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_arp_tm_t, stepperY_pos) }, \
         { "stepperY_delta", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_arp_tm_t, stepperY_delta) }, \
         { "stepperY_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_arp_tm_t, stepperY_speed) }, \
         { "gps_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_arp_tm_t, gps_latitude) }, \
         { "gps_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_arp_tm_t, gps_longitude) }, \
         { "gps_height", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_arp_tm_t, gps_height) }, \
         { "gps_fix", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_arp_tm_t, gps_fix) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_arp_tm_t, battery_voltage) }, \
         { "log_number", NULL, MAVLINK_TYPE_INT16_T, 0, 68, offsetof(mavlink_arp_tm_t, log_number) }, \
         } \
}
#endif

/**
 * @brief Pack a arp_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp
 * @param state  State Machine Controller State
 * @param yaw [deg] Current Yaw
 * @param pitch [deg] Current Pitch
 * @param roll [deg] Current Roll
 * @param target_yaw [deg] Target Yaw
 * @param target_pitch [deg] Target Pitch
 * @param stepperX_pos [deg] StepperX current position wrt the boot position
 * @param stepperX_delta [deg] StepperX last actuated delta angle
 * @param stepperX_speed [rps] StepperX current speed
 * @param stepperY_pos [deg] StepperY current position wrt the boot position
 * @param stepperY_delta [deg] StepperY last actuated delta angle
 * @param stepperY_speed [rps] StepperY current Speed
 * @param gps_latitude [deg] Latitude
 * @param gps_longitude [deg] Longitude
 * @param gps_height [m] Altitude
 * @param gps_fix  Wether the GPS has a FIX
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arp_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float yaw, float pitch, float roll, float target_yaw, float target_pitch, float stepperX_pos, float stepperX_delta, float stepperX_speed, float stepperY_pos, float stepperY_delta, float stepperY_speed, float gps_latitude, float gps_longitude, float gps_height, uint8_t gps_fix, float battery_voltage, int16_t log_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARP_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, target_yaw);
    _mav_put_float(buf, 24, target_pitch);
    _mav_put_float(buf, 28, stepperX_pos);
    _mav_put_float(buf, 32, stepperX_delta);
    _mav_put_float(buf, 36, stepperX_speed);
    _mav_put_float(buf, 40, stepperY_pos);
    _mav_put_float(buf, 44, stepperY_delta);
    _mav_put_float(buf, 48, stepperY_speed);
    _mav_put_float(buf, 52, gps_latitude);
    _mav_put_float(buf, 56, gps_longitude);
    _mav_put_float(buf, 60, gps_height);
    _mav_put_float(buf, 64, battery_voltage);
    _mav_put_int16_t(buf, 68, log_number);
    _mav_put_uint8_t(buf, 70, state);
    _mav_put_uint8_t(buf, 71, gps_fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARP_TM_LEN);
#else
    mavlink_arp_tm_t packet;
    packet.timestamp = timestamp;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.target_yaw = target_yaw;
    packet.target_pitch = target_pitch;
    packet.stepperX_pos = stepperX_pos;
    packet.stepperX_delta = stepperX_delta;
    packet.stepperX_speed = stepperX_speed;
    packet.stepperY_pos = stepperY_pos;
    packet.stepperY_delta = stepperY_delta;
    packet.stepperY_speed = stepperY_speed;
    packet.gps_latitude = gps_latitude;
    packet.gps_longitude = gps_longitude;
    packet.gps_height = gps_height;
    packet.battery_voltage = battery_voltage;
    packet.log_number = log_number;
    packet.state = state;
    packet.gps_fix = gps_fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARP_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARP_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
}

/**
 * @brief Pack a arp_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp
 * @param state  State Machine Controller State
 * @param yaw [deg] Current Yaw
 * @param pitch [deg] Current Pitch
 * @param roll [deg] Current Roll
 * @param target_yaw [deg] Target Yaw
 * @param target_pitch [deg] Target Pitch
 * @param stepperX_pos [deg] StepperX current position wrt the boot position
 * @param stepperX_delta [deg] StepperX last actuated delta angle
 * @param stepperX_speed [rps] StepperX current speed
 * @param stepperY_pos [deg] StepperY current position wrt the boot position
 * @param stepperY_delta [deg] StepperY last actuated delta angle
 * @param stepperY_speed [rps] StepperY current Speed
 * @param gps_latitude [deg] Latitude
 * @param gps_longitude [deg] Longitude
 * @param gps_height [m] Altitude
 * @param gps_fix  Wether the GPS has a FIX
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arp_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float yaw,float pitch,float roll,float target_yaw,float target_pitch,float stepperX_pos,float stepperX_delta,float stepperX_speed,float stepperY_pos,float stepperY_delta,float stepperY_speed,float gps_latitude,float gps_longitude,float gps_height,uint8_t gps_fix,float battery_voltage,int16_t log_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARP_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, target_yaw);
    _mav_put_float(buf, 24, target_pitch);
    _mav_put_float(buf, 28, stepperX_pos);
    _mav_put_float(buf, 32, stepperX_delta);
    _mav_put_float(buf, 36, stepperX_speed);
    _mav_put_float(buf, 40, stepperY_pos);
    _mav_put_float(buf, 44, stepperY_delta);
    _mav_put_float(buf, 48, stepperY_speed);
    _mav_put_float(buf, 52, gps_latitude);
    _mav_put_float(buf, 56, gps_longitude);
    _mav_put_float(buf, 60, gps_height);
    _mav_put_float(buf, 64, battery_voltage);
    _mav_put_int16_t(buf, 68, log_number);
    _mav_put_uint8_t(buf, 70, state);
    _mav_put_uint8_t(buf, 71, gps_fix);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARP_TM_LEN);
#else
    mavlink_arp_tm_t packet;
    packet.timestamp = timestamp;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.target_yaw = target_yaw;
    packet.target_pitch = target_pitch;
    packet.stepperX_pos = stepperX_pos;
    packet.stepperX_delta = stepperX_delta;
    packet.stepperX_speed = stepperX_speed;
    packet.stepperY_pos = stepperY_pos;
    packet.stepperY_delta = stepperY_delta;
    packet.stepperY_speed = stepperY_speed;
    packet.gps_latitude = gps_latitude;
    packet.gps_longitude = gps_longitude;
    packet.gps_height = gps_height;
    packet.battery_voltage = battery_voltage;
    packet.log_number = log_number;
    packet.state = state;
    packet.gps_fix = gps_fix;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARP_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARP_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
}

/**
 * @brief Encode a arp_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arp_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arp_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arp_tm_t* arp_tm)
{
    return mavlink_msg_arp_tm_pack(system_id, component_id, msg, arp_tm->timestamp, arp_tm->state, arp_tm->yaw, arp_tm->pitch, arp_tm->roll, arp_tm->target_yaw, arp_tm->target_pitch, arp_tm->stepperX_pos, arp_tm->stepperX_delta, arp_tm->stepperX_speed, arp_tm->stepperY_pos, arp_tm->stepperY_delta, arp_tm->stepperY_speed, arp_tm->gps_latitude, arp_tm->gps_longitude, arp_tm->gps_height, arp_tm->gps_fix, arp_tm->battery_voltage, arp_tm->log_number);
}

/**
 * @brief Encode a arp_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arp_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arp_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arp_tm_t* arp_tm)
{
    return mavlink_msg_arp_tm_pack_chan(system_id, component_id, chan, msg, arp_tm->timestamp, arp_tm->state, arp_tm->yaw, arp_tm->pitch, arp_tm->roll, arp_tm->target_yaw, arp_tm->target_pitch, arp_tm->stepperX_pos, arp_tm->stepperX_delta, arp_tm->stepperX_speed, arp_tm->stepperY_pos, arp_tm->stepperY_delta, arp_tm->stepperY_speed, arp_tm->gps_latitude, arp_tm->gps_longitude, arp_tm->gps_height, arp_tm->gps_fix, arp_tm->battery_voltage, arp_tm->log_number);
}

/**
 * @brief Send a arp_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp
 * @param state  State Machine Controller State
 * @param yaw [deg] Current Yaw
 * @param pitch [deg] Current Pitch
 * @param roll [deg] Current Roll
 * @param target_yaw [deg] Target Yaw
 * @param target_pitch [deg] Target Pitch
 * @param stepperX_pos [deg] StepperX current position wrt the boot position
 * @param stepperX_delta [deg] StepperX last actuated delta angle
 * @param stepperX_speed [rps] StepperX current speed
 * @param stepperY_pos [deg] StepperY current position wrt the boot position
 * @param stepperY_delta [deg] StepperY last actuated delta angle
 * @param stepperY_speed [rps] StepperY current Speed
 * @param gps_latitude [deg] Latitude
 * @param gps_longitude [deg] Longitude
 * @param gps_height [m] Altitude
 * @param gps_fix  Wether the GPS has a FIX
 * @param battery_voltage [V] Battery voltage
 * @param log_number  Currently active log file, -1 if the logger is inactive
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arp_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float yaw, float pitch, float roll, float target_yaw, float target_pitch, float stepperX_pos, float stepperX_delta, float stepperX_speed, float stepperY_pos, float stepperY_delta, float stepperY_speed, float gps_latitude, float gps_longitude, float gps_height, uint8_t gps_fix, float battery_voltage, int16_t log_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARP_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, target_yaw);
    _mav_put_float(buf, 24, target_pitch);
    _mav_put_float(buf, 28, stepperX_pos);
    _mav_put_float(buf, 32, stepperX_delta);
    _mav_put_float(buf, 36, stepperX_speed);
    _mav_put_float(buf, 40, stepperY_pos);
    _mav_put_float(buf, 44, stepperY_delta);
    _mav_put_float(buf, 48, stepperY_speed);
    _mav_put_float(buf, 52, gps_latitude);
    _mav_put_float(buf, 56, gps_longitude);
    _mav_put_float(buf, 60, gps_height);
    _mav_put_float(buf, 64, battery_voltage);
    _mav_put_int16_t(buf, 68, log_number);
    _mav_put_uint8_t(buf, 70, state);
    _mav_put_uint8_t(buf, 71, gps_fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARP_TM, buf, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
#else
    mavlink_arp_tm_t packet;
    packet.timestamp = timestamp;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.target_yaw = target_yaw;
    packet.target_pitch = target_pitch;
    packet.stepperX_pos = stepperX_pos;
    packet.stepperX_delta = stepperX_delta;
    packet.stepperX_speed = stepperX_speed;
    packet.stepperY_pos = stepperY_pos;
    packet.stepperY_delta = stepperY_delta;
    packet.stepperY_speed = stepperY_speed;
    packet.gps_latitude = gps_latitude;
    packet.gps_longitude = gps_longitude;
    packet.gps_height = gps_height;
    packet.battery_voltage = battery_voltage;
    packet.log_number = log_number;
    packet.state = state;
    packet.gps_fix = gps_fix;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARP_TM, (const char *)&packet, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
#endif
}

/**
 * @brief Send a arp_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_arp_tm_send_struct(mavlink_channel_t chan, const mavlink_arp_tm_t* arp_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_arp_tm_send(chan, arp_tm->timestamp, arp_tm->state, arp_tm->yaw, arp_tm->pitch, arp_tm->roll, arp_tm->target_yaw, arp_tm->target_pitch, arp_tm->stepperX_pos, arp_tm->stepperX_delta, arp_tm->stepperX_speed, arp_tm->stepperY_pos, arp_tm->stepperY_delta, arp_tm->stepperY_speed, arp_tm->gps_latitude, arp_tm->gps_longitude, arp_tm->gps_height, arp_tm->gps_fix, arp_tm->battery_voltage, arp_tm->log_number);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARP_TM, (const char *)arp_tm, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARP_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arp_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float yaw, float pitch, float roll, float target_yaw, float target_pitch, float stepperX_pos, float stepperX_delta, float stepperX_speed, float stepperY_pos, float stepperY_delta, float stepperY_speed, float gps_latitude, float gps_longitude, float gps_height, uint8_t gps_fix, float battery_voltage, int16_t log_number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, pitch);
    _mav_put_float(buf, 16, roll);
    _mav_put_float(buf, 20, target_yaw);
    _mav_put_float(buf, 24, target_pitch);
    _mav_put_float(buf, 28, stepperX_pos);
    _mav_put_float(buf, 32, stepperX_delta);
    _mav_put_float(buf, 36, stepperX_speed);
    _mav_put_float(buf, 40, stepperY_pos);
    _mav_put_float(buf, 44, stepperY_delta);
    _mav_put_float(buf, 48, stepperY_speed);
    _mav_put_float(buf, 52, gps_latitude);
    _mav_put_float(buf, 56, gps_longitude);
    _mav_put_float(buf, 60, gps_height);
    _mav_put_float(buf, 64, battery_voltage);
    _mav_put_int16_t(buf, 68, log_number);
    _mav_put_uint8_t(buf, 70, state);
    _mav_put_uint8_t(buf, 71, gps_fix);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARP_TM, buf, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
#else
    mavlink_arp_tm_t *packet = (mavlink_arp_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->target_yaw = target_yaw;
    packet->target_pitch = target_pitch;
    packet->stepperX_pos = stepperX_pos;
    packet->stepperX_delta = stepperX_delta;
    packet->stepperX_speed = stepperX_speed;
    packet->stepperY_pos = stepperY_pos;
    packet->stepperY_delta = stepperY_delta;
    packet->stepperY_speed = stepperY_speed;
    packet->gps_latitude = gps_latitude;
    packet->gps_longitude = gps_longitude;
    packet->gps_height = gps_height;
    packet->battery_voltage = battery_voltage;
    packet->log_number = log_number;
    packet->state = state;
    packet->gps_fix = gps_fix;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARP_TM, (const char *)packet, MAVLINK_MSG_ID_ARP_TM_MIN_LEN, MAVLINK_MSG_ID_ARP_TM_LEN, MAVLINK_MSG_ID_ARP_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ARP_TM UNPACKING


/**
 * @brief Get field timestamp from arp_tm message
 *
 * @return [us] Timestamp
 */
static inline uint64_t mavlink_msg_arp_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from arp_tm message
 *
 * @return  State Machine Controller State
 */
static inline uint8_t mavlink_msg_arp_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field yaw from arp_tm message
 *
 * @return [deg] Current Yaw
 */
static inline float mavlink_msg_arp_tm_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pitch from arp_tm message
 *
 * @return [deg] Current Pitch
 */
static inline float mavlink_msg_arp_tm_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field roll from arp_tm message
 *
 * @return [deg] Current Roll
 */
static inline float mavlink_msg_arp_tm_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field target_yaw from arp_tm message
 *
 * @return [deg] Target Yaw
 */
static inline float mavlink_msg_arp_tm_get_target_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field target_pitch from arp_tm message
 *
 * @return [deg] Target Pitch
 */
static inline float mavlink_msg_arp_tm_get_target_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field stepperX_pos from arp_tm message
 *
 * @return [deg] StepperX current position wrt the boot position
 */
static inline float mavlink_msg_arp_tm_get_stepperX_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field stepperX_delta from arp_tm message
 *
 * @return [deg] StepperX last actuated delta angle
 */
static inline float mavlink_msg_arp_tm_get_stepperX_delta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field stepperX_speed from arp_tm message
 *
 * @return [rps] StepperX current speed
 */
static inline float mavlink_msg_arp_tm_get_stepperX_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field stepperY_pos from arp_tm message
 *
 * @return [deg] StepperY current position wrt the boot position
 */
static inline float mavlink_msg_arp_tm_get_stepperY_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field stepperY_delta from arp_tm message
 *
 * @return [deg] StepperY last actuated delta angle
 */
static inline float mavlink_msg_arp_tm_get_stepperY_delta(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field stepperY_speed from arp_tm message
 *
 * @return [rps] StepperY current Speed
 */
static inline float mavlink_msg_arp_tm_get_stepperY_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field gps_latitude from arp_tm message
 *
 * @return [deg] Latitude
 */
static inline float mavlink_msg_arp_tm_get_gps_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field gps_longitude from arp_tm message
 *
 * @return [deg] Longitude
 */
static inline float mavlink_msg_arp_tm_get_gps_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field gps_height from arp_tm message
 *
 * @return [m] Altitude
 */
static inline float mavlink_msg_arp_tm_get_gps_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field gps_fix from arp_tm message
 *
 * @return  Wether the GPS has a FIX
 */
static inline uint8_t mavlink_msg_arp_tm_get_gps_fix(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  71);
}

/**
 * @brief Get field battery_voltage from arp_tm message
 *
 * @return [V] Battery voltage
 */
static inline float mavlink_msg_arp_tm_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field log_number from arp_tm message
 *
 * @return  Currently active log file, -1 if the logger is inactive
 */
static inline int16_t mavlink_msg_arp_tm_get_log_number(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  68);
}

/**
 * @brief Decode a arp_tm message into a struct
 *
 * @param msg The message to decode
 * @param arp_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_arp_tm_decode(const mavlink_message_t* msg, mavlink_arp_tm_t* arp_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    arp_tm->timestamp = mavlink_msg_arp_tm_get_timestamp(msg);
    arp_tm->yaw = mavlink_msg_arp_tm_get_yaw(msg);
    arp_tm->pitch = mavlink_msg_arp_tm_get_pitch(msg);
    arp_tm->roll = mavlink_msg_arp_tm_get_roll(msg);
    arp_tm->target_yaw = mavlink_msg_arp_tm_get_target_yaw(msg);
    arp_tm->target_pitch = mavlink_msg_arp_tm_get_target_pitch(msg);
    arp_tm->stepperX_pos = mavlink_msg_arp_tm_get_stepperX_pos(msg);
    arp_tm->stepperX_delta = mavlink_msg_arp_tm_get_stepperX_delta(msg);
    arp_tm->stepperX_speed = mavlink_msg_arp_tm_get_stepperX_speed(msg);
    arp_tm->stepperY_pos = mavlink_msg_arp_tm_get_stepperY_pos(msg);
    arp_tm->stepperY_delta = mavlink_msg_arp_tm_get_stepperY_delta(msg);
    arp_tm->stepperY_speed = mavlink_msg_arp_tm_get_stepperY_speed(msg);
    arp_tm->gps_latitude = mavlink_msg_arp_tm_get_gps_latitude(msg);
    arp_tm->gps_longitude = mavlink_msg_arp_tm_get_gps_longitude(msg);
    arp_tm->gps_height = mavlink_msg_arp_tm_get_gps_height(msg);
    arp_tm->battery_voltage = mavlink_msg_arp_tm_get_battery_voltage(msg);
    arp_tm->log_number = mavlink_msg_arp_tm_get_log_number(msg);
    arp_tm->state = mavlink_msg_arp_tm_get_state(msg);
    arp_tm->gps_fix = mavlink_msg_arp_tm_get_gps_fix(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARP_TM_LEN? msg->len : MAVLINK_MSG_ID_ARP_TM_LEN;
        memset(arp_tm, 0, MAVLINK_MSG_ID_ARP_TM_LEN);
    memcpy(arp_tm, _MAV_PAYLOAD(msg), len);
#endif
}
