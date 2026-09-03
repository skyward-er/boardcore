#pragma once
// MESSAGE NAS_TM PACKING

#define MAVLINK_MSG_ID_NAS_TM 206


typedef struct __mavlink_nas_tm_t {
 uint64_t timestamp; /*< [us] When was this logged*/
 float nas_n; /*< [deg] Navigation system estimated noth position*/
 float nas_e; /*< [deg] Navigation system estimated east position*/
 float nas_d; /*< [m] Navigation system estimated down position*/
 float nas_vn; /*< [m/s] Navigation system estimated north velocity*/
 float nas_ve; /*< [m/s] Navigation system estimated east velocity*/
 float nas_vd; /*< [m/s] Navigation system estimated down velocity*/
 float nas_qx; /*< [deg] Navigation system estimated attitude (qx)*/
 float nas_qy; /*< [deg] Navigation system estimated attitude (qy)*/
 float nas_qz; /*< [deg] Navigation system estimated attitude (qz)*/
 float nas_qw; /*< [deg] Navigation system estimated attitude (qw)*/
 float nas_bias_x; /*<  Navigation system gyroscope bias on x axis*/
 float nas_bias_y; /*<  Navigation system gyroscope bias on x axis*/
 float nas_bias_z; /*<  Navigation system gyroscope bias on x axis*/
 float ref_pressure; /*< [Pa] Calibration pressure*/
 float ref_temperature; /*< [degC] Calibration temperature*/
 float ref_latitude; /*< [deg] Calibration latitude*/
 float ref_longitude; /*< [deg] Calibration longitude*/
 uint8_t state; /*<  NAS current state*/
} mavlink_nas_tm_t;

#define MAVLINK_MSG_ID_NAS_TM_LEN 77
#define MAVLINK_MSG_ID_NAS_TM_MIN_LEN 77
#define MAVLINK_MSG_ID_206_LEN 77
#define MAVLINK_MSG_ID_206_MIN_LEN 77

#define MAVLINK_MSG_ID_NAS_TM_CRC 66
#define MAVLINK_MSG_ID_206_CRC 66



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAS_TM { \
    206, \
    "NAS_TM", \
    19, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nas_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_nas_tm_t, state) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nas_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nas_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nas_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nas_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nas_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nas_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_nas_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_nas_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_nas_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_nas_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_nas_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_nas_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_nas_tm_t, nas_bias_z) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_nas_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_nas_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_nas_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_nas_tm_t, ref_longitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAS_TM { \
    "NAS_TM", \
    19, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_nas_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_nas_tm_t, state) }, \
         { "nas_n", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_nas_tm_t, nas_n) }, \
         { "nas_e", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_nas_tm_t, nas_e) }, \
         { "nas_d", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_nas_tm_t, nas_d) }, \
         { "nas_vn", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_nas_tm_t, nas_vn) }, \
         { "nas_ve", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_nas_tm_t, nas_ve) }, \
         { "nas_vd", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_nas_tm_t, nas_vd) }, \
         { "nas_qx", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_nas_tm_t, nas_qx) }, \
         { "nas_qy", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_nas_tm_t, nas_qy) }, \
         { "nas_qz", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_nas_tm_t, nas_qz) }, \
         { "nas_qw", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_nas_tm_t, nas_qw) }, \
         { "nas_bias_x", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_nas_tm_t, nas_bias_x) }, \
         { "nas_bias_y", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_nas_tm_t, nas_bias_y) }, \
         { "nas_bias_z", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_nas_tm_t, nas_bias_z) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_nas_tm_t, ref_pressure) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_nas_tm_t, ref_temperature) }, \
         { "ref_latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_nas_tm_t, ref_latitude) }, \
         { "ref_longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_nas_tm_t, ref_longitude) }, \
         } \
}
#endif

/**
 * @brief Pack a nas_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] When was this logged
 * @param state  NAS current state
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nas_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, nas_n);
    _mav_put_float(buf, 12, nas_e);
    _mav_put_float(buf, 16, nas_d);
    _mav_put_float(buf, 20, nas_vn);
    _mav_put_float(buf, 24, nas_ve);
    _mav_put_float(buf, 28, nas_vd);
    _mav_put_float(buf, 32, nas_qx);
    _mav_put_float(buf, 36, nas_qy);
    _mav_put_float(buf, 40, nas_qz);
    _mav_put_float(buf, 44, nas_qw);
    _mav_put_float(buf, 48, nas_bias_x);
    _mav_put_float(buf, 52, nas_bias_y);
    _mav_put_float(buf, 56, nas_bias_z);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_uint8_t(buf, 76, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAS_TM_LEN);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
}

/**
 * @brief Pack a nas_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] When was this logged
 * @param state  NAS current state
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_nas_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float nas_n,float nas_e,float nas_d,float nas_vn,float nas_ve,float nas_vd,float nas_qx,float nas_qy,float nas_qz,float nas_qw,float nas_bias_x,float nas_bias_y,float nas_bias_z,float ref_pressure,float ref_temperature,float ref_latitude,float ref_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, nas_n);
    _mav_put_float(buf, 12, nas_e);
    _mav_put_float(buf, 16, nas_d);
    _mav_put_float(buf, 20, nas_vn);
    _mav_put_float(buf, 24, nas_ve);
    _mav_put_float(buf, 28, nas_vd);
    _mav_put_float(buf, 32, nas_qx);
    _mav_put_float(buf, 36, nas_qy);
    _mav_put_float(buf, 40, nas_qz);
    _mav_put_float(buf, 44, nas_qw);
    _mav_put_float(buf, 48, nas_bias_x);
    _mav_put_float(buf, 52, nas_bias_y);
    _mav_put_float(buf, 56, nas_bias_z);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_uint8_t(buf, 76, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAS_TM_LEN);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
}

/**
 * @brief Encode a nas_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param nas_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nas_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_nas_tm_t* nas_tm)
{
    return mavlink_msg_nas_tm_pack(system_id, component_id, msg, nas_tm->timestamp, nas_tm->state, nas_tm->nas_n, nas_tm->nas_e, nas_tm->nas_d, nas_tm->nas_vn, nas_tm->nas_ve, nas_tm->nas_vd, nas_tm->nas_qx, nas_tm->nas_qy, nas_tm->nas_qz, nas_tm->nas_qw, nas_tm->nas_bias_x, nas_tm->nas_bias_y, nas_tm->nas_bias_z, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude);
}

/**
 * @brief Encode a nas_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nas_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_nas_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_nas_tm_t* nas_tm)
{
    return mavlink_msg_nas_tm_pack_chan(system_id, component_id, chan, msg, nas_tm->timestamp, nas_tm->state, nas_tm->nas_n, nas_tm->nas_e, nas_tm->nas_d, nas_tm->nas_vn, nas_tm->nas_ve, nas_tm->nas_vd, nas_tm->nas_qx, nas_tm->nas_qy, nas_tm->nas_qz, nas_tm->nas_qw, nas_tm->nas_bias_x, nas_tm->nas_bias_y, nas_tm->nas_bias_z, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude);
}

/**
 * @brief Send a nas_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] When was this logged
 * @param state  NAS current state
 * @param nas_n [deg] Navigation system estimated noth position
 * @param nas_e [deg] Navigation system estimated east position
 * @param nas_d [m] Navigation system estimated down position
 * @param nas_vn [m/s] Navigation system estimated north velocity
 * @param nas_ve [m/s] Navigation system estimated east velocity
 * @param nas_vd [m/s] Navigation system estimated down velocity
 * @param nas_qx [deg] Navigation system estimated attitude (qx)
 * @param nas_qy [deg] Navigation system estimated attitude (qy)
 * @param nas_qz [deg] Navigation system estimated attitude (qz)
 * @param nas_qw [deg] Navigation system estimated attitude (qw)
 * @param nas_bias_x  Navigation system gyroscope bias on x axis
 * @param nas_bias_y  Navigation system gyroscope bias on x axis
 * @param nas_bias_z  Navigation system gyroscope bias on x axis
 * @param ref_pressure [Pa] Calibration pressure
 * @param ref_temperature [degC] Calibration temperature
 * @param ref_latitude [deg] Calibration latitude
 * @param ref_longitude [deg] Calibration longitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_nas_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, nas_n);
    _mav_put_float(buf, 12, nas_e);
    _mav_put_float(buf, 16, nas_d);
    _mav_put_float(buf, 20, nas_vn);
    _mav_put_float(buf, 24, nas_ve);
    _mav_put_float(buf, 28, nas_vd);
    _mav_put_float(buf, 32, nas_qx);
    _mav_put_float(buf, 36, nas_qy);
    _mav_put_float(buf, 40, nas_qz);
    _mav_put_float(buf, 44, nas_qw);
    _mav_put_float(buf, 48, nas_bias_x);
    _mav_put_float(buf, 52, nas_bias_y);
    _mav_put_float(buf, 56, nas_bias_z);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_uint8_t(buf, 76, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, buf, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#else
    mavlink_nas_tm_t packet;
    packet.timestamp = timestamp;
    packet.nas_n = nas_n;
    packet.nas_e = nas_e;
    packet.nas_d = nas_d;
    packet.nas_vn = nas_vn;
    packet.nas_ve = nas_ve;
    packet.nas_vd = nas_vd;
    packet.nas_qx = nas_qx;
    packet.nas_qy = nas_qy;
    packet.nas_qz = nas_qz;
    packet.nas_qw = nas_qw;
    packet.nas_bias_x = nas_bias_x;
    packet.nas_bias_y = nas_bias_y;
    packet.nas_bias_z = nas_bias_z;
    packet.ref_pressure = ref_pressure;
    packet.ref_temperature = ref_temperature;
    packet.ref_latitude = ref_latitude;
    packet.ref_longitude = ref_longitude;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)&packet, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}

/**
 * @brief Send a nas_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_nas_tm_send_struct(mavlink_channel_t chan, const mavlink_nas_tm_t* nas_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_nas_tm_send(chan, nas_tm->timestamp, nas_tm->state, nas_tm->nas_n, nas_tm->nas_e, nas_tm->nas_d, nas_tm->nas_vn, nas_tm->nas_ve, nas_tm->nas_vd, nas_tm->nas_qx, nas_tm->nas_qy, nas_tm->nas_qz, nas_tm->nas_qw, nas_tm->nas_bias_x, nas_tm->nas_bias_y, nas_tm->nas_bias_z, nas_tm->ref_pressure, nas_tm->ref_temperature, nas_tm->ref_latitude, nas_tm->ref_longitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)nas_tm, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_nas_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float nas_n, float nas_e, float nas_d, float nas_vn, float nas_ve, float nas_vd, float nas_qx, float nas_qy, float nas_qz, float nas_qw, float nas_bias_x, float nas_bias_y, float nas_bias_z, float ref_pressure, float ref_temperature, float ref_latitude, float ref_longitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, nas_n);
    _mav_put_float(buf, 12, nas_e);
    _mav_put_float(buf, 16, nas_d);
    _mav_put_float(buf, 20, nas_vn);
    _mav_put_float(buf, 24, nas_ve);
    _mav_put_float(buf, 28, nas_vd);
    _mav_put_float(buf, 32, nas_qx);
    _mav_put_float(buf, 36, nas_qy);
    _mav_put_float(buf, 40, nas_qz);
    _mav_put_float(buf, 44, nas_qw);
    _mav_put_float(buf, 48, nas_bias_x);
    _mav_put_float(buf, 52, nas_bias_y);
    _mav_put_float(buf, 56, nas_bias_z);
    _mav_put_float(buf, 60, ref_pressure);
    _mav_put_float(buf, 64, ref_temperature);
    _mav_put_float(buf, 68, ref_latitude);
    _mav_put_float(buf, 72, ref_longitude);
    _mav_put_uint8_t(buf, 76, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, buf, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#else
    mavlink_nas_tm_t *packet = (mavlink_nas_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->nas_n = nas_n;
    packet->nas_e = nas_e;
    packet->nas_d = nas_d;
    packet->nas_vn = nas_vn;
    packet->nas_ve = nas_ve;
    packet->nas_vd = nas_vd;
    packet->nas_qx = nas_qx;
    packet->nas_qy = nas_qy;
    packet->nas_qz = nas_qz;
    packet->nas_qw = nas_qw;
    packet->nas_bias_x = nas_bias_x;
    packet->nas_bias_y = nas_bias_y;
    packet->nas_bias_z = nas_bias_z;
    packet->ref_pressure = ref_pressure;
    packet->ref_temperature = ref_temperature;
    packet->ref_latitude = ref_latitude;
    packet->ref_longitude = ref_longitude;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAS_TM, (const char *)packet, MAVLINK_MSG_ID_NAS_TM_MIN_LEN, MAVLINK_MSG_ID_NAS_TM_LEN, MAVLINK_MSG_ID_NAS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE NAS_TM UNPACKING


/**
 * @brief Get field timestamp from nas_tm message
 *
 * @return [us] When was this logged
 */
static inline uint64_t mavlink_msg_nas_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from nas_tm message
 *
 * @return  NAS current state
 */
static inline uint8_t mavlink_msg_nas_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field nas_n from nas_tm message
 *
 * @return [deg] Navigation system estimated noth position
 */
static inline float mavlink_msg_nas_tm_get_nas_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field nas_e from nas_tm message
 *
 * @return [deg] Navigation system estimated east position
 */
static inline float mavlink_msg_nas_tm_get_nas_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field nas_d from nas_tm message
 *
 * @return [m] Navigation system estimated down position
 */
static inline float mavlink_msg_nas_tm_get_nas_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field nas_vn from nas_tm message
 *
 * @return [m/s] Navigation system estimated north velocity
 */
static inline float mavlink_msg_nas_tm_get_nas_vn(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field nas_ve from nas_tm message
 *
 * @return [m/s] Navigation system estimated east velocity
 */
static inline float mavlink_msg_nas_tm_get_nas_ve(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field nas_vd from nas_tm message
 *
 * @return [m/s] Navigation system estimated down velocity
 */
static inline float mavlink_msg_nas_tm_get_nas_vd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field nas_qx from nas_tm message
 *
 * @return [deg] Navigation system estimated attitude (qx)
 */
static inline float mavlink_msg_nas_tm_get_nas_qx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field nas_qy from nas_tm message
 *
 * @return [deg] Navigation system estimated attitude (qy)
 */
static inline float mavlink_msg_nas_tm_get_nas_qy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field nas_qz from nas_tm message
 *
 * @return [deg] Navigation system estimated attitude (qz)
 */
static inline float mavlink_msg_nas_tm_get_nas_qz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field nas_qw from nas_tm message
 *
 * @return [deg] Navigation system estimated attitude (qw)
 */
static inline float mavlink_msg_nas_tm_get_nas_qw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field nas_bias_x from nas_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_nas_tm_get_nas_bias_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field nas_bias_y from nas_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_nas_tm_get_nas_bias_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field nas_bias_z from nas_tm message
 *
 * @return  Navigation system gyroscope bias on x axis
 */
static inline float mavlink_msg_nas_tm_get_nas_bias_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field ref_pressure from nas_tm message
 *
 * @return [Pa] Calibration pressure
 */
static inline float mavlink_msg_nas_tm_get_ref_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field ref_temperature from nas_tm message
 *
 * @return [degC] Calibration temperature
 */
static inline float mavlink_msg_nas_tm_get_ref_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field ref_latitude from nas_tm message
 *
 * @return [deg] Calibration latitude
 */
static inline float mavlink_msg_nas_tm_get_ref_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field ref_longitude from nas_tm message
 *
 * @return [deg] Calibration longitude
 */
static inline float mavlink_msg_nas_tm_get_ref_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Decode a nas_tm message into a struct
 *
 * @param msg The message to decode
 * @param nas_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_nas_tm_decode(const mavlink_message_t* msg, mavlink_nas_tm_t* nas_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    nas_tm->timestamp = mavlink_msg_nas_tm_get_timestamp(msg);
    nas_tm->nas_n = mavlink_msg_nas_tm_get_nas_n(msg);
    nas_tm->nas_e = mavlink_msg_nas_tm_get_nas_e(msg);
    nas_tm->nas_d = mavlink_msg_nas_tm_get_nas_d(msg);
    nas_tm->nas_vn = mavlink_msg_nas_tm_get_nas_vn(msg);
    nas_tm->nas_ve = mavlink_msg_nas_tm_get_nas_ve(msg);
    nas_tm->nas_vd = mavlink_msg_nas_tm_get_nas_vd(msg);
    nas_tm->nas_qx = mavlink_msg_nas_tm_get_nas_qx(msg);
    nas_tm->nas_qy = mavlink_msg_nas_tm_get_nas_qy(msg);
    nas_tm->nas_qz = mavlink_msg_nas_tm_get_nas_qz(msg);
    nas_tm->nas_qw = mavlink_msg_nas_tm_get_nas_qw(msg);
    nas_tm->nas_bias_x = mavlink_msg_nas_tm_get_nas_bias_x(msg);
    nas_tm->nas_bias_y = mavlink_msg_nas_tm_get_nas_bias_y(msg);
    nas_tm->nas_bias_z = mavlink_msg_nas_tm_get_nas_bias_z(msg);
    nas_tm->ref_pressure = mavlink_msg_nas_tm_get_ref_pressure(msg);
    nas_tm->ref_temperature = mavlink_msg_nas_tm_get_ref_temperature(msg);
    nas_tm->ref_latitude = mavlink_msg_nas_tm_get_ref_latitude(msg);
    nas_tm->ref_longitude = mavlink_msg_nas_tm_get_ref_longitude(msg);
    nas_tm->state = mavlink_msg_nas_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAS_TM_LEN? msg->len : MAVLINK_MSG_ID_NAS_TM_LEN;
        memset(nas_tm, 0, MAVLINK_MSG_ID_NAS_TM_LEN);
    memcpy(nas_tm, _MAV_PAYLOAD(msg), len);
#endif
}
