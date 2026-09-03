#pragma once
// MESSAGE ADA_TM PACKING

#define MAVLINK_MSG_ID_ADA_TM 167

MAVPACKED(
typedef struct __mavlink_ada_tm_t {
 uint64_t timestamp; /*<  When was this logged*/
 float target_dpl_altitude; /*<  Target deployment altitude*/
 float kalman_x0; /*<  Kalman state variable 0 (pressure)*/
 float kalman_x1; /*<  Kalman state variable 1 (pressure velocity)*/
 float kalman_x2; /*<  Kalman state variable 2 (pressure acceleration)*/
 float kalman_acc_x0; /*<  Kalman2 (acc) state variable 0 (altitude)*/
 float kalman_acc_x1; /*<  Kalman2 (acc) state variable 1 (Vertical speed)*/
 float kalman_acc_x2; /*<  Kalman2 (acc) state variable 2 (Vertical acceleration)*/
 float ref_pressure; /*<  Calibration pressure*/
 float msl_pressure; /*<  Expected pressure at mean sea level*/
 float ref_pressure_mean; /*<  Calibration pressure mean*/
 float ref_pressure_stddev; /*<  Calibration pressure std deviation*/
 uint32_t ref_pressure_nsamples; /*<  Calibration pressure number of samples*/
 float ref_altitude; /*<  Calibration altitude*/
 float ref_temperature; /*<  Calibration temperature*/
 float msl_temperature; /*<  Expected temperature at mean sea level*/
 uint8_t state; /*<  ADA current state*/
}) mavlink_ada_tm_t;

#define MAVLINK_MSG_ID_ADA_TM_LEN 69
#define MAVLINK_MSG_ID_ADA_TM_MIN_LEN 69
#define MAVLINK_MSG_ID_167_LEN 69
#define MAVLINK_MSG_ID_167_MIN_LEN 69

#define MAVLINK_MSG_ID_ADA_TM_CRC 204
#define MAVLINK_MSG_ID_167_CRC 204



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ADA_TM { \
    167, \
    "ADA_TM", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ada_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_ada_tm_t, state) }, \
         { "target_dpl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ada_tm_t, target_dpl_altitude) }, \
         { "kalman_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ada_tm_t, kalman_x0) }, \
         { "kalman_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ada_tm_t, kalman_x1) }, \
         { "kalman_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ada_tm_t, kalman_x2) }, \
         { "kalman_acc_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ada_tm_t, kalman_acc_x0) }, \
         { "kalman_acc_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ada_tm_t, kalman_acc_x1) }, \
         { "kalman_acc_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ada_tm_t, kalman_acc_x2) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ada_tm_t, ref_pressure) }, \
         { "msl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ada_tm_t, msl_pressure) }, \
         { "ref_pressure_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ada_tm_t, ref_pressure_mean) }, \
         { "ref_pressure_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ada_tm_t, ref_pressure_stddev) }, \
         { "ref_pressure_nsamples", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_ada_tm_t, ref_pressure_nsamples) }, \
         { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ada_tm_t, ref_altitude) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ada_tm_t, ref_temperature) }, \
         { "msl_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_ada_tm_t, msl_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ADA_TM { \
    "ADA_TM", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_ada_tm_t, timestamp) }, \
         { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_ada_tm_t, state) }, \
         { "target_dpl_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ada_tm_t, target_dpl_altitude) }, \
         { "kalman_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ada_tm_t, kalman_x0) }, \
         { "kalman_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_ada_tm_t, kalman_x1) }, \
         { "kalman_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_ada_tm_t, kalman_x2) }, \
         { "kalman_acc_x0", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_ada_tm_t, kalman_acc_x0) }, \
         { "kalman_acc_x1", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_ada_tm_t, kalman_acc_x1) }, \
         { "kalman_acc_x2", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_ada_tm_t, kalman_acc_x2) }, \
         { "ref_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_ada_tm_t, ref_pressure) }, \
         { "msl_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_ada_tm_t, msl_pressure) }, \
         { "ref_pressure_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_ada_tm_t, ref_pressure_mean) }, \
         { "ref_pressure_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_ada_tm_t, ref_pressure_stddev) }, \
         { "ref_pressure_nsamples", NULL, MAVLINK_TYPE_UINT32_T, 0, 52, offsetof(mavlink_ada_tm_t, ref_pressure_nsamples) }, \
         { "ref_altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_ada_tm_t, ref_altitude) }, \
         { "ref_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_ada_tm_t, ref_temperature) }, \
         { "msl_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_ada_tm_t, msl_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a ada_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp  When was this logged
 * @param state  ADA current state
 * @param target_dpl_altitude  Target deployment altitude
 * @param kalman_x0  Kalman state variable 0 (pressure)
 * @param kalman_x1  Kalman state variable 1 (pressure velocity)
 * @param kalman_x2  Kalman state variable 2 (pressure acceleration)
 * @param kalman_acc_x0  Kalman2 (acc) state variable 0 (altitude)
 * @param kalman_acc_x1  Kalman2 (acc) state variable 1 (Vertical speed)
 * @param kalman_acc_x2  Kalman2 (acc) state variable 2 (Vertical acceleration)
 * @param ref_pressure  Calibration pressure
 * @param msl_pressure  Expected pressure at mean sea level
 * @param ref_pressure_mean  Calibration pressure mean
 * @param ref_pressure_stddev  Calibration pressure std deviation
 * @param ref_pressure_nsamples  Calibration pressure number of samples
 * @param ref_altitude  Calibration altitude
 * @param ref_temperature  Calibration temperature
 * @param msl_temperature  Expected temperature at mean sea level
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ada_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint8_t state, float target_dpl_altitude, float kalman_x0, float kalman_x1, float kalman_x2, float kalman_acc_x0, float kalman_acc_x1, float kalman_acc_x2, float ref_pressure, float msl_pressure, float ref_pressure_mean, float ref_pressure_stddev, uint32_t ref_pressure_nsamples, float ref_altitude, float ref_temperature, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_dpl_altitude);
    _mav_put_float(buf, 12, kalman_x0);
    _mav_put_float(buf, 16, kalman_x1);
    _mav_put_float(buf, 20, kalman_x2);
    _mav_put_float(buf, 24, kalman_acc_x0);
    _mav_put_float(buf, 28, kalman_acc_x1);
    _mav_put_float(buf, 32, kalman_acc_x2);
    _mav_put_float(buf, 36, ref_pressure);
    _mav_put_float(buf, 40, msl_pressure);
    _mav_put_float(buf, 44, ref_pressure_mean);
    _mav_put_float(buf, 48, ref_pressure_stddev);
    _mav_put_uint32_t(buf, 52, ref_pressure_nsamples);
    _mav_put_float(buf, 56, ref_altitude);
    _mav_put_float(buf, 60, ref_temperature);
    _mav_put_float(buf, 64, msl_temperature);
    _mav_put_uint8_t(buf, 68, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADA_TM_LEN);
#else
    mavlink_ada_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_dpl_altitude = target_dpl_altitude;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.kalman_acc_x0 = kalman_acc_x0;
    packet.kalman_acc_x1 = kalman_acc_x1;
    packet.kalman_acc_x2 = kalman_acc_x2;
    packet.ref_pressure = ref_pressure;
    packet.msl_pressure = msl_pressure;
    packet.ref_pressure_mean = ref_pressure_mean;
    packet.ref_pressure_stddev = ref_pressure_stddev;
    packet.ref_pressure_nsamples = ref_pressure_nsamples;
    packet.ref_altitude = ref_altitude;
    packet.ref_temperature = ref_temperature;
    packet.msl_temperature = msl_temperature;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADA_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
}

/**
 * @brief Pack a ada_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp  When was this logged
 * @param state  ADA current state
 * @param target_dpl_altitude  Target deployment altitude
 * @param kalman_x0  Kalman state variable 0 (pressure)
 * @param kalman_x1  Kalman state variable 1 (pressure velocity)
 * @param kalman_x2  Kalman state variable 2 (pressure acceleration)
 * @param kalman_acc_x0  Kalman2 (acc) state variable 0 (altitude)
 * @param kalman_acc_x1  Kalman2 (acc) state variable 1 (Vertical speed)
 * @param kalman_acc_x2  Kalman2 (acc) state variable 2 (Vertical acceleration)
 * @param ref_pressure  Calibration pressure
 * @param msl_pressure  Expected pressure at mean sea level
 * @param ref_pressure_mean  Calibration pressure mean
 * @param ref_pressure_stddev  Calibration pressure std deviation
 * @param ref_pressure_nsamples  Calibration pressure number of samples
 * @param ref_altitude  Calibration altitude
 * @param ref_temperature  Calibration temperature
 * @param msl_temperature  Expected temperature at mean sea level
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ada_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint8_t state,float target_dpl_altitude,float kalman_x0,float kalman_x1,float kalman_x2,float kalman_acc_x0,float kalman_acc_x1,float kalman_acc_x2,float ref_pressure,float msl_pressure,float ref_pressure_mean,float ref_pressure_stddev,uint32_t ref_pressure_nsamples,float ref_altitude,float ref_temperature,float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_dpl_altitude);
    _mav_put_float(buf, 12, kalman_x0);
    _mav_put_float(buf, 16, kalman_x1);
    _mav_put_float(buf, 20, kalman_x2);
    _mav_put_float(buf, 24, kalman_acc_x0);
    _mav_put_float(buf, 28, kalman_acc_x1);
    _mav_put_float(buf, 32, kalman_acc_x2);
    _mav_put_float(buf, 36, ref_pressure);
    _mav_put_float(buf, 40, msl_pressure);
    _mav_put_float(buf, 44, ref_pressure_mean);
    _mav_put_float(buf, 48, ref_pressure_stddev);
    _mav_put_uint32_t(buf, 52, ref_pressure_nsamples);
    _mav_put_float(buf, 56, ref_altitude);
    _mav_put_float(buf, 60, ref_temperature);
    _mav_put_float(buf, 64, msl_temperature);
    _mav_put_uint8_t(buf, 68, state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADA_TM_LEN);
#else
    mavlink_ada_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_dpl_altitude = target_dpl_altitude;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.kalman_acc_x0 = kalman_acc_x0;
    packet.kalman_acc_x1 = kalman_acc_x1;
    packet.kalman_acc_x2 = kalman_acc_x2;
    packet.ref_pressure = ref_pressure;
    packet.msl_pressure = msl_pressure;
    packet.ref_pressure_mean = ref_pressure_mean;
    packet.ref_pressure_stddev = ref_pressure_stddev;
    packet.ref_pressure_nsamples = ref_pressure_nsamples;
    packet.ref_altitude = ref_altitude;
    packet.ref_temperature = ref_temperature;
    packet.msl_temperature = msl_temperature;
    packet.state = state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADA_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ADA_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
}

/**
 * @brief Encode a ada_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ada_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ada_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ada_tm_t* ada_tm)
{
    return mavlink_msg_ada_tm_pack(system_id, component_id, msg, ada_tm->timestamp, ada_tm->state, ada_tm->target_dpl_altitude, ada_tm->kalman_x0, ada_tm->kalman_x1, ada_tm->kalman_x2, ada_tm->kalman_acc_x0, ada_tm->kalman_acc_x1, ada_tm->kalman_acc_x2, ada_tm->ref_pressure, ada_tm->msl_pressure, ada_tm->ref_pressure_mean, ada_tm->ref_pressure_stddev, ada_tm->ref_pressure_nsamples, ada_tm->ref_altitude, ada_tm->ref_temperature, ada_tm->msl_temperature);
}

/**
 * @brief Encode a ada_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ada_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ada_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ada_tm_t* ada_tm)
{
    return mavlink_msg_ada_tm_pack_chan(system_id, component_id, chan, msg, ada_tm->timestamp, ada_tm->state, ada_tm->target_dpl_altitude, ada_tm->kalman_x0, ada_tm->kalman_x1, ada_tm->kalman_x2, ada_tm->kalman_acc_x0, ada_tm->kalman_acc_x1, ada_tm->kalman_acc_x2, ada_tm->ref_pressure, ada_tm->msl_pressure, ada_tm->ref_pressure_mean, ada_tm->ref_pressure_stddev, ada_tm->ref_pressure_nsamples, ada_tm->ref_altitude, ada_tm->ref_temperature, ada_tm->msl_temperature);
}

/**
 * @brief Send a ada_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp  When was this logged
 * @param state  ADA current state
 * @param target_dpl_altitude  Target deployment altitude
 * @param kalman_x0  Kalman state variable 0 (pressure)
 * @param kalman_x1  Kalman state variable 1 (pressure velocity)
 * @param kalman_x2  Kalman state variable 2 (pressure acceleration)
 * @param kalman_acc_x0  Kalman2 (acc) state variable 0 (altitude)
 * @param kalman_acc_x1  Kalman2 (acc) state variable 1 (Vertical speed)
 * @param kalman_acc_x2  Kalman2 (acc) state variable 2 (Vertical acceleration)
 * @param ref_pressure  Calibration pressure
 * @param msl_pressure  Expected pressure at mean sea level
 * @param ref_pressure_mean  Calibration pressure mean
 * @param ref_pressure_stddev  Calibration pressure std deviation
 * @param ref_pressure_nsamples  Calibration pressure number of samples
 * @param ref_altitude  Calibration altitude
 * @param ref_temperature  Calibration temperature
 * @param msl_temperature  Expected temperature at mean sea level
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ada_tm_send(mavlink_channel_t chan, uint64_t timestamp, uint8_t state, float target_dpl_altitude, float kalman_x0, float kalman_x1, float kalman_x2, float kalman_acc_x0, float kalman_acc_x1, float kalman_acc_x2, float ref_pressure, float msl_pressure, float ref_pressure_mean, float ref_pressure_stddev, uint32_t ref_pressure_nsamples, float ref_altitude, float ref_temperature, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ADA_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_dpl_altitude);
    _mav_put_float(buf, 12, kalman_x0);
    _mav_put_float(buf, 16, kalman_x1);
    _mav_put_float(buf, 20, kalman_x2);
    _mav_put_float(buf, 24, kalman_acc_x0);
    _mav_put_float(buf, 28, kalman_acc_x1);
    _mav_put_float(buf, 32, kalman_acc_x2);
    _mav_put_float(buf, 36, ref_pressure);
    _mav_put_float(buf, 40, msl_pressure);
    _mav_put_float(buf, 44, ref_pressure_mean);
    _mav_put_float(buf, 48, ref_pressure_stddev);
    _mav_put_uint32_t(buf, 52, ref_pressure_nsamples);
    _mav_put_float(buf, 56, ref_altitude);
    _mav_put_float(buf, 60, ref_temperature);
    _mav_put_float(buf, 64, msl_temperature);
    _mav_put_uint8_t(buf, 68, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADA_TM, buf, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
#else
    mavlink_ada_tm_t packet;
    packet.timestamp = timestamp;
    packet.target_dpl_altitude = target_dpl_altitude;
    packet.kalman_x0 = kalman_x0;
    packet.kalman_x1 = kalman_x1;
    packet.kalman_x2 = kalman_x2;
    packet.kalman_acc_x0 = kalman_acc_x0;
    packet.kalman_acc_x1 = kalman_acc_x1;
    packet.kalman_acc_x2 = kalman_acc_x2;
    packet.ref_pressure = ref_pressure;
    packet.msl_pressure = msl_pressure;
    packet.ref_pressure_mean = ref_pressure_mean;
    packet.ref_pressure_stddev = ref_pressure_stddev;
    packet.ref_pressure_nsamples = ref_pressure_nsamples;
    packet.ref_altitude = ref_altitude;
    packet.ref_temperature = ref_temperature;
    packet.msl_temperature = msl_temperature;
    packet.state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADA_TM, (const char *)&packet, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
#endif
}

/**
 * @brief Send a ada_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ada_tm_send_struct(mavlink_channel_t chan, const mavlink_ada_tm_t* ada_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ada_tm_send(chan, ada_tm->timestamp, ada_tm->state, ada_tm->target_dpl_altitude, ada_tm->kalman_x0, ada_tm->kalman_x1, ada_tm->kalman_x2, ada_tm->kalman_acc_x0, ada_tm->kalman_acc_x1, ada_tm->kalman_acc_x2, ada_tm->ref_pressure, ada_tm->msl_pressure, ada_tm->ref_pressure_mean, ada_tm->ref_pressure_stddev, ada_tm->ref_pressure_nsamples, ada_tm->ref_altitude, ada_tm->ref_temperature, ada_tm->msl_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADA_TM, (const char *)ada_tm, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_ADA_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ada_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint8_t state, float target_dpl_altitude, float kalman_x0, float kalman_x1, float kalman_x2, float kalman_acc_x0, float kalman_acc_x1, float kalman_acc_x2, float ref_pressure, float msl_pressure, float ref_pressure_mean, float ref_pressure_stddev, uint32_t ref_pressure_nsamples, float ref_altitude, float ref_temperature, float msl_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, target_dpl_altitude);
    _mav_put_float(buf, 12, kalman_x0);
    _mav_put_float(buf, 16, kalman_x1);
    _mav_put_float(buf, 20, kalman_x2);
    _mav_put_float(buf, 24, kalman_acc_x0);
    _mav_put_float(buf, 28, kalman_acc_x1);
    _mav_put_float(buf, 32, kalman_acc_x2);
    _mav_put_float(buf, 36, ref_pressure);
    _mav_put_float(buf, 40, msl_pressure);
    _mav_put_float(buf, 44, ref_pressure_mean);
    _mav_put_float(buf, 48, ref_pressure_stddev);
    _mav_put_uint32_t(buf, 52, ref_pressure_nsamples);
    _mav_put_float(buf, 56, ref_altitude);
    _mav_put_float(buf, 60, ref_temperature);
    _mav_put_float(buf, 64, msl_temperature);
    _mav_put_uint8_t(buf, 68, state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADA_TM, buf, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
#else
    mavlink_ada_tm_t *packet = (mavlink_ada_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->target_dpl_altitude = target_dpl_altitude;
    packet->kalman_x0 = kalman_x0;
    packet->kalman_x1 = kalman_x1;
    packet->kalman_x2 = kalman_x2;
    packet->kalman_acc_x0 = kalman_acc_x0;
    packet->kalman_acc_x1 = kalman_acc_x1;
    packet->kalman_acc_x2 = kalman_acc_x2;
    packet->ref_pressure = ref_pressure;
    packet->msl_pressure = msl_pressure;
    packet->ref_pressure_mean = ref_pressure_mean;
    packet->ref_pressure_stddev = ref_pressure_stddev;
    packet->ref_pressure_nsamples = ref_pressure_nsamples;
    packet->ref_altitude = ref_altitude;
    packet->ref_temperature = ref_temperature;
    packet->msl_temperature = msl_temperature;
    packet->state = state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADA_TM, (const char *)packet, MAVLINK_MSG_ID_ADA_TM_MIN_LEN, MAVLINK_MSG_ID_ADA_TM_LEN, MAVLINK_MSG_ID_ADA_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE ADA_TM UNPACKING


/**
 * @brief Get field timestamp from ada_tm message
 *
 * @return  When was this logged
 */
static inline uint64_t mavlink_msg_ada_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field state from ada_tm message
 *
 * @return  ADA current state
 */
static inline uint8_t mavlink_msg_ada_tm_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field target_dpl_altitude from ada_tm message
 *
 * @return  Target deployment altitude
 */
static inline float mavlink_msg_ada_tm_get_target_dpl_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field kalman_x0 from ada_tm message
 *
 * @return  Kalman state variable 0 (pressure)
 */
static inline float mavlink_msg_ada_tm_get_kalman_x0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field kalman_x1 from ada_tm message
 *
 * @return  Kalman state variable 1 (pressure velocity)
 */
static inline float mavlink_msg_ada_tm_get_kalman_x1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field kalman_x2 from ada_tm message
 *
 * @return  Kalman state variable 2 (pressure acceleration)
 */
static inline float mavlink_msg_ada_tm_get_kalman_x2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field kalman_acc_x0 from ada_tm message
 *
 * @return  Kalman2 (acc) state variable 0 (altitude)
 */
static inline float mavlink_msg_ada_tm_get_kalman_acc_x0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field kalman_acc_x1 from ada_tm message
 *
 * @return  Kalman2 (acc) state variable 1 (Vertical speed)
 */
static inline float mavlink_msg_ada_tm_get_kalman_acc_x1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field kalman_acc_x2 from ada_tm message
 *
 * @return  Kalman2 (acc) state variable 2 (Vertical acceleration)
 */
static inline float mavlink_msg_ada_tm_get_kalman_acc_x2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ref_pressure from ada_tm message
 *
 * @return  Calibration pressure
 */
static inline float mavlink_msg_ada_tm_get_ref_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field msl_pressure from ada_tm message
 *
 * @return  Expected pressure at mean sea level
 */
static inline float mavlink_msg_ada_tm_get_msl_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field ref_pressure_mean from ada_tm message
 *
 * @return  Calibration pressure mean
 */
static inline float mavlink_msg_ada_tm_get_ref_pressure_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field ref_pressure_stddev from ada_tm message
 *
 * @return  Calibration pressure std deviation
 */
static inline float mavlink_msg_ada_tm_get_ref_pressure_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ref_pressure_nsamples from ada_tm message
 *
 * @return  Calibration pressure number of samples
 */
static inline uint32_t mavlink_msg_ada_tm_get_ref_pressure_nsamples(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  52);
}

/**
 * @brief Get field ref_altitude from ada_tm message
 *
 * @return  Calibration altitude
 */
static inline float mavlink_msg_ada_tm_get_ref_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field ref_temperature from ada_tm message
 *
 * @return  Calibration temperature
 */
static inline float mavlink_msg_ada_tm_get_ref_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field msl_temperature from ada_tm message
 *
 * @return  Expected temperature at mean sea level
 */
static inline float mavlink_msg_ada_tm_get_msl_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Decode a ada_tm message into a struct
 *
 * @param msg The message to decode
 * @param ada_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_ada_tm_decode(const mavlink_message_t* msg, mavlink_ada_tm_t* ada_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ada_tm->timestamp = mavlink_msg_ada_tm_get_timestamp(msg);
    ada_tm->target_dpl_altitude = mavlink_msg_ada_tm_get_target_dpl_altitude(msg);
    ada_tm->kalman_x0 = mavlink_msg_ada_tm_get_kalman_x0(msg);
    ada_tm->kalman_x1 = mavlink_msg_ada_tm_get_kalman_x1(msg);
    ada_tm->kalman_x2 = mavlink_msg_ada_tm_get_kalman_x2(msg);
    ada_tm->kalman_acc_x0 = mavlink_msg_ada_tm_get_kalman_acc_x0(msg);
    ada_tm->kalman_acc_x1 = mavlink_msg_ada_tm_get_kalman_acc_x1(msg);
    ada_tm->kalman_acc_x2 = mavlink_msg_ada_tm_get_kalman_acc_x2(msg);
    ada_tm->ref_pressure = mavlink_msg_ada_tm_get_ref_pressure(msg);
    ada_tm->msl_pressure = mavlink_msg_ada_tm_get_msl_pressure(msg);
    ada_tm->ref_pressure_mean = mavlink_msg_ada_tm_get_ref_pressure_mean(msg);
    ada_tm->ref_pressure_stddev = mavlink_msg_ada_tm_get_ref_pressure_stddev(msg);
    ada_tm->ref_pressure_nsamples = mavlink_msg_ada_tm_get_ref_pressure_nsamples(msg);
    ada_tm->ref_altitude = mavlink_msg_ada_tm_get_ref_altitude(msg);
    ada_tm->ref_temperature = mavlink_msg_ada_tm_get_ref_temperature(msg);
    ada_tm->msl_temperature = mavlink_msg_ada_tm_get_msl_temperature(msg);
    ada_tm->state = mavlink_msg_ada_tm_get_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ADA_TM_LEN? msg->len : MAVLINK_MSG_ID_ADA_TM_LEN;
        memset(ada_tm, 0, MAVLINK_MSG_ID_ADA_TM_LEN);
    memcpy(ada_tm, _MAV_PAYLOAD(msg), len);
#endif
}
