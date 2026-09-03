#pragma once
// MESSAGE TASK_STATS_TM PACKING

#define MAVLINK_MSG_ID_TASK_STATS_TM 165

MAVPACKED(
typedef struct __mavlink_task_stats_tm_t {
 uint64_t timestamp; /*< [ms] When was this logged */
 float task_sensors_6ms_min; /*<  6 ms task min period*/
 float task_sensors_6ms_max; /*<  6 ms task max period*/
 float task_sensors_6ms_mean; /*<  6 ms task mean period*/
 float task_sensors_6ms_stddev; /*<  6 ms task period std deviation*/
 float task_sensors_15ms_min; /*<  15 ms task min period*/
 float task_sensors_15ms_max; /*<  15 ms task max period*/
 float task_sensors_15ms_mean; /*<  15 ms task mean period*/
 float task_sensors_15ms_stddev; /*<  15 ms task period std deviation*/
 float task_sensors_20ms_min; /*<  20 ms task min period*/
 float task_sensors_20ms_max; /*<  20 ms task max period*/
 float task_sensors_20ms_mean; /*<  20 ms task mean period*/
 float task_sensors_20ms_stddev; /*<  20 ms task period std deviation*/
 float task_sensors_24ms_min; /*<  24 ms task min period*/
 float task_sensors_24ms_max; /*<  24 ms task max period*/
 float task_sensors_24ms_mean; /*<  24 ms task mean period*/
 float task_sensors_24ms_stddev; /*<  24 ms task period std deviation*/
 float task_sensors_40ms_min; /*<  40 ms task min period*/
 float task_sensors_40ms_max; /*<  40 ms task max period*/
 float task_sensors_40ms_mean; /*<  40 ms task mean period*/
 float task_sensors_40ms_stddev; /*<  40 ms task period std deviation*/
 float task_sensors_1000ms_min; /*<  1000 ms task min period*/
 float task_sensors_1000ms_max; /*<  1000 ms task max period*/
 float task_sensors_1000ms_mean; /*<  1000 ms task mean period*/
 float task_sensors_1000ms_stddev; /*<  1000 ms task period std deviation*/
 float task_ada_min; /*<  ADA task min period*/
 float task_ada_max; /*<  ADA task max period*/
 float task_ada_mean; /*<  ADA task mean period*/
 float task_ada_stddev; /*<  ADA task period std deviation*/
 float task_nas_min; /*<  NavigationSystem task min period*/
 float task_nas_max; /*<  NavigationSystem task max period*/
 float task_nas_mean; /*<  NavigationSystem task mean period*/
 float task_nas_stddev; /*<  NavigationSystem task period std deviation*/
 float task_abk_min; /*<  Aerobrakes task min period*/
 float task_abk_max; /*<  Aerobrakes task max period*/
 float task_abk_mean; /*<  Aerobrakes task mean period*/
 float task_abk_stddev; /*<  Aerobrakes task period std deviation*/
}) mavlink_task_stats_tm_t;

#define MAVLINK_MSG_ID_TASK_STATS_TM_LEN 152
#define MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN 152
#define MAVLINK_MSG_ID_165_LEN 152
#define MAVLINK_MSG_ID_165_MIN_LEN 152

#define MAVLINK_MSG_ID_TASK_STATS_TM_CRC 170
#define MAVLINK_MSG_ID_165_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    165, \
    "TASK_STATS_TM", \
    37, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_sensors_6ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_min) }, \
         { "task_sensors_6ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_max) }, \
         { "task_sensors_6ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_mean) }, \
         { "task_sensors_6ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_stddev) }, \
         { "task_sensors_15ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_min) }, \
         { "task_sensors_15ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_max) }, \
         { "task_sensors_15ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_mean) }, \
         { "task_sensors_15ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_stddev) }, \
         { "task_sensors_20ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_min) }, \
         { "task_sensors_20ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_max) }, \
         { "task_sensors_20ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_mean) }, \
         { "task_sensors_20ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_stddev) }, \
         { "task_sensors_24ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_min) }, \
         { "task_sensors_24ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_max) }, \
         { "task_sensors_24ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_mean) }, \
         { "task_sensors_24ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_stddev) }, \
         { "task_sensors_40ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_min) }, \
         { "task_sensors_40ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_max) }, \
         { "task_sensors_40ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_mean) }, \
         { "task_sensors_40ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_stddev) }, \
         { "task_sensors_1000ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_min) }, \
         { "task_sensors_1000ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_max) }, \
         { "task_sensors_1000ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_mean) }, \
         { "task_sensors_1000ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_stddev) }, \
         { "task_ada_min", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_task_stats_tm_t, task_ada_min) }, \
         { "task_ada_max", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_task_stats_tm_t, task_ada_max) }, \
         { "task_ada_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_task_stats_tm_t, task_ada_mean) }, \
         { "task_ada_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_task_stats_tm_t, task_ada_stddev) }, \
         { "task_nas_min", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_task_stats_tm_t, task_nas_min) }, \
         { "task_nas_max", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_task_stats_tm_t, task_nas_max) }, \
         { "task_nas_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_task_stats_tm_t, task_nas_mean) }, \
         { "task_nas_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_task_stats_tm_t, task_nas_stddev) }, \
         { "task_abk_min", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_task_stats_tm_t, task_abk_min) }, \
         { "task_abk_max", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_task_stats_tm_t, task_abk_max) }, \
         { "task_abk_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_task_stats_tm_t, task_abk_mean) }, \
         { "task_abk_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_task_stats_tm_t, task_abk_stddev) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TASK_STATS_TM { \
    "TASK_STATS_TM", \
    37, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_task_stats_tm_t, timestamp) }, \
         { "task_sensors_6ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_min) }, \
         { "task_sensors_6ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_max) }, \
         { "task_sensors_6ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_mean) }, \
         { "task_sensors_6ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_task_stats_tm_t, task_sensors_6ms_stddev) }, \
         { "task_sensors_15ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_min) }, \
         { "task_sensors_15ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_max) }, \
         { "task_sensors_15ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_mean) }, \
         { "task_sensors_15ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_task_stats_tm_t, task_sensors_15ms_stddev) }, \
         { "task_sensors_20ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_min) }, \
         { "task_sensors_20ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_max) }, \
         { "task_sensors_20ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_mean) }, \
         { "task_sensors_20ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_task_stats_tm_t, task_sensors_20ms_stddev) }, \
         { "task_sensors_24ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_min) }, \
         { "task_sensors_24ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_max) }, \
         { "task_sensors_24ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_mean) }, \
         { "task_sensors_24ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_task_stats_tm_t, task_sensors_24ms_stddev) }, \
         { "task_sensors_40ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_min) }, \
         { "task_sensors_40ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_max) }, \
         { "task_sensors_40ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_mean) }, \
         { "task_sensors_40ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_task_stats_tm_t, task_sensors_40ms_stddev) }, \
         { "task_sensors_1000ms_min", NULL, MAVLINK_TYPE_FLOAT, 0, 88, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_min) }, \
         { "task_sensors_1000ms_max", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_max) }, \
         { "task_sensors_1000ms_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_mean) }, \
         { "task_sensors_1000ms_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_task_stats_tm_t, task_sensors_1000ms_stddev) }, \
         { "task_ada_min", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_task_stats_tm_t, task_ada_min) }, \
         { "task_ada_max", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_task_stats_tm_t, task_ada_max) }, \
         { "task_ada_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_task_stats_tm_t, task_ada_mean) }, \
         { "task_ada_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_task_stats_tm_t, task_ada_stddev) }, \
         { "task_nas_min", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_task_stats_tm_t, task_nas_min) }, \
         { "task_nas_max", NULL, MAVLINK_TYPE_FLOAT, 0, 124, offsetof(mavlink_task_stats_tm_t, task_nas_max) }, \
         { "task_nas_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 128, offsetof(mavlink_task_stats_tm_t, task_nas_mean) }, \
         { "task_nas_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 132, offsetof(mavlink_task_stats_tm_t, task_nas_stddev) }, \
         { "task_abk_min", NULL, MAVLINK_TYPE_FLOAT, 0, 136, offsetof(mavlink_task_stats_tm_t, task_abk_min) }, \
         { "task_abk_max", NULL, MAVLINK_TYPE_FLOAT, 0, 140, offsetof(mavlink_task_stats_tm_t, task_abk_max) }, \
         { "task_abk_mean", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_task_stats_tm_t, task_abk_mean) }, \
         { "task_abk_stddev", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_task_stats_tm_t, task_abk_stddev) }, \
         } \
}
#endif

/**
 * @brief Pack a task_stats_tm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [ms] When was this logged 
 * @param task_sensors_6ms_min  6 ms task min period
 * @param task_sensors_6ms_max  6 ms task max period
 * @param task_sensors_6ms_mean  6 ms task mean period
 * @param task_sensors_6ms_stddev  6 ms task period std deviation
 * @param task_sensors_15ms_min  15 ms task min period
 * @param task_sensors_15ms_max  15 ms task max period
 * @param task_sensors_15ms_mean  15 ms task mean period
 * @param task_sensors_15ms_stddev  15 ms task period std deviation
 * @param task_sensors_20ms_min  20 ms task min period
 * @param task_sensors_20ms_max  20 ms task max period
 * @param task_sensors_20ms_mean  20 ms task mean period
 * @param task_sensors_20ms_stddev  20 ms task period std deviation
 * @param task_sensors_24ms_min  24 ms task min period
 * @param task_sensors_24ms_max  24 ms task max period
 * @param task_sensors_24ms_mean  24 ms task mean period
 * @param task_sensors_24ms_stddev  24 ms task period std deviation
 * @param task_sensors_40ms_min  40 ms task min period
 * @param task_sensors_40ms_max  40 ms task max period
 * @param task_sensors_40ms_mean  40 ms task mean period
 * @param task_sensors_40ms_stddev  40 ms task period std deviation
 * @param task_sensors_1000ms_min  1000 ms task min period
 * @param task_sensors_1000ms_max  1000 ms task max period
 * @param task_sensors_1000ms_mean  1000 ms task mean period
 * @param task_sensors_1000ms_stddev  1000 ms task period std deviation
 * @param task_ada_min  ADA task min period
 * @param task_ada_max  ADA task max period
 * @param task_ada_mean  ADA task mean period
 * @param task_ada_stddev  ADA task period std deviation
 * @param task_nas_min  NavigationSystem task min period
 * @param task_nas_max  NavigationSystem task max period
 * @param task_nas_mean  NavigationSystem task mean period
 * @param task_nas_stddev  NavigationSystem task period std deviation
 * @param task_abk_min  Aerobrakes task min period
 * @param task_abk_max  Aerobrakes task max period
 * @param task_abk_mean  Aerobrakes task mean period
 * @param task_abk_stddev  Aerobrakes task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float task_sensors_6ms_min, float task_sensors_6ms_max, float task_sensors_6ms_mean, float task_sensors_6ms_stddev, float task_sensors_15ms_min, float task_sensors_15ms_max, float task_sensors_15ms_mean, float task_sensors_15ms_stddev, float task_sensors_20ms_min, float task_sensors_20ms_max, float task_sensors_20ms_mean, float task_sensors_20ms_stddev, float task_sensors_24ms_min, float task_sensors_24ms_max, float task_sensors_24ms_mean, float task_sensors_24ms_stddev, float task_sensors_40ms_min, float task_sensors_40ms_max, float task_sensors_40ms_mean, float task_sensors_40ms_stddev, float task_sensors_1000ms_min, float task_sensors_1000ms_max, float task_sensors_1000ms_mean, float task_sensors_1000ms_stddev, float task_ada_min, float task_ada_max, float task_ada_mean, float task_ada_stddev, float task_nas_min, float task_nas_max, float task_nas_mean, float task_nas_stddev, float task_abk_min, float task_abk_max, float task_abk_mean, float task_abk_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_sensors_6ms_min);
    _mav_put_float(buf, 12, task_sensors_6ms_max);
    _mav_put_float(buf, 16, task_sensors_6ms_mean);
    _mav_put_float(buf, 20, task_sensors_6ms_stddev);
    _mav_put_float(buf, 24, task_sensors_15ms_min);
    _mav_put_float(buf, 28, task_sensors_15ms_max);
    _mav_put_float(buf, 32, task_sensors_15ms_mean);
    _mav_put_float(buf, 36, task_sensors_15ms_stddev);
    _mav_put_float(buf, 40, task_sensors_20ms_min);
    _mav_put_float(buf, 44, task_sensors_20ms_max);
    _mav_put_float(buf, 48, task_sensors_20ms_mean);
    _mav_put_float(buf, 52, task_sensors_20ms_stddev);
    _mav_put_float(buf, 56, task_sensors_24ms_min);
    _mav_put_float(buf, 60, task_sensors_24ms_max);
    _mav_put_float(buf, 64, task_sensors_24ms_mean);
    _mav_put_float(buf, 68, task_sensors_24ms_stddev);
    _mav_put_float(buf, 72, task_sensors_40ms_min);
    _mav_put_float(buf, 76, task_sensors_40ms_max);
    _mav_put_float(buf, 80, task_sensors_40ms_mean);
    _mav_put_float(buf, 84, task_sensors_40ms_stddev);
    _mav_put_float(buf, 88, task_sensors_1000ms_min);
    _mav_put_float(buf, 92, task_sensors_1000ms_max);
    _mav_put_float(buf, 96, task_sensors_1000ms_mean);
    _mav_put_float(buf, 100, task_sensors_1000ms_stddev);
    _mav_put_float(buf, 104, task_ada_min);
    _mav_put_float(buf, 108, task_ada_max);
    _mav_put_float(buf, 112, task_ada_mean);
    _mav_put_float(buf, 116, task_ada_stddev);
    _mav_put_float(buf, 120, task_nas_min);
    _mav_put_float(buf, 124, task_nas_max);
    _mav_put_float(buf, 128, task_nas_mean);
    _mav_put_float(buf, 132, task_nas_stddev);
    _mav_put_float(buf, 136, task_abk_min);
    _mav_put_float(buf, 140, task_abk_max);
    _mav_put_float(buf, 144, task_abk_mean);
    _mav_put_float(buf, 148, task_abk_stddev);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_sensors_6ms_min = task_sensors_6ms_min;
    packet.task_sensors_6ms_max = task_sensors_6ms_max;
    packet.task_sensors_6ms_mean = task_sensors_6ms_mean;
    packet.task_sensors_6ms_stddev = task_sensors_6ms_stddev;
    packet.task_sensors_15ms_min = task_sensors_15ms_min;
    packet.task_sensors_15ms_max = task_sensors_15ms_max;
    packet.task_sensors_15ms_mean = task_sensors_15ms_mean;
    packet.task_sensors_15ms_stddev = task_sensors_15ms_stddev;
    packet.task_sensors_20ms_min = task_sensors_20ms_min;
    packet.task_sensors_20ms_max = task_sensors_20ms_max;
    packet.task_sensors_20ms_mean = task_sensors_20ms_mean;
    packet.task_sensors_20ms_stddev = task_sensors_20ms_stddev;
    packet.task_sensors_24ms_min = task_sensors_24ms_min;
    packet.task_sensors_24ms_max = task_sensors_24ms_max;
    packet.task_sensors_24ms_mean = task_sensors_24ms_mean;
    packet.task_sensors_24ms_stddev = task_sensors_24ms_stddev;
    packet.task_sensors_40ms_min = task_sensors_40ms_min;
    packet.task_sensors_40ms_max = task_sensors_40ms_max;
    packet.task_sensors_40ms_mean = task_sensors_40ms_mean;
    packet.task_sensors_40ms_stddev = task_sensors_40ms_stddev;
    packet.task_sensors_1000ms_min = task_sensors_1000ms_min;
    packet.task_sensors_1000ms_max = task_sensors_1000ms_max;
    packet.task_sensors_1000ms_mean = task_sensors_1000ms_mean;
    packet.task_sensors_1000ms_stddev = task_sensors_1000ms_stddev;
    packet.task_ada_min = task_ada_min;
    packet.task_ada_max = task_ada_max;
    packet.task_ada_mean = task_ada_mean;
    packet.task_ada_stddev = task_ada_stddev;
    packet.task_nas_min = task_nas_min;
    packet.task_nas_max = task_nas_max;
    packet.task_nas_mean = task_nas_mean;
    packet.task_nas_stddev = task_nas_stddev;
    packet.task_abk_min = task_abk_min;
    packet.task_abk_max = task_abk_max;
    packet.task_abk_mean = task_abk_mean;
    packet.task_abk_stddev = task_abk_stddev;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASK_STATS_TM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
}

/**
 * @brief Pack a task_stats_tm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [ms] When was this logged 
 * @param task_sensors_6ms_min  6 ms task min period
 * @param task_sensors_6ms_max  6 ms task max period
 * @param task_sensors_6ms_mean  6 ms task mean period
 * @param task_sensors_6ms_stddev  6 ms task period std deviation
 * @param task_sensors_15ms_min  15 ms task min period
 * @param task_sensors_15ms_max  15 ms task max period
 * @param task_sensors_15ms_mean  15 ms task mean period
 * @param task_sensors_15ms_stddev  15 ms task period std deviation
 * @param task_sensors_20ms_min  20 ms task min period
 * @param task_sensors_20ms_max  20 ms task max period
 * @param task_sensors_20ms_mean  20 ms task mean period
 * @param task_sensors_20ms_stddev  20 ms task period std deviation
 * @param task_sensors_24ms_min  24 ms task min period
 * @param task_sensors_24ms_max  24 ms task max period
 * @param task_sensors_24ms_mean  24 ms task mean period
 * @param task_sensors_24ms_stddev  24 ms task period std deviation
 * @param task_sensors_40ms_min  40 ms task min period
 * @param task_sensors_40ms_max  40 ms task max period
 * @param task_sensors_40ms_mean  40 ms task mean period
 * @param task_sensors_40ms_stddev  40 ms task period std deviation
 * @param task_sensors_1000ms_min  1000 ms task min period
 * @param task_sensors_1000ms_max  1000 ms task max period
 * @param task_sensors_1000ms_mean  1000 ms task mean period
 * @param task_sensors_1000ms_stddev  1000 ms task period std deviation
 * @param task_ada_min  ADA task min period
 * @param task_ada_max  ADA task max period
 * @param task_ada_mean  ADA task mean period
 * @param task_ada_stddev  ADA task period std deviation
 * @param task_nas_min  NavigationSystem task min period
 * @param task_nas_max  NavigationSystem task max period
 * @param task_nas_mean  NavigationSystem task mean period
 * @param task_nas_stddev  NavigationSystem task period std deviation
 * @param task_abk_min  Aerobrakes task min period
 * @param task_abk_max  Aerobrakes task max period
 * @param task_abk_mean  Aerobrakes task mean period
 * @param task_abk_stddev  Aerobrakes task period std deviation
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_task_stats_tm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float task_sensors_6ms_min,float task_sensors_6ms_max,float task_sensors_6ms_mean,float task_sensors_6ms_stddev,float task_sensors_15ms_min,float task_sensors_15ms_max,float task_sensors_15ms_mean,float task_sensors_15ms_stddev,float task_sensors_20ms_min,float task_sensors_20ms_max,float task_sensors_20ms_mean,float task_sensors_20ms_stddev,float task_sensors_24ms_min,float task_sensors_24ms_max,float task_sensors_24ms_mean,float task_sensors_24ms_stddev,float task_sensors_40ms_min,float task_sensors_40ms_max,float task_sensors_40ms_mean,float task_sensors_40ms_stddev,float task_sensors_1000ms_min,float task_sensors_1000ms_max,float task_sensors_1000ms_mean,float task_sensors_1000ms_stddev,float task_ada_min,float task_ada_max,float task_ada_mean,float task_ada_stddev,float task_nas_min,float task_nas_max,float task_nas_mean,float task_nas_stddev,float task_abk_min,float task_abk_max,float task_abk_mean,float task_abk_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_sensors_6ms_min);
    _mav_put_float(buf, 12, task_sensors_6ms_max);
    _mav_put_float(buf, 16, task_sensors_6ms_mean);
    _mav_put_float(buf, 20, task_sensors_6ms_stddev);
    _mav_put_float(buf, 24, task_sensors_15ms_min);
    _mav_put_float(buf, 28, task_sensors_15ms_max);
    _mav_put_float(buf, 32, task_sensors_15ms_mean);
    _mav_put_float(buf, 36, task_sensors_15ms_stddev);
    _mav_put_float(buf, 40, task_sensors_20ms_min);
    _mav_put_float(buf, 44, task_sensors_20ms_max);
    _mav_put_float(buf, 48, task_sensors_20ms_mean);
    _mav_put_float(buf, 52, task_sensors_20ms_stddev);
    _mav_put_float(buf, 56, task_sensors_24ms_min);
    _mav_put_float(buf, 60, task_sensors_24ms_max);
    _mav_put_float(buf, 64, task_sensors_24ms_mean);
    _mav_put_float(buf, 68, task_sensors_24ms_stddev);
    _mav_put_float(buf, 72, task_sensors_40ms_min);
    _mav_put_float(buf, 76, task_sensors_40ms_max);
    _mav_put_float(buf, 80, task_sensors_40ms_mean);
    _mav_put_float(buf, 84, task_sensors_40ms_stddev);
    _mav_put_float(buf, 88, task_sensors_1000ms_min);
    _mav_put_float(buf, 92, task_sensors_1000ms_max);
    _mav_put_float(buf, 96, task_sensors_1000ms_mean);
    _mav_put_float(buf, 100, task_sensors_1000ms_stddev);
    _mav_put_float(buf, 104, task_ada_min);
    _mav_put_float(buf, 108, task_ada_max);
    _mav_put_float(buf, 112, task_ada_mean);
    _mav_put_float(buf, 116, task_ada_stddev);
    _mav_put_float(buf, 120, task_nas_min);
    _mav_put_float(buf, 124, task_nas_max);
    _mav_put_float(buf, 128, task_nas_mean);
    _mav_put_float(buf, 132, task_nas_stddev);
    _mav_put_float(buf, 136, task_abk_min);
    _mav_put_float(buf, 140, task_abk_max);
    _mav_put_float(buf, 144, task_abk_mean);
    _mav_put_float(buf, 148, task_abk_stddev);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_sensors_6ms_min = task_sensors_6ms_min;
    packet.task_sensors_6ms_max = task_sensors_6ms_max;
    packet.task_sensors_6ms_mean = task_sensors_6ms_mean;
    packet.task_sensors_6ms_stddev = task_sensors_6ms_stddev;
    packet.task_sensors_15ms_min = task_sensors_15ms_min;
    packet.task_sensors_15ms_max = task_sensors_15ms_max;
    packet.task_sensors_15ms_mean = task_sensors_15ms_mean;
    packet.task_sensors_15ms_stddev = task_sensors_15ms_stddev;
    packet.task_sensors_20ms_min = task_sensors_20ms_min;
    packet.task_sensors_20ms_max = task_sensors_20ms_max;
    packet.task_sensors_20ms_mean = task_sensors_20ms_mean;
    packet.task_sensors_20ms_stddev = task_sensors_20ms_stddev;
    packet.task_sensors_24ms_min = task_sensors_24ms_min;
    packet.task_sensors_24ms_max = task_sensors_24ms_max;
    packet.task_sensors_24ms_mean = task_sensors_24ms_mean;
    packet.task_sensors_24ms_stddev = task_sensors_24ms_stddev;
    packet.task_sensors_40ms_min = task_sensors_40ms_min;
    packet.task_sensors_40ms_max = task_sensors_40ms_max;
    packet.task_sensors_40ms_mean = task_sensors_40ms_mean;
    packet.task_sensors_40ms_stddev = task_sensors_40ms_stddev;
    packet.task_sensors_1000ms_min = task_sensors_1000ms_min;
    packet.task_sensors_1000ms_max = task_sensors_1000ms_max;
    packet.task_sensors_1000ms_mean = task_sensors_1000ms_mean;
    packet.task_sensors_1000ms_stddev = task_sensors_1000ms_stddev;
    packet.task_ada_min = task_ada_min;
    packet.task_ada_max = task_ada_max;
    packet.task_ada_mean = task_ada_mean;
    packet.task_ada_stddev = task_ada_stddev;
    packet.task_nas_min = task_nas_min;
    packet.task_nas_max = task_nas_max;
    packet.task_nas_mean = task_nas_mean;
    packet.task_nas_stddev = task_nas_stddev;
    packet.task_abk_min = task_abk_min;
    packet.task_abk_max = task_abk_max;
    packet.task_abk_mean = task_abk_mean;
    packet.task_abk_stddev = task_abk_stddev;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TASK_STATS_TM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
}

/**
 * @brief Encode a task_stats_tm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param task_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_stats_tm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_task_stats_tm_t* task_stats_tm)
{
    return mavlink_msg_task_stats_tm_pack(system_id, component_id, msg, task_stats_tm->timestamp, task_stats_tm->task_sensors_6ms_min, task_stats_tm->task_sensors_6ms_max, task_stats_tm->task_sensors_6ms_mean, task_stats_tm->task_sensors_6ms_stddev, task_stats_tm->task_sensors_15ms_min, task_stats_tm->task_sensors_15ms_max, task_stats_tm->task_sensors_15ms_mean, task_stats_tm->task_sensors_15ms_stddev, task_stats_tm->task_sensors_20ms_min, task_stats_tm->task_sensors_20ms_max, task_stats_tm->task_sensors_20ms_mean, task_stats_tm->task_sensors_20ms_stddev, task_stats_tm->task_sensors_24ms_min, task_stats_tm->task_sensors_24ms_max, task_stats_tm->task_sensors_24ms_mean, task_stats_tm->task_sensors_24ms_stddev, task_stats_tm->task_sensors_40ms_min, task_stats_tm->task_sensors_40ms_max, task_stats_tm->task_sensors_40ms_mean, task_stats_tm->task_sensors_40ms_stddev, task_stats_tm->task_sensors_1000ms_min, task_stats_tm->task_sensors_1000ms_max, task_stats_tm->task_sensors_1000ms_mean, task_stats_tm->task_sensors_1000ms_stddev, task_stats_tm->task_ada_min, task_stats_tm->task_ada_max, task_stats_tm->task_ada_mean, task_stats_tm->task_ada_stddev, task_stats_tm->task_nas_min, task_stats_tm->task_nas_max, task_stats_tm->task_nas_mean, task_stats_tm->task_nas_stddev, task_stats_tm->task_abk_min, task_stats_tm->task_abk_max, task_stats_tm->task_abk_mean, task_stats_tm->task_abk_stddev);
}

/**
 * @brief Encode a task_stats_tm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param task_stats_tm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_task_stats_tm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_task_stats_tm_t* task_stats_tm)
{
    return mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, chan, msg, task_stats_tm->timestamp, task_stats_tm->task_sensors_6ms_min, task_stats_tm->task_sensors_6ms_max, task_stats_tm->task_sensors_6ms_mean, task_stats_tm->task_sensors_6ms_stddev, task_stats_tm->task_sensors_15ms_min, task_stats_tm->task_sensors_15ms_max, task_stats_tm->task_sensors_15ms_mean, task_stats_tm->task_sensors_15ms_stddev, task_stats_tm->task_sensors_20ms_min, task_stats_tm->task_sensors_20ms_max, task_stats_tm->task_sensors_20ms_mean, task_stats_tm->task_sensors_20ms_stddev, task_stats_tm->task_sensors_24ms_min, task_stats_tm->task_sensors_24ms_max, task_stats_tm->task_sensors_24ms_mean, task_stats_tm->task_sensors_24ms_stddev, task_stats_tm->task_sensors_40ms_min, task_stats_tm->task_sensors_40ms_max, task_stats_tm->task_sensors_40ms_mean, task_stats_tm->task_sensors_40ms_stddev, task_stats_tm->task_sensors_1000ms_min, task_stats_tm->task_sensors_1000ms_max, task_stats_tm->task_sensors_1000ms_mean, task_stats_tm->task_sensors_1000ms_stddev, task_stats_tm->task_ada_min, task_stats_tm->task_ada_max, task_stats_tm->task_ada_mean, task_stats_tm->task_ada_stddev, task_stats_tm->task_nas_min, task_stats_tm->task_nas_max, task_stats_tm->task_nas_mean, task_stats_tm->task_nas_stddev, task_stats_tm->task_abk_min, task_stats_tm->task_abk_max, task_stats_tm->task_abk_mean, task_stats_tm->task_abk_stddev);
}

/**
 * @brief Send a task_stats_tm message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [ms] When was this logged 
 * @param task_sensors_6ms_min  6 ms task min period
 * @param task_sensors_6ms_max  6 ms task max period
 * @param task_sensors_6ms_mean  6 ms task mean period
 * @param task_sensors_6ms_stddev  6 ms task period std deviation
 * @param task_sensors_15ms_min  15 ms task min period
 * @param task_sensors_15ms_max  15 ms task max period
 * @param task_sensors_15ms_mean  15 ms task mean period
 * @param task_sensors_15ms_stddev  15 ms task period std deviation
 * @param task_sensors_20ms_min  20 ms task min period
 * @param task_sensors_20ms_max  20 ms task max period
 * @param task_sensors_20ms_mean  20 ms task mean period
 * @param task_sensors_20ms_stddev  20 ms task period std deviation
 * @param task_sensors_24ms_min  24 ms task min period
 * @param task_sensors_24ms_max  24 ms task max period
 * @param task_sensors_24ms_mean  24 ms task mean period
 * @param task_sensors_24ms_stddev  24 ms task period std deviation
 * @param task_sensors_40ms_min  40 ms task min period
 * @param task_sensors_40ms_max  40 ms task max period
 * @param task_sensors_40ms_mean  40 ms task mean period
 * @param task_sensors_40ms_stddev  40 ms task period std deviation
 * @param task_sensors_1000ms_min  1000 ms task min period
 * @param task_sensors_1000ms_max  1000 ms task max period
 * @param task_sensors_1000ms_mean  1000 ms task mean period
 * @param task_sensors_1000ms_stddev  1000 ms task period std deviation
 * @param task_ada_min  ADA task min period
 * @param task_ada_max  ADA task max period
 * @param task_ada_mean  ADA task mean period
 * @param task_ada_stddev  ADA task period std deviation
 * @param task_nas_min  NavigationSystem task min period
 * @param task_nas_max  NavigationSystem task max period
 * @param task_nas_mean  NavigationSystem task mean period
 * @param task_nas_stddev  NavigationSystem task period std deviation
 * @param task_abk_min  Aerobrakes task min period
 * @param task_abk_max  Aerobrakes task max period
 * @param task_abk_mean  Aerobrakes task mean period
 * @param task_abk_stddev  Aerobrakes task period std deviation
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_task_stats_tm_send(mavlink_channel_t chan, uint64_t timestamp, float task_sensors_6ms_min, float task_sensors_6ms_max, float task_sensors_6ms_mean, float task_sensors_6ms_stddev, float task_sensors_15ms_min, float task_sensors_15ms_max, float task_sensors_15ms_mean, float task_sensors_15ms_stddev, float task_sensors_20ms_min, float task_sensors_20ms_max, float task_sensors_20ms_mean, float task_sensors_20ms_stddev, float task_sensors_24ms_min, float task_sensors_24ms_max, float task_sensors_24ms_mean, float task_sensors_24ms_stddev, float task_sensors_40ms_min, float task_sensors_40ms_max, float task_sensors_40ms_mean, float task_sensors_40ms_stddev, float task_sensors_1000ms_min, float task_sensors_1000ms_max, float task_sensors_1000ms_mean, float task_sensors_1000ms_stddev, float task_ada_min, float task_ada_max, float task_ada_mean, float task_ada_stddev, float task_nas_min, float task_nas_max, float task_nas_mean, float task_nas_stddev, float task_abk_min, float task_abk_max, float task_abk_mean, float task_abk_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TASK_STATS_TM_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_sensors_6ms_min);
    _mav_put_float(buf, 12, task_sensors_6ms_max);
    _mav_put_float(buf, 16, task_sensors_6ms_mean);
    _mav_put_float(buf, 20, task_sensors_6ms_stddev);
    _mav_put_float(buf, 24, task_sensors_15ms_min);
    _mav_put_float(buf, 28, task_sensors_15ms_max);
    _mav_put_float(buf, 32, task_sensors_15ms_mean);
    _mav_put_float(buf, 36, task_sensors_15ms_stddev);
    _mav_put_float(buf, 40, task_sensors_20ms_min);
    _mav_put_float(buf, 44, task_sensors_20ms_max);
    _mav_put_float(buf, 48, task_sensors_20ms_mean);
    _mav_put_float(buf, 52, task_sensors_20ms_stddev);
    _mav_put_float(buf, 56, task_sensors_24ms_min);
    _mav_put_float(buf, 60, task_sensors_24ms_max);
    _mav_put_float(buf, 64, task_sensors_24ms_mean);
    _mav_put_float(buf, 68, task_sensors_24ms_stddev);
    _mav_put_float(buf, 72, task_sensors_40ms_min);
    _mav_put_float(buf, 76, task_sensors_40ms_max);
    _mav_put_float(buf, 80, task_sensors_40ms_mean);
    _mav_put_float(buf, 84, task_sensors_40ms_stddev);
    _mav_put_float(buf, 88, task_sensors_1000ms_min);
    _mav_put_float(buf, 92, task_sensors_1000ms_max);
    _mav_put_float(buf, 96, task_sensors_1000ms_mean);
    _mav_put_float(buf, 100, task_sensors_1000ms_stddev);
    _mav_put_float(buf, 104, task_ada_min);
    _mav_put_float(buf, 108, task_ada_max);
    _mav_put_float(buf, 112, task_ada_mean);
    _mav_put_float(buf, 116, task_ada_stddev);
    _mav_put_float(buf, 120, task_nas_min);
    _mav_put_float(buf, 124, task_nas_max);
    _mav_put_float(buf, 128, task_nas_mean);
    _mav_put_float(buf, 132, task_nas_stddev);
    _mav_put_float(buf, 136, task_abk_min);
    _mav_put_float(buf, 140, task_abk_max);
    _mav_put_float(buf, 144, task_abk_mean);
    _mav_put_float(buf, 148, task_abk_stddev);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t packet;
    packet.timestamp = timestamp;
    packet.task_sensors_6ms_min = task_sensors_6ms_min;
    packet.task_sensors_6ms_max = task_sensors_6ms_max;
    packet.task_sensors_6ms_mean = task_sensors_6ms_mean;
    packet.task_sensors_6ms_stddev = task_sensors_6ms_stddev;
    packet.task_sensors_15ms_min = task_sensors_15ms_min;
    packet.task_sensors_15ms_max = task_sensors_15ms_max;
    packet.task_sensors_15ms_mean = task_sensors_15ms_mean;
    packet.task_sensors_15ms_stddev = task_sensors_15ms_stddev;
    packet.task_sensors_20ms_min = task_sensors_20ms_min;
    packet.task_sensors_20ms_max = task_sensors_20ms_max;
    packet.task_sensors_20ms_mean = task_sensors_20ms_mean;
    packet.task_sensors_20ms_stddev = task_sensors_20ms_stddev;
    packet.task_sensors_24ms_min = task_sensors_24ms_min;
    packet.task_sensors_24ms_max = task_sensors_24ms_max;
    packet.task_sensors_24ms_mean = task_sensors_24ms_mean;
    packet.task_sensors_24ms_stddev = task_sensors_24ms_stddev;
    packet.task_sensors_40ms_min = task_sensors_40ms_min;
    packet.task_sensors_40ms_max = task_sensors_40ms_max;
    packet.task_sensors_40ms_mean = task_sensors_40ms_mean;
    packet.task_sensors_40ms_stddev = task_sensors_40ms_stddev;
    packet.task_sensors_1000ms_min = task_sensors_1000ms_min;
    packet.task_sensors_1000ms_max = task_sensors_1000ms_max;
    packet.task_sensors_1000ms_mean = task_sensors_1000ms_mean;
    packet.task_sensors_1000ms_stddev = task_sensors_1000ms_stddev;
    packet.task_ada_min = task_ada_min;
    packet.task_ada_max = task_ada_max;
    packet.task_ada_mean = task_ada_mean;
    packet.task_ada_stddev = task_ada_stddev;
    packet.task_nas_min = task_nas_min;
    packet.task_nas_max = task_nas_max;
    packet.task_nas_mean = task_nas_mean;
    packet.task_nas_stddev = task_nas_stddev;
    packet.task_abk_min = task_abk_min;
    packet.task_abk_max = task_abk_max;
    packet.task_abk_mean = task_abk_mean;
    packet.task_abk_stddev = task_abk_stddev;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)&packet, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}

/**
 * @brief Send a task_stats_tm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_task_stats_tm_send_struct(mavlink_channel_t chan, const mavlink_task_stats_tm_t* task_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_task_stats_tm_send(chan, task_stats_tm->timestamp, task_stats_tm->task_sensors_6ms_min, task_stats_tm->task_sensors_6ms_max, task_stats_tm->task_sensors_6ms_mean, task_stats_tm->task_sensors_6ms_stddev, task_stats_tm->task_sensors_15ms_min, task_stats_tm->task_sensors_15ms_max, task_stats_tm->task_sensors_15ms_mean, task_stats_tm->task_sensors_15ms_stddev, task_stats_tm->task_sensors_20ms_min, task_stats_tm->task_sensors_20ms_max, task_stats_tm->task_sensors_20ms_mean, task_stats_tm->task_sensors_20ms_stddev, task_stats_tm->task_sensors_24ms_min, task_stats_tm->task_sensors_24ms_max, task_stats_tm->task_sensors_24ms_mean, task_stats_tm->task_sensors_24ms_stddev, task_stats_tm->task_sensors_40ms_min, task_stats_tm->task_sensors_40ms_max, task_stats_tm->task_sensors_40ms_mean, task_stats_tm->task_sensors_40ms_stddev, task_stats_tm->task_sensors_1000ms_min, task_stats_tm->task_sensors_1000ms_max, task_stats_tm->task_sensors_1000ms_mean, task_stats_tm->task_sensors_1000ms_stddev, task_stats_tm->task_ada_min, task_stats_tm->task_ada_max, task_stats_tm->task_ada_mean, task_stats_tm->task_ada_stddev, task_stats_tm->task_nas_min, task_stats_tm->task_nas_max, task_stats_tm->task_nas_mean, task_stats_tm->task_nas_stddev, task_stats_tm->task_abk_min, task_stats_tm->task_abk_max, task_stats_tm->task_abk_mean, task_stats_tm->task_abk_stddev);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)task_stats_tm, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}

#if MAVLINK_MSG_ID_TASK_STATS_TM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_task_stats_tm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float task_sensors_6ms_min, float task_sensors_6ms_max, float task_sensors_6ms_mean, float task_sensors_6ms_stddev, float task_sensors_15ms_min, float task_sensors_15ms_max, float task_sensors_15ms_mean, float task_sensors_15ms_stddev, float task_sensors_20ms_min, float task_sensors_20ms_max, float task_sensors_20ms_mean, float task_sensors_20ms_stddev, float task_sensors_24ms_min, float task_sensors_24ms_max, float task_sensors_24ms_mean, float task_sensors_24ms_stddev, float task_sensors_40ms_min, float task_sensors_40ms_max, float task_sensors_40ms_mean, float task_sensors_40ms_stddev, float task_sensors_1000ms_min, float task_sensors_1000ms_max, float task_sensors_1000ms_mean, float task_sensors_1000ms_stddev, float task_ada_min, float task_ada_max, float task_ada_mean, float task_ada_stddev, float task_nas_min, float task_nas_max, float task_nas_mean, float task_nas_stddev, float task_abk_min, float task_abk_max, float task_abk_mean, float task_abk_stddev)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, task_sensors_6ms_min);
    _mav_put_float(buf, 12, task_sensors_6ms_max);
    _mav_put_float(buf, 16, task_sensors_6ms_mean);
    _mav_put_float(buf, 20, task_sensors_6ms_stddev);
    _mav_put_float(buf, 24, task_sensors_15ms_min);
    _mav_put_float(buf, 28, task_sensors_15ms_max);
    _mav_put_float(buf, 32, task_sensors_15ms_mean);
    _mav_put_float(buf, 36, task_sensors_15ms_stddev);
    _mav_put_float(buf, 40, task_sensors_20ms_min);
    _mav_put_float(buf, 44, task_sensors_20ms_max);
    _mav_put_float(buf, 48, task_sensors_20ms_mean);
    _mav_put_float(buf, 52, task_sensors_20ms_stddev);
    _mav_put_float(buf, 56, task_sensors_24ms_min);
    _mav_put_float(buf, 60, task_sensors_24ms_max);
    _mav_put_float(buf, 64, task_sensors_24ms_mean);
    _mav_put_float(buf, 68, task_sensors_24ms_stddev);
    _mav_put_float(buf, 72, task_sensors_40ms_min);
    _mav_put_float(buf, 76, task_sensors_40ms_max);
    _mav_put_float(buf, 80, task_sensors_40ms_mean);
    _mav_put_float(buf, 84, task_sensors_40ms_stddev);
    _mav_put_float(buf, 88, task_sensors_1000ms_min);
    _mav_put_float(buf, 92, task_sensors_1000ms_max);
    _mav_put_float(buf, 96, task_sensors_1000ms_mean);
    _mav_put_float(buf, 100, task_sensors_1000ms_stddev);
    _mav_put_float(buf, 104, task_ada_min);
    _mav_put_float(buf, 108, task_ada_max);
    _mav_put_float(buf, 112, task_ada_mean);
    _mav_put_float(buf, 116, task_ada_stddev);
    _mav_put_float(buf, 120, task_nas_min);
    _mav_put_float(buf, 124, task_nas_max);
    _mav_put_float(buf, 128, task_nas_mean);
    _mav_put_float(buf, 132, task_nas_stddev);
    _mav_put_float(buf, 136, task_abk_min);
    _mav_put_float(buf, 140, task_abk_max);
    _mav_put_float(buf, 144, task_abk_mean);
    _mav_put_float(buf, 148, task_abk_stddev);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, buf, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#else
    mavlink_task_stats_tm_t *packet = (mavlink_task_stats_tm_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->task_sensors_6ms_min = task_sensors_6ms_min;
    packet->task_sensors_6ms_max = task_sensors_6ms_max;
    packet->task_sensors_6ms_mean = task_sensors_6ms_mean;
    packet->task_sensors_6ms_stddev = task_sensors_6ms_stddev;
    packet->task_sensors_15ms_min = task_sensors_15ms_min;
    packet->task_sensors_15ms_max = task_sensors_15ms_max;
    packet->task_sensors_15ms_mean = task_sensors_15ms_mean;
    packet->task_sensors_15ms_stddev = task_sensors_15ms_stddev;
    packet->task_sensors_20ms_min = task_sensors_20ms_min;
    packet->task_sensors_20ms_max = task_sensors_20ms_max;
    packet->task_sensors_20ms_mean = task_sensors_20ms_mean;
    packet->task_sensors_20ms_stddev = task_sensors_20ms_stddev;
    packet->task_sensors_24ms_min = task_sensors_24ms_min;
    packet->task_sensors_24ms_max = task_sensors_24ms_max;
    packet->task_sensors_24ms_mean = task_sensors_24ms_mean;
    packet->task_sensors_24ms_stddev = task_sensors_24ms_stddev;
    packet->task_sensors_40ms_min = task_sensors_40ms_min;
    packet->task_sensors_40ms_max = task_sensors_40ms_max;
    packet->task_sensors_40ms_mean = task_sensors_40ms_mean;
    packet->task_sensors_40ms_stddev = task_sensors_40ms_stddev;
    packet->task_sensors_1000ms_min = task_sensors_1000ms_min;
    packet->task_sensors_1000ms_max = task_sensors_1000ms_max;
    packet->task_sensors_1000ms_mean = task_sensors_1000ms_mean;
    packet->task_sensors_1000ms_stddev = task_sensors_1000ms_stddev;
    packet->task_ada_min = task_ada_min;
    packet->task_ada_max = task_ada_max;
    packet->task_ada_mean = task_ada_mean;
    packet->task_ada_stddev = task_ada_stddev;
    packet->task_nas_min = task_nas_min;
    packet->task_nas_max = task_nas_max;
    packet->task_nas_mean = task_nas_mean;
    packet->task_nas_stddev = task_nas_stddev;
    packet->task_abk_min = task_abk_min;
    packet->task_abk_max = task_abk_max;
    packet->task_abk_mean = task_abk_mean;
    packet->task_abk_stddev = task_abk_stddev;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TASK_STATS_TM, (const char *)packet, MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_LEN, MAVLINK_MSG_ID_TASK_STATS_TM_CRC);
#endif
}
#endif

#endif

// MESSAGE TASK_STATS_TM UNPACKING


/**
 * @brief Get field timestamp from task_stats_tm message
 *
 * @return [ms] When was this logged 
 */
static inline uint64_t mavlink_msg_task_stats_tm_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field task_sensors_6ms_min from task_stats_tm message
 *
 * @return  6 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_6ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field task_sensors_6ms_max from task_stats_tm message
 *
 * @return  6 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_6ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field task_sensors_6ms_mean from task_stats_tm message
 *
 * @return  6 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_6ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field task_sensors_6ms_stddev from task_stats_tm message
 *
 * @return  6 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_6ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field task_sensors_15ms_min from task_stats_tm message
 *
 * @return  15 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_15ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field task_sensors_15ms_max from task_stats_tm message
 *
 * @return  15 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_15ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field task_sensors_15ms_mean from task_stats_tm message
 *
 * @return  15 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_15ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field task_sensors_15ms_stddev from task_stats_tm message
 *
 * @return  15 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_15ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field task_sensors_20ms_min from task_stats_tm message
 *
 * @return  20 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_20ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field task_sensors_20ms_max from task_stats_tm message
 *
 * @return  20 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_20ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field task_sensors_20ms_mean from task_stats_tm message
 *
 * @return  20 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_20ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field task_sensors_20ms_stddev from task_stats_tm message
 *
 * @return  20 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_20ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field task_sensors_24ms_min from task_stats_tm message
 *
 * @return  24 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_24ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field task_sensors_24ms_max from task_stats_tm message
 *
 * @return  24 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_24ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field task_sensors_24ms_mean from task_stats_tm message
 *
 * @return  24 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_24ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field task_sensors_24ms_stddev from task_stats_tm message
 *
 * @return  24 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_24ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field task_sensors_40ms_min from task_stats_tm message
 *
 * @return  40 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_40ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field task_sensors_40ms_max from task_stats_tm message
 *
 * @return  40 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_40ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field task_sensors_40ms_mean from task_stats_tm message
 *
 * @return  40 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_40ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field task_sensors_40ms_stddev from task_stats_tm message
 *
 * @return  40 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_40ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Get field task_sensors_1000ms_min from task_stats_tm message
 *
 * @return  1000 ms task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_1000ms_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  88);
}

/**
 * @brief Get field task_sensors_1000ms_max from task_stats_tm message
 *
 * @return  1000 ms task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_1000ms_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field task_sensors_1000ms_mean from task_stats_tm message
 *
 * @return  1000 ms task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_1000ms_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field task_sensors_1000ms_stddev from task_stats_tm message
 *
 * @return  1000 ms task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_sensors_1000ms_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field task_ada_min from task_stats_tm message
 *
 * @return  ADA task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_ada_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field task_ada_max from task_stats_tm message
 *
 * @return  ADA task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_ada_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field task_ada_mean from task_stats_tm message
 *
 * @return  ADA task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_ada_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field task_ada_stddev from task_stats_tm message
 *
 * @return  ADA task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_ada_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field task_nas_min from task_stats_tm message
 *
 * @return  NavigationSystem task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_nas_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field task_nas_max from task_stats_tm message
 *
 * @return  NavigationSystem task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_nas_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  124);
}

/**
 * @brief Get field task_nas_mean from task_stats_tm message
 *
 * @return  NavigationSystem task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_nas_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  128);
}

/**
 * @brief Get field task_nas_stddev from task_stats_tm message
 *
 * @return  NavigationSystem task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_nas_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  132);
}

/**
 * @brief Get field task_abk_min from task_stats_tm message
 *
 * @return  Aerobrakes task min period
 */
static inline float mavlink_msg_task_stats_tm_get_task_abk_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  136);
}

/**
 * @brief Get field task_abk_max from task_stats_tm message
 *
 * @return  Aerobrakes task max period
 */
static inline float mavlink_msg_task_stats_tm_get_task_abk_max(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  140);
}

/**
 * @brief Get field task_abk_mean from task_stats_tm message
 *
 * @return  Aerobrakes task mean period
 */
static inline float mavlink_msg_task_stats_tm_get_task_abk_mean(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field task_abk_stddev from task_stats_tm message
 *
 * @return  Aerobrakes task period std deviation
 */
static inline float mavlink_msg_task_stats_tm_get_task_abk_stddev(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Decode a task_stats_tm message into a struct
 *
 * @param msg The message to decode
 * @param task_stats_tm C-struct to decode the message contents into
 */
static inline void mavlink_msg_task_stats_tm_decode(const mavlink_message_t* msg, mavlink_task_stats_tm_t* task_stats_tm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    task_stats_tm->timestamp = mavlink_msg_task_stats_tm_get_timestamp(msg);
    task_stats_tm->task_sensors_6ms_min = mavlink_msg_task_stats_tm_get_task_sensors_6ms_min(msg);
    task_stats_tm->task_sensors_6ms_max = mavlink_msg_task_stats_tm_get_task_sensors_6ms_max(msg);
    task_stats_tm->task_sensors_6ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_6ms_mean(msg);
    task_stats_tm->task_sensors_6ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_6ms_stddev(msg);
    task_stats_tm->task_sensors_15ms_min = mavlink_msg_task_stats_tm_get_task_sensors_15ms_min(msg);
    task_stats_tm->task_sensors_15ms_max = mavlink_msg_task_stats_tm_get_task_sensors_15ms_max(msg);
    task_stats_tm->task_sensors_15ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_15ms_mean(msg);
    task_stats_tm->task_sensors_15ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_15ms_stddev(msg);
    task_stats_tm->task_sensors_20ms_min = mavlink_msg_task_stats_tm_get_task_sensors_20ms_min(msg);
    task_stats_tm->task_sensors_20ms_max = mavlink_msg_task_stats_tm_get_task_sensors_20ms_max(msg);
    task_stats_tm->task_sensors_20ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_20ms_mean(msg);
    task_stats_tm->task_sensors_20ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_20ms_stddev(msg);
    task_stats_tm->task_sensors_24ms_min = mavlink_msg_task_stats_tm_get_task_sensors_24ms_min(msg);
    task_stats_tm->task_sensors_24ms_max = mavlink_msg_task_stats_tm_get_task_sensors_24ms_max(msg);
    task_stats_tm->task_sensors_24ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_24ms_mean(msg);
    task_stats_tm->task_sensors_24ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_24ms_stddev(msg);
    task_stats_tm->task_sensors_40ms_min = mavlink_msg_task_stats_tm_get_task_sensors_40ms_min(msg);
    task_stats_tm->task_sensors_40ms_max = mavlink_msg_task_stats_tm_get_task_sensors_40ms_max(msg);
    task_stats_tm->task_sensors_40ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_40ms_mean(msg);
    task_stats_tm->task_sensors_40ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_40ms_stddev(msg);
    task_stats_tm->task_sensors_1000ms_min = mavlink_msg_task_stats_tm_get_task_sensors_1000ms_min(msg);
    task_stats_tm->task_sensors_1000ms_max = mavlink_msg_task_stats_tm_get_task_sensors_1000ms_max(msg);
    task_stats_tm->task_sensors_1000ms_mean = mavlink_msg_task_stats_tm_get_task_sensors_1000ms_mean(msg);
    task_stats_tm->task_sensors_1000ms_stddev = mavlink_msg_task_stats_tm_get_task_sensors_1000ms_stddev(msg);
    task_stats_tm->task_ada_min = mavlink_msg_task_stats_tm_get_task_ada_min(msg);
    task_stats_tm->task_ada_max = mavlink_msg_task_stats_tm_get_task_ada_max(msg);
    task_stats_tm->task_ada_mean = mavlink_msg_task_stats_tm_get_task_ada_mean(msg);
    task_stats_tm->task_ada_stddev = mavlink_msg_task_stats_tm_get_task_ada_stddev(msg);
    task_stats_tm->task_nas_min = mavlink_msg_task_stats_tm_get_task_nas_min(msg);
    task_stats_tm->task_nas_max = mavlink_msg_task_stats_tm_get_task_nas_max(msg);
    task_stats_tm->task_nas_mean = mavlink_msg_task_stats_tm_get_task_nas_mean(msg);
    task_stats_tm->task_nas_stddev = mavlink_msg_task_stats_tm_get_task_nas_stddev(msg);
    task_stats_tm->task_abk_min = mavlink_msg_task_stats_tm_get_task_abk_min(msg);
    task_stats_tm->task_abk_max = mavlink_msg_task_stats_tm_get_task_abk_max(msg);
    task_stats_tm->task_abk_mean = mavlink_msg_task_stats_tm_get_task_abk_mean(msg);
    task_stats_tm->task_abk_stddev = mavlink_msg_task_stats_tm_get_task_abk_stddev(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TASK_STATS_TM_LEN? msg->len : MAVLINK_MSG_ID_TASK_STATS_TM_LEN;
        memset(task_stats_tm, 0, MAVLINK_MSG_ID_TASK_STATS_TM_LEN);
    memcpy(task_stats_tm, _MAV_PAYLOAD(msg), len);
#endif
}
