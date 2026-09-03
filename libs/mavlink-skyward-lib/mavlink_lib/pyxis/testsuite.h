/** @file
 *    @brief MAVLink comm protocol testsuite generated from pyxis.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef PYXIS_TESTSUITE_H
#define PYXIS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_pyxis(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_pyxis(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ping_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PING_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ping_tc_t packet_in = {
        93372036854775807ULL
    };
    mavlink_ping_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PING_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PING_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_pack(system_id, component_id, &msg , packet1.timestamp );
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp );
    mavlink_msg_ping_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ping_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ping_tc_send(MAVLINK_COMM_1 , packet1.timestamp );
    mavlink_msg_ping_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PING_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PING_TC) != NULL);
#endif
}

static void mavlink_test_command_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_COMMAND_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_command_tc_t packet_in = {
        5
    };
    mavlink_command_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.command_id = packet_in.command_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_COMMAND_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_COMMAND_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_tc_pack(system_id, component_id, &msg , packet1.command_id );
    mavlink_msg_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command_id );
    mavlink_msg_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_command_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_command_tc_send(MAVLINK_COMM_1 , packet1.command_id );
    mavlink_msg_command_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("COMMAND_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_COMMAND_TC) != NULL);
#endif
}

static void mavlink_test_system_tm_request_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_system_tm_request_tc_t packet_in = {
        5
    };
    mavlink_system_tm_request_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tm_id = packet_in.tm_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_tm_request_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_system_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_tm_request_tc_pack(system_id, component_id, &msg , packet1.tm_id );
    mavlink_msg_system_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_tm_request_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tm_id );
    mavlink_msg_system_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_system_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_system_tm_request_tc_send(MAVLINK_COMM_1 , packet1.tm_id );
    mavlink_msg_system_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYSTEM_TM_REQUEST_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYSTEM_TM_REQUEST_TC) != NULL);
#endif
}

static void mavlink_test_sensor_tm_request_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_tm_request_tc_t packet_in = {
        5
    };
    mavlink_sensor_tm_request_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.sensor_id = packet_in.sensor_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_tm_request_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_tm_request_tc_pack(system_id, component_id, &msg , packet1.sensor_id );
    mavlink_msg_sensor_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_tm_request_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensor_id );
    mavlink_msg_sensor_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_tm_request_tc_send(MAVLINK_COMM_1 , packet1.sensor_id );
    mavlink_msg_sensor_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENSOR_TM_REQUEST_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENSOR_TM_REQUEST_TC) != NULL);
#endif
}

static void mavlink_test_servo_tm_request_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_servo_tm_request_tc_t packet_in = {
        5
    };
    mavlink_servo_tm_request_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_request_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_servo_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_request_tc_pack(system_id, component_id, &msg , packet1.servo_id );
    mavlink_msg_servo_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_request_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id );
    mavlink_msg_servo_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_servo_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_request_tc_send(MAVLINK_COMM_1 , packet1.servo_id );
    mavlink_msg_servo_tm_request_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SERVO_TM_REQUEST_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SERVO_TM_REQUEST_TC) != NULL);
#endif
}

static void mavlink_test_set_servo_angle_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_servo_angle_tc_t packet_in = {
        17.0,17
    };
    mavlink_set_servo_angle_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.angle = packet_in.angle;
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_servo_angle_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_servo_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_servo_angle_tc_pack(system_id, component_id, &msg , packet1.servo_id , packet1.angle );
    mavlink_msg_set_servo_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_servo_angle_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id , packet1.angle );
    mavlink_msg_set_servo_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_servo_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_servo_angle_tc_send(MAVLINK_COMM_1 , packet1.servo_id , packet1.angle );
    mavlink_msg_set_servo_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_SERVO_ANGLE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_SERVO_ANGLE_TC) != NULL);
#endif
}

static void mavlink_test_wiggle_servo_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WIGGLE_SERVO_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wiggle_servo_tc_t packet_in = {
        5
    };
    mavlink_wiggle_servo_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WIGGLE_SERVO_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WIGGLE_SERVO_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wiggle_servo_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wiggle_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wiggle_servo_tc_pack(system_id, component_id, &msg , packet1.servo_id );
    mavlink_msg_wiggle_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wiggle_servo_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id );
    mavlink_msg_wiggle_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wiggle_servo_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wiggle_servo_tc_send(MAVLINK_COMM_1 , packet1.servo_id );
    mavlink_msg_wiggle_servo_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WIGGLE_SERVO_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WIGGLE_SERVO_TC) != NULL);
#endif
}

static void mavlink_test_reset_servo_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RESET_SERVO_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_reset_servo_tc_t packet_in = {
        5
    };
    mavlink_reset_servo_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RESET_SERVO_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RESET_SERVO_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_servo_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_reset_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_servo_tc_pack(system_id, component_id, &msg , packet1.servo_id );
    mavlink_msg_reset_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_servo_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id );
    mavlink_msg_reset_servo_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_reset_servo_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reset_servo_tc_send(MAVLINK_COMM_1 , packet1.servo_id );
    mavlink_msg_reset_servo_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RESET_SERVO_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RESET_SERVO_TC) != NULL);
#endif
}

static void mavlink_test_set_reference_altitude_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_reference_altitude_tc_t packet_in = {
        17.0
    };
    mavlink_set_reference_altitude_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ref_altitude = packet_in.ref_altitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_reference_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_tc_pack(system_id, component_id, &msg , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_reference_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_altitude_tc_send(MAVLINK_COMM_1 , packet1.ref_altitude );
    mavlink_msg_set_reference_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_REFERENCE_ALTITUDE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_REFERENCE_ALTITUDE_TC) != NULL);
#endif
}

static void mavlink_test_set_reference_temperature_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_reference_temperature_tc_t packet_in = {
        17.0
    };
    mavlink_set_reference_temperature_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ref_temp = packet_in.ref_temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_pack(system_id, component_id, &msg , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_reference_temperature_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_reference_temperature_tc_send(MAVLINK_COMM_1 , packet1.ref_temp );
    mavlink_msg_set_reference_temperature_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_REFERENCE_TEMPERATURE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_REFERENCE_TEMPERATURE_TC) != NULL);
#endif
}

static void mavlink_test_set_orientation_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ORIENTATION_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_orientation_tc_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_set_orientation_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.yaw = packet_in.yaw;
        packet1.pitch = packet_in.pitch;
        packet1.roll = packet_in.roll;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ORIENTATION_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_tc_pack(system_id, component_id, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_orientation_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_orientation_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_tc_send(MAVLINK_COMM_1 , packet1.yaw , packet1.pitch , packet1.roll );
    mavlink_msg_set_orientation_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ORIENTATION_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ORIENTATION_TC) != NULL);
#endif
}

static void mavlink_test_set_coordinates_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_COORDINATES_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_coordinates_tc_t packet_in = {
        17.0,45.0
    };
    mavlink_set_coordinates_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_COORDINATES_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_COORDINATES_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_coordinates_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_coordinates_tc_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_coordinates_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_coordinates_tc_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude );
    mavlink_msg_set_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_COORDINATES_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_COORDINATES_TC) != NULL);
#endif
}

static void mavlink_test_raw_event_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RAW_EVENT_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_raw_event_tc_t packet_in = {
        5,72
    };
    mavlink_raw_event_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.topic_id = packet_in.topic_id;
        packet1.event_id = packet_in.event_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RAW_EVENT_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_pack(system_id, component_id, &msg , packet1.topic_id , packet1.event_id );
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.topic_id , packet1.event_id );
    mavlink_msg_raw_event_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_raw_event_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_raw_event_tc_send(MAVLINK_COMM_1 , packet1.topic_id , packet1.event_id );
    mavlink_msg_raw_event_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RAW_EVENT_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RAW_EVENT_TC) != NULL);
#endif
}

static void mavlink_test_set_deployment_altitude_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_deployment_altitude_tc_t packet_in = {
        17.0
    };
    mavlink_set_deployment_altitude_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.dpl_altitude = packet_in.dpl_altitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_pack(system_id, component_id, &msg , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_deployment_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_deployment_altitude_tc_send(MAVLINK_COMM_1 , packet1.dpl_altitude );
    mavlink_msg_set_deployment_altitude_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_DEPLOYMENT_ALTITUDE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_DEPLOYMENT_ALTITUDE_TC) != NULL);
#endif
}

static void mavlink_test_set_target_coordinates_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_target_coordinates_tc_t packet_in = {
        17.0,45.0
    };
    mavlink_set_target_coordinates_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_target_coordinates_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_target_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_target_coordinates_tc_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_target_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_target_coordinates_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude );
    mavlink_msg_set_target_coordinates_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_target_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_target_coordinates_tc_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude );
    mavlink_msg_set_target_coordinates_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_TARGET_COORDINATES_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_TARGET_COORDINATES_TC) != NULL);
#endif
}

static void mavlink_test_set_algorithm_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ALGORITHM_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_algorithm_tc_t packet_in = {
        5
    };
    mavlink_set_algorithm_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.algorithm_number = packet_in.algorithm_number;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ALGORITHM_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ALGORITHM_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_algorithm_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_algorithm_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_algorithm_tc_pack(system_id, component_id, &msg , packet1.algorithm_number );
    mavlink_msg_set_algorithm_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_algorithm_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.algorithm_number );
    mavlink_msg_set_algorithm_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_algorithm_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_algorithm_tc_send(MAVLINK_COMM_1 , packet1.algorithm_number );
    mavlink_msg_set_algorithm_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ALGORITHM_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ALGORITHM_TC) != NULL);
#endif
}

static void mavlink_test_ack_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ACK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ack_tm_t packet_in = {
        5,72
    };
    mavlink_ack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.recv_msgid = packet_in.recv_msgid;
        packet1.seq_ack = packet_in.seq_ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ACK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ACK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_ack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ACK_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ACK_TM) != NULL);
#endif
}

static void mavlink_test_nack_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NACK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_nack_tm_t packet_in = {
        5,72
    };
    mavlink_nack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.recv_msgid = packet_in.recv_msgid;
        packet1.seq_ack = packet_in.seq_ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NACK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NACK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_nack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack );
    mavlink_msg_nack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NACK_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NACK_TM) != NULL);
#endif
}

static void mavlink_test_gps_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GPS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gps_tm_t packet_in = {
        93372036854775807ULL,179.0,235.0,291.0,241.0,269.0,297.0,325.0,353.0,"ABCDEFGHIJKLMNOPQRS",221,32
    };
    mavlink_gps_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.height = packet_in.height;
        packet1.vel_north = packet_in.vel_north;
        packet1.vel_east = packet_in.vel_east;
        packet1.vel_down = packet_in.vel_down;
        packet1.speed = packet_in.speed;
        packet1.track = packet_in.track;
        packet1.fix = packet_in.fix;
        packet1.n_satellites = packet_in.n_satellites;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GPS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GPS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gps_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GPS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GPS_TM) != NULL);
#endif
}

static void mavlink_test_imu_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,"STUVWXYZABCDEFGHIJK"
    };
    mavlink_imu_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("IMU_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_IMU_TM) != NULL);
#endif
}

static void mavlink_test_pressure_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PRESSURE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pressure_tm_t packet_in = {
        93372036854775807ULL,73.0,"MNOPQRSTUVWXYZABCDE"
    };
    mavlink_pressure_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure = packet_in.pressure;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PRESSURE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pressure_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.pressure );
    mavlink_msg_pressure_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.pressure );
    mavlink_msg_pressure_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pressure_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.pressure );
    mavlink_msg_pressure_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PRESSURE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PRESSURE_TM) != NULL);
#endif
}

static void mavlink_test_adc_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADC_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_adc_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,"YZABCDEFGHIJKLMNOPQ"
    };
    mavlink_adc_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.channel_0 = packet_in.channel_0;
        packet1.channel_1 = packet_in.channel_1;
        packet1.channel_2 = packet_in.channel_2;
        packet1.channel_3 = packet_in.channel_3;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADC_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADC_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_adc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 );
    mavlink_msg_adc_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADC_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADC_TM) != NULL);
#endif
}

static void mavlink_test_voltage_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_VOLTAGE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_voltage_tm_t packet_in = {
        93372036854775807ULL,73.0,"MNOPQRSTUVWXYZABCDE"
    };
    mavlink_voltage_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.voltage = packet_in.voltage;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_VOLTAGE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_VOLTAGE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_voltage_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_voltage_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_voltage_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.voltage );
    mavlink_msg_voltage_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_voltage_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.voltage );
    mavlink_msg_voltage_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_voltage_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_voltage_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.voltage );
    mavlink_msg_voltage_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("VOLTAGE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_VOLTAGE_TM) != NULL);
#endif
}

static void mavlink_test_current_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CURRENT_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_current_tm_t packet_in = {
        93372036854775807ULL,73.0,"MNOPQRSTUVWXYZABCDE"
    };
    mavlink_current_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.current = packet_in.current;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CURRENT_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_current_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_current_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_current_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.current );
    mavlink_msg_current_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_current_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.current );
    mavlink_msg_current_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_current_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_current_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.current );
    mavlink_msg_current_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CURRENT_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CURRENT_TM) != NULL);
#endif
}

static void mavlink_test_temp_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TEMP_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_temp_tm_t packet_in = {
        93372036854775807ULL,73.0,"MNOPQRSTUVWXYZABCDE"
    };
    mavlink_temp_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.temperature = packet_in.temperature;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TEMP_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TEMP_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_temp_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_temp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_temp_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.temperature );
    mavlink_msg_temp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_temp_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.temperature );
    mavlink_msg_temp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_temp_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_temp_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.temperature );
    mavlink_msg_temp_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TEMP_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TEMP_TM) != NULL);
#endif
}

static void mavlink_test_load_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOAD_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_load_tm_t packet_in = {
        93372036854775807ULL,73.0,"MNOPQRSTUVWXYZABCDE"
    };
    mavlink_load_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.load = packet_in.load;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOAD_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOAD_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_load_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_load_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_load_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.load );
    mavlink_msg_load_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_load_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.load );
    mavlink_msg_load_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_load_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_load_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.load );
    mavlink_msg_load_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOAD_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOAD_TM) != NULL);
#endif
}

static void mavlink_test_attitude_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,"KLMNOPQRSTUVWXYZABC"
    };
    mavlink_attitude_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.quat_x = packet_in.quat_x;
        packet1.quat_y = packet_in.quat_y;
        packet1.quat_z = packet_in.quat_z;
        packet1.quat_w = packet_in.quat_w;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_attitude_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_attitude_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_id , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_attitude_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ATTITUDE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ATTITUDE_TM) != NULL);
#endif
}

static void mavlink_test_sensor_state_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SENSOR_STATE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sensor_state_tm_t packet_in = {
        "ABCDEFGHIJKLMNOPQRS",65
    };
    mavlink_sensor_state_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.state = packet_in.state;
        
        mav_array_memcpy(packet1.sensor_id, packet_in.sensor_id, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SENSOR_STATE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_state_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sensor_state_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_state_tm_pack(system_id, component_id, &msg , packet1.sensor_id , packet1.state );
    mavlink_msg_sensor_state_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_state_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensor_id , packet1.state );
    mavlink_msg_sensor_state_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sensor_state_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_state_tm_send(MAVLINK_COMM_1 , packet1.sensor_id , packet1.state );
    mavlink_msg_sensor_state_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SENSOR_STATE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SENSOR_STATE_TM) != NULL);
#endif
}

static void mavlink_test_servo_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SERVO_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_servo_tm_t packet_in = {
        17.0,17
    };
    mavlink_servo_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.servo_position = packet_in.servo_position;
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SERVO_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SERVO_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_servo_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_pack(system_id, component_id, &msg , packet1.servo_id , packet1.servo_position );
    mavlink_msg_servo_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id , packet1.servo_position );
    mavlink_msg_servo_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_servo_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_servo_tm_send(MAVLINK_COMM_1 , packet1.servo_id , packet1.servo_position );
    mavlink_msg_servo_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SERVO_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SERVO_TM) != NULL);
#endif
}

static void mavlink_test_pin_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PIN_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pin_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,53,120,187
    };
    mavlink_pin_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.last_change_timestamp = packet_in.last_change_timestamp;
        packet1.pin_id = packet_in.pin_id;
        packet1.changes_counter = packet_in.changes_counter;
        packet1.current_state = packet_in.current_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PIN_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PIN_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pin_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pin_id , packet1.last_change_timestamp , packet1.changes_counter , packet1.current_state );
    mavlink_msg_pin_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pin_id , packet1.last_change_timestamp , packet1.changes_counter , packet1.current_state );
    mavlink_msg_pin_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pin_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pin_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pin_id , packet1.last_change_timestamp , packet1.changes_counter , packet1.current_state );
    mavlink_msg_pin_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PIN_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PIN_TM) != NULL);
#endif
}

static void mavlink_test_receiver_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RECEIVER_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_receiver_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,185.0,213.0
    };
    mavlink_receiver_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.rx_bitrate = packet_in.rx_bitrate;
        packet1.tx_bitrate = packet_in.tx_bitrate;
        packet1.rssi = packet_in.rssi;
        packet1.fei = packet_in.fei;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RECEIVER_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_receiver_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_receiver_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_receiver_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.rssi , packet1.fei , packet1.rx_bitrate , packet1.tx_bitrate );
    mavlink_msg_receiver_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_receiver_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.rssi , packet1.fei , packet1.rx_bitrate , packet1.tx_bitrate );
    mavlink_msg_receiver_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_receiver_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_receiver_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.rssi , packet1.fei , packet1.rx_bitrate , packet1.tx_bitrate );
    mavlink_msg_receiver_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RECEIVER_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RECEIVER_TM) != NULL);
#endif
}

static void mavlink_test_sys_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SYS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sys_tm_t packet_in = {
        93372036854775807ULL,29,96,163,230,41,108
    };
    mavlink_sys_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.logger = packet_in.logger;
        packet1.event_broker = packet_in.event_broker;
        packet1.radio = packet_in.radio;
        packet1.pin_observer = packet_in.pin_observer;
        packet1.sensors = packet_in.sensors;
        packet1.board_scheduler = packet_in.board_scheduler;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SYS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SYS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.pin_observer , packet1.sensors , packet1.board_scheduler );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.pin_observer , packet1.sensors , packet1.board_scheduler );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sys_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.pin_observer , packet1.sensors , packet1.board_scheduler );
    mavlink_msg_sys_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYS_TM) != NULL);
#endif
}

static void mavlink_test_fsm_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_FSM_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_fsm_tm_t packet_in = {
        93372036854775807ULL,29,96,163,230,41
    };
    mavlink_fsm_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.ada_state = packet_in.ada_state;
        packet1.abk_state = packet_in.abk_state;
        packet1.dpl_state = packet_in.dpl_state;
        packet1.fmm_state = packet_in.fmm_state;
        packet1.nas_state = packet_in.nas_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_FSM_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_FSM_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fsm_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_fsm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fsm_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.ada_state , packet1.abk_state , packet1.dpl_state , packet1.fmm_state , packet1.nas_state );
    mavlink_msg_fsm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fsm_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.ada_state , packet1.abk_state , packet1.dpl_state , packet1.fmm_state , packet1.nas_state );
    mavlink_msg_fsm_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_fsm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_fsm_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.ada_state , packet1.abk_state , packet1.dpl_state , packet1.fmm_state , packet1.nas_state );
    mavlink_msg_fsm_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("FSM_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_FSM_TM) != NULL);
#endif
}

static void mavlink_test_logger_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LOGGER_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_logger_tm_t packet_in = {
        93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,963498920,963499128,963499336,963499544,19523
    };
    mavlink_logger_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.too_large_samples = packet_in.too_large_samples;
        packet1.dropped_samples = packet_in.dropped_samples;
        packet1.queued_samples = packet_in.queued_samples;
        packet1.buffers_filled = packet_in.buffers_filled;
        packet1.buffers_written = packet_in.buffers_written;
        packet1.writes_failed = packet_in.writes_failed;
        packet1.last_write_error = packet_in.last_write_error;
        packet1.average_write_time = packet_in.average_write_time;
        packet1.max_write_time = packet_in.max_write_time;
        packet1.log_number = packet_in.log_number;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LOGGER_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.log_number , packet1.too_large_samples , packet1.dropped_samples , packet1.queued_samples , packet1.buffers_filled , packet1.buffers_written , packet1.writes_failed , packet1.last_write_error , packet1.average_write_time , packet1.max_write_time );
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.log_number , packet1.too_large_samples , packet1.dropped_samples , packet1.queued_samples , packet1.buffers_filled , packet1.buffers_written , packet1.writes_failed , packet1.last_write_error , packet1.average_write_time , packet1.max_write_time );
    mavlink_msg_logger_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_logger_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_logger_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.log_number , packet1.too_large_samples , packet1.dropped_samples , packet1.queued_samples , packet1.buffers_filled , packet1.buffers_written , packet1.writes_failed , packet1.last_write_error , packet1.average_write_time , packet1.max_write_time );
    mavlink_msg_logger_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LOGGER_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LOGGER_TM) != NULL);
#endif
}

static void mavlink_test_mavlink_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAVLINK_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mavlink_stats_tm_t packet_in = {
        93372036854775807ULL,963497880,17859,17963,18067,18171,18275,199,10,77,144,211,22
    };
    mavlink_mavlink_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.parse_state = packet_in.parse_state;
        packet1.n_send_queue = packet_in.n_send_queue;
        packet1.max_send_queue = packet_in.max_send_queue;
        packet1.n_send_errors = packet_in.n_send_errors;
        packet1.packet_rx_success_count = packet_in.packet_rx_success_count;
        packet1.packet_rx_drop_count = packet_in.packet_rx_drop_count;
        packet1.msg_received = packet_in.msg_received;
        packet1.buffer_overrun = packet_in.buffer_overrun;
        packet1.parse_error = packet_in.parse_error;
        packet1.packet_idx = packet_in.packet_idx;
        packet1.current_rx_seq = packet_in.current_rx_seq;
        packet1.current_tx_seq = packet_in.current_tx_seq;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAVLINK_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAVLINK_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mavlink_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mavlink_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mavlink_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_mavlink_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mavlink_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_mavlink_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mavlink_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mavlink_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.n_send_queue , packet1.max_send_queue , packet1.n_send_errors , packet1.msg_received , packet1.buffer_overrun , packet1.parse_error , packet1.parse_state , packet1.packet_idx , packet1.current_rx_seq , packet1.current_tx_seq , packet1.packet_rx_success_count , packet1.packet_rx_drop_count );
    mavlink_msg_mavlink_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MAVLINK_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MAVLINK_STATS_TM) != NULL);
#endif
}

static void mavlink_test_task_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_TASK_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_task_stats_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,18483,211
    };
    mavlink_task_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.task_min = packet_in.task_min;
        packet1.task_max = packet_in.task_max;
        packet1.task_mean = packet_in.task_mean;
        packet1.task_stddev = packet_in.task_stddev;
        packet1.task_period = packet_in.task_period;
        packet1.task_id = packet_in.task_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_TASK_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_min , packet1.task_max , packet1.task_mean , packet1.task_stddev );
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_min , packet1.task_max , packet1.task_mean , packet1.task_stddev );
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_task_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_min , packet1.task_max , packet1.task_mean , packet1.task_stddev );
    mavlink_msg_task_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("TASK_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_TASK_STATS_TM) != NULL);
#endif
}

static void mavlink_test_ada_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ADA_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ada_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,161
    };
    mavlink_ada_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.kalman_x0 = packet_in.kalman_x0;
        packet1.kalman_x1 = packet_in.kalman_x1;
        packet1.kalman_x2 = packet_in.kalman_x2;
        packet1.vertical_speed = packet_in.vertical_speed;
        packet1.msl_altitude = packet_in.msl_altitude;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.ref_altitude = packet_in.ref_altitude;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.msl_pressure = packet_in.msl_pressure;
        packet1.msl_temperature = packet_in.msl_temperature;
        packet1.dpl_altitude = packet_in.dpl_altitude;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ADA_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ADA_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vertical_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature , packet1.dpl_altitude );
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vertical_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature , packet1.dpl_altitude );
    mavlink_msg_ada_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ada_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ada_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.vertical_speed , packet1.msl_altitude , packet1.ref_pressure , packet1.ref_altitude , packet1.ref_temperature , packet1.msl_pressure , packet1.msl_temperature , packet1.dpl_altitude );
    mavlink_msg_ada_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ADA_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ADA_TM) != NULL);
#endif
}

static void mavlink_test_nas_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_NAS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_nas_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,233
    };
    mavlink_nas_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.nas_n = packet_in.nas_n;
        packet1.nas_e = packet_in.nas_e;
        packet1.nas_d = packet_in.nas_d;
        packet1.nas_vn = packet_in.nas_vn;
        packet1.nas_ve = packet_in.nas_ve;
        packet1.nas_vd = packet_in.nas_vd;
        packet1.nas_qx = packet_in.nas_qx;
        packet1.nas_qy = packet_in.nas_qy;
        packet1.nas_qz = packet_in.nas_qz;
        packet1.nas_qw = packet_in.nas_qw;
        packet1.nas_bias_x = packet_in.nas_bias_x;
        packet1.nas_bias_y = packet_in.nas_bias_y;
        packet1.nas_bias_z = packet_in.nas_bias_z;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.ref_latitude = packet_in.ref_latitude;
        packet1.ref_longitude = packet_in.ref_longitude;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_NAS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_NAS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude );
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude );
    mavlink_msg_nas_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_nas_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nas_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude );
    mavlink_msg_nas_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NAS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NAS_TM) != NULL);
#endif
}

static void mavlink_test_rocket_flight_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROCKET_FLIGHT_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rocket_flight_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,1081.0,217,28,95,162,229,40,107,174,241,52,119
    };
    mavlink_rocket_flight_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_ada = packet_in.pressure_ada;
        packet1.pressure_digi = packet_in.pressure_digi;
        packet1.pressure_static = packet_in.pressure_static;
        packet1.pressure_dpl = packet_in.pressure_dpl;
        packet1.airspeed_pitot = packet_in.airspeed_pitot;
        packet1.altitude_agl = packet_in.altitude_agl;
        packet1.ada_vert_speed = packet_in.ada_vert_speed;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.gps_lat = packet_in.gps_lat;
        packet1.gps_lon = packet_in.gps_lon;
        packet1.gps_alt = packet_in.gps_alt;
        packet1.abk_angle = packet_in.abk_angle;
        packet1.abk_estimated_cd = packet_in.abk_estimated_cd;
        packet1.parachute_load = packet_in.parachute_load;
        packet1.nas_n = packet_in.nas_n;
        packet1.nas_e = packet_in.nas_e;
        packet1.nas_d = packet_in.nas_d;
        packet1.nas_vn = packet_in.nas_vn;
        packet1.nas_ve = packet_in.nas_ve;
        packet1.nas_vd = packet_in.nas_vd;
        packet1.nas_qx = packet_in.nas_qx;
        packet1.nas_qy = packet_in.nas_qy;
        packet1.nas_qz = packet_in.nas_qz;
        packet1.nas_qw = packet_in.nas_qw;
        packet1.nas_bias_x = packet_in.nas_bias_x;
        packet1.nas_bias_y = packet_in.nas_bias_y;
        packet1.nas_bias_z = packet_in.nas_bias_z;
        packet1.vbat = packet_in.vbat;
        packet1.temperature = packet_in.temperature;
        packet1.ada_state = packet_in.ada_state;
        packet1.fmm_state = packet_in.fmm_state;
        packet1.dpl_state = packet_in.dpl_state;
        packet1.abk_state = packet_in.abk_state;
        packet1.nas_state = packet_in.nas_state;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.pin_launch = packet_in.pin_launch;
        packet1.pin_nosecone = packet_in.pin_nosecone;
        packet1.pin_expulsion = packet_in.pin_expulsion;
        packet1.cutter_presence = packet_in.cutter_presence;
        packet1.logger_error = packet_in.logger_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROCKET_FLIGHT_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_flight_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rocket_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_flight_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.abk_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.abk_angle , packet1.abk_estimated_cd , packet1.parachute_load , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.vbat , packet1.temperature , packet1.logger_error );
    mavlink_msg_rocket_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_flight_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.abk_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.abk_angle , packet1.abk_estimated_cd , packet1.parachute_load , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.vbat , packet1.temperature , packet1.logger_error );
    mavlink_msg_rocket_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rocket_flight_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_flight_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.dpl_state , packet1.abk_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.abk_angle , packet1.abk_estimated_cd , packet1.parachute_load , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.vbat , packet1.temperature , packet1.logger_error );
    mavlink_msg_rocket_flight_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROCKET_FLIGHT_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROCKET_FLIGHT_TM) != NULL);
#endif
}

static void mavlink_test_payload_flight_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_payload_flight_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,205,16,83,150,217,28
    };
    mavlink_payload_flight_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_ada = packet_in.pressure_ada;
        packet1.pressure_digi = packet_in.pressure_digi;
        packet1.pressure_static = packet_in.pressure_static;
        packet1.pressure_dpl = packet_in.pressure_dpl;
        packet1.airspeed_pitot = packet_in.airspeed_pitot;
        packet1.altitude_agl = packet_in.altitude_agl;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.gps_lat = packet_in.gps_lat;
        packet1.gps_lon = packet_in.gps_lon;
        packet1.gps_alt = packet_in.gps_alt;
        packet1.left_servo_angle = packet_in.left_servo_angle;
        packet1.right_servo_angle = packet_in.right_servo_angle;
        packet1.nas_n = packet_in.nas_n;
        packet1.nas_e = packet_in.nas_e;
        packet1.nas_d = packet_in.nas_d;
        packet1.nas_vn = packet_in.nas_vn;
        packet1.nas_ve = packet_in.nas_ve;
        packet1.nas_vd = packet_in.nas_vd;
        packet1.nas_qx = packet_in.nas_qx;
        packet1.nas_qy = packet_in.nas_qy;
        packet1.nas_qz = packet_in.nas_qz;
        packet1.nas_qw = packet_in.nas_qw;
        packet1.nas_bias_x = packet_in.nas_bias_x;
        packet1.nas_bias_y = packet_in.nas_bias_y;
        packet1.nas_bias_z = packet_in.nas_bias_z;
        packet1.vbat = packet_in.vbat;
        packet1.vsupply_5v = packet_in.vsupply_5v;
        packet1.temperature = packet_in.temperature;
        packet1.ada_state = packet_in.ada_state;
        packet1.fmm_state = packet_in.fmm_state;
        packet1.nas_state = packet_in.nas_state;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.pin_nosecone = packet_in.pin_nosecone;
        packet1.logger_error = packet_in.logger_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_flight_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_payload_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_flight_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_nosecone , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.logger_error );
    mavlink_msg_payload_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_flight_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_nosecone , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.logger_error );
    mavlink_msg_payload_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_payload_flight_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_flight_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.ada_state , packet1.fmm_state , packet1.nas_state , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.pin_nosecone , packet1.vbat , packet1.vsupply_5v , packet1.temperature , packet1.logger_error );
    mavlink_msg_payload_flight_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PAYLOAD_FLIGHT_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM) != NULL);
#endif
}

static void mavlink_test_rocket_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROCKET_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rocket_stats_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,93372036854777823ULL,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,963502040
    };
    mavlink_rocket_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.liftoff_ts = packet_in.liftoff_ts;
        packet1.liftoff_max_acc_ts = packet_in.liftoff_max_acc_ts;
        packet1.dpl_ts = packet_in.dpl_ts;
        packet1.max_z_speed_ts = packet_in.max_z_speed_ts;
        packet1.apogee_ts = packet_in.apogee_ts;
        packet1.liftoff_max_acc = packet_in.liftoff_max_acc;
        packet1.dpl_max_acc = packet_in.dpl_max_acc;
        packet1.max_z_speed = packet_in.max_z_speed;
        packet1.max_airspeed_pitot = packet_in.max_airspeed_pitot;
        packet1.max_speed_altitude = packet_in.max_speed_altitude;
        packet1.apogee_lat = packet_in.apogee_lat;
        packet1.apogee_lon = packet_in.apogee_lon;
        packet1.apogee_alt = packet_in.apogee_alt;
        packet1.min_pressure = packet_in.min_pressure;
        packet1.ada_min_pressure = packet_in.ada_min_pressure;
        packet1.dpl_vane_max_pressure = packet_in.dpl_vane_max_pressure;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROCKET_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rocket_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_stats_tm_pack(system_id, component_id, &msg , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.ada_min_pressure , packet1.dpl_vane_max_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_rocket_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.ada_min_pressure , packet1.dpl_vane_max_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_rocket_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rocket_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_stats_tm_send(MAVLINK_COMM_1 , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.ada_min_pressure , packet1.dpl_vane_max_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_rocket_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROCKET_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROCKET_STATS_TM) != NULL);
#endif
}

static void mavlink_test_payload_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PAYLOAD_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_payload_stats_tm_t packet_in = {
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,963501208
    };
    mavlink_payload_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.liftoff_max_acc_ts = packet_in.liftoff_max_acc_ts;
        packet1.dpl_ts = packet_in.dpl_ts;
        packet1.max_z_speed_ts = packet_in.max_z_speed_ts;
        packet1.apogee_ts = packet_in.apogee_ts;
        packet1.liftoff_max_acc = packet_in.liftoff_max_acc;
        packet1.dpl_max_acc = packet_in.dpl_max_acc;
        packet1.max_z_speed = packet_in.max_z_speed;
        packet1.max_airspeed_pitot = packet_in.max_airspeed_pitot;
        packet1.max_speed_altitude = packet_in.max_speed_altitude;
        packet1.apogee_lat = packet_in.apogee_lat;
        packet1.apogee_lon = packet_in.apogee_lon;
        packet1.apogee_alt = packet_in.apogee_alt;
        packet1.min_pressure = packet_in.min_pressure;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PAYLOAD_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_payload_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_stats_tm_pack(system_id, component_id, &msg , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_payload_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_payload_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_payload_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_stats_tm_send(MAVLINK_COMM_1 , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.dpl_ts , packet1.dpl_max_acc , packet1.max_z_speed_ts , packet1.max_z_speed , packet1.max_airspeed_pitot , packet1.max_speed_altitude , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap );
    mavlink_msg_payload_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PAYLOAD_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PAYLOAD_STATS_TM) != NULL);
#endif
}

static void mavlink_test_pyxis(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ping_tc(system_id, component_id, last_msg);
    mavlink_test_command_tc(system_id, component_id, last_msg);
    mavlink_test_system_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_sensor_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_servo_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_set_servo_angle_tc(system_id, component_id, last_msg);
    mavlink_test_wiggle_servo_tc(system_id, component_id, last_msg);
    mavlink_test_reset_servo_tc(system_id, component_id, last_msg);
    mavlink_test_set_reference_altitude_tc(system_id, component_id, last_msg);
    mavlink_test_set_reference_temperature_tc(system_id, component_id, last_msg);
    mavlink_test_set_orientation_tc(system_id, component_id, last_msg);
    mavlink_test_set_coordinates_tc(system_id, component_id, last_msg);
    mavlink_test_raw_event_tc(system_id, component_id, last_msg);
    mavlink_test_set_deployment_altitude_tc(system_id, component_id, last_msg);
    mavlink_test_set_target_coordinates_tc(system_id, component_id, last_msg);
    mavlink_test_set_algorithm_tc(system_id, component_id, last_msg);
    mavlink_test_ack_tm(system_id, component_id, last_msg);
    mavlink_test_nack_tm(system_id, component_id, last_msg);
    mavlink_test_gps_tm(system_id, component_id, last_msg);
    mavlink_test_imu_tm(system_id, component_id, last_msg);
    mavlink_test_pressure_tm(system_id, component_id, last_msg);
    mavlink_test_adc_tm(system_id, component_id, last_msg);
    mavlink_test_voltage_tm(system_id, component_id, last_msg);
    mavlink_test_current_tm(system_id, component_id, last_msg);
    mavlink_test_temp_tm(system_id, component_id, last_msg);
    mavlink_test_load_tm(system_id, component_id, last_msg);
    mavlink_test_attitude_tm(system_id, component_id, last_msg);
    mavlink_test_sensor_state_tm(system_id, component_id, last_msg);
    mavlink_test_servo_tm(system_id, component_id, last_msg);
    mavlink_test_pin_tm(system_id, component_id, last_msg);
    mavlink_test_receiver_tm(system_id, component_id, last_msg);
    mavlink_test_sys_tm(system_id, component_id, last_msg);
    mavlink_test_fsm_tm(system_id, component_id, last_msg);
    mavlink_test_logger_tm(system_id, component_id, last_msg);
    mavlink_test_mavlink_stats_tm(system_id, component_id, last_msg);
    mavlink_test_task_stats_tm(system_id, component_id, last_msg);
    mavlink_test_ada_tm(system_id, component_id, last_msg);
    mavlink_test_nas_tm(system_id, component_id, last_msg);
    mavlink_test_rocket_flight_tm(system_id, component_id, last_msg);
    mavlink_test_payload_flight_tm(system_id, component_id, last_msg);
    mavlink_test_rocket_stats_tm(system_id, component_id, last_msg);
    mavlink_test_payload_stats_tm(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // PYXIS_TESTSUITE_H
