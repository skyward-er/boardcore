/** @file
 *    @brief MAVLink comm protocol testsuite generated from lyra.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef LYRA_TESTSUITE_H
#define LYRA_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_lyra(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_lyra(system_id, component_id, last_msg);
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
        packet1.sensor_name = packet_in.sensor_name;
        
        
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
    mavlink_msg_sensor_tm_request_tc_pack(system_id, component_id, &msg , packet1.sensor_name );
    mavlink_msg_sensor_tm_request_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_tm_request_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensor_name );
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
    mavlink_msg_sensor_tm_request_tc_send(MAVLINK_COMM_1 , packet1.sensor_name );
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

static void mavlink_test_set_orientation_quat_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_orientation_quat_tc_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_set_orientation_quat_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.quat_x = packet_in.quat_x;
        packet1.quat_y = packet_in.quat_y;
        packet1.quat_z = packet_in.quat_z;
        packet1.quat_w = packet_in.quat_w;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_quat_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_orientation_quat_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_quat_tc_pack(system_id, component_id, &msg , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_set_orientation_quat_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_quat_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_set_orientation_quat_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_orientation_quat_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_orientation_quat_tc_send(MAVLINK_COMM_1 , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_set_orientation_quat_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ORIENTATION_QUAT_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ORIENTATION_QUAT_TC) != NULL);
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

static void mavlink_test_set_calibration_pressure_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_calibration_pressure_tc_t packet_in = {
        17.0
    };
    mavlink_set_calibration_pressure_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pressure = packet_in.pressure;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_calibration_pressure_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_calibration_pressure_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_calibration_pressure_tc_pack(system_id, component_id, &msg , packet1.pressure );
    mavlink_msg_set_calibration_pressure_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_calibration_pressure_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pressure );
    mavlink_msg_set_calibration_pressure_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_calibration_pressure_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_calibration_pressure_tc_send(MAVLINK_COMM_1 , packet1.pressure );
    mavlink_msg_set_calibration_pressure_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_CALIBRATION_PRESSURE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_CALIBRATION_PRESSURE_TC) != NULL);
#endif
}

static void mavlink_test_set_initial_mea_mass_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_initial_mea_mass_tc_t packet_in = {
        17.0
    };
    mavlink_set_initial_mea_mass_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.mass = packet_in.mass;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_mea_mass_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_initial_mea_mass_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_mea_mass_tc_pack(system_id, component_id, &msg , packet1.mass );
    mavlink_msg_set_initial_mea_mass_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_mea_mass_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.mass );
    mavlink_msg_set_initial_mea_mass_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_initial_mea_mass_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_initial_mea_mass_tc_send(MAVLINK_COMM_1 , packet1.mass );
    mavlink_msg_set_initial_mea_mass_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_INITIAL_MEA_MASS_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_INITIAL_MEA_MASS_TC) != NULL);
#endif
}

static void mavlink_test_set_atomic_valve_timing_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_atomic_valve_timing_tc_t packet_in = {
        963497464,17
    };
    mavlink_set_atomic_valve_timing_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.maximum_timing = packet_in.maximum_timing;
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_atomic_valve_timing_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_atomic_valve_timing_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_atomic_valve_timing_tc_pack(system_id, component_id, &msg , packet1.servo_id , packet1.maximum_timing );
    mavlink_msg_set_atomic_valve_timing_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_atomic_valve_timing_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id , packet1.maximum_timing );
    mavlink_msg_set_atomic_valve_timing_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_atomic_valve_timing_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_atomic_valve_timing_tc_send(MAVLINK_COMM_1 , packet1.servo_id , packet1.maximum_timing );
    mavlink_msg_set_atomic_valve_timing_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ATOMIC_VALVE_TIMING_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ATOMIC_VALVE_TIMING_TC) != NULL);
#endif
}

static void mavlink_test_set_valve_maximum_aperture_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_valve_maximum_aperture_tc_t packet_in = {
        17.0,17
    };
    mavlink_set_valve_maximum_aperture_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.maximum_aperture = packet_in.maximum_aperture;
        packet1.servo_id = packet_in.servo_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_valve_maximum_aperture_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_valve_maximum_aperture_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_valve_maximum_aperture_tc_pack(system_id, component_id, &msg , packet1.servo_id , packet1.maximum_aperture );
    mavlink_msg_set_valve_maximum_aperture_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_valve_maximum_aperture_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servo_id , packet1.maximum_aperture );
    mavlink_msg_set_valve_maximum_aperture_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_valve_maximum_aperture_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_valve_maximum_aperture_tc_send(MAVLINK_COMM_1 , packet1.servo_id , packet1.maximum_aperture );
    mavlink_msg_set_valve_maximum_aperture_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_VALVE_MAXIMUM_APERTURE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_VALVE_MAXIMUM_APERTURE_TC) != NULL);
#endif
}

static void mavlink_test_conrig_state_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONRIG_STATE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_conrig_state_tc_t packet_in = {
        5,72,139,206,17,84,151
    };
    mavlink_conrig_state_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.ignition_btn = packet_in.ignition_btn;
        packet1.filling_valve_btn = packet_in.filling_valve_btn;
        packet1.venting_valve_btn = packet_in.venting_valve_btn;
        packet1.release_pressure_btn = packet_in.release_pressure_btn;
        packet1.quick_connector_btn = packet_in.quick_connector_btn;
        packet1.start_tars_btn = packet_in.start_tars_btn;
        packet1.arm_switch = packet_in.arm_switch;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONRIG_STATE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_conrig_state_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_conrig_state_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_conrig_state_tc_pack(system_id, component_id, &msg , packet1.ignition_btn , packet1.filling_valve_btn , packet1.venting_valve_btn , packet1.release_pressure_btn , packet1.quick_connector_btn , packet1.start_tars_btn , packet1.arm_switch );
    mavlink_msg_conrig_state_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_conrig_state_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ignition_btn , packet1.filling_valve_btn , packet1.venting_valve_btn , packet1.release_pressure_btn , packet1.quick_connector_btn , packet1.start_tars_btn , packet1.arm_switch );
    mavlink_msg_conrig_state_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_conrig_state_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_conrig_state_tc_send(MAVLINK_COMM_1 , packet1.ignition_btn , packet1.filling_valve_btn , packet1.venting_valve_btn , packet1.release_pressure_btn , packet1.quick_connector_btn , packet1.start_tars_btn , packet1.arm_switch );
    mavlink_msg_conrig_state_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CONRIG_STATE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CONRIG_STATE_TC) != NULL);
#endif
}

static void mavlink_test_set_ignition_time_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_IGNITION_TIME_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_ignition_time_tc_t packet_in = {
        963497464
    };
    mavlink_set_ignition_time_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timing = packet_in.timing;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_IGNITION_TIME_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_IGNITION_TIME_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_ignition_time_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_ignition_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_ignition_time_tc_pack(system_id, component_id, &msg , packet1.timing );
    mavlink_msg_set_ignition_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_ignition_time_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timing );
    mavlink_msg_set_ignition_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_ignition_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_ignition_time_tc_send(MAVLINK_COMM_1 , packet1.timing );
    mavlink_msg_set_ignition_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_IGNITION_TIME_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_IGNITION_TIME_TC) != NULL);
#endif
}

static void mavlink_test_set_nitrogen_time_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_NITROGEN_TIME_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_nitrogen_time_tc_t packet_in = {
        963497464
    };
    mavlink_set_nitrogen_time_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timing = packet_in.timing;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_NITROGEN_TIME_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_NITROGEN_TIME_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_nitrogen_time_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_nitrogen_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_nitrogen_time_tc_pack(system_id, component_id, &msg , packet1.timing );
    mavlink_msg_set_nitrogen_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_nitrogen_time_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timing );
    mavlink_msg_set_nitrogen_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_nitrogen_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_nitrogen_time_tc_send(MAVLINK_COMM_1 , packet1.timing );
    mavlink_msg_set_nitrogen_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_NITROGEN_TIME_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_NITROGEN_TIME_TC) != NULL);
#endif
}

static void mavlink_test_set_cooling_time_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_COOLING_TIME_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_cooling_time_tc_t packet_in = {
        963497464
    };
    mavlink_set_cooling_time_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timing = packet_in.timing;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_COOLING_TIME_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_COOLING_TIME_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_cooling_time_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_cooling_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_cooling_time_tc_pack(system_id, component_id, &msg , packet1.timing );
    mavlink_msg_set_cooling_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_cooling_time_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timing );
    mavlink_msg_set_cooling_time_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_cooling_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_cooling_time_tc_send(MAVLINK_COMM_1 , packet1.timing );
    mavlink_msg_set_cooling_time_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_COOLING_TIME_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_COOLING_TIME_TC) != NULL);
#endif
}

static void mavlink_test_set_stepper_angle_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_stepper_angle_tc_t packet_in = {
        17.0,17
    };
    mavlink_set_stepper_angle_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.angle = packet_in.angle;
        packet1.stepper_id = packet_in.stepper_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_angle_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_stepper_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_angle_tc_pack(system_id, component_id, &msg , packet1.stepper_id , packet1.angle );
    mavlink_msg_set_stepper_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_angle_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stepper_id , packet1.angle );
    mavlink_msg_set_stepper_angle_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_stepper_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_angle_tc_send(MAVLINK_COMM_1 , packet1.stepper_id , packet1.angle );
    mavlink_msg_set_stepper_angle_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_STEPPER_ANGLE_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_STEPPER_ANGLE_TC) != NULL);
#endif
}

static void mavlink_test_set_stepper_steps_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_STEPPER_STEPS_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_stepper_steps_tc_t packet_in = {
        17.0,17
    };
    mavlink_set_stepper_steps_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.steps = packet_in.steps;
        packet1.stepper_id = packet_in.stepper_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_STEPPER_STEPS_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_STEPPER_STEPS_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_steps_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_stepper_steps_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_steps_tc_pack(system_id, component_id, &msg , packet1.stepper_id , packet1.steps );
    mavlink_msg_set_stepper_steps_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_steps_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stepper_id , packet1.steps );
    mavlink_msg_set_stepper_steps_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_stepper_steps_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_steps_tc_send(MAVLINK_COMM_1 , packet1.stepper_id , packet1.steps );
    mavlink_msg_set_stepper_steps_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_STEPPER_STEPS_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_STEPPER_STEPS_TC) != NULL);
#endif
}

static void mavlink_test_set_stepper_multiplier_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_STEPPER_MULTIPLIER_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_stepper_multiplier_tc_t packet_in = {
        17.0,17
    };
    mavlink_set_stepper_multiplier_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.multiplier = packet_in.multiplier;
        packet1.stepper_id = packet_in.stepper_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_STEPPER_MULTIPLIER_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_STEPPER_MULTIPLIER_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_multiplier_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_stepper_multiplier_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_multiplier_tc_pack(system_id, component_id, &msg , packet1.stepper_id , packet1.multiplier );
    mavlink_msg_set_stepper_multiplier_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_multiplier_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.stepper_id , packet1.multiplier );
    mavlink_msg_set_stepper_multiplier_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_stepper_multiplier_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_stepper_multiplier_tc_send(MAVLINK_COMM_1 , packet1.stepper_id , packet1.multiplier );
    mavlink_msg_set_stepper_multiplier_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_STEPPER_MULTIPLIER_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_STEPPER_MULTIPLIER_TC) != NULL);
#endif
}

static void mavlink_test_set_antenna_coordinates_arp_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ANTENNA_COORDINATES_ARP_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_antenna_coordinates_arp_tc_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_set_antenna_coordinates_arp_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.height = packet_in.height;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ANTENNA_COORDINATES_ARP_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ANTENNA_COORDINATES_ARP_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_antenna_coordinates_arp_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_antenna_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_antenna_coordinates_arp_tc_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_antenna_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_antenna_coordinates_arp_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_antenna_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_antenna_coordinates_arp_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_antenna_coordinates_arp_tc_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_antenna_coordinates_arp_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ANTENNA_COORDINATES_ARP_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ANTENNA_COORDINATES_ARP_TC) != NULL);
#endif
}

static void mavlink_test_set_rocket_coordinates_arp_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_set_rocket_coordinates_arp_tc_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_set_rocket_coordinates_arp_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.height = packet_in.height;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rocket_coordinates_arp_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_set_rocket_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rocket_coordinates_arp_tc_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_rocket_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rocket_coordinates_arp_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_rocket_coordinates_arp_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_set_rocket_coordinates_arp_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_set_rocket_coordinates_arp_tc_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.height );
    mavlink_msg_set_rocket_coordinates_arp_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SET_ROCKET_COORDINATES_ARP_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SET_ROCKET_COORDINATES_ARP_TC) != NULL);
#endif
}

static void mavlink_test_arp_command_tc(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARP_COMMAND_TC >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_arp_command_tc_t packet_in = {
        5
    };
    mavlink_arp_command_tc_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.command_id = packet_in.command_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARP_COMMAND_TC_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARP_COMMAND_TC_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_command_tc_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_arp_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_command_tc_pack(system_id, component_id, &msg , packet1.command_id );
    mavlink_msg_arp_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_command_tc_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.command_id );
    mavlink_msg_arp_command_tc_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_arp_command_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_command_tc_send(MAVLINK_COMM_1 , packet1.command_id );
    mavlink_msg_arp_command_tc_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARP_COMMAND_TC") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARP_COMMAND_TC) != NULL);
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
        17235,139,206
    };
    mavlink_nack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.err_id = packet_in.err_id;
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
    mavlink_msg_nack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
    mavlink_msg_nack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_nack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
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
    mavlink_msg_nack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
    mavlink_msg_nack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("NACK_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_NACK_TM) != NULL);
#endif
}

static void mavlink_test_wack_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_WACK_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_wack_tm_t packet_in = {
        17235,139,206
    };
    mavlink_wack_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.err_id = packet_in.err_id;
        packet1.recv_msgid = packet_in.recv_msgid;
        packet1.seq_ack = packet_in.seq_ack;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_WACK_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_WACK_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wack_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_wack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wack_tm_pack(system_id, component_id, &msg , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
    mavlink_msg_wack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wack_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
    mavlink_msg_wack_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_wack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_wack_tm_send(MAVLINK_COMM_1 , packet1.recv_msgid , packet1.seq_ack , packet1.err_id );
    mavlink_msg_wack_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("WACK_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_WACK_TM) != NULL);
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_gps_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
    mavlink_msg_gps_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gps_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
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
    mavlink_msg_gps_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.fix , packet1.latitude , packet1.longitude , packet1.height , packet1.vel_north , packet1.vel_east , packet1.vel_down , packet1.speed , packet1.track , packet1.n_satellites );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_imu_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
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
    mavlink_msg_imu_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_pressure_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.pressure );
    mavlink_msg_pressure_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.pressure );
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
    mavlink_msg_pressure_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.pressure );
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
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,"OPQRSTUVWXYZABCDEFG"
    };
    mavlink_adc_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.channel_0 = packet_in.channel_0;
        packet1.channel_1 = packet_in.channel_1;
        packet1.channel_2 = packet_in.channel_2;
        packet1.channel_3 = packet_in.channel_3;
        packet1.channel_4 = packet_in.channel_4;
        packet1.channel_5 = packet_in.channel_5;
        packet1.channel_6 = packet_in.channel_6;
        packet1.channel_7 = packet_in.channel_7;
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_adc_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.channel_5 , packet1.channel_6 , packet1.channel_7 );
    mavlink_msg_adc_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_adc_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.channel_5 , packet1.channel_6 , packet1.channel_7 );
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
    mavlink_msg_adc_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.channel_0 , packet1.channel_1 , packet1.channel_2 , packet1.channel_3 , packet1.channel_4 , packet1.channel_5 , packet1.channel_6 , packet1.channel_7 );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_voltage_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.voltage );
    mavlink_msg_voltage_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_voltage_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.voltage );
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
    mavlink_msg_voltage_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.voltage );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_current_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.current );
    mavlink_msg_current_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_current_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.current );
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
    mavlink_msg_current_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.current );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_temp_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.temperature );
    mavlink_msg_temp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_temp_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.temperature );
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
    mavlink_msg_temp_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.temperature );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_load_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.load );
    mavlink_msg_load_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_load_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.load );
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
    mavlink_msg_load_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.load );
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
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_attitude_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.sensor_name , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
    mavlink_msg_attitude_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.sensor_name , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
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
    mavlink_msg_attitude_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.sensor_name , packet1.roll , packet1.pitch , packet1.yaw , packet1.quat_x , packet1.quat_y , packet1.quat_z , packet1.quat_w );
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
        "ABCDEFGHIJKLMNOPQRS",65,132
    };
    mavlink_sensor_state_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.initialized = packet_in.initialized;
        packet1.enabled = packet_in.enabled;
        
        mav_array_memcpy(packet1.sensor_name, packet_in.sensor_name, sizeof(char)*20);
        
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
    mavlink_msg_sensor_state_tm_pack(system_id, component_id, &msg , packet1.sensor_name , packet1.initialized , packet1.enabled );
    mavlink_msg_sensor_state_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sensor_state_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.sensor_name , packet1.initialized , packet1.enabled );
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
    mavlink_msg_sensor_state_tm_send(MAVLINK_COMM_1 , packet1.sensor_name , packet1.initialized , packet1.enabled );
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

static void mavlink_test_reference_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REFERENCE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_reference_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0
    };
    mavlink_reference_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.ref_altitude = packet_in.ref_altitude;
        packet1.ref_pressure = packet_in.ref_pressure;
        packet1.ref_temperature = packet_in.ref_temperature;
        packet1.ref_latitude = packet_in.ref_latitude;
        packet1.ref_longitude = packet_in.ref_longitude;
        packet1.msl_pressure = packet_in.msl_pressure;
        packet1.msl_temperature = packet_in.msl_temperature;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REFERENCE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reference_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_reference_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reference_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.ref_altitude , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_reference_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reference_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.ref_altitude , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_reference_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_reference_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_reference_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.ref_altitude , packet1.ref_pressure , packet1.ref_temperature , packet1.ref_latitude , packet1.ref_longitude , packet1.msl_pressure , packet1.msl_temperature );
    mavlink_msg_reference_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REFERENCE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REFERENCE_TM) != NULL);
#endif
}

static void mavlink_test_registry_float_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REGISTRY_FLOAT_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_registry_float_tm_t packet_in = {
        93372036854775807ULL,963497880,101.0,"QRSTUVWXYZABCDEFGHI"
    };
    mavlink_registry_float_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.key_id = packet_in.key_id;
        packet1.value = packet_in.value;
        
        mav_array_memcpy(packet1.key_name, packet_in.key_name, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REGISTRY_FLOAT_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REGISTRY_FLOAT_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_float_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_registry_float_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_float_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_float_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_float_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_float_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_registry_float_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_float_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_float_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REGISTRY_FLOAT_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REGISTRY_FLOAT_TM) != NULL);
#endif
}

static void mavlink_test_registry_int_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REGISTRY_INT_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_registry_int_tm_t packet_in = {
        93372036854775807ULL,963497880,963498088,"QRSTUVWXYZABCDEFGHI"
    };
    mavlink_registry_int_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.key_id = packet_in.key_id;
        packet1.value = packet_in.value;
        
        mav_array_memcpy(packet1.key_name, packet_in.key_name, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REGISTRY_INT_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REGISTRY_INT_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_int_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_registry_int_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_int_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_int_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_int_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_int_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_registry_int_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_int_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.value );
    mavlink_msg_registry_int_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REGISTRY_INT_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REGISTRY_INT_TM) != NULL);
#endif
}

static void mavlink_test_registry_coord_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_REGISTRY_COORD_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_registry_coord_tm_t packet_in = {
        93372036854775807ULL,963497880,101.0,129.0,"UVWXYZABCDEFGHIJKLM"
    };
    mavlink_registry_coord_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.key_id = packet_in.key_id;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        
        mav_array_memcpy(packet1.key_name, packet_in.key_name, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_REGISTRY_COORD_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_coord_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_registry_coord_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_coord_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.latitude , packet1.longitude );
    mavlink_msg_registry_coord_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_coord_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.latitude , packet1.longitude );
    mavlink_msg_registry_coord_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_registry_coord_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_registry_coord_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.key_id , packet1.key_name , packet1.latitude , packet1.longitude );
    mavlink_msg_registry_coord_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("REGISTRY_COORD_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_REGISTRY_COORD_TM) != NULL);
#endif
}

static void mavlink_test_arp_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ARP_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_arp_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,21187,21291,21395,21499,21603,21707,21811,21915,22019,22123,22227,171,238,49,116,183,250
    };
    mavlink_arp_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.yaw = packet_in.yaw;
        packet1.pitch = packet_in.pitch;
        packet1.roll = packet_in.roll;
        packet1.target_yaw = packet_in.target_yaw;
        packet1.target_pitch = packet_in.target_pitch;
        packet1.stepperX_pos = packet_in.stepperX_pos;
        packet1.stepperX_delta = packet_in.stepperX_delta;
        packet1.stepperX_speed = packet_in.stepperX_speed;
        packet1.stepperY_pos = packet_in.stepperY_pos;
        packet1.stepperY_delta = packet_in.stepperY_delta;
        packet1.stepperY_speed = packet_in.stepperY_speed;
        packet1.gps_latitude = packet_in.gps_latitude;
        packet1.gps_longitude = packet_in.gps_longitude;
        packet1.gps_height = packet_in.gps_height;
        packet1.main_rx_rssi = packet_in.main_rx_rssi;
        packet1.payload_rx_rssi = packet_in.payload_rx_rssi;
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.main_packet_tx_error_count = packet_in.main_packet_tx_error_count;
        packet1.main_tx_bitrate = packet_in.main_tx_bitrate;
        packet1.main_packet_rx_success_count = packet_in.main_packet_rx_success_count;
        packet1.main_packet_rx_drop_count = packet_in.main_packet_rx_drop_count;
        packet1.main_rx_bitrate = packet_in.main_rx_bitrate;
        packet1.payload_packet_tx_error_count = packet_in.payload_packet_tx_error_count;
        packet1.payload_tx_bitrate = packet_in.payload_tx_bitrate;
        packet1.payload_packet_rx_success_count = packet_in.payload_packet_rx_success_count;
        packet1.payload_packet_rx_drop_count = packet_in.payload_packet_rx_drop_count;
        packet1.payload_rx_bitrate = packet_in.payload_rx_bitrate;
        packet1.log_number = packet_in.log_number;
        packet1.state = packet_in.state;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.main_radio_present = packet_in.main_radio_present;
        packet1.payload_radio_present = packet_in.payload_radio_present;
        packet1.ethernet_present = packet_in.ethernet_present;
        packet1.ethernet_status = packet_in.ethernet_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ARP_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ARP_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_arp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.yaw , packet1.pitch , packet1.roll , packet1.target_yaw , packet1.target_pitch , packet1.stepperX_pos , packet1.stepperX_delta , packet1.stepperX_speed , packet1.stepperY_pos , packet1.stepperY_delta , packet1.stepperY_speed , packet1.gps_latitude , packet1.gps_longitude , packet1.gps_height , packet1.gps_fix , packet1.main_radio_present , packet1.main_packet_tx_error_count , packet1.main_tx_bitrate , packet1.main_packet_rx_success_count , packet1.main_packet_rx_drop_count , packet1.main_rx_bitrate , packet1.main_rx_rssi , packet1.payload_radio_present , packet1.payload_packet_tx_error_count , packet1.payload_tx_bitrate , packet1.payload_packet_rx_success_count , packet1.payload_packet_rx_drop_count , packet1.payload_rx_bitrate , packet1.payload_rx_rssi , packet1.ethernet_present , packet1.ethernet_status , packet1.battery_voltage , packet1.log_number );
    mavlink_msg_arp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.yaw , packet1.pitch , packet1.roll , packet1.target_yaw , packet1.target_pitch , packet1.stepperX_pos , packet1.stepperX_delta , packet1.stepperX_speed , packet1.stepperY_pos , packet1.stepperY_delta , packet1.stepperY_speed , packet1.gps_latitude , packet1.gps_longitude , packet1.gps_height , packet1.gps_fix , packet1.main_radio_present , packet1.main_packet_tx_error_count , packet1.main_tx_bitrate , packet1.main_packet_rx_success_count , packet1.main_packet_rx_drop_count , packet1.main_rx_bitrate , packet1.main_rx_rssi , packet1.payload_radio_present , packet1.payload_packet_tx_error_count , packet1.payload_tx_bitrate , packet1.payload_packet_rx_success_count , packet1.payload_packet_rx_drop_count , packet1.payload_rx_bitrate , packet1.payload_rx_rssi , packet1.ethernet_present , packet1.ethernet_status , packet1.battery_voltage , packet1.log_number );
    mavlink_msg_arp_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_arp_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_arp_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.yaw , packet1.pitch , packet1.roll , packet1.target_yaw , packet1.target_pitch , packet1.stepperX_pos , packet1.stepperX_delta , packet1.stepperX_speed , packet1.stepperY_pos , packet1.stepperY_delta , packet1.stepperY_speed , packet1.gps_latitude , packet1.gps_longitude , packet1.gps_height , packet1.gps_fix , packet1.main_radio_present , packet1.main_packet_tx_error_count , packet1.main_tx_bitrate , packet1.main_packet_rx_success_count , packet1.main_packet_rx_drop_count , packet1.main_rx_bitrate , packet1.main_rx_rssi , packet1.payload_radio_present , packet1.payload_packet_tx_error_count , packet1.payload_tx_bitrate , packet1.payload_packet_rx_success_count , packet1.payload_packet_rx_drop_count , packet1.payload_rx_bitrate , packet1.payload_rx_rssi , packet1.ethernet_present , packet1.ethernet_status , packet1.battery_voltage , packet1.log_number );
    mavlink_msg_arp_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ARP_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ARP_TM) != NULL);
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
        93372036854775807ULL,29,96,163,230,41,108,175,242
    };
    mavlink_sys_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.logger = packet_in.logger;
        packet1.event_broker = packet_in.event_broker;
        packet1.radio = packet_in.radio;
        packet1.sensors = packet_in.sensors;
        packet1.actuators = packet_in.actuators;
        packet1.pin_handler = packet_in.pin_handler;
        packet1.can_handler = packet_in.can_handler;
        packet1.scheduler = packet_in.scheduler;
        
        
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
    mavlink_msg_sys_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.sensors , packet1.actuators , packet1.pin_handler , packet1.can_handler , packet1.scheduler );
    mavlink_msg_sys_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sys_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.sensors , packet1.actuators , packet1.pin_handler , packet1.can_handler , packet1.scheduler );
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
    mavlink_msg_sys_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.logger , packet1.event_broker , packet1.radio , packet1.sensors , packet1.actuators , packet1.pin_handler , packet1.can_handler , packet1.scheduler );
    mavlink_msg_sys_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SYS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SYS_TM) != NULL);
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

static void mavlink_test_can_stats_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CAN_STATS_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_can_stats_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0
    };
    mavlink_can_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.payload_bit_rate = packet_in.payload_bit_rate;
        packet1.total_bit_rate = packet_in.total_bit_rate;
        packet1.load_percent = packet_in.load_percent;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CAN_STATS_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_stats_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_can_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.payload_bit_rate , packet1.total_bit_rate , packet1.load_percent );
    mavlink_msg_can_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.payload_bit_rate , packet1.total_bit_rate , packet1.load_percent );
    mavlink_msg_can_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_can_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_can_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.payload_bit_rate , packet1.total_bit_rate , packet1.load_percent );
    mavlink_msg_can_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CAN_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CAN_STATS_TM) != NULL);
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
        93372036854775807ULL,963497880,963498088,129.0,157.0,185.0,213.0,241.0,269.0,19315,19419
    };
    mavlink_task_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.task_missed_events = packet_in.task_missed_events;
        packet1.task_failed_events = packet_in.task_failed_events;
        packet1.task_activation_mean = packet_in.task_activation_mean;
        packet1.task_activation_stddev = packet_in.task_activation_stddev;
        packet1.task_period_mean = packet_in.task_period_mean;
        packet1.task_period_stddev = packet_in.task_period_stddev;
        packet1.task_workload_mean = packet_in.task_workload_mean;
        packet1.task_workload_stddev = packet_in.task_workload_stddev;
        packet1.task_id = packet_in.task_id;
        packet1.task_period = packet_in.task_period;
        
        
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
    mavlink_msg_task_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_missed_events , packet1.task_failed_events , packet1.task_activation_mean , packet1.task_activation_stddev , packet1.task_period_mean , packet1.task_period_stddev , packet1.task_workload_mean , packet1.task_workload_stddev );
    mavlink_msg_task_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_task_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_missed_events , packet1.task_failed_events , packet1.task_activation_mean , packet1.task_activation_stddev , packet1.task_period_mean , packet1.task_period_stddev , packet1.task_workload_mean , packet1.task_workload_stddev );
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
    mavlink_msg_task_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.task_id , packet1.task_period , packet1.task_missed_events , packet1.task_failed_events , packet1.task_activation_mean , packet1.task_activation_stddev , packet1.task_period_mean , packet1.task_period_stddev , packet1.task_workload_mean , packet1.task_workload_stddev );
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

static void mavlink_test_mea_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MEA_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mea_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,89
    };
    mavlink_mea_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.kalman_x0 = packet_in.kalman_x0;
        packet1.kalman_x1 = packet_in.kalman_x1;
        packet1.kalman_x2 = packet_in.kalman_x2;
        packet1.mass = packet_in.mass;
        packet1.corrected_pressure = packet_in.corrected_pressure;
        packet1.state = packet_in.state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MEA_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MEA_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mea_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mea_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mea_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.mass , packet1.corrected_pressure );
    mavlink_msg_mea_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mea_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.mass , packet1.corrected_pressure );
    mavlink_msg_mea_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mea_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mea_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.state , packet1.kalman_x0 , packet1.kalman_x1 , packet1.kalman_x2 , packet1.mass , packet1.corrected_pressure );
    mavlink_msg_mea_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MEA_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MEA_TM) != NULL);
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
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,1081.0,1109.0,1137.0,1165.0,1193.0,1221.0,21,88
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
        packet1.mea_mass = packet_in.mea_mass;
        packet1.mea_apogee = packet_in.mea_apogee;
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        packet1.vn100_qx = packet_in.vn100_qx;
        packet1.vn100_qy = packet_in.vn100_qy;
        packet1.vn100_qz = packet_in.vn100_qz;
        packet1.vn100_qw = packet_in.vn100_qw;
        packet1.gps_lat = packet_in.gps_lat;
        packet1.gps_lon = packet_in.gps_lon;
        packet1.gps_alt = packet_in.gps_alt;
        packet1.abk_angle = packet_in.abk_angle;
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
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.cam_battery_voltage = packet_in.cam_battery_voltage;
        packet1.temperature = packet_in.temperature;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.fmm_state = packet_in.fmm_state;
        
        
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
    mavlink_msg_rocket_flight_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.mea_mass , packet1.mea_apogee , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.vn100_qx , packet1.vn100_qy , packet1.vn100_qz , packet1.vn100_qw , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.abk_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
    mavlink_msg_rocket_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_flight_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.mea_mass , packet1.mea_apogee , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.vn100_qx , packet1.vn100_qy , packet1.vn100_qz , packet1.vn100_qw , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.abk_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
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
    mavlink_msg_rocket_flight_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure_ada , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dpl , packet1.airspeed_pitot , packet1.altitude_agl , packet1.ada_vert_speed , packet1.mea_mass , packet1.mea_apogee , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.vn100_qx , packet1.vn100_qy , packet1.vn100_qz , packet1.vn100_qw , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.abk_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
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
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,1081.0,217,28
    };
    mavlink_payload_flight_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.pressure_digi = packet_in.pressure_digi;
        packet1.pressure_static = packet_in.pressure_static;
        packet1.pressure_dynamic = packet_in.pressure_dynamic;
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
        packet1.wes_n = packet_in.wes_n;
        packet1.wes_e = packet_in.wes_e;
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.cam_battery_voltage = packet_in.cam_battery_voltage;
        packet1.temperature = packet_in.temperature;
        packet1.gps_fix = packet_in.gps_fix;
        packet1.fmm_state = packet_in.fmm_state;
        
        
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
    mavlink_msg_payload_flight_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dynamic , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.wes_n , packet1.wes_e , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
    mavlink_msg_payload_flight_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_flight_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dynamic , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.wes_n , packet1.wes_e , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
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
    mavlink_msg_payload_flight_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.pressure_digi , packet1.pressure_static , packet1.pressure_dynamic , packet1.airspeed_pitot , packet1.altitude_agl , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z , packet1.gps_fix , packet1.gps_lat , packet1.gps_lon , packet1.gps_alt , packet1.fmm_state , packet1.left_servo_angle , packet1.right_servo_angle , packet1.nas_n , packet1.nas_e , packet1.nas_d , packet1.nas_vn , packet1.nas_ve , packet1.nas_vd , packet1.nas_qx , packet1.nas_qy , packet1.nas_qz , packet1.nas_qw , packet1.nas_bias_x , packet1.nas_bias_y , packet1.nas_bias_z , packet1.wes_n , packet1.wes_e , packet1.battery_voltage , packet1.cam_battery_voltage , packet1.temperature );
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
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,93372036854777823ULL,93372036854778327ULL,93372036854778831ULL,93372036854779335ULL,93372036854779839ULL,93372036854780343ULL,93372036854780847ULL,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,1053.0,963505368,1109.0,963505784,25763,119,186,253,64,131,198,9,76,143,210,21,88,155,222
    };
    mavlink_rocket_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.liftoff_ts = packet_in.liftoff_ts;
        packet1.liftoff_max_acc_ts = packet_in.liftoff_max_acc_ts;
        packet1.max_speed_ts = packet_in.max_speed_ts;
        packet1.max_mach_ts = packet_in.max_mach_ts;
        packet1.shutdown_ts = packet_in.shutdown_ts;
        packet1.apogee_ts = packet_in.apogee_ts;
        packet1.apogee_max_acc_ts = packet_in.apogee_max_acc_ts;
        packet1.dpl_ts = packet_in.dpl_ts;
        packet1.dpl_max_acc_ts = packet_in.dpl_max_acc_ts;
        packet1.dpl_bay_max_pressure_ts = packet_in.dpl_bay_max_pressure_ts;
        packet1.liftoff_max_acc = packet_in.liftoff_max_acc;
        packet1.max_speed = packet_in.max_speed;
        packet1.max_speed_altitude = packet_in.max_speed_altitude;
        packet1.max_mach = packet_in.max_mach;
        packet1.shutdown_alt = packet_in.shutdown_alt;
        packet1.apogee_lat = packet_in.apogee_lat;
        packet1.apogee_lon = packet_in.apogee_lon;
        packet1.apogee_alt = packet_in.apogee_alt;
        packet1.apogee_max_acc = packet_in.apogee_max_acc;
        packet1.dpl_alt = packet_in.dpl_alt;
        packet1.dpl_max_acc = packet_in.dpl_max_acc;
        packet1.dpl_bay_max_pressure = packet_in.dpl_bay_max_pressure;
        packet1.ref_lat = packet_in.ref_lat;
        packet1.ref_lon = packet_in.ref_lon;
        packet1.ref_alt = packet_in.ref_alt;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        packet1.exp_angle = packet_in.exp_angle;
        packet1.log_good = packet_in.log_good;
        packet1.log_number = packet_in.log_number;
        packet1.ada_state = packet_in.ada_state;
        packet1.abk_state = packet_in.abk_state;
        packet1.nas_state = packet_in.nas_state;
        packet1.mea_state = packet_in.mea_state;
        packet1.pin_launch = packet_in.pin_launch;
        packet1.pin_nosecone = packet_in.pin_nosecone;
        packet1.pin_expulsion = packet_in.pin_expulsion;
        packet1.cutter_presence = packet_in.cutter_presence;
        packet1.payload_board_state = packet_in.payload_board_state;
        packet1.motor_board_state = packet_in.motor_board_state;
        packet1.payload_can_status = packet_in.payload_can_status;
        packet1.motor_can_status = packet_in.motor_can_status;
        packet1.rig_can_status = packet_in.rig_can_status;
        packet1.hil_state = packet_in.hil_state;
        
        
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
    mavlink_msg_rocket_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.shutdown_ts , packet1.shutdown_alt , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.dpl_bay_max_pressure_ts , packet1.dpl_bay_max_pressure , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.cpu_load , packet1.free_heap , packet1.ada_state , packet1.abk_state , packet1.nas_state , packet1.mea_state , packet1.exp_angle , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.payload_board_state , packet1.motor_board_state , packet1.payload_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
    mavlink_msg_rocket_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocket_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.shutdown_ts , packet1.shutdown_alt , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.dpl_bay_max_pressure_ts , packet1.dpl_bay_max_pressure , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.cpu_load , packet1.free_heap , packet1.ada_state , packet1.abk_state , packet1.nas_state , packet1.mea_state , packet1.exp_angle , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.payload_board_state , packet1.motor_board_state , packet1.payload_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
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
    mavlink_msg_rocket_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.shutdown_ts , packet1.shutdown_alt , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.dpl_bay_max_pressure_ts , packet1.dpl_bay_max_pressure , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.cpu_load , packet1.free_heap , packet1.ada_state , packet1.abk_state , packet1.nas_state , packet1.mea_state , packet1.exp_angle , packet1.pin_launch , packet1.pin_nosecone , packet1.pin_expulsion , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.payload_board_state , packet1.motor_board_state , packet1.payload_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
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
        93372036854775807ULL,93372036854776311ULL,93372036854776815ULL,93372036854777319ULL,93372036854777823ULL,93372036854778327ULL,93372036854778831ULL,93372036854779335ULL,93372036854779839ULL,521.0,549.0,577.0,605.0,633.0,661.0,689.0,717.0,745.0,773.0,801.0,829.0,857.0,885.0,913.0,941.0,969.0,997.0,1025.0,963505160,963505368,25347,95,162,229,40,107,174,241,52,119,186,253,64
    };
    mavlink_payload_stats_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.liftoff_ts = packet_in.liftoff_ts;
        packet1.liftoff_max_acc_ts = packet_in.liftoff_max_acc_ts;
        packet1.max_speed_ts = packet_in.max_speed_ts;
        packet1.max_mach_ts = packet_in.max_mach_ts;
        packet1.apogee_ts = packet_in.apogee_ts;
        packet1.apogee_max_acc_ts = packet_in.apogee_max_acc_ts;
        packet1.dpl_ts = packet_in.dpl_ts;
        packet1.dpl_max_acc_ts = packet_in.dpl_max_acc_ts;
        packet1.liftoff_max_acc = packet_in.liftoff_max_acc;
        packet1.max_speed = packet_in.max_speed;
        packet1.max_speed_altitude = packet_in.max_speed_altitude;
        packet1.max_mach = packet_in.max_mach;
        packet1.apogee_lat = packet_in.apogee_lat;
        packet1.apogee_lon = packet_in.apogee_lon;
        packet1.apogee_alt = packet_in.apogee_alt;
        packet1.apogee_max_acc = packet_in.apogee_max_acc;
        packet1.wing_target_lat = packet_in.wing_target_lat;
        packet1.wing_target_lon = packet_in.wing_target_lon;
        packet1.wing_active_target_n = packet_in.wing_active_target_n;
        packet1.wing_active_target_e = packet_in.wing_active_target_e;
        packet1.dpl_alt = packet_in.dpl_alt;
        packet1.dpl_max_acc = packet_in.dpl_max_acc;
        packet1.ref_lat = packet_in.ref_lat;
        packet1.ref_lon = packet_in.ref_lon;
        packet1.ref_alt = packet_in.ref_alt;
        packet1.min_pressure = packet_in.min_pressure;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        packet1.log_good = packet_in.log_good;
        packet1.log_number = packet_in.log_number;
        packet1.wing_algorithm = packet_in.wing_algorithm;
        packet1.nas_state = packet_in.nas_state;
        packet1.wnc_state = packet_in.wnc_state;
        packet1.pin_launch = packet_in.pin_launch;
        packet1.pin_nosecone = packet_in.pin_nosecone;
        packet1.cutter_presence = packet_in.cutter_presence;
        packet1.main_board_state = packet_in.main_board_state;
        packet1.motor_board_state = packet_in.motor_board_state;
        packet1.main_can_status = packet_in.main_can_status;
        packet1.motor_can_status = packet_in.motor_can_status;
        packet1.rig_can_status = packet_in.rig_can_status;
        packet1.hil_state = packet_in.hil_state;
        
        
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
    mavlink_msg_payload_stats_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.wing_target_lat , packet1.wing_target_lon , packet1.wing_active_target_n , packet1.wing_active_target_e , packet1.wing_algorithm , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap , packet1.nas_state , packet1.wnc_state , packet1.pin_launch , packet1.pin_nosecone , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.main_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
    mavlink_msg_payload_stats_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_payload_stats_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.wing_target_lat , packet1.wing_target_lon , packet1.wing_active_target_n , packet1.wing_active_target_e , packet1.wing_algorithm , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap , packet1.nas_state , packet1.wnc_state , packet1.pin_launch , packet1.pin_nosecone , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.main_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
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
    mavlink_msg_payload_stats_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.liftoff_ts , packet1.liftoff_max_acc_ts , packet1.liftoff_max_acc , packet1.max_speed_ts , packet1.max_speed , packet1.max_speed_altitude , packet1.max_mach_ts , packet1.max_mach , packet1.apogee_ts , packet1.apogee_lat , packet1.apogee_lon , packet1.apogee_alt , packet1.apogee_max_acc_ts , packet1.apogee_max_acc , packet1.wing_target_lat , packet1.wing_target_lon , packet1.wing_active_target_n , packet1.wing_active_target_e , packet1.wing_algorithm , packet1.dpl_ts , packet1.dpl_alt , packet1.dpl_max_acc_ts , packet1.dpl_max_acc , packet1.ref_lat , packet1.ref_lon , packet1.ref_alt , packet1.min_pressure , packet1.cpu_load , packet1.free_heap , packet1.nas_state , packet1.wnc_state , packet1.pin_launch , packet1.pin_nosecone , packet1.cutter_presence , packet1.log_number , packet1.log_good , packet1.main_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.motor_can_status , packet1.rig_can_status , packet1.hil_state );
    mavlink_msg_payload_stats_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("PAYLOAD_STATS_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_PAYLOAD_STATS_TM) != NULL);
#endif
}

static void mavlink_test_gse_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_GSE_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_gse_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,963498920,241.0,269.0,297.0,963499752,19731,27,94,161,228,39,106,173,240,51,118,185,252,63,130
    };
    mavlink_gse_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.loadcell_rocket = packet_in.loadcell_rocket;
        packet1.loadcell_vessel = packet_in.loadcell_vessel;
        packet1.filling_pressure = packet_in.filling_pressure;
        packet1.vessel_pressure = packet_in.vessel_pressure;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.free_heap = packet_in.free_heap;
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.current_consumption = packet_in.current_consumption;
        packet1.umbilical_current_consumption = packet_in.umbilical_current_consumption;
        packet1.log_good = packet_in.log_good;
        packet1.log_number = packet_in.log_number;
        packet1.filling_valve_state = packet_in.filling_valve_state;
        packet1.venting_valve_state = packet_in.venting_valve_state;
        packet1.release_valve_state = packet_in.release_valve_state;
        packet1.main_valve_state = packet_in.main_valve_state;
        packet1.nitrogen_valve_state = packet_in.nitrogen_valve_state;
        packet1.gmm_state = packet_in.gmm_state;
        packet1.tars_state = packet_in.tars_state;
        packet1.arming_state = packet_in.arming_state;
        packet1.main_board_state = packet_in.main_board_state;
        packet1.payload_board_state = packet_in.payload_board_state;
        packet1.motor_board_state = packet_in.motor_board_state;
        packet1.main_can_status = packet_in.main_can_status;
        packet1.payload_can_status = packet_in.payload_can_status;
        packet1.motor_can_status = packet_in.motor_can_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_GSE_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_GSE_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gse_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_gse_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gse_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.loadcell_rocket , packet1.loadcell_vessel , packet1.filling_pressure , packet1.vessel_pressure , packet1.filling_valve_state , packet1.venting_valve_state , packet1.release_valve_state , packet1.main_valve_state , packet1.nitrogen_valve_state , packet1.gmm_state , packet1.tars_state , packet1.arming_state , packet1.cpu_load , packet1.free_heap , packet1.battery_voltage , packet1.current_consumption , packet1.umbilical_current_consumption , packet1.main_board_state , packet1.payload_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.payload_can_status , packet1.motor_can_status , packet1.log_number , packet1.log_good );
    mavlink_msg_gse_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gse_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.loadcell_rocket , packet1.loadcell_vessel , packet1.filling_pressure , packet1.vessel_pressure , packet1.filling_valve_state , packet1.venting_valve_state , packet1.release_valve_state , packet1.main_valve_state , packet1.nitrogen_valve_state , packet1.gmm_state , packet1.tars_state , packet1.arming_state , packet1.cpu_load , packet1.free_heap , packet1.battery_voltage , packet1.current_consumption , packet1.umbilical_current_consumption , packet1.main_board_state , packet1.payload_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.payload_can_status , packet1.motor_can_status , packet1.log_number , packet1.log_good );
    mavlink_msg_gse_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_gse_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_gse_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.loadcell_rocket , packet1.loadcell_vessel , packet1.filling_pressure , packet1.vessel_pressure , packet1.filling_valve_state , packet1.venting_valve_state , packet1.release_valve_state , packet1.main_valve_state , packet1.nitrogen_valve_state , packet1.gmm_state , packet1.tars_state , packet1.arming_state , packet1.cpu_load , packet1.free_heap , packet1.battery_voltage , packet1.current_consumption , packet1.umbilical_current_consumption , packet1.main_board_state , packet1.payload_board_state , packet1.motor_board_state , packet1.main_can_status , packet1.payload_can_status , packet1.motor_can_status , packet1.log_number , packet1.log_good );
    mavlink_msg_gse_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("GSE_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_GSE_TM) != NULL);
#endif
}

static void mavlink_test_motor_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,963498920,18899,235,46,113
    };
    mavlink_motor_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.top_tank_pressure = packet_in.top_tank_pressure;
        packet1.bottom_tank_pressure = packet_in.bottom_tank_pressure;
        packet1.combustion_chamber_pressure = packet_in.combustion_chamber_pressure;
        packet1.tank_temperature = packet_in.tank_temperature;
        packet1.battery_voltage = packet_in.battery_voltage;
        packet1.log_good = packet_in.log_good;
        packet1.log_number = packet_in.log_number;
        packet1.main_valve_state = packet_in.main_valve_state;
        packet1.venting_valve_state = packet_in.venting_valve_state;
        packet1.hil_state = packet_in.hil_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.top_tank_pressure , packet1.bottom_tank_pressure , packet1.combustion_chamber_pressure , packet1.tank_temperature , packet1.main_valve_state , packet1.venting_valve_state , packet1.battery_voltage , packet1.log_number , packet1.log_good , packet1.hil_state );
    mavlink_msg_motor_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.top_tank_pressure , packet1.bottom_tank_pressure , packet1.combustion_chamber_pressure , packet1.tank_temperature , packet1.main_valve_state , packet1.venting_valve_state , packet1.battery_voltage , packet1.log_number , packet1.log_good , packet1.hil_state );
    mavlink_msg_motor_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.top_tank_pressure , packet1.bottom_tank_pressure , packet1.combustion_chamber_pressure , packet1.tank_temperature , packet1.main_valve_state , packet1.venting_valve_state , packet1.battery_voltage , packet1.log_number , packet1.log_good , packet1.hil_state );
    mavlink_msg_motor_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_TM) != NULL);
#endif
}

static void mavlink_test_calibration_tm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CALIBRATION_TM >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_calibration_tm_t packet_in = {
        93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,437.0,465.0,493.0,521.0
    };
    mavlink_calibration_tm_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.timestamp = packet_in.timestamp;
        packet1.gyro_bias_x = packet_in.gyro_bias_x;
        packet1.gyro_bias_y = packet_in.gyro_bias_y;
        packet1.gyro_bias_z = packet_in.gyro_bias_z;
        packet1.mag_bias_x = packet_in.mag_bias_x;
        packet1.mag_bias_y = packet_in.mag_bias_y;
        packet1.mag_bias_z = packet_in.mag_bias_z;
        packet1.mag_scale_x = packet_in.mag_scale_x;
        packet1.mag_scale_y = packet_in.mag_scale_y;
        packet1.mag_scale_z = packet_in.mag_scale_z;
        packet1.static_press_1_bias = packet_in.static_press_1_bias;
        packet1.static_press_1_scale = packet_in.static_press_1_scale;
        packet1.static_press_2_bias = packet_in.static_press_2_bias;
        packet1.static_press_2_scale = packet_in.static_press_2_scale;
        packet1.dpl_bay_press_bias = packet_in.dpl_bay_press_bias;
        packet1.dpl_bay_press_scale = packet_in.dpl_bay_press_scale;
        packet1.dynamic_press_bias = packet_in.dynamic_press_bias;
        packet1.dynamic_press_scale = packet_in.dynamic_press_scale;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CALIBRATION_TM_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_calibration_tm_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_calibration_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_calibration_tm_pack(system_id, component_id, &msg , packet1.timestamp , packet1.gyro_bias_x , packet1.gyro_bias_y , packet1.gyro_bias_z , packet1.mag_bias_x , packet1.mag_bias_y , packet1.mag_bias_z , packet1.mag_scale_x , packet1.mag_scale_y , packet1.mag_scale_z , packet1.static_press_1_bias , packet1.static_press_1_scale , packet1.static_press_2_bias , packet1.static_press_2_scale , packet1.dpl_bay_press_bias , packet1.dpl_bay_press_scale , packet1.dynamic_press_bias , packet1.dynamic_press_scale );
    mavlink_msg_calibration_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_calibration_tm_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.timestamp , packet1.gyro_bias_x , packet1.gyro_bias_y , packet1.gyro_bias_z , packet1.mag_bias_x , packet1.mag_bias_y , packet1.mag_bias_z , packet1.mag_scale_x , packet1.mag_scale_y , packet1.mag_scale_z , packet1.static_press_1_bias , packet1.static_press_1_scale , packet1.static_press_2_bias , packet1.static_press_2_scale , packet1.dpl_bay_press_bias , packet1.dpl_bay_press_scale , packet1.dynamic_press_bias , packet1.dynamic_press_scale );
    mavlink_msg_calibration_tm_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_calibration_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_calibration_tm_send(MAVLINK_COMM_1 , packet1.timestamp , packet1.gyro_bias_x , packet1.gyro_bias_y , packet1.gyro_bias_z , packet1.mag_bias_x , packet1.mag_bias_y , packet1.mag_bias_z , packet1.mag_scale_x , packet1.mag_scale_y , packet1.mag_scale_z , packet1.static_press_1_bias , packet1.static_press_1_scale , packet1.static_press_2_bias , packet1.static_press_2_scale , packet1.dpl_bay_press_bias , packet1.dpl_bay_press_scale , packet1.dynamic_press_bias , packet1.dynamic_press_scale );
    mavlink_msg_calibration_tm_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CALIBRATION_TM") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CALIBRATION_TM) != NULL);
#endif
}

static void mavlink_test_lyra(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ping_tc(system_id, component_id, last_msg);
    mavlink_test_command_tc(system_id, component_id, last_msg);
    mavlink_test_system_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_sensor_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_servo_tm_request_tc(system_id, component_id, last_msg);
    mavlink_test_set_servo_angle_tc(system_id, component_id, last_msg);
    mavlink_test_reset_servo_tc(system_id, component_id, last_msg);
    mavlink_test_wiggle_servo_tc(system_id, component_id, last_msg);
    mavlink_test_set_reference_altitude_tc(system_id, component_id, last_msg);
    mavlink_test_set_reference_temperature_tc(system_id, component_id, last_msg);
    mavlink_test_set_orientation_tc(system_id, component_id, last_msg);
    mavlink_test_set_orientation_quat_tc(system_id, component_id, last_msg);
    mavlink_test_set_coordinates_tc(system_id, component_id, last_msg);
    mavlink_test_raw_event_tc(system_id, component_id, last_msg);
    mavlink_test_set_deployment_altitude_tc(system_id, component_id, last_msg);
    mavlink_test_set_target_coordinates_tc(system_id, component_id, last_msg);
    mavlink_test_set_algorithm_tc(system_id, component_id, last_msg);
    mavlink_test_set_calibration_pressure_tc(system_id, component_id, last_msg);
    mavlink_test_set_initial_mea_mass_tc(system_id, component_id, last_msg);
    mavlink_test_set_atomic_valve_timing_tc(system_id, component_id, last_msg);
    mavlink_test_set_valve_maximum_aperture_tc(system_id, component_id, last_msg);
    mavlink_test_conrig_state_tc(system_id, component_id, last_msg);
    mavlink_test_set_ignition_time_tc(system_id, component_id, last_msg);
    mavlink_test_set_nitrogen_time_tc(system_id, component_id, last_msg);
    mavlink_test_set_cooling_time_tc(system_id, component_id, last_msg);
    mavlink_test_set_stepper_angle_tc(system_id, component_id, last_msg);
    mavlink_test_set_stepper_steps_tc(system_id, component_id, last_msg);
    mavlink_test_set_stepper_multiplier_tc(system_id, component_id, last_msg);
    mavlink_test_set_antenna_coordinates_arp_tc(system_id, component_id, last_msg);
    mavlink_test_set_rocket_coordinates_arp_tc(system_id, component_id, last_msg);
    mavlink_test_arp_command_tc(system_id, component_id, last_msg);
    mavlink_test_ack_tm(system_id, component_id, last_msg);
    mavlink_test_nack_tm(system_id, component_id, last_msg);
    mavlink_test_wack_tm(system_id, component_id, last_msg);
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
    mavlink_test_reference_tm(system_id, component_id, last_msg);
    mavlink_test_registry_float_tm(system_id, component_id, last_msg);
    mavlink_test_registry_int_tm(system_id, component_id, last_msg);
    mavlink_test_registry_coord_tm(system_id, component_id, last_msg);
    mavlink_test_arp_tm(system_id, component_id, last_msg);
    mavlink_test_sys_tm(system_id, component_id, last_msg);
    mavlink_test_logger_tm(system_id, component_id, last_msg);
    mavlink_test_mavlink_stats_tm(system_id, component_id, last_msg);
    mavlink_test_can_stats_tm(system_id, component_id, last_msg);
    mavlink_test_task_stats_tm(system_id, component_id, last_msg);
    mavlink_test_ada_tm(system_id, component_id, last_msg);
    mavlink_test_nas_tm(system_id, component_id, last_msg);
    mavlink_test_mea_tm(system_id, component_id, last_msg);
    mavlink_test_rocket_flight_tm(system_id, component_id, last_msg);
    mavlink_test_payload_flight_tm(system_id, component_id, last_msg);
    mavlink_test_rocket_stats_tm(system_id, component_id, last_msg);
    mavlink_test_payload_stats_tm(system_id, component_id, last_msg);
    mavlink_test_gse_tm(system_id, component_id, last_msg);
    mavlink_test_motor_tm(system_id, component_id, last_msg);
    mavlink_test_calibration_tm(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LYRA_TESTSUITE_H
