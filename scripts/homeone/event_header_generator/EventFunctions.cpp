#include "Events.h"
#include "Topics.h"

#include <map>

namespace HomeoneBoard
{

string getEventString(uint8_t event)
{
    static const map<uint8_t, string> event_string_map {
        { EV_ADA_APOGEE_DETECTED, "EV_ADA_APOGEE_DETECTED" },
        { EV_ADA_DPL_ALT_DETECTED, "EV_ADA_DPL_ALT_DETECTED" },
        { EV_APOGEE, "EV_APOGEE" },
        { EV_ARMED, "EV_ARMED" },
        { EV_DPL_ALTITUDE, "EV_DPL_ALTITUDE" },
        { EV_GS_OFFLINE, "EV_GS_OFFLINE" },
        { EV_IGN_ABORTED, "EV_IGN_ABORTED" },
        { EV_IGN_GETSTATUS, "EV_IGN_GETSTATUS" },
        { EV_IGN_OFFLINE, "EV_IGN_OFFLINE" },
        { EV_INIT_ERROR, "EV_INIT_ERROR" },
        { EV_INIT_OK, "EV_INIT_OK" },
        { EV_LANDED, "EV_LANDED" },
        { EV_LAUNCH, "EV_LAUNCH" },
        { EV_LIFTOFF, "EV_LIFTOFF" },
        { EV_NC_DETACHED, "EV_NC_DETACHED" },
        { EV_NC_GETSTATUS, "EV_NC_GETSTATUS" },
        { EV_NC_OFFLINE, "EV_NC_OFFLINE" },
        { EV_NEW_CAN_MSG, "EV_NEW_CAN_MSG" },
        { EV_SEND_HR_TM, "EV_SEND_HR_TM" },
        { EV_SEND_LR_TM, "EV_SEND_LR_TM" },
        { EV_TC_ABORT_LAUNCH, "EV_TC_ABORT_LAUNCH" },
        { EV_TC_ARM, "EV_TC_ARM" },
        { EV_TC_BARO_CALIBRATION, "EV_TC_BARO_CALIBRATION" },
        { EV_TC_BOARD_RESET, "EV_TC_BOARD_RESET" },
        { EV_TC_CUT_ALL, "EV_TC_CUT_ALL" },
        { EV_TC_CUT_FIRST_DROGUE, "EV_TC_CUT_FIRST_DROGUE" },
        { EV_TC_DISARM, "EV_TC_DISARM" },
        { EV_TC_END_MISSION, "EV_TC_END_MISSION" },
        { EV_TC_LAUNCH, "EV_TC_LAUNCH" },
        { EV_TC_MANUAL_MODE, "EV_TC_MANUAL_MODE" },
        { EV_TC_NC_CLOSE, "EV_TC_NC_CLOSE" },
        { EV_TC_NC_OPEN, "EV_TC_NC_OPEN" },
        { EV_TC_START_LOGGING, "EV_TC_START_LOGGING" },
        { EV_TC_STOP_LOGGING, "EV_TC_STOP_LOGGING" },
        { EV_TC_TEST_MODE, "EV_TC_TEST_MODE" },
        { EV_TIMEOUT_APOGEE, "EV_TIMEOUT_APOGEE" },
        { EV_TIMEOUT_ARM, "EV_TIMEOUT_ARM" },
        { EV_TIMEOUT_CUTTING, "EV_TIMEOUT_CUTTING" },
        { EV_TIMEOUT_DPL_ALT, "EV_TIMEOUT_DPL_ALT" },
        { EV_TIMEOUT_END_MISSION, "EV_TIMEOUT_END_MISSION" },
        { EV_TIMEOUT_SHADOW_MODE, "EV_TIMEOUT_SHADOW_MODE" },
        { EV_UMBILICAL_DETACHED, "EV_UMBILICAL_DETACHED" }
    };
    auto   it  = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

string getTopicString(uint8_t topic)
{
	static const map<uint8_t, string> topic_string_map{
        { TOPIC_ADA, "TOPIC_ADA" },
        { TOPIC_DEPLOYMENT, "TOPIC_DEPLOYMENT" },
        { TOPIC_FLIGHT_EVENTS, "TOPIC_FLIGHT_EVENTS" },
        { TOPIC_FMM, "TOPIC_FMM" },
        { TOPIC_IGNITION, "TOPIC_IGNITION" },
        { TOPIC_TC, "TOPIC_TC" }
	};
	auto it = topic_string_map.find(topic);
	return it == topic_string_map.end() ? "TOPIC_UNKNOWN" : it->second; 
}

}