/** @file
 *  @brief Skyward Status Repository
 */
#pragma once

namespace StatusRepo
{
    /* Enumeration of all possible TMs */
    enum MavTMList: uint8_t
    {
         MAV_NOARG_TC_ID,
        
    };

    /* Struct containing all tms in the form of mavlink structs */
    static struct status_repo_t
    {
         mavlink_noarg_tc_t noarg_tc;
        
    } status repo;

    /**
     * Retrieve one of the telemetry structs
     * @req_tm          required telemetry
     * @return          packed mavlink struct of that telemetry
     */
    static mavlink_message_t getTM(uint8_t req_tm, uint8_t sys_id, uint8_t comp_id)
    {
        mavlink_message_t m;

        switch (req_tm) 
        {
             case MavTMList::MAV_NOARG_TC_ID:
                mavlink_msg_noarg_tc_encode(sys_id, comp_id, &m, &(repo.noarg_tc));
                break;
            
            default:
                // TODO: manage error
                break;
        }
       
        return m;
    }

}
