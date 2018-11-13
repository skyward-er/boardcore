/*
 * Status of the component, wraps the mavlink stats
 * which are updated at every byte reception.
 */
struct TMTCStatus
{
    mavlink_status_t mavstatus;
    uint8_t sendErrors;
    uint8_t ackErrors;
};