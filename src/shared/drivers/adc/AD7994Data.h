#ifndef SRC_SHARED_DRIVERS_ADC_AD7994DATA_H
#define SRC_SHARED_DRIVERS_ADC_AD7994DATA_H

#include <cstdint>
#include <ostream>
#include <string>

struct AD7994Sample
{
    uint32_t timestamp;

    uint8_t channel_id; // [1-4]
    bool alert_flag;
    uint16_t value;

    // Functions used to deserialize the binary logs into csv files

    static std::string header() { return "timestamp,ch_id,value,alert_flag\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << channel_id << "," << value << ","
           << (int)alert_flag << "\n";
    }
};

#endif