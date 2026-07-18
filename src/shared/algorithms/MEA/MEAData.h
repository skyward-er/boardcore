#pragma once

#include <algorithms/MEA/MEA_types.h>

#include <reflect.hpp>

namespace Boardcore
{

struct MEAState
{
    uint64_t timestamp;
    float mass;

    MEAState() : timestamp(0), mass(0.0f) {};

    MEAState(uint64_t timestamp, float mass)
        : timestamp(timestamp), mass(mass) {};

    MEAState(uint64_t timestamp, MEA::MEAOut out)
        : timestamp(timestamp), mass(out.Mass) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MEAState, FIELD_DEF(timestamp) FIELD_DEF(mass));
    }
};

struct MEALogsWrapper
{
    MEA::MEALogs logs;

    MEALogsWrapper() : logs() {};

    MEALogsWrapper(uint64_t timestamp, MEA::MEALogs logs) : logs(logs)
    {
        logs.Timestamp = timestamp;
    };

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MEALogsWrapper,
                          FIELD_DEF2(logs, Timestamp) FIELD_DEF2(logs, Mass)
                              FIELD_DEF2(logs, States)
                                  FIELD_DEF2(logs, Pressure));
    }
};

}  // namespace Boardcore
