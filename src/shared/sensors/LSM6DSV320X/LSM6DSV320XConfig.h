
#pragma once

#include <cstdint>

namespace Boardcore
{

struct LSM6DSV320XConfig
{
    // Low-g accelerometer output data rate
    enum class LOW_G_ODR : uint8_t
    {
        POWER_DOWN = 0,
        HZ_1_875   = 1,
        HZ_7_5     = 2,
        HZ_15      = 3,
        HZ_30      = 4,
        HZ_60      = 5,
        HZ_120     = 6,
        HZ_240     = 7,
        HZ_480     = 8,
        HZ_960     = 9,
        HZ_1920    = 10,
        HZ_3840    = 11,
        HZ_7680    = 12
    };

    // Low-g accelerometer full scale
    enum class LOW_G_FULLSCALE : uint8_t
    {
        G2  = 0b00,
        G4  = 0b01,
        G8  = 0b10,
        G16 = 0b11
    };

    // Low-g accelerometer operating mode
    enum class LOW_G_OPERATING_MODE : uint8_t
    {
        HIGH_PERFORMANCE = 0b000,
        HIGH_ACCURACY    = 0b001,
        ODR_TRIGGERED    = 0b011,
        LOW_POWER_1      = 0b100,
        LOW_POWER_2      = 0b101,
        LOW_POWER_3      = 0b110,
        NORMAL           = 0b111
    };

    // High-g accelerometer output data rate
    enum class HIGH_G_ODR : uint8_t
    {
        POWER_DOWN = 0b000,
        HZ_480     = 0b011,
        HZ_960     = 0b100,
        HZ_1920    = 0b101,
        HZ_3840    = 0b110,
        HZ_7680    = 0b111
    };

    // High-g accelerometer full scale
    enum class HIGH_G_FULLSCALE : uint8_t
    {
        G32  = 0b000,
        G64  = 0b001,
        G128 = 0b010,
        G256 = 0b011,
        G320 = 0b100
    };

    // Gyroscope output data rate
    enum class GYR_ODR : uint8_t
    {
        POWER_DOWN = 0b0000,
        HZ_7_5     = 0b0010,
        HZ_15      = 0b0011,
        HZ_30      = 0b0100,
        HZ_60      = 0b0101,
        HZ_120     = 0b0110,
        HZ_240     = 0b0111,
        HZ_480     = 0b1000,
        HZ_960     = 0b1001,
        HZ_1920    = 0b1010,
        HZ_3840    = 0b1011,
        HZ_7680    = 0b1100
    };

    // Gyroscope full scale
    enum class GYR_FULLSCALE : uint8_t
    {
        DPS_125  = 0b000,
        DPS_250  = 0b001,
        DPS_500  = 0b010,
        DPS_1000 = 0b011,
        DPS_2000 = 0b100,
        DPS_4000 = 0b101
    };

    // Gyroscope operating mode
    enum class GYR_OPERATING_MODE : uint8_t
    {
        HIGH_PERFORMANCE = 0b000,
        HIGH_ACCURACY    = 0b001,
        ODR_TRIGGERED    = 0b011,
        SLEEP            = 0b100,
        LOW_POWER        = 0b101
    };

    // Block Data Update
    enum class BDU : uint8_t
    {
        CONTINUOUS_UPDATE = 0,
        UPDATE_AFTER_READ = 1
    };

    // FIFO operating mode
    enum class FIFO_MODE : uint8_t
    {
        BYPASS                 = 0b000,
        FIFO                   = 0b001,
        CONTINUOUS_WTM_TO_FULL = 0b010,
        CONTINUOUS_TO_FIFO     = 0b011,
        BYPASS_TO_CONTINUOUS   = 0b100,
        CONTINUOUS             = 0b110,
        BYPASS_TO_FIFO         = 0b111
    };

    // FIFO batch data rate for low-g accelerometer and gyroscope
    enum class FIFO_BDR : uint8_t
    {
        DISABLED = 0,
        HZ_1_875 = 1,
        HZ_7_5   = 2,
        HZ_15    = 3,
        HZ_30    = 4,
        HZ_60    = 5,
        HZ_120   = 6,
        HZ_240   = 7,
        HZ_480   = 8,
        HZ_960   = 9,
        HZ_1920  = 10,
        HZ_3840  = 11,
        HZ_7680  = 12
    };

    // FIFO timestamp decimation
    enum class FIFO_TIMESTAMP_DECIMATION : uint8_t
    {
        DISABLED = 0b00,
        DEC_1    = 0b01,
        DEC_8    = 0b10,
        DEC_32   = 0b11
    };

    // FIFO temperature batch data rate
    enum class FIFO_TEMPERATURE_BDR : uint8_t
    {
        DISABLED = 0b00,
        HZ_1_875 = 0b01,
        HZ_15    = 0b10,
        HZ_60    = 0b11
    };

    // Interrupt sources shared by INT1 and INT2
    enum class INTERRUPT : uint8_t
    {
        NOTHING        = 0,
        ACC_DRDY       = 0x01,
        GYR_DRDY       = 0x02,
        FIFO_THRESHOLD = 0x08,
        FIFO_OVERRUN   = 0x10,
        FIFO_FULL      = 0x20,
        COUNTER_BDR    = 0x40
    };

    // Allows combining interrupt sources with bitwise OR
    constexpr friend INTERRUPT operator|(INTERRUPT lhs, INTERRUPT rhs)
    {
        return static_cast<INTERRUPT>(static_cast<uint8_t>(lhs) |
                                      static_cast<uint8_t>(rhs));
    }

    // Sensor configuration
    BDU bdu;

    // Low-g accelerometer config
    LOW_G_ODR odrLowG;
    LOW_G_OPERATING_MODE opModeLowG;
    LOW_G_FULLSCALE fsLowG;

    // High-g accelerometer config
    HIGH_G_ODR odrHighG;
    HIGH_G_FULLSCALE fsHighG;

    // Gyroscope config
    GYR_ODR odrGyr;
    GYR_OPERATING_MODE opModeGyr;
    GYR_FULLSCALE fsGyr;

    // FIFO configuration
    FIFO_MODE fifoMode;
    FIFO_BDR fifoBdrLowG;
    FIFO_BDR fifoBdrGyr;
    bool fifoHighGBatchEnabled;
    FIFO_TIMESTAMP_DECIMATION fifoTimestampDecimation;
    FIFO_TEMPERATURE_BDR fifoTemperatureBdr;

    // FIFO watermark: 1 LSB corresponds to one FIFO word (max 255 in 8-bit WTM)
    uint16_t fifoWatermark;

    // Interrupt routing
    INTERRUPT int1InterruptSelection;
    INTERRUPT int2InterruptSelection;
};

}  // namespace Boardcore
