
#pragma once

#include <cstdint>

namespace Boardcore
{

struct LSM6DSV320XDefs
{
    // General constants
    static constexpr uint8_t WHO_AM_I_VALUE = 0x73; // Expected device ID
    static constexpr uint16_t FIFO_SIZE = 200;  // same as LSM6DSRX, but can be changed if needed
    static constexpr uint16_t SENSOR_FIFO_SIZE = 512; 
    static constexpr uint8_t FIFO_WORD_SIZE = 7; // 1 tag byte + 6 data bytes

    // FIFO tag masks
    static constexpr uint8_t FIFO_TAG_MASK = 0x1F;
    static constexpr uint8_t FIFO_TAG_COUNTER_MASK = 0x60;
    static constexpr uint8_t FIFO_TAG_COUNTER_SHIFT = 5;

    // Registers addresses
    enum class Registers : uint8_t
    {
        // FIFO configuration
        FIFO_CTRL1       = 0x07, // FIFO watermark (lower 8 bits)
        FIFO_CTRL2       = 0x08, // FIFO watermark MSB and FIFO options
        FIFO_CTRL3       = 0x09, // Low-G and gyroscope batching data rates
        FIFO_CTRL4       = 0x0A, // FIFO mode, timestamp and temperature batching

        // FIFO counter and interrupt configuration
        COUNTER_BDR_REG1 = 0x0B, // High-G batching and counter configuration
        COUNTER_BDR_REG2 = 0x0C, // Counter threshold configuration
        INT1_CTRL        = 0x0D, // Interrupt routing to INT1
        INT2_CTRL        = 0x0E, // Interrupt routing to INT2

        // Identification
        WHO_AM_I         = 0x0F, // Device identification

        // sensor configuration
        CTRL1            = 0x10, // Low-G accelerometer ODR and full scale
        CTRL2            = 0x11, // Gyroscope ODR and full scale
        CTRL3            = 0x12, // BDU, byte order and interface configuration
        CTRL4            = 0x13, // Interrupt and sensor configuration
        CTRL5            = 0x14, // Sensor configuration
        CTRL6            = 0x15, // Low-G accelerometer operating mode
        CTRL7            = 0x16, // Gyroscope and High-G interrupt configuration
        CTRL8            = 0x17, // Low-G accelerometer configuration
        CTRL9            = 0x18, // Low-G accelerometer configuration
        CTRL10           = 0x19, // General sensor configuration

        // Status registers
        CTRL_STATUS      = 0x1A, // Internal configuration status
        FIFO_STATUS1     = 0x1B, // FIFO unread words: lower bits
        FIFO_STATUS2     = 0x1C, // FIFO unread words: upper bits and status flags
        ALL_INT_SRC      = 0x1D, // Interrupt source status
        STATUS_REG       = 0x1E, // Accelerometer and gyroscope data-ready flags

        // Temperature output
        OUT_TEMP_L       = 0x20, // Temperature output: low byte
        OUT_TEMP_H       = 0x21, // Temperature output: high byte

        // Gyroscope output
        OUTX_L_G         = 0x22, // Gyroscope X: low byte
        OUTX_H_G         = 0x23, // Gyroscope X: high byte
        OUTY_L_G         = 0x24, // Gyroscope Y: low byte
        OUTY_H_G         = 0x25, // Gyroscope Y: high byte
        OUTZ_L_G         = 0x26, // Gyroscope Z: low byte
        OUTZ_H_G         = 0x27, // Gyroscope Z: high byte

        // Low-g accelerometer output
        OUTX_L_A         = 0x28, // Low-G X: low byte
        OUTX_H_A         = 0x29, // Low-G X: high byte
        OUTY_L_A         = 0x2A, // Low-G Y: low byte
        OUTY_H_A         = 0x2B, // Low-G Y: high byte
        OUTZ_L_A         = 0x2C, // Low-G Z: low byte
        OUTZ_H_A         = 0x2D, // Low-G Z: high byte

        // High-g accelerometer output
        UI_OUTX_L_A_OIS_HG = 0x34, // High-G X: low byte
        UI_OUTX_H_A_OIS_HG = 0x35, // High-G X: high byte
        UI_OUTY_L_A_OIS_HG = 0x36, // High-G Y: low byte
        UI_OUTY_H_A_OIS_HG = 0x37, // High-G Y: high byte
        UI_OUTZ_L_A_OIS_HG = 0x38, // High-G Z: low byte
        UI_OUTZ_H_A_OIS_HG = 0x39, // High-G Z: high byte

        // Timestamp
        TIMESTAMP0       = 0x40, // Timestamp: byte 0 (LSB)
        TIMESTAMP1       = 0x41, // Timestamp: byte 1
        TIMESTAMP2       = 0x42, // Timestamp: byte 2
        TIMESTAMP3       = 0x43, // Timestamp: byte 3 (MSB)

        // High-g accelerometer configuration
        CTRL2_XL_HG      = 0x4D, // High-G self-test configuration
        CTRL1_XL_HG      = 0x4E, // High-G ODR, full scale and output routing

        // Internal frequency
        INTERNAL_FREQ_FINE = 0x4F, // Internal oscillator frequency correction

        // FIFO output
        FIFO_DATA_OUT_TAG = 0x78, // FIFO word tag: identifies the data source
        FIFO_DATA_OUT_X_L = 0x79, // FIFO sample X: low byte
        FIFO_DATA_OUT_X_H = 0x7A, // FIFO sample X: high byte
        FIFO_DATA_OUT_Y_L = 0x7B, // FIFO sample Y: low byte
        FIFO_DATA_OUT_Y_H = 0x7C, // FIFO sample Y: high byte
        FIFO_DATA_OUT_Z_L = 0x7D, // FIFO sample Z: low byte
        FIFO_DATA_OUT_Z_H = 0x7E  // FIFO sample Z: high byte
    };

    // FIFO sensor tags
    enum class FifoTag : uint8_t
    {
        EMPTY                  = 0x00,
        GYROSCOPE              = 0x01,
        LOW_G_ACCELEROMETER    = 0x02,
        TEMPERATURE            = 0x03,
        TIMESTAMP              = 0x04,
        CONFIG_CHANGE          = 0x05,

        HIGH_G_PEAK             = 0x18,
        HIGH_G_ACCELEROMETER   = 0x1D
    };

    // Raw FIFO word
    struct RawFifoData
    {
        uint8_t tag; // Sensor tag

        uint8_t xLow;  // X axis: low byte
        uint8_t xHigh; // X axis: high byte

        uint8_t yLow;  // Y axis: low byte
        uint8_t yHigh; // Y axis: high byte

        uint8_t zLow;  // Z axis: low byte
        uint8_t zHigh; // Z axis: high byte
    };
};

}
