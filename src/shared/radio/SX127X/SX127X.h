
#pragma once

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "SX127XFsk.h"
#include "SX127XLora.h"

namespace Boardcore
{
class SX127X
{
public:
    struct Timeouts
    {
        int txPacketMs = 800;
        int rxPollMs   = 20;
        int opMs       = 200;
    };

    struct Buffers
    {
        size_t txBytes = 1024;
        size_t rxBytes = 1024;
    };

    struct FskConfig
    {
        SX127XFsk::Config phy;
    };

    struct LoraConfig
    {
        SX127XLora::Config phy;
    };

    struct Config
    {
        enum class Mode
        {
            FSK,
            LORA
        };

        Config() = default;

        Config(const FskConfig& cfg) : mode(Mode::FSK), fsk(cfg) {}

        Config(const LoraConfig& cfg) : mode(Mode::LORA), lora(cfg) {}

        Config& operator=(const FskConfig& cfg)
        {
            mode = Mode::FSK;
            fsk  = cfg;
            return *this;
        }

        Config& operator=(const LoraConfig& cfg)
        {
            mode = Mode::LORA;
            lora = cfg;
            return *this;
        }

        Mode mode = Mode::FSK;
        FskConfig fsk{};
        LoraConfig lora{};
    };

    struct Diagnostics
    {
        uint32_t txPackets        = 0;
        uint32_t rxPackets        = 0;
        uint32_t txDroppedPackets = 0;
        uint32_t rxDroppedPackets = 0;
        uint32_t rxDroppedBytes   = 0;
        uint32_t recoveries       = 0;

        float lastRxRssiDbm = NAN;
        float lastRxFeiHz   = NAN;
        float lastRxSnrDb   = NAN;
    };

    SX127X(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin dio0,
           miosix::GpioPin dio1, miosix::GpioPin dio3,
           SPI::ClockDivider clock_divider,
           std::unique_ptr<SX127x::ISX127XFrontend> frontend, Config config,
           Buffers buffers, Timeouts timeouts, size_t packetMtu = 255);

    ~SX127X();

    SX127X(const SX127X&)            = delete;
    SX127X& operator=(const SX127X&) = delete;

    bool start(unsigned int stack = STACK_MIN_FOR_SKYWARD,
               int priority = 2);
    void stop();

    void handleDioIRQ();

    bool enqueuePacket(const uint8_t* data, size_t len);
    ssize_t dequeuePacket(uint8_t* out, size_t maxLen);

    size_t write(const uint8_t* data, size_t len);
    size_t read(uint8_t* out, size_t maxLen);
    size_t available() const;

    Diagnostics getDiagnostics() const;

    bool setFrequency(uint32_t freqHz);
 
    bool setTxPower(int8_t powerDbm);

private:
    static void threadLauncher(void* arg);
    void threadMain();

    bool popTxPacket(std::vector<uint8_t>& pkt);
    void pushRxPacket(const uint8_t* pkt, size_t len);

    class FramedByteRing
    {
    public:
        explicit FramedByteRing(size_t capacity);

        bool pushFrame(const uint8_t* data, size_t len,
                       uint32_t* droppedBytes);
        bool popFrame(std::vector<uint8_t>& out);

        ssize_t packetPop(uint8_t* out, size_t maxLen);

        size_t streamRead(uint8_t* out, size_t maxLen);
        size_t streamAvailable() const;

    private:
        bool hasFrameUnlocked() const;
        void dropOldestUnlocked(uint32_t* droppedBytes);
        size_t freeSpaceUnlocked() const;

        mutable miosix::FastMutex mtx;
        std::vector<uint8_t> buf;
        size_t head = 0;
        size_t tail = 0;
        size_t used = 0;

        std::vector<uint8_t> cur;
        size_t curOff = 0;
    };

    SPIBus& bus;
    miosix::GpioPin cs;
    miosix::GpioPin dio0;
    miosix::GpioPin dio1;
    miosix::GpioPin dio3;
    SPI::ClockDivider clock_divider;
    std::unique_ptr<SX127x::ISX127XFrontend> frontend;
    Config config;
    Timeouts timeouts;
    size_t packetMtu;

    std::unique_ptr<SX127x::ISX127X> dev;

    FramedByteRing txRing;
    FramedByteRing rxRing;

    mutable miosix::FastMutex diagMtx;
    Diagnostics diag;

    std::atomic<bool> running{false};
    std::atomic<bool> stopReq{false};
    miosix::Thread* th = nullptr;
};

}  // namespace Boardcore
