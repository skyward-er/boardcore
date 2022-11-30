/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <kernel/kernel.h>
#include <radio/Transceiver.h>

#include <cstdint>

#include "SX1278Defs.h"

/*
# Description
## Concurrent access
Access to the bus is managed by the SX1278BusManager, the API is very simple,
lock the bus and then you can access the underlying SPISlave. Most of the time
though, you need the device in a particular mode, in that case you can use the
LockMode API to also change mode. Then via waitForIrq you can comfortably wait
for stuff to happen, and crucially, you can also release the lock to allow
others to access (for example when waiting for packet).
*/

namespace Boardcore
{

/**
 * @brief SX1278 bus access manager.
 */
class SX1278BusManager
{
public:
    using Mode = SX1278Defs::RegOpMode::Mode;
    using Irq  = uint16_t;

    enum Dio : uint8_t
    {
        DIO0 = 1 << 0,
        DIO1 = 1 << 1,
        DIO2 = 1 << 2,
        DIO3 = 1 << 3,
        DIO4 = 1 << 4,
        DIO5 = 1 << 5
    };

private:
    struct DeviceState
    {
        // Current device mode
        Mode mode = Mode::MODE_SLEEP;
        // Thread waiting listening for interrupts
        miosix::Thread *irq_wait_thread = nullptr;
        // What DIOs are we waiting on
        Dio waiting_on_dio = static_cast<Dio>(0);
    };

public:
    /**
     * @brief RAII scoped bus lock guard.
     */
    class Lock
    {
    public:
        explicit Lock(SX1278BusManager &bus) : bus(bus) { bus.lock(); }

        ~Lock() { bus.unlock(); }

    private:
        SX1278BusManager &bus;
    };

    /**
     * @brief RAII scoped bus lock guard, which also changes mode.
     */
    class LockMode
    {
    public:
        LockMode(SX1278BusManager &bus, Mode mode) : bus(bus)
        {
            // cppcheck-suppress useInitializationList
            old_state = bus.lockMode(mode);
        }

        ~LockMode() { bus.unlockMode(old_state); }

    private:
        SX1278BusManager &bus;
        DeviceState old_state;
    };

    SX1278BusManager(SPIBusInterface &bus, miosix::GpioPin cs);

    /**
     * @brief Get underlying bus.
     */
    SPISlave &getBus(Lock &_guard) { return slave; }

    /**
     * @brief Get underlying bus.
     */
    SPISlave &getBus(LockMode &_guard) { return slave; }

    /**
     * @brief Handle generic DIO irq.
     */
    void handleDioIRQ(Dio dio);

    /**
     * @brief Wait for generic irq.
     *
     * WARNING: Currently only supports:
     * - PACKET_SENT
     * - PAYLOAD_READY
     * - TX_READY
     * - FIFO_LEVEL
     *
     * To add more interrupts edit irq_to_dio!
     */
    void waitForIrq(const LockMode &_guard, Irq irq, bool unlock = false);

    /**
     * @brief Busy waits for an interrupt by polling the irq register
     *
     * USE ONLY DURING INITIALIZATION! BAD THINGS *HAVE* HAPPENED DUE TO THIS!
     */
    bool waitForIrqBusy(const LockMode &_guard, Irq irq, int timeout);

private:
    DeviceState lockMode(Mode mode);
    void unlockMode(DeviceState old_state);

    void lock();
    void unlock();

    void enterMode(Mode mode);

    uint16_t getIrqFlags();
    void setMode(Mode mode);

    Dio irq_to_dio(Irq irq)
    {
        Dio dio = static_cast<Dio>(0);
        if (irq & SX1278Defs::RegIrqFlags::PACKET_SENT ||
            irq & SX1278Defs::RegIrqFlags::PAYLOAD_READY)
            dio = DIO0;

        if (irq & SX1278Defs::RegIrqFlags::FIFO_LEVEL)
            dio = static_cast<Dio>(dio | DIO1);

        if (irq & SX1278Defs::RegIrqFlags::TX_READY)
            dio = static_cast<Dio>(dio | DIO3);

        return dio;
    }

    SPISlave slave;
    FastMutex mutex;
    DeviceState state;
};

/**
 * @brief Various SX1278 register/enums definitions.
 */
class SX1278 : public Transceiver
{
public:
    using Dio = SX1278BusManager::Dio;

    /**
     * @brief Channel filter bandwidth.
     */
    enum class RxBw
    {
        HZ_2600   = (0b10 << 3) | 7,
        HZ_3100   = (0b01 << 3) | 7,
        HZ_3900   = 7,
        HZ_5200   = (0b10 << 3) | 6,
        HZ_6300   = (0b01 << 3) | 6,
        HZ_7800   = 6,
        HZ_10400  = (0b10 << 3) | 5,
        HZ_12500  = (0b01 << 3) | 5,
        HZ_15600  = 5,
        HZ_20800  = (0b10 << 3) | 4,
        HZ_25000  = (0b01 << 3) | 4,
        HZ_31300  = 4,
        HZ_41700  = (0b10 << 3) | 3,
        HZ_50000  = (0b01 << 3) | 3,
        HZ_62500  = 3,
        HZ_83300  = (0b10 << 3) | 2,
        HZ_100000 = (0b01 << 3) | 2,
        HZ_125000 = 2,
        HZ_166700 = (0b10 << 3) | 1,
        HZ_200000 = (0b01 << 3) | 1,
        HZ_250000 = 1,
    };

    enum class Shaping
    {
        NONE = SX1278Defs::RegPaRamp::MODULATION_SHAPING_NONE,
        GAUSSIAN_BT_1_0 =
            SX1278Defs::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_1_0,
        GAUSSIAN_BT_0_5 =
            SX1278Defs::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_5,
        GAUSSIAN_BT_0_3 =
            SX1278Defs::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_3,
    };

    enum class DcFree
    {
        NONE       = SX1278Defs::RegPacketConfig1::DC_FREE_NONE,
        MANCHESTER = SX1278Defs::RegPacketConfig1::DC_FREE_MANCHESTER,
        WHITENING  = SX1278Defs::RegPacketConfig1::DC_FREE_WHITENING
    };

    /**
     * @brief Requested SX1278 configuration.
     */
    struct Config
    {
        int freq_rf  = 434000000;        //< RF Frequency in Hz.
        int freq_dev = 50000;            //< Frequency deviation in Hz.
        int bitrate  = 48000;            //< Bitrate in b/s.
        RxBw rx_bw   = RxBw::HZ_125000;  //< Rx filter bandwidth.
        RxBw afc_bw  = RxBw::HZ_125000;  //< Afc filter bandwidth.
        int ocp =
            120;  //< Over current protection limit in mA (0 for no limit).
        int power = 10;  //< Output power in dB (Between +2 and +17, or +20 for
                         // full power).
        Shaping shaping = Shaping::GAUSSIAN_BT_1_0;  //< Shaping applied to
                                                     // the modulation stream.
        DcFree dc_free = DcFree::WHITENING;  //< Dc free encoding scheme.
    };

    /**
     * @brief Error enum.
     */
    enum class Error
    {
        NONE,              //< No error encountered.
        BAD_VALUE,         //< A requested value was outside the valid range.
        BAD_VERSION,       //< Chip isn't connected.
        CONFIGURE_FAILED,  //< Timeout on IRQ register.
    };

    /**
     * @brief Construct a new SX1278
     *
     * @param bus SPI bus used.
     * @param cs Chip select pin.
     */
    SX1278(SPIBusInterface &bus, miosix::GpioPin cs);

    /**
     * @brief Setup the device.
     */
    Error init(const Config &config);

    /*
     * @brief Check if this device is connected.
     */
    bool probe();

    /**
     * @brief Configure this device on the fly.
     */
    bool configure(const Config &config);

    /**
     * @brief Wait until a new packet is received.
     *
     * @param pkt       Buffer to store the received packet into.
     * @param pkt_len   Maximum length of the received data.
     * @return          Size of the data received or -1 if failure
     */
    ssize_t receive(uint8_t *pkt, size_t max_len);

    /**
     * @brief Send a packet.
     * The function must block until the packet is sent (successfully or not)
     *
     * @param pkt       Pointer to the packet (needs to be at least pkt_len
     * bytes).
     * @param pkt_len   Lenght of the packet to be sent.
     * @return          True if the message was sent correctly.
     */
    bool send(uint8_t *pkt, size_t len);

    /**
     * @brief Return device version.
     */
    uint8_t getVersion();

    /**
     * @brief Get the current perceived RSSI in dBm.
     */
    float getCurRssi();

    /**
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    float getLastRxRssi();

    /**
     * @brief Get the frequency error index in Hz, during last packet receive.
     */
    float getLastRxFei();

    /**
     * @brief Handle an incoming interrupt.
     */
    void handleDioIRQ(Dio dio);

    /**
     * @brief Dump all registers via TRACE.
     */
    void debugDumpRegisters();

public:
    using Mode = SX1278BusManager::Mode;

    void rateLimitTx();

    static void imageCalibrate(SPISlave &slave);

    static float getRssi(SPISlave &slave);
    static float getFei(SPISlave &slave);

    static void setBitrate(SPISlave &slave, int bitrate);
    static void setFreqDev(SPISlave &slave, int freq_dev);
    static void setFreqRF(SPISlave &slave, int freq_rf);
    static void setOcp(SPISlave &slave, int ocp);
    static void setSyncWord(SPISlave &slave, uint8_t value[], int size);
    static void setRxBw(SPISlave &slave, RxBw rx_bw);
    static void setAfcBw(SPISlave &slave, RxBw afc_bw);
    static void setPreableLen(SPISlave &slave, int len);
    static void setPa(SPISlave &slave, int power, bool pa_boost);
    static void setShaping(SPISlave &slave, Shaping shaping);

    long long last_tx  = 0;
    float last_rx_rssi = 0.0f;
    SX1278BusManager bus_mgr;
    PrintLogger logger = Logging::getLogger("sx1278");
};

}  // namespace Boardcore
