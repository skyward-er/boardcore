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
The device is stateful, in order to send/receive data you need to set in a
particular state. This is handled by SX1278BusManager::setMode. This causes a
lot of issues with multithreaded code, and receiving/transmitting concurrently.

So the driver wraps the bus in a BusManager for this reason, in order to perform
anything you need to lock the bus. The driver then locks the internal mutex and
sets the device in the correct mode for that operation.

But this causes problems when receiving, as receive can block indefinetly, and
would prevent any thread from actually sending stuff. The solution is to release
the lock only when waiting for a packet. This allows other threads to lock it
and change mode, and unlocking the bus resets its state back to RX so it can
continue to listen for incoming packets.

The BusManager also multiplexes who is currently waiting for an interrupt. This
is needed because due to the limited interrupts pins, the device signals
multiples of them on one pin. And for some random reasons, the designers decided
to put "packet received" and "packed sent" on same pin (DIO0)...

Again, the solution is to track who is actually using the bus, so when sending a
packet that triggers the "packet sent" interrupt, the manager knows to dispatch
it to the currect thread, and not wake up the RX thread.
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
     * @brief RAII scoped bus lock guard.
     */
    class LockMode
    {
    public:
        LockMode(SX1278BusManager &bus, Mode mode) : bus(bus)
        {
            bus.lockMode(mode);
        }

        ~LockMode() { bus.unlockMode(); }

    private:
        SX1278BusManager &bus;
    };

    SX1278BusManager(SPIBusInterface &bus, miosix::GpioPin cs);

    /**
     * @brief Lock bus for exclusive access (does not change mode).
     */
    void lock();

    /**
     * @brief Release bus for exclusive access.
     */
    void unlock();

    /**
     * @brief Lock bus for exclusive access.
     *
     * @param mode Device mode requested.
     */
    void lockMode(Mode mode);

    /**
     * @brief Release bus for exclusive access.
     */
    void unlockMode();

    /**
     * @brief Get underlying bus.
     */
    SPISlave &getBus();

    /**
     * @brief Handle DIO0 irq.
     */
    void handleDioIRQ();

    /**
     * @brief Wait for generic irq (DOES NOT RELEASE LOCK!).
     */
    void waitForIrq(uint16_t mask);

    /**
     * @brief Wait for RX irq (releases lock safely).
     */
    void waitForRxIrq();

private:
    void enterMode(Mode mode);

    uint16_t getIrqFlags();
    void setMode(Mode mode);

    miosix::Thread *irq_wait_thread = nullptr;
    miosix::Thread *rx_wait_thread  = nullptr;
    miosix::FastMutex mutex;

    SPISlave slave;
    Mode mode;
};

/**
 * @brief Various SX1278 register/enums definitions.
 */
class SX1278 : public Transceiver
{
public:
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
        NONE,         //< No error encountered.
        BAD_VALUE,    //< A requested value was outside the valid range.
        BAD_VERSION,  //< Chip isn't connected.
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
    void configure(const Config &config);

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
    void handleDioIRQ();

    /**
     * @brief Dump all registers via TRACE.
     */
    void debugDumpRegisters();

public:
    using Mode = SX1278BusManager::Mode;

    void rateLimitTx();
    void imageCalibrate();

    float getRssi();
    float getFei();

    void setBitrate(int bitrate);
    void setFreqDev(int freq_dev);
    void setFreqRF(int freq_rf);
    void setOcp(int ocp);
    void setSyncWord(uint8_t value[], int size);
    void setRxBw(RxBw rx_bw);
    void setAfcBw(RxBw afc_bw);
    void setPreableLen(int len);
    void setPa(int power, bool pa_boost);
    void setShaping(Shaping shaping);

    void waitForIrq(uint16_t mask);

    long long last_tx  = 0;
    float last_rx_rssi = 0.0f;
    SX1278BusManager bus_mgr;
    PrintLogger logger = Logging::getLogger("sx1278");
};

}  // namespace Boardcore
