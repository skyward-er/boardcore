
#include "SX127X.h"

#include <utils/Debug.h>
#include <utils/KernelTime.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace Boardcore
{

namespace
{

int loraPacketTimeMs(const SX127XLora::Config& cfg, size_t payloadLen)
{
    using namespace SX127x::Lora;

    const auto bwEnum =
        static_cast<RegModemConfig1::Bw>(cfg.bandwidth);
    const auto sfEnum =
        static_cast<RegModemConfig2::Sf>(cfg.spreading_factor);
    const auto crEnum =
        static_cast<RegModemConfig1::Cr>(cfg.coding_rate);

    const int bw = static_cast<int>(RegModemConfig1::bandwidthToInt(bwEnum));
    const int sf = static_cast<int>(sfEnum);
    const int cr = static_cast<int>(crEnum);
    const int crc = cfg.enable_crc ? 1 : 0;
    const int ih  = 0;
    const int de =
        (cfg.low_data_rate_optimize || symbolDuration(sf, bw) > 16000) ? 1 : 0;

    const double tSymMs = (static_cast<double>(1u << sf) * 1000.0) / bw;
    const double tPreambleMs = (8.0 + 4.25) * tSymMs;

    const double payloadNumerator =
        8.0 * static_cast<double>(payloadLen) - 4.0 * sf + 28.0 +
        16.0 * crc - 20.0 * ih;
    const double payloadDenominator = 4.0 * (sf - 2 * de);
    const double codingOverhead = cr + 4;
    const double payloadSymbols =
        8.0 + std::max(std::ceil(payloadNumerator / payloadDenominator) *
                           codingOverhead,
                       0.0);

    return static_cast<int>(std::ceil(tPreambleMs + payloadSymbols * tSymMs));
}

int computeTxTimeoutMs(const SX127X::Config& config,
                       const SX127X::Timeouts& timeouts, size_t payloadLen)
{
    if (config.mode != SX127X::Config::Mode::LORA)
        return timeouts.txPacketMs;

    return std::max(timeouts.txPacketMs,
                    loraPacketTimeMs(config.lora.phy, payloadLen) + 100);
}

int computeRxTimeoutMs(const SX127X::Config& config,
                       const SX127X::Timeouts& timeouts, size_t packetMtu)
{
    if (config.mode != SX127X::Config::Mode::LORA)
        return timeouts.rxPollMs;

    return std::max(timeouts.rxPollMs,
                    loraPacketTimeMs(config.lora.phy, packetMtu) + 50);
}

}  // namespace

SX127X::FramedByteRing::FramedByteRing(size_t capacity)
{
    buf.resize(std::max<size_t>(capacity, 8));
}

size_t SX127X::FramedByteRing::freeSpaceUnlocked() const
{
    return buf.size() - used;
}

bool SX127X::FramedByteRing::hasFrameUnlocked() const
{
    if (used < 2)
        return false;

    auto peek = [&](size_t off) -> uint8_t { return buf[(tail + off) % buf.size()]; };
    uint16_t len = static_cast<uint16_t>(peek(0)) |
                   (static_cast<uint16_t>(peek(1)) << 8);
    return used >= (size_t)(2 + len);
}

void SX127X::FramedByteRing::dropOldestUnlocked(uint32_t* droppedBytes)
{
    if (!hasFrameUnlocked())
    {
        if (droppedBytes)
            *droppedBytes += used;
        tail = head;
        used = 0;
        cur.clear();
        curOff = 0;
        return;
    }

    auto peek = [&](size_t off) -> uint8_t { return buf[(tail + off) % buf.size()]; };
    uint16_t len = static_cast<uint16_t>(peek(0)) |
                   (static_cast<uint16_t>(peek(1)) << 8);

    size_t drop = 2 + len;
    if (droppedBytes)
        *droppedBytes += len;

    tail = (tail + drop) % buf.size();
    used -= drop;
}

bool SX127X::FramedByteRing::pushFrame(const uint8_t* data, size_t len,
                                      uint32_t* droppedBytes)
{
    if (len > 0xFFFF)
        return false;

    const size_t need = 2 + len;
    miosix::Lock<miosix::FastMutex> l(mtx);

    if (need > buf.size())
        return false;

    while (freeSpaceUnlocked() < need)
        dropOldestUnlocked(droppedBytes);

    auto put = [&](uint8_t v) {
        buf[head] = v;
        head = (head + 1) % buf.size();
        used++;
    };

    put(static_cast<uint8_t>(len & 0xFF));
    put(static_cast<uint8_t>((len >> 8) & 0xFF));
    for (size_t i = 0; i < len; i++)
        put(data[i]);

    return true;
}

bool SX127X::FramedByteRing::popFrame(std::vector<uint8_t>& out)
{
    miosix::Lock<miosix::FastMutex> l(mtx);
    if (!hasFrameUnlocked())
        return false;

    auto get = [&]() -> uint8_t {
        uint8_t v = buf[tail];
        tail = (tail + 1) % buf.size();
        used--;
        return v;
    };

    uint16_t len = static_cast<uint16_t>(get()) |
                   (static_cast<uint16_t>(get()) << 8);
    out.resize(len);
    for (size_t i = 0; i < len; i++)
        out[i] = get();

    return true;
}

ssize_t SX127X::FramedByteRing::packetPop(uint8_t* out, size_t maxLen)
{
    std::vector<uint8_t> pkt;
    if (!popFrame(pkt))
        return 0;

    if (pkt.size() > maxLen)
    {
        return -1;
    }

    memcpy(out, pkt.data(), pkt.size());
    return static_cast<ssize_t>(pkt.size());
}

size_t SX127X::FramedByteRing::streamAvailable() const
{
    miosix::Lock<miosix::FastMutex> l(mtx);

    size_t avail = 0;

    if (!cur.empty() && curOff < cur.size())
        avail += (cur.size() - curOff);

    size_t tmpTail = tail;
    size_t tmpUsed = used;

    auto peek = [&](size_t base, size_t off) -> uint8_t {
        return buf[(base + off) % buf.size()];
    };

    while (tmpUsed >= 2)
    {
        uint16_t len = static_cast<uint16_t>(peek(tmpTail, 0)) |
                       (static_cast<uint16_t>(peek(tmpTail, 1)) << 8);
        if (tmpUsed < (size_t)(2 + len))
            break;

        avail += len;
        tmpTail = (tmpTail + 2 + len) % buf.size();
        tmpUsed -= (2 + len);
    }

    return avail;
}

size_t SX127X::FramedByteRing::streamRead(uint8_t* out, size_t maxLen)
{
    size_t outN = 0;

    while (outN < maxLen)
    {
        if (cur.empty() || curOff >= cur.size())
        {
            std::vector<uint8_t> pkt;
            if (!popFrame(pkt))
                break;
            cur = std::move(pkt);
            curOff = 0;
        }

        size_t n = std::min(maxLen - outN, cur.size() - curOff);
        memcpy(out + outN, cur.data() + curOff, n);
        curOff += n;
        outN += n;
    }

    return outN;
}

SX127X::SX127X(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin dio0,
               miosix::GpioPin dio1, miosix::GpioPin dio3,
               SPI::ClockDivider clock_divider,
               std::unique_ptr<SX127x::ISX127XFrontend> frontend, Config config,
               Buffers buffers, Timeouts timeouts, size_t packetMtu)
    : bus(bus), cs(cs), dio0(dio0), dio1(dio1), dio3(dio3),
      clock_divider(clock_divider), frontend(std::move(frontend)),
      config(std::move(config)), timeouts(timeouts), packetMtu(packetMtu),
      txRing(buffers.txBytes), rxRing(buffers.rxBytes)
{
    if (this->packetMtu == 0)
        this->packetMtu = 1;

    if (this->config.mode == Config::Mode::FSK)
    {
        auto impl = std::make_unique<SX127XFsk>(bus, cs, dio0, dio1, dio3,
                                                clock_divider,
                                                std::move(this->frontend));
        auto& cfg = this->config.fsk.phy;
        auto err  = impl->init(cfg);
        if (err == SX127XFsk::Error::NONE)
            dev = std::move(impl);
        else
            TRACE("[sx127x-core] fsk init failed err=%d\n",
                  static_cast<int>(err));
    }
    else
    {
        auto impl = std::make_unique<SX127XLora>(bus, cs, dio0, dio1, dio3,
                                                 clock_divider,
                                                 std::move(this->frontend));
        auto& cfg = this->config.lora.phy;
        auto err  = impl->init(cfg);
        if (err == SX127XLora::Error::NONE)
            dev = std::move(impl);
        else
            TRACE("[sx127x-core] lora init failed err=%d\n",
                  static_cast<int>(err));
    }
}

SX127X::~SX127X() { stop(); }

bool SX127X::start(unsigned int stack, int priority)
{
    if (running.load())
        return true;

    if (!dev)
        return false;

    stopReq.store(false);
    th = miosix::Thread::create(threadLauncher, stack, priority, this,
                               miosix::Thread::JOINABLE);
    if (!th)
        return false;

    running.store(true);
    return true;
}

void SX127X::stop()
{
    if (!running.load())
        return;

    stopReq.store(true);

    if (th)
    {
        th->join();
        th = nullptr;
    }

    running.store(false);
}

void SX127X::handleDioIRQ()
{
    if (dev)
        dev->handleDioIRQ();
}

bool SX127X::enqueuePacket(const uint8_t* data, size_t len)
{
    if (!dev)
        return false;

    if (!data && len != 0)
        return false;

    if (len > packetMtu)
        return false;

    bool ok = txRing.pushFrame(data, len, nullptr);
    if (!ok)
    {
        miosix::Lock<miosix::FastMutex> l(diagMtx);
        diag.txDroppedPackets++;
        return false;
    }

    return true;
}

ssize_t SX127X::dequeuePacket(uint8_t* out, size_t maxLen)
{
    auto r = rxRing.packetPop(out, maxLen);
    return r;
}

size_t SX127X::write(const uint8_t* data, size_t len)
{
    if (!data && len != 0)
        return 0;

    size_t written = 0;
    while (written < len)
    {
        size_t chunk = std::min(packetMtu, len - written);
        if (!enqueuePacket(data + written, chunk))
            break;
        written += chunk;
    }
    return written;
}

size_t SX127X::read(uint8_t* out, size_t maxLen) { return rxRing.streamRead(out, maxLen); }

size_t SX127X::available() const { return rxRing.streamAvailable(); }

SX127X::Diagnostics SX127X::getDiagnostics() const
{
    miosix::Lock<miosix::FastMutex> l(diagMtx);
    auto d = diag;
    if (dev)
    {
        d.lastRxRssiDbm = dev->getLastRxRssi();
        d.lastRxFeiHz   = dev->getLastRxFei();
        d.lastRxSnrDb   = dev->getLastRxSnr();
    }
    return d;
}

bool SX127X::setFrequency(uint32_t freqHz)
{
    if (!dev)
        return false;
    return dev->setFrequency(freqHz);
}

bool SX127X::setTxPower(int8_t powerDbm)
{
    if (!dev)
        return false;
    if (running.load())
        return false;

    if (config.mode == Config::Mode::FSK)
    {
        auto* fsk = static_cast<SX127XFsk*>(dev.get());
        auto phy  = config.fsk.phy;
        phy.power = powerDbm;

        if (fsk->configure(phy) != SX127XFsk::Error::NONE)
            return false;

        config.fsk.phy = phy;
        return true;
    }

    auto* lora = static_cast<SX127XLora*>(dev.get());
    auto phy   = config.lora.phy;
    phy.power  = powerDbm;

    if (lora->configure(phy) != SX127XLora::Error::NONE)
        return false;

    config.lora.phy = phy;
    return true;
}

void SX127X::threadLauncher(void* arg)
{
    static_cast<SX127X*>(arg)->threadMain();
}

bool SX127X::popTxPacket(std::vector<uint8_t>& pkt) { return txRing.popFrame(pkt); }

void SX127X::pushRxPacket(const uint8_t* pkt, size_t len)
{
    uint32_t dropped = 0;
    bool ok          = rxRing.pushFrame(pkt, len, &dropped);

    miosix::Lock<miosix::FastMutex> l(diagMtx);
    if (!ok)
        diag.rxDroppedPackets++;
    diag.rxDroppedBytes += dropped;
}

void SX127X::threadMain()
{
    std::vector<uint8_t> pkt;
    std::vector<uint8_t> rxbuf;
    rxbuf.resize(packetMtu);

    while (!stopReq.load())
    {
        bool didWork = false;

        if (popTxPacket(pkt))
        {
            didWork = true;
            int txTimeoutMs = computeTxTimeoutMs(config, timeouts, pkt.size());
            bool ok = dev->sendTimeout(pkt.data(), pkt.size(), txTimeoutMs);
            bool needRecovery = false;

            {
                miosix::Lock<miosix::FastMutex> l(diagMtx);
                if (ok)
                {
                    diag.txPackets++;
                }
                else
                {
                    diag.txDroppedPackets++;
                    diag.recoveries++;
                    needRecovery = true;
                }
            }

            if (!ok)
                TRACE("[sx127x-core] tx failed len=%u, starting recovery\n",
                      static_cast<unsigned int>(pkt.size()));

            if (needRecovery)
            {
                bool recovered = false;
                if (config.mode == Config::Mode::FSK)
                {
                    auto* fsk = static_cast<SX127XFsk*>(dev.get());
                    recovered = fsk->init(config.fsk.phy) ==
                                SX127XFsk::Error::NONE;
                }
                else
                {
                    auto* lora = static_cast<SX127XLora*>(dev.get());
                    recovered = lora->init(config.lora.phy) ==
                                SX127XLora::Error::NONE;
                }

                if (!recovered)
                {
                    TRACE("[sx127x-core] recovery failed, stopping device\n");
                    dev.reset();
                    stopReq.store(true);
                }
                else
                {
                    TRACE("[sx127x-core] recovery completed\n");
                }
            }
        }

        if (!didWork && dev)
        {
            int rxTimeoutMs = computeRxTimeoutMs(config, timeouts, packetMtu);
            ssize_t r = dev->receiveTimeout(rxbuf.data(), rxbuf.size(),
                                            rxTimeoutMs);

            if (r > 0)
            {
                pushRxPacket(rxbuf.data(), static_cast<size_t>(r));
                miosix::Lock<miosix::FastMutex> l(diagMtx);
                diag.rxPackets++;
            }
            else if (r == -1)
            {
                TRACE("[sx127x-core] rx packet rejected by driver\n");
            }
        }

    }
}

}  // namespace Boardcore
