/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Skyward
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file system-show.cpp
 * @brief Linear Boardcore demo for stm32f767zi_skyward_compute_unit.
 *
 * Runs one self-test after another, prints the result on USART1 (kernel
 * console) and blinks the LEDs once per PASS. No CLI, no USART3.
 *
 * Tests: Stats, MovingAverage, CircularBuffer, SyncPacketQueue,
 * SkyQuaternion, EventBroker, FSM, PI controller, TaskScheduler, CpuMeter,
 * DMA mem-to-mem, PWM timer init, I2C bus scan.
 */

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

#include <Eigen/Core>

#include "algorithms/PIController.h"
#include "diagnostic/CpuMeter/CpuMeter.h"
#include "drivers/dma/DMA.h"
#include "drivers/i2c/I2CDriver.h"
#include "drivers/timer/PWM.h"
#include "events/EventBroker.h"
#include "events/FSM.h"
#include "events/utils/EventCounter.h"
#include "miosix.h"
#include "scheduler/TaskScheduler.h"
#include "utils/MovingAverage.h"
#include "utils/SkyQuaternion/SkyQuaternion.h"
#include "utils/Stats/Stats.h"
#include "utils/collections/CircularBuffer.h"
#include "utils/collections/SyncPacketQueue.h"

#include <interfaces-impl/bsp_impl.h>

using namespace miosix;
using namespace Boardcore;

// Visible LEDs: PC13 (led3), PC2 (led4), PC14, PC15 (+ BSP PB7/PE3/PG9)
static GpioPin ledPc14(GPIOC_BASE, 14);
static GpioPin ledPc15(GPIOC_BASE, 15);

volatile uint32_t g_passCount = 0;
volatile uint32_t g_totalCount = 0;
volatile uint32_t g_failFirst  = 0;

static void allLedsOn()
{
    //led1On();
    //led2On();
    led3On();
    userLed4::high();
    ledPc14.high();
    ledPc15.high();
}

static void allLedsOff()
{
    //led1Off();
    //led2Off();
    led3Off();
    userLed4::low();
    ledPc14.low();
    ledPc15.low();
}

static void blinkOnce()
{
    allLedsOn();
    Thread::sleep(80);
    allLedsOff();
    Thread::sleep(80);
}

static void report(const char* name, bool ok)
{
    g_totalCount++;
    if (ok)
        g_passCount++;
    else if (g_failFirst == 0)
        g_failFirst = g_totalCount;
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (ok)
        blinkOnce();
    else
    {
        for (int i = 0; i < 3; i++)
        {
            allLedsOn();
            Thread::sleep(200);
            allLedsOff();
            Thread::sleep(200);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static bool testStats()
{
    Stats s;
    for (int i = 1; i <= 5; i++)
        s.add(static_cast<float>(i));
    StatsResult r = s.getStats();
    return r.nSamples == 5 && std::abs(r.mean - 3.0f) < 1e-5f &&
           std::abs(r.minValue - 1.0f) < 1e-5f &&
           std::abs(r.maxValue - 5.0f) < 1e-5f;
}

static bool testMovingAverage()
{
    MovingAverage<float, 4> avg;
    for (int i = 1; i <= 4; i++)
        avg.push(static_cast<float>(i));
    return std::abs(avg.getAverage() - 2.5f) < 1e-5f;
}

static bool testCircularBuffer()
{
    CircularBuffer<int, 4> b;
    for (int i = 1; i <= 4; i++)
        b.put(i);
    int expected = 1;
    while (!b.isEmpty())
    {
        if (b.pop() != expected)
            return false;
        expected++;
    }
    return expected == 5;
}

static bool testSyncPacketQueue()
{
    SyncPacketQueue<16, 2> q;
    const char msg[] = "hello";
    if (!q.put(reinterpret_cast<uint8_t*>(const_cast<char*>(msg)),
               sizeof(msg) - 1))
        return false;
    Packet<16> p = q.pop();
    uint8_t out[16] = {0};
    size_t n        = p.dump(out);
    return n == sizeof(msg) - 1 && memcmp(out, msg, n) == 0;
}

static bool testSkyQuaternion()
{
    using namespace Boardcore::SkyQuaternion;
    using namespace Eigen;
    const Vector3f in(30.0f, 10.0f, -20.0f);
    const Vector3f back = quat2eul(eul2quat(in));
    const Vector4f q45  = eul2quat(Vector3f(45.0f, 0.0f, 0.0f));
    const Vector3f eul  = quat2eul(quatProd(q45, q45));
    return (back - Vector3f(-20.0f, 10.0f, 30.0f)).norm() < 0.2f &&
           std::abs(eul.z() - 90.0f) < 0.1f;
}

static bool g_brokerStarted = false;
static constexpr uint8_t SHOW_TOPIC = 1;
static constexpr Event SHOW_EVENT = 7;

static bool testEventBroker()
{
    if (!g_brokerStarted)
    {
        EventBroker::getInstance().start();
        g_brokerStarted = true;
    }
    EventCounter counter(EventBroker::getInstance());
    counter.subscribe(SHOW_TOPIC);
    for (int i = 0; i < 10; i++)
        EventBroker::getInstance().post(Event{SHOW_EVENT}, SHOW_TOPIC);
    Thread::sleep(200);
    return counter.getCount(SHOW_EVENT) == 10;
}

namespace
{
constexpr Event EV_GO = EV_FIRST_CUSTOM;

class MiniFsm : public FSM<MiniFsm>
{
public:
    MiniFsm() : FSM(&MiniFsm::stateIdle)
    {
        EventBroker::getInstance().subscribe(this, SHOW_TOPIC);
    }
    ~MiniFsm() { EventBroker::getInstance().unsubscribe(this); }

    void stateIdle(const Event& e)
    {
        if (e == EV_GO)
            transition(&MiniFsm::stateDone);
    }
    void stateDone(const Event&) {}
};
}  // namespace

static bool testFsm()
{
    MiniFsm fsm;
    fsm.start();
    EventBroker::getInstance().post(Event{EV_GO}, SHOW_TOPIC);
    Thread::sleep(200);
    return fsm.testState(&MiniFsm::stateDone);
}

static bool testPIController()
{
    PIController pi(1.0f, 0.1f, 0.1f);
    float first = pi.update(1.0f);
    for (int i = 0; i < 200; i++)
        pi.update(1.0f);
    return pi.update(1.0f) > first + 1.0f;
}

static std::atomic<unsigned int> g_ticksA{0};
static std::atomic<unsigned int> g_ticksB{0};

static void taskA() { ++g_ticksA; }
static void taskB() { ++g_ticksB; }

static bool testTaskScheduler()
{
    g_ticksA = 0;
    g_ticksB = 0;
    TaskScheduler sched;
    sched.addTask(&taskA, 50, TaskScheduler::Policy::RECOVER);
    sched.addTask(&taskB, 100, TaskScheduler::Policy::RECOVER);
    if (!sched.start())
        return false;
    Thread::sleep(1000);
    sched.stop();
    return g_ticksA.load() >= 12 && g_ticksB.load() >= 6;
}

static bool testCpuMeter()
{
    CpuMeter::resetCpuStats();
    Thread::sleep(700);
    CpuMeterData d = CpuMeter::getCpuStats();
    return d.nSamples >= 2 && d.mean >= 0.0f && d.mean <= 100.0f;
}

static bool testDmaMemToMem()
{
    uint32_t src[8];
    uint32_t dst[8] = {0};
    for (int i = 0; i < 8; i++)
        src[i] = 0x11111111u * static_cast<uint32_t>(i + 1);

    // Memory-to-memory is only supported by DMA2 on the STM32F7
    DMAStreamGuard stream = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_MEM_ONLY,
        std::chrono::milliseconds(250));
    if (!stream.isValid())
        return false;

    DMATransaction tx{
        .direction           = DMATransaction::Direction::MEM_TO_MEM,
        .priority            = DMATransaction::Priority::MEDIUM,
        .srcSize             = DMATransaction::DataSize::BITS_32,
        .dstSize             = DMATransaction::DataSize::BITS_32,
        .srcAddress          = src,
        .dstAddress          = dst,
        .secondMemoryAddress = nullptr,
        .numberOfDataItems   = 8,
        .srcIncrement        = true,
        .dstIncrement        = true,
        .enableTransferCompleteInterrupt = true,
    };

    stream->setup(tx);
    stream->enable();
    bool ok = stream->timedWaitForTransferComplete(
        std::chrono::milliseconds(250));
    stream->disable();

    tx.enableTransferCompleteInterrupt = false;
    stream->setup(tx);

    return ok && memcmp(dst, src, sizeof(src)) == 0;
}

static bool probePwm()
{
    bool running = false;
    {
        PWM pwm(TIM4, 1000);
        running = (TIM4->CR1 & TIM_CR1_CEN) != 0 && TIM4->ARR > 0;
    }
    return running;
}

static bool probeI2c()
{
    GpioPin scl(GPIOB_BASE, 8);
    GpioPin sda(GPIOB_BASE, 9);
    unsigned int found = 0;
    {
        I2CDriver i2c(I2C1, scl, sda);
        for (unsigned int addr = 0x08; addr <= 0x77; addr++)
        {
            I2CDriver::I2CSlaveConfig config;
            config.slaveAddress = addr;
            config.speed        = I2CDriver::Speed::STANDARD;
            uint8_t byte        = 0;
            if (i2c.read(config, &byte, 1))
                found++;
        }
    }
    printf("I2C1 scan: %u device(s) found\n", found);
    return true;  // bus was initialized and scanned
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    ledPc14.mode(Mode::OUTPUT);
    ledPc15.mode(Mode::OUTPUT);
    allLedsOff();

    printf("Boardcore system show started\n");
    Thread::sleep(200);

    report("utils/Stats", testStats());
    report("utils/MovingAverage", testMovingAverage());
    report("utils/CircularBuffer", testCircularBuffer());
    report("utils/SyncPacketQueue", testSyncPacketQueue());
    report("utils/SkyQuaternion", testSkyQuaternion());
    report("events/EventBroker", testEventBroker());
    report("events/FSM", testFsm());
    report("algorithms/PIController", testPIController());
    report("scheduler/TaskScheduler", testTaskScheduler());
    report("diagnostic/CpuMeter", testCpuMeter());
    report("drivers/dma (mem-to-mem)", testDmaMemToMem());
    report("drivers/timer/PWM (init)", probePwm());
    report("drivers/i2c (bus scan)", probeI2c());

    printf("Summary: %u/%u tests passed\n",
           static_cast<unsigned int>(g_passCount),
           static_cast<unsigned int>(g_totalCount));

    bool ok = g_passCount == g_totalCount;
    for (int i = 0; i < (ok ? 5 : 3); i++)
    {
        allLedsOn();
        Thread::sleep(ok ? 150 : 400);
        allLedsOff();
        Thread::sleep(ok ? 150 : 400);
    }

    for (;;)
    {
        CpuMeterData cpu = CpuMeter::getCpuStats();
        printf("alive t=%lld ms ok=%u/%u cpu=%.1f%% heap=%u stack=%u\n",
               miosix::getTime() / 1000000,
               static_cast<unsigned int>(g_passCount),
               static_cast<unsigned int>(g_totalCount), cpu.mean,
               static_cast<unsigned int>(cpu.freeHeap),
               static_cast<unsigned int>(cpu.freeStack));
        allLedsOn();
        Thread::sleep(500);
        allLedsOff();
        Thread::sleep(500);
    }
}
