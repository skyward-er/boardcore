/* CAN-Bus Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla, Alain Carlucci
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

#ifndef CANMANAGER_H
#define CANMANAGER_H

#include <Common.h>
#include "CanBus.h"
#include "CanUtils.h"

class CanBus;

static const int8_t AF_NONE = -1;

extern CanBus *global_bus_ptr[2];
extern uint32_t global_bus_ctr;

/** CanBus Init structure */
struct canbus_init_t
{
    /** CAN1, CAN2, ... */
    CAN_TypeDef *can;

    /** Pin Mode */
    const miosix::Mode::Mode_ mode;

    /** Alternate function id or AF_NONE */
    const int8_t af;

    /** Array of interrupts */
    const std::vector<IRQn_Type> interrupts;
};

class CanManager
{
    // friend class Singleton<CanManager>;
public:
    /**
     * @brief Adds a filter to receive Canbus messages.
     *
     * @param id        filter ID
     * @param can_id    on which canbus
     * @return true     ok
     * @return false    invalid filters
     */
    bool addHWFilter(uint16_t id, uint32_t can_id);
    bool delHWFilter(uint16_t id, uint32_t can_id);

    unsigned getNumFilters(unsigned can_id) const;

    /**
     * Add a new bus to the canmanager. Can add AT MOST 2 different buses
     */
    template <uint32_t gpio, uint8_t rx, uint8_t tx>
    void addBus(const canbus_init_t &i, CanDispatcher dispatcher)
    {
        typedef miosix::Gpio<gpio, rx> rport;
        typedef miosix::Gpio<gpio, tx> tport;

        rport::mode(i.mode);
        tport::mode(i.mode);

        if (i.af >= 0)
        {
#ifndef _ARCH_CORTEXM3_STM32  // Only stm32f2 and stm32f4 have it
            rport::alternateFunction(i.af);
            tport::alternateFunction(i.af);
#endif  //_ARCH_CORTEXM3_STM32
        }

        // TODO de-hardcode this part
        {
            miosix::FastInterruptDisableLock dLock;
#ifdef RCC_APB1ENR_CAN2EN
            RCC->APB1ENR |= RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN;
#else
            RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
#endif
            RCC_SYNC();
        }

        for (const auto &j : i.interrupts)
        {
            NVIC_SetPriority(j, 15);
            NVIC_EnableIRQ(j);
        }

        CanBus *canbus = new CanBus(i.can, this, bus.size(), dispatcher);
        bus.push_back(canbus);
        canbus->start();

        // Used by CanInterrupt.cpp
        global_bus_ptr[global_bus_ctr++] = canbus;
    }

    CanBus *getBus(uint32_t id);

    /** Rule of 5 */
    CanManager(const CanManager &)  = delete;
    CanManager(const CanManager &&) = delete;
    CanManager &operator=(const CanManager &) = delete;

    ~CanManager()
    {
        // Disable interrupts
        // TODO: Only disable the interrupts that we enabled
#ifdef CAN1_RX0_IRQn
        NVIC_DisableIRQ(CAN1_RX0_IRQn);
#endif
#ifdef CAN1_RX1_IRQn
        NVIC_DisableIRQ(CAN1_RX1_IRQn);
#endif
#ifdef CAN2_RX0_IRQn
        NVIC_DisableIRQ(CAN2_RX0_IRQn);
#endif
#ifdef CAN2_RX1_IRQn
        NVIC_DisableIRQ(CAN2_RX1_IRQn);
#endif

        global_bus_ptr[0] = NULL;
        global_bus_ptr[1] = NULL;

        global_bus_ctr = 0;

        // TODO Maybe unconfigure ports?
        while (bus.size() > 0)
        {
            bus[bus.size() - 1]->stop();  // Stop canbus thread

            delete bus[bus.size() - 1];
            bus.pop_back();
        }

        {
            miosix::FastInterruptDisableLock dLock;
#ifdef RCC_APB1ENR_CAN2EN
            RCC->APB1ENR &= ~(RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
#else
            RCC->APB1ENR &= ~RCC_APB1ENR_CAN1EN;
#endif
            RCC_SYNC();
        }
    }

    explicit CanManager(volatile CAN_TypeDef *Config) : Config(Config)
    {
        memset(enabled_filters, 0, sizeof(enabled_filters));
    }

    // sizeof(id) = 11 bit
    static constexpr int filter_max_id_log2 = 11;
    static constexpr int filter_max_id      = (1 << filter_max_id_log2);

    // 32 bit = 2 filters * 16 bit
    static constexpr int filterbank_size_bit = 32;
    static constexpr int filters_per_bank    = 2;
    static constexpr int filters_per_row     = 4;
    static constexpr int filter_size_bit     =  // 16
        filterbank_size_bit / filters_per_bank;

    // registers per bank: 2, FR1, FR2
    static constexpr int registers_per_bank = 2;
    // TODO check this formula --v
    static constexpr int separation_bit =  // 2
        filters_per_row / registers_per_bank - 1;

    // 16 bit - 11 bit = 5 bit
    static constexpr uint32_t filter_id_shift =
        filter_size_bit - filter_max_id_log2;
    static constexpr uint32_t filter_null = 0xffff;

    static constexpr int max_chan_filters = 14 * filters_per_row;

    // TODO 2 == number of can buses
    static constexpr int max_glob_filters = 2 * max_chan_filters;

private:
    std::map<uint16_t, uint8_t> filters[2];
    std::vector<CanBus *> bus;

    // TODO change "2" with number of available CAN buses
    uint16_t enabled_filters[2];
    volatile CAN_TypeDef *const Config;
};

//#define sCanManager CanManager::getInstance()

#endif /* CANMANAGER_H */
