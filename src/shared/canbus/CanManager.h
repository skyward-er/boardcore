/* CAN-Bus Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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
#include <Singleton.h>
#include "CanUtils.h"
#include "CanBus.h"

using namespace miosix;

using std::vector;
using std::initializer_list;

/* Classe per la gestione dell'interfaccia can su stm32.
 * La classe funziona simil-socket, alla costruzione viene inizializzata e 
 * configurata l'interfaccia. Un oggetto di tipo CanSocket si registra alla 
 * classe e tramite essa può inviare o ricevere messaggi
 *
 * TODO: la classe dovrà dividere il messaggio in più pacchetti e 
 * ricostruirlo dall'altra parte
 */

static const int8_t AF_NONE = -1;

/** CanBus Init structure */
struct canbus_init_t {
    /** CAN1, CAN2, ... */
    CAN_TypeDef *can;

    /** GPIOx_BASE */
    const uint32_t gpio;

    /** RX port */
    const uint8_t rx; 

    /** TX port */
    const uint8_t tx; 

    /** Pin Mode */
    const Mode mode;

    /** Alternate function id or AF_NONE */
    const int8_t af;
};

class CanManager {
    //friend class Singleton<CanManager>;
    public:

        bool addHWFilter(uint16_t id, unsigned can_id);
        void delHWFilter(uint16_t id, unsigned can_id);

        unsigned getNumFilters(unsigned can_id) const;
        
        template< uint32_t gpio, uint8_t rx>
        void addBus(const canbus_init_t& i);

        CanBus *getBus(uint32_t id);

        /** Rule of 5 */
        CanManager(const CanManager&) = delete;
        CanManager(const CanManager&&) = delete;
        CanManager& operator=(const CanManager&) = delete;

        ~CanManager() {
            // TODO Maybe unconfigure ports?
            while(bus.size() > 0) {
                delete bus[bus.size()-1];
                bus.pop_back();
            } 
        }
        // Private constructor
        CanManager(CAN_TypeDef *config_bus) : Config(config_bus) {
             memset(filters, 0, sizeof(filters));
        }
    private:
        // sizeof(id) = 11 bit 
        static constexpr int filter_max_id_log2 = 11;
        static constexpr int filter_max_id = (1 << filter_max_id_log2);

        // 32 bit = 2 filters * 16 bit
        static constexpr int filterbank_size_bit = 32;
        static constexpr int filters_per_bank = 2;
        static constexpr int filters_per_row = 4;
        static constexpr int filter_size_bit = 
            filterbank_size_bit / filters_per_bank;
        
        // registers per bank: 2, FR1, FR2
        static constexpr int registers_per_bank = 2;
        // TODO check this formula --v
        static constexpr int separation_bit =  // 2
            filters_per_row / registers_per_bank;

        // 16 bit - 11 bit = 5 bit
        static constexpr int filter_id_shift = 
            filter_size_bit / filter_max_id_log2;
        static constexpr uint32_t filter_null = 0xffff;

        static constexpr int max_chan_filters = 14 * filters_per_row;

        // TODO 2 == number of can buses
        static constexpr int max_glob_filters = 2 * max_chan_filters;

        uint16_t filters[CanManager::filter_max_id];

        vector<CanBus* > bus;

        // TODO change "2" with number of available CAN buses
        uint16_t enabled_filters[2];
        CAN_TypeDef * const Config;
};


#endif /* CANMANAGER_H */
