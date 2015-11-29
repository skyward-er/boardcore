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
#include "CanUtils.h"

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
    GPIO_TypeDef *gpio;

    /** RX port */
    uint8_t rx; 

    /** TX port */
    uint8_t tx; 

    /** Pin Mode */
    Mode mode;

    /** Alternate function id or AF_NONE */
    int8_t af;
};

class CanManager : public Singleton<CanManager>{
    friend class Singleton<CanManager>;
    public:

        bool addHWFilter(uint8_t id, unsigned can_id);
        void delHWFilter(uint8_t id, unsigned can_id);

        unsigned getNumFilters(unsigned can_id) const;
        
        void addBus(initializer_list<canbus_init_t> init_list);

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
    private:
        CanManager() {
             memset(filters, 0, sizeof(filters));
        }

        vector<CanBus> bus;
        uint8_t filters[256];
        uint8_t enabled_filters;

        const int max_filters = 14;
};


#endif /* CANMANAGER_H */
