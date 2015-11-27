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
#include <canbus/CanUtils.h>

using namespace miosix;

class CanSocket;

/* Classe per la gestione dell'interfaccia can su stm32.
 * La classe funziona simil-socket, alla costruzione viene inizializzata e 
 * configurata l'interfaccia. Un oggetto di tipo CanSocket si registra alla 
 * classe e tramite essa può inviare o ricevere messaggi
 *
 * TODO: la classe dovrà dividere il messaggio in più pacchetti e 
 * ricostruirlo dall'altra parte
 */

static const int FILTER_ROWS_PER_CAN = 14;
static const int FILTER_IDS_PER_ROW  = 4;
static const int FILTER_CAN1_INDEX   = 0;
static const int FILTER_CAN2_INDEX   = 14;

class CanManager {
    public:
        uint8_t CAN_ID_TYPE;
        Queue<CanMsg,6> messageQueue;

        void showMatrix();

        static CanManager* getCanManager(CAN_TypeDef* CanStruct);

        void registerSocket(CanSocket *socket, uint8_t id);
        void unregisterSocket(CanSocket *socket, uint8_t id);

        void sendMessage(uint8_t id, const unsigned char *message, int size);
        void dispatchMessage(CanMsg message);

        void canSetup();
        void configureInterrupt();
        void queueHandler();

        CanManager(const CanManager&)=delete;
        CanManager& operator=(const CanManager&)=delete;
        ~CanManager();

    private:
        volatile CAN_TypeDef* CANx;

        FastMutex mutex;
        std::multimap<int,CanSocket *> messageConsumers;

        volatile bool terminate;
        pthread_t t;

        CanManager(CAN_TypeDef* Canx);
        void addToFilterBank(uint8_t id);
        bool addToFiltersMatrix(uint16_t filter,uint8_t* row);

        static void *threadLauncher(void *arg);

        //Filters Bank Matrix
        uint16_t filterMatrix
            [FILTER_ROWS_PER_CAN+FILTER_ROWS_PER_CAN]
            [FILTER_IDS_PER_ROW];
        uint8_t availableMatrix
            [FILTER_ROWS_PER_CAN+FILTER_ROWS_PER_CAN]
            [FILTER_IDS_PER_ROW];
};

#endif /* CANMANAGER_H */
