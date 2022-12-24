/*
 * Inter Stormtrooper Boards communication protocol through STM32's
 * USART3 interface
 *
 * Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include "IsbProtocol_serial3.h"

using namespace std;
using namespace miosix;

typedef Gpio<GPIOB_BASE, 10> u3tx;
typedef Gpio<GPIOB_BASE, 11> u3rx;
typedef Gpio<GPIOB_BASE, 14> u3rts;

void __attribute__((weak)) USART3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z11Serial3_Irqv");
    restoreContext();
}

void __attribute__((used)) Serial3_Irq()
{
    auto inst = IsbProtocol_serial3::instance();
    inst.IRQHandler();
}

IsbProtocol_serial3::IsbProtocol_serial3()
{
    {
        FastInterruptDisableLock dLock;
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        RCC_SYNC();

        u3tx::mode(Mode::ALTERNATE);
        u3tx::alternateFunction(7);

#ifdef _ARCH_CORTEXM3_STM32F1
        u3rx::mode(Mode::INPUT);
#else
        u3rx::mode(Mode::ALTERNATE);
        u3rx::alternateFunction(7);
#endif

        u3rts::mode(Mode::OUTPUT);
        u3rts::low();
    }

    /*** USART 3 configuration ***/
    // Enable parity, 9 bit frame length, no parity,
    // generate interrupt in case a new byte is received
    USART3->CR1 |= USART_CR1_M | USART_CR1_RXNEIE | USART_CR1_TE  // enable tx
                   | USART_CR1_RE;                                // enable rx

    // CR2 and CR3 registers are left untouched since their default values are
    // OK

    NVIC_SetPriority(USART3_IRQn, 15);  // Lowest priority for serial
    NVIC_ClearPendingIRQ(USART3_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
}

IsbProtocol_serial3::~IsbProtocol_serial3()
{
    FastInterruptDisableLock dLock;
    RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
    RCC_SYNC();
}

void IsbProtocol_serial3::IRQHandler()
{
    if (USART3->SR & USART_SR_RXNE)
    {
        /* This protocol uses 9 bit long usart frames.
         * The STM32's data register is 9 bit long, so the
         * lower 8 bits are used to transport the data byte and
         * the MSB - the 9th bit - is used to select between
         * data or address byte
         */
        uint16_t byte = USART->DR;

        uint16_t checkMask = 0x100 | nodeAddress;
        if (byte == checkMask)
        {
            rxStatus.rxIndex        = 0;
            rxStatus.rxInProgress   = true;
            rxStatus.dataLenPending = true;

            rxBuf[rxStatus.rxIndex] = byte & 0xFF;  // strip away the 9th byte
            rxStatus.rxIndex++;
        }
        else if (rxStatus.rxInProgress)
        {
            if (rxStatus.dataLenPending)
            {
                // Second byte in the packet is the data len field. To this
                // value we have to add 4 bytes: address, data len and two bytes
                // of CRC
                rxStatus.packetSize     = (byte & 0xFF) + 4;
                rxStatus.dataLenPending = false;
            }

            rxBuf[rxStatus.rxIndex] = byte & 0xFF;
            rxStatus.rxIndex++;

            if (rxStatus.rxIndex >= rxStatus.packetSize)
            {
                rxStatus.rxInProgress = false;
            }
        }
    }
}

size_t IsbProtocol_serial3::newDataAvailable()
{
    if ((rxStatus.rxInProgress == true) || (rxStatus.packetSize = 0))
    {
        return 0;
    }

    // we pass packetSize - 2 in order to avoid including in the CRC calculation
    // the CRC value inserted from the sender
    uint16_t crc = CRC16(rxBuf, rxStatus.packetSize - 2);
    uint16_t pktCrc =
        (rxBuf[rxStatus.packetSize - 2] << 8) | rxBuf[rxStatus.packetSize - 1];

    // Packet length check: the value contained in the second byte must be
    // equal to the number of bytes received minus 4
    bool checkLen = (rxBuf[1] == (rxStatus.packetSize - 4)) ? true : false;

    if ((crc != pktCrc) || (!checkLen))
    {
        rxStatus.packetSize = 0;  // packet is corrupt, discard it
        return 0;
    }

    // Return length of the data field
    return rxBuf[1];
}

void IsbProtocol_serial3::getData(uint8_t* buffer)
{
    std::memcpy(buffer, rxBuf[2], rxBuf[1]);
}

void IsbProtocol_serial3::sendData(uint8_t dstAddr, uint8_t* data, size_t len)
{
    // packet too long
    if (len > 0xFF)
    {
        return;
    }

    txBuf[0] = dstAddr;
    txBuf[1] = len;
    std::memcpy(txBuf[2], data, len);

    // Include in CRC also address and data len field
    uint16_t crc = CRC16(txBuf, len + 2);

    size_t crcBegin     = len + 2;
    txBuf[crcBegin]     = crc >> 8;
    txBuf[crcBegin + 1] = crc & 0xFF;

    u3rts::high();

    for (size_t i = 0; i < len + 4; i++)
    {
        // the first byte is the address, so it has to be sent with the 9th bit
        // set to 1
        if (i == 0)
        {
            USART3->DR = txBuf[i] | 0x100;
        }
        else
        {
            USART3->DR = txBuf[i];
        }

        // Wait until tx buffer is empty again
        while ((USART3->SR & USART_SR_TXE) == 0)
            ;
    }

    u3rts::low();
}

void IsbProtocol_serial3::setBaud(uint32_t baud)
{
    uint32_t busFreq = SystemCoreClock;

#ifdef _ARCH_CORTEXM3_STM32F1
    if (RCC->CFGR & RCC_CFGR_PPRE1_2)
    {
        busFreq /= 1 << (((RCC->CFGR >> 8) & 0x3) + 1);
    }
#else
    if (RCC->CFGR & RCC_CFGR_PPRE1_2)
    {
        busFreq /= 1 << (((RCC->CFGR >> 10) & 0x3) + 1);
    }
#endif

    uint32_t quot = 2 * busFreq / baud;  // 2*freq for round to nearest
    USART3->BRR   = quot / 2 + (quot & 1);
}

void IsbProtocol_serial3::setNodeAddress(uint8_t address)
{
    nodeAddress = address;
}

uint16_t IsbProtocol_serial3::CRC16(uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        uint16_t x = ((crc >> 8) ^ data[i]) & 0xff;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }

    return crc;
}

IsbProtocol_serial3& IsbProtocol_serial3::instance()
{
    static IsbProtocol_serial3 inst;
    return inst;
}
