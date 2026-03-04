/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "SPIBus.h"

#include <utils/ClockUtils.h>

namespace Boardcore
{
SPIBus::SPIBus(SPI_TypeDef* spi) : spi(spi)
{
    ClockUtils::enablePeripheralClock(spi);
    SPI::Reset(spi);
}

SPI_TypeDef* SPIBus::getSpi() { return spi; }

void SPIBus::configure(const SPIBusConfig& newConfig)
{
    // Do not reconfigure if already in the correct configuration.
    if (!firstConfigApplied || newConfig != config)
    {
        // Save the new configuration
        config             = newConfig;
        firstConfigApplied = true;

        // Wait until the peripheral is done before changing configuration
        SPI::WaitNotBusy(spi);

        // Disable the peripheral
        SPI::Disable(spi);

        // Configure clock polarity and phase
        SPI::SetMode(spi, config.mode);

        // Configure clock frequency
        SPI::SetClockDiver(spi, config.clockDivider);

        // Configure bit order
        SPI::SetBitOrder(spi, config.bitOrder);

        // Configure chip select and master mode
        SPI::EnableSoftwareSlaveManagement(spi);
        SPI::EnableInternalSlaveSelect(spi);
        SPI::SetMasterConfiguration(spi);

        SPI::Set8bitFrameFormat(spi);

        // Enable the peripheral
        SPI::Enable(spi);
    }
}

void SPIBus::transfer(const uint8_t* txData, uint8_t* rxData, size_t size)
{
    /*
     * On STM32F7xx and STM32F4xx series chips, on SPI3 only, the RXNE flag
     * may be erroneously set at the beginning of the transaction with the
     * RX buffer containing garbage data.
     * On F7xx chips the issue can be reproduced by re-configuring the SPI from
     * Mode 0 (CPOL=0, CPHA=0) to Mode 3 (CPOL=1, CPHA=1), after performing at
     * least one transaction in Mode 0.
     *
     * We work around this issue by flushing the RX buffer at the beginning of
     * the transaction.
     */
    SPI::FlushRx(spi);

    if (txData && rxData)
    {
        // Transfer: send data and overwrite it with received bytes
        for (size_t i = 0; i < size; i++)
            rxData[i] = transferByte(txData[i]);
    }
    else if (txData)
    {
        // Write: send data and discard received bytes
        for (size_t i = 0; i < size; i++)
            transferByte(txData[i]);
    }
    else if (rxData)
    {
        // Read: send dummy bytes to read data from the bus
        for (size_t i = 0; i < size; i++)
            rxData[i] = transferByte(0);
    }
    else
    {
        // Clock-only: send dummy bytes and discard received bytes
        for (size_t i = 0; i < size; i++)
            transferByte(0);
    }

    SPI::WaitNotBusy(spi);
}

uint8_t SPIBus::transferByte(uint8_t data)
{
    // Wait until the peripheral is ready to transmit
    SPI::WaitTxEmpty(spi);

    // Write the data item to transmit
    *reinterpret_cast<volatile uint8_t*>(&spi->DR) = static_cast<uint8_t>(data);

    // Wait until data is received
    SPI::WaitRxNotEmpty(spi);

    // Read the received data item
    return static_cast<uint8_t>(spi->DR);
}

}  // namespace Boardcore
