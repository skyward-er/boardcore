/* Copyright (c) 2020-2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <ActiveObject.h>
#include <drivers/dma/DMA.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <utils/Debug.h>

#include "HIL.h"
#include "drivers/usart/USART.h"

namespace Boardcore
{

template <class FlightPhases, class SimulatorData, class ActuatorData>
class HIL;

class HILTransceiverBase : public ActiveObject
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm
     */
    explicit HILTransceiverBase(USART& hilSerial) : hilSerial(hilSerial) {}

    /**
     * @brief Returns the number of lost updates.
     * @return the number of updates lost due to communication with the
     * simulator.
     */
    int getLostUpdates() const { return nLostUpdates; }

    /**
     * @brief Returns the value in ns of the timestamp of the last received
     * simulatorData.
     */
    int64_t getTimestampSimulatorData() const { return timestampSimulatorData; }

protected:
    /**
     * @brief Waits for the control algorithm(s) to update actuatorData.
     */
    void waitActuatorData()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        while (!updated)
            condVar.wait(l);
        updated = false;
    }

    USART& hilSerial;
    bool receivedFirstPacket       = false;
    bool updated                   = false;
    int nLostUpdates               = 0;
    int64_t timestampSimulatorData = 0;  // timestamp of the last received
                                         // simulatorData [ns]
    miosix::FastMutex mutex;
    miosix::ConditionVariable condVar;
    PrintLogger logger = Logging::getLogger("HILTransceiver");
};

/**
 * @brief HILTransceiver is a Singleton and provides an easy interface for
 * the control algorithms to send and receive data during a simulation
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
class HILTransceiver : public HILTransceiverBase
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm.
     *
     * @param hilSerial Serial port for the HIL communication.
     */
    explicit HILTransceiver(USART& hilSerial,
                            HILPhasesManager<FlightPhases, SimulatorData,
                                             ActuatorData>* hilPhasesManager)
        : HILTransceiverBase(hilSerial), actuatorData(),
          hilPhasesManager(hilPhasesManager),
          dmaStreamTransmit(DMADriver::instance().acquireStreamForPeripheral(
              // NOTE: change here and also in writeDma() the required
              // peripheral
              DMADefs::Peripherals::PE_UART4_TX, std::chrono::seconds(1)))
    {
        if (!dmaStreamTransmit.isValid())
        {
            std::cout << "Error, cannot acquire dma stream for usart"
                      << std::endl;
        }
        else
            UART4->CR3 |= USART_CR3_DMAT;  // Enable DMA transmission
    }

    /**
     * @brief sets the actuator data and then wakes up the
     * MatlabTransceiver thread in order to send the data back to the
     * simulator (called by the control algorithm)
     * @param actuatorData sets the data that will be sent to the
     * simulator
     */
    void setActuatorData(ActuatorData actuatorData)
    {
        miosix::Lock<miosix::FastMutex> l(mutex);

        // If already updated increment lost updates
        if (updated)
            nLostUpdates++;

        this->actuatorData = actuatorData;
        updated            = true;
        condVar.signal();
    }

    /**
     * @brief returns the reference of the SimulatorData
     *
     * @return reference to the data simulated by matlab
     */
    const SimulatorData* getSensorData() const { return &simulatorData; }

private:
    void run() override;

    SimulatorData simulatorData;
    ActuatorData actuatorData;
    HILPhasesManager<FlightPhases, SimulatorData, ActuatorData>*
        hilPhasesManager;

    /**
     * @brief Write with dma. Returns true if the operation is successful,
     * false if the dma timeout was reached.
     */
    bool writeDma(void* buffer, uint16_t nBytes);
    DMAStreamGuard dmaStreamTransmit;
};

/**
 * @brief The thread deals with the communication between the simulator and the
 * board.
 *
 * After the first time the data is received, the loop of this thread:
 * - Reads the simulated data and copies them in the SensorData structure;
 * - Notifies every sensor that new data arrived;
 * - Waits for the control algorithms to update the actuator data;
 * - Sends back the value to the simulator.
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
void HILTransceiver<FlightPhases, SimulatorData, ActuatorData>::run()
{
    LOG_INFO(logger, "HIL Transceiver started");
    hilSerial.clearQueue();

    miosix::led2On();
    hilSerial.write(&actuatorData, sizeof(ActuatorData));
    miosix::led2Off();

    while (!shouldStop())
    {
        // Pausing the kernel in order to copy the data in the shared structure
        {
            SimulatorData tempData;
            nLostUpdates = 0;
            miosix::led3On();
            size_t nRead = 0;
            if (!hilSerial.readBlocking(&tempData, sizeof(SimulatorData),
                                        nRead))
            {
                LOG_ERR(logger, "Failed serial read");
            }

            assert(nRead == sizeof(SimulatorData) &&
                   "Read less then SimulatorData bytes");

            hilSerial.clearQueue();
            miosix::led3Off();

            miosix::PauseKernelLock kLock;
            simulatorData          = tempData;
            timestampSimulatorData = miosix::getTime();
        }

        // If this is the first packet to be received, then update the flight
        // phase manager
        if (!receivedFirstPacket)
        {
            receivedFirstPacket = true;
            hilPhasesManager->simulationStarted();
        }

        // Trigger events relative to the flight phases
        hilPhasesManager->processFlags(simulatorData);

        if (nLostUpdates > 0)
        {
            // This means also that the number of samples used for the mean sent
            // to the HIL simulator is made up of more than the number of
            // samples we though
            LOG_WARN(logger, "{} lost updates", nLostUpdates);
        }

        waitActuatorData();
        miosix::led2On();

        // hilSerial.write(&actuatorData, sizeof(ActuatorData));
        writeDma(&actuatorData, sizeof(ActuatorData));

        miosix::led2Off();
    }
}

template <class FlightPhases, class SimulatorData, class ActuatorData>
bool HILTransceiver<FlightPhases, SimulatorData, ActuatorData>::writeDma(
    void* buffer, uint16_t nBytes)
{
    // NOTE: change here and also in the constructor the required peripheral
    USARTType* usart      = UART4;  // TODO: SET PROPERLY
    const auto trnTimeout = std::chrono::milliseconds(250);  // TODO: RESIZE

    DMATransaction setup{
        .direction         = DMATransaction::Direction::MEM_TO_PER,
        .priority          = DMATransaction::Priority::VERY_HIGH,
        .srcSize           = DMATransaction::DataSize::BITS_8,
        .dstSize           = DMATransaction::DataSize::BITS_8,
        .srcAddress        = buffer,
        .dstAddress        = (void*)usart->TDR,
        .numberOfDataItems = nBytes,
        .srcIncrement      = true,
        .dstIncrement      = false,
        .enableTransferCompleteInterrupt = true,
        .enableTransferErrorInterrupt    = true,
    };

    dmaStreamTransmit->setup(setup);

    // Clear the TC flag in the USART_ISR register by setting
    // the TCCF bit in the USART_ICR register
    usart->ICR |= USART_ICR_TCCF;

    dmaStreamTransmit->enable();
    bool ret = dmaStreamTransmit->timedWaitForTransferComplete(trnTimeout);

    /*
    The TC flag can be monitored to make sure that the USART
    communication is complete. This is required to avoid corrupting the last
    transmission before disabling the USART or entering Stop mode. Software must
    wait until TC=1. The TC flag remains cleared during all data transfers and
    it is set by hardware at the end of transmission of the last frame.
     */
    while ((usart->ISR & USART_ISR_TC) == 0)
    {
    }

    return ret;
}
}  // namespace Boardcore
