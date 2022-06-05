/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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
#include <miosix.h>
#include <radio/Transceiver.h>
#include <utils/collections/CircularBuffer.h>

#include <functional>

#include "APIFrameParser.h"
#include "XbeeStatus.h"

using miosix::FastMutex;

#ifndef USE_MOCK_PERIPHERALS
using GpioType = miosix::GpioPin;
#else
#include <utils/TestUtils/MockGpioPin.h>
using GpioType = MockGpioPin;
#endif

namespace Boardcore
{

namespace Xbee
{
using ParseResult = APIFrameParser::ParseResult;

static constexpr unsigned int FRAME_POLL_INTERVAL = 10;    // ms
static constexpr unsigned int DEFAULT_TX_TIMEOUT  = 5000;  // ms

// Size (in frames) of the receive circular buffer
static constexpr unsigned int RX_FRAMES_BUF_SIZE = 3;

class Xbee : public Transceiver
{
public:
    using OnFrameReceivedListener = std::function<void(APIFrame& frame)>;

    /**
     * @brief Constructs a new instance of the Xbee driver.
     *
     * @param bus The SPI bus where the xbee is connected
     * @param cs Xbee SPI Chip Select Pin
     * @param attn Xbee ATTN Pin
     * @param rst Xbee RST PIN
     * @param txTimeout How long to wait to for a TX Status
     */
    Xbee(SPIBusInterface& bus, GpioType cs, GpioType attn, GpioType rst,
         long long txTimeout = DEFAULT_TX_TIMEOUT);

    /**
     * @brief Constructs a new instance of the Xbee driver.
     *
     * @param bus The SPI bus where the xbee is connected
     * @param config Custom SPI bus configuration
     * @param cs Xbee SPI Chip Select Pin
     * @param attn Xbee ATTN Pin
     * @param rst Xbee RST PIN
     * @param txTimeout How long to wait to for a TX Status
     *
     */
    Xbee(SPIBusInterface& bus, SPIBusConfig config, GpioType cs, GpioType attn,
         GpioType rst, long long txTimeout = DEFAULT_TX_TIMEOUT);

    ~Xbee();

    /**
     * @brief Sends a packet.
     * The function blocks until the packet is sent to the peripheral, but does
     * not wait for an ACK or send confirmation from the Xbee. Thus, it always
     * returns true.
     *
     * @param pkt Pointer to the packet (needs to be at least packetLength
     * bytes).
     * @param packetLength Length of the packet to be sent.
     * @return Always true
     */
    bool send(uint8_t* pkt, size_t packetLength) override;

    /**
     * @brief Waits until a new packet is received.
     *
     * @param buf Buffer to store the received packet into.
     * @param bufferMaxSize Maximum length of the received data.
     * @return Size of the data received or -1 if failure
     */
    ssize_t receive(uint8_t* buf, size_t bufferMaxSize) override;

    /**
     * @brief Hardware resets the Xbee.
     */
    void reset();

    /**
     * @brief Signals the receive() function that there is new data available.
     * Call this from the ATTN pin interrupt service routine, and nowhere else
     * please.
     *
     */
    void handleATTNInterrupt();

    /**
     * @brief Wakes the receive function without needing an interrupt
     *
     * @param forceReturn Wether receive(..) should return even if it has not
     * received any data
     */
    void wakeReceiver(bool forceReturn = false);

    /**
     * @brief Sends an AT Command to the Xbee (see datasheet) without waiting
     * for a response
     *
     * @param cmd Two character string with the name of the command (eg "DB")
     * @param params Optional command parameters
     * @param paramsLength Length in bytes of the params array
     */
    void sendATCommand(const char* cmd, uint8_t* params = nullptr,
                       size_t paramsLength = 0);

    /**
     * @brief Sends an AT Command to the Xbee and wait for a response (see
     * datasheet)
     *
     * @param cmd Two character string with the name of the command (eg "DB")
     * @param params Optional command parameters
     * @param paramsLength Length in bytes of the params array
     * @param response Where to store the response
     * @param timeout Maximum time to wait for the response
     * @return true if response received before the timeout, false otherwise
     */
    bool sendATCommand(const char* cmd, ATCommandResponseFrame* response,
                       uint8_t* params = nullptr, size_t paramsLength = 0,
                       unsigned int timeout = 1000);

    /**
     * @brief Set the frame received listener, called each time a new APIFrame
     * is received from the device
     *
     * @param listener The listener
     */
    void setOnFrameReceivedListener(OnFrameReceivedListener listener);

    XbeeStatus getStatus();

private:
    /**
     * @brief Handles an API Frame
     * @attention  mutexXbeeCommunication must be locked before calling this
     * function.
     */
    void handleFrame(APIFrame& frame);

    /**
     * @brief Sends an AT command to the devices
     * @attention  mutexXbeeCommunication must be locked before calling this
     * function.
     */
    uint8_t sendATCommandInternal(const char* cmd, uint8_t* params = nullptr,
                                  size_t paramsLength = 0);

    /**
     * @brief Sends an AT command to the devices
     * @attention  mutexXbeeCommunication must be locked before calling this
     * function.
     */
    uint8_t sendATCommandInternal(uint8_t frameId, const char* cmd,
                                  uint8_t* params     = nullptr,
                                  size_t paramsLength = 0);

    uint8_t buildTXRequestFrame(TXRequestFrame& txReq, uint8_t* pkt,
                                size_t packetLength);

    /**
     * @brief Fills the provided buffer with data contained in rxFramesBuffer
     */
    size_t fillReceiveBuf(uint8_t* buf, size_t bufferMaxSize);

    /**
     * @brief Reads a single frame from the SPI bus.
     * @attention mutexXbeeCommunication must be locked before calling this
     * function.
     */
    ParseResult readOneFrame();

    /**
     * @brief Reads the SPI bus until an RXPacketFrame is encountered or no more
     * data is available
     * @attention  mutexXbeeCommunication must be locked before calling this
     * function.
     *
     * @return true RXPacketFrame received
     * @return false No more data available on SPI and RXPacketFrame not
     * received
     */
    bool readRXFrame();

    /**
     * @brief Writes a frame from the SPI bus.
     * @attention  mutexXbeeCommunication must be locked before calling this
     * function.
     */
    void writeFrame(APIFrame& frame);

    uint8_t getNewFrameID();

    /**
     * @brief Waits until an APIFrame with frame type \p frameType is received
     * on the SPI or until the timeouts expires. The frame is then stored in the
     * parsingApiFrame local class variable. Any frame received by this
     * function is automatically passed to the handleFrame() function
     * @attention mutexXbeeCommunication must be locked before calling this
     * function.
     *
     * @param pollInterval Polling interval for the attn pin in milliseconds
     * @param timeoutTick Tick after which the function should stop waiting for
     * frames
     * @return true If a frame with type equal to \p frameType is received
     * @return false If the timeoutTick reached
     */
    bool waitForFrame(uint8_t frameType, unsigned int pollInterval,
                      long long timeoutTick);

    // Synchronizes all communications to the xbee, as they may happen on
    // multiple threads
    FastMutex mutexXbeeCommunication;

    APIFrameParser parser;
    // Frame being parsed. Only valid if last call to
    // parser.parse() returned ParseResult::SUCCESS
    APIFrame parsingApiFrame;

    // Temporary storage for RX packet frames, waiting to be returned by
    // receive()
    CircularBuffer<RXPacketFrame, RX_FRAMES_BUF_SIZE> rxFramesBuffer;
    FastMutex mutexRxFrames;
    // RX Packet currently being returned by receive()
    RXPacketFrame currRxFrame;
    // Index of the first byte of the payload of currRxFrame that will be
    // returned on the next call to receive()
    int currRxPayloadPointer = -1;

    // SPI defs
    SPISlave spiXbee;
    GpioType attn;
    GpioType rst;

    // How long to wait for a TX Status
    long long txTimeout;

    OnFrameReceivedListener frameListener;

    // Used to generate a unique frame id
    uint8_t frameIdCounter = 1;

    // Forces receive() to return even if there is no new data
    bool forceRcvReturn = false;

    // Status structs
    XbeeStatus status;
    Stats timeToSendStats;

    // Waiting thread to be woken when something has been received.
    miosix::Thread* receiveThread = 0;

    PrintLogger logger = Logging::getLogger("xbee");
};

}  // namespace Xbee

}  // namespace Boardcore
