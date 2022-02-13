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

#include <radio/Xbee/APIFrameParser.h>
#include <radio/Xbee/APIFrames.h>
#include <utils/testutils/MockGpioPin.h>
#include <utils/testutils/MockSPIBus.h>

#include <deque>
#include <functional>
#include <memory>

namespace Boardcore
{

class MockXbeeSPIBus : public MockSPIBus
{
public:
    MockXbeeSPIBus(SPIBusConfig expectedConfig, MockGpioPin& attn)
        : MockSPIBus(expectedConfig), attn(attn)
    {
    }

    void registerCallback(std::function<void(MockGpioPin&)> callback)
    {
        this->callback = callback;
    }

    void removeCallback() { this->callback = nullptr; }

    void pushApiFrame(Xbee::APIFrame& api)
    {
        unique_ptr<uint8_t> bytes(new uint8_t[Xbee::MAX_API_FRAME_SIZE]);

        size_t len = api.toBytes(bytes.get());
        push(bytes.get(), len);
    }

    std::deque<Xbee::APIFrame> getParsedFrames()
    {
        Lock<FastMutex> l(mutex);
        return parsedFrames;
    }

    /**
     * @brief Wether to generate a tx_status responde upon receiving a tx
     * request
     *
     */
    void setRespondWithTxStatus(bool respond, uint8_t deliveryStatus = 0)
    {
        txStatusDeliveryStatus = deliveryStatus;
        respondWithTxStatus    = respond;
    }

protected:
    void _push(uint8_t* data, size_t len) override
    {
        MockSPIBus::_push(data, len);

        assertATTN();
    }

    uint8_t _read() override
    {
        uint8_t r = MockSPIBus::_read();

        if (inBufPosCntr == inBuf.size())
        {
            resetATTN();
        }

        return r;
    }

    void _write(uint8_t byte) override
    {
        MockSPIBus::_write(byte);

        Xbee::APIFrameParser::ParseResult res =
            parser.parse(byte, &parsingFrame);

        if (res == Xbee::APIFrameParser::ParseResult::SUCCESS)
        {
            parsedFrames.push_back(parsingFrame);

            if (parsingFrame.frameType == Xbee::FTYPE_TX_REQUEST &&
                respondWithTxStatus)
            {
                Xbee::TXRequestFrame* txReq =
                    parsingFrame.toFrameType<Xbee::TXRequestFrame>();
                Xbee::TXStatusFrame txStat;

                txStat.setFrameID(txReq->getFrameID());
                txStat.setDeliveryStatus(txStatusDeliveryStatus);
                txStat.setDiscoveryStatus(0);
                txStat.setTransmitRetryCount(0);
                txStat.calcChecksum();

                _pushApiFrame(txStat);
            }
        }
        else if (res == Xbee::APIFrameParser::ParseResult::FAIL)
        {
            printf("[MockXbeeSPI] Failed parsing frame written to SPI bus\n");
        }
    }

private:
    void _pushApiFrame(Xbee::APIFrame& api)
    {
        uint8_t* bytes = new uint8_t[Xbee::MAX_API_FRAME_SIZE];
        size_t len     = api.toBytes(bytes);
        _push(bytes, len);

        delete[] bytes;
    }

    void assertATTN()
    {
        if (attn.value() != 0)
        {
            attn.low();
            if (callback)
                callback(attn);
        }
    }

    void resetATTN()
    {
        if (attn.value() == 0)
        {
            attn.high();
            if (callback)
                callback(attn);
        }
    }

    std::deque<Xbee::APIFrame> parsedFrames;

    Xbee::APIFrameParser parser;
    Xbee::APIFrame parsingFrame;

    std::function<void(MockGpioPin&)> callback;
    MockGpioPin attn;

    bool respondWithTxStatus       = true;
    uint8_t txStatusDeliveryStatus = 0;
};

}  // namespace Boardcore
