/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#pragma once

#include <deque>
#include <functional>
#include <memory>

#include "drivers/Xbee/APIFrameParser.h"
#include "drivers/Xbee/APIFrames.h"
#include "drivers/spi/test/MockSPIBus.h"
#include "utils/testutils/MockGpioPin.h"

using std::deque;
using std::function;
using std::unique_ptr;

class MockXbeeSPIBus : public MockSPIBus
{
public:
    MockXbeeSPIBus(SPIBusConfig expected_config, MockGpioPin& attn)
        : MockSPIBus(expected_config), attn(attn)
    {
    }

    void registerCallback(function<void(MockGpioPin&)> callback)
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

    deque<Xbee::APIFrame> getParsedFrames()
    {
        Lock<FastMutex> l(mutex);
        return parsed_frames;
    }

    /**
     * @brief Wether to generate a tx_status responde upon receiving a tx
     * request
     *
     */
    void setRespondWithTxStatus(bool respond, uint8_t delivery_status = 0)
    {
        tx_status_delivery_status = delivery_status;
        respond_with_tx_status    = respond;
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

        if (in_buf_pos_cntr == in_buf.size())
        {
            resetATTN();
        }

        return r;
    }

    void _write(uint8_t byte) override
    {
        MockSPIBus::_write(byte);

        Xbee::APIFrameParser::ParseResult res =
            parser.parse(byte, &parsing_frame);

        if (res == Xbee::APIFrameParser::ParseResult::SUCCESS)
        {
            parsed_frames.push_back(parsing_frame);

            if (parsing_frame.frame_type == Xbee::FTYPE_TX_REQUEST &&
                respond_with_tx_status)
            {
                Xbee::TXRequestFrame* tx_req =
                    parsing_frame.toFrameType<Xbee::TXRequestFrame>();
                Xbee::TXStatusFrame tx_stat;

                tx_stat.setFrameID(tx_req->getFrameID());
                tx_stat.setDeliveryStatus(tx_status_delivery_status);
                tx_stat.setDiscoveryStatus(0);
                tx_stat.setTransmitRetryCount(0);
                tx_stat.calcChecksum();
                
                _pushApiFrame(tx_stat);
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

    deque<Xbee::APIFrame> parsed_frames;

    Xbee::APIFrameParser parser;
    Xbee::APIFrame parsing_frame;

    function<void(MockGpioPin&)> callback;
    MockGpioPin attn;

    bool respond_with_tx_status       = true;
    uint8_t tx_status_delivery_status = 0;
};