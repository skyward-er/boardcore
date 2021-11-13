/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include "MBLoadCell.h"

#include "MBLoadCellData.h"
#include "SerialInterface.h"
#include "stdlib.h"

using ctrlPin1 = miosix::Gpio<GPIOC_BASE, 1>;
using ctrlPin2 = miosix::Gpio<GPIOC_BASE, 2>;

MBLoadCell::MBLoadCell(LoadCellModes mode, int serialPortNum,
                       int baudrate = 2400)
{
    this->mode = mode;

    // creating the instance of the serial interface
    serial = new SerialInterface(baudrate, serialPortNum);

    // initializing the serial connection
    if (!serial->init())
    {
        TRACE("[MBLoadCell] Wrong initialization\n");
    }

    {
        FastInterruptDisableLock dLock;
        ctrlPin1::mode(Mode::OUTPUT);
        ctrlPin2::mode(Mode::OUTPUT);
    }
}

void MBLoadCell::sample() { sampleImpl(); }

MBLoadCellDataStr MBLoadCell::sampleImpl()
{
    switch (mode)
    {
        case LoadCellModes::CONT_MOD_T:
            sampleContModT();
            break;
        case LoadCellModes::CONT_MOD_TD:
            sampleContModTd();
            break;
        case LoadCellModes::ASCII_MOD_TD:
            sampleAsciiModTd();
            break;
    }
    return last_sample;
}

void MBLoadCell::sampleContModT()
{
    TRACE("CONTINUOUS MOD T\n");
    DataModT data;
    receive(&data);

    this->last_sample.gross_weight = {atof(data.weight) / 10.0, true};
}

void MBLoadCell::sampleContModTd()
{
    TRACE("CONTINUOUS MOD TD\n");
    DataModTd data;
    receive(&data);

    this->last_sample.gross_weight = {atof(data.weightT) / 10.0, true};

    // [REVIEW] What is this really?
    this->last_sample.net_weight = {atof(data.weightP) / 10.0, true};
}

void MBLoadCell::sampleAsciiModTd()
{
    TRACE("ASCII MOD TD\n");

    char data[64];
    // sending the request
    DataAsciiRequest request;
    generateRequest(LoadCellValues::GROSS_WEIGHT, request);

    char requestStr[64];
    strcpy(requestStr, request.to_string().c_str());
    TRACE("GROSS_WEIGHT: '%s'\n", requestStr);

    // transmitting request
    transmitASCII(requestStr);

    // waiting for the response
    receiveASCII(data);

    // last_sample.gross_weight = {atof(data) / 10.0, true};
}

void MBLoadCell::generateRequest(LoadCellValues toRequest,
                                 DataAsciiRequest &req)
{
    req.req[0] = (char)toRequest;
    req.req[1] = '\0';

    req.setChecksum();
}

void MBLoadCell::transmitASCII(char *buf)
{
    ctrlPin1::high();
    ctrlPin2::high();
    serial->sendString(buf);
    Thread::sleep(10);  // needs some time (>5ms) idk why
}

void MBLoadCell::receiveASCII(char *buf)
{
    ctrlPin1::low();
    ctrlPin2::low();
    serial->recvString(buf, 64);
    TRACE("received: '%s'\n", buf);
}

template <typename T>
void MBLoadCell::receive(T *buf)
{
    ctrlPin1::low();
    ctrlPin2::low();
    serial->recvData(buf);
}

bool MBLoadCell::init() { return true; }

bool MBLoadCell::selfTest() { return true; }
