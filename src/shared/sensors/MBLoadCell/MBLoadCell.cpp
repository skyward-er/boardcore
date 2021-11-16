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

#include <cmath>

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
    ReturnsStates ret;

    ret = asciiRequest(LoadCellValues::GET_SETPOINT_1);
    if (ret != ReturnsStates::VALID_RETURN)
    {
        TRACE("Setpoint 1 not available: %d\n", ret);
    }

    ret = asciiRequest(LoadCellValues::GROSS_WEIGHT);
    if (ret != ReturnsStates::VALID_RETURN)
    {
        TRACE("Gross weight not available: %d\n", ret);
    }

    ret = asciiRequest(LoadCellValues::NET_WEIGHT);
    if (ret != ReturnsStates::VALID_RETURN)
    {
        TRACE("Net weight not available: %d\n", ret);
    }

    ret = asciiRequest(LoadCellValues::PEAK_WEIGHT);
    if (ret != ReturnsStates::VALID_RETURN)
    {
        TRACE("Peak weight not available: %d\n", ret);
    }
}

ReturnsStates MBLoadCell::asciiRequest(LoadCellValues reqType, int value)
{
    DataAsciiRequest request;

    // generating the request
    generateRequest(request, reqType, value);

    // transmitting request
    transmitASCII(request.to_string());

    // waiting for the response
    string response = receiveASCII();

    if (response.find('!') != std::string::npos)
        return ReturnsStates::RECEPTION_ERROR;

    if (response.find('#') != std::string::npos)
        return ReturnsStates::EXECUTION_ERROR;

    // memorizing the requested data
    switch (reqType)
    {
        case LoadCellValues::SET_SETPOINT_1:
            asciiRequest(GET_SETPOINT_1);
            break;
        case LoadCellValues::SET_SETPOINT_2:
            asciiRequest(GET_SETPOINT_2);
            break;
        case LoadCellValues::SET_SETPOINT_3:
            asciiRequest(GET_SETPOINT_3);
            break;
        case LoadCellValues::GROSS_WEIGHT:
        case LoadCellValues::NET_WEIGHT:
        case LoadCellValues::PEAK_WEIGHT:
        case LoadCellValues::GET_SETPOINT_1:
        case LoadCellValues::GET_SETPOINT_2:
        case LoadCellValues::GET_SETPOINT_3:
        {
            // taking the value returned
            float value = stof(response.substr(3, 6)) / 10.0;

            // updating the last_value struct
            last_sample.updateValue(reqType, value);
            break;
        }
        case LoadCellValues::RESET_TARE:
            TRACE("TARE RESETTED\n");
            break;
    }

    return ReturnsStates::VALID_RETURN;
}

void MBLoadCell::generateRequest(DataAsciiRequest &req,
                                 LoadCellValues toRequest, int value)
{
    req.req[0] = (char)toRequest;
    req.req[1] = '\0';

    if (toRequest == LoadCellValues::SET_SETPOINT_1 ||
        toRequest == LoadCellValues::SET_SETPOINT_2 ||
        toRequest == LoadCellValues::SET_SETPOINT_3)
    {
        string strVal = fmt::format("{:d}", abs(value));
        strVal.insert(strVal.begin(), 6 - strVal.length(), '0');

        if (value < 0)
            strVal[0] = '-';

        strcpy(req.value, strVal.c_str());
        TRACE("value sent: %s\n", strVal.c_str());
    }

    req.setChecksum();
}

void MBLoadCell::transmitASCII(string buf)
{
    ctrlPin1::high();
    ctrlPin2::high();
    serial->sendString(buf);
    Thread::sleep(10);  // needs some time (>5ms) idk why
}

string MBLoadCell::receiveASCII()
{
    char buf[64];

    ctrlPin1::low();
    ctrlPin2::low();
    int len  = serial->recvString(buf, 64);
    buf[len] = '\0';

    return string(buf);
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
