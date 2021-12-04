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
#include <iostream>

#include "stdlib.h"

using ctrlPin1 = miosix::Gpio<GPIOC_BASE, 1>;  ///< control R/W pin 1
using ctrlPin2 = miosix::Gpio<GPIOC_BASE, 2>;  ///< control R/W pin 2

namespace Boardcore
{

MBLoadCell::MBLoadCell(LoadCellModes mode, int serialPortNum,
                       int baudrate = 2400)
{
    this->settings.mode = mode;
    max_print = max_setted = min_print = min_setted = 0;

    // creating the instance of the serial interface
    serial = new SerialInterface(baudrate, serialPortNum);
}

bool MBLoadCell::init()
{
    if (serial->isInit())
    {
        last_error = SensorErrors::ALREADY_INIT;
    }

    // initializing the serial connection
    if (!serial->init())
    {
        last_error = SensorErrors::INIT_FAIL;
        TRACE("[MBLoadCell] init of the serial communication failed\n");
        return false;
    }

    {
        // disabling interrupts in order to set with no problems the ctrl pins
        miosix::FastInterruptDisableLock dLock;
        ctrlPin1::mode(miosix::Mode::OUTPUT);
        ctrlPin2::mode(miosix::Mode::OUTPUT);
    }

    return true;
}

bool MBLoadCell::selfTest() { return true; }

Data MBLoadCell::sampleImpl()
{
    Data value;
    switch (settings.mode)
    {
        case LoadCellModes::CONT_MOD_T:
            value = sampleContModT();
            break;
        case LoadCellModes::CONT_MOD_TD:
            value = sampleContModTd();
            break;
        case LoadCellModes::ASCII_MOD_TD:
            value = sampleAsciiModTd();
            break;
        default:
            last_error = SensorErrors::NO_NEW_DATA;
            return last_sample;
    }

    // memorizing also the maximum gross weight registered
    if (!max_weight.valid || max_weight.weight < value.weight)
    {
        max_weight = value;
        max_setted = true;
    }
    else if (max_setted)
    {
        max_setted = false;
        max_print  = true;
    }

    // memorizing also the minimum gross weight registered
    if (!min_weight.valid || min_weight.weight > value.weight)
    {
        min_weight = value;
        min_setted = true;
    }
    else if (min_setted)
    {
        min_setted = false;
        min_print  = true;
    }

    return value;
}

Data MBLoadCell::sampleContModT()
{
    DataModT data;
    receive(&data);

    return Data(atof(data.weight) / 10.0);
}

Data MBLoadCell::sampleContModTd()
{
    DataModTd data;
    receive(&data);

    return Data(atof(data.weightT) / 10.0);
}

Data MBLoadCell::sampleAsciiModTd()
{
    DataAsciiRequest request;

    // generating the request
    generateRequest(request, LoadCellValuesEnum::GROSS_WEIGHT);

    // transmitting request
    transmitASCII(request.to_string());

    // waiting for the response
    std::string response = receiveASCII();

    // if response invalid returns last_sample, otherwise returns new sample
    if (response.find('!') != std::string::npos)
    {
        TRACE("Gross weight reception error\n");
        last_error = SensorErrors::NO_NEW_DATA;
        return last_sample;
    }
    else if (response.find('#') != std::string::npos)
    {
        TRACE("Gross weight execution error\n");
        last_error = SensorErrors::NO_NEW_DATA;
        return last_sample;
    }
    else
    {
        // taking the value returned
        return Data(stof(response.substr(3, 6)) / 10.0);
    }
}

void MBLoadCell::printData()
{
    if (last_sample.valid)
    {
        #ifdef PRINT_ALL_SAMPLES
        last_sample.print(std::cout);
        #endif
    }
    else
    {
        last_error = SensorErrors::NO_NEW_DATA;
        TRACE("No valida data has been collected\n");
    }

    if (max_print)
    {
        TRACE("NEW MAX %.2f (ts: %.3f [s])\n", max_weight.weight, max_weight.loadcell_timestamp/1000000.0);
        max_print = false;
    }

    if (min_print)
    {
        TRACE("NEW MIN %.2f (ts: %.3f [s])\n", min_weight.weight, min_weight.loadcell_timestamp/1000000.0);
        min_print = false;
    }
}

ReturnsStates MBLoadCell::asciiRequest(LoadCellValuesEnum reqType, int value)
{
    DataAsciiRequest request;

    // generating the request
    generateRequest(request, reqType, value);

    // transmitting request
    transmitASCII(request.to_string());

    // waiting for the response
    std::string response = receiveASCII();

    if (response.find('!') != std::string::npos)
        return ReturnsStates::RECEPTION_ERROR;

    if (response.find('#') != std::string::npos)
        return ReturnsStates::EXECUTION_ERROR;

    // memorizing the requested data
    switch (reqType)
    {
        case LoadCellValuesEnum::SET_SETPOINT_1:
            asciiRequest(GET_SETPOINT_1);
            break;
        case LoadCellValuesEnum::SET_SETPOINT_2:
            asciiRequest(GET_SETPOINT_2);
            break;
        case LoadCellValuesEnum::SET_SETPOINT_3:
            asciiRequest(GET_SETPOINT_3);
            break;
        case LoadCellValuesEnum::GROSS_WEIGHT:
        case LoadCellValuesEnum::NET_WEIGHT:
        case LoadCellValuesEnum::PEAK_WEIGHT:
        case LoadCellValuesEnum::GET_SETPOINT_1:
        case LoadCellValuesEnum::GET_SETPOINT_2:
        case LoadCellValuesEnum::GET_SETPOINT_3:
        {
            // taking the value returned
            float val = stof(response.substr(3, 6)) / 10.0;

            // updating the last_value struct
            settings.updateValue(reqType, val);
            break;
        }
        case LoadCellValuesEnum::RESET_TARE:
            TRACE("TARE RESETTED\n");
            break;
        case LoadCellValuesEnum::COMMUTE_TO_GROSS:
            settings.gross_mode = true;
            TRACE("COMMUTED TO GROSS WEIGHT\n");
            break;
        case LoadCellValuesEnum::COMMUTE_TO_NET:
            settings.gross_mode = false;
            TRACE("COMMUTED TO NET WEIGHT\n");
            break;
    }

    return ReturnsStates::VALID_RETURN;
}

Data MBLoadCell::getMaxWeight() { return max_weight; }

Data MBLoadCell::getMinWeight() { return min_weight; }

void MBLoadCell::resetMaxMinWeights()
{
    TRACE("max and min values erased!\n");
    max_weight.valid = false;
    min_weight.valid = false;
}

void MBLoadCell::generateRequest(DataAsciiRequest &req,
                                 const LoadCellValuesEnum toRequest, int value)
{
    strcpy(req.req, loadCellValues[toRequest].c_str());

    if (toRequest == LoadCellValuesEnum::SET_SETPOINT_1 ||
        toRequest == LoadCellValuesEnum::SET_SETPOINT_2 ||
        toRequest == LoadCellValuesEnum::SET_SETPOINT_3)
    {
        std::string strVal = std::to_string(abs(value));
        strVal.insert(strVal.begin(), 6 - strVal.length(), '0');

        if (value < 0)
            strVal[0] = '-';

        strcpy(req.value, strVal.c_str());
        TRACE("value sent: %s\n", strVal.c_str());
    }

    req.setChecksum();
}

void MBLoadCell::transmitASCII(std::string buf)
{
    // setting both the control pins to high in order to transmit
    ctrlPin1::high();
    ctrlPin2::high();
    serial->sendString(buf);
    miosix::Thread::sleep(10);  // needs some time (>5ms) idk why
}

std::string MBLoadCell::receiveASCII()
{
    char buf[64];

    // setting both the control pins to low in order to receive
    ctrlPin1::low();
    ctrlPin2::low();
    int len  = serial->recvString(buf, 64);
    buf[len] = '\0';

    return std::string(buf);
}

template <typename T>
void MBLoadCell::receive(T *buf)
{
    // setting both the control pins to low in order to receive
    ctrlPin1::low();
    ctrlPin2::low();
    serial->recvData(buf);
}

}  // namespace Boardcore
