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

using ctrlPin1 = miosix::Gpio<GPIOC_BASE, 1>;  ///< Control R/W pin 1
using ctrlPin2 = miosix::Gpio<GPIOC_BASE, 2>;  ///< Control R/W pin 2

namespace Boardcore
{

MBLoadCell::MBLoadCell(LoadCellModes mode, int serialPortNum,
                       int baudrate = 2400)
{
    this->settings.mode = mode;
    maxPrint = maxSetted = minPrint = minSetted = 0;

    // Creating the instance of the serial interface
    serial = new SerialInterface(baudrate, serialPortNum);
}

bool MBLoadCell::init()
{
    if (serial->isInit())
    {
        lastError = SensorErrors::ALREADY_INIT;
    }

    // Initializing the serial connection
    if (!serial->init())
    {
        lastError = SensorErrors::INIT_FAIL;
        TRACE("[MBLoadCell] init of the serial communication failed\n");
        return false;
    }

    {
        // Disabling interrupts in order to set with no problems the ctrl pins
        miosix::FastInterruptDisableLock dLock;
        ctrlPin1::mode(miosix::Mode::OUTPUT);
        ctrlPin2::mode(miosix::Mode::OUTPUT);
    }

    return true;
}

ReturnsStates MBLoadCell::asciiRequest(LoadCellValuesEnum reqType, int value)
{
    DataAsciiRequest request;

    // Generating the request
    generateRequest(request, reqType, value);

    // Transmitting request
    transmitASCII(request.to_string());

    // Waiting for the response
    std::string response = receiveASCII();

    if (response.find('!') != std::string::npos)
        return ReturnsStates::RECEPTION_ERROR;

    if (response.find('#') != std::string::npos)
        return ReturnsStates::EXECUTION_ERROR;

    // Memorizing the requested data
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
            // Taking the value returned
            float val = stof(response.substr(3, 6)) / 10.0;

            // Updating the last value struct
            settings.updateValue(reqType, val);
            break;
        }
        case LoadCellValuesEnum::RESET_TARE:
            TRACE("TARE RESETTED\n");
            break;
        case LoadCellValuesEnum::COMMUTE_TO_GROSS:
            settings.grossMode = true;
            TRACE("COMMUTED TO GROSS WEIGHT\n");
            break;
        case LoadCellValuesEnum::COMMUTE_TO_NET:
            settings.grossMode = false;
            TRACE("COMMUTED TO NET WEIGHT\n");
            break;
    }

    return ReturnsStates::VALID_RETURN;
}

void MBLoadCell::resetMaxMinWeights()
{
    TRACE("max and min values erased!\n");
    maxWeight.valid = false;
    minWeight.valid = false;
}

void MBLoadCell::printData()
{
    if (lastSample.valid)
    {
#ifdef PRINT_ALL_SAMPLES
        lastSample.print(std::cout);
#endif
    }
    else
    {
        lastError = SensorErrors::NO_NEW_DATA;
        TRACE("No valida data has been collected\n");
    }

    if (maxPrint)
    {
        TRACE("NEW MAX %.2f (ts: %.3f [s])\n", maxWeight.load,
              maxWeight.loadTimestamp / 1000000.0);
        maxPrint = false;
    }

    if (minPrint)
    {
        TRACE("NEW MIN %.2f (ts: %.3f [s])\n", minWeight.load,
              minWeight.loadTimestamp / 1000000.0);
        minPrint = false;
    }
}

MBLoadCellData MBLoadCell::getMaxWeight() { return maxWeight; }

MBLoadCellData MBLoadCell::getMinWeight() { return minWeight; }

bool MBLoadCell::selfTest() { return true; }

MBLoadCellData MBLoadCell::sampleImpl()
{
    MBLoadCellData value;
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
            lastError = SensorErrors::NO_NEW_DATA;
            return lastSample;
    }

    // Memorizing also the maximum gross weight registered
    if (!maxWeight.valid || maxWeight.load < value.load)
    {
        maxWeight = value;
        maxSetted = true;
    }
    else if (maxSetted)
    {
        maxSetted = false;
        maxPrint  = true;
    }

    // Memorizing also the minimum gross weight registered
    if (!minWeight.valid || minWeight.load > value.load)
    {
        minWeight = value;
        minSetted = true;
    }
    else if (minSetted)
    {
        minSetted = false;
        minPrint  = true;
    }

    return value;
}

MBLoadCellData MBLoadCell::sampleContModT()
{
    DataModT data;
    receive(&data);

    return MBLoadCellData(atof(data.weight) / 10.0);
}

MBLoadCellData MBLoadCell::sampleContModTd()
{
    DataModTd data;
    receive(&data);

    return MBLoadCellData(atof(data.weightT) / 10.0);
}

MBLoadCellData MBLoadCell::sampleAsciiModTd()
{
    DataAsciiRequest request;

    // Generating the request
    generateRequest(request, LoadCellValuesEnum::GROSS_WEIGHT);

    // Transmitting request
    transmitASCII(request.to_string());

    // Waiting for the response
    std::string response = receiveASCII();

    // If response invalid returns lastSample, otherwise returns new sample
    if (response.find('!') != std::string::npos)
    {
        TRACE("Gross weight reception error\n");
        lastError = SensorErrors::NO_NEW_DATA;
        return lastSample;
    }
    else if (response.find('#') != std::string::npos)
    {
        TRACE("Gross weight execution error\n");
        lastError = SensorErrors::NO_NEW_DATA;
        return lastSample;
    }
    else
    {
        // Taking the value returned
        return MBLoadCellData(stof(response.substr(3, 6)) / 10.0);
    }
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
    // Setting both the control pins to high in order to transmit
    ctrlPin1::high();
    ctrlPin2::high();
    serial->sendString(buf);
    miosix::Thread::sleep(10);  // Needs some time (>5ms) idk why
}

std::string MBLoadCell::receiveASCII()
{
    char buf[64];

    // Setting both the control pins to low in order to receive
    ctrlPin1::low();
    ctrlPin2::low();
    int len  = serial->recvString(buf, 64);
    buf[len] = '\0';

    return std::string(buf);
}

template <typename T>
void MBLoadCell::receive(T *buf)
{
    // Setting both the control pins to low in order to receive
    ctrlPin1::low();
    ctrlPin2::low();
    serial->recvData(buf);
}

}  // namespace Boardcore
