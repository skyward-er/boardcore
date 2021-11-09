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

MBLoadCell::MBLoadCell(LoadCellModes mode, int serialPortNum,
                       int baudrate = 115200)
{
    this->mode = mode;
    serial     = new SerialInterface(baudrate, serialPortNum);

    // initializing the serial connection
    if (!serial->init())
    {
        TRACE("[MBLoadCell] Wrong initialization\n");
    }
}

float MBLoadCell::sampleImpl()
{
    DataModTd data;
    serial->recvData(&data);
    TRACE("%d, %d\n", atoi(data.weightT), atoi(data.weightP));
    return atof(data.weightT) / 10.0;
}

int MBLoadCell::readBusy()
{
    DataModTd data;
    serial->recvData(&data);
    TRACE("%d, %d\n", atoi(data.weightT), atoi(data.weightP));
    return atoi(data.weightT);
}

bool MBLoadCell::init() { return true; }

bool MBLoadCell::selfTest() { return true; }

string MBLoadCell::calculateChecksum(char *message)
{
    uint8_t ck = 0;
    for (int i = 0; i < strlen(message); i++)
    {
        ck ^= message[i];
    }

    char hex[3];
    itoa(ck, hex, 16);

    return string(hex);
}
