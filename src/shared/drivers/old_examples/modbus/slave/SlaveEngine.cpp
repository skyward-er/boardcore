/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include "SlaveEngine.h"

using namespace std;

SlaveEngine::SlaveEngine(HooksInterface* hook)
{
    if (hook == nullptr)
    {
        handlers = new HooksInterface();
    }
    else
    {
        handlers = hook;
    }
}

SlaveEngine::~SlaveEngine() { delete handlers; }

unique_ptr<PDU> SlaveEngine::ProcessRequest(unique_ptr<PDU> request)
{
    PDU* response = nullptr;
    uint8_t* data = (request->data()).second;

    switch (request->funcCode())
    {
        // read multiple coils
        case 0x01:
            response = DoReadCoils(data);
            break;

        // write single coil
        case 0x05:
            response = DoWriteCoil(data);
            break;

        // write multiple coils
        case 0x0F:
            response = DoWriteMultipleCoils(data);
            break;

        // read holding registers
        case 0x03:
            response = DoReadRegisters(data);
            break;

        // write single register
        case 0x06:
            response = DoWriteRegister(data);
            break;

        // write multiple registers
        case 0x10:
            response = DoWriteMultipleRegisters(data);
            break;

        default:
            // If we arrived here means that the function is not supported, so
            // reply with an exception packet
            uint8_t errc       = modbus::EXC_ILLEGAL_FUN;
            uint8_t exceptCode = request->funcCode() + 0x80;
            response           = new PDU(exceptCode, &errc, 1);
    }

    return unique_ptr<PDU>(response);
}

PDU* SlaveEngine::DoReadCoils(uint8_t* data)
{
    /* In this case the packet is made of two fields, each 16 bit long.
     * So, its safe to cast data to uint16_t* and access it as a two element
     * array of uint16_t
     */
    uint16_t* ptr      = reinterpret_cast<uint16_t*>(data);
    uint16_t startAddr = toLittleEndian16(ptr[0]) - 1;
    uint16_t coilsNum  = toLittleEndian16(ptr[1]);

    if (coilsNum == 0 || coilsNum > 0x07D0)
    {
        uint8_t excCode = modbus::EXC_ILLEGAL_VAL;
        return new PDU(0x81, &excCode, 1);
    }

    bool result[coilsNum] = {false};
    uint8_t retVal        = 0;
    for (uint16_t i = 0; i < coilsNum; i++)
    {
        retVal = handlers->ReadCoil(result[i], startAddr + i);
        if (retVal != 0)
        {
            break;
        }
    }

    // Bad things happened inside ReadCoil, report problem using and exception
    // packet
    if (retVal != 0)
    {
        return new PDU(0x81, &retVal, 1);
    }

    // Success, prepare a suitable response. For info about packing process
    // see the specification
    uint8_t byteCnt            = coilsNum / 8 + 1;
    uint8_t reply[byteCnt + 1] = {0};
    reply[0]                   = byteCnt;

    for (uint16_t i = 0; i < coilsNum; i++)
    {
        uint8_t chunk = i / 8;
        uint8_t shift = i % 8;
        reply[chunk + 1] |= (result[i] ? 0x01 : 0x00) << shift;
    }

    return new PDU(0x01, &reply, byteCnt + 1);
}

PDU* SlaveEngine::DoWriteCoil(uint8_t* data)
{
    /* In this case the packet is made of two fields, each 16 bit long.
     * So, its safe to cast data to uint16_t* and access it as a two element
     * array of uint16_t
     */
    uint16_t* ptr    = reinterpret_cast<uint16_t*>(data);
    uint16_t address = toLittleEndian16(ptr[0]) - 1;
    uint16_t value   = toLittleEndian16(ptr[1]);

    // value can be either 0x0000 or 0x00FF, other values are illegal.
    if ((value != 0x0000) && (value != 0x00FF))
    {
        uint8_t except = modbus::EXC_ILLEGAL_VAL;
        return new PDU(0x85, &except, 1);
    }

    // doing this is safe, because here we can have only 0x0000 or 0x00FF
    bool val       = (value == 0x00FF) ? true : false;
    uint8_t retVal = handlers->WriteCoil(val, address);

    if (retVal != 0)
    {
        return new PDU(0x85, &retVal, 1);
    }

    // Success, return response
    return new PDU(0x05, data, 4);
}

PDU* SlaveEngine::DoWriteMultipleCoils(uint8_t* data)
{
    /* Packet structure:
     * -> 2 bytes for starting address (data[0] and data[1])
     * -> 2 bytes for outputs quantity (data[2] and data[3])
     * -> 1 byte for byte count        (data [4])
     * -> N bytes for outputs' values
     */
    uint16_t* ptr      = reinterpret_cast<uint16_t*>(data);
    uint16_t startAddr = toLittleEndian16(ptr[0]) - 1;
    uint16_t outpCount = toLittleEndian16(ptr[1]);
    uint8_t byteCount  = ptr[4];

    if (outpCount == 0x00 || outpCount > 0x07B0)
    {
        uint8_t excCode = modbus::EXC_ILLEGAL_VAL;
        return new PDU(0x8F, &excCode, 1);
    }

    uint8_t retVal = 0;
    for (uint16_t i = 0; i < outpCount; i++)
    {
        uint8_t chunk = i / 8;
        uint8_t mask  = i % 8;

        // This is really bad: overflow
        if (chunk > byteCount)
        {
            retVal = modbus::EXC_ILLEGAL_VAL;
            break;
        }

        uint8_t elem = data[5 + chunk] & mask;
        bool val     = (elem != 0) ? true : false;
        retVal       = handlers->WriteCoil(val, startAddr + i);

        if (retVal != 0)
        {
            break;
        }
    }

    if (retVal != 0)
    {
        return new PDU(0x8F, &retVal, 1);
    }

    // success, return start address and output count
    return new PDU(0x0F, data, 4);
}

PDU* SlaveEngine::DoReadRegisters(uint8_t* data)
{
    /* In this case the packet is made of two fields, each 16 bit long.
     * So, its safe to cast data to uint16_t* and access it as a two element
     * array of uint16_t
     */
    uint16_t* ptr      = reinterpret_cast<uint16_t*>(data);
    uint16_t startAddr = toLittleEndian16(ptr[0]) - 1;
    uint16_t regsNum   = toLittleEndian16(ptr[1]);

    if (regsNum == 0 || regsNum > 0x007D)
    {
        uint8_t excCode = modbus::EXC_ILLEGAL_VAL;
        return new PDU(0x83, &excCode, 1);
    }

    uint16_t result[regsNum] = {0};
    uint8_t retVal           = 0;

    for (uint16_t i = 0; i < regsNum; i++)
    {
        uint16_t temp = 0;
        retVal        = handlers->ReadRegister(temp, startAddr + i);
        result[i]     = toBigEndian16(temp);

        if (retVal != 0)
        {
            break;
        }
    }

    if (retVal != 0)
    {
        return new PDU(0x83, &retVal, 1);
    }

    return new PDU(0x03, reinterpret_cast<uint8_t*>(&result), 2 * regsNum);
}

PDU* SlaveEngine::DoWriteRegister(uint8_t* data)
{
    /* In this case the packet is made of two fields, each 16 bit long.
     * So, its safe to cast data to uint16_t* and access it as a two element
     * array of uint16_t
     */
    uint16_t* ptr    = reinterpret_cast<uint16_t*>(data);
    uint16_t address = toLittleEndian16(ptr[0]) - 1;
    uint16_t value   = toLittleEndian16(ptr[1]);

    uint8_t retVal = handlers->WriteRegister(value, address);

    if (retVal != 0)
    {
        return new PDU(0x86, &retVal, 1);
    }

    return new PDU(0x06, data, 4);
}

PDU* SlaveEngine::DoWriteMultipleRegisters(uint8_t* data)
{
    /* Packet structure:
     * -> 2 bytes for starting address   (data[0] and data[1])
     * -> 2 bytes for registers quantity (data[2] and data[2])
     * -> 1 byte for byte count          (data [4])
     * -> 2*N bytes for registers' values
     */
    uint16_t* ptr      = reinterpret_cast<uint16_t*>(data);
    uint16_t startAddr = toLittleEndian16(ptr[0]) - 1;
    uint16_t regsNum   = toLittleEndian16(ptr[1]);
    uint8_t byteCount  = ptr[4];

    bool outRange = (regsNum == 0x00 || regsNum > 0x07B);
    bool mismatch = ((byteCount / 2) != regsNum);

    if (outRange || mismatch)
    {
        uint8_t excCode = modbus::EXC_ILLEGAL_VAL;
        return new PDU(0x90, &excCode, 1);
    }

    // Data values begin from the 5th byte and are all 16 bit values, so this
    // conversion is safe
    uint16_t* values = reinterpret_cast<uint16_t*>(data + 5);
    uint8_t retVal   = 0;

    for (uint16_t i = 0; i < byteCount; i++)
    {
        retVal = handlers->WriteRegister(values[i], startAddr + i);
        if (retVal != 0)
        {
            break;
        }
    }

    if (retVal != 0)
    {
        return new PDU(0x90, &retVal, 1);
    }

    return new PDU(0x10, data, 4);
}
