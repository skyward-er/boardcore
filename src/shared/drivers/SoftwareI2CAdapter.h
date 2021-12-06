/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <util/software_i2c.h>

namespace Boardcore
{

/**
 * @brief Adapter for the SoftwareI2C class, in order to have a compatible
 * interface with the ProtocolI2C class in BusTemplate.h
 */
template <typename SDA, typename SCL, unsigned stretchTimeout = 50,
          bool fast = false>
class SoftwareI2CAdapter
{
    typedef miosix::SoftwareI2C<SDA, SCL, stretchTimeout, fast> SwI2CType;

public:
    static void init() { SwI2CType::init(); }

    /**
     * Sends the \param len bytes stored in \param *data buffer
     * to the register specified by \param regAddress
     */
    static inline void write(uint8_t address, uint8_t regAddr, uint8_t* data,
                             uint8_t len)
    {
        SwI2CType::sendStart();

        SwI2CType::send(address);  // Send 7-bit address + write bit (0)
        SwI2CType::send(regAddr);  // Send register address
        for (int i = 0; i < len; i++)
        {
            SwI2CType::send(data[i]);  // Send data
        }

        SwI2CType::sendStop();
    }

    /**
     * Sends the \param len bytes stored in \param *data buffer without
     * specifying a register
     */
    static inline void directWrite(uint8_t address, uint8_t* data, uint8_t len)
    {
        SwI2CType::sendStart();

        SwI2CType::send(address);  // Send 7-bit address + write bit (0)
        for (int i = 0; i < len; i++)
        {
            SwI2CType::send(data[i]);  // Send data
        }

        SwI2CType::sendStop();
    }

    /**
     * Reads \param len bytes storing them into \param *data buffer
     * from the register specified by \param regAddress
     */
    static inline void read(uint8_t address, uint8_t regAddr, uint8_t* data,
                            uint8_t len)
    {
        // First write the address of the register we want to read
        SwI2CType::sendStart();
        SwI2CType::send(address);
        SwI2CType::send(regAddr);

        // Now we can read
        SwI2CType::sendRepeatedStart();
        SwI2CType::send((address) | 0x01);  // Read request: last bit is 1

        for (int i = 0; i < len - 1; i++)
        {
            data[i] = SwI2CType::recvWithAck();
        }

        // Read the last byte. End with NACK to signal we want to stop reading.
        data[len - 1] = SwI2CType::recvWithNack();

        SwI2CType::sendStop();
    }

    /**
     * Reads \param len bytes storing them into \param *data buffer
     * from the register specified by \param regAddress
     */
    static inline void directRead(uint8_t address, uint8_t* data, uint8_t len)
    {
        // Now we can read
        SwI2CType::sendStart();
        SwI2CType::send((address) | 0x01);  // Read request: last bit is 1

        for (int i = 0; i < len - 1; i++)
        {
            data[i] = SwI2CType::recvWithAck();
        }

        // Read the last byte. End with NACK to signal we want to stop reading.
        data[len - 1] = SwI2CType::recvWithNack();

        SwI2CType::sendStop();
    }

private:
    SoftwareI2CAdapter();
};

}  // namespace Boardcore
