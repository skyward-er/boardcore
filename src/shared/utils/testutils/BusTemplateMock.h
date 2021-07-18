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

#include <Singleton.h>
#include <cstring>
#include <vector>
#include <cstdint>

using std::vector;

class BusSPIMock : public Singleton<BusSPIMock>
{
    friend class Singleton<BusSPIMock>;
public:
    inline static void write(const void* buffer, size_t len)
    {
        BusSPIMock::getInstance()->_write(buffer, len);
    }
    inline static void write(uint8_t byte)
    {
        BusSPIMock::getInstance()->_write(byte);
    }

    inline static void init() { BusSPIMock::getInstance(); }

    inline static int read(void* buffer, size_t max_len)
    {
        return BusSPIMock::getInstance()->_read(buffer, max_len);
    }

    inline static uint8_t read() { return BusSPIMock::getInstance()->_read(); }

    inline static void transfer(uint8_t* buf, size_t max_len)
    {
        BusSPIMock::getInstance()->_transfer(buf, max_len);
    }

    inline static uint8_t transfer(uint8_t* byte)
    {
        return BusSPIMock::getInstance()->_transfer(byte);
    }

    /**
     * Concatenate bytes to the MISO buffer to simulate a response from a
     * sensor. If no bytes are present in the MISO buffer, 0x00 is assumed.
     */
    void addMISO(const vector<uint8_t> miso)
    {
        MISO.insert(MISO.end(), miso.begin(), miso.end());
    }

    const vector<uint8_t>& getMOSI() { return MOSI; }

    void restoreState()
    {
        MISO.clear();
        MOSI.clear();
        miso_index = 0;
    }

private:
    BusSPIMock() {}

    inline void _write(const void* buffer, size_t len) const
    {
        const uint8_t* buf_ptr = (const uint8_t*)buffer;
        MOSI.insert(MOSI.end(), buf_ptr, buf_ptr + len);
    }

    inline void _write(uint8_t byte) const { MOSI.push_back(byte); }

    inline int _read(void* buffer, size_t max_len) const
    {
        uint8_t* buf_ptr = ( uint8_t*)buffer;
        for (size_t i = 0; i < max_len; i++)
        {
            if (miso_index < MISO.size())
            {
                buf_ptr[i] = MISO.at(miso_index++);
            }
            else
            {
                buf_ptr[i] = 0;
            }
        }
        return 0;
    }

    inline uint8_t _read() const
    {
        if (miso_index < MISO.size())
        {
            return MISO.at(miso_index++);
        }
        else
        {
            return 0;
        }
    }

    inline void _transfer(uint8_t* buf, size_t max_len)
    {
        MOSI.reserve(MOSI.size() + max_len);
        MOSI.insert(MOSI.end(), buf, buf + max_len);

        for (size_t i = 0; i < max_len; i++)
        {
            if (miso_index < MISO.size())
            {
                buf[i] = MISO.at(miso_index++);
            }
            else
            {
                buf[i] = 0;
            }
        }
    }

    inline uint8_t _transfer(uint8_t* byte)
    {
        MOSI.push_back(*byte);
        if (miso_index < MISO.size())
        {
            *byte = MISO.at(miso_index++);
        }
        else
        {
            *byte = 0;
        }
        return *byte;
    }

    mutable size_t miso_index = 0;
    mutable vector<uint8_t> MISO;

    mutable vector<uint8_t> MOSI;
};