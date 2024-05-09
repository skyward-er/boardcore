/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "FileBackend.h"

#include <fstream>
#include <vector>

namespace Boardcore
{

bool FileBackend::start() { return true; }

bool FileBackend::load(std::vector<uint8_t>& buf)
{
    std::ifstream is(path, std::ifstream::ate | std::ifstream::binary);
    if (!is.good())
        return false;

    size_t size = is.tellg();
    is.seekg(0);

    buf.resize(size);
    is.read(reinterpret_cast<char*>(buf.data()), size);

    return is.good();
}

bool FileBackend::save(std::vector<uint8_t>& buf)
{
    std::ofstream os(path, std::ifstream::binary);
    if (!os.good())
        return false;

    os.write(reinterpret_cast<char*>(buf.data()), buf.size());
    return os.good();
}

}  // namespace Boardcore
