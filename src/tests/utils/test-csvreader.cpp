/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <utils/CSVReader/CSVReader.h>

#include <vector>

using namespace Boardcore;

struct TestData
{
    uint64_t timestamp;
    int counter;
};

std::istream& operator>>(std::istream& input, TestData& data)
{
    input >> data.timestamp;
    input.ignore(1, ',');
    input >> data.counter;

    return input;
}

int main()
{
    // Use the iterator sequentially in a for loop
    for (auto& data : CSVParser<TestData>("/sd/config.csv"))
        printf("%llu,%d\n", data.timestamp, data.counter);

    // First retrieve all the data and then display it
    auto fileData = CSVParser<TestData>("/sd/config.csv").collect();
    printf("vector size: %d\n", fileData.size());
    for (auto& data : fileData)
        printf("%llu,%d\n", data.timestamp, data.counter);
}
