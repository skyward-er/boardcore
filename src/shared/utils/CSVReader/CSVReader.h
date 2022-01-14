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

/**
 * @brief Utility to parse a CSV file.
 *
 * Use CSVParser with the filename to iterate through all the CSV entries. You
 * need to specify a type with the >> stream operator.
 *
 * Here an example you can find in test-csvreader.cpp
 *
 * struct TestData
 * {
 *     uint64_t timestamp;
 *     int counter;
 * };
 *
 * std::istream& operator>>(std::istream& input, TestData& data)
 * {
 *     input >> data.timestamp;
 *     input.ignore(1, ',');
 *     input >> data.counter;
 *
 *     return input;
 * }
 *
 * int main()
 * {
 *     for (auto& data : CSVParser<TestData>("/sd/config.csv"))
 *         printf("%llu,%d\n", data.timestamp, data.counter); // linter off
 * }
 *
 * Reference:
 * https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
 */

#pragma once

#include <fstream>
#include <iterator>
#include <string>

namespace Boardcore
{

/**
 * @brief Implements
 *
 * @tparam Data
 */
template <typename Data>
class CSVIterator
{
public:
    CSVIterator(std::istream& inputStream)
        : inputStream(inputStream.good() ? &inputStream : nullptr)
    {
        ++(*this);
    }

    CSVIterator() : inputStream(nullptr) {}

    Data const& operator*() const { return parsedData; }

    Data const* operator->() const { return &parsedData; }

    ///< Prefix increment
    CSVIterator& operator++()
    {
        // Checks for end of file or errors in the stream
        if (inputStream && !((*inputStream) >> parsedData))
            inputStream = nullptr;

        return *this;
    }

    ///< Postfix increment
    CSVIterator operator++(int)
    {
        CSVIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    bool operator==(CSVIterator& iterator)
    {
        return (this == &iterator || (this->inputStream == nullptr &&
                                      iterator.inputStream == nullptr));
    }

    bool operator!=(CSVIterator& iterator) { return !(*this == iterator); }

private:
    std::istream* inputStream;
    Data parsedData;
};

template <typename Data>
class CSVParser
{
public:
    CSVParser(const char* fileName) : fileStream(fileName) {}

    CSVIterator<Data> begin() { return CSVIterator<Data>(fileStream); }

    CSVIterator<Data> end() { return CSVIterator<Data>(); }

private:
    std::ifstream fileStream;
};

}  // namespace Boardcore
