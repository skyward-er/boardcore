/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <utils/Numeric.h>
#include <utils/collections/CircularAllocator.h>

#include <catch2/catch.hpp>

TEST_CASE("CircularAllocator - Single put/get/pop")
{
    CircularAllocator<16, uint32_t> buffer;

    // Test empty buffer
    REQUIRE(buffer.isEmpty());
    REQUIRE(buffer.tryGet().empty());
    // Popping from an empty buffer should do nothing
    buffer.pop();
    REQUIRE(buffer.isEmpty());
    REQUIRE(buffer.tryGet().empty());

    // Insert a chunk
    float data1    = 3.14f;
    auto data1Span = std::as_bytes(std::span<float>{&data1, 1});
    REQUIRE(buffer.tryPut(data1Span));
    REQUIRE_FALSE(buffer.isEmpty());

    // tryGet should return the chunk without removing it
    auto chunk1 = buffer.tryGet();
    REQUIRE_FALSE(chunk1.empty());
    // Check data integrity
    REQUIRE(chunk1.size() == sizeof(float));
    float retrievedData1;
    std::memcpy(&retrievedData1, chunk1.data(), sizeof(float));
    REQUIRE(retrievedData1 == data1);

    // Pop and ensure empty again
    buffer.pop();
    REQUIRE(buffer.isEmpty());
}

TEST_CASE("CircularAllocator - Multiple puts and gets")
{
    CircularAllocator<32, uint32_t> buffer;
    REQUIRE(buffer.isEmpty());

    // Insert multiple chunks
    int data1      = 42;
    double data2   = 2.71828;
    auto data1Span = std::as_bytes(std::span<int>{&data1, 1});
    auto data2Span = std::as_bytes(std::span<double>{&data2, 1});
    REQUIRE(buffer.tryPut(data1Span));
    REQUIRE_FALSE(buffer.isEmpty());
    REQUIRE(buffer.tryPut(data2Span));
    REQUIRE_FALSE(buffer.isEmpty());

    // Retrieve first chunk
    auto chunk1 = buffer.tryGet();
    REQUIRE_FALSE(buffer.isEmpty());
    REQUIRE_FALSE(chunk1.empty());
    REQUIRE(chunk1.size() == sizeof(int));
    int retrievedData1;
    std::memcpy(&retrievedData1, chunk1.data(), sizeof(int));
    REQUIRE(retrievedData1 == data1);

    // Pop first chunk
    buffer.pop();
    REQUIRE_FALSE(buffer.isEmpty());

    // Retrieve second chunk
    auto chunk2 = buffer.tryGet();
    REQUIRE_FALSE(buffer.isEmpty());
    REQUIRE_FALSE(chunk2.empty());
    REQUIRE(chunk2.size() == sizeof(double));
    double retrievedData2;
    std::memcpy(&retrievedData2, chunk2.data(), sizeof(double));
    REQUIRE(retrievedData2 == data2);

    // Pop second chunk and ensure empty
    buffer.pop();
    REQUIRE(buffer.isEmpty());
}

TEST_CASE("CircularAllocator - Fill test")
{
    CircularAllocator<64, uint32_t> buffer;

    // Each element takes 8 bytes in the buffer (4 header + 4 data), 64 / 8 = 8
    std::array<uint32_t, 8> data;
    std::generate(data.begin(), data.end(), [] { return rand(); });

    // Fill the buffer
    for (auto elem : data)
    {
        auto dataSpan = std::as_bytes(std::span<const uint32_t>{&elem, 1});
        REQUIRE(buffer.tryPut(dataSpan));
    }

    // Try to insert more, should fail
    uint32_t data1 = 0;
    auto data1Span = std::as_bytes(std::span<const uint32_t>{&data1, 1});
    REQUIRE_FALSE(buffer.tryPut(data1Span));

    // Read and pop all elements
    for (auto elem : data)
    {
        auto dataSpan = buffer.tryGet();
        buffer.pop();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == elem);
    }

    // Try read more, buffer should be empty
    auto dataSpan = buffer.tryGet();
    REQUIRE(dataSpan.empty());
}

TEST_CASE("CircularAllocator - Wrap around test")
{
    CircularAllocator<32, uint16_t> buffer;

    // Each element takes 6 bytes in the buffer (2 header + 4 data), 32 / 6 = 5
    std::array<uint32_t, 5> data;
    std::generate(data.begin(), data.end(), [] { return rand(); });

    // Fill the buffer, buffer will reach 30/32 bytes used
    for (auto elem : data)
    {
        auto dataSpan = std::as_bytes(std::span<const uint32_t>{&elem, 1});
        REQUIRE(buffer.tryPut(dataSpan));
    }

    // Pop two elements to create space at the beginning
    buffer.pop();
    buffer.pop();

    // Insert new element that should wrap around to the beginning
    uint32_t newData = 12345;
    auto newDataSpan = std::as_bytes(std::span<const uint32_t>{&newData, 1});
    REQUIRE(buffer.tryPut(newDataSpan));

    // Read and pop all elements to verify order and integrity
    for (size_t i = 2; i < data.size(); ++i)
    {
        auto dataSpan = buffer.tryGet();
        buffer.pop();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == data[i]);
    }

    // Verify the wrapped around element is correct
    auto wrappedDataSpan = buffer.tryGet();
    REQUIRE_FALSE(wrappedDataSpan.empty());
    uint32_t retrievedWrappedData;
    std::memcpy(&retrievedWrappedData, wrappedDataSpan.data(),
                sizeof(uint32_t));
    REQUIRE(retrievedWrappedData == newData);
}

TEST_CASE("CircularAllocator - Stress test")
{
    CircularAllocator<256, uint32_t> buffer;

    // Fill until full, count how many elements
    uint32_t count = 0;
    uint32_t elem  = rand();
    auto elemSpan  = std::as_bytes(std::span<const uint32_t>{&elem, 1});

    while (buffer.tryPut(elemSpan))
        count++;

    // Ensure we were able to insert at least one element
    REQUIRE(count > 0);
    uint32_t halfCount = count / 2;

    while (count > halfCount)
    {
        count--;
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == elem);
        buffer.pop();
    }
    REQUIRE_FALSE(buffer.isEmpty());

    // Fill again until full
    while (buffer.tryPut(elemSpan))
        count++;

    REQUIRE(count > halfCount);

    // Pop all elements and verify integrity
    while (count)
    {
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == elem);
        buffer.pop();
        count--;
    }

    REQUIRE(buffer.isEmpty());
}

TEST_CASE("CircularAllocator - Stress test (unaligned)")
{
    CircularAllocator<256, uint16_t> buffer;

    // Fill until full, count how many elements
    uint32_t count = 0;
    uint32_t elem  = rand();
    auto elemSpan  = std::as_bytes(std::span<const uint32_t>{&elem, 1});

    while (buffer.tryPut(elemSpan))
        count++;

    // Ensure we were able to insert at least one element
    REQUIRE(count > 0);
    uint32_t halfCount = count / 2;

    while (count > halfCount)
    {
        count--;
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == elem);
        buffer.pop();
    }
    REQUIRE_FALSE(buffer.isEmpty());

    // Fill again until full
    while (buffer.tryPut(elemSpan))
        count++;

    REQUIRE(count > halfCount);

    // Pop all elements and verify integrity
    while (count)
    {
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        uint32_t retrievedData;
        std::memcpy(&retrievedData, dataSpan.data(), sizeof(uint32_t));
        REQUIRE(retrievedData == elem);
        buffer.pop();
        count--;
    }

    REQUIRE(buffer.isEmpty());
}

TEST_CASE("CircularAllocator - Stress test (variable element size)")
{
    CircularAllocator<256, uint16_t> buffer;

    // Fill until full, count how many elements
    uint32_t count = 0;
    std::vector<std::vector<uint8_t>> dataChunks;
    while (true)
    {
        size_t chunkSize = rand() % 8 + 1;  // Random chunk size between 1 and 8
        std::vector<uint8_t> chunk(chunkSize);
        std::generate(chunk.begin(), chunk.end(), [] { return rand() % 256; });
        auto chunkSpan =
            std::as_bytes(std::span<const uint8_t>{chunk.data(), chunk.size()});
        if (!buffer.tryPut(chunkSpan))
            break;
        dataChunks.push_back(std::move(chunk));
        count++;
    }

    REQUIRE(count > 0);
    uint32_t halfCount = count / 2;

    // Pop half of the elements
    for (size_t i = 0; i < halfCount; ++i)
    {
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        const auto& expectedChunk = dataChunks[i];
        REQUIRE(dataSpan.size() == expectedChunk.size());
        REQUIRE(std::equal(dataSpan.begin(), dataSpan.end(),
                           expectedChunk.begin()));
        buffer.pop();
    }
    REQUIRE_FALSE(buffer.isEmpty());

    // Fill again until full
    while (true)
    {
        size_t chunkSize = rand() % 8 + 1;
        std::vector<uint8_t> chunk(chunkSize);
        std::generate(chunk.begin(), chunk.end(), [] { return rand() % 256; });
        auto chunkSpan =
            std::as_bytes(std::span<const uint8_t>{chunk.data(), chunk.size()});
        if (!buffer.tryPut(chunkSpan))
            break;
        dataChunks.push_back(std::move(chunk));
        count++;
    }

    REQUIRE(count > halfCount);

    // Pop all elements and verify integrity
    for (size_t i = halfCount; i < dataChunks.size(); ++i)
    {
        auto dataSpan = buffer.tryGet();
        REQUIRE_FALSE(dataSpan.empty());
        const auto& expectedChunk = dataChunks[i];
        REQUIRE(dataSpan.size() == expectedChunk.size());
        REQUIRE(std::equal(dataSpan.begin(), dataSpan.end(),
                           expectedChunk.begin()));
        buffer.pop();
    }

    REQUIRE(buffer.isEmpty());
}
