/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <pthread.h>
#include <stdio.h>
#include "gamma_config.h"

class CircularBuffer
{
public:
    /*
     * Creates a buffer with fixed dimension (decided in gamma_config).
     */
    CircularBuffer(){};

    /*
     * TODO check if it's good practice.
     * Creates a buffer with given dimension (uses new!).
     *
    CircularBuffer(int totalSize){
        this.totalSize = totalSize;
        buffer = new char[totalSize];
    }

    */

    /*
     * Calculates the occupied portion of the buffer.
     */

    unsigned int size()
    {
        if (last == -1)
            return totalSize;
        else if (last < first)
            return totalSize - (first - last);
        else
            return last - first;
    }

    /*
     * Reads at maximum n chars from the buffer and stores them in *buf
     * in subsequent positions starting from offset.
     * Returns the number of characters stored (could be less than n
     * if the buffer contains less than n characters).
     */
    unsigned int read(unsigned int n, char *buf, int offset)
    {
        unsigned int nChar = size() > n ? n : size();
        int i              = 0;

        //     printf("BEFORE READING\n");
        //     print();

        pthread_mutex_lock(&outBufMutex);
        for (i = 0; i < (int)nChar; i++)
        {
            if (i == 0 && last == -1)
                last = first;

            buf[offset + i] = buffer[first];
            first++;
            if (first == totalSize)
                first = 0;
            if (first == last)
                break;
        }
        pthread_mutex_unlock(&outBufMutex);

        //     printf("AFTER READING\n");
        //     print();

        return i;
    }

    /*
     * Writes n characters in the buffer (or until it is full).
     * Returns the number of chars effectively written.
     */
    unsigned int write(unsigned int n, const char *chars)
    {
        unsigned int i = 0;

        //     printf("BEFORE WRITING %s\n", chars);
        //     print();

        pthread_mutex_lock(&outBufMutex);
        if ((int)size() < totalSize)
        {
            for (i = 0; i < n; i++)
            {
                buffer[last] = chars[i];
                last++;

                if (last == totalSize)
                    last = 0;
                if (last == first)
                {
                    last = -1;
                    break;
                }
            }
        }
        pthread_mutex_unlock(&outBufMutex);

        //     printf("AFTER WRITING\n");
        //     print();

        return i;
    }

    /*
     * TODO toString.
     * Prints the whole content of the buffer (for debug pupouses).

     void print(){
        printf("Buffer: first %d last %d occupied %u total size %d\n",
                first, last, size(), totalSize);

        int offset = first;
        for (int i = 0; i < totalSize; i++){
            if (offset + i == totalSize) offset = -i;
            printf("%c", buffer[i]);
        }
        printf("\n");
    }
    */

private:
    int totalSize = OUT_BUFFER_SIZE;
    int first     = 0;
    int last      = 0;
    char buffer[OUT_BUFFER_SIZE];

    pthread_mutex_t outBufMutex;
};

#endif /* CIRCULARBUFFER_H */
