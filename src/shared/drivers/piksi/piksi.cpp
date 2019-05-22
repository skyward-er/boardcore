/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Federico Terraneo
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

#include "piksi.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>

using namespace std;

#ifdef _MIOSIX

#include <miosix.h>

using namespace miosix;

#endif  //_MIOSIX

/*
 * Contrary to the standard CCITT CRC that starts from 0xffff, the Piksi
 * people decided to start from 0. So we need a special CRC16 just for them
 */

static inline void crc16piksiUpdate(unsigned short &crc, unsigned char data)
{
    unsigned short x = ((crc >> 8) ^ data) & 0xff;
    x ^= x >> 4;
    crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
}

static unsigned short crc16piksi(const void *message, unsigned int length)
{
    const unsigned char *m = reinterpret_cast<const unsigned char *>(message);
    unsigned short result  = 0;
    for (unsigned int i = 0; i < length; i++)
        crc16piksiUpdate(result, m[i]);
    return result;
}

//
// class Piksi
//

Piksi::Piksi(const char *serialPath)
{
    memset(&data, 0, sizeof(data));
    memset(&partialData, 0, sizeof(partialData));

    fd = open(serialPath, O_RDWR);
    if (fd < 0)
        throw runtime_error(string("Cannot open ") + serialPath);
    if (isatty(fd))
    {
        termios t;
        tcgetattr(fd, &t);
        t.c_lflag &= ~(ISIG | ICANON | ECHO);
#ifndef _MIOSIX
        cfsetospeed(&t, B115200);
        cfsetispeed(&t, B115200);
#endif  //_MIOSIX
        tcsetattr(fd, TCSANOW, &t);
    }
    pthread_create(&thread, NULL, &threadLauncher, this);
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);
}

GPSData Piksi::getGpsData()
{
    GPSData result;
    pthread_mutex_lock(&mutex);
    result = data;
    
    if (!firstFixReceived)
    {
        pthread_mutex_unlock(&mutex);
        throw runtime_error("No fix");
    }

    pthread_mutex_unlock(&mutex);
    return result;
}

GPSData Piksi::waitForGpsData()
{
    GPSData result;
    pthread_mutex_lock(&mutex);
    pthread_cond_wait(&cond, &mutex);
    result = data;
    pthread_mutex_unlock(&mutex);
    return result;
}

Piksi::~Piksi()
{
    quit = true;
    pthread_join(thread, NULL);
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);
    close(fd);
}

void *Piksi::threadLauncher(void *arg)
{
    reinterpret_cast<Piksi *>(arg)->run();
    return nullptr;
}

void Piksi::run()
{
    do
    {
        bytes.added(readData(bytes.addEnd(), bytes.availableToAdd()));
        bytes.removed(
            lookForMessages(bytes.removeEnd(), bytes.availableToRemove()));
    } while (quit == false);
}

unsigned int Piksi::readData(unsigned char *buffer, unsigned int size)
{
    int result = read(fd, buffer, size);
    if (result > 0)
        return result;
    usleep(10000);  // We want to retry but avoid 100% CPU utilization
    return 0;       // To go to a loop of run() and notice the quit flag
}

unsigned int Piksi::lookForMessages(uint8_t *buffer, unsigned int size)
{
    //     puts("###");
    //     for(unsigned int i=0;i<size;i++) printf("%02x ",buffer[i]);
    //     puts("");

    const uint8_t preamble = 0x55;
    unsigned int consumed  = 0;
    auto consume           = [&](unsigned int n) {
        consumed += n;
        buffer += n;
        size -= n;
    };
    for (;;)
    {
        uint8_t *index = find(buffer, buffer + size, preamble);
        consume(index - buffer);  // Consume eventual characters between
                                  // messages

        if (size < sizeof(Header))
            return consumed;  // We don't have the header
        auto header              = reinterpret_cast<Header *>(buffer);
        unsigned int messageSize = sizeof(Header) + header->length + crcSize;
        if (messageSize > size)
            return consumed;  // We don't have the entire message

        uint16_t crc =
            *reinterpret_cast<uint16_t *>(buffer + messageSize - crcSize);
        if (crc16piksi(buffer + 1, messageSize - crcSize - 1) == crc)
        {
            processValidMessage(buffer, messageSize);
            consume(messageSize);
        }
        else
        {
            // TODO: fault counter?
            consume(1);  // Consume the preamble of the invalid message
        }
    }
}

void Piksi::processValidMessage(uint8_t *buffer, unsigned int size)
{
    Header *header = reinterpret_cast<Header *>(buffer);
    switch (header->type)
    {
        case MSG_POS_LLH:
            if (size < sizeof(MsgPosLlh)) /* TODO: fault counter? */
                ;
            else
                processPosLlh(reinterpret_cast<MsgPosLlh *>(buffer));
            break;
        case MSG_VEL_NED:
            if (size < sizeof(MsgVelNed)) /* TODO: fault counter? */
                ;
            else
                processVelNed(reinterpret_cast<MsgVelNed *>(buffer));
            break;
        default:
            // A valid message we're not interested in
            break;
    }
}

void Piksi::processPosLlh(Piksi::MsgPosLlh *msg)
{
    partialData.latitude      = msg->lat;
    partialData.longitude     = msg->lon;
    partialData.height        = msg->height;
    partialData.numSatellites = msg->n_sats;
#ifdef _MIOSIX
    partialData.timestamp = getTick();
#else   //_MIOSIX
    partialData.timestamp = clock() / (CLOCKS_PER_SEC / 1000);
#endif  //_MIOSIX

    if (vel && gpsTimestamp == msg->ms)
    {
        vel = pos = false;

        pthread_mutex_lock(&mutex);
        data             = partialData;
        firstFixReceived = true;
        pthread_cond_broadcast(&cond);
        pthread_mutex_unlock(&mutex);
    }
    else
    {
        pos          = true;
        gpsTimestamp = msg->ms;
    }
}

void Piksi::processVelNed(Piksi::MsgVelNed *msg)
{
    partialData.velocityNorth = static_cast<float>(msg->n) / 1000.f;
    partialData.velocityEast  = static_cast<float>(msg->e) / 1000.f;
    partialData.velocityDown  = static_cast<float>(msg->d) / 1000.f;

    partialData.speed = sqrtf(powf(partialData.velocityNorth, 2) + 
                              powf(partialData.velocityEast, 2) + 
                              powf(partialData.velocityDown, 2));

    if (pos && gpsTimestamp == msg->ms)
    {
        vel = pos = false;

        pthread_mutex_lock(&mutex);
        data             = partialData;
        firstFixReceived = true;
        pthread_cond_broadcast(&cond);
        pthread_mutex_unlock(&mutex);
    }
    else
    {
        vel          = true;
        gpsTimestamp = msg->ms;
    }
}
