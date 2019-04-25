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

#ifndef PIKSI_H
#define PIKSI_H

#include <pthread.h>
#include "contiguous_queue.h"
#include "piksi_data.h"

/**
 * Class to access the Piksi GPS.
 *
 * Should be connected to the Piksi UARTB configured as
 * MODE                    SBP
 * SBP message mask        65280
 * telemetry radio on boot False
 * baudrate                115200
 */
class Piksi
{
public:
    /**
     * Constructor
     * \param serialPath path to the device file of the Piksi serial port
     * \throws runtime_error if the serial port cannot be opened
     */
    explicit Piksi(const char *serialPath);

    /**
     * \return the latest GPS data, or throws if the GPS has not yet got a fix.
     * If the GPS has lost the fix, the same data is returned repeatedly,
     * use the timestamp field of the GPSData struct to know this.
     * \throws runtime_error is no data is available
     */
    GPSData getGpsData();

    /**
     * \return the latest GPS data. If the GPS has yet got a fix or has lost
     * the fix, this function will block until the fix is regained
     */
    GPSData waitForGpsData();

    /**
     * Destructor
     */
    ~Piksi();

private:
    Piksi(const Piksi &) = delete;
    Piksi &operator=(const Piksi &) = delete;

    struct __attribute__((packed)) Header
    {
        uint8_t preamble;
        uint16_t type;
        uint16_t sender;
        uint8_t length;
    };

    static const unsigned int crcSize = 2;

    static const uint16_t MSG_POS_LLH = 0x0201;

    struct __attribute__((packed)) MsgPosLlh
    {
        Header header;
        uint32_t ms;          // [ms]
        double lat;           // [deg]
        double lon;           // [deg]
        double height;        // [m]
        uint16_t h_accuracy;  // Piksi says unimplemented
        uint16_t v_accuracy;  // Piksi says unimplemented
        uint8_t n_sats;
        uint8_t flags;
    };

    static const uint16_t MSG_VEL_NED = 0x0205;

    struct __attribute__((packed)) MsgVelNed
    {
        Header header;
        uint32_t ms;          // [ms]
        int32_t n;            // [mm/s]
        int32_t e;            // [mm/s]
        int32_t d;            // [mm/s]
        uint16_t h_accuracy;  // Piksi says unimplemented
        uint16_t v_accuracy;  // Piksi says unimplemented
        uint8_t n_sats;
        uint8_t flags;
    };

    /**
     * Launches run() from the background thread
     * \param arg this
     */
    static void *threadLauncher(void *arg);

    /**
     * Piksi main processing loop
     */
    void run();

    /**
     * Fill a buffer from the serial port where the piksi is connected
     * \param buffer where to store read data
     * \param size how many bytes to read
     */
    unsigned int readData(unsigned char *buffer, unsigned int size);

    /**
     * Tries to find one or more piksi message in the buffer. There are no
     * alignment requirements, the given buffer can begin and end in the middle
     * of a packet.
     * \param buffer buffer read from the serial port
     * \param size buffer size
     * \return the number of consumed characters. the last size-consumed bytes
     * of the buffer are not processed yet, most likely because they contain an
     * incomplete message, and must be prepended to the buffer given at the
     * next call in order not to miss some packets.
     */
    unsigned int lookForMessages(uint8_t *buffer, unsigned int size);

    /**
     * Called on a message that has already passed the CRC check.
     * \param buffer pointer to the first message byte (0x55)
     * \param size message size
     */
    void processValidMessage(uint8_t *buffer, unsigned int size);

    /**
     * Processes a POS_LLH message
     * \param msg the message
     */
    void processPosLlh(MsgPosLlh *msg);

    /**
     * Processes a VEL_NED message
     * \param msg the message
     */
    void processVelNed(MsgVelNed *msg);

    // The queue should be large enough to contain the largest message (256+8)
    ContiguousQueue<uint8_t, 384> bytes;
    int fd;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    GPSData data, partialData;
    uint32_t gpsTimestamp = 0;
    bool pos              = false;
    bool vel              = false;
    bool firstFixReceived = false;
    volatile bool quit    = false;
};

#endif  // PIKSI_H
