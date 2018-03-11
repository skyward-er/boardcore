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

#include "Gamma868.h"
#include "miosix.h"

/*
 * A serial port attached to the Gamma868 RX and TX pins is expected
 * to be passed to the object in order to communicate with the device.
 *
 * NOTE: The serial port has to be already open and at the baudrate
 * at which the module has been configured (default is 9600 baud).
 *
 * NOTE: The object also uses 2 other pins, gammaSwitch and gammaLed, which are
 * defined in the gamma_config.h file.
 */
Gamma868::Gamma868(const char *serialPath)
{
    fd = open(serialPath, O_RDWR);
    if (fd < 0)
        printf("Cannot open %s\n", serialPath);
    gammaLed::mode(Mode::INPUT);
    gammaSwitch::mode(Mode::OUTPUT);
    gammaSwitch::high();
}

void Gamma868::start()
{
    writerThread = Thread::create(&Gamma868::static_writerThreadTask,
                                  STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY,
                                  reinterpret_cast<void *>(this));
}

/*
 * Adds data to the output buffer (non blocking).
 * Returns how many chars could be effectively stored in the buffer.
 */
unsigned int Gamma868::send(unsigned int msg_len, const char *msg)
{
    return outBuffer.write(msg_len, msg);
}

/*
 * Immediately sends command (blocking).
 */
bool Gamma868::sendCmd(int cmd_len, const char *cmd)
{

    char pkt[HEAD_LEN + cmd_len + END_LEN];

    // Prepare packet
    pkt[0]                  = START;
    pkt[1]                  = CMD;
    pkt[2]                  = cmd[0];
    pkt[HEAD_LEN + CMD_LEN] = END;

    // Send to gamma
    pthread_mutex_lock(&writingMutex);
    write(fd, pkt, HEAD_LEN + cmd_len + END_LEN);
    pthread_mutex_unlock(&writingMutex);

    return true;
}

/*
 * Reads from the gamma868 serial. (blocking)
 */
bool Gamma868::receive(int bufLen, char *buf)
{
    char init = (char)0;
    char type = (char)0;
    char end  = (char)0;
    bool cmd  = false;

    // Read until you find the start byte.
    while (init != START)
    {
        read(fd, &init, 1);
    }

    pthread_mutex_lock(&readingMutex);  // TODO is sync needed?

    // Read second byte (type of data)
    read(fd, &type, 1);
    if (type == DATA)
    {
        read(fd, buf, bufLen);  // If it's data, read all the bufLen chars
    }
    else if (type == CMD)
    {
        read(fd, buf, CMD_LEN);  // If it's a command, just read 1 char
        cmd = true;
    }

    // End byte
    read(fd, &end, 1);

    pthread_mutex_unlock(&readingMutex);

    if (end != END)
        printf("Did not find end char! stream may be inconsistent\n");

    return cmd;
}

/*
 * Continuously checks the buffer and, if there's something in it, sends
 * it in packets of fixed dimension, waiting each time until the end of
 * transmission.
 */
void Gamma868::writerThreadTask()
{
    while (1)
    {
        if (outBuffer.size() >= MIN_TO_SEND)
        {
            // Prepare header
            char pkt[HEAD_LEN + DATA_LEN + END_LEN];
            pkt[0] = START;
            pkt[1] = DATA;

            // Send in packets of fixed size
            while (outBuffer.size() >= MIN_TO_SEND)
            {
                // Read from the buffer
                outBuffer.read(DATA_LEN, pkt, 2);
                pkt[DATA_LEN + HEAD_LEN] = END;

                // Start thread that will notify when the device has finished
                // transmission
                Thread::create(&Gamma868::static_waitForLed,
                               STACK_DEFAULT_FOR_PTHREAD, MAIN_PRIORITY,
                               reinterpret_cast<void *>(this));
                // Send
                pthread_mutex_lock(&writingMutex);
                write(fd, pkt, HEAD_LEN + DATA_LEN + END_LEN);
                // TODO start timeout
                {
                    Lock<FastMutex> l(ledMutex);
                    while (pktSent == 0)
                        ledCond.wait(l);  // waitForLed will change the
                                          // variable.
                    pktSent = 0;
                }
                pthread_mutex_unlock(&writingMutex);
            }

            // Wait some time (set in gamma_config) before checking the buffer
            // again
            Thread::sleep(SEND_SLEEP_TIME);
        }
    }
}

/*
 * Changes the associated condition variable when the device's led goes off.
 * TODO put it inside the function?
 */
void Gamma868::waitForLed()
{
    bool sending = false;
    while (1)
    {
        if (gammaLed::value() == 1 /*&& !sending*/)
        {
            sending = true;
        }
        if (/*sending &&*/ gammaLed::value() == 0)
        {
            Lock<FastMutex> l(ledMutex);
            pktSent++;
            ledCond.signal();
            break;
        }
    }
}
