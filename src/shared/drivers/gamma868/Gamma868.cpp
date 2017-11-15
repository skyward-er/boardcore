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

using namespace std;

#ifdef _MIOSIX

#include <miosix.h>

using namespace miosix;

#endif //_MIOSIX

Gamma868::Gamma868(const char *serialPath)
{
    fd=open(serialPath,O_RDWR);
    if(fd<0) printf("Cannot open %s\n", serialPath);
}

bool Gamma868::send(const char *msg)
{
    //TODO output buffer and synchronize
    int length = strlen(msg);
    write(fd, msg, length);
    return true;
}

bool Gamma868::receive(int bufLen, char *buf)
{
    pthread_mutex_lock(&readingMutex);
    char received[bufLen+1];
    read(fd, &received, bufLen+1);
    
    for(int i = 0; i < bufLen; i++){
        buf[i] = received[i];
    }
    pthread_mutex_unlock(&readingMutex);
    return true;
}