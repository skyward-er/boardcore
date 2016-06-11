/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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

#include <Common.h>
#include <ethernet/UdpSocket.h>

using namespace miosix;
using namespace std;

int main() {
    const uint8_t ipAddr[] = {192,168,1,30}; // Device's IP address
    const uint8_t mask[] = {255,255,255,0};  // Subnet mask
    const uint8_t destIp[] = {192,168,1,4};  // Destination IP address
    
    unsigned char message[] = "Message from the hell...\n";
    
    W5200& eth = W5200::instance();
    eth.setIpAddress(ipAddr);
    eth.setSubnetMask(mask);
    
    UdpSocket sock(2020);

    float arr[2] = {0,0};
    
    while(1) {
        arr[0] += 10.0;
        arr[1] += 0.10;
        sock.sendTo(destIp,1234,arr,sizeof(arr));
        Thread::sleep(20);
    }
    
    return 0;
}
