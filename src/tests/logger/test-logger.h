/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#pragma once

#include <cstring>

#ifdef _MIOSIX
#include <miosix.h>
#endif //_MIOSIX

class Dummy
{
public:
    Dummy()
    {
#ifdef _MIOSIX
        timestamp=miosix::getTick();
#else //_MIOSIX
        timestamp=0;
#endif //_MIOSIX
        memset(x,0,sizeof(x));
    }
    
    void correctValue()
    {
        for(int i=0;i<num;i++) x[i]=42;
    }
    
    void print(std::ostream& os) const
    {
        os<<"timestamp="<<timestamp<<' ';
        for(int i=0;i<num;i++)
        {
            if(x[i]==42) continue;
            os<<"unserialized incorrectly, x["<<i<<"]="<<x[i];
            return;
        }
        os<<"ok";
    }
private:
    long long timestamp;
    static const int num=50;
    int x[num];
};
