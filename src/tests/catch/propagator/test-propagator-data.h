/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

namespace TestPropagator1
{
Boardcore::PropagatorState STATE0(0, 0, {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}});

Boardcore::NASState ROCKET_STATES[] = {{0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}},
                            {1,{1.279749e-09,-3.011258e-09,-159.96,-7.404952e-05,0.0001742389,0.1900779,-0.6136712,0.266825,0.6814973,0.296266,8.208404e-09,-1.067651e-08,-7.21759e-09}},
                            {121,{-10.41248,7.573311,-276.8531,-11.85164,8.873552,-105.6077,-0.6068109,0.2361907,0.7229159,0.2310563,-4.514764e-09,2.254351e-08,-1.425117e-08}},
                            {1280,{-755.1172,334.6108,-3154.909,-25.18862,16.98399,0.8057303,-0.05272663,0.5832517,0.8098273,-0.03488965,-1.057722e-08,3.176376e-09,1.256228e-08}},};
}  // namespace TestPropagator1