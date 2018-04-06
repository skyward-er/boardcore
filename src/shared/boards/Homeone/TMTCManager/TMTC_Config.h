/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#ifndef TMTC_CONFIG
#define TMTC_CONFIG

#define TMTC_OUT_BUFFER_SIZE (20*sizeof(mavlink_message_t)) // Default size of the output messages buffer
#define TMTC_SEND_TIMEOUT 10 // Default timeout before sending next packet
#define TMTC_MAX_PKT_SIZE (5*sizeof(mavlink_message_t)) 	// Maxmimum dimension of the packet
#define TMTC_MAX_TRIES_PER_PACKET 3	// Maximum number of tries when sending a packet

#define TMTC_SENDER_STACKSIZE STACK_DEFAULT_FOR_PTHREAD
#define TMTC_SENDER_PRIORITY MAIN_PRIORITY
#define TMTC_RECEIVER_STACKSIZE STACK_DEFAULT_FOR_PTHREAD
#define TMTC_RECEIVER_PRIORITY MAIN_PRIORITY

#endif /* CONFIG_H */
