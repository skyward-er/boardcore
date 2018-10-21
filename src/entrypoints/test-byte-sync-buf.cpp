/* Copyright (c) 2018 Skyward Experimental Rocketry
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


#include "../shared/utils/ByteSyncedCircularbuffer.h"
#include <assert.h>
#include <stdio.h>

int main() 
{
	ByteSyncedCircularBuffer* buffer = new ByteSyncedCircularBuffer(10);
	uint8_t b[11];
	size_t a;

	assert(buffer->count() == 0);
	assert(buffer->getSize() == 10);
	assert(buffer->isEmpty());

	/* Pop with empty buffer */
	a = buffer->pop(b, 5);
	assert(a == 0);
	assert(buffer->count() == 0);
	assert(buffer->isEmpty() == true);

	a = buffer->pop(b, 50);
	assert(a == 0);
	assert(buffer->count() == 0);
	assert(buffer->isEmpty() == true);

	/* put with empty buffer */
	a = buffer->put((uint8_t*)"foo", 3);
	assert(a == 3);
	assert(buffer->count() == 3);
	assert(buffer->isEmpty() == false);

	/* Fill buffer */
	a = buffer->put((uint8_t*)"skyward", 7);
	assert(a == 7);
	assert(buffer->count() == 10);

	/* Pop with full buffer */
	a = buffer->pop(b, 10);
	assert(a == 10);
	assert(buffer->count() == 0);
	assert(buffer->isEmpty() == true);

	/* String too long */
	a = buffer->put((uint8_t*)"sucameloforte", 13);
	assert(a == 0);
	assert(buffer->isEmpty() == true);
	cout << *buffer;

	a = buffer->put((uint8_t*)"rocksanne", 9);
	assert(a == 9);
	assert(buffer->count() == 9);

	a = buffer->pop(b, 50);
	assert(a == 9);
	assert(buffer->count() == 0);
	assert(buffer->isEmpty() == true);

	assert(buffer->getSize() == 10);

	printf("Test passed!\n");

}