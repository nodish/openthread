/*
 *  Copyright (c) 2018, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <openthread/config.h>

#include <stdlib.h>

#include "test_platform.h"
#include "test_util.h"

/**
 * Verify allocate int.
 *
 */
void TestAllocateInt(void)
{
    ot::Instance *instance = testInitInstance();
    size_t        before   = instance->GetHeap().GetFreeSize();
    int *         i        = new int(0);
    size_t        after    = instance->GetHeap().GetFreeSize();
    VerifyOrQuit(i != NULL, "TestAllocateInt allocation failed!\n");
    VerifyOrQuit(before - after >= sizeof(*i), "TestAllocateInt usage check failed!\n");
    delete i;
    testFreeInstance(instance);
}

/**
 * Verify allocate char.
 *
 */
void TestAllocateChar(void)
{
    ot::Instance *instance = testInitInstance();
    size_t        before   = instance->GetHeap().GetFreeSize();
    char *        c        = new char(0);
    size_t        after    = instance->GetHeap().GetFreeSize();
    VerifyOrQuit(c != NULL, "TestAllocateChar allocation failed!\n");
    VerifyOrQuit(before - after >= sizeof(*c), "TestAllocateChar usage check failed!\n");
    delete c;
    testFreeInstance(instance);
}

/**
 * Verify allocate array.
 *
 */
void TestAllocateArray(void)
{
    ot::Instance *instance = testInitInstance();
    size_t        before   = instance->GetHeap().GetFreeSize();
    int *         a        = new int[10];
    size_t        after    = instance->GetHeap().GetFreeSize();
    VerifyOrQuit(a != NULL, "TestAllocateArray allocation failed!\n");
    VerifyOrQuit(before - after >= sizeof(int) * 10, "TestAllocateArray usage check failed!\n");
    delete[] a;
    testFreeInstance(instance);
}

void RunTimerTests(void)
{
    TestAllocateInt();
    TestAllocateChar();
    TestAllocateArray();
}

#ifdef ENABLE_TEST_MAIN
int main(void)
{
    RunTimerTests();
    printf("All tests passed\n");
    return 0;
}
#endif
