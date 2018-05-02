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

#include "platform-posix.h"

#include "frame_cache.hpp"

#include <assert.h>
#include <string.h>

#include "utils/code_utils.h"

#if OPENTHREAD_ENABLE_POSIX_RADIO_NCP

namespace ot {

void FrameCache::Push(const uint8_t *aFrame, uint16_t aLength)
{
    assert(aLength <= 255 && aFrame);
    uint16_t tail = mTail + aLength + 1;

    if (mHead > mTail)
    {
        assert(tail < mHead);
        otEXPECT(tail < mHead);
    }
    else if (tail >= sizeof(mBuffer))
    {
        tail -= sizeof(mBuffer);
        assert(tail < mHead);
        otEXPECT(tail < mHead);
    }

    mBuffer[mTail] = static_cast<uint8_t>(aLength);

    if (tail > mTail)
    {
        memcpy(mBuffer + mTail + 1, aFrame, aLength);
    }
    else
    {
        uint16_t half = (sizeof(mBuffer) - mTail - 1);
        memcpy(mBuffer + mTail + 1, aFrame, half);
        memcpy(mBuffer, aFrame + half, (aLength - half));
    }
    mTail = tail;

exit:
    return;
}

void FrameCache::Shift(void)
{
    if (mHead != mTail)
    {
        mHead += 1 + mBuffer[mHead];
        mHead %= sizeof(mBuffer);
    }
}

const uint8_t *FrameCache::Peek(uint8_t *aFrame, uint16_t &aLength)
{
    const uint8_t *frame = NULL;
    uint16_t       next;

    otEXPECT(mHead != mTail);

    aLength = mBuffer[mHead];
    next    = mHead + 1 + aLength;

    if (next >= sizeof(mBuffer))
    {
        uint16_t half = sizeof(mBuffer) - mHead - 1;
        memcpy(aFrame, mBuffer + mHead + 1, half);
        memcpy(aFrame + half, mBuffer, aLength - half);
        frame = aFrame;
    }
    else
    {
        frame = mBuffer + mHead + 1;
    }

exit:
    return frame;
}

} // namespace ot

#if SELF_TEST
void main(void)
{
    ot::FrameCache fc;
    uint16_t       l;
    uint8_t        a[] = "12345";
    fc.Push(a, sizeof(a));
    assert(fc.Peek(NULL, l));
    assert(!fc.IsEmpty());
    fc.Shift();
    fc.Push(a, sizeof(a));
    assert(!fc.IsEmpty());
    fc.Peek(NULL, l);
    fc.Shift();
    assert(fc.IsEmpty());
}
#endif

#endif // OPENTHREAD_ENABLE_POSIX_RADIO_NCP
