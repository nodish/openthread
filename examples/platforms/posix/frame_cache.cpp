#include "frame_cache.hpp"

#include <assert.h>
#include <string.h>

#include "utils/code_utils.h"

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
    uint16_t next;

    otEXPECT(mHead != mTail);

    aLength = mBuffer[mHead];
    next = mHead + 1 + aLength;

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
    uint16_t l;
    uint8_t a[] = "12345";
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
