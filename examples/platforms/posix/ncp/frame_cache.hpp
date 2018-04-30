#ifndef FRAME_CACHE_HPP_
#define FRAME_CACHE_HPP_

#include <stdint.h>

namespace ot {

class FrameCache
{
public:
    FrameCache(void)
        : mHead(0)
        , mTail(0)
    {
    }
    bool           IsEmpty(void) const { return mHead == mTail; }
    void           Shift(void);
    void           Push(const uint8_t *aFrame, uint16_t aLength);
    const uint8_t *Peek(uint8_t *aFrame, uint16_t &aLength);

private:
    enum
    {
        kCacheSize = 4096,
    };

    uint8_t  mBuffer[kCacheSize];
    uint16_t mHead;
    uint16_t mTail;
};

} // namespace ot

#endif // FRAME_CACHE_HPP_
