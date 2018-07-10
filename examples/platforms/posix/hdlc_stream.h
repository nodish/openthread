#ifndef OPENTHREAD_HDLC_FRAMING_
#define OPENTHREAD_HDLC_FRAMING_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*HdlcWriter)(const uint8_t *aBuffer, size_t aSize);
typedef void (*FrameHandler)(void *aContext, const uint8_t *aFrame, uint16_t aFrameLength);

void hdlcDecode(uint8_t *aBuffer, size_t aLength, FrameHandler aFrameHandler, void* aContext);
void hdlcEncode(const uint8_t *aBuffer, size_t aLength, HdlcWriter aWriter);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // OPENTHREAD_HDLC_FRAMING_
