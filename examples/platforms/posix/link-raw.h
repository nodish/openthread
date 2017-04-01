#ifndef LINK_RAW_H_
#define LINK_RAW_H_

#include <ncp/spinel.h>
#include "openthread/platform/radio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*MacFrameHandler)(void *aContext, const uint8_t *aFrame, uint16_t aFrameLength);

int otLinkRawInit();

void otNcpReceive(otInstance *aInstance, MacFrameHandler aFrameHandler);
ThreadError otNcpSetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);
ThreadError otNcpInsertProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);
ThreadError otNcpRemoveProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

ThreadError otNcpSendPacket(otInstance *aInstance, const struct RadioPacket *pkt);

bool
try_spinel_datatype_unpack(
    const uint8_t *data_in,
    spinel_size_t data_len,
    const char *pack_format,
    ...
    );
#ifdef __cplusplus
}  // extern "C"
#endif

#endif // LINK_RAW_H_
