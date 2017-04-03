#ifndef OTC_H_
#define OTC_H_

#include <ncp/spinel.h>
#include "openthread/platform/radio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*MacFrameHandler)(void *aContext, const uint8_t *aFrame, uint16_t aFrameLength);
typedef void (*MacFrameDoneHandler)(otInstance *aInstance);
typedef void (*SpinelCmdHandler)( otInstance *aInstance, void* Context, uint32_t command, spinel_prop_key_t key, const uint8_t* data, spinel_size_t dataLength);

int otcOpen(MacFrameHandler aMacFrameHandler, MacFrameDoneHandler aMacFrameDoneHandler);

/**
 * read bytes
 */
void otcReceive(otInstance *aInstance);

ThreadError otcGetProp(otInstance *aInstance, spinel_prop_key_t key, const char* aFormat, ...);
ThreadError otcSetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);
ThreadError otcInsertProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);
ThreadError otcRemoveProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

ThreadError otcSendPacket(otInstance *aInstance, const struct RadioPacket *pkt, bool wait);

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

#endif // OTC_H_
