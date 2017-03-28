#ifndef API_H_
#define API_H_
#include <openthread/types.h>
#include <openthread/tasklet.h>
#include <openthread/message.h>
#include <openthread/udp.h>

#ifdef __cplusplus
extern "C" {
#endif

void otUdpHandleMessage(otInstance *aInstance, otMessage *aMessage, otMessageInfo *aMessageInfo);

otMessage *otMessageNew(otInstance *aInstance, uint16_t aReserved);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif //API_H_
