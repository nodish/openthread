#ifndef API_H_
#define API_H_
#include <openthread/types.h>
#include <openthread/tasklet.h>
#include <openthread/message.h>
#include <openthread/udp.h>

#ifdef __cplusplus
extern "C" {
#endif

//void otSetKek(otInstance *aInstance, uint8_t *aKek);

void otUdpHandleMessage(otInstance *aInstance, otMessage *aMessage, otMessageInfo *aMessageInfo);

otMessage *otMessageNew(otInstance *aInstance, uint16_t aReserved);

void otUdpSocketHandleReceive(otUdpSocket *aUdpSocket, otMessage *aMessage, otMessageInfo *aMessageInfo);

otUdpSocket *otUdpFindSocketByHandle(otInstance *aInstance, void* aHandle);

const char* otGetMeshLocal16();
const char* otGetMeshLocal64();

void ncp_ip6_send(otMessage *aMessage);


#ifdef __cplusplus
}  // extern "C"
#endif

#endif //API_H_
