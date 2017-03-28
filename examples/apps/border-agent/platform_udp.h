#ifndef PLATFORM_UDP_H
#define PLATFORM_UDP_H

#include <openthread/udp.h>
#include <openthread/types.h>

#ifdef __cplusplus
extern "C" {
#endif

ThreadError platform_udp_socket(otUdpSocket *aUdpSocket);
ThreadError platform_udp_close(otUdpSocket *aUdpSocket);
ThreadError platform_udp_bind(otUdpSocket *aUdpSocket);
ThreadError platform_udp_send(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo);

void udp_init(otInstance *aInstance);
void udp_process(otInstance *aInstance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
