#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include "api.h"
#include "platform_udp.h"
#include <stdio.h>

fd_set sSocketFdSet;
otUdpSocket *sSockets = NULL;
int sMaxFd;

#define FD_TO_HANDLE(fd)        ((void*)(long)(fd))
#define HANDLE_TO_FD(handle)    ((int)(long)(fd))
#define INVALID_HANDLE          ((void*)-1)

ThreadError otPlatUdpSocket(otUdpSocket *aUdpSocket)
{
    int fd = socket(PF_INET6, SOCK_DGRAM, 0);

    if (fd < 0)
    {
        return kThreadError_NoBufs;
    }

    aUdpSocket->mHandle = (void*)(long)fd;
    aUdpSocket->mNext = sSockets;
    sSockets = aUdpSocket;

    return kThreadError_None;
}

ThreadError otPlatUdpClose(otUdpSocket *aUdpSocket)
{
    int fd = (int)(long)aUdpSocket->mHandle;

    if (fd < 0)
    {
        return kThreadError_InvalidArgs;
    }

    close(fd);
    aUdpSocket->mHandle = INVALID_HANDLE;

    if (aUdpSocket == sSockets)
    {
        sSockets = sSockets->mNext;
    }
    else
    {
        otUdpSocket *udpSocket = sSockets;
        while (udpSocket && udpSocket->mNext != aUdpSocket);
        udpSocket->mNext = aUdpSocket->mNext;
    }

    return kThreadError_None;
}

const char* dump_ip(const otIp6Address *aAddress)
{
    static char ip[120];
    const uint16_t *m16 = aAddress->mFields.m16;
    snprintf(ip, sizeof(ip), "%x:%x:%x:%x:%x:%x:%x:%x",
             htons(m16[0]), htons(m16[1]),
             htons(m16[2]), htons(m16[3]),
             htons(m16[4]), htons(m16[5]),
             htons(m16[6]), htons(m16[7]));
    return ip;
}

static int isIp4Address(otIp6Address *aAddress)
{
    return (aAddress->mFields.m32[0] == 0 &&
            aAddress->mFields.m32[1] == 0 &&
            aAddress->mFields.m16[4] == 0 &&
            aAddress->mFields.m16[5] == 0xffff);
}

ThreadError otPlatUdpBind(otUdpSocket *aUdpSocket)
{
    ThreadError error = kThreadError_None;

    struct sockaddr_in6 sin6;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(aUdpSocket->mSockName.mPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aUdpSocket->mSockName.mAddress, sizeof(sin6.sin6_addr));

    int fd = (int)(long)aUdpSocket->mHandle;
    VerifyOrExit(0 == bind(fd, (struct sockaddr *)&sin6, sizeof(sin6)), error = kThreadError_Error);

    int flags = 1;
#ifdef __APPLE__
    setsockopt(fd, IPPROTO_IPV6, IP_RECVDSTADDR, &flags, sizeof(flags));
#else
    setsockopt(fd, IPPROTO_IPV6, IP_RECVORIGDSTADDR, &flags, sizeof(flags));
#endif

exit:

    return error;
}

ThreadError otPlatUdpSend(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    struct sockaddr_in6 sin6;
    int fd = (int)(long)aUdpSocket->mHandle;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));

    // dest
    sin6.sin6_port = htons(aMessageInfo->mPeerPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aMessageInfo->mPeerAddr, sizeof(sin6.sin6_addr));

    // src
    uint8_t payload[1280] = {0};
    uint16_t len = otMessageGetLength(aMessage);
    otMessageRead(aMessage, 0, payload, len);
    sendto(fd, payload, len, 0, (struct sockaddr *)&sin6, sizeof(sin6));

    return kThreadError_None;
}

Threvoid udp_init(otInstance *aInstance)
{
    (void)aInstance;
    FD_ZERO(&sSocketFdSet);
}

void udp_process(otInstance *aInstance)
{
    uint8_t payload[1300];

    fd_set socketFdSet = sSocketFdSet;
    struct timeval tv = {0, 0};
    while (select(sMaxFd + 1, &socketFdSet, NULL, NULL, &tv) > 0) {
        for (int i = 0; i < sMaxFd + 1; i++) {
            if (FD_ISSET(i, &socketFdSet)) {
                //printf("Processing from sock %d\n", i);
                otUdpSocket *udpSocket = otUdpFindSocketByHandle(aInstance, (void*)(long)i);
                if (udpSocket == NULL) continue;
                if (isIp4Address(&udpSocket->mSockName.mAddress))
                {
                    struct sockaddr_in sin;
                    socklen_t socklen = sizeof(sin);
                    ssize_t len = recvfrom(i, payload, sizeof(payload), 0, (struct sockaddr*)&sin, &socklen);
                    //printf("Processing from socklen %d\n", socklen);
                    otMessage* message = otMessageNew(aInstance, 0);
                    otMessageAppend(message, payload, (uint16_t)len);
                    otMessageInfo messageInfo;
                    messageInfo.mPeerAddr.mFields.m32[0] = 0xffffffff;
                    messageInfo.mPeerAddr.mFields.m32[1] = 0xffffffff;
                    messageInfo.mPeerAddr.mFields.m32[2] = 0xffffffff;
                    memcpy(&messageInfo.mPeerAddr.mFields.m32[3], &sin.sin_addr, sizeof(messageInfo.mPeerAddr));
                    messageInfo.mPeerPort = ntohs(sin.sin_port);
                    otUdpSocketHandleReceive(udpSocket, message, &messageInfo);
                    otMessageFree(message);
                }
                else
                {
                    struct sockaddr_in6 sin6;
                    socklen_t socklen = sizeof(sin6);
                    ssize_t len = recvfrom(i, payload, sizeof(payload), 0, (struct sockaddr*)&sin6, &socklen);
                    //printf("Processing from socklen %d\n", socklen);
                    otMessage* message = otMessageNew(aInstance, 0);
                    otMessageAppend(message, payload, (uint16_t)len);
                    otMessageInfo messageInfo;
                    memcpy(&messageInfo.mPeerAddr, &sin6.sin6_addr, sizeof(messageInfo.mPeerAddr));
                    messageInfo.mPeerPort = ntohs(sin6.sin6_port);
                    otUdpSocketHandleReceive(udpSocket, message, &messageInfo);
                    otMessageFree(message);
                }

            }
        }
        socketFdSet = sSocketFdSet;
    }
}

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}
void platformRadioUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, int *aMaxFd)
{
    if (aReadFdSet != NULL && (sState != kStateTransmit || sAckWait))
    {
        FD_SET(sSockFd, aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }

    if (aWriteFdSet != NULL && sState == kStateTransmit && !sAckWait)
    {
        FD_SET(sSockFd, aWriteFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }
}

