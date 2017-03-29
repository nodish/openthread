#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include "api.h"
#include "platform_udp.h"
#include <stdio.h>

fd_set sSocketFdSet;
otUdpSocket *sSockets;
int sMaxFd;

static ThreadError udp_socket(otUdpSocket *aUdpSocket, int aDomain)
{
    int fd = socket(aDomain, SOCK_DGRAM, 0);

    if (fd < 0)
    {
        return kThreadError_Error;
    }

    if (fd > sMaxFd)
    {
      sMaxFd = fd;
    }

    FD_SET(fd, &sSocketFdSet);

    aUdpSocket->mHandle = (void*)(long)fd;

    return kThreadError_None;
}

ThreadError platform_udp_socket_ip4(otUdpSocket *aUdpSocket)
{
    return udp_socket(aUdpSocket, PF_INET);
}

ThreadError platform_udp_socket(otUdpSocket *aUdpSocket)
{
    return udp_socket(aUdpSocket, PF_INET6);
}

ThreadError platform_udp_close(otUdpSocket *aUdpSocket)
{
    int fd = (int)(long)aUdpSocket->mHandle;

    if (fd < 0)
    {
        return kThreadError_None;
    }

    close(fd);

    FD_CLR(fd, &sSocketFdSet);

    if (fd == sMaxFd)
    {
        for (--sMaxFd; sMaxFd && !FD_ISSET(sMaxFd, &sSocketFdSet); --sMaxFd);
    }

    aUdpSocket->mHandle = (void*)-1;

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
    return (aAddress->mFields.m32[0] == 0xffffffff &&
            aAddress->mFields.m32[1] == 0xffffffff &&
            aAddress->mFields.m32[2] == 0xffffffff);
}

ThreadError platform_udp_bind_ip4(otUdpSocket *aUdpSocket)
{
    platform_udp_close(aUdpSocket);
    platform_udp_socket_ip4(aUdpSocket);

    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_port = htons(aUdpSocket->mSockName.mPort);
    sin.sin_family = AF_INET;
    memcpy(&sin.sin_addr, &aUdpSocket->mSockName.mAddress.mFields.m32[3], sizeof(sin.sin_addr));

    int fd = (int)(long)aUdpSocket->mHandle;
    bind(fd, (struct sockaddr *)&sin, sizeof(sin));
    return kThreadError_None;
}

ThreadError platform_udp_bind_ip6(otUdpSocket *aUdpSocket)
{
    struct sockaddr_in6 sin6;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(aUdpSocket->mSockName.mPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aUdpSocket->mSockName.mAddress, sizeof(sin6.sin6_addr));

    int fd = (int)(long)aUdpSocket->mHandle;
    bind(fd, (struct sockaddr *)&sin6, sizeof(sin6));
    return kThreadError_None;
}
ThreadError platform_udp_bind(otUdpSocket *aUdpSocket)
{
    if (isIp4Address(&aUdpSocket->mSockName.mAddress))
    {
        platform_udp_bind_ip4(aUdpSocket);
    }
    else
    {
        platform_udp_bind_ip6(aUdpSocket);
    }

    // TODO get destination of udp
    //int flags = 1;
    //setsockopt(mSocket, IPPROTO_IP, IP_RECVORIGDSTADDR, &flags, sizeof(flags));

    return kThreadError_None;
}

ThreadError platform_udp_send_ip4(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    struct sockaddr_in sin;
    int fd = (int)(long)aUdpSocket->mHandle;
    memset(&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_port = htons(aMessageInfo->mPeerPort);
    sin.sin_family = AF_INET;
    memcpy(&sin.sin_addr, &aMessageInfo->mPeerAddr.mFields.m32[3], sizeof(sin.sin_addr));

    uint8_t payload[1280] = {0};
    uint16_t len = otMessageGetLength(aMessage);
    otMessageRead(aMessage, 0, payload, len);
    sendto(fd, payload, len, 0, (struct sockaddr *)&sin, sizeof(sin));

    return kThreadError_None;
}

ThreadError platform_udp_send_ip6(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    struct sockaddr_in6 sin6;
    int fd = (int)(long)aUdpSocket->mHandle;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(aMessageInfo->mPeerPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aMessageInfo->mPeerAddr, sizeof(sin6.sin6_addr));

    uint8_t payload[1280] = {0};
    uint16_t len = otMessageGetLength(aMessage);
    otMessageRead(aMessage, 0, payload, len);
    sendto(fd, payload, len, 0, (struct sockaddr *)&sin6, sizeof(sin6));

    return kThreadError_None;
}

ThreadError platform_udp_send(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    if (isIp4Address(&aUdpSocket->mSockName.mAddress))
    {
        platform_udp_send_ip4(aUdpSocket, aMessage, aMessageInfo);
    }
    else
    {
        platform_udp_send_ip6(aUdpSocket, aMessage, aMessageInfo);
    }
    otMessageFree(aMessage);
    return kThreadError_None;
}

void udp_init(otInstance *aInstance)
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
                printf("Processing from sock %d\n", i);
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
