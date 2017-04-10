#include "platform-posix.h"

#include <arpa/inet.h>
#include <sys/select.h>
#include <net/if.h>
#include <openthread/udp.h>
#include <openthread/platform/udp.h>

#define FD_TO_HANDLE(fd)        ((void*)(long)(fd))
#define HANDLE_TO_FD(handle)    ((int)(long)(handle))
#define INVALID_HANDLE          (FD_TO_HANDLE(-1))

static otUdpSocket *sSockets = NULL;
static int sGroupSocket = -1;
static const char *kAllRoutersMulticast = "ff02::2";
static const char *sTunInterface = "utun1";
static bool sInitialized = false;

ThreadError otPlatUdpSocket(otUdpSocket *aUdpSocket)
{
    int fd = socket(AF_INET6, SOCK_DGRAM, 0);

    if (fd < 0)
    {
        return kThreadError_Error;
    }

    aUdpSocket->mHandle = FD_TO_HANDLE(fd);
    aUdpSocket->mNext = sSockets;
    sSockets = aUdpSocket;

    return kThreadError_None;
}

ThreadError otPlatUdpClose(otUdpSocket *aUdpSocket)
{
    int fd = HANDLE_TO_FD(aUdpSocket->mHandle);

    if (fd < 0)
    {
        return kThreadError_None;
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
        while (udpSocket && udpSocket->mNext != aUdpSocket) udpSocket = udpSocket->mNext;
        if (udpSocket != NULL)
        {
            udpSocket->mNext = aUdpSocket->mNext;
        }
        aUdpSocket->mNext = NULL;
    }

    return kThreadError_None;
}

ThreadError otPlatUdpBind(otUdpSocket *aUdpSocket)
{
    ThreadError error = kThreadError_None;
    struct sockaddr_in6 sin6;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(aUdpSocket->mSockName.mPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aUdpSocket->mSockName.mAddress, sizeof(sin6.sin6_addr));

    int fd = HANDLE_TO_FD(aUdpSocket->mHandle);
    VerifyOrExit(-1 == bind(fd, (struct sockaddr *)&sin6, sizeof(sin6)), error = kThreadError_Error);

    if (aUdpSocket->mSockName.mPort == 0)
    {
        socklen_t len = sizeof(sin6);
        getsockname(fd, (struct sockaddr *)&sin6, &len);
        aUdpSocket->mSockName.mPort = ntohs(sin6.sin6_port);
    }

exit:

    return error;
}

ThreadError otPlatUdpSend(otUdpSocket *aUdpSocket, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    ThreadError error = kThreadError_None;
    int fd = HANDLE_TO_FD(aUdpSocket->mHandle);

    struct sockaddr_in6 sin6;
    memset(&sin6, 0, sizeof(struct sockaddr_in6));
    sin6.sin6_port = htons(aMessageInfo->mPeerPort);
    sin6.sin6_family = AF_INET6;
    memcpy(&sin6.sin6_addr, &aMessageInfo->mPeerAddr, sizeof(sin6.sin6_addr));

    uint8_t payload[1280] = {0};
    uint16_t len = otMessageGetLength(aMessage);
    otMessageRead(aMessage, 0, payload, len);
    VerifyOrExit(0 == sendto(fd, payload, len, 0, (struct sockaddr *)&sin6, sizeof(sin6)), error = kThreadError_Error);

exit:

    return kThreadError_None;
}

void platformUdpUpdateFdSet(fd_set *aReadFdSet, int *aMaxFd)
{
    for (otUdpSocket *socket = sSockets; socket != NULL; socket = socket->mNext)
    {
        int fd = HANDLE_TO_FD(socket->mHandle);
        FD_SET(fd, aReadFdSet);
        if (aMaxFd != NULL && *aMaxFd < fd)
        {
            *aMaxFd = fd;
        }
    }
}

void platformStateChangedCallback(uint32_t aFlags, void *aContext)
{
    otInstance *instance = (otInstance *)aContext;

    if (sGroupSocket == -1)
    {
        sGroupSocket = socket(AF_INET6, SOCK_DGRAM, 0);
    }

    if (aFlags & OT_NET_ROLE)
    {
        otDeviceRole role = otThreadGetDeviceRole(instance);

        struct ipv6_mreq mreq;
        inet_pton(AF_INET6, kAllRoutersMulticast, &mreq.ipv6mr_multiaddr);
        mreq.ipv6mr_interface = if_nametoindex(sTunInterface);

        switch (role)
        {
        case kDeviceRoleLeader:
            break;

        case kDeviceRoleRouter:
            setsockopt(sGroupSocket, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq));
            break;

        default:
            setsockopt(sGroupSocket, IPPROTO_IPV6, IPV6_LEAVE_GROUP, &mreq, sizeof(mreq));
            break;
        }
    }
}

void platformUdpInit(otInstance *aInstance)
{
    otSetStateChangedCallback(aInstance, platformStateChangedCallback, aInstance);
}

void platformUdpProcess(otInstance *aInstance, fd_set *aReadFdSet)
{
    if (!sInitialized)
    {
        platformUdpInit(aInstance);
    }

    for (otUdpSocket *socket = sSockets; socket != NULL; socket = socket->mNext)
    {
        int fd = HANDLE_TO_FD(socket->mHandle);
        if (FD_ISSET(fd, aReadFdSet))
        {
            struct sockaddr_in6 sin6;
            socklen_t socklen = sizeof(sin6);
            uint8_t payload[1280] = {0};
            otMessageInfo messageInfo;

            ssize_t len = recvfrom(fd, payload, sizeof(payload), 0, (struct sockaddr*)&sin6, &socklen);
            memcpy(&messageInfo.mPeerAddr, &sin6.sin6_addr, sizeof(messageInfo.mPeerAddr));
            otUdpInput(socket, payload, (uint16_t)len, &messageInfo);
        }
    }
}
