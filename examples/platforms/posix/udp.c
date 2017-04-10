#include "platform-posix.h"

#include <sys/select.h>
#include <openthread/udp.h>
#include <openthread/platform/udp.h>

#define FD_TO_HANDLE(fd)        ((void*)(long)(fd))
#define HANDLE_TO_FD(handle)    ((int)(long)(handle))
#define INVALID_HANDLE          (FD_TO_HANDLE(-1))

otUdpSocket *sSockets = NULL;

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
        while (udpSocket && udpSocket->mNext != aUdpSocket);
        udpSocket->mNext = aUdpSocket->mNext;
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

void platformUdpProcess(fd_set *aReadFdSet)
{
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
