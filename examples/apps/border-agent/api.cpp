#include <common/message.hpp>
#include "openthread-instance.h"
#include "api.h"
#include <net/udp6.hpp>
#include <stdio.h>

using namespace Thread;

void otUdpSocketHandleReceive(otUdpSocket *aUdpSocket, otMessage *aMessage, otMessageInfo *aMessageInfo)
{
    Ip6::Udp &udp = *static_cast<Ip6::Udp*>(aUdpSocket->mTransport);
    udp.HandleMessage(*static_cast<Message *>(aMessage), *static_cast<Ip6::MessageInfo*>(aMessageInfo), *static_cast<Ip6::UdpSocket*>(aUdpSocket));
}

otUdpSocket *otUdpFindSocketByHandle(otInstance *aInstance, void* aHandle)
{
    return aInstance->mUdp.FindSocketByHandle(aHandle);
}

otMessage *otMessageNew(otInstance *aInstance, uint16_t aReserved)
{
    return aInstance->mMessagePool.New(Thread::Message::kTypeIp6, aReserved);
}

ThreadError otMessageFree(otMessage *aMessage)
{
    return static_cast<Message *>(aMessage)->Free();
}

uint16_t otMessageGetLength(otMessage *aMessage)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->GetLength();
}

ThreadError otMessageSetLength(otMessage *aMessage, uint16_t aLength)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->SetLength(aLength);
}

uint16_t otMessageGetOffset(otMessage *aMessage)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->GetOffset();
}

ThreadError otMessageSetOffset(otMessage *aMessage, uint16_t aOffset)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->SetOffset(aOffset);
}

bool otMessageIsLinkSecurityEnabled(otMessage *aMessage)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->IsLinkSecurityEnabled();
}

void otMessageSetDirectTransmission(otMessage *aMessage, bool aEnabled)
{
    Message *message = static_cast<Message *>(aMessage);

    if (aEnabled)
    {
        message->SetDirectTransmission();
    }
    else
    {
        message->ClearDirectTransmission();
    }
}

ThreadError otMessageAppend(otMessage *aMessage, const void *aBuf, uint16_t aLength)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->Append(aBuf, aLength);
}

int otMessageRead(otMessage *aMessage, uint16_t aOffset, void *aBuf, uint16_t aLength)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->Read(aOffset, aLength, aBuf);
}

int otMessageWrite(otMessage *aMessage, uint16_t aOffset, const void *aBuf, uint16_t aLength)
{
    Message *message = static_cast<Message *>(aMessage);
    return message->Write(aOffset, aLength, aBuf);
}

void otTaskletsProcess(otInstance *aInstance)
{
    aInstance->mTaskletScheduler.ProcessQueuedTasklets();
}

bool otTaskletsArePending(otInstance *aInstance)
{
    return aInstance->mTaskletScheduler.AreTaskletsPending();
}

const char* shell(const char* cmd)
{
    static char buf[128] = {0};

    FILE* fp = popen(cmd, "r");
    fgets(buf, sizeof(buf) - 1, fp);
    pclose(fp);
    return buf;
}

#define CMD_PREFIX "/Users/xyk/wpantund-output/usr/local/bin/wpanctl -I utun1 "
const char* otGetMeshLocal16()
{
    static char buf[128] = {0};

    char* p = buf;
    p += sprintf(p, "%sff:fe00:", shell(CMD_PREFIX "getprop IPv6:MeshLocalPrefix | grep 'fd[0-9a-f:]*' -o | tr -d '\\n'"));
    p += sprintf(p, "%s", shell(CMD_PREFIX "getprop Thread:RLOC16 | cut -dx -f2 | tr -d '\\n'"));
    *p = 0;
    return buf;
}
