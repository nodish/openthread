/*
 *  Copyright (c) 2016, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements UDP/IPv6 sockets.
 */

#include <stdio.h>

#if PLATFORM_UDP
#include "platform_udp.h"
#include "openthread-instance.h"
#endif

#include <common/code_utils.hpp>
#include <common/encoding.hpp>
#include <net/ip6.hpp>
#include <net/udp6.hpp>

using Thread::Encoding::BigEndian::HostSwap16;

namespace Thread {
namespace Ip6 {

UdpSocket::UdpSocket(Udp &aUdp)
{
    mTransport = &aUdp;
}

Message *UdpSocket::NewMessage(uint16_t aReserved)
{
    return static_cast<Udp *>(mTransport)->NewMessage(aReserved);
}

ThreadError UdpSocket::Open(otUdpReceive aHandler, void *aContext)
{
    memset(&mSockName, 0, sizeof(mSockName));
    memset(&mPeerName, 0, sizeof(mPeerName));
    mHandler = aHandler;
    mContext = aContext;

    return static_cast<Udp *>(mTransport)->AddSocket(*this);
}

ThreadError UdpSocket::Bind(const SockAddr &aSockAddr)
{
    mSockName = aSockAddr;
#if PLATFORM_UDP
    return platform_udp_bind(this);
#else
    return kThreadError_None;
#endif
}

ThreadError UdpSocket::Close(void)
{
    ThreadError error = kThreadError_None;

    SuccessOrExit(error = static_cast<Udp *>(mTransport)->RemoveSocket(*this));
    memset(&mSockName, 0, sizeof(mSockName));
    memset(&mPeerName, 0, sizeof(mPeerName));

exit:
    return error;
}

ThreadError UdpSocket::SendTo(Message &aMessage, const MessageInfo &aMessageInfo)
{
#if PLATFORM_UDP
    return platform_udp_send(this, &aMessage, &aMessageInfo);
#else
    ThreadError error = kThreadError_None;
    MessageInfo messageInfoLocal;
    UdpHeader udpHeader;

    messageInfoLocal = aMessageInfo;

    if (messageInfoLocal.GetSockAddr().IsUnspecified())
    {
        messageInfoLocal.SetSockAddr(GetSockName().GetAddress());
    }

    if (GetSockName().mPort == 0)
    {
        GetSockName().mPort = static_cast<Udp *>(mTransport)->GetEphemeralPort();
    }

    udpHeader.SetSourcePort(GetSockName().mPort);
    udpHeader.SetDestinationPort(messageInfoLocal.mPeerPort);
    udpHeader.SetLength(sizeof(udpHeader) + aMessage.GetLength());
    udpHeader.SetChecksum(0);

    SuccessOrExit(error = aMessage.Prepend(&udpHeader, sizeof(udpHeader)));
    aMessage.SetOffset(0);
    SuccessOrExit(error = static_cast<Udp *>(mTransport)->SendDatagram(aMessage, messageInfoLocal, kProtoUdp));

exit:
    return error;
#endif
}

Udp::Udp():
    mEphemeralPort(kDynamicPortMin),
    mSockets(NULL)
{
}

ThreadError Udp::AddSocket(UdpSocket &aSocket)
{
    ThreadError error = kThreadError_None;
#if PLATFORM_UDP
    SuccessOrExit(error = platform_udp_socket(&aSocket));
#endif
    for (UdpSocket *cur = mSockets; cur; cur = cur->GetNext())
    {
        if (cur == &aSocket)
        {
            ExitNow();
        }
    }

    aSocket.SetNext(mSockets);
    mSockets = &aSocket;

exit:
    return error;
}

ThreadError Udp::RemoveSocket(UdpSocket &aSocket)
{
#if PLATFORM_UDP
    platform_udp_close(&aSocket);
#endif
    if (mSockets == &aSocket)
    {
        mSockets = mSockets->GetNext();
    }
    else
    {
        for (UdpSocket *socket = mSockets; socket; socket = socket->GetNext())
        {
            if (socket->GetNext() == &aSocket)
            {
                socket->SetNext(aSocket.GetNext());
                break;
            }
        }
    }

    aSocket.SetNext(NULL);

    return kThreadError_None;
}

uint16_t Udp::GetEphemeralPort(void)
{
    uint16_t rval = mEphemeralPort;

    if (mEphemeralPort < kDynamicPortMax)
    {
        mEphemeralPort++;
    }
    else
    {
        mEphemeralPort = kDynamicPortMin;
    }

    return rval;
}

Message *Udp::NewMessage(uint16_t aReserved)
{
#if PLATFORM_UDP
    return otInstanceFromUdp(this)->mMessagePool.New(Message::kTypeIp6, aReserved);
#else
    return mMessagePool.New(Message::kTypeIp6, aReserved);
#endif
}

#if PLATFORM_UDP
ThreadError Udp::HandleMessage(Message &aMessage, MessageInfo &aMessageInfo)
{
    for (UdpSocket *socket = mSockets; socket; socket = socket->GetNext())
    {
        if (socket->mHandle == (void*)(long)aMessageInfo.mSockAddr.mFields.m32[0])
            socket->HandleUdpReceive(aMessage, aMessageInfo);
    }
    return kThreadError_None;
}
#else
ThreadError Udp::SendDatagram(Message &aMessage, MessageInfo &aMessageInfo, IpProto aIpProto)
{
    return mIp6.SendDatagram(aMessage, aMessageInfo, aIpProto);
}

ThreadError Udp::HandleMessage(Message &aMessage, MessageInfo &aMessageInfo)
{
    ThreadError error = kThreadError_None;
    UdpHeader udpHeader;
    uint16_t payloadLength;
    uint16_t checksum;

    payloadLength = aMessage.GetLength() - aMessage.GetOffset();

    // check length
    VerifyOrExit(payloadLength >= sizeof(UdpHeader), error = kThreadError_Parse);

    // verify checksum
    checksum = Ip6::ComputePseudoheaderChecksum(aMessageInfo.GetPeerAddr(), aMessageInfo.GetSockAddr(),
                                                payloadLength, kProtoUdp);
    checksum = aMessage.UpdateChecksum(checksum, aMessage.GetOffset(), payloadLength);
    VerifyOrExit(checksum == 0xffff, ;);

    VerifyOrExit(aMessage.Read(aMessage.GetOffset(), sizeof(udpHeader), &udpHeader) == sizeof(udpHeader),);
    aMessage.MoveOffset(sizeof(udpHeader));
    aMessageInfo.mPeerPort = udpHeader.GetSourcePort();
    aMessageInfo.mSockPort = udpHeader.GetDestinationPort();

    // find socket
    for (UdpSocket *socket = mSockets; socket; socket = socket->GetNext())
    {
        if (socket->GetSockName().mPort != udpHeader.GetDestinationPort())
        {
            continue;
        }

        if (socket->GetSockName().mScopeId != 0 &&
            socket->GetSockName().mScopeId != aMessageInfo.mInterfaceId)
        {
            continue;
        }

        if (!aMessageInfo.GetSockAddr().IsMulticast() &&
            !socket->GetSockName().GetAddress().IsUnspecified() &&
            socket->GetSockName().GetAddress() != aMessageInfo.GetSockAddr())
        {
            continue;
        }

        // verify source if connected socket
        if (socket->GetPeerName().mPort != 0)
        {
            if (socket->GetPeerName().mPort != udpHeader.GetSourcePort())
            {
                continue;
            }

            if (!socket->GetPeerName().GetAddress().IsUnspecified() &&
                socket->GetPeerName().GetAddress() != aMessageInfo.GetPeerAddr())
            {
                continue;
            }
        }

        socket->HandleUdpReceive(aMessage, aMessageInfo);
    }

exit:
    return error;
}

ThreadError Udp::UpdateChecksum(Message &aMessage, uint16_t aChecksum)
{
    aChecksum = aMessage.UpdateChecksum(aChecksum, aMessage.GetOffset(), aMessage.GetLength() - aMessage.GetOffset());

    if (aChecksum != 0xffff)
    {
        aChecksum = ~aChecksum;
    }

    aChecksum = HostSwap16(aChecksum);
    aMessage.Write(aMessage.GetOffset() + UdpHeader::GetChecksumOffset(), sizeof(aChecksum), &aChecksum);
    return kThreadError_None;
}
#endif

}  // namespace Ip6
}  // namespace Thread
