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
 *   This file implements a BorderAgentProxy role.
 */

#define WPP_NAME "border_agent_proxy.tmh"

#ifdef OPENTHREAD_CONFIG_FILE
#include OPENTHREAD_CONFIG_FILE
#else
#include <openthread-config.h>
#endif

#include <openthread/types.h>
#include <coap/coap_header.hpp>
#include <thread/thread_uris.hpp>
#include <thread/thread_tlvs.hpp>
#include <net/ip6_address.hpp>

#include "border_agent_proxy.hpp"

namespace Thread {
namespace MeshCoP {

OT_TOOL_PACKED_BEGIN
class BorderAgentProxyMeta
{
public:
    BorderAgentProxyMeta(const otIp6Address &aAddress, uint16_t aPort) {
        mAddress = aAddress;
        mPort = HostSwap16(aPort);
    }

    BorderAgentProxyMeta(const Message &aMessage) {
        aMessage.Read(aMessage.GetLength() - sizeof(*this), sizeof(*this), this);
    }

    const Ip6::Address &GetAddress() {
        return *static_cast<const Ip6::Address *>(&mAddress);
    }

    uint16_t GetPort() {
        return HostSwap16(mPort);
    }

    const uint8_t *GetBytes() {
        return (uint8_t *)this;
    }

private:
    otIp6Address mAddress;
    uint16_t     mPort;
} OT_TOOL_PACKED_END;

BorderAgentProxy::BorderAgentProxy(Coap::Server &aCoapServer, Coap::Client &aCoapClient):
    mRelayReceive(OPENTHREAD_URI_RELAY_RX, &BorderAgentProxy::HandleRelayReceive, this),
    mBorderAgentProxyStreamHandler(NULL), mContext(NULL),
    mCoapServer(aCoapServer), mCoapClient(aCoapClient)
{
}

ThreadError BorderAgentProxy::Start(otBorderAgentProxyStreamHandler aBorderAgentProxyStreamHandler, void *aContext)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(!mBorderAgentProxyStreamHandler, error = kThreadError_InvalidState);

    mCoapServer.AddResource(mRelayReceive);
    mBorderAgentProxyStreamHandler = aBorderAgentProxyStreamHandler;
    mContext = aContext;

exit:
    return error;
}

ThreadError BorderAgentProxy::Stop(void)
{
    ThreadError error = kThreadError_None;

    VerifyOrExit(mBorderAgentProxyStreamHandler != NULL, error = kThreadError_InvalidState);

    mCoapServer.RemoveResource(mRelayReceive);

    mBorderAgentProxyStreamHandler = NULL;

exit:
    return error;
}

bool BorderAgentProxy::IsEnabled(void) const
{
    return mBorderAgentProxyStreamHandler != NULL;
}

void BorderAgentProxy::HandleRelayReceive(void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
                                          const otMessageInfo *aMessageInfo)
{
    static_cast<BorderAgentProxy *>(aContext)->DelieverMessage(
        *static_cast<Coap::Header *>(aHeader), *static_cast<Message *>(aMessage),
        *static_cast<const Ip6::MessageInfo *>(aMessageInfo));
}

void BorderAgentProxy::HandleResponse(void *aContext, otCoapHeader *aHeader, otMessage *aMessage,
                                      const otMessageInfo *aMessageInfo, ThreadError aResult)
{
    if (aResult != kThreadError_None)
    {
        return;
    }

    static_cast<BorderAgentProxy *>(aContext)->DelieverMessage(*static_cast<Coap::Header *>(aHeader),
                                                               *static_cast<Message *>(aMessage),
                                                               *static_cast<const Ip6::MessageInfo *>(aMessageInfo));
}

void BorderAgentProxy::DelieverMessage(Coap::Header &aHeader, Message &aMessage,
                                       const Ip6::MessageInfo &aMessageInfo)
{
    ThreadError error;
    Message *message = NULL;

    VerifyOrExit(mBorderAgentProxyStreamHandler != NULL, error = kThreadError_InvalidState);

    VerifyOrExit((message = aMessage.Clone()) != NULL, error = kThreadError_NoBufs);
    message->RemoveHeader(message->GetOffset() - aHeader.GetLength());

    {
        BorderAgentProxyMeta meta(aMessageInfo.GetPeerAddr(), aMessageInfo.GetPeerPort());
        SuccessOrExit(error = message->Append(meta.GetBytes(), sizeof(meta)));
        mBorderAgentProxyStreamHandler(message, mContext);
    }

exit:

    if (error != kThreadError_None && message != NULL)
    {
        message->Free();
    }
}

ThreadError BorderAgentProxy::Send(Message &aMessage)
{
    ThreadError error = kThreadError_None;
    Ip6::MessageInfo messageInfo;

    BorderAgentProxyMeta meta(aMessage);
    aMessage.SetLength(aMessage.GetLength() - sizeof(meta));

    messageInfo.SetPeerAddr(meta.GetAddress());
    messageInfo.SetPeerPort(meta.GetPort());

    if (meta.GetPort() == kCoapUdpPort)
    {
        // this is request to server, send with client
        error = mCoapClient.SendMessage(aMessage, messageInfo, BorderAgentProxy::HandleResponse, this);
    }
    else
    {
        error = mCoapServer.SendMessage(aMessage, messageInfo);
    }

    if (error != kThreadError_None)
    {
        aMessage.Free();
    }

    return error;
}

}  // namespace MeshCoP
}  // namespace Thread
