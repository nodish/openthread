/*
 *  Copyright (c) 2018, The OpenThread Authors.
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
 *   This file implements a BorderAgent role.
 */

#define WPP_NAME "border_agent.tmh"

#include "border_agent.hpp"

#include <openthread/types.h>

#include "coap/coap_header.hpp"
#include "common/instance.hpp"
#include "common/logging.hpp"
#include "common/new.hpp"
#include "common/owner-locator.hpp"
#include "meshcop/meshcop.hpp"
#include "meshcop/meshcop_tlvs.hpp"
#include "thread/thread_netif.hpp"
#include "thread/thread_tlvs.hpp"
#include "thread/thread_uri_paths.hpp"

#if OPENTHREAD_ENABLE_BORDER_AGENT

using ot::Encoding::BigEndian::HostSwap64;

namespace ot {
namespace MeshCoP {

#define OT_COAP_MAX_TOKEN_LENGTH 8

const char *kLeaderPetition  = OT_URI_PATH_LEADER_PETITION;
const char *kLeaderKeepAlive = OT_URI_PATH_LEADER_KEEP_ALIVE;
const char *kRelayTransmit   = OT_URI_PATH_RELAY_TX;
const char *kRelayReceive    = OT_URI_PATH_RELAY_RX;
const char *kCommissionerGet = OT_URI_PATH_COMMISSIONER_GET;
const char *kCommissionerSet = OT_URI_PATH_COMMISSIONER_SET;
const char *kActiveGet       = OT_URI_PATH_COMMISSIONER_GET;
const char *kActiveSet       = OT_URI_PATH_COMMISSIONER_SET;
const char *kPendingGet      = OT_URI_PATH_COMMISSIONER_GET;
const char *kPendingSet      = OT_URI_PATH_COMMISSIONER_SET;

class ForwardContext
{
public:
    ForwardContext(BorderAgent &aBorderAgent)
        : mBorderAgent(aBorderAgent)
    {
    }

    BorderAgent &GetBorderAgent(void) { return mBorderAgent; }

    void FromHeader(const Coap::Header &aHeader, bool aSeparate)
    {
        mSeparate    = aSeparate;
        mMessageId   = aHeader.GetMessageId();
        mType        = (aHeader.GetType() >> kTypeOffset);
        mTokenLength = aHeader.GetTokenLength();
        memcpy(mToken, aHeader.GetToken(), mTokenLength);
    }

    void ToHeader(Coap::Header &aHeader, Coap::Header::Code aCode)
    {
        if (mType == (OT_COAP_TYPE_NON_CONFIRMABLE >> kTypeOffset) || mSeparate)
        {
            aHeader.Init(OT_COAP_TYPE_NON_CONFIRMABLE, aCode);
        }
        else
        {
            aHeader.Init(OT_COAP_TYPE_ACKNOWLEDGMENT, aCode);
        }

        aHeader.SetMessageId(mSeparate ? 0 : mMessageId);
        aHeader.SetToken(mToken, mTokenLength);
    }

private:
    enum
    {
        kTypeOffset = 4,
    };
    BorderAgent &mBorderAgent;
    uint16_t     mMessageId; ///< The CoAP Message ID
    bool         mSeparate : 1;
    uint8_t      mType : 2;        ///< Type
    uint8_t      mTokenLength : 4; ///< The CoAP Version, Type, and Token Length
    uint8_t      mToken[OT_COAP_MAX_TOKEN_LENGTH];
};

static Coap::Header::Code CoapCodeFromError(otError aError)
{
    switch (aError)
    {
    case OT_ERROR_NONE:
        return OT_COAP_CODE_CHANGED;

    case OT_ERROR_PARSE:
        return OT_COAP_CODE_BAD_REQUEST;

    default:
        return OT_COAP_CODE_INTERNAL_ERROR;
    }
}

void BorderAgent::SendErrorMessage(const Coap::Header &aHeader)
{
    otError      error   = OT_ERROR_NONE;
    Message *    message = NULL;
    ThreadNetif &netif   = GetNetif();

    VerifyOrExit((message = NewMeshCoPMessage(netif.GetCoapSecure(), aHeader)) != NULL, error = OT_ERROR_NO_BUFS);

    SuccessOrExit(error = netif.GetCoapSecure().SendMessage(*message, netif.GetCoapSecure().GetPeerMessageInfo()));

exit:

    if (error != OT_ERROR_NONE && message != NULL)
    {
        message->Free();
    }

    if (error != OT_ERROR_NONE)
    {
        otLogWarnMeshCoP(GetInstance(), "Failed to send CoAP message: %s", otThreadErrorToString(error));
    }
}

void BorderAgent::HandleCoapResponse(void *               aContext,
                                     otCoapHeader *       aHeader,
                                     otMessage *          aMessage,
                                     const otMessageInfo *aMessageInfo,
                                     otError              aResult)
{
    ForwardContext &forwardContext = *static_cast<ForwardContext *>(aContext);
    BorderAgent &   borderAgent    = forwardContext.GetBorderAgent();
    ThreadNetif &   netif          = borderAgent.GetNetif();
    Coap::Header    header;

    OT_UNUSED_VARIABLE(aMessageInfo);
    otLogInfoMeshCoP(GetInstance(), "Got CoAP response[%s]", otThreadErrorToString(aResult));

    if (aResult != OT_ERROR_NONE)
    {
        forwardContext.ToHeader(header, CoapCodeFromError(aResult));
        ExitNow(borderAgent.SendErrorMessage(header));
    }

    if (aMessage == NULL)
    {
        forwardContext.ToHeader(header, CoapCodeFromError(OT_ERROR_RESPONSE_TIMEOUT));
        ExitNow(borderAgent.SendErrorMessage(header));
    }

    forwardContext.ToHeader(header, static_cast<Coap::Header *>(aHeader)->GetCode());

    if (static_cast<Message *>(aMessage)->GetLength() - static_cast<Message *>(aMessage)->GetOffset() > 0)
    {
        header.SetPayloadMarker();
    }

    borderAgent.ForwardToCommissioner(header, *static_cast<Message *>(aMessage));

exit:
    netif.GetInstance().GetHeap().Free(&forwardContext);
}

template <>
void BorderAgent::HandleRequest<&kLeaderPetition>(void *               aContext,
                                                  otCoapHeader *       aHeader,
                                                  otMessage *          aMessage,
                                                  const otMessageInfo *aMessageInfo)
{
    static_cast<BorderAgent *>(aContext)->ForwardToLeader(
        *static_cast<Coap::Header *>(aHeader), *static_cast<Message *>(aMessage),
        *static_cast<const Ip6::MessageInfo *>(aMessageInfo), kLeaderPetition, true);
}

template <>
void BorderAgent::HandleRequest<&kLeaderKeepAlive>(void *               aContext,
                                                   otCoapHeader *       aHeader,
                                                   otMessage *          aMessage,
                                                   const otMessageInfo *aMessageInfo)
{
    static_cast<BorderAgent *>(aContext)->ForwardToLeader(
        *static_cast<Coap::Header *>(aHeader), *static_cast<Message *>(aMessage),
        *static_cast<const Ip6::MessageInfo *>(aMessageInfo), kLeaderKeepAlive, true);
}

template <>
void BorderAgent::HandleRequest<&kRelayTransmit>(void *               aContext,
                                                 otCoapHeader *       aHeader,
                                                 otMessage *          aMessage,
                                                 const otMessageInfo *aMessageInfo)
{
    OT_UNUSED_VARIABLE(aMessageInfo);
    static_cast<BorderAgent *>(aContext)->HandleRelayTransmit(*static_cast<Coap::Header *>(aHeader),
                                                              *static_cast<Message *>(aMessage));
}

template <>
void BorderAgent::HandleRequest<&kRelayReceive>(void *               aContext,
                                                otCoapHeader *       aHeader,
                                                otMessage *          aMessage,
                                                const otMessageInfo *aMessageInfo)
{
    OT_UNUSED_VARIABLE(aMessageInfo);
    static_cast<BorderAgent *>(aContext)->HandleRelayReceive(*static_cast<Coap::Header *>(aHeader),
                                                             *static_cast<Message *>(aMessage));
}

BorderAgent::BorderAgent(Instance &aInstance)
    : InstanceLocator(aInstance)
    , mCommissionerPetition(OT_URI_PATH_COMMISSIONER_PETITION, BorderAgent::HandleRequest<&kLeaderPetition>, this)
    , mCommissionerKeepAlive(OT_URI_PATH_COMMISSIONER_KEEP_ALIVE, BorderAgent::HandleRequest<&kLeaderKeepAlive>, this)
    , mRelayTransmit(OT_URI_PATH_RELAY_TX, BorderAgent::HandleRequest<&kRelayTransmit>, this)
    , mRelayReceive(OT_URI_PATH_RELAY_RX, BorderAgent::HandleRequest<&kRelayReceive>, this)
    , mCommissionerGet(OT_URI_PATH_COMMISSIONER_GET, BorderAgent::HandleRequest<&kCommissionerGet>, this)
    , mCommissionerSet(OT_URI_PATH_COMMISSIONER_SET, BorderAgent::HandleRequest<&kCommissionerSet>, this)
    , mActiveGet(OT_URI_PATH_ACTIVE_GET, BorderAgent::HandleRequest<&kActiveGet>, this)
    , mActiveSet(OT_URI_PATH_ACTIVE_SET, BorderAgent::HandleRequest<&kActiveSet>, this)
    , mPendingGet(OT_URI_PATH_PENDING_GET, BorderAgent::HandleRequest<&kPendingGet>, this)
    , mPendingSet(OT_URI_PATH_PENDING_SET, BorderAgent::HandleRequest<&kPendingSet>, this)
    , mIsStarted(false)
{
}

void BorderAgent::HandleRelayReceive(const Coap::Header &aHeader, const Message &aMessage)
{
    Coap::Header header;

    VerifyOrExit(aHeader.GetType() == OT_COAP_TYPE_NON_CONFIRMABLE && aHeader.GetCode() == OT_COAP_CODE_POST);

    header.Init(OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
    header.AppendUriPathOptions(OT_URI_PATH_RELAY_RX);

    if (aMessage.GetLength() - aMessage.GetOffset() > 0)
    {
        header.SetPayloadMarker();
    }

    SuccessOrExit(ForwardToCommissioner(header, aMessage));
    otLogInfoMeshCoP(GetInstance(), "Sent to leader on %s", kRelayReceive);

exit:
    return;
}

otError BorderAgent::ForwardToCommissioner(const Coap::Header &aHeader, const Message &aMessage)
{
    ThreadNetif &netif   = GetNetif();
    otError      error   = OT_ERROR_NONE;
    Message *    message = NULL;
    uint16_t     offset  = 0;

    VerifyOrExit((message = NewMeshCoPMessage(netif.GetCoapSecure(), aHeader)) != NULL, error = OT_ERROR_NO_BUFS);

    offset = message->GetLength();
    SuccessOrExit(error = message->SetLength(offset + aMessage.GetLength() - aMessage.GetOffset()));
    aMessage.CopyTo(aMessage.GetOffset(), offset, aMessage.GetLength() - aMessage.GetOffset(), *message);

    SuccessOrExit(error = netif.GetCoapSecure().SendMessage(*message, netif.GetCoapSecure().GetPeerMessageInfo()));

    otLogInfoMeshCoP(GetInstance(), "Sent to commissioner");

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnMeshCoP(GetInstance(), "Failed to send to commissioner: %s",
                         otThreadErrorToString(error));
        if (message != NULL)
        {
            message->Free();
        }
    }

    return error;
}

void BorderAgent::HandleRelayTransmit(const Coap::Header &aHeader, const Message &aMessage)
{
    ThreadNetif &          netif = GetNetif();
    otError                error = OT_ERROR_NONE;
    JoinerRouterLocatorTlv joinerRouterRloc;
    Message *              message = NULL;
    Ip6::MessageInfo       messageInfo;
    Coap::Header           header;
    uint16_t               offset = 0;

    VerifyOrExit(aHeader.GetType() == OT_COAP_TYPE_NON_CONFIRMABLE && aHeader.GetCode() == OT_COAP_CODE_POST);

    SuccessOrExit(error = Tlv::GetTlv(aMessage, Tlv::kJoinerRouterLocator, sizeof(joinerRouterRloc), joinerRouterRloc));
    VerifyOrExit(joinerRouterRloc.IsValid(), error = OT_ERROR_PARSE);

    header.Init(OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
    header.SetToken(Coap::Header::kDefaultTokenLength);
    header.AppendUriPathOptions(OT_URI_PATH_RELAY_TX);
    header.SetPayloadMarker();

    VerifyOrExit((message = NewMeshCoPMessage(netif.GetCoap(), header)) != NULL, error = OT_ERROR_NO_BUFS);

    offset = message->GetLength();
    SuccessOrExit(error = message->SetLength(offset + aMessage.GetLength() - aMessage.GetOffset()));
    aMessage.CopyTo(aMessage.GetOffset(), offset, aMessage.GetLength() - aMessage.GetOffset(), *message);

    memset(&messageInfo, 0, sizeof(messageInfo));

    messageInfo.SetSockPort(kCoapUdpPort);
    messageInfo.SetSockAddr(netif.GetMle().GetMeshLocal16());
    messageInfo.SetPeerPort(kCoapUdpPort);
    messageInfo.SetPeerAddr(netif.GetMle().GetMeshLocal16());
    messageInfo.GetPeerAddr().mFields.m16[7] = HostSwap16(joinerRouterRloc.GetJoinerRouterLocator());

    SuccessOrExit(error = netif.GetCoap().SendMessage(*message, messageInfo));

    otLogInfoMeshCoP(borderAgent->GetInstance(), "Sent to joiner router request on %s", OT_URI_PATH_RELAY_TX);

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnMeshCoP(borderAgent->GetInstance(), "Failed to sent to joiner router request " OT_URI_PATH_RELAY_TX " %s",
                         otThreadErrorToString(error));
        if (message != NULL)
        {
            message->Free();
        }
    }
}

void BorderAgent::ForwardToLeader(const Coap::Header &    aHeader,
                                  const Message &         aMessage,
                                  const Ip6::MessageInfo &aMessageInfo,
                                  const char *            aPath,
                                  bool                    aSeparate)
{
    ThreadNetif &    netif          = GetNetif();
    otError          error          = OT_ERROR_NONE;
    ForwardContext * forwardContext = NULL;
    Ip6::MessageInfo messageInfo;
    Coap::Header     header;
    Message *        message = NULL;
    uint16_t         offset  = 0;

    if (aSeparate)
    {
        SuccessOrExit(error = netif.GetCoapSecure().SendAck(aHeader, aMessageInfo));
    }

    forwardContext = static_cast<ForwardContext *>(GetInstance().GetHeap().CAlloc(1, sizeof(ForwardContext)));
    VerifyOrExit(forwardContext != NULL, error = OT_ERROR_NO_BUFS);

    forwardContext = new (forwardContext) ForwardContext(*this);
    forwardContext->FromHeader(aHeader, aSeparate);

    header.Init(OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_POST);
    header.SetToken(Coap::Header::kDefaultTokenLength);
    header.AppendUriPathOptions(aPath);

    // Payload of c/cg may be empty
    if (aMessage.GetLength() - aMessage.GetOffset() > 0)
    {
        header.SetPayloadMarker();
    }

    VerifyOrExit((message = NewMeshCoPMessage(netif.GetCoap(), header)) != NULL, error = OT_ERROR_NO_BUFS);

    offset = message->GetLength();
    SuccessOrExit(error = message->SetLength(offset + aMessage.GetLength() - aMessage.GetOffset()));
    aMessage.CopyTo(aMessage.GetOffset(), offset, aMessage.GetLength() - aMessage.GetOffset(), *message);

    memset(&messageInfo, 0, sizeof(messageInfo));
    SuccessOrExit(netif.GetMle().GetLeaderAloc(messageInfo.GetPeerAddr()));
    messageInfo.SetPeerPort(kCoapUdpPort);
    messageInfo.SetSockAddr(netif.GetMle().GetMeshLocal16());
    messageInfo.SetSockPort(kCoapUdpPort);

    SuccessOrExit(error = netif.GetCoap().SendMessage(*message, messageInfo, HandleCoapResponse, forwardContext));

    otLogInfoMeshCoP(GetInstance(), "Forwarded request to leader on %s", aPath);

exit:
    if (error != OT_ERROR_NONE)
    {
        otLogWarnMeshCoP(GetInstance(), "Failed to forward to leader: %s", otThreadErrorToString(error));
        if (forwardContext != NULL)
        {
            GetInstance().GetHeap().Free(forwardContext);
        }

        if (message != NULL)
        {
            message->Free();
        }

        if (aHeader.GetType() == OT_COAP_TYPE_NON_CONFIRMABLE || aSeparate)
        {
            header.Init(OT_COAP_TYPE_NON_CONFIRMABLE, CoapCodeFromError(error));
        }
        else
        {
            header.Init(OT_COAP_TYPE_ACKNOWLEDGMENT, CoapCodeFromError(error));
        }

        header.SetMessageId(aSeparate ? 0 : aHeader.GetMessageId());
        header.SetToken(aHeader.GetToken(), aHeader.GetTokenLength());

        SendErrorMessage(header);
    }
}

otError BorderAgent::Start(void)
{
    otError           error;
    ThreadNetif &     netif = GetNetif();
    Coap::CoapSecure &coaps = netif.GetCoapSecure();
    Coap::Coap &      coap  = netif.GetCoap();

    VerifyOrExit(!mIsStarted, error = OT_ERROR_ALREADY);

    coaps.AddResource(mActiveGet);
    coaps.AddResource(mActiveSet);
    coaps.AddResource(mPendingGet);
    coaps.AddResource(mPendingSet);
    coaps.AddResource(mCommissionerPetition);
    coaps.AddResource(mCommissionerKeepAlive);
    coaps.AddResource(mCommissionerSet);
    coaps.AddResource(mCommissionerGet);
    coaps.AddResource(mRelayTransmit);

    coap.AddResource(mRelayReceive);

    SuccessOrExit(error = coaps.Start(kBorderAgentUdpPort));
    SuccessOrExit(error = coaps.SetPsk(netif.GetKeyManager().GetPSKc(), OT_PSKC_MAX_SIZE));

    mIsStarted = true;

exit:
    return error;
}

otError BorderAgent::Stop(void)
{
    otError           error;
    ThreadNetif &     netif = GetNetif();
    Coap::CoapSecure &coaps = netif.GetCoapSecure();
    Coap::Coap &      coap  = netif.GetCoap();

    VerifyOrExit(mIsStarted, error = OT_ERROR_ALREADY);

    SuccessOrExit(error = coaps.Stop());

    coaps.RemoveResource(mCommissionerPetition);
    coaps.RemoveResource(mCommissionerKeepAlive);
    coaps.RemoveResource(mCommissionerSet);
    coaps.RemoveResource(mCommissionerGet);
    coaps.RemoveResource(mActiveGet);
    coaps.RemoveResource(mActiveSet);
    coaps.RemoveResource(mPendingGet);
    coaps.RemoveResource(mPendingSet);
    coaps.RemoveResource(mRelayTransmit);

    coap.RemoveResource(mRelayReceive);

    mIsStarted = false;

exit:
    return error;
}

} // namespace MeshCoP
} // namespace ot

#endif // OPENTHREAD_ENABLE_BORDER_AGENT
