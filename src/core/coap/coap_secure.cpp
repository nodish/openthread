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

#define WPP_NAME "coap_secure.tmh"

#include "coap_secure.hpp"

#include "common/instance.hpp"
#include "common/logging.hpp"
#include "common/owner-locator.hpp"
#include "meshcop/dtls.hpp"
#include "thread/thread_netif.hpp"

#if OPENTHREAD_ENABLE_DTLS

/**
 * @file
 *   This file implements the secure CoAP agent.
 */

namespace ot {
namespace Coap {

CoapSecure::CoapSecure(Instance &aInstance)
    : CoapBase(aInstance, &CoapSecure::HandleRetransmissionTimer, &CoapSecure::HandleResponsesQueueTimer)
    , mDtls(aInstance)
    , mConnectedCallback(NULL)
    , mConnectedContext(NULL)
    , mSocket(aInstance.GetThreadNetif().GetIp6().GetUdp())
{
}

#if OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE
CoapSecure::CoapSecure(Instance &aInstance, Timer::Handler aRetransmissionTimer, Timer::Handler aResponsesQueueTimer)
    : CoapBase(aInstance, aRetransmissionTimer, aResponsesQueueTimer)
    , mDtls(aInstance)
    , mConnectedCallback(NULL)
    , mConnectedContext(NULL)
    , mSocket(aInstance.GetThreadNetif().GetIp6().GetUdp())
{
}
#endif // OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE

otError CoapSecure::Start(uint16_t aPort)
{
    otError error      = OT_ERROR_NONE;
    mConnectedCallback = NULL;
    mConnectedContext  = NULL;

    // Passing mTransportCallback means that we do not want to use socket
    // to transmit/receive messages, so do not open it in that case.
    SuccessOrExit(error = mDtls.Open(HandleDtlsReceive, this));
    SuccessOrExit(error = mDtls.Start(aPort, HandleDtlsConnected));

exit:
    return error;
}

void CoapSecure::SetConnectedCallback(ConnectedCallback aCallback, void *aContext)
{
    mConnectedCallback = aCallback;
    mConnectedContext  = aContext;
}

otError CoapSecure::Stop(void)
{
    otError error;

    SuccessOrExit(error = mSocket.Close());

    if (IsConnectionActive())
    {
        Disconnect();
    }

    ClearCaches();

exit:
    return error;
}

otError CoapSecure::Connect(const Ip6::SockAddr &aSockAddr, ConnectedCallback aCallback, void *aContext)
{
    mConnectedCallback = aCallback;
    mConnectedContext  = aContext;

    return mDtls.Connect(aSockAddr, &CoapSecure::HandleDtlsConnected);
}

bool CoapSecure::IsConnectionActive(void)
{
    return mDtls.IsStarted();
}

bool CoapSecure::IsConnected(void)
{
    return mDtls.IsConnected();
}

otError CoapSecure::Disconnect(void)
{
    return mDtls.Stop();
}

MeshCoP::Dtls &CoapSecure::GetDtls(void)
{
    return mDtls;
}

otError CoapSecure::SetPsk(const uint8_t *aPsk, uint8_t aPskLength)
{
    return mDtls.SetPsk(aPsk, aPskLength);
}

#if OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE

#ifdef MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
otError CoapSecure::SetCertificate(const uint8_t *aX509Cert,
                                   uint32_t       aX509Length,
                                   const uint8_t *aPrivateKey,
                                   uint32_t       aPrivateKeyLength)
{
    return mDtls.SetCertificate(aX509Cert, aX509Length, aPrivateKey, aPrivateKeyLength);
}

otError CoapSecure::SetCaCertificateChain(const uint8_t *aX509CaCertificateChain, uint32_t aX509CaCertChainLenth)
{
    return mDtls.SetCaCertificateChain(aX509CaCertificateChain, aX509CaCertChainLenth);
}
#endif // MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED

#ifdef MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
otError CoapSecure::SetPreSharedKey(const uint8_t *aPsk,
                                    uint16_t       aPskLength,
                                    const uint8_t *aPskIdentity,
                                    uint16_t       aPskIdLength)
{
    return mDtls.SetPreSharedKey(aPsk, aPskLength, aPskIdentity, aPskIdLength);
}
#endif // MBEDTLS_KEY_EXCHANGE_PSK_ENABLED

#ifdef MBEDTLS_BASE64_C
otError CoapSecure::GetPeerCertificateBase64(unsigned char *aPeerCert, size_t *aCertLength, size_t aCertBufferSize)
{
    return mDtls.GetPeerCertificateBase64(aPeerCert, aCertLength, aCertBufferSize);
}
#endif // MBEDTLS_BASE64_C

void CoapSecure::SetClientConnectedCallback(ConnectedCallback aCallback, void *aContext)
{
    mConnectedCallback = aCallback;
    mConnectedContext  = aContext;
}

void CoapSecure::SetSslAuthMode(bool aVerifyPeerCertificate)
{
    mDtls.SetSslAuthMode(aVerifyPeerCertificate);
}

#endif // OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE

otError CoapSecure::SendMessage(Message &aMessage, otCoapResponseHandler aHandler, void *aContext)
{
    otError error = OT_ERROR_NONE;

    VerifyOrExit(IsConnected(), error = OT_ERROR_INVALID_STATE);

    error = CoapBase::SendMessage(aMessage, mDtls.GetMessageInfo(), aHandler, aContext);

exit:
    return error;
}

otError CoapSecure::SendMessage(Message &               aMessage,
                                const Ip6::MessageInfo &aMessageInfo,
                                otCoapResponseHandler   aHandler,
                                void *                  aContext)
{
    return CoapBase::SendMessage(aMessage, aMessageInfo, aHandler, aContext);
}

otError CoapSecure::Send(Message &aMessage, const Ip6::MessageInfo &aMessageInfo)
{
    OT_UNUSED_VARIABLE(aMessageInfo);
    return mDtls.Send(aMessage, aMessage.GetLength());
}

void CoapSecure::HandleDtlsConnected(void *aContext, bool aConnected)
{
    return static_cast<CoapSecure *>(aContext)->HandleDtlsConnected(aConnected);
}

void CoapSecure::HandleDtlsConnected(bool aConnected)
{
    if (mConnectedCallback != NULL)
    {
        mConnectedCallback(aConnected, mConnectedContext);
    }
}

void CoapSecure::HandleRetransmissionTimer(Timer &aTimer)
{
    aTimer.GetOwner<CoapSecure>().CoapBase::HandleRetransmissionTimer();
}

void CoapSecure::HandleResponsesQueueTimer(Timer &aTimer)
{
    aTimer.GetOwner<CoapSecure>().CoapBase::HandleResponsesQueueTimer();
}

void CoapSecure::HandleDtlsReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    static_cast<CoapSecure *>(aContext)->HandleDtlsReceive(*static_cast<Message *>(aMessage),
                                                           *static_cast<const Ip6::MessageInfo *>(aMessageInfo));
}

void CoapSecure::HandleDtlsReceive(Message &aMessage, const Ip6::MessageInfo &aMessageInfo)
{
    CoapBase::Receive(aMessage, aMessageInfo);
}

#if OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE

ApplicationCoapSecure::ApplicationCoapSecure(Instance &aInstance)
    : CoapSecure(aInstance,
                 &ApplicationCoapSecure::HandleUdpTransmit,
                 &ApplicationCoapSecure::HandleRetransmissionTimer,
                 &ApplicationCoapSecure::HandleResponsesQueueTimer)
{
}

void ApplicationCoapSecure::HandleUdpTransmit(Tasklet &aTasklet)
{
    aTasklet.GetOwner<ApplicationCoapSecure>().CoapSecure::HandleUdpTransmit();
}

void ApplicationCoapSecure::HandleRetransmissionTimer(Timer &aTimer)
{
    aTimer.GetOwner<ApplicationCoapSecure>().CoapBase::HandleRetransmissionTimer();
}

void ApplicationCoapSecure::HandleResponsesQueueTimer(Timer &aTimer)
{
    aTimer.GetOwner<ApplicationCoapSecure>().CoapBase::HandleResponsesQueueTimer();
}

#endif // OPENTHREAD_ENABLE_APPLICATION_COAP_SECURE

} // namespace Coap
} // namespace ot

#endif // OPENTHREAD_ENABLE_DTLS
