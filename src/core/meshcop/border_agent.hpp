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
 *   This file includes definitions for the BorderAgent role.
 */

#ifndef BORDER_AGENT_HPP_
#define BORDER_AGENT_HPP_

#include "openthread-core-config.h"

#include <openthread/commissioner.h>

#include "coap/coap.hpp"
#include "coap/coap_secure.hpp"
#include "common/locator.hpp"
#include "common/timer.hpp"
#include "mac/mac_frame.hpp"
#include "meshcop/announce_begin_client.hpp"
#include "meshcop/dtls.hpp"
#include "meshcop/energy_scan_client.hpp"
#include "meshcop/panid_query_client.hpp"
#include "net/udp6.hpp"
#include "thread/mle.hpp"

namespace ot {

class ThreadNetif;

namespace MeshCoP {

class BorderAgent: public InstanceLocator
{
public:
    /**
     * This constructor initializes the BorderAgent object.
     *
     * @param[in]  aInstance     A reference to the OpenThread instance.
     *
     */
    explicit BorderAgent(Instance &aInstance);

    /**
     * This method starts the BorderAgent service.
     *
     * @retval OT_ERROR_NONE  Successfully started the BorderAgent service.
     *
     */
    otError Start(void);

    /**
     * This method stops the BorderAgent service.
     *
     * @retval OT_ERROR_NONE  Successfully stopped the BorderAgent service.
     *
     */
    otError Stop(void);


private:

    template <const char** aPath> static void HandleRequest(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
    {
        static_cast<BorderAgent *>(aContext)->ForwardToLeader(*static_cast<Coap::Header *>(aHeader), *static_cast<Message *>(aMessage), *static_cast<const Ip6::MessageInfo *>(aMessageInfo), *aPath, false);
    }

    static void HandleCoapResponse(void *aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo, otError aResult);

    void SendErrorMessage(const Coap::Header &aHeader);
    void ForwardToLeader(const Coap::Header &aHeader, const Message &aMessage, const Ip6::MessageInfo &aMessageInfo, const char* aPath, bool aSeparate);
    otError ForwardToCommissioner(const Coap::Header &aHeader, const Message &aMessage);
    void HandleRelayTransmit(const Coap::Header &aHeader, const Message &aMessage);
    void HandleRelayReceive(const Coap::Header &aHeader, const Message &aMessage);

    enum {
        kBorderAgentUdpPort = 49191,
    };

    Ip6::MessageInfo mMessageInfo;

    Coap::Resource mCommissionerPetition;
    Coap::Resource mCommissionerKeepAlive;
    Coap::Resource mRelayTransmit;
    Coap::Resource mRelayReceive;
    Coap::Resource mCommissionerGet;
    Coap::Resource mCommissionerSet;
    Coap::Resource mActiveGet;
    Coap::Resource mActiveSet;
    Coap::Resource mPendingGet;
    Coap::Resource mPendingSet;

    bool mIsStarted;
};

}  // namespace MeshCoP
}  // namespace ot

#endif  // BORDER_AGENT_HPP_
