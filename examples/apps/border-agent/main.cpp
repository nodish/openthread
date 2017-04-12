#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <coap/coap_server.hpp>
#include <coap/coap_client.hpp>
#include <coap/secure_coap_server.hpp>
#include <meshcop/border_agent.hpp>
#include "platform_udp.h"
#include "api.h"
#include "bap.h"

extern "C" {
#include "posix/platform-posix.h"
};

using namespace Thread;

uint32_t NODE_ID = 1;

void platform_init()
{
    platformAlarmInit();
    platformRandomInit();
}

void handle_coap_packet(const void* aBuffer, uint16_t aLength, const uint8_t* aAddress, uint16_t aPort, void* aContext)
{
    const int kCoapUdpPort = 61631;
    otInstance *instance = (otInstance*)aContext;
    char hex[3000];
    sprint_hex(hex, (const uint8_t*)aBuffer, aLength);
    printf("received a packet. buf=%s instance=%p len=%u port=%u ip=%p\n", hex, instance, aLength, aPort, aAddress);
    otMessage* message = otMessageNew(instance, 0);
    otMessageAppend(message, aBuffer, aLength);
    otMessageInfo messageInfo;
    memcpy(messageInfo.mPeerAddr.mFields.m8, aAddress, 16);
    messageInfo.mPeerPort = aPort;
    if (aPort == kCoapUdpPort)
    {
        instance->GetCoapClient().ProcessReceivedMessage(*static_cast<Message*>(message), *static_cast<Ip6::MessageInfo*>(&messageInfo));
    }
    else
    {
        instance->GetCoapServer().ProcessReceivedMessage(*static_cast<Message*>(message), *static_cast<Ip6::MessageInfo*>(&messageInfo));
    }
    otMessageFree(message);
}

void test_border_agent()
{
    Ip6::Address meshLocal16;
    meshLocal16.FromString(otGetMeshLocal16());
    otInstance nc(meshLocal16);
    udp_init(&nc);

    Coap::Server cs(nc, 61631);
    cs.Start();
    nc.mCoapServer = &cs;

    Coap::Client cc(nc);
    cc.Start();
    nc.mCoapClient = &cc;

    MeshCoP::Dtls serverDtls(nc);
    Coap::SecureServer css(nc, serverDtls, 49191);
    nc.mSecureCoapServer = &css;

    MeshCoP::BorderAgent ba(nc);
    ba.Start();

    ncp_enable_border_agent_proxy();
    otBorderAgentProxyStart(handle_coap_packet, &nc);

    while (!usleep(1000)) {
      otTaskletsProcess(&nc);
      platformAlarmProcess(&nc);
      udp_process(&nc);
      otBorderAgentProxyProcess();
    }
}

int main()
{
    platform_init();
    test_border_agent();
}
