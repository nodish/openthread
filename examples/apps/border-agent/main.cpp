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

    while (!usleep(1000)) {
      otTaskletsProcess(&nc);
      platformAlarmProcess(&nc);
      udp_process(&nc);
    }
}

int main()
{
    platform_init();
    test_border_agent();
}
