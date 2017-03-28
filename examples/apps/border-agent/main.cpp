#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <coap/coap_server.hpp>
#include <coap/coap_client.hpp>
#include <coap/secure_coap_client.hpp>
#include <coap/secure_coap_server.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "platform_udp.h"
extern "C" {
#include "posix/platform-posix.h"
};

using namespace Thread;

const uint8_t psk[] =
{
    0xc3, 0xf5, 0x93, 0x68, 0x44, 0x5a, 0x1b, 0x61,
    0x06, 0xbe, 0x42, 0x0a, 0x70, 0x6d, 0x4c, 0xc9,
};

uint32_t NODE_ID = 1;
Ip6::Address sClientAddress;
Ip6::Address sServerAddress;

void coap_handler(void* aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
  Coap::Server &server = *(Coap::Server*)aContext;
  Coap::Header header;
  const Ip6::MessageInfo& messageInfo = *(const Ip6::MessageInfo*)aMessageInfo;
  printf("handling coap\n");
  //server.SendEmptyAck(*(Coap::Header*)aHeader, messageInfo);
  Message* message = server.NewMessage(0);
  header.SetDefaultResponseHeader(*(Coap::Header*)aHeader);
  header.SetPayloadMarker();
  message->Append(header.GetBytes(), header.GetLength());
  message->Append("Good", 4);
  server.SendMessage(*message, messageInfo);
  (void)aMessage;
}

void coap_handler_secure(void* aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
  Coap::SecureServer &server = *(Coap::SecureServer*)aContext;
  Coap::Header header;
  const Ip6::MessageInfo& messageInfo = *(const Ip6::MessageInfo*)aMessageInfo;
  printf("handling coap\n");
  //server.SendEmptyAck(*(Coap::Header*)aHeader, messageInfo);
  Message* message = server.NewMessage(0);
  header.SetDefaultResponseHeader(*(Coap::Header*)aHeader);
  header.SetPayloadMarker();
  message->Append(header.GetBytes(), header.GetLength());
  message->Append("Good", 4);
  server.SendMessage(*message, messageInfo);
  (void)aMessage;
}

void response_handler(void* aContext, otCoapHeader *aHeader, otMessage *aMessage, const otMessageInfo *aMessageInfo, ThreadError aError)
{
  Message &message = *(Message*)aMessage;
  char msg[250];
  msg[message.Read(message.GetOffset(), 250, msg)] = 0;
  printf("received %s\n", msg);
  (void)aContext;
  (void)aHeader;
  (void)aMessageInfo;
  (void)aError;
}

void test_request_secure(Coap::SecureClient &cc)
{
  Coap::Header header;
  header.Init(kCoapTypeConfirmable, kCoapRequestPost);
  header.SetToken(Coap::Header::kDefaultTokenLength);
  header.AppendUriPathOptions("o");
  Message *message = cc.NewMessage(header);
  cc.SendMessage(*message, response_handler, NULL);
}

void connect_handler(bool aConnected, void* aContext)
{
  Coap::SecureClient &sc = *(Coap::SecureClient*)aContext;
  if (aConnected) {
    printf("connected\n");
    test_request_secure(sc);
  } else {
    printf("not connected\n");
  }
}


void test_request(Coap::Client &cc)
{
  Coap::Header header;
  header.Init(kCoapTypeConfirmable, kCoapRequestPost);
  header.SetToken(Coap::Header::kDefaultTokenLength);
  header.AppendUriPathOptions("o");
  Message *message = cc.NewMessage(header);
  Ip6::MessageInfo messageInfo;
  messageInfo.SetPeerAddr(sServerAddress);
  messageInfo.SetPeerPort(4433);
  cc.SendMessage(*message, messageInfo, response_handler, NULL);
}

void test_connect(Coap::SecureClient &cc)
{
  Ip6::MessageInfo messageInfo;
  messageInfo.SetPeerAddr(sServerAddress);
  messageInfo.SetPeerPort(4433);
  cc.Connect(messageInfo, connect_handler, &cc);
}

void coaps_connect(void* aContext)
{
  printf("*************************\n");
  Coap::SecureClient &cc = *(Coap::SecureClient*)aContext;
  test_connect(cc);
}

void coap_request(void* aContext)
{
  Coap::Client &cc = *(Coap::Client*)aContext;
  test_request(cc);
}

void test_coaps()
{
  otInstance nc;
  udp_init(&nc);

  Coap::SecureServer ss(nc, 4433);
  Coap::SecureClient cc(nc);
  Coap::Resource test("o", coap_handler_secure, &ss);
  ss.AddResource(test);
  ss.SetPsk(psk, sizeof(psk));
  ss.Start(NULL, NULL, sServerAddress);
  cc.SetPsk(psk, sizeof(psk));
  cc.Start(sClientAddress);

  Thread::Timer timer(nc.mTimerScheduler, coaps_connect, &cc);;
  timer.Start(1000);

  while (!usleep(1000)) {
    otTaskletsProcess(&nc);
    platformAlarmProcess(&nc);
    udp_process(&nc);
  }
}

void test_coap()
{
  otInstance nc;
  udp_init(&nc);

  Coap::Server ss(nc, 4433);
  Coap::Client cc(nc);
  Coap::Resource test("o", coap_handler, &ss);
  ss.AddResource(test);
  ss.Start();
  cc.Start();

  Thread::Timer timer(nc.mTimerScheduler, coap_request, &cc);;
  timer.Start(1000);

  while (!usleep(1000)) {
    otTaskletsProcess(&nc);
    platformAlarmProcess(&nc);
    udp_process(&nc);
  }
}

void platform_init()
{
    platformAlarmInit();
    platformRandomInit();
}

int main()
{
    // test ip4
    //sClientAddress.FromString("ffff:ffff:ffff:ffff:ffff:ffff:7f00:1");
    //sServerAddress.FromString("ffff:ffff:ffff:ffff:ffff:ffff:7f00:1");
    // test ip6
    sServerAddress.FromString("::1");
    platform_init();
    test_coaps();
}
