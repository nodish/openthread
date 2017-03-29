#ifndef INSTANCE_H_
#define INSTANCE_H_
#include <common/message.hpp>
#include <common/timer.hpp>
#include <common/tasklet.hpp>
#include <crypto/mbedtls.hpp>
#include <meshcop/dtls.hpp>
#include <net/ip6_address.hpp>
#include <net/udp6.hpp>

namespace Thread {
namespace Coap {
class Server;
class SecureServer;
class Client;
};
};

using namespace Thread;

typedef struct otInstance {
public:
  Ip6::Udp mUdp;
  TaskletScheduler mTaskletScheduler;
  TimerScheduler mTimerScheduler;
  Crypto::MbedTls mMbedTls;
  Coap::Server *mCoapServer;
  Coap::Client *mCoapClient;
  Coap::SecureServer *mSecureCoapServer;
  Ip6::Address mMeshLocal16;
  MessagePool mMessagePool;


  Ip6::Address &GetMeshLocal16()
  {
      return mMeshLocal16;
  }

  Coap::Client &GetCoapClient()
  {
      return *mCoapClient;
  }

  Coap::Server &GetCoapServer()
  {
      return *mCoapServer;
  }

  Coap::SecureServer &GetSecureCoapServer()
  {
      return *mSecureCoapServer;
  }

  ThreadError GetLeaderAloc(Ip6::Address &aAddress) const
  {
      const int kAloc16Leader = 0xfc00;
      ThreadError error = kThreadError_None;

      memcpy(&aAddress, &mMeshLocal16, 8);
      aAddress.mFields.m16[4] = HostSwap16(0x0000);
      aAddress.mFields.m16[5] = HostSwap16(0x00ff);
      aAddress.mFields.m16[6] = HostSwap16(0xfe00);
      aAddress.mFields.m16[7] = HostSwap16(kAloc16Leader);

      return error;
  }

  const uint8_t *GetPskc()
  {
      static uint8_t aThreadPSKc[] =
      {
          0xc3, 0xf5, 0x93, 0x68, 0x44, 0x5a, 0x1b, 0x61,
          0x06, 0xbe, 0x42, 0x0a, 0x70, 0x6d, 0x4c, 0xc9,

      };
      return aThreadPSKc;
  }
  otInstance(Ip6::Address &aMeshLocal16):
    mMeshLocal16(aMeshLocal16),
    mMessagePool(this) {}

} otInstance;

static inline otInstance *otInstanceFromUdp(Ip6::Udp *aUdp)
{
    return (otInstance *)CONTAINING_RECORD(aUdp, otInstance, mUdp);
}

static inline otInstance *otInstanceFromTimerScheduler(TimerScheduler *aTimerScheduler)
{
    return (otInstance *)CONTAINING_RECORD(aTimerScheduler, otInstance, mTimerScheduler);
}

static inline otInstance *otInstanceFromTaskletScheduler(TaskletScheduler *aTaskletScheduler)
{
    return (otInstance *)CONTAINING_RECORD(aTaskletScheduler, otInstance, mTaskletScheduler);
}

#endif //INSTANCE_H_
