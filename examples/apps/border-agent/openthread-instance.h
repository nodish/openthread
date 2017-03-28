#ifndef INSTANCE_H_
#define INSTANCE_H_
#include <common/message.hpp>
#include <common/timer.hpp>
#include <common/tasklet.hpp>
#include <crypto/mbedtls.hpp>
#include <meshcop/dtls.hpp>
#include <net/ip6_address.hpp>
#include <net/udp6.hpp>

typedef struct otInstance {
public:
  Thread::Ip6::Udp mUdp;
  Thread::MeshCoP::Dtls mDtls;
  Thread::MeshCoP::Dtls mClientDtls;
  Thread::TaskletScheduler mTaskletScheduler;
  Thread::TimerScheduler mTimerScheduler;
  Thread::MessagePool mMessagePool;
  Thread::Crypto::MbedTls mMbedTls;
  otInstance():
    mDtls(*this),
    mClientDtls(*this),
    mMessagePool(this) {}
} otInstance;

static inline otInstance *otInstanceFromUdp(Thread::Ip6::Udp *aUdp)
{
    return (otInstance *)CONTAINING_RECORD(aUdp, otInstance, mUdp);
}

static inline otInstance *otInstanceFromTimerScheduler(Thread::TimerScheduler *aTimerScheduler)
{
    return (otInstance *)CONTAINING_RECORD(aTimerScheduler, otInstance, mTimerScheduler);
}

static inline otInstance *otInstanceFromTaskletScheduler(Thread::TaskletScheduler *aTaskletScheduler)
{
    return (otInstance *)CONTAINING_RECORD(aTaskletScheduler, otInstance, mTaskletScheduler);
}

#endif //INSTANCE_H_
