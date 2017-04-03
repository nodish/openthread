#ifndef OTC_H_
#define OTC_H_

#include <ncp/spinel.h>
#include "openthread/platform/radio.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Initialize OpenThread Controller and return the handle.
 */
int otcOpen();

/**
 * Try to receive a data. Must be called when \p sSockFd data available.
 */
void otcReceive(otInstance *aInstance);

/**
 * Get property
 */
ThreadError otcGetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Set property
 */
ThreadError otcSetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Insert property
 */
ThreadError otcInsertProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Remove property
 */
ThreadError otcRemoveProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Send a packet.
 *
 * If the packet request ack, block until ack is received or timeout. Otherwise, return once data
 * is delivered to OpenThread Controller.
 *
 * @param aPacket packet to send
 *
 */
ThreadError otcSendPacket(otInstance *aInstance, const struct RadioPacket *pkt);

// radio callbacks

/**
 * called when a mac frame is received.
 */
void radioReceiveFrame(otInstance *aInstance);

/**
 * called when transmit done
 */
void radioTransmitDone(otInstance *aInstance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // OTC_H_
