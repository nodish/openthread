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
int ncpOpen(void);

/**
 * Try to receive a data. Must be called when @p sSockFd data available.
 */
void ncpReceive(otInstance *aInstance, otRadioFrame *aFrame);

/**
 * Get property
 */
otError ncpGetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Set property
 */
otError ncpSetProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Insert property
 */
otError ncpInsertProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Remove property
 */
otError ncpRemoveProp(otInstance *aInstance, spinel_prop_key_t aKey, const char* aFormat, ...);

/**
 * Send a packet.
 *
 * If the packet request ack, block until ack is received or timeout. Otherwise, return once data
 * is delivered to OpenThread Controller.
 *
 * @param[in]   aFrame      A pointer to the frame to be transmitted.
 * @param[in]   aAckFrame   A pointer to the frame to receive ACK.
 *
 * @returns The error.
 *
 */
otError ncpSendPacket(otInstance *aInstance, const otRadioFrame *aFrame, otRadioFrame *aAckFrame);

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
