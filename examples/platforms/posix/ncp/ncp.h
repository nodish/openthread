#ifndef PLATFORM_NCP_H_
#define PLATFORM_NCP_H_

#include <ncp/spinel.h>
#include <openthread/platform/radio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function pointer is called when a MAC frame is received.
 *
 * @param[in]   aInstance   The OpenThread instance structure.
 *
 */
typedef void (*ReceivedHandler)(otInstance *aInstance);

/**
 * This function pointer is called when a transmit request is done.
 *
 * @param[in]   aInstance   The OpenThread instance structure.
 * @param[in]   aError      Error of this request.
 *
 */
typedef void (*TransmittedHandler)(otInstance *aInstance, otError aError);

/**
 * Initialize OpenThread Controller and return the handle.
 */
int ncpOpen(void);

/**
 * Close OpenThread.
 *
 */
void ncpClose(void);

/**
 * Try to receive a data. Must be called when @p sSockFd data available.
 */
void ncpProcess(otRadioFrame *aFrame, bool aRead);

/**
 * Get property
 */
otError ncpGet(spinel_prop_key_t aKey, const char *aFormat, ...);

/**
 * Set property
 */
otError ncpSet(spinel_prop_key_t aKey, const char *aFormat, ...);

/**
 * Insert property
 */
otError ncpInsert(spinel_prop_key_t aKey, const char *aFormat, ...);

/**
 * Remove property
 */
otError ncpRemove(spinel_prop_key_t aKey, const char *aFormat, ...);

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
otError ncpTransmit(const otRadioFrame *aFrame, otRadioFrame *aAckFrame);

/**
 * Enable NCP radio layer.
 *
 * @param[in]   aInstance   The OpenThread instance structure.
 *
 * @retval  OT_ERROR_NONE   Successfully enabled NCP radio layer.
 *
 */
otError ncpEnable(otInstance *aInstance, ReceivedHandler aReceivedHandler, TransmittedHandler aTransmittedHandler);

/**
 * Disable NCP radio layer.
 *
 * @retval  OT_ERROR_NONE   Successfully disabled NCP radio layer.
 *
 */
otError ncpDisable(void);

/**
 * Returns whether there is cached spinel frames.
 *
 * @retval  true    There are spinel frames cached.
 * @retval  false   There are no spinel frame cached.
 *
 */
bool ncpIsFrameCached(void);

/**
 * called when a mac frame is received.
 */
void radioProcessFrame(otInstance *aInstance);

/**
 * called when transmit done
 */
void radioTransmitDone(otInstance *aInstance, otError aError);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // PLATFORM_NCP_H_
