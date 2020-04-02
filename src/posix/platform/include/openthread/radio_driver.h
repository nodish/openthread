#ifndef OT_POSIX_RADIO_DRIVER_H_
#define OT_POSIX_RADIO_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include <openthread/error.h>
#include <openthread/platform/radio.h>

#include "openthread-system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct otPosixRadioInstance otPosixRadioInstance;

typedef struct otPosixRadioOperations
{
    /**
     * This method gets current state of the radio.
     *
     * @return  Current state of the radio.
     *
     */
    otRadioState (*RadioStateGet)(otPosixRadioInstance *aInstance);

    /**
     * Enable the radio.
     *
     * @param[in]   aInstance   A pointer to the OpenThread instance.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio could not be enabled.
     *
     */
    bool (*RadioEnable)(otPosixRadioInstance *aInstance);

    /**
     * This method switches the radio state from Receive to Sleep.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Sleep.
     * @retval OT_ERROR_BUSY          The radio was transmitting
     * @retval OT_ERROR_INVALID_STATE The radio was disabled
     *
     */
    otError (*RadioSleep)(void);

    /**
     * Disable the radio.
     *
     * @retval  OT_ERROR_NONE               Successfully transitioned to Disabled.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    bool (*RadioDisable)(otPosixRadioInstance *aInstance);

    /**
     * This method switches the radio state from Sleep to Receive.
     *
     * @param[in]  aChannel   The channel to use for receiving.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Receive.
     * @retval OT_ERROR_INVALID_STATE The radio was disabled or transmitting.
     *
     */
    bool (*RadioReceive)(otPosixRadioInstance *aInstance, uint8_t aChannel);

    /**
     * This method switches the radio state from Receive to Transmit.
     *
     * @param[in] aFrame     A reference to the transmitted frame.
     *
     * @retval  OT_ERROR_NONE               Successfully transitioned to Transmit.
     * @retval  OT_ERROR_BUSY               Failed due to another transmission is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     * @retval OT_ERROR_INVALID_STATE The radio was not in the Receive state.
     */
    bool (*RadioTransmit)(otPosixRadioInstance *aInstance, otRadioFrame *aFrame);

    /**
     * This method returns the radio capabilities.
     *
     * @returns The radio capability bit vector.
     *
     */
    otRadioCaps RadioCapsGet(void);

    /**
     * This method returns the radio sw version string.
     *
     * @returns A pointer to the radio version string.
     *
     */
    const char *(*VersionGet)(otPosixRadioInstance *aInstance);

    /**
     * This method gets the status of promiscuous mode.
     *
     * @retval true   Promiscuous mode is enabled.
     * @retval false  Promiscuous mode is disabled.
     *
     */
    bool (*PromiscuousGet)(otPosixRadioInstance *aInstance);

    /**
     * This method sets the status of promiscuous mode.
     *
     * @param[in]   aEnable     Whether to enable or disable promiscuous mode.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*PromiscuousSet)(otPosixRadioInstance *aInstance, bool aEnable);

    /**
     * This method gets the factory-assigned IEEE EUI-64 for this transceiver.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     * @param[out] aIeeeEui64  A pointer to the factory-assigned IEEE EUI-64.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*IeeeEui64Get)(otPosixRadioInstance *aInstance, uint8_t *aIeeeEui64);

    /**
     * This method returns a reference to the transmit buffer.
     *
     * The caller forms the IEEE 802.15.4 frame in this buffer then calls otPlatRadioTransmit() to request
     * transmission.
     *
     * @returns A reference to the transmit buffer.
     *
     */
    otRadioFrame (*TransmitBufferGet)(otPosixRadioInstance *aInstance);

    /**
     * This method gets the radio's transmit power in dBm.
     *
     * @param[out]  aPower    The transmit power in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (*TransmitPowerGet)(otPosixRadioInstance *aInstance, int8_t *aPower);

    /**
     * This method sets the radio's transmit power in dBm.
     *
     * @param[in]   aPower     The transmit power in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (*TransmitPowerSet)(otPosixRadioInstance *aInstance, int8_t aPower);

    /**
     * This method gets the most recent RSSI measurement.
     *
     * @returns The RSSI in dBm when it is valid.  127 when RSSI is invalid.
     */
    int8_t (*RssiGet)(otPosixRadioInstance *aInstance);

    /**
     * This method sets the PAN ID for address filtering.
     *
     * @param[in]   aPanId  The IEEE 802.15.4 PAN ID.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*PanIdSet)(otPosixRadioInstance *aInstance, uint16_t aPanId);

    /**
     * This method sets the Extended Address for address filtering.
     *
     * @param[in] aExtAddress  A pointer to the IEEE 802.15.4 Extended Address stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*ExtendedAddressSet)(otPosixRadioInstance *aInstance, const otExtAddress *aAddress);

    /**
     * This method sets the Short Address for address filtering.
     *
     * @param[in] aShortAddress  The IEEE 802.15.4 Short Address.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*ShortAddressSet)(otPosixRadioInstance *aInstance, const uint16_t aShortAddress);

    /**
     * This method enables or disables source address match feature.
     *
     * @param[in]  aEnable     Enable/disable source address match feature.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*SrcMatchEnable)(otPosixRadioInstance *aInstance, bool aEnable);

    /**
     * Add an extended address to the source address match table.
     *
     * @param[in]  aInstance    The OpenThread instance structure.
     * @param[in]  aExtAddress  The extended address to be added stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Successfully added extended address to the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_BUFS            No available entry in the source match table.
     */
    otError (*SrcMatchExtEntryAdd)(otPosixRadioInstance *aInstance, const otExtAddress *aExtAddress);

    /**
     * Remove an extended address from the source address match table.
     *
     * @param[in]  aInstance    The OpenThread instance structure.
     * @param[in]  aExtAddress  The extended address to be removed stored in little-endian byte order.
     *
     * @retval  OT_ERROR_NONE               Successfully removed the extended address from the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_ADDRESS         The extended address is not in source address match table.
     */
    otError (*SrcMatchExtEntryClear)(otPosixRadioInstance *aInstance, const otExtAddress *aExtAddress);

    /**
     * Clear all the extended/long addresses from source address match table.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*SrcMatchExtEntriesClear)(otPosixRadioInstance *aInstance);

    /**
     * This method adds a short address to the source address match table.
     *
     * @param[in]  aInstance      The OpenThread instance structure.
     * @param[in]  aShortAddress  The short address to be added.
     *
     * @retval  OT_ERROR_NONE               Successfully added short address to the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_BUFS            No available entry in the source match table.
     */
    otError (*SrcMatchShortEntryAdd)(otPosixRadioInstance *aInstance, uint16_t aShortAddress);

    /**
     * This method removes a short address from the source address match table.
     *
     * @param[in]  aInstance      The OpenThread instance structure.
     * @param[in]  aShortAddress  The short address to be removed.
     *
     * @retval  OT_ERROR_NONE               Successfully removed short address from the source match table.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     * @retval  OT_ERROR_NO_ADDRESS         The short address is not in source address match table.
     */
    otError (*SrcMatchShortEntryClear)(otPosixRadioInstance *aInstance, uint16_t aShortAddress);
    /**
     * Clear all short addresses from the source address match table.
     *
     * @param[in]  aInstance   The OpenThread instance structure.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    void (*SrcMatchShortEntriesClear)(otPosixRadioInstance *aInstance);

    /**
     * This method begins the energy scan sequence on the radio.
     *
     * @param[in]  aScanChannel     The channel to perform the energy scan on.
     * @param[in]  aScanDuration    The duration, in milliseconds, for the channel to be scanned.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (*EnergyScan)(otPosixRadioInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration);

    /**
     * This method gets the radio's CCA ED threshold in dBm.
     *
     * @param[out]  aThreshold    The CCA ED threshold in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (*CcaEnergyDetectThresholdGet)(otPosixRadioInstance *aInstance, int8_t *aThreshold);

    /**
     * This method sets the radio's CCA ED threshold in dBm.
     *
     * @param[in]   aThreshold     The CCA ED threshold in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (*CcaEnergyDetectThresholdSet)(otPosixRadioInstance *aInstance, int8_t aThreshold);

    /**
     * This method returns the radio receive sensitivity value.
     *
     * @returns The radio receive sensitivity value in dBm.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    int8_t (*ReceiveSensitivityGet)(otPosixRadioInstance *aInstance);

    /**
     * Enable the radio coex.
     *
     * @param[in] aInstance  The OpenThread instance structure.
     * @param[in] aEnabled   TRUE to enable the radio coex, FALSE otherwise.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio coex could not be enabled.
     *
     */
    otError (*CoexEnable)(otPosixRadioInstance *aInstance, bool aEnable);

    /**
     * Check whether radio coex is enabled or not.
     *
     * @param[in] aInstance  The OpenThread instance structure.
     *
     * @returns TRUE if the radio coex is enabled, FALSE otherwise.
     *
     */
    bool (*CoexEnabled)(otPosixRadioInstance *aInstance);

    /**
     * This method retrieves the radio coexistence metrics.
     *
     * @param[out] aCoexMetrics  A reference to the coexistence metrics structure.
     *
     * @retval OT_ERROR_NONE          Successfully retrieved the coex metrics.
     * @retval OT_ERROR_INVALID_ARGS  @p aCoexMetrics was NULL.
     *
     */
    otError (*CoexMetricsGet)(otPosixRadioInstance *aInstance, otRadioCoexMetrics *aCoexMetrics);

    /**
     * This method processes platform diagnostics commands.
     *
     * @param[in]   aString         A NULL-terminated input string.
     * @param[out]  aOutput         The diagnostics execution result.
     * @param[in]   aOutputMaxLen   The output buffer size.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError (
        *DiagProcess)(otPosixRadioInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);

    /**
     * This method returns the radio channel mask.
     *
     * @param[in]   aPreferred  TRUE to get preferred channel mask, FALSE to get supported channel mask.
     *
     * @returns The radio channel mask according to @aPreferred:
     *   The radio supported channel mask that the device is allowed to be on.
     *   The radio preferred channel mask that the device prefers to form on.
     *
     */
    uint32_t (*ChannelMaskGet)(otPosixRadioInstance *aInstance, bool aPreferred);

} otPosixRadioOperations;

typedef otError (*otPosixDataCallback)(void *aContext, const uint8_t *aBuffer, uint16_t aLength);

typedef struct otPosixRadioDataFuncs
{
    void (*SetCallback)(otPosixRadioInstance *aInstance, otPosixDataCallback aCallback, void *aContext);
    otError (*Write)(otPosixRadioInstance *aInstance, const uint8_t *aBuffer, uint16_t aLength);
    otError (*Wait)(otPosixRadioInstance *aContext, uint32_t aTimeout);
} otPosixRadioDataFuncs;

typedef struct otPosixRadioPollFuncs
{
    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]    aMainloop   A reference to the mainloop context.
     *
     */
    void (*mPoll)(otPosixRadioInstance *aInstance, otSysMainloopContext *aMainloop);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]       aMainloop   A reference to the mainloop context.
     *
     */
    void (*Process)(otPosixRadioInstance *aInstance, const otSysMainloopContext *aMainloop);
} otPosixRadioPollFuncs;

typedef struct otPosixRadioArguments otPosixRadioArguments;

const char *otPosixRadioArgumentsGetPath(otPosixRadioArguments *aArguments);

const char *otPosixRadioArgumentsGetValue(otPosixRadioArguments *aArguments, const char *aName, const char *aLastValue);

typedef struct otPosixRadioDriver
{
    struct otPosixRadioDriver *mNext;

    const char *mName;

    otPosixRadioInstance *(*Create)(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext);
    otError (*Delete)(otPosixRadioInstance *aInstance);

    const otPosixRadioOperations *mOperations;
    const otPosixRadioDataFuncs * mData;
    const otPosixRadioPollFuncs * mPoll;
} otPosixRadioDriver;

struct otPosixRadioInstance
{
    otPosixRadioDriver *mDriver;
};

otError otPosixModuleInit(void);

extern otError otPosixRadioDriverRegister(otPosixRadioDriver *aDriver);

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif // OT_POSIX_RADIO_DRIVER_H_
