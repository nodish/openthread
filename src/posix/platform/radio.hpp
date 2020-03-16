#ifndef POSIX_PLATFORM_RADIO_HPP_
#define POSIX_PLATFORM_RADIO_HPP_

#include <stdint.h>

#include <openthread/error.h>
#include <openthread/openthread-system.h>
#include <openthread/platform/radio.h>

extern const char kDriverSpinel[]  = "spinel";
extern const char kDriverHdlc[]    = "hdlc";
extern const char kDriverSpi[]     = "spi";
extern const char kDriverUart[]    = "uart";
extern const char kDriverForkpty[] = "forkpty";

namespace ot {
namespace PosixApp {

struct Radio
{
};

struct TopDriver
{
};

typedef otError (*InputFunc)(void *aContext, const uint8_t *aBuffer, uint16_t aLength);
typedef otError (*OutputFunc)(void *aContext, const uint8_t *aBuffer, uint16_t aLength);
typedef otError (*WaitFunc)(void *aContext, uint32_t aTimeout);
typedef void (*SetUpperInput)(void *aUpperHandle, InputFunc aUpperInput);

struct otRadioDriverConnector
{
    OutputFunc mOutput;
    WaitFunc   mWait;
    InputFunc  mInput;
};

struct otPosixRadioBasic
{
    // Getter/Setter
    // Poll/Process
};

struct otPosixRadioFuncs
{
    OutputFunc    mOutput;
    WaitFunc      mWait;
    SetUpperInput mSetUpperInput;
};

struct BottomDriver
{
    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]    aMainloop   A reference to the mainloop context.
     *
     */
    void (*mPoll)(BottomDriver *aBottom, otSysMainloopContext &aMainloop);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]       aMainloop   A reference to the mainloop context.
     *
     */
    void (*Process)(BottomDriver *aBottom, const otSysMainloopContext &aMainloop);
};

/**
 * This function returns the value of the argument.
 *
 * @retval NULL The named argument is not found.
 * @retval "" The named argument is found but has no value.
 *
 * @returns A pointer to the value.
 *
 */
const char *otPosixArgumentsGetValue(Arguments *aArguments, const char *aName);

typedef otError (*InitModule)(const char *aDevice, Arguments *aArguments);

class Driver
{
public:
    template <const char *kDriverName>
    static Driver *Create(const char *aDevice, Arguments &aArguments, LowerDriver *lower);
};

class UpperDriver
{
public:
    virtual otError Input(const uint8_t *aBuffer, uint16_t aLength) = 0;
};

class LowerDriver
{
public:
    void SetUpper(UpperDriver *aDriver) { mUpper = aDriver; }

    virtual otError Wait(uint32_t aTimeout) = 0;

    virtual otError Output(const uint8_t *aBuffer, uint16_t aLength) = 0;

protected:
    UpperDriver *mUpper;
};

class BottomDriver : public Driver
{
};

class TopDriver : public Driver
{
public:
    /**
     * This method gets the status of promiscuous mode.
     *
     * @retval true   Promiscuous mode is enabled.
     * @retval false  Promiscuous mode is disabled.
     *
     */
    virtual bool IsPromiscuous(void) const = 0;

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
    virtual otError SetPromiscuous(bool aEnable) = 0;

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
    virtual otError SetShortAddress(uint16_t aAddress) = 0;

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
    virtual otError GetIeeeEui64(uint8_t *aIeeeEui64) = 0;

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
    virtual otError SetExtendedAddress(const otExtAddress &aAddress) = 0;

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
    virtual otError SetPanId(uint16_t aPanId) = 0;

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
    virtual otError GetTransmitPower(int8_t &aPower) = 0;

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
    virtual otError SetTransmitPower(int8_t aPower) = 0;

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
    virtual otError GetCcaEnergyDetectThreshold(int8_t &aThreshold) = 0;

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
    virtual otError SetCcaEnergyDetectThreshold(int8_t aThreshold) = 0;

    /**
     * This method returns the radio sw version string.
     *
     * @returns A pointer to the radio version string.
     *
     */
    virtual const char *GetVersion(void) const = 0;

    /**
     * This method returns the radio capabilities.
     *
     * @returns The radio capability bit vector.
     *
     */
    virtual otRadioCaps GetRadioCaps(void) const = 0;

    /**
     * This method gets the most recent RSSI measurement.
     *
     * @returns The RSSI in dBm when it is valid.  127 when RSSI is invalid.
     */
    virtual int8_t GetRssi(void) = 0;

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
    virtual int8_t GetReceiveSensitivity(void) const = 0;

    /**
     * This method gets current state of the radio.
     *
     * @return  Current state of the radio.
     *
     */
    virtual otRadioState GetState(void) const = 0;

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
    virtual otError SetCoexEnabled(bool aEnabled) = 0;

    /**
     * Check whether radio coex is enabled or not.
     *
     * @param[in] aInstance  The OpenThread instance structure.
     *
     * @returns TRUE if the radio coex is enabled, FALSE otherwise.
     *
     */
    virtual bool IsCoexEnabled(void) = 0;

    /**
     * This method retrieves the radio coexistence metrics.
     *
     * @param[out] aCoexMetrics  A reference to the coexistence metrics structure.
     *
     * @retval OT_ERROR_NONE          Successfully retrieved the coex metrics.
     * @retval OT_ERROR_INVALID_ARGS  @p aCoexMetrics was NULL.
     *
     */
    virtual otError GetCoexMetrics(otRadioCoexMetrics &aCoexMetrics) = 0;

    /**
     * This method returns a reference to the transmit buffer.
     *
     * The caller forms the IEEE 802.15.4 frame in this buffer then calls otPlatRadioTransmit() to request
     * transmission.
     *
     * @returns A reference to the transmit buffer.
     *
     */
    virtual otRadioFrame &GetTransmitFrame(void) = 0;

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
    virtual otError EnableSrcMatch(bool aEnable) = 0;

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
    virtual otError AddSrcMatchShortEntry(const uint16_t aShortAddress) = 0;

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
    virtual otError ClearSrcMatchShortEntry(const uint16_t aShortAddress) = 0;

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
    virtual otError ClearSrcMatchShortEntries(void) = 0;

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
    virtual otError AddSrcMatchExtEntry(const otExtAddress &aExtAddress) = 0;

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
    virtual otError ClearSrcMatchExtEntry(const otExtAddress &aExtAddress) = 0;

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
    virtual otError ClearSrcMatchExtEntries(void) = 0;

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
    virtual otError EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration) = 0;

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
    virtual otError Transmit(otRadioFrame &aFrame) = 0;

    /**
     * This method switches the radio state from Sleep to Receive.
     *
     * @param[in]  aChannel   The channel to use for receiving.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Receive.
     * @retval OT_ERROR_INVALID_STATE The radio was disabled or transmitting.
     *
     */
    virtual otError Receive(uint8_t aChannel) = 0;

    /**
     * This method switches the radio state from Receive to Sleep.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Sleep.
     * @retval OT_ERROR_BUSY          The radio was transmitting
     * @retval OT_ERROR_INVALID_STATE The radio was disabled
     *
     */
    virtual otError Sleep(void) = 0;

    /**
     * Enable the radio.
     *
     * @param[in]   aInstance   A pointer to the OpenThread instance.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio could not be enabled.
     *
     */
    virtual otError Enable(otInstance *aInstance) = 0;

    /**
     * Disable the radio.
     *
     * @retval  OT_ERROR_NONE               Successfully transitioned to Disabled.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    virtual otError Disable(void) = 0;

    /**
     * This method checks whether radio is enabled or not.
     *
     * @returns TRUE if the radio is enabled, FALSE otherwise.
     *
     */
    virtual bool IsEnabled(void) const = 0;

#if OPENTHREAD_POSIX_VIRTUAL_TIME
    /**
     * This method performs radio spinel processing in simulation mode.
     *
     * @param[in]   aEvent  A reference to the current received simulation event.
     *
     */
    virtual void Process(const struct Event &aEvent) = 0;

    /**
     * This method updates the @p aTimeout for processing radio spinel in simulation mode.
     *
     * @param[out]   aTimeout    A reference to the current timeout.
     *
     */
    virtual void Update(struct timeval &aTimeout) = 0;
#endif

#if OPENTHREAD_CONFIG_DIAG_ENABLE
    /**
     * This method enables/disables the factory diagnostics mode.
     *
     * @param[in]  aMode  TRUE to enable diagnostics mode, FALSE otherwise.
     *
     */
    virtual void SetDiagEnabled(bool aMode) = 0;

    /**
     * This method indicates whether or not factory diagnostics mode is enabled.
     *
     * @returns TRUE if factory diagnostics mode is enabled, FALSE otherwise.
     *
     */
    virtual bool IsDiagEnabled(void) = 0;

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
    virtual otError PlatDiagProcess(const char *aString, char *aOutput, size_t aOutputMaxLen) = 0;
#endif

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
    virtual uint32_t GetRadioChannelMask(bool aPreferred) = 0;

    virtual ~TopDriver(void){};
};

} // namespace PosixApp
} // namespace ot

#endif // POSIX_PLATFORM_RADIO_HPP_
