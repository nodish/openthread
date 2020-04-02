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

#if 0
class TopDriver : public Driver
{
public:
    virtual bool IsPromiscuous(void) const = 0;

    virtual otError SetPromiscuous(bool aEnable) = 0;

    virtual otError SetShortAddress(uint16_t aAddress) = 0;

    virtual otError GetIeeeEui64(uint8_t *aIeeeEui64) = 0;

    virtual otError SetExtendedAddress(const otExtAddress &aAddress) = 0;

    virtual otError SetPanId(uint16_t aPanId) = 0;

    virtual otError GetTransmitPower(int8_t &aPower) = 0;

    virtual otError SetTransmitPower(int8_t aPower) = 0;

    virtual otError GetCcaEnergyDetectThreshold(int8_t &aThreshold) = 0;

    virtual otError SetCcaEnergyDetectThreshold(int8_t aThreshold) = 0;

    virtual const char *GetVersion(void) const = 0;

    virtual otRadioCaps GetRadioCaps(void) const = 0;

    virtual int8_t GetRssi(void) = 0;

    virtual int8_t GetReceiveSensitivity(void) const = 0;

    virtual otRadioState GetState(void) const = 0;

    virtual otError SetCoexEnabled(bool aEnabled) = 0;

    virtual bool IsCoexEnabled(void) = 0;

    virtual otError GetCoexMetrics(otRadioCoexMetrics &aCoexMetrics) = 0;

    virtual otRadioFrame &GetTransmitFrame(void) = 0;

    virtual otError EnableSrcMatch(bool aEnable) = 0;

    virtual otError AddSrcMatchShortEntry(const uint16_t aShortAddress) = 0;

    virtual otError ClearSrcMatchShortEntry(const uint16_t aShortAddress) = 0;

    virtual otError ClearSrcMatchShortEntries(void) = 0;

    virtual otError AddSrcMatchExtEntry(const otExtAddress &aExtAddress) = 0;

    virtual otError ClearSrcMatchExtEntry(const otExtAddress &aExtAddress) = 0;

    virtual otError ClearSrcMatchExtEntries(void) = 0;

    virtual otError EnergyScan(uint8_t aScanChannel, uint16_t aScanDuration) = 0;

    virtual otError Transmit(otRadioFrame &aFrame) = 0;

    virtual otError Receive(uint8_t aChannel) = 0;

    virtual otError Sleep(void) = 0;

    virtual otError Enable(otInstance *aInstance) = 0;

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

    virtual otError PlatDiagProcess(const char *aString, char *aOutput, size_t aOutputMaxLen) = 0;
#endif

    virtual uint32_t GetRadioChannelMask(bool aPreferred) = 0;

    virtual ~TopDriver(void){};
};

#endif
} // namespace PosixApp
} // namespace ot

#endif // POSIX_PLATFORM_RADIO_HPP_
