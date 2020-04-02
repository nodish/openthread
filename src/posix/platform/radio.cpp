#include "posix/platform/radio.hpp"

#include <string.h>

#include <openthread/openthread-system.h>
#include <openthread/radio_driver.h>

#include <openthread/platform/radio.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "posix/platform/platform-posix.h"
#include "posix/platform/radio_module.h"

otPosixRadioInstance *  sRadioInstance   = NULL;
otPosixRadioOperations *sRadioOperations = NULL;
otPosixRadioPollFuncs * sRadioPoll       = NULL;

void platformRadioInit(otPlatformConfig *aPlatformConfig)
{
    sRadioInstance   = platformCreateInstance(aUrl, Arguments(aUrl));
    sRadioOperations = sRadioInstance->mDriver->mOperations;
    sRadioPoll       = sRadioInstance->mDriver->mPoll;
}

void platformRadioDeinit(void)
{
    delete sRadioInstance;
    sRadioInstance = NULL;
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    SuccessOrDie(sRadioOperations->GetIeeeEui64(sRadioInstance, aIeeeEui64));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    SuccessOrDie(sRadioOperations->SetPanId(panid));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aAddress->m8[sizeof(addr) - 1 - i];
    }

    SuccessOrDie(sRadioOperations->SetExtendedAddress(addr));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    SuccessOrDie(sRadioOperations->SetShortAddress(aAddress));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    SuccessOrDie(sRadioOperations->SetPromiscuous(aEnable));
    OT_UNUSED_VARIABLE(aInstance);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioInstance->mDriver->mRadioOperations->IsEnabled(sRadioInstance);
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    return sRadioOperations->Enable(aInstance);
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->Disable();
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->Sleep();
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->Receive(aChannel);
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->Transmit(*aFrame);
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return &sRadioOperations->GetTransmitFrame();
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetRssi();
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetRadioCaps();
}

const char *otPlatRadioGetVersionString(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetVersion();
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->IsPromiscuous();
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    SuccessOrDie(sRadioOperations->EnableSrcMatch(aEnable));
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->AddSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    OT_UNUSED_VARIABLE(aInstance);
    return sRadioInstance->AddSrcMatchExtEntry(addr);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->ClearSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->ClearSrcMatchExtEntry(addr);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    SuccessOrDie(sRadioOperations->ClearSrcMatchShortEntries());
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    SuccessOrDie(sRadioOperations->ClearSrcMatchExtEntries());
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->EnergyScan(aScanChannel, aScanDuration);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_ASSERT(aPower != NULL);
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetTransmitPower(*aPower);
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->SetTransmitPower(aPower);
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetCcaEnergyDetectThreshold(*aThreshold);
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->SetCcaEnergyDetectThreshold(aThreshold);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetReceiveSensitivity();
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->SetCoexEnabled(aEnabled);
}

bool otPlatRadioIsCoexEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->IsCoexEnabled();
}

otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    VerifyOrExit(aCoexMetrics != NULL, error = OT_ERROR_INVALID_ARGS);

    error = sRadioOperations->GetCoexMetrics(*aCoexMetrics);

exit:
    return error;
}
#endif

#if OPENTHREAD_POSIX_VIRTUAL_TIME
void ot::Posix::RadioSpinel::Process(const Event &aEvent)
{
    if (mRxFrameBuffer.HasSavedFrame())
    {
        ProcessFrameQueue();
    }

    // The current event can be other event types
    if (aEvent.mEvent == OT_SIM_EVENT_RADIO_SPINEL_WRITE)
    {
        mSpinelInterface.ProcessReadData(aEvent.mData, aEvent.mDataLength);
    }

    if (mRxFrameBuffer.HasSavedFrame())
    {
        ProcessFrameQueue();
    }

    if (mState == kStateTransmitDone)
    {
        mState        = kStateReceive;
        mTxRadioEndUs = UINT64_MAX;

        TransmitDone(mTransmitFrame, (mAckRadioFrame.mLength != 0) ? &mAckRadioFrame : NULL, mTxError);
    }
    else if (mState == kStateTransmitting && platformGetTime() >= mTxRadioEndUs)
    {
        // Frame has been successfully passed to radio, but no `TransmitDone` event received within TX_WAIT_US.
        DieNowWithMessage("radio tx timeout", OT_EXIT_FAILURE);
    }
}

void virtualTimeRadioSpinelProcess(otInstance *aInstance, const struct Event *aEvent)
{
    sRadioOperations->Process(*aEvent);
    OT_UNUSED_VARIABLE(aInstance);
}
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME

#if OPENTHREAD_CONFIG_DIAG_ENABLE
otError otPlatDiagProcess(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    // deliver the platform specific diags commands to radio only ncp.
    OT_UNUSED_VARIABLE(aInstance);
    char  cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE] = {'\0'};
    char *cur                                              = cmd;
    char *end                                              = cmd + sizeof(cmd);

    for (int index = 0; index < argc; index++)
    {
        cur += snprintf(cur, static_cast<size_t>(end - cur), "%s ", argv[index]);
    }

    return sRadioOperations->PlatDiagProcess(cmd, aOutput, aOutputMaxLen);
}

void otPlatDiagModeSet(bool aMode)
{
    SuccessOrExit(sRadioOperations->PlatDiagProcess(aMode ? "start" : "stop", NULL, 0));
    sRadioOperations->SetDiagEnabled(aMode);

exit:
    return;
}

bool otPlatDiagModeGet(void)
{
    return sRadioOperations->IsDiagEnabled();
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
    char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "power %d", aTxPower);
    SuccessOrExit(sRadioOperations->PlatDiagProcess(cmd, NULL, 0));

exit:
    return;
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
    char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "channel %d", aChannel);
    SuccessOrExit(sRadioOperations->PlatDiagProcess(cmd, NULL, 0));

exit:
    return;
}

void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aFrame);
    OT_UNUSED_VARIABLE(aError);
}

void otPlatDiagAlarmCallback(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
}
#endif // OPENTHREAD_CONFIG_DIAG_ENABLE

uint32_t otPlatRadioGetSupportedChannelMask(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetRadioChannelMask(false);
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetRadioChannelMask(true);
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sRadioOperations->GetState();
}
void platformRadioPoll(otSysMainloopContext *aMainloop)
{
    sRadioPoll->Poll(*aMainloop);
}

void platformRadioProcess(const otSysMainloopContext *aMainloop)
{
    sRadioPoll->Process(*aMainloop);
}

