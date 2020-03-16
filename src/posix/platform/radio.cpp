#include "posix/platform/radio.hpp"

#include <string.h>

#include <openthread/openthread-system.h>
#include <openthread/radio_driver.h>

#include <openthread/platform/radio.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "posix/platform/platform-posix.h"

struct otPosixRadioArguments
{
    const char *mDevice;
    char *      mStart;
    char *      mEnd;
};

otPosixRadioInstance *sRadioInstance = NULL;

namespace ot {
namespace Posix {

class Arguments : public otPosixRadioArguments
{
    Arguments(char *aUrl)
    {
        char *device = NULL;

        device = strstr(aUrl, "://");
        VerifyOrDie(device != NULL, OT_EXIT_INVALID_ARGUMENTS);
        device[0] = '\0';
        device += sizeof("://") - 1; // sizeof("://")
        mDevice = device;

        mStart = strstr(device, "?");
        VerifyOrDie(mStart != NULL, OT_EXIT_INVALID_ARGUMENTS);
        mStart[0] = '\0';

        mStart += sizeof("?") - 1; // sizeof("?")

        mEnd = mStart + strlen(mStart);
    }

    const char *GetValue(const char *aName)
    {
        const char * rval  = NULL;
        char *       start = mStart;
        const size_t len   = strlen(aName);

        while (start < mEnd)
        {
            char *last = NULL;

            for (char *cur = strtok(start, "&"); cur != NULL; cur = strtok(NULL, "&"))
            {
                if (!strncmp(aName, cur, len))
                {
                    if (cur[len] == '=')
                    {
                        ExitNow(rval = &cur[len + 1]);
                    }
                    else if (cur[len] == '&' || cur[len] == '\0')
                    {
                        ExitNow(rval = "");
                    }
                }
                last = cur;
            }

            start = last + strlen(last) + 1;
        }

    exit:
        return rval;
    }
};

#if 0

char *   mSpiGpioIntDevice;   ///< Path to the Linux GPIO character device for the `I̅N̅T̅` pin.
char *   mSpiGpioResetDevice; ///< Path to the Linux GPIO character device for the `R̅E̅S̅E̅T̅` pin.
uint8_t  mSpiGpioIntLine;     ///< Line index of the `I̅N̅T̅` pin for the associated GPIO character device.
uint8_t  mSpiGpioResetLine;   ///< Line index of the `R̅E̅S̅E̅T̅` pin for the associated GPIO character device.
uint8_t  mSpiMode;            ///< SPI mode to use (0-3).
uint32_t mSpiSpeed;           ///< SPI speed in hertz.
uint32_t mSpiResetDelay;      ///< The delay after R̅E̅S̅E̅T̅ OT_ASSERTion, in miliseconds.
uint16_t mSpiCsDelay;         ///< The delay after SPI C̅S̅ OT_ASSERTion, in µsec.
uint8_t  mSpiAlignAllowance;  ///< Maximum number of 0xFF bytes to clip from start of MISO frame.
uint8_t  mSpiSmallPacketSize; ///< Smallest SPI packet size we can receive in a single transaction.
#if OPENTHREAD_POSIX_CONFIG_RCP_SPI_ENABLE
{"gpio-int-dev", required_argument, NULL, ARG_SPI_GPIO_INT_DEV},
    {"gpio-int-line", required_argument, NULL, ARG_SPI_GPIO_INT_LINE},
    {"gpio-reset-dev", required_argument, NULL, ARG_SPI_GPIO_RESET_DEV},
    {"gpio-reset-line", required_argument, NULL, ARG_SPI_GPIO_RESET_LINE},
    {"spi-mode", required_argument, NULL, ARG_SPI_MODE}, {"spi-speed", required_argument, NULL, ARG_SPI_SPEED},
    {"spi-cs-delay", required_argument, NULL, ARG_SPI_CS_DELAY},
    {"spi-reset-delay", required_argument, NULL, ARG_SPI_RESET_DELAY},
    {"spi-align-allowance", required_argument, NULL, ARG_SPI_ALIGN_ALLOWANCE},
    {"spi-small-packet", required_argument, NULL, ARG_SPI_SMALL_PACKET},
#endif
#if OPENTHREAD_POSIX_CONFIG_RCP_SPI_ENABLE
    fprintf(aStream,
            "        --gpio-int-dev[=gpio-device-path]\n"
            "                                  Specify a path to the Linux sysfs-exported GPIO device for the\n"
            "                                  `I̅N̅T̅` pin. If not specified, `SPI` interface will fall back to\n"
            "                                  polling, which is inefficient.\n"
            "        --gpio-int-line[=line-offset]\n"
            "                                  The offset index of `I̅N̅T̅` pin for the associated GPIO device.\n"
            "                                  If not specified, `SPI` interface will fall back to polling,\n"
            "                                  which is inefficient.\n"
            "        --gpio-reset-dev[=gpio-device-path]\n"
            "                                  Specify a path to the Linux sysfs-exported GPIO device for the\n"
            "                                  `R̅E̅S̅` pin.\n"
            "        --gpio-reset-line[=line-offset]"
            "                                  The offset index of `R̅E̅S̅` pin for the associated GPIO device.\n"
            "        --spi-mode[=mode]         Specify the SPI mode to use (0-3).\n"
            "        --spi-speed[=hertz]       Specify the SPI speed in hertz.\n"
            "        --spi-cs-delay[=usec]     Specify the delay after C̅S̅ OT_ASSERTion, in µsec.\n"
            "        --spi-reset-delay[=ms]    Specify the delay after R̅E̅S̅E̅T̅ OT_ASSERTion, in milliseconds.\n"
            "        --spi-align-allowance[=n] Specify the maximum number of 0xFF bytes to clip from start of\n"
            "                                  MISO frame. Max value is 16.\n"
            "        --spi-small-packet=[n]    Specify the smallest packet we can receive in a single transaction.\n"
            "                                  (larger packets will require two transactions). Default value is 32.\n");
#endif
#endif

static otPosixRadioDriver *sDriverList = NULL;

static otPosixRadioDriver *FindDriver(const char *aName)
{
    otPosixRadioDriver *rval = NULL;

    for (otPosixRadioDriver *driver = sDriverList; driver != NULL; driver = driver->mNext)
    {
        if (!strcmp(aName, driver->mName))
        {
            rval = driver;
            break;
        }
    }

    return rval;
}

otError otPosixRadioDriverRegister(otPosixRadioDriver *aDriver)
{
    otError rval = OT_ERROR_NONE;

    for (otPosixRadioDriver *driver = sDriverList; driver != NULL; driver = driver->mNext)
    {
        if (driver == aDriver)
        {
            ExitNow(rval = OT_ERROR_ALREADY);
        }
    }

    aDriver->mNext = sDriverList;
    sDriverList    = aDriver;

exit:
    return rval;
}

void LoadDrivers(void)
{
    // Try built-in drivers first
    if (!strcmp(aProto, kDriverSpinel))
    {
        driver = Driver::Create<kDriverSpinel>(aDevice, aArguments, lower);
    }
    else if (!strcmp(aProto, kDriverHdlc))
    {
        driver = Driver::Create<kDriverHdlc>(aDevice, aArguments, lower);
    }
    else if (!strcmp(aProto, kDriverSpi))
    {
        driver = Driver::Create<kDriverSpi>(aDevice, aArguments, lower);
    }
    else if (!strcmp(aProto, kDriverUart))
    {
        driver = Driver::Create<kDriverUart>(aDevice, aArguments, lower);
    }
    else if (!strcmp(aProto, kDriverForkpty))
    {
        driver = Driver::Create<kDriverForkpty>(aDevice, aArguments, lower);
    }
    else
    {
        driver = NULL;
    }

    if (nextProto == NULL)
    {
        sBottomDriver = static_cast<BottomDriver *>(driver);
    }

    return driver;
}

//
//
//
// 1. Load all drivers.
//
// all-in-one
// NewDriver(aRadio, aProto, aDevice, aArguments);
//
// two
// lower = NewDriver(aProto, aDevice, aArguments, aRadio, NULL, NULL, aLowerFuncs);
// top = NewDriver(aProto, aDevice, aArguments, aRadio, aLower, aLowerFuncs);
//    aLowerFuncs.SetUppere(lower, top, top->mInput);
//       lower->mUpperHandle = aUpperHandle
//       lower->mUpperInput = aUpperInput
//
//    top->mLowerOutput = aLowerFuncs->mOutput;
//    top->mLowerWait = aLowerFuncs->mWait;
//    top->mLowerHandle = lower;
//
otPosixRadioInstance *CreateInstance(char *aProto, Arguments &aArguments)
{
    char *                nextProto = strstr(aProto, "+");
    otPosixRadioInstance *instance  = NULL;
    otPosixRadioInstance *next      = NULL;
    otPosixRadioDriver *  driver    = NULL;

    if (nextProto != NULL)
    {
        nextProto[0] = '\0';
        nextProto += sizeof("+") - 1;
        next = CreateInstance(nextProto, aArguments);
    }

    driver = FindDriver(aProto);
    VerifyOrDie(driver == NULL, OT_EXIT_INVALID_ARGUMENTS);

    instance = driver->Create(aArguments, next);
    VerifyOrDie(instance == NULL && instance->mDriver == driver, OT_EXIT_INVALID_ARGUMENTS);

    return instance;
}

otPosixRadioInstance *InitRadioFromUrl(char *aUrl)
{
    return CreateInstance(aUrl, Arguments(aUrl));
}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    SuccessOrDie(sTopDriver->GetIeeeEui64(aIeeeEui64));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    SuccessOrDie(sTopDriver->SetPanId(panid));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, const otExtAddress *aAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aAddress->m8[sizeof(addr) - 1 - i];
    }

    SuccessOrDie(sTopDriver->SetExtendedAddress(addr));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t aAddress)
{
    SuccessOrDie(sTopDriver->SetShortAddress(aAddress));
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    SuccessOrDie(sTopDriver->SetPromiscuous(aEnable));
    OT_UNUSED_VARIABLE(aInstance);
}

void platformRadioInit(otPlatformConfig *aPlatformConfig)
{
    sRadioInstance = InitRadioFromUrl(aPlatformConfig->mRadioUrl);
}

void platformRadioDeinit(void)
{
    delete sTopDriver;
    sTopDriver = NULL;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->IsEnabled();
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    return sTopDriver->Enable(aInstance);
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->Disable();
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->Sleep();
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->Receive(aChannel);
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->Transmit(*aFrame);
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return &sTopDriver->GetTransmitFrame();
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetRssi();
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetRadioCaps();
}

const char *otPlatRadioGetVersionString(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetVersion();
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->IsPromiscuous();
}

void platformRadioPoll(otSysMainloopContext *aMainloop)
{
    sTopDriver->Poll(*aMainloop);
}

void platformRadioProcess(const otSysMainloopContext *aMainloop)
{
    sTopDriver->Process(*aMainloop);
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    SuccessOrDie(sTopDriver->EnableSrcMatch(aEnable));
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->AddSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->AddSrcMatchExtEntry(addr);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, uint16_t aShortAddress)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->ClearSrcMatchShortEntry(aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const otExtAddress *aExtAddress)
{
    otExtAddress addr;

    for (size_t i = 0; i < sizeof(addr); i++)
    {
        addr.m8[i] = aExtAddress->m8[sizeof(addr) - 1 - i];
    }

    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->ClearSrcMatchExtEntry(addr);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    SuccessOrDie(sTopDriver->ClearSrcMatchShortEntries());
    OT_UNUSED_VARIABLE(aInstance);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    SuccessOrDie(sTopDriver->ClearSrcMatchExtEntries());
    OT_UNUSED_VARIABLE(aInstance);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->EnergyScan(aScanChannel, aScanDuration);
}

otError otPlatRadioGetTransmitPower(otInstance *aInstance, int8_t *aPower)
{
    OT_ASSERT(aPower != NULL);
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetTransmitPower(*aPower);
}

otError otPlatRadioSetTransmitPower(otInstance *aInstance, int8_t aPower)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->SetTransmitPower(aPower);
}

otError otPlatRadioGetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t *aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetCcaEnergyDetectThreshold(*aThreshold);
}

otError otPlatRadioSetCcaEnergyDetectThreshold(otInstance *aInstance, int8_t aThreshold)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->SetCcaEnergyDetectThreshold(aThreshold);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetReceiveSensitivity();
}

#if OPENTHREAD_CONFIG_PLATFORM_RADIO_COEX_ENABLE
otError otPlatRadioSetCoexEnabled(otInstance *aInstance, bool aEnabled)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->SetCoexEnabled(aEnabled);
}

bool otPlatRadioIsCoexEnabled(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->IsCoexEnabled();
}

otError otPlatRadioGetCoexMetrics(otInstance *aInstance, otRadioCoexMetrics *aCoexMetrics)
{
    OT_UNUSED_VARIABLE(aInstance);

    otError error = OT_ERROR_NONE;

    VerifyOrExit(aCoexMetrics != NULL, error = OT_ERROR_INVALID_ARGS);

    error = sTopDriver->GetCoexMetrics(*aCoexMetrics);

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
    sTopDriver->Process(*aEvent);
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

    return sTopDriver->PlatDiagProcess(cmd, aOutput, aOutputMaxLen);
}

void otPlatDiagModeSet(bool aMode)
{
    SuccessOrExit(sTopDriver->PlatDiagProcess(aMode ? "start" : "stop", NULL, 0));
    sTopDriver->SetDiagEnabled(aMode);

exit:
    return;
}

bool otPlatDiagModeGet(void)
{
    return sTopDriver->IsDiagEnabled();
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
    char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "power %d", aTxPower);
    SuccessOrExit(sTopDriver->PlatDiagProcess(cmd, NULL, 0));

exit:
    return;
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
    char cmd[OPENTHREAD_CONFIG_DIAG_CMD_LINE_BUFFER_SIZE];

    snprintf(cmd, sizeof(cmd), "channel %d", aChannel);
    SuccessOrExit(sTopDriver->PlatDiagProcess(cmd, NULL, 0));

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
    return sTopDriver->GetRadioChannelMask(false);
}

uint32_t otPlatRadioGetPreferredChannelMask(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetRadioChannelMask(true);
}

otRadioState otPlatRadioGetState(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    return sTopDriver->GetState();
}
#if 0

    aConfig->mPlatformConfig.mSpiSpeed           = OT_PLATFORM_CONFIG_SPI_DEFAULT_SPEED_HZ;
    aConfig->mPlatformConfig.mSpiCsDelay         = OT_PLATFORM_CONFIG_SPI_DEFAULT_CS_DELAY_US;
    aConfig->mPlatformConfig.mSpiResetDelay      = OT_PLATFORM_CONFIG_SPI_DEFAULT_RESET_DELAY_MS;
    aConfig->mPlatformConfig.mSpiAlignAllowance  = OT_PLATFORM_CONFIG_SPI_DEFAULT_ALIGN_ALLOWANCE;
    aConfig->mPlatformConfig.mSpiSmallPacketSize = OT_PLATFORM_CONFIG_SPI_DEFAULT_SMALL_PACKET_SIZE;
    aConfig->mPlatformConfig.mSpiMode            = OT_PLATFORM_CONFIG_SPI_DEFAULT_MODE;
    aConfig->mLogLevel                           = OT_LOG_LEVEL_CRIT;

case ARG_NO_RADIO_RESET:
    aConfig->mPlatformConfig.mResetRadio = false;
    break;
case ARG_RESTORE_NCP_DATASET:
    aConfig->mPlatformConfig.mRestoreDatasetFromNcp = true;
    break;
case ARG_SPI_GPIO_INT_DEV:
    aConfig->mPlatformConfig.mSpiGpioIntDevice = optarg;
    break;
case ARG_SPI_GPIO_INT_LINE:
    aConfig->mPlatformConfig.mSpiGpioIntLine = (uint8_t)atoi(optarg);
    break;
case ARG_SPI_GPIO_RESET_DEV:
    aConfig->mPlatformConfig.mSpiGpioResetDevice = optarg;
    break;
case ARG_SPI_GPIO_RESET_LINE:
    aConfig->mPlatformConfig.mSpiGpioResetLine = (uint8_t)atoi(optarg);
    break;
case ARG_SPI_MODE:
    aConfig->mPlatformConfig.mSpiMode = (uint8_t)atoi(optarg);
    break;
case ARG_SPI_SPEED:
    aConfig->mPlatformConfig.mSpiSpeed = (uint32_t)atoi(optarg);
    break;
case ARG_SPI_CS_DELAY:
    aConfig->mPlatformConfig.mSpiCsDelay = (uint16_t)atoi(optarg);
    break;
case ARG_SPI_RESET_DELAY:
    aConfig->mPlatformConfig.mSpiResetDelay = (uint32_t)atoi(optarg);
    break;
case ARG_SPI_ALIGN_ALLOWANCE:
    aConfig->mPlatformConfig.mSpiAlignAllowance = (uint8_t)atoi(optarg);
    break;
case ARG_SPI_SMALL_PACKET:
    aConfig->mPlatformConfig.mSpiSmallPacketSize = (uint8_t)atoi(optarg);
    break;
}
#endif

} // namespace Posix
} // namespace ot
