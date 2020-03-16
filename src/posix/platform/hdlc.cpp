#include "posix/platform/hdlc.hpp"

#include "posix/platform/platform-posix.h"

#include "common/code_utils.hpp"
#include "common/debug.hpp"

namespace ot {
namespace Posix {

Hdlc::Hdlc(void)
    : mHdlcDecoder(mRxFrameBuffer, HandleHdlcFrame, this)
{
}

Hdlc::~Hdlc(void)
{
    Deinit();
}

otError Hdlc::Init(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    OT_UNUSED_VARIABLE(aArguments);

    aNext->mDriver->mData->SetCallback(aNext, Hdlc::Input, this);

    return OT_ERROR_NONE;
}

otError Hdlc::Input(void *aInstance, const uint8_t *aBuffer, uint16_t aLength)
{
    static_cast<Hdlc *>(aInstance)->mHdlcDecoder.Decode(aBuffer, aLength);

    return OT_ERROR_NONE;
}

otError Hdlc::Write(const uint8_t *aFrame, uint16_t aLength)
{
    otError                              error = OT_ERROR_NONE;
    ot::Hdlc::FrameBuffer<kMaxFrameSize> encoderBuffer;
    ot::Hdlc::Encoder                    hdlcEncoder(encoderBuffer);

    SuccessOrExit(error = hdlcEncoder.BeginFrame());
    SuccessOrExit(error = hdlcEncoder.Encode(aFrame, aLength));
    SuccessOrExit(error = hdlcEncoder.EndFrame());

    error = mNext->mDriver->mData->Write(mNext, encoderBuffer.GetFrame(), encoderBuffer.GetLength());

exit:
    return error;
}

otError Hdlc::Wait(uint32_t aTimeout)
{
    return mNext->mDriver->mData->Wait(mNext, aTimeout);
}

void Hdlc::HandleHdlcFrame(otError aError)
{
    if (aError == OT_ERROR_NONE)
    {
        otError error = mDataCallback(mDataContext, mRxFrameBuffer.GetFrame(), mRxFrameBuffer.GetLength());

        if (error == OT_ERROR_BUSY)
        {
            mRxFrameBuffer.SaveFrame();
        }
        else
        {
            mRxFrameBuffer.DiscardFrame();
        }
    }
    else
    {
        mRxFrameBuffer.DiscardFrame();
        otLogWarnPlat("Error decoding hdlc frame: %s", otThreadErrorToString(aError));
    }
}

} // namespace Posix
} // namespace ot

static otPosixRadioInstance *Create(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    ot::Posix::Hdlc *driver = new ot::Posix::Hdlc();

    driver->Init(aArguments, aNext);

    return driver;
}

static otError Delete(otPosixRadioInstance *aInstance)
{
    delete static_cast<ot::Posix::Hdlc *>(aInstance);

    return OT_ERROR_NONE;
}

static otError Write(otPosixRadioInstance *aInstance, const uint8_t *aBuffer, uint16_t aLength)
{
    return static_cast<ot::Posix::Hdlc *>(aInstance)->Write(aBuffer, aLength);
}

static otError Wait(otPosixRadioInstance *aInstance, uint32_t aTimeout)
{
    return static_cast<ot::Posix::Hdlc *>(aInstance)->Wait(aTimeout);
}

static void SetCallback(otPosixRadioInstance *aInstance, otPosixDataCallback aCallback, void *aContext)
{
    static_cast<ot::Posix::Hdlc *>(aInstance)->SetDataCallback(aCallback, aContext);
}

otPosixRadioDataFuncs sDataFuncs = {
    .SetCallback = SetCallback,
    .Write       = Write,
    .Wait        = Wait,
};

static otPosixRadioDriver sHdlcDriver = {
    .mNext       = NULL,
    .mName       = "hdlc",
    .Create      = Create,
    .Delete      = Delete,
    .mOperations = NULL,
    .mData       = &sDataFuncs,
    .mPoll       = NULL,
};

#if OPENTHREAD_POSIX_SPINEL_MODULE_ENABLE
extern "C" otError otPosixModuleInit(void)
#else
otError platformModuleInitHdlc(void)
#endif
{
    return otPosixRadioDriverRegister(&sHdlcDriver);
}
