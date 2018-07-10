#include "hdlc_stream.h"

#include <hdlc.hpp>

using namespace ot;

static void HandleSpinelFrame(void *aContext, uint8_t *aBuffer, uint16_t aLength);
static void HandleHdlcError(void *aContext, otError aError, uint8_t *aBuffer, uint16_t aLength);

const size_t         kMaxSpinelFrame = 2048;
uint8_t              sHdlcBuffer[kMaxSpinelFrame];
static void *        sContext      = NULL;
static FrameHandler  sFrameHandler = NULL;
static Hdlc::Decoder sHdlcDecoder(sHdlcBuffer, sizeof(sHdlcBuffer), HandleSpinelFrame, HandleHdlcError, NULL);
static Hdlc::Encoder sHdlcEncoder;

class UartTxBuffer : public Hdlc::Encoder::BufferWriteIterator
{
public:
    UartTxBuffer(void)
    {
        mWritePointer    = mBuffer;
        mRemainingLength = sizeof(mBuffer);
    }

    uint16_t       GetLength(void) const { return static_cast<uint16_t>(mWritePointer - mBuffer); }
    const uint8_t *GetBuffer(void) const { return mBuffer; }

private:
    enum
    {
        kUartTxBufferSize = 512, // Uart tx buffer size.
    };
    uint8_t mBuffer[kUartTxBufferSize];
};

static void HandleSpinelFrame(void *aContext, uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aContext);
    sFrameHandler(sContext, aBuffer, aLength);
}

static void HandleHdlcError(void *aContext, otError aError, uint8_t *aBuffer, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aContext);
    OT_UNUSED_VARIABLE(aError);
    OT_UNUSED_VARIABLE(aBuffer);
    OT_UNUSED_VARIABLE(aLength);
}

void hdlcDecode(uint8_t *aBuffer, size_t aLength, FrameHandler aFrameHandler, void *aContext)
{
    sContext      = aContext;
    sFrameHandler = aFrameHandler;

    sHdlcDecoder.Decode(aBuffer, aLength);
}

void hdlcEncode(const uint8_t *aBuffer, size_t aLength, HdlcWriter aWriter)
{
    UartTxBuffer txBuffer;

    sHdlcEncoder.Init(txBuffer);
    for (size_t i = 0; i < aLength; i++)
    {
        sHdlcEncoder.Encode(aBuffer[i], txBuffer);
    }
    sHdlcEncoder.Finalize(txBuffer);

    aWriter(txBuffer.GetBuffer(), txBuffer.GetLength());
}
