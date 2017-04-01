#ifndef NCP_SPINEL_HPP_
#define NCP_SPINEL_HPP_

#include <ncp/hdlc.hpp>

#include "frame_cache.hpp"
#include "ncp.h"

namespace ot {

class NcpSpinel
{
public:
    NcpSpinel(void);
    void Init(const char *aNcpFile);
    void Deinit(void);
    int GetFd() const { return mSockFd; };
    void Process(otRadioFrame *aFrame, bool aRead);
    void Bind(otInstance *aInstance, ReceivedHandler aReceivedHandler, TransmittedHandler aTransmittedHandler)
    {
        mInstance = aInstance;
        mReceivedHandler = aReceivedHandler;
        mTransmittedHandler = aTransmittedHandler;
    }
    bool IsFrameCached(void) const;

    otError Get(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs);
    otError Set(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs);
    otError Insert(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs);
    otError Remove(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs);
    otError Transmit(const otRadioFrame *aFrame, otRadioFrame *aAckFrame);

private:
    enum {
        kMaxSpinelFrame = 512,  /// Max size in bytes for transfering spinel frames.
        kMaxWaitTime = 1,       ///< Max time to wait for response in milliseconds.
    };

    void Receive(void);
    spinel_tid_t GetNextTid(void);
    void FreeTid(spinel_tid_t tid)
    {
        mCmdTidsInUse &= ~(1 << tid);
    }
    otError RequestV(bool wait, uint32_t command, spinel_prop_key_t key,
                         const char *pack_format, va_list args);
    otError Request(bool wait, uint32_t command, spinel_prop_key_t key,
                        const char *pack_format, ...);
    otError WaitReply(void);
    otError SendCommand(uint32_t command, spinel_prop_key_t key, spinel_tid_t tid, const char *pack_format, va_list args);

    static void HandleHdlcError(void* aContext, otError aError, uint8_t *aBuffer, uint16_t aLength)
    {
        (void)aContext;
        (void)aError;
        (void)aBuffer;
        (void)aLength;
    }

    static void HandleHdlcFrame(void* aContext, uint8_t *aBuffer, uint16_t aLength)
    {
        static_cast<NcpSpinel *>(aContext)->HandleHdlcFrame(aBuffer, aLength);
    }
    void HandleHdlcFrame(const uint8_t *aBuffer, uint16_t aLength);

    otError ParseRawStream(otRadioFrame *aFrame, const uint8_t *aBuffer, uint16_t aLength);
    void ProcessCommand(uint32_t aCommand, const uint8_t* aBuffer, uint16_t aLength);
    void ProcessValueIs(spinel_prop_key_t aKey, const uint8_t* aBuffer, uint16_t aLength);
    void ProcessValueInserted(spinel_prop_key_t key, const uint8_t* value_data_ptr, uint16_t value_data_len);
    void HandleResult(uint32_t command, spinel_prop_key_t key, const uint8_t* data, uint16_t dataLength);
    void HandleTransmitDone(uint32_t Command, spinel_prop_key_t Key, const uint8_t* Data, uint16_t DataLength);
    void ProcessNotification(const uint8_t* aBuffer, uint16_t aLength);
    void ProcessReply(const uint8_t* aBuffer, uint16_t aLength);
    void ProcessCache(void);

    uint16_t     mCmdTidsInUse;
    spinel_tid_t mCmdNextTid;

    spinel_tid_t        mStreamTid;
    spinel_tid_t        mWaitingTid;
    spinel_prop_key_t   mWaitingKey;
    const char*         mFormat;
    va_list             mArgs;
    uint32_t            mExpectedCommand;

    uint8_t         mHdlcBuffer[kMaxSpinelFrame];
    Hdlc::Decoder   mHdlcDecoder;
    Hdlc::Encoder   mHdlcEncoder;
    FrameCache      mFrameCache;

    int     mSockFd;
    bool    mIsReady;
    otError mLastError;

    otRadioFrame* mReceiveFrame;
    otRadioFrame* mAckFrame;

    otInstance         *mInstance;
    ReceivedHandler     mReceivedHandler;
    TransmittedHandler  mTransmittedHandler;
};

} // namespace ot

#endif // NCP_SPINEL_HPP_
