#ifndef OT_POSIX_PLATFORM_HDLC_HPP_
#define OT_POSIX_PLATFORM_HDLC_HPP_

#include <openthread/radio_driver.h>

#include "lib/hdlc/hdlc.hpp"

namespace ot {
namespace Posix {

/**
 * This class defines an HDLC interface to the Radio Co-processor (RCP)
 *
 */
class Hdlc : public otPosixRadioInstance
{
public:
    /**
     * This constructor initializes the object.
     *
     * @param[in] aCallback     A reference to a `Callback` object.
     * @param[in] aFrameBuffer  A reference to a `RxFrameBuffer` object.
     *
     */
    Hdlc(void);

    /**
     * This destructor deinitializes the object.
     *
     */
    ~Hdlc(void);

    void SetDataCallback(otPosixDataCallback aCallback, void *aContext)
    {
        mDataCallback = aCallback;
        mDataContext  = aContext;
    }

    /**
     * This method initializes the interface to the Radio Co-processor (RCP)
     *
     * @note This method should be called before reading and sending spinel frames to the interface.
     *
     * @param[in]  aPlatformConfig  Platform configuration structure.
     *
     * @retval OT_ERROR_NONE          The interface is initialized successfully
     * @retval OT_ERROR_ALREADY       The interface is already initialized.
     * @retval OT_ERROR_INVALID_ARGS  The UART device or executable cannot be found or failed to open/run.
     *
     */
    otError Init(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext);

    /**
     * This method deinitializes the interface to the RCP.
     *
     */
    void Deinit(void);

    /**
     * This method encodes and sends a spinel frame to Radio Co-processor (RCP) over the socket.
     *
     * This is blocking call, i.e., if the socket is not writable, this method waits for it to become writable for
     * up to `kMaxWaitTime` interval.
     *
     * @param[in] aFrame     A pointer to buffer containing the spinel frame to send.
     * @param[in] aLength    The length (number of bytes) in the frame.
     *
     * @retval OT_ERROR_NONE     Successfully encoded and sent the spinel frame.
     * @retval OT_ERROR_NO_BUFS  Insufficient buffer space available to encode the frame.
     * @retval OT_ERROR_FAILED   Failed to send due to socket not becoming writable within `kMaxWaitTime`.
     *
     */
    otError Write(const uint8_t *aFrame, uint16_t aLength);

    /**
     * This method waits for receiving part or all of spinel frame within specified interval.
     *
     * @param[in]  aTimeout  A reference to the timeout.
     *
     * @retval OT_ERROR_NONE             Part or all of spinel frame is received.
     * @retval OT_ERROR_RESPONSE_TIMEOUT No spinel frame is received within @p aTimeout.
     *
     */
    otError Wait(uint32_t aTimeout);

    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]  aReadFdSet   A reference to the read file descriptors.
     * @param[inout]  aWriteFdSet  A reference to the write file descriptors.
     * @param[inout]  aMaxFd       A reference to the max file descriptor.
     * @param[inout]  aTimeout     A reference to the timeout.
     *
     */
    void UpdateFdSet(fd_set &aReadFdSet, fd_set &aWriteFdSet, int &aMaxFd, struct timeval &aTimeout);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]   aReadFdSet      A reference to the read file descriptors.
     * @param[in]   aWriteFdSet     A reference to the write file descriptors.
     *
     */
    void Process(const fd_set &aReadFdSet, const fd_set &aWriteFdSet);

private:
    /**
     * This method waits for the socket file descriptor associated with the HDLC interface to become writable within
     * `kMaxWaitTime` interval.
     *
     * @retval OT_ERROR_NONE   Socket is writable.
     * @retval OT_ERROR_FAILED Socket did not become writable within `kMaxWaitTime`.
     *
     */
    otError WaitForWritable(void);

    static void HandleHdlcFrame(void *aContext, otError aError);
    void        HandleHdlcFrame(otError aError);

    static otError Input(void *aInstance, const uint8_t *aBuffer, uint16_t aLength);

    enum
    {
        kMaxFrameSize = 2048, ///< Maximum frame size (number of bytes).
    };
    otPosixDataCallback   mDataCallback;
    void *                mDataContext;
    otPosixRadioInstance *mNext;

    /**
     * This type defines a receive frame buffer to store received spinel frame(s).
     *
     * @note The receive frame buffer is an `Hdlc::MultiFrameBuffer` and therefore it is capable of storing multiple
     * frames in a FIFO queue manner.
     *
     */
    ot::Hdlc::MultiFrameBuffer<kMaxFrameSize> mRxFrameBuffer;

    ot::Hdlc::Decoder mHdlcDecoder;
};

} // namespace Posix
} // namespace ot

#endif // OT_POSIX_PLATFORM_HDLC_HPP_
