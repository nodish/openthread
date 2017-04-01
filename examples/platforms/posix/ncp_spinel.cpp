// OpenThread Controller
#include "platform-posix.h"

#include "ncp_spinel.hpp"

#include <assert.h>
#include <termios.h>
#include <errno.h>
#include <openthread-core-config.h>
#include <pty.h>
#include <syslog.h>

#include <common/logging.hpp>
#include <openthread/platform/alarm-milli.h>

#include "utils/code_utils.h"

#if OPENTHREAD_ENABLE_POSIX_RADIO_SPINEL

static ot::NcpSpinel sNcpSpinel;

namespace ot {

#ifndef SOCKET_UTILS_DEFAULT_SHELL
#define SOCKET_UTILS_DEFAULT_SHELL         "/bin/sh"
#endif

otError SpinelStatusToOtError(spinel_status_t aError)
{
    otError ret;

    switch (aError)
    {
    case SPINEL_STATUS_OK:
        ret = OT_ERROR_NONE;
        break;

    case SPINEL_STATUS_FAILURE:
        ret = OT_ERROR_FAILED;
        break;

    case SPINEL_STATUS_DROPPED:
        ret = OT_ERROR_DROP;
        break;

    case SPINEL_STATUS_NOMEM:
        ret = OT_ERROR_NO_BUFS;
        break;

    case SPINEL_STATUS_BUSY:
        ret = OT_ERROR_BUSY;
        break;

    case SPINEL_STATUS_PARSE_ERROR:
        ret = OT_ERROR_PARSE;
        break;

    case SPINEL_STATUS_INVALID_ARGUMENT:
        ret = OT_ERROR_INVALID_ARGS;
        break;

    case SPINEL_STATUS_UNIMPLEMENTED:
        ret = OT_ERROR_NOT_IMPLEMENTED;
        break;

    case SPINEL_STATUS_INVALID_STATE:
        ret = OT_ERROR_INVALID_STATE;
        break;

    case SPINEL_STATUS_NO_ACK:
        ret = OT_ERROR_NO_ACK;
        break;

    case SPINEL_STATUS_CCA_FAILURE:
        ret = OT_ERROR_CHANNEL_ACCESS_FAILURE;
        break;

    case SPINEL_STATUS_ALREADY:
        ret = OT_ERROR_ALREADY;
        break;

    case SPINEL_STATUS_PROP_NOT_FOUND:
    case SPINEL_STATUS_ITEM_NOT_FOUND:
        ret = OT_ERROR_NOT_FOUND;
        break;

    default:
        if (aError >= SPINEL_STATUS_STACK_NATIVE__BEGIN && aError <= SPINEL_STATUS_STACK_NATIVE__END)
        {
            ret = static_cast<otError>(aError - SPINEL_STATUS_STACK_NATIVE__BEGIN);
        }
        else
        {
            ret = OT_ERROR_FAILED;
        }
        break;
    }

    return ret;
}

int sprint_hex(char *out, const uint8_t* buf, uint16_t len)
{
    static const char hex_str[]= "0123456789abcdef";
    unsigned int  i;

    out[len * 2] = '\0';

    if (!len) return 0;

    for (i = 0; i < len; i++)
    {
        out[i * 2 + 0] = hex_str[(buf[i] >> 4) & 0x0F];
        out[i * 2 + 1] = hex_str[(buf[i]     ) & 0x0F];
    }

    return len * 2;
}

extern "C" const char* dump_hex(const uint8_t* buf, uint16_t len)
{
    static char str[3072];
    sprint_hex(str, buf, len);
    return str;
}

static int open_system_socket_forkpty(const char* aFile, uint32_t aNodeId)
{
	int ret_fd = -1;
	int stderr_copy_fd;
    int pid;
	struct termios tios;
    char command[255];

    sprintf(command, "%s %u", aFile, aNodeId);
	cfmakeraw(&tios);

    memset(&tios, 0, sizeof(tios));
    tios.c_cflag = CS8|HUPCL|CREAD|CLOCAL;
	// Duplicate stderr so that we can hook it back up in the forked process.
	stderr_copy_fd = dup(STDERR_FILENO);
	if (stderr_copy_fd < 0) {
		goto cleanup_and_fail;
	}

	pid = forkpty(&ret_fd, NULL, &tios, NULL);
	if (pid < 0) {
		goto cleanup_and_fail;
	}

	// Check to see if we are the forked process or not.
	if (0 == pid) {
		// We are the forked process.
		const int dtablesize = getdtablesize();
		int i;

#if defined(_LINUX_PRCTL_H)
		prctl(PR_SET_PDEATHSIG, SIGHUP);
#endif

		// Re-instate our original stderr.
		dup2(stderr_copy_fd, STDERR_FILENO);


		// Re-instate our original stderr (clobbered by login_tty)
		dup2(stderr_copy_fd, STDERR_FILENO);

		// Set the shell environment variable if it isn't set already.
		setenv("SHELL", SOCKET_UTILS_DEFAULT_SHELL, 0);

		// Close all file descriptors larger than STDERR_FILENO.
		for (i = (STDERR_FILENO + 1); i < dtablesize; i++) {
			close(i);
		}


		execl(getenv("SHELL"), getenv("SHELL"), "-c", command, NULL);


        perror("failed");
		//_exit(errno);
        assert(false);
	}

	// Clean up our copy of stderr
	close(stderr_copy_fd);

#if HAVE_PTSNAME
	// See http://stackoverflow.com/questions/3486491/
	close(open(ptsname(ret_fd), O_RDWR | O_NOCTTY));
#endif

	//system_socket_table_add_(ret_fd, pid);

	return ret_fd;

cleanup_and_fail:
	{
		int prevErrno = errno;

		close(ret_fd);
		close(stderr_copy_fd);

		errno = prevErrno;
	}
	return -1;
}

class UartTxBuffer : public Hdlc::Encoder::BufferWriteIterator
{
public:
    UartTxBuffer(void){
        mWritePointer = mBuffer;
        mRemainingLength = sizeof(mBuffer);
    }

    uint16_t GetLength(void) const { return static_cast<uint16_t>(mWritePointer - mBuffer); }
    const uint8_t *GetBuffer(void) const { return mBuffer; }

private:
    enum {
        kUartTxBufferSize = 512,  // Uart tx buffer size.
    };
    uint8_t mBuffer[kUartTxBufferSize];
};

NcpSpinel::NcpSpinel(void) :
    mStreamTid(0),
    mWaitingTid(0),
    mHdlcDecoder(mHdlcBuffer, sizeof(mHdlcBuffer), HandleHdlcFrame, HandleHdlcError, this),
    mIsReady(false)
{
}

void NcpSpinel::Init(const char* aNcpFile)
{
    struct stat st;

    if (stat(aNcpFile, &st) == -1) {
        perror("stat");
        assert(false);
        exit(EXIT_FAILURE);
    }

    if (S_ISCHR(st.st_mode))
    {
        mSockFd = open(aNcpFile, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (mSockFd == -1)
        {
            perror("open");
            assert(false);
            exit(EXIT_FAILURE);
        }

        fcntl(mSockFd, F_SETFL, O_NONBLOCK);
        tcflush(mSockFd, TCIOFLUSH);
    }
    else if (S_ISREG(st.st_mode))
    {
        mSockFd = open_system_socket_forkpty(aNcpFile, NODE_ID);
    }

    assert(mSockFd != -1);

    while (!mIsReady)
    {
        fd_set fds;
        int rval;

        FD_ZERO(&fds);
        FD_SET(mSockFd, &fds);
        rval = select(mSockFd + 1, &fds, NULL, NULL, NULL);
        assert(rval > 0);
        Receive();
    }
}

void NcpSpinel::Deinit(void)
{
    if (close(mSockFd))
    {
        perror("close");
        abort();
    }
}

bool NcpSpinel::IsFrameCached(void) const
{
    return !mFrameCache.IsEmpty();
}

void NcpSpinel::HandleHdlcFrame(const uint8_t *aBuffer, uint16_t aLength)
{
    uint8_t header;

    assert(spinel_datatype_unpack(aBuffer, aLength, "C", &header) > 0);
    assert((header & SPINEL_HEADER_FLAG) == SPINEL_HEADER_FLAG);
    assert(SPINEL_HEADER_GET_IID(header) == 0);

    if (SPINEL_HEADER_GET_TID(header) == 0)
    {
        if (mIsReady)
        {
            mFrameCache.Push(aBuffer, aLength);
        }
        else
        {
            ProcessNotification(aBuffer, aLength);
        }
    }
    else
    {
        ProcessReply(aBuffer, aLength);
    }
}

void NcpSpinel::ProcessNotification(const uint8_t* aBuffer, uint16_t aLength)
{
    spinel_prop_key_t key;
    uint8_t* data = NULL;
    spinel_size_t len = 0;
    uint8_t header;
    uint32_t command;
    spinel_ssize_t rval;

    rval = spinel_datatype_unpack(aBuffer, aLength, "CiiD", &header, &command, &key, &data, &len);
    assert(rval > 0);

    assert(SPINEL_HEADER_GET_TID(header) == 0);
    assert(command >= SPINEL_CMD_PROP_VALUE_IS && command <= SPINEL_CMD_PROP_VALUE_REMOVED);

    if (key == SPINEL_PROP_LAST_STATUS)
    {
        assert(command == SPINEL_CMD_PROP_VALUE_IS);
        spinel_status_t status = SPINEL_STATUS_OK;
        rval = spinel_datatype_unpack(data, len, "i", &status);
        assert(rval > 0);

        if ((status >= SPINEL_STATUS_RESET__BEGIN) && (status <= SPINEL_STATUS_RESET__END))
        {
            otLogWarnPlat(mInstance, "NCP reset for %d", status - SPINEL_STATUS_RESET__BEGIN);
            mIsReady = true;
        }
    }
    else
    {
        if (command == SPINEL_CMD_PROP_VALUE_IS)
        {
            ProcessValueIs(key, data, static_cast<uint16_t>(len));
        }
        else if (command == SPINEL_CMD_PROP_VALUE_INSERTED)
        {
            ProcessValueInserted(key, data, static_cast<uint16_t>(len));
        }
        else if (command == SPINEL_CMD_PROP_VALUE_REMOVED)
        {
            // TODO
        }
    }
}

void NcpSpinel::ProcessReply(const uint8_t* aBuffer, uint16_t aLength)
{
    spinel_prop_key_t key;
    uint8_t* data = NULL;
    spinel_size_t len = 0;
    uint8_t header;
    uint32_t command;
    spinel_ssize_t rval;

    rval = spinel_datatype_unpack(aBuffer, aLength, "CiiD", &header, &command, &key, &data, &len);
    assert(rval > 0);
    assert(command >= SPINEL_CMD_PROP_VALUE_IS && command <= SPINEL_CMD_PROP_VALUE_REMOVED);

    if (mWaitingTid == SPINEL_HEADER_GET_TID(header))
    {
        HandleResult(command, key, data, static_cast<uint16_t>(len));
        FreeTid(mWaitingTid);
        mWaitingTid = 0;
    }
    else if (mStreamTid == SPINEL_HEADER_GET_TID(header))
    {
        HandleTransmitDone(command, key, data, static_cast<uint16_t>(len));
        FreeTid(mStreamTid);
        mStreamTid = 0;
    }
    else
    {
        assert(false);
    }
}

void NcpSpinel::HandleResult(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t* aBuffer, uint16_t aLength)
{
    otEXPECT_ACTION(aBuffer != NULL, mLastError = OT_ERROR_ABORT);

    if (aKey == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status;
        spinel_ssize_t length = spinel_datatype_unpack(aBuffer, aLength, "i", &status);

        if (length < 0 || static_cast<uint32_t>(length) > aLength)
        {
            mLastError = OT_ERROR_NO_BUFS;
        }
        else
        {
            mLastError = SpinelStatusToOtError(status);
        }
    }
    else if (aKey == mWaitingKey)
    {
        if (mFormat)
        {
            spinel_ssize_t rval = spinel_datatype_vunpack_in_place(aBuffer, aLength, mFormat, mArgs);
            if (rval < 0 || static_cast<uint16_t>(rval) > aLength)
            {
                mLastError = OT_ERROR_NO_BUFS;
                assert(false);
            }
            else
            {
                mLastError = OT_ERROR_NONE;
            }
        }
        else
        {
            if (aCommand == mExpectedCommand)
            {
                mLastError = OT_ERROR_NONE;
            }
            else
            {
                mLastError = OT_ERROR_INVALID_ARGS;
                assert(false);
            }
        }
    }
    else
    {
        assert(false);
    }

exit:
    return;
}

void NcpSpinel::ProcessValueIs(spinel_prop_key_t aKey, const uint8_t* value_data_ptr, uint16_t value_data_len)
{
    if (aKey == SPINEL_PROP_STREAM_RAW)
    {
        assert(mReceiveFrame);
        assert(OT_ERROR_NONE == ParseRawStream(mReceiveFrame, value_data_ptr, (uint8_t)value_data_len));
        mReceivedHandler(mInstance);
    }
    else if (aKey == SPINEL_PROP_MAC_ENERGY_SCAN_RESULT)
    {
        uint8_t scanChannel;
        int8_t maxRssi;
        spinel_ssize_t ret;

        ret = spinel_datatype_unpack(
            value_data_ptr,
            value_data_len,
            "Cc",
            &scanChannel,
            &maxRssi);

        if (ret > 0)
        {
#if !OPENTHREAD_ENABLE_DIAG
            otPlatRadioEnergyScanDone(aInstance, maxRssi);
#endif
        }
    }
    else if (aKey == SPINEL_PROP_STREAM_DEBUG)
    {
        const char* message = NULL;
        uint32_t length = 0;
        spinel_ssize_t ret;

        ret = spinel_datatype_unpack(
            value_data_ptr,
            value_data_len,
            SPINEL_DATATYPE_DATA_S,
            &message,
            &length);

        if (ret > 0 && message && length <= static_cast<uint32_t>(ret))
        {
            if (strnlen(message, length) != length)
            {
                otLogDebgPlat(mInstance, "NCP DEBUG INFO: corrupted");
            }
            else
            {
                otLogDebgPlat(mInstance, "NCP DEBUG INFO: %s", output);
            }
        }
    }
}

void NcpSpinel::ProcessValueInserted(spinel_prop_key_t aKey, const uint8_t* value_data_ptr, uint16_t value_data_len)
{
    (void)aKey;
    (void)value_data_ptr;
    (void)value_data_len;
}

otError NcpSpinel::ParseRawStream(otRadioFrame *aFrame, const uint8_t *aBuffer, uint16_t aLength)
{
    otError error = OT_ERROR_PARSE;
    uint16_t packetLength = 0;
    spinel_ssize_t rval;
    uint16_t flags = 0;
    int8_t noiseFloor = -128;
    spinel_size_t size = OT_RADIO_FRAME_MAX_SIZE;

    rval = spinel_datatype_unpack(
            aBuffer, aLength,
            SPINEL_DATATYPE_UINT16_S,
            &packetLength);

    otEXPECT(rval > 0);

    if (packetLength > OT_RADIO_FRAME_MAX_SIZE)
    {
        otLogWarnPlat(mInstance, "Mac frame corrupted!");
        otEXIT_NOW();
    }

    aFrame->mLength = static_cast<uint8_t>(packetLength);

    rval = spinel_datatype_unpack_in_place(
            aBuffer, aLength,
            SPINEL_DATATYPE_DATA_WLEN_S
            SPINEL_DATATYPE_INT8_S
            SPINEL_DATATYPE_INT8_S
            SPINEL_DATATYPE_UINT16_S
            SPINEL_DATATYPE_STRUCT_S(   // PHY-data
                SPINEL_DATATYPE_UINT8_S // 802.15.4 channel
                SPINEL_DATATYPE_UINT8_S // 802.15.4 LQI
            ),
            aFrame->mPsdu, &size,
            &aFrame->mRssi, &noiseFloor, &flags,
            &aFrame->mChannel, &aFrame->mLqi);

    otEXPECT(rval > 0);

    error = OT_ERROR_NONE;

exit:
    return error;
}

void NcpSpinel::Receive(void)
{
    uint8_t buf[kMaxSpinelFrame];
    ssize_t rval = read(mSockFd, buf, sizeof(buf));

    if (rval <= 0)
    {
        perror("read");
        abort();
    }

    mHdlcDecoder.Decode(buf, static_cast<uint16_t>(rval));
}

void NcpSpinel::ProcessCache(void)
{
    uint16_t length;
    uint8_t buffer[kMaxSpinelFrame];
    const uint8_t *frame;

    while ((frame = mFrameCache.Peek(buffer, length)) != NULL)
    {
        ProcessNotification(frame, length);
        mFrameCache.Shift();
    }
}

void NcpSpinel::Process(otRadioFrame *aFrame, bool aRead)
{
    mReceiveFrame = aFrame;
    ProcessCache();

    if (aRead)
    {
        Receive();
        ProcessCache();
    }

    mReceiveFrame = NULL;
}

otError NcpSpinel::Get(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mFormat = aFormat;
    mArgs = aArgs;
    error = RequestV(true, SPINEL_CMD_PROP_VALUE_GET, aKey, NULL, NULL);
    mFormat = NULL;

    return error;
}

otError NcpSpinel::Set(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_IS;
    error = RequestV(true, SPINEL_CMD_PROP_VALUE_SET, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::Insert(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_INSERTED;
    error = RequestV(true, SPINEL_CMD_PROP_VALUE_INSERT, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::Remove(spinel_prop_key_t aKey, const char* aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_REMOVED;
    error = RequestV(true, SPINEL_CMD_PROP_VALUE_REMOVE, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::WaitReply(void)
{
    otError error = OT_ERROR_NONE;
    struct timeval started;
    long expire;
    long now;

    gettimeofday(&started, NULL);
    expire = started.tv_usec + kMaxWaitTime * 1000 * 10;
    now = started.tv_usec;

    //printf("%s { mWaitingTid %d\r\n", __func__, mWaitingTid);

    do
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(mSockFd, &fds);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = expire - now;
        int rval = select(mSockFd + 1, &fds, NULL, NULL, &timeout);
        if (rval > 0)
        {
            Receive();
        }
        else if (rval == 0)
        {
            FreeTid(mWaitingTid);
            mWaitingTid = 0;
            //printf("Timeout reply\r\n");
            assert(false);
            otEXIT_NOW(error = OT_ERROR_NO_FRAME_RECEIVED);
        }
        else if (errno != EINTR)
        {
            perror("select");
            assert(false);
            //exit(EXIT_FAILURE);
        }
        {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            now = tv.tv_usec;
            if (tv.tv_sec != started.tv_sec)
            {
                now += 1000 * 1000 * (tv.tv_sec - started.tv_sec);
            }
        }
    }
    while (mWaitingTid && expire > now);
    error = mLastError;

exit:
    //printf("%s } mWaitingTid %d\r\n", __func__, mWaitingTid);
    return error;
}

spinel_tid_t NcpSpinel::GetNextTid(void)
{
    spinel_tid_t tid = 0;
    while (tid == 0)
    {
        if (((1 << mCmdNextTid) & mCmdTidsInUse) == 0)
        {
            tid = mCmdNextTid;
            mCmdNextTid = SPINEL_GET_NEXT_TID(mCmdNextTid);
            mCmdTidsInUse |= (1 << tid);
        }

        if (tid == 0)
        {
            // TODO - Wait for event
        }
    }
    return tid;
}

otError NcpSpinel::Transmit(const otRadioFrame *aFrame, otRadioFrame *aAckFrame)
{
    otError error;

    mAckFrame = aAckFrame;
    error = Request(
            true,
            SPINEL_CMD_PROP_VALUE_SET,
            SPINEL_PROP_STREAM_RAW,
            SPINEL_DATATYPE_DATA_WLEN_S
            SPINEL_DATATYPE_UINT8_S
            SPINEL_DATATYPE_INT8_S,
            aFrame->mPsdu,
            aFrame->mLength,
            aFrame->mChannel,
            aFrame->mRssi);

    return error;
}

otError NcpSpinel::SendCommand(uint32_t aCommand, spinel_prop_key_t aKey, spinel_tid_t tid, const char *aFormat, va_list args)
{
    otError error = OT_ERROR_NONE;
    uint8_t buffer[kMaxSpinelFrame];
    UartTxBuffer txBuffer;
    spinel_ssize_t rval;

    // Pack the header, command and key
    rval = spinel_datatype_pack( buffer, sizeof(buffer), "Cii",
            SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0 | tid, aCommand, aKey);

    if (rval < 0 || static_cast<size_t>(rval) > sizeof(buffer))
    {
        return OT_ERROR_NO_BUFS;
    }

    uint16_t offset = static_cast<uint16_t>(rval);

    // Pack the data (if any)
    if (aFormat)
    {
        rval = spinel_datatype_vpack( buffer + offset, sizeof(buffer) - offset, aFormat, args);
        if (rval < 0 || static_cast<size_t>(rval + offset) > sizeof(buffer))
        {
            return OT_ERROR_NO_BUFS;
        }

        offset += static_cast<uint16_t>(rval);
    }

    mHdlcEncoder.Init(txBuffer);
    for (uint8_t i = 0; i < offset; i++)
    {
        mHdlcEncoder.Encode(buffer[i], txBuffer);
    }
    mHdlcEncoder.Finalize(txBuffer);

    rval = write(mSockFd, txBuffer.GetBuffer(), txBuffer.GetLength());
    assert(rval == txBuffer.GetLength());
    otEXPECT_ACTION(rval == txBuffer.GetLength(), error = OT_ERROR_FAILED);

exit:
    return error;
}

otError NcpSpinel::RequestV(bool aWait, uint32_t command, spinel_prop_key_t aKey,
                                const char *aFormat, va_list args)
{
    otError error = OT_ERROR_NONE;
    spinel_tid_t tid = (aWait ? GetNextTid() : 0);

    error = SendCommand(command, aKey, tid, aFormat, args);
    otEXPECT(error == OT_ERROR_NONE);

    if (aKey == SPINEL_PROP_STREAM_RAW)
    {
        if (mStreamTid)
        {
            FreeTid(mStreamTid);
        }
        mStreamTid = tid;
        otEXIT_NOW();
    }
    else if (aWait)
    {
        mWaitingKey = aKey;
        mWaitingTid = tid;
        error = WaitReply();
        otEXPECT(error == OT_ERROR_NONE);
    }

exit:
    return error;
}

otError NcpSpinel::Request(bool aWait, uint32_t aCommand, spinel_prop_key_t aKey,
                               const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError status = RequestV(aWait, aCommand, aKey, aFormat, args);
    va_end(args);
    return status;
}

void NcpSpinel::HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t* aBuffer, uint16_t aLength)
{
    otError error = OT_ERROR_ABORT;
    bool framePending = false;

    if (aBuffer && aCommand == SPINEL_CMD_PROP_VALUE_IS)
    {
        if (aKey == SPINEL_PROP_LAST_STATUS)
        {
            spinel_status_t status = SPINEL_STATUS_OK;
            spinel_ssize_t packedLength = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT_PACKED_S, &status);
            if (packedLength > 0)
            {
                aBuffer += packedLength;
                aLength -= static_cast<spinel_size_t>(packedLength);
                if (status == SPINEL_STATUS_OK)
                {
                    error = OT_ERROR_NONE;
                    packedLength = spinel_datatype_unpack(
                        aBuffer,
                        aLength,
                        SPINEL_DATATYPE_BOOL_S,
                        &framePending);
                    assert(packedLength > 0);
                    otLogInfoPlat(mInstance, "Frame pending %d", framePending);
                    aBuffer += packedLength;
                    aLength -= static_cast<spinel_size_t>(packedLength);
                    if (mAckFrame)
                    {
                        if (aLength)
                        {
                            ParseRawStream(mAckFrame, aBuffer, static_cast<uint16_t>(aLength));
                        }
                        else
                        {
                            syslog(LOG_CRIT, "No ack frame.");
                            otLogWarnPlat(mInstance, "No ack frame.");
                        }
                    }
                }
                else
                {
                    syslog(LOG_CRIT, "Spinel status: %d.", status);
                    otLogWarnPlat(mInstance, "Spinel status: %d.", status);
                    error = SpinelStatusToOtError(status);
                }
            } else {
                otLogWarnPlat(mInstance, "Spinal parse error!");
            }
        }
    }

    mTransmittedHandler(mInstance, error);
}

} // namespace ot

extern "C" {

int ncpOpen(void)
{
    //test();
    //printf("calling %s\r\n", __func__);
    sNcpSpinel.Init(getenv("NCP_FILE"));
    return sNcpSpinel.GetFd();
}

void ncpClose(void)
{
    //printf("calling %s\r\n", __func__);
    sNcpSpinel.Deinit();
}

void ncpProcess(otRadioFrame *aFrame, bool aRead)
{
    //printf("calling %s\r\n", __func__);
    sNcpSpinel.Process(aFrame, aRead);
}

otError ncpSet(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    //printf("calling %s key=%s\r\n", __func__, spinel_prop_key_to_cstr(key));
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Set(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpInsert(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    //printf("calling %s key=%s\r\n", __func__, spinel_prop_key_to_cstr(key));
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Insert(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpRemove(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    //printf("calling %s key=%s\r\n", __func__, spinel_prop_key_to_cstr(key));
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Remove(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpGet(spinel_prop_key_t aKey, const char* aFormat, ...)
{
    //printf("calling %s key=%s\r\n", __func__, spinel_prop_key_to_cstr(key));
    otError error;
    va_list args;
    va_start(args, aFormat);
    error = sNcpSpinel.Get(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpTransmit(const otRadioFrame *aFrame, otRadioFrame *aAckFrame)
{
    //otLogCritMac(NULL, "calling %s channel=%d", __func__, aFrame->mChannel);
    return sNcpSpinel.Transmit(aFrame, aAckFrame);
}

bool ncpIsFrameCached(void)
{
    return sNcpSpinel.IsFrameCached();
}

otError ncpEnable(otInstance *aInstance, ReceivedHandler aReceivedHandler, TransmittedHandler aTransmittedHandler)
{
    sNcpSpinel.Bind(aInstance, aReceivedHandler, aTransmittedHandler);

    otError error = ncpSet(SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, true);
    otEXPECT(error == OT_ERROR_NONE);

exit:
    return error;
}

otError ncpDisable(void)
{
    sNcpSpinel.Bind(NULL, NULL, NULL);
    return ncpSet(SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, false);
}

}

#endif // OPENTHREAD_ENABLE_POSIX_RADIO_SPINEL
