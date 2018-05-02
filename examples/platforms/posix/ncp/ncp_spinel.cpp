/*
 *  Copyright (c) 2018, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "platform-posix.h"

#include "ncp_spinel.hpp"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>

#include <common/logging.hpp>
#include <openthread/platform/alarm-milli.h>

#include "utils/code_utils.h"

#if OPENTHREAD_ENABLE_POSIX_RADIO_NCP

static ot::NcpSpinel sNcpSpinel;

extern const char *NODE_FILE;
extern const char *NODE_CONFIG;

namespace ot {

#ifndef SOCKET_UTILS_DEFAULT_SHELL
#define SOCKET_UTILS_DEFAULT_SHELL "/bin/sh"
#endif

static otError SpinelStatusToOtError(spinel_status_t aError)
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

static int OpenPty(const char *aFile, const char *aConfig)
{
    int masterFd     = -1;
    int stderrCopyFd = -1;
    int pid;

    {
        struct termios tios;

        memset(&tios, 0, sizeof(tios));
        cfmakeraw(&tios);
        tios.c_cflag = CS8 | HUPCL | CREAD | CLOCAL;
        // Duplicate stderr so that we can hook it back up in the forked process.
        stderrCopyFd = dup(STDERR_FILENO);
        otEXPECT(stderrCopyFd > 0);

        pid = forkpty(&masterFd, NULL, &tios, NULL);
        otEXPECT(pid >= 0);
    }

    // Check to see if we are the forked process or not.
    if (0 == pid)
    {
        char command[255];
        // We are the forked process.
        const int dtablesize = getdtablesize();
        int       i;

        sprintf(command, "%s %s", aFile, aConfig);
        // Re-instate our original stderr.
        dup2(stderrCopyFd, STDERR_FILENO);

        // Set the shell environment variable if it isn't set already.
        setenv("SHELL", SOCKET_UTILS_DEFAULT_SHELL, 0);

        // Close all file descriptors larger than STDERR_FILENO.
        for (i = (STDERR_FILENO + 1); i < dtablesize; i++)
        {
            close(i);
        }

        execl(getenv("SHELL"), getenv("SHELL"), "-c", command, NULL);

        perror("failed");
        assert(false);
        _exit(EXIT_FAILURE);
    }
    else
    {
        int flags = fcntl(masterFd, F_GETFL);
        fcntl(masterFd, F_SETFL, flags | O_NONBLOCK);
    }

exit:
    if (stderrCopyFd != -1)
    {
        int prevErrno = errno;
        close(stderrCopyFd);
        errno = prevErrno;
    }

    return masterFd;
}

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

NcpSpinel::NcpSpinel(void)
    : mStreamTid(0)
    , mWaitingTid(0)
    , mHdlcDecoder(mHdlcBuffer, sizeof(mHdlcBuffer), HandleHdlcFrame, HandleHdlcError, this)
    , mIsReady(false)
{
}

static int OpenUart(const char *aNcpFile, const char *aNcpConfig)
{
    char stty[100];
    sprintf(stty, "stty -F %s %s", aNcpFile, aNcpConfig);
    int rval = system(stty);
    assert(rval == 0);
    int fd = open(aNcpFile, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        perror("open");
        assert(false);
        exit(EXIT_FAILURE);
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void NcpSpinel::Init(const char *aNcpFile, const char *aNcpConfig)
{
    struct stat st;

    if (stat(aNcpFile, &st) == -1)
    {
        perror("stat");
        assert(false);
        exit(EXIT_FAILURE);
    }

    if (S_ISCHR(st.st_mode))
    {
        mSockFd = OpenUart(aNcpFile, aNcpConfig);
    }
    else if (S_ISREG(st.st_mode))
    {
        mSockFd = OpenPty(aNcpFile, aNcpConfig);
    }

    assert(mSockFd != -1);

    SendReset();

    mIsReady = true;
}

void NcpSpinel::Deinit(void)
{
    if (close(mSockFd))
    {
        perror("close");
        abort();
    }
    wait(NULL);
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

void NcpSpinel::ProcessNotification(const uint8_t *aBuffer, uint16_t aLength)
{
    spinel_prop_key_t key;
    uint8_t *         data = NULL;
    spinel_size_t     len  = 0;
    uint8_t           header;
    uint32_t          command;
    spinel_ssize_t    rval;

    rval = spinel_datatype_unpack(aBuffer, aLength, "CiiD", &header, &command, &key, &data, &len);
    assert(rval > 0);

    assert(SPINEL_HEADER_GET_TID(header) == 0);
    assert(command >= SPINEL_CMD_PROP_VALUE_IS && command <= SPINEL_CMD_PROP_VALUE_REMOVED);

    if (key == SPINEL_PROP_LAST_STATUS)
    {
        assert(command == SPINEL_CMD_PROP_VALUE_IS);
        spinel_status_t status = SPINEL_STATUS_OK;
        rval                   = spinel_datatype_unpack(data, len, "i", &status);
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

void NcpSpinel::ProcessReply(const uint8_t *aBuffer, uint16_t aLength)
{
    spinel_prop_key_t key;
    uint8_t *         data = NULL;
    spinel_size_t     len  = 0;
    uint8_t           header;
    uint32_t          command;
    spinel_ssize_t    rval;

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

void NcpSpinel::HandleResult(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    otEXPECT_ACTION(aBuffer != NULL, mLastError = OT_ERROR_ABORT);

    if (aKey == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t status;
        spinel_ssize_t  length = spinel_datatype_unpack(aBuffer, aLength, "i", &status);

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

void NcpSpinel::ProcessValueIs(spinel_prop_key_t aKey, const uint8_t *value_data_ptr, uint16_t value_data_len)
{
    if (aKey == SPINEL_PROP_STREAM_RAW)
    {
        assert(mReceiveFrame);
        assert(OT_ERROR_NONE == ParseRawStream(mReceiveFrame, value_data_ptr, (uint8_t)value_data_len));
        mReceivedHandler(mInstance);
    }
    else if (aKey == SPINEL_PROP_MAC_ENERGY_SCAN_RESULT)
    {
        uint8_t        scanChannel;
        int8_t         maxRssi;
        spinel_ssize_t ret;

        ret = spinel_datatype_unpack(value_data_ptr, value_data_len, "Cc", &scanChannel, &maxRssi);

        if (ret > 0)
        {
#if !OPENTHREAD_ENABLE_DIAG
            otPlatRadioEnergyScanDone(aInstance, maxRssi);
#endif
        }
    }
    else if (aKey == SPINEL_PROP_STREAM_DEBUG)
    {
        const char *   message = NULL;
        uint32_t       length  = 0;
        spinel_ssize_t ret;

        ret = spinel_datatype_unpack(value_data_ptr, value_data_len, SPINEL_DATATYPE_DATA_S, &message, &length);

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

void NcpSpinel::ProcessValueInserted(spinel_prop_key_t aKey, const uint8_t *value_data_ptr, uint16_t value_data_len)
{
    (void)aKey;
    (void)value_data_ptr;
    (void)value_data_len;
}

otError NcpSpinel::ParseRawStream(otRadioFrame *aFrame, const uint8_t *aBuffer, uint16_t aLength)
{
    otError        error        = OT_ERROR_PARSE;
    uint16_t       packetLength = 0;
    spinel_ssize_t rval;
    uint16_t       flags      = 0;
    int8_t         noiseFloor = -128;
    spinel_size_t  size       = OT_RADIO_FRAME_MAX_SIZE;

    rval = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT16_S, &packetLength);

    otEXPECT(rval > 0);

    if (packetLength > OT_RADIO_FRAME_MAX_SIZE)
    {
        otLogWarnPlat(mInstance, "Mac frame corrupted!");
        otEXIT_NOW();
    }

    aFrame->mLength = static_cast<uint8_t>(packetLength);

    rval = spinel_datatype_unpack_in_place(aBuffer, aLength,
                                           SPINEL_DATATYPE_DATA_WLEN_S SPINEL_DATATYPE_INT8_S SPINEL_DATATYPE_INT8_S
                                               SPINEL_DATATYPE_UINT16_S SPINEL_DATATYPE_STRUCT_S( // PHY-data
                                                   SPINEL_DATATYPE_UINT8_S                        // 802.15.4 channel
                                                       SPINEL_DATATYPE_UINT8_S                    // 802.15.4 LQI
                                                   ),
                                           aFrame->mPsdu, &size, &aFrame->mRssi, &noiseFloor, &flags, &aFrame->mChannel,
                                           &aFrame->mLqi);

    otEXPECT(rval > 0);

    error = OT_ERROR_NONE;

exit:
    return error;
}

void NcpSpinel::Receive(void)
{
    uint8_t buf[kMaxSpinelFrame];
    ssize_t rval = read(mSockFd, buf, sizeof(buf));

    if (rval < 0)
    {
        perror("read spinel");
        if (errno != EAGAIN)
        {
            abort();
        }
    }
    if (rval > 0)
    {
        mHdlcDecoder.Decode(buf, static_cast<uint16_t>(rval));
    }
}

void NcpSpinel::ProcessCache(void)
{
    uint16_t       length;
    uint8_t        buffer[kMaxSpinelFrame];
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

otError NcpSpinel::Get(spinel_prop_key_t aKey, const char *aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mFormat = aFormat;
    va_copy(mArgs, aArgs);
    error   = RequestV(true, SPINEL_CMD_PROP_VALUE_GET, aKey, NULL, aArgs);
    mFormat = NULL;

    return error;
}

otError NcpSpinel::Set(spinel_prop_key_t aKey, const char *aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_IS;
    error            = RequestV(true, SPINEL_CMD_PROP_VALUE_SET, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::Insert(spinel_prop_key_t aKey, const char *aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_INSERTED;
    error            = RequestV(true, SPINEL_CMD_PROP_VALUE_INSERT, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::Remove(spinel_prop_key_t aKey, const char *aFormat, va_list aArgs)
{
    otError error;

    assert(mWaitingTid == 0);

    mExpectedCommand = SPINEL_CMD_PROP_VALUE_REMOVED;
    error            = RequestV(true, SPINEL_CMD_PROP_VALUE_REMOVE, aKey, aFormat, aArgs);
    mExpectedCommand = SPINEL_CMD_NOOP;

    return error;
}

otError NcpSpinel::WaitReply(void)
{
    otError        error = OT_ERROR_NONE;
    struct timeval end;
    struct timeval now;
    struct timeval timeout = {kMaxWaitTime / 1000, (kMaxWaitTime % 1000) * 1000};

    gettimeofday(&now, NULL);
    timeradd(&now, &timeout, &end);

    do
    {
        fd_set read_fds;
        fd_set error_fds;

        FD_ZERO(&read_fds);
        FD_ZERO(&error_fds);
        FD_SET(mSockFd, &read_fds);
        FD_SET(mSockFd, &error_fds);
        int rval = select(mSockFd + 1, &read_fds, NULL, &error_fds, &timeout);
        if (rval > 0)
        {
            assert(!FD_ISSET(mSockFd, &error_fds));
            if (FD_ISSET(mSockFd, &read_fds))
            {
                Receive();
            }
            else if (FD_ISSET(mSockFd, &error_fds))
            {
                assert(false);
                exit(EXIT_FAILURE);
            }
            else
            {
                assert(false);
                exit(EXIT_FAILURE);
            }
        }
        else if (rval == 0)
        {
            FreeTid(mWaitingTid);
            mWaitingTid = 0;
            assert(false);
            otEXIT_NOW(error = OT_ERROR_NO_FRAME_RECEIVED);
        }
        else if (errno != EINTR)
        {
            perror("select");
            assert(false);
            exit(EXIT_FAILURE);
        }

        gettimeofday(&now, NULL);
        if (timercmp(&end, &now, >))
        {
            timersub(&end, &now, &timeout);
        }
        else
        {
            break;
        }
    } while (mWaitingTid);

    assert(mWaitingTid == 0);
    error = mLastError;

exit:
    return error;
}

spinel_tid_t NcpSpinel::GetNextTid(void)
{
    spinel_tid_t tid = 0;
    while (tid == 0)
    {
        if (((1 << mCmdNextTid) & mCmdTidsInUse) == 0)
        {
            tid         = mCmdNextTid;
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
    error     = Request(true, SPINEL_CMD_PROP_VALUE_SET, SPINEL_PROP_STREAM_RAW,
                    SPINEL_DATATYPE_DATA_WLEN_S SPINEL_DATATYPE_UINT8_S SPINEL_DATATYPE_INT8_S, aFrame->mPsdu,
                    aFrame->mLength, aFrame->mChannel, aFrame->mRssi);

    return error;
}

otError NcpSpinel::SendReset(void)
{
    otError        error = OT_ERROR_NONE;
    uint8_t        buffer[kMaxSpinelFrame];
    UartTxBuffer   txBuffer;
    spinel_ssize_t rval;

    // Pack the header, command and key
    rval =
        spinel_datatype_pack(buffer, sizeof(buffer), "Ci", SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0, SPINEL_CMD_RESET);

    if (rval < 0 || static_cast<size_t>(rval) > sizeof(buffer))
    {
        return OT_ERROR_NO_BUFS;
    }

    mHdlcEncoder.Init(txBuffer);
    for (int i = 0; i < rval; i++)
    {
        mHdlcEncoder.Encode(buffer[i], txBuffer);
    }
    mHdlcEncoder.Finalize(txBuffer);

    rval = write(mSockFd, txBuffer.GetBuffer(), txBuffer.GetLength());
    assert(rval == txBuffer.GetLength());
    otEXPECT_ACTION(rval == txBuffer.GetLength(), error = OT_ERROR_FAILED);

    sleep(1);
exit:
    return error;
}

otError NcpSpinel::SendCommand(uint32_t          aCommand,
                               spinel_prop_key_t aKey,
                               spinel_tid_t      tid,
                               const char *      aFormat,
                               va_list           args)
{
    otError        error = OT_ERROR_NONE;
    uint8_t        buffer[kMaxSpinelFrame];
    UartTxBuffer   txBuffer;
    spinel_ssize_t rval;

    // Pack the header, command and key
    rval = spinel_datatype_pack(buffer, sizeof(buffer), "Cii", SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0 | tid, aCommand,
                                aKey);

    if (rval < 0 || static_cast<size_t>(rval) > sizeof(buffer))
    {
        return OT_ERROR_NO_BUFS;
    }

    uint16_t offset = static_cast<uint16_t>(rval);

    // Pack the data (if any)
    if (aFormat)
    {
        rval = spinel_datatype_vpack(buffer + offset, sizeof(buffer) - offset, aFormat, args);
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

otError NcpSpinel::RequestV(bool aWait, uint32_t command, spinel_prop_key_t aKey, const char *aFormat, va_list args)
{
    otError      error = OT_ERROR_NONE;
    spinel_tid_t tid   = (aWait ? GetNextTid() : 0);

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
        error       = WaitReply();
        otEXPECT(error == OT_ERROR_NONE);
    }

exit:
    return error;
}

otError NcpSpinel::Request(bool aWait, uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError status = RequestV(aWait, aCommand, aKey, aFormat, args);
    va_end(args);
    return status;
}

void NcpSpinel::HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength)
{
    otError error        = OT_ERROR_ABORT;
    bool    framePending = false;

    if (aBuffer && aCommand == SPINEL_CMD_PROP_VALUE_IS)
    {
        if (aKey == SPINEL_PROP_LAST_STATUS)
        {
            spinel_status_t status = SPINEL_STATUS_OK;
            spinel_ssize_t  packedLength =
                spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_UINT_PACKED_S, &status);
            if (packedLength > 0)
            {
                aBuffer += packedLength;
                aLength -= static_cast<spinel_size_t>(packedLength);
                if (status == SPINEL_STATUS_OK)
                {
                    error        = OT_ERROR_NONE;
                    packedLength = spinel_datatype_unpack(aBuffer, aLength, SPINEL_DATATYPE_BOOL_S, &framePending);
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
            }
            else
            {
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
    otError error;
    sNcpSpinel.Init(NODE_FILE, NODE_CONFIG);
    error = ncpGet(SPINEL_PROP_HWADDR, SPINEL_DATATYPE_UINT64_S, &NODE_ID);
    assert(error == OT_ERROR_NONE);
    return sNcpSpinel.GetFd();
}

void ncpClose(void)
{
    sNcpSpinel.Deinit();
}

void ncpProcess(otRadioFrame *aFrame, bool aRead)
{
    sNcpSpinel.Process(aFrame, aRead);
}

otError ncpSet(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Set(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpInsert(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Insert(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpRemove(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    va_list args;
    va_start(args, aFormat);
    otError error = sNcpSpinel.Remove(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpGet(spinel_prop_key_t aKey, const char *aFormat, ...)
{
    otError error;
    va_list args;
    va_start(args, aFormat);
    error = sNcpSpinel.Get(aKey, aFormat, args);
    va_end(args);
    return error;
}

otError ncpTransmit(const otRadioFrame *aFrame, otRadioFrame *aAckFrame)
{
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

#endif // OPENTHREAD_ENABLE_POSIX_RADIO_NCP
