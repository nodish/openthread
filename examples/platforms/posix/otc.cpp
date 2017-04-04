// TODO rename to otc.cpp
// OpenThread Controller
#include "platform-posix.h"

#include <termios.h>
#include <util.h>
#include <errno.h>
#include <openthread-core-config.h>

#include <ncp/hdlc.hpp>
#include <ncp/ncp_uart.hpp>
#include <openthread/platform/alarm.h>
#include "otc.h"

#ifdef __cplusplus
extern "C" {
#endif

static int sSockFd;

#ifndef SOCKET_UTILS_DEFAULT_SHELL
#define SOCKET_UTILS_DEFAULT_SHELL         "/bin/sh"
#endif

enum {
        kUartTxBufferSize = 256,  // Uart tx buffer size.
};
                                                                       // one whole (decoded) received frame).
class UartTxBuffer : public Thread::Hdlc::Encoder::BufferWriteIterator
{
public:
    UartTxBuffer(void){
    Clear();
    }

    void           Clear(void) {
    mWritePointer = mBuffer;
    mRemainingLength = sizeof(mBuffer);
    }
    bool           IsEmpty(void) const{
    return mWritePointer == mBuffer;
}
    uint16_t       GetLength(void) const {
    return static_cast<uint16_t>(mWritePointer - mBuffer);
    }
    const uint8_t *GetBuffer(void) const {
    return mBuffer;
    }

private:
    uint8_t        mBuffer[kUartTxBufferSize];
};

uint8_t sOutBuf[512];
void                               *sContext;
Thread::Hdlc::Encoder               sFrameEncoder;

UartTxBuffer sTxBuffer;

int sCmdResetReason;

extern bool sLastTransmitFramePending;
extern bool sLastTransmitError;

bool try_spinel_datatype_unpack(
    const uint8_t *data_in,
    spinel_size_t data_len,
    const char *pack_format,
    ...
    );

spinel_tid_t                    sCmdNextTID;
uint16_t                        sCmdTIDsInUse;

spinel_tid_t                    sStreamTID;
spinel_tid_t                    sWaitingTID;
spinel_prop_key_t               sWaitingKey;
uint32_t                        sExpectedResultCommand;
const char*                     sFormat;
va_list                         sArgs;
ThreadError                     sLastError;
static RadioPacket* sReceiveFrame;

void handleHdlcFrame(void *aContext, uint8_t *aFrame, uint16_t aFrameLength);

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
void handleError(void *aContext, ThreadError aError, uint8_t *aFrame, uint16_t aFrameLength)
{
    (void)aContext;
    (void)aError;
    (void)aFrame;
    (void)aFrameLength;
}

Thread::Hdlc::Decoder sDecoder(sOutBuf, sizeof(sOutBuf), handleHdlcFrame, handleError, NULL);

static int
open_system_socket_forkpty(const char* command)
{
	int ret_fd = -1;
	int stderr_copy_fd;
	pid_t pid;
	struct termios tios = { .c_cflag = CS8|HUPCL|CREAD|CLOCAL };

	cfmakeraw(&tios);

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


		execl(getenv("SHELL"),getenv("SHELL"),"-c",command,NULL);


        perror("failed");
		_exit(errno);
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

int otcOpen()
{
    char *ncpFile = getenv("NCP_FILE");
    sSockFd = open_system_socket_forkpty(ncpFile);
    return sSockFd;
}

ThreadError otcWaitReply(otInstance *aInstance, spinel_prop_key_t key, spinel_tid_t tid)
{
    sWaitingKey = key;
    if (key == SPINEL_PROP_STREAM_RAW)
    {
        sStreamTID = tid;
        return kThreadError_None;
    }

    sWaitingTID = tid;

    uint32_t now = otPlatAlarmGetNow();
    uint32_t expire = now + 10000;
    while (now < expire && sWaitingTID > 0)
    {
        printf("Waiting reply of key %d tid %d\r\n", key, sWaitingTID);
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sSockFd, &read_fds);
        struct timeval timeout;
        timeout.tv_sec = (expire - now) / 1000;
        timeout.tv_usec = (int)(((expire - now) % 1000) * 1000);
        int rval = select(sSockFd + 1, &read_fds, NULL, NULL, &timeout);
        if (rval > 0)
        {
            otcReceive(aInstance, sReceiveFrame);
        }
        else if (rval == 0)
        {
            printf("Timeout reply\r\n");
            return kThreadError_NoFrameReceived;
        }
        else if (errno != EINTR)
        {
            printf("Error reply\r\n");
            perror("select");
            exit(EXIT_FAILURE);
        }
        now = otPlatAlarmGetNow();
    }

    printf("Good reply\r\n");
    return kThreadError_None;
}

void otcReceive(otInstance *aInstance, RadioPacket *aPacket)
{
    uint8_t buf[512];
    ssize_t rval = read(sSockFd, buf, sizeof(buf));
    if (rval < 0)
    {
        perror("read");
        exit(EXIT_FAILURE);
    }
    char hex[512];
    sprint_hex(hex, buf, (uint16_t)rval);
    printf("data received length=%ld\r\n[%s]\r\n", rval, hex);
    sContext = aInstance;
    sReceiveFrame = aPacket;
    sDecoder.Decode(buf, (uint16_t)rval);
}

spinel_tid_t
otcGetNextTID()
{
    spinel_tid_t TID = 0;
    while (TID == 0)
    {
        if (((1 << sCmdNextTID) & sCmdTIDsInUse) == 0)
        {
            TID = sCmdNextTID;
            sCmdNextTID = SPINEL_GET_NEXT_TID(sCmdNextTID);
            sCmdTIDsInUse |= (1 << TID);
        }

        if (TID == 0)
        {
            // TODO - Wait for event
        }
    }
    return TID;
}

ThreadError
SpinelStatusToThreadError(
    spinel_status_t error
    )
{
    ThreadError ret;

    switch (error)
    {
    case SPINEL_STATUS_OK:
        ret = kThreadError_None;
        break;

    case SPINEL_STATUS_FAILURE:
        ret = kThreadError_Failed;
        break;

    case SPINEL_STATUS_DROPPED:
        ret = kThreadError_Drop;
        break;

    case SPINEL_STATUS_NOMEM:
        ret = kThreadError_NoBufs;
        break;

    case SPINEL_STATUS_BUSY:
        ret = kThreadError_Busy;
        break;

    case SPINEL_STATUS_PARSE_ERROR:
        ret = kThreadError_Parse;
        break;

    case SPINEL_STATUS_INVALID_ARGUMENT:
        ret = kThreadError_InvalidArgs;
        break;

    case SPINEL_STATUS_UNIMPLEMENTED:
        ret = kThreadError_NotImplemented;
        break;

    case SPINEL_STATUS_INVALID_STATE:
        ret = kThreadError_InvalidState;
        break;

    case SPINEL_STATUS_NO_ACK:
        ret = kThreadError_NoAck;
        break;

    case SPINEL_STATUS_CCA_FAILURE:
        ret = kThreadError_ChannelAccessFailure;
        break;

    case SPINEL_STATUS_ALREADY:
        ret = kThreadError_Already;
        break;

    case SPINEL_STATUS_ITEM_NOT_FOUND:
        ret = kThreadError_NotFound;
        break;

    default:
        if (error >= SPINEL_STATUS_STACK_NATIVE__BEGIN && error <= SPINEL_STATUS_STACK_NATIVE__END)
        {
            ret = (ThreadError)(error - SPINEL_STATUS_STACK_NATIVE__BEGIN);
        }
        else
        {
            ret = kThreadError_Failed;
        }
        break;
    }

    return ret;
}

ThreadError
otcEncodeAndSendAsync(
    otInstance *aInstance,
    uint32_t command,
    spinel_prop_key_t key,
    spinel_tid_t tid,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status = kThreadError_None;
    uint8_t buffer[512];

    // Pack the header, command and key
    spinel_ssize_t packedLength =
        spinel_datatype_pack(
            buffer,
            sizeof(buffer),
            "Cii",
            SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0 | tid,
            command,
            key);

    if (packedLength < 0 || (uint32_t)packedLength > sizeof(buffer))
    {
        return kThreadError_NoBufs;
    }

    uint16_t offset = (uint16_t)packedLength;

    // Pack the data (if any)
    if (pack_format)
    {
        packedLength =
            spinel_datatype_vpack(
                buffer + offset,
                sizeof(buffer) - offset,
                pack_format,
                args);
        if (packedLength < 0 || (uint32_t)(packedLength + offset) > sizeof(buffer))
        {
            return kThreadError_NoBufs;
        }

        offset += (uint16_t)packedLength;
    }

    sFrameEncoder.Init(sTxBuffer);
    for (uint8_t i = 0; i < offset; i++)
    {
        sFrameEncoder.Encode(buffer[i], sTxBuffer);
    }
    sFrameEncoder.Finalize(sTxBuffer);

    //char data[256];
    //sprint_hex(data, sTxBuffer.GetBuffer(), sTxBuffer.GetLength());
    //printf("spinel sending %u %s\n", sTxBuffer.GetLength(), data);
    ssize_t rval = write(sSockFd, sTxBuffer.GetBuffer(), sTxBuffer.GetLength());
    printf("%s written rval=%ld command=%d key=%d tid=%d\r\n", __func__, rval, command, key, tid);
    sTxBuffer.Clear();

    if (tid && rval > 0)
    {
        status = otcWaitReply(aInstance, key, tid);
    }

    return status;
}

void otcFreeTid(spinel_tid_t tid)
{
    sCmdTIDsInUse &= ~(1 << tid);
}

ThreadError
otcSendAsyncV(
    otInstance *aInstance,
    spinel_tid_t *pTid,
    uint32_t command,
    spinel_prop_key_t key,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status = kThreadError_None;

    if (pTid) *pTid = otcGetNextTID();

    status = otcEncodeAndSendAsync(aInstance, command, key, pTid ? (*pTid) : 0, pack_format, args);

    // Remove the handler entry from the list
    if (status != kThreadError_None)
    {
        // Remove the transaction ID from the 'in use' bit field
        if (pTid) otcFreeTid(*pTid);
    }

    return status;
}

ThreadError
otcSendAsync(
    otInstance *aInstance,
    spinel_tid_t *pTid,
    uint32_t command,
    spinel_prop_key_t key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otcSendAsyncV(aInstance, pTid, command, key, pack_format, args);
    va_end(args);
    return status;
}

void handleStreamRaw(otInstance* aInstance, const uint8_t *aBuf, uint16_t aLength)
{
    uint16_t packetLength = 0;

    if (!(try_spinel_datatype_unpack(
                    aBuf,
                    aLength,
                    SPINEL_DATATYPE_UINT16_S,
                    &packetLength) &&
                packetLength <= kMaxPHYPacketSize &&
                aLength > sizeof(uint16_t) + packetLength))
    {
        printf("Invalid mac frame packet\n");
        return;
    }

    sReceiveFrame->mLength = (uint8_t)packetLength;

    uint8_t offset = 2; // spinel header
    uint8_t length = (uint8_t)(aLength - 2);

    if (packetLength != 0)
    {
        memcpy(sReceiveFrame->mPsdu, aBuf + offset, packetLength);
        offset += sReceiveFrame->mLength;
        length -= sReceiveFrame->mLength;
    }

    uint16_t flags = 0;
    int8_t noiseFloor = -128;
    if (try_spinel_datatype_unpack(
                aBuf + offset,
                length,
                SPINEL_DATATYPE_INT8_S
                SPINEL_DATATYPE_INT8_S
                SPINEL_DATATYPE_UINT16_S,
                &sReceiveFrame->mPower,
                &noiseFloor,
                &flags))
    {
        radioReceiveFrame(aInstance);
    }

}

void
otcValueIs(
    otInstance *aInstance,
    spinel_prop_key_t key,
    const uint8_t* value_data_ptr,
    spinel_size_t value_data_len
    )
{
    if (key == SPINEL_PROP_MAC_ENERGY_SCAN_RESULT)
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
            otPlatRadioEnergyScanDone(aInstance, maxRssi);
        }
    }
    else if (key == SPINEL_PROP_STREAM_RAW)
    {
        if (value_data_len < 256)
        {
            handleStreamRaw(
                aInstance,
                value_data_ptr,
                (uint8_t)value_data_len);
        }
    }
    else if (key == SPINEL_PROP_STREAM_DEBUG)
    {
        const uint8_t* output = NULL;
        uint32_t output_len = 0;
        spinel_ssize_t ret;

        ret = spinel_datatype_unpack(
            value_data_ptr,
            value_data_len,
            SPINEL_DATATYPE_DATA_S,
            &output,
            &output_len);

        if (ret > 0 && output && output_len <= (uint32_t)ret)
        {
            if (strnlen((char*)output, output_len) != output_len)
            {
                printf("DEBUG INFO corrupt\r\n");
            }
            else
            {
                printf("DEBUG INFO %s\r\n", output);
            }
        }
    }

}

void
otcValueInserted(
    otInstance* aInstance,
    spinel_prop_key_t key,
    const uint8_t* value_data_ptr,
    spinel_size_t value_data_len
    )
{
    (void)(aInstance);
    (void)(key);
    (void)(value_data_ptr);
    (void)(value_data_len);
}

void
cmdSendMacFrameComplete(
    otInstance *aInstance,
    uint32_t Command,
    spinel_prop_key_t Key,
    const uint8_t* Data,
    spinel_size_t DataLength
    )
{
    sLastTransmitError = kThreadError_Abort;

    if (Data && Command == SPINEL_CMD_PROP_VALUE_IS)
    {
        if (Key == SPINEL_PROP_LAST_STATUS)
        {
            spinel_status_t spinel_status = SPINEL_STATUS_OK;
            spinel_ssize_t packed_len = spinel_datatype_unpack(Data, DataLength, "i", &spinel_status);
            if (packed_len > 0)
            {
                if (spinel_status == SPINEL_STATUS_OK)
                {
                    sLastTransmitError = kThreadError_None;
                    (void)spinel_datatype_unpack(
                        Data + packed_len,
                        DataLength - (spinel_size_t)packed_len,
                        "b",
                        &sLastTransmitFramePending);
                }
                else
                {
                    printf("status is not ok %d\r\n", spinel_status);
                    sLastTransmitError = SpinelStatusToThreadError(spinel_status);
                }
            }
        }
    }

    radioTransmitDone(aInstance);
}


void
cmdSetPropHandler(
    otInstance *aInstance,
    uint32_t command,
    spinel_prop_key_t key,
    const uint8_t* data,
    spinel_size_t dataLength
    )
{
    if (data == NULL)
    {
        sLastError = kThreadError_Abort;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_IS && key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t spinel_status = SPINEL_STATUS_OK;
        spinel_ssize_t packed_len = spinel_datatype_unpack(data, dataLength, "i", &spinel_status);
        if (packed_len < 0 || (uint32_t)packed_len > dataLength)
        {
            sLastError = kThreadError_NoBufs;
        }
        else
        {
            ThreadError errorCode = SpinelStatusToThreadError(spinel_status);
            sLastError = errorCode;
        }
    }
    else if (command != sExpectedResultCommand)
    {
        sLastError = kThreadError_InvalidArgs;
    }
    else if (key == sWaitingKey)
    {
        sLastError = kThreadError_None;
    }
    else
    {
        sLastError = kThreadError_InvalidArgs;
    }

    (void) aInstance;
}


void
cmdGetPropHandler(
    otInstance *aInstance,
    uint32_t command,
    spinel_prop_key_t key,
    const uint8_t* data,
    spinel_size_t dataLength,
    const char* aFormat,
    va_list aArgs
    )
{
    ThreadError error;

    (void)aInstance;

    if (data == NULL)
    {
        error = kThreadError_Abort;
    }
    else if (command != SPINEL_CMD_PROP_VALUE_IS)
    {
        error = kThreadError_InvalidArgs;
    }
    else if (key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t spinel_status = SPINEL_STATUS_OK;
        spinel_ssize_t packed_len = spinel_datatype_unpack(data, dataLength, "i", &spinel_status);
        if (packed_len < 0 || (uint32_t)packed_len > dataLength)
        {
            error = kThreadError_NoBufs;
        }
        else
        {
            error = SpinelStatusToThreadError(spinel_status);
        }
    }
    else if (key == sWaitingKey)
    {
        spinel_ssize_t packed_len = spinel_datatype_vunpack(data, dataLength, aFormat, aArgs);
        if (packed_len < 0 || (uint32_t)packed_len > dataLength)
        {
            error = kThreadError_NoBufs;
        }
        else
        {
            error = kThreadError_None;
        }

    } else {
        error = kThreadError_InvalidArgs;
    }

    sLastError = error;
}

void
otcProcess(
    otInstance *aInstance,
    uint32_t command,
    const uint8_t* cmd_data_ptr,
    spinel_size_t cmd_data_len
    )
{
    printf("%s processing\r\n", __func__);
    uint8_t header;
    spinel_prop_key_t key;
    uint8_t* value_data_ptr = NULL;
    spinel_size_t value_data_len = 0;

    // Make sure it's an expected command
    if (command < SPINEL_CMD_PROP_VALUE_IS || command > SPINEL_CMD_PROP_VALUE_REMOVED)
    {
        printf("%s processing command=%d\r\n", __func__, command);
        return;
    }

    // Decode the key and data
    if (spinel_datatype_unpack(cmd_data_ptr, cmd_data_len, "CiiD", &header, NULL, &key, &value_data_ptr, &value_data_len) == -1)
    {
        printf("%s processing parse failed\r\n", __func__);
        return;
    }

    // Get the transaction ID
    if (SPINEL_HEADER_GET_TID(header) == 0)
    {
        printf("%s processing notification command=%d key=%d\r\n", __func__, command, key);
        // Handle out of band last status locally
        if (command == SPINEL_CMD_PROP_VALUE_IS && key == SPINEL_PROP_LAST_STATUS)
        {
            // Check if this is a reset
            spinel_status_t status = SPINEL_STATUS_OK;
            spinel_datatype_unpack(value_data_ptr, value_data_len, "i", &status);

            if ((status >= SPINEL_STATUS_RESET__BEGIN) && (status <= SPINEL_STATUS_RESET__END))
            {
                sCmdResetReason = status - SPINEL_STATUS_RESET__BEGIN;
                // TODO - Should this be passed on to Thread or Tunnel logic?
            }
        }
        else
        {
            // If this is a 'Value Is' command, process it for notification of state changes.
            if (command == SPINEL_CMD_PROP_VALUE_IS)
            {
                otcValueIs(aInstance, key, value_data_ptr, value_data_len);
            }
            else if (command == SPINEL_CMD_PROP_VALUE_INSERTED)
            {
                otcValueInserted(aInstance, key, value_data_ptr, value_data_len);
            }
        }
    }
    // If there was a transaction ID, then look for the corresponding command handler
    else
    {
        printf("%s processing tid=%d\r\n", __func__, SPINEL_HEADER_GET_TID(header));
        if (sWaitingTID == SPINEL_HEADER_GET_TID(header))
        {
            if (sExpectedResultCommand > 0)
            {
                cmdSetPropHandler(aInstance, command, key, value_data_ptr, value_data_len);
            }
            else
            {
                cmdGetPropHandler(aInstance, command, key, value_data_ptr, value_data_len, sFormat, sArgs);
            }
            otcFreeTid(sWaitingTID);
            sWaitingTID = 0;
        }
        else if (sStreamTID == SPINEL_HEADER_GET_TID(header))
        {
            cmdSendMacFrameComplete(aInstance, command, key, value_data_ptr, value_data_len);
            otcFreeTid(sStreamTID);
            sStreamTID = 0;
        }
        else
        {
            otcFreeTid(sWaitingTID);
            sWaitingTID = 0;
            printf("missing tid %d\r\n", sWaitingTID);
        }
    }
}

void handleHdlcFrame(
    void* aContext,
    uint8_t *buffer,
    uint16_t bufferLength)
{
    uint8_t header;
    uint32_t command;

    //char hex[256];
    //sprint_hex(hex, buffer, bufferLength);
    //printf("frame received %u %s\n", bufferLength, hex);

    (void)aContext;

    // Unpack the header from the buffer
    if (spinel_datatype_unpack(buffer, bufferLength, "Ci", &header, &command) <= 0)
    {
        return;
    }

    // Validate the header
    if ((header & SPINEL_HEADER_FLAG) != SPINEL_HEADER_FLAG)
    {
        return;
    }

    // We only support IID zero for now
    if (SPINEL_HEADER_GET_IID(header) != 0)
    {
        return;
    }

    // Process the received command
    otcProcess((otInstance*)sContext, command, buffer, bufferLength);
}


bool
try_spinel_datatype_unpack(
    const uint8_t *data_in,
    spinel_size_t data_len,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    spinel_ssize_t packed_len = spinel_datatype_vunpack(data_in, data_len, pack_format, args);
    va_end(args);

    return !(packed_len < 0 || (spinel_size_t)packed_len > data_len);
}

ThreadError
otcSendPacket(otInstance *aInstance, const struct RadioPacket *aPacket)
{
    const int IEEE802154_ACK_REQUEST = (1 << 5);
    bool wait = (aPacket->mPsdu[0] & IEEE802154_ACK_REQUEST);
    printf("%s wait=%d\r\n", __func__, wait);
    char hex[512];
    sprint_hex(hex, aPacket->mPsdu, aPacket->mLength);
    printf("%s length=%u packet=[%s]\r\n", __func__, aPacket->mLength, hex);
    spinel_tid_t tid;
    ThreadError error = otcSendAsync(
            aInstance,
            (wait ? &tid : NULL),
            SPINEL_CMD_PROP_VALUE_SET,
            SPINEL_PROP_STREAM_RAW,
            SPINEL_DATATYPE_DATA_WLEN_S
            SPINEL_DATATYPE_UINT8_S
            SPINEL_DATATYPE_INT8_S,
            aPacket->mPsdu,
            (uint32_t)aPacket->mLength,
            aPacket->mChannel,
            aPacket->mPower);

    return error;
}


ThreadError
otcSetPropV(
    otInstance *aInstance,
    uint32_t command,
    spinel_prop_key_t key,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status;
    spinel_tid_t tid;

    if (command == SPINEL_CMD_PROP_VALUE_SET)
    {
        sExpectedResultCommand = SPINEL_CMD_PROP_VALUE_IS;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_INSERT)
    {
        sExpectedResultCommand = SPINEL_CMD_PROP_VALUE_INSERTED;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_REMOVE)
    {
        sExpectedResultCommand = SPINEL_CMD_PROP_VALUE_REMOVED;
    }
    else
    {
        ExitNow(status = kThreadError_Parse);
    }

    // Send the request transaction
    status = otcSendAsyncV(
            aInstance,
            &tid,
            command,
            key,
            pack_format,
            args);

    sExpectedResultCommand = SPINEL_CMD_NOOP;
exit:

    return status;
}

ThreadError
otcSetProp(
    otInstance *aInstance,
    spinel_prop_key_t key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otcSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_SET, key, pack_format, args);
    va_end(args);
    return status;
}

ThreadError
otcInsertProp(
    otInstance *aInstance,
    spinel_prop_key_t key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otcSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_INSERT, key, pack_format, args);
    va_end(args);
    return status;
}

ThreadError
otcRemoveProp(
    otInstance *aInstance,
    spinel_prop_key_t key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otcSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_REMOVE, key, pack_format, args);
    va_end(args);
    return status;
}


ThreadError
otcGetProp(
    otInstance *aInstance,
    spinel_prop_key_t key,
    const char* aFormat,
    ...
    )
{
    ThreadError status;
    spinel_tid_t tid;
    va_start(sArgs, aFormat);
    sFormat = aFormat;
    // Send the request transaction
    status = otcSendAsyncV(
            aInstance,
            &tid,
            SPINEL_CMD_PROP_VALUE_GET,
            key,
            NULL,
            NULL);

    va_end(sArgs);
    return status;
}


#ifdef __cplusplus
}  // extern "C"
#endif

