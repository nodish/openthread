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
        kUartTxBufferSize = OPENTHREAD_CONFIG_NCP_UART_TX_CHUNK_SIZE,  // Uart tx buffer size.
        kRxBufferSize = OPENTHREAD_CONFIG_NCP_UART_RX_BUFFER_SIZE,     // Rx buffer size (should be large enough to fit
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

uint8_t sOutBuf[128];
void                               *sContext;
Thread::Hdlc::Encoder               sFrameEncoder;
MacFrameHandler                     sMacFrameHandler;
MacFrameDoneHandler                 sMacFrameDoneHandler;

UartTxBuffer sTxBuffer;

int sCmdResetReason;

extern bool sLastTransmitFramePending;
extern bool sLastTransmitError;

typedef struct SpinelCmdHandlerEntry
{
    SpinelCmdHandlerEntry  *mNext;
    SpinelCmdHandler        Handler;
    void*                   Context;
    spinel_tid_t            TransactionId;
} SpinelCmdHandlerEntry;

typedef struct _SpinelSetPropContext
{
    uint32_t                Expire;
    uint32_t                ExpectedResultCommand;
    spinel_prop_key_t       key;
    ThreadError             Status;
} SpinelSetPropContext;

typedef struct _SpinelGetPropContext
{
    uint32_t                Expire;
    spinel_prop_key_t       key;
    ThreadError             Status;
} SpinelGetPropContext;

SpinelCmdHandlerEntry sCmdHandlerEntry;
SpinelSetPropContext sSetPropContext;
SpinelGetPropContext sGetPropContext;

SpinelCmdHandlerEntry          *sCmdHandlers = NULL;
spinel_tid_t                    sCmdNextTID;
uint16_t                        sCmdTIDsInUse;

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

int otcOpen(MacFrameHandler aMacFrameHandler, MacFrameDoneHandler aMacFrameDoneHandler)
{
    char *ncpFile = getenv("NCP_FILE");
    sSockFd = open_system_socket_forkpty(ncpFile);
    sMacFrameHandler = aMacFrameHandler;
    sMacFrameDoneHandler = aMacFrameDoneHandler;
    return sSockFd;
}

ThreadError otcWaitReply(otInstance *aInstance)
{
    while (sCmdHandlers != NULL)
    {
        printf("Waiting reply\r\n");
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sSockFd, &read_fds);
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        int rval = select(sSockFd + 1, &read_fds, NULL, NULL, &timeout);
        if (rval > 0)
        {
            otcReceive(aInstance);
        }
        else if (rval == 0)
        {
            printf("Timeout reply\r\n");
            return kThreadError_NoFrameReceived;
        }
        else if ((rval < 0) && (errno != EINTR))
        {
            printf("Error reply\r\n");
            perror("select");
            exit(EXIT_FAILURE);
        }
    }

    printf("Good reply\r\n");
    return kThreadError_None;
}

void otcReceive(otInstance *aInstance)
{
    uint8_t buf[255];
    ssize_t rval = read(sSockFd, buf, sizeof(buf));
    if (rval < 0)
    {
        perror("read");
        exit(EXIT_FAILURE);
    }
    //new(&sDecoder) Thread::Hdlc::Decoder(sOutBuf, sizeof(sOutBuf), handleHdlcFrame, handleError, aInstance);
    sContext = aInstance;
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


void
otcAddCmdHandler(
    SpinelCmdHandlerEntry *pEntry
    )
{
    // TODO cancel expired

    // Get the next transaction ID. This call will block if there are
    // none currently available.
    pEntry->TransactionId = otcGetNextTID();

    // Add to the handlers list
    pEntry->mNext = sCmdHandlers;

    sCmdHandlers = pEntry;
}

SpinelCmdHandlerEntry* otcRemoveCmdHandler(spinel_tid_t tid, bool error)
{
    SpinelCmdHandlerEntry* pEntry = sCmdHandlers;
    SpinelCmdHandlerEntry* Prev = NULL;

    while (pEntry != NULL)
    {
        if (tid == pEntry->TransactionId)
        {
            // Remove the transaction ID from the 'in use' bit field
            sCmdTIDsInUse &= ~(1 << pEntry->TransactionId);

            break;
        }
        Prev = pEntry;
        pEntry = pEntry->mNext;
    }

    if (pEntry)
    {
        // Call the handler function
        if (error)
        {
            pEntry->Handler(NULL, pEntry->Context, 0, (spinel_prop_key_t)0, NULL, 0);
        }

        if (Prev)
        {
            Prev->mNext = pEntry->mNext;
        }
        else
        {
            sCmdHandlers = pEntry->mNext;
        }
    }

    return pEntry;
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
    uint8_t buffer[256];

    //printf("encode and send\n");
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
    write(sSockFd, sTxBuffer.GetBuffer(), sTxBuffer.GetLength());
    sTxBuffer.Clear();

    if (tid > 0)
    {
        status = otcWaitReply(aInstance);
    }

    return status;
}

ThreadError
otcSendAsyncV(
    otInstance *aInstance,
    SpinelCmdHandler Handler,
    void* HandlerContext,
    spinel_tid_t *pTid,
    uint32_t command,
    spinel_prop_key_t key,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status = kThreadError_None;
    SpinelCmdHandlerEntry *pEntry = NULL;

    if (pTid) *pTid = 0;

    // Create the handler entry and add it to the list
    if (Handler)
    {
        pEntry = new SpinelCmdHandlerEntry;
        if (pEntry == NULL)
        {
            status = kThreadError_NoBufs;
            goto exit;
        }

        pEntry->Handler = Handler;
        pEntry->Context = HandlerContext;

        otcAddCmdHandler(pEntry);

        if (pTid) *pTid = pEntry->TransactionId;
    }

    status = otcEncodeAndSendAsync(aInstance, command, key, pEntry ? pEntry->TransactionId : 0, pack_format, args);

    // Remove the handler entry from the list
    if (status != kThreadError_None && pEntry)
    {
        if (sCmdHandlers == pEntry)
        {
            sCmdHandlers = sCmdHandlers->mNext;
        }
        delete pEntry;
        // Remove the transaction ID from the 'in use' bit field
        sCmdTIDsInUse &= ~(1 << pEntry->TransactionId);
    }

exit:

    return status;
}

ThreadError
otcSendAsync(
    otInstance *aInstance,
    SpinelCmdHandler Handler,
    void* HandlerContext,
    spinel_tid_t *pTid,
    uint32_t command,
    spinel_prop_key_t key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otcSendAsyncV(aInstance, Handler, HandlerContext, pTid, command, key, pack_format, args);
    va_end(args);
    return status;
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
            sMacFrameHandler(
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
otcProcess(
    otInstance *aInstance,
    uint32_t command,
    const uint8_t* cmd_data_ptr,
    spinel_size_t cmd_data_len
    )
{
    uint8_t header;
    spinel_prop_key_t key;
    uint8_t* value_data_ptr = NULL;
    spinel_size_t value_data_len = 0;

    // Make sure it's an expected command
    if (command < SPINEL_CMD_PROP_VALUE_IS || command > SPINEL_CMD_PROP_VALUE_REMOVED)
    {
        return;
    }

    // Decode the key and data
    if (spinel_datatype_unpack(cmd_data_ptr, cmd_data_len, "CiiD", &header, NULL, &key, &value_data_ptr, &value_data_len) == -1)
    {
        return;
    }

    // Get the transaction ID
    if (SPINEL_HEADER_GET_TID(header) == 0)
    {
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
        SpinelCmdHandlerEntry* pEntry = otcRemoveCmdHandler(SPINEL_HEADER_GET_TID(header), false);
        if (pEntry)
        {
            // Call the handler function
            pEntry->Handler(aInstance, pEntry->Context, command, key, value_data_ptr, value_data_len);
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

void
cmdSendMacFrameComplete(
    otInstance *aInstance,
    void* Context,
    uint32_t Command,
    spinel_prop_key_t Key,
    const uint8_t* Data,
    spinel_size_t DataLength
    )
{
    (void)Context;
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

    sMacFrameDoneHandler(aInstance);
}

ThreadError
otcSendPacket(otInstance *aInstance, const struct RadioPacket *pkt)
{
    ThreadError error = otcSendAsync(
            aInstance,
            cmdSendMacFrameComplete,
            NULL,
            NULL,
            SPINEL_CMD_PROP_VALUE_SET,
            SPINEL_PROP_STREAM_RAW,
            SPINEL_DATATYPE_DATA_WLEN_S
            SPINEL_DATATYPE_UINT8_S
            SPINEL_DATATYPE_INT8_S,
            pkt->mPsdu,
            (uint32_t)pkt->mLength,
            pkt->mChannel,
            pkt->mPower);

    return error;
}

void
otcSetPropHandler(
    otInstance *aInstance,
    void* Context,
    uint32_t command,
    spinel_prop_key_t key,
    const uint8_t* data,
    spinel_size_t dataLength
    )
{
    SpinelSetPropContext* cmdContext = (SpinelSetPropContext*)Context;

    if (data == NULL)
    {
        cmdContext->Status = kThreadError_Abort;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_IS && key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t spinel_status = SPINEL_STATUS_OK;
        spinel_ssize_t packed_len = spinel_datatype_unpack(data, dataLength, "i", &spinel_status);
        if (packed_len < 0 || (uint32_t)packed_len > dataLength)
        {
            cmdContext->Status = kThreadError_NoBufs;
        }
        else
        {
            ThreadError errorCode = SpinelStatusToThreadError(spinel_status);
            cmdContext->Status = errorCode;
        }
    }
    else if (command != cmdContext->ExpectedResultCommand)
    {
        cmdContext->Status = kThreadError_InvalidArgs;
    }
    else if (key == cmdContext->key)
    {
        cmdContext->Status = kThreadError_None;
    }
    else
    {
        cmdContext->Status = kThreadError_InvalidArgs;
    }

    cmdContext->Expire = 0;
    (void) aInstance;
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

    // Create the context structure
    uint32_t now = otPlatAlarmGetNow();
    if (now < sSetPropContext.Expire)
    {
        return kThreadError_Busy;
    }

    sSetPropContext.key = key;
    sSetPropContext.Status = kThreadError_None;
    sSetPropContext.Expire = now + 1000;

    if (command == SPINEL_CMD_PROP_VALUE_SET)
    {
        sSetPropContext.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_IS;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_INSERT)
    {
        sSetPropContext.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_INSERTED;
    }
    else if (command == SPINEL_CMD_PROP_VALUE_REMOVE)
    {
        sSetPropContext.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_REMOVED;
    }
    else
    {
        ExitNow(status = kThreadError_Parse);
    }

    // Send the request transaction
    status = otcSendAsyncV(
            aInstance,
            otcSetPropHandler,
            &sSetPropContext,
            &tid,
            command,
            key,
            pack_format,
            args);

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
otcGetPropHandler(
    otInstance *aInstance,
    uint32_t command,
    spinel_prop_key_t key,
    const uint8_t* data,
    spinel_size_t dataLength,
    const char* aFormat,
    ...
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
    else if (key == sGetPropContext.key)
    {
        va_list args;
        va_start(args, aFormat);
        spinel_ssize_t packed_len = spinel_datatype_vunpack(data, dataLength, aFormat, args);
        if (packed_len < 0 || (uint32_t)packed_len > dataLength)
        {
            error = kThreadError_NoBufs;
        }
        else
        {
            error = kThreadError_None;
        }
        va_end(args);

    } else {
        error = kThreadError_InvalidArgs;
    }

    return error;
}

ThreadError
otcGetProp(
    otInstance *aInstance,
    spinel_prop_key_t key,
    SpinelCmdHandler aHandler,
    void *aContext)
{
    ThreadError status;
    spinel_tid_t tid;

    // Create the context structure
    uint32_t now = otPlatAlarmGetNow();
    if (now < sGetPropContext.Expire)
    {
        return kThreadError_Busy;
    }

    // Send the request transaction
    status = otcSendAsyncV(
            aInstance,
            aHandler,
            aContext,
            &tid,
            SPINEL_CMD_PROP_VALUE_GET,
            key,
            NULL,
            NULL);

    sGetPropContext.key = key;
    sGetPropContext.Expire = now + 1000;
    return status;
}

#ifdef __cplusplus
}  // extern "C"
#endif

