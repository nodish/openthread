#include "platform-posix.h"

#include <termios.h>
#include <util.h>
#include <errno.h>
#include <openthread-core-config.h>

#include <ncp/hdlc.hpp>
#include <ncp/ncp_uart.hpp>
#include <openthread/platform/alarm.h>
#include "link-raw.h"

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
Thread::Hdlc::Encoder   sFrameEncoder;
MacFrameHandler         sMacFrameHandler;

UartTxBuffer sTxBuffer;

int sCmdResetReason;

typedef void (*SpinelCmdHandler)( otInstance *aInstance, void* Context, uint32_t Command, spinel_prop_key_t Key, const uint8_t* Data, spinel_size_t DataLength);

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
    spinel_prop_key_t       Key;
    ThreadError             Status;
} SpinelSetPropContext;

typedef struct _SpinelGetPropContext
{
    uint32_t                Expire;
    spinel_prop_key_t   Key;
    void              *DataBuffer;
    const char*         Format;
    va_list             Args;
    ThreadError            Status;
} SpinelGetPropContext;

SpinelCmdHandlerEntry          *sCmdHandlers = NULL;
spinel_tid_t                    sCmdNextTID;
uint16_t                        sCmdTIDsInUse;

void handleFrame(void *aContext, uint8_t *aFrame, uint16_t aFrameLength);

void handleError(void *aContext, ThreadError aError, uint8_t *aFrame, uint16_t aFrameLength)
{
    (void)aContext;
    (void)aError;
    (void)aFrame;
    (void)aFrameLength;
}

Thread::Hdlc::Decoder sDecoder(sOutBuf, sizeof(sOutBuf), handleFrame, handleError, NULL);

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

int otLinkRawInit()
{
    char *ncpFile = getenv("NCP_FILE");
    sSockFd = open_system_socket_forkpty(ncpFile);
    return sSockFd;
}

void otNcpReceive(otInstance *aInstance, MacFrameHandler aFrameHandler)
{
    uint8_t buf[255];
    ssize_t rval = read(sSockFd, buf, sizeof(buf));

    if (rval < 0)
    {
        perror("read");
        exit(EXIT_FAILURE);
    }

    sMacFrameHandler = aFrameHandler;
    sContext = aInstance;
    sDecoder.Decode(buf, (uint16_t)rval);
}

spinel_tid_t
otNcpGetNextTID()
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

void
otNcpAddHandler(
    SpinelCmdHandlerEntry *pEntry
    )
{
    // TODO cancel expired

    // Get the next transaction ID. This call will block if there are
    // none currently available.
    pEntry->TransactionId = otNcpGetNextTID();

    // Add to the handlers list
    pEntry->mNext = sCmdHandlers;

    sCmdHandlers = pEntry;
}

ThreadError
otNcpEncodeAndSendAsync(
    otInstance *aInstance,
    uint32_t Command,
    spinel_prop_key_t Key,
    spinel_tid_t tid,
    uint32_t MaxDataLength,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status = kThreadError_None;
    uint8_t buffer[256];

    (void)MaxDataLength;
    // Pack the header, command and key
    spinel_ssize_t packedLength =
        spinel_datatype_pack(
            buffer,
            sizeof(buffer),
            "Cii",
            SPINEL_HEADER_FLAG | SPINEL_HEADER_IID_0 | tid,
            Command,
            Key);

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

    write(sSockFd, sTxBuffer.GetBuffer(), sTxBuffer.GetLength());

    (void)aInstance;
    return status;
}

ThreadError
otNcpSendAsyncV(
    otInstance *aInstance,
    SpinelCmdHandler Handler,
    void* HandlerContext,
    spinel_tid_t *pTid,
    uint32_t Command,
    spinel_prop_key_t Key,
    uint32_t MaxDataLength,
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

        otNcpAddHandler(pEntry);

        if (pTid) *pTid = pEntry->TransactionId;
    }

    status = otNcpEncodeAndSendAsync(aInstance, Command, Key, pEntry ? pEntry->TransactionId : 0, MaxDataLength, pack_format, args);

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

    if (pEntry) delete pEntry;

    return status;
}

ThreadError
otNcpSendAsync(
    otInstance *aInstance,
    SpinelCmdHandler Handler,
    void* HandlerContext,
    spinel_tid_t *pTid,
    uint32_t Command,
    spinel_prop_key_t Key,
    uint32_t MaxDataLength,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otNcpSendAsyncV(aInstance, Handler, HandlerContext, pTid, Command, Key, MaxDataLength, pack_format, args);
    va_end(args);
    return status;
}

void
otNcpValueIs(
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
            }
            else if (output_len < 128)
            {
                char strOutput[128] = { 0 };
                memcpy(strOutput, output, output_len);
            }
        }
    }

}

void
otNcpValueInserted(
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
otNcpProcess(
    otInstance *aInstance,
    uint32_t command,
    const uint8_t* cmd_data_ptr,
    spinel_size_t cmd_data_len
    )
{
    uint8_t Header;
    spinel_prop_key_t key;
    uint8_t* value_data_ptr = NULL;
    spinel_size_t value_data_len = 0;

    // Make sure it's an expected command
    if (command < SPINEL_CMD_PROP_VALUE_IS || command > SPINEL_CMD_PROP_VALUE_REMOVED)
    {
        return;
    }

    // Decode the key and data
    if (spinel_datatype_unpack(cmd_data_ptr, cmd_data_len, "CiiD", &Header, NULL, &key, &value_data_ptr, &value_data_len) == -1)
    {
        return;
    }

    // Get the transaction ID
    if (SPINEL_HEADER_GET_TID(Header) == 0)
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
                otNcpValueIs(aInstance, key, value_data_ptr, value_data_len);
            }
            else if (command == SPINEL_CMD_PROP_VALUE_INSERTED)
            {
                otNcpValueInserted(aInstance, key, value_data_ptr, value_data_len);
            }
        }
    }
    // If there was a transaction ID, then look for the corresponding command handler
    else
    {
        SpinelCmdHandlerEntry* pEntry = sCmdHandlers;
        SpinelCmdHandlerEntry* Prev = NULL;

        // Search for matching handlers for this command
        while (pEntry != NULL)
        {
            if (SPINEL_HEADER_GET_TID(Header) == pEntry->TransactionId)
            {
                // Remove the transaction ID from the 'in use' bit field
                sCmdTIDsInUse &= ~(1 << pEntry->TransactionId);

                break;
            }
            Prev = pEntry;
            pEntry = pEntry->mNext;
        }

        // TODO - Set event

        // Process the handler we found, outside the lock
        if (pEntry)
        {
            // Call the handler function
            pEntry->Handler(NULL, pEntry->Context, command, key, value_data_ptr, value_data_len);

            if (Prev)
            {
                Prev->mNext = pEntry->mNext;
            }
            else
            {
                sCmdHandlers = pEntry->mNext;
            }

            // Free the entry
            delete pEntry;
        }
    }
}

void handleFrame(
    void* aContext,
    uint8_t *Buffer,
    uint16_t BufferLength)
{
    uint8_t Header;
    uint32_t Command;

    // Unpack the header from the buffer
    if (spinel_datatype_unpack(Buffer, BufferLength, "Ci", &Header, &Command) <= 0)
    {
        return;
    }

    // Validate the header
    if ((Header & SPINEL_HEADER_FLAG) != SPINEL_HEADER_FLAG)
    {
        return;
    }

    // We only support IID zero for now
    if (SPINEL_HEADER_GET_IID(Header) != 0)
    {
        return;
    }

    // Process the received command
    otNcpProcess((otInstance*)aContext,Command, Buffer, BufferLength);
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
otNcpSendPacket(otInstance *aInstance, const struct RadioPacket *pkt)
{
    return otNcpSendAsync(
            aInstance,
            NULL,
            NULL,
            NULL,
            SPINEL_CMD_PROP_VALUE_SET,
            SPINEL_PROP_STREAM_RAW,
            pkt->mLength + 20,
            SPINEL_DATATYPE_DATA_WLEN_S
            SPINEL_DATATYPE_UINT8_S
            SPINEL_DATATYPE_INT8_S,
            pkt->mPsdu,
            (uint32_t)pkt->mLength,
            pkt->mChannel,
            pkt->mPower);
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
otNcpSetPropHandler(
    otInstance *aInstance,
    void* Context,
    uint32_t Command,
    spinel_prop_key_t Key,
    const uint8_t* Data,
    spinel_size_t DataLength
    )
{
    SpinelSetPropContext* CmdContext = (SpinelSetPropContext*)Context;

    if (Data == NULL)
    {
        CmdContext->Status = kThreadError_Abort;
    }
    else if (Command == SPINEL_CMD_PROP_VALUE_IS && Key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t spinel_status = SPINEL_STATUS_OK;
        spinel_ssize_t packed_len = spinel_datatype_unpack(Data, DataLength, "i", &spinel_status);
        if (packed_len < 0 || (uint32_t)packed_len > DataLength)
        {
            CmdContext->Status = kThreadError_NoBufs;
        }
        else
        {
            ThreadError errorCode = SpinelStatusToThreadError(spinel_status);
            CmdContext->Status = errorCode;
        }
    }
    else if (Command != CmdContext->ExpectedResultCommand)
    {
        CmdContext->Status = kThreadError_InvalidArgs;
    }
    else if (Key == CmdContext->Key)
    {
        CmdContext->Status = kThreadError_None;
    }
    else
    {
        CmdContext->Status = kThreadError_InvalidArgs;
    }

    // TODO
    (void)aInstance;
    //otNcpCancel();
}
void
otNcpGetPropHandler(
    otInstance *aInstance,
    void* Context,
    uint32_t Command,
    spinel_prop_key_t Key,
    const uint8_t* Data,
    spinel_size_t DataLength
    )
{
    SpinelGetPropContext* CmdContext = (SpinelGetPropContext*)Context;

    if (Data == NULL)
    {
        CmdContext->Status = kThreadError_Abort;
    }
    else if (Command != SPINEL_CMD_PROP_VALUE_IS)
    {
        CmdContext->Status = kThreadError_InvalidArgs;
    }
    else if (Key == SPINEL_PROP_LAST_STATUS)
    {
        spinel_status_t spinel_status = SPINEL_STATUS_OK;
        spinel_ssize_t packed_len = spinel_datatype_unpack(Data, DataLength, "i", &spinel_status);
        if (packed_len < 0 || (uint32_t)packed_len > DataLength)
        {
            CmdContext->Status = kThreadError_NoBufs;
        }
        else
        {
            CmdContext->Status = SpinelStatusToThreadError(spinel_status);
        }
    }
    else if (Key == CmdContext->Key)
    {
        spinel_ssize_t packed_len = spinel_datatype_vunpack(Data, DataLength, CmdContext->Format, CmdContext->Args);
        if (packed_len < 0 || (uint32_t)packed_len > DataLength)
        {
            CmdContext->Status = kThreadError_NoBufs;
        }
        else
        {
            CmdContext->Status = kThreadError_None;
        }
    }
    else
    {
        CmdContext->Status = kThreadError_InvalidArgs;
    }

    // Set the completion event
    //KeSetEvent(&CmdContext->CompletionEvent, 0, false);
    (void)aInstance;
}

void otNcpCancel(spinel_tid_t tid)
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
        pEntry->Handler(NULL, pEntry->Context, 0, (spinel_prop_key_t)0, NULL, 0);

        if (Prev) Prev->mNext = pEntry->mNext;
        else
            sCmdHandlers = pEntry->mNext;
        // Free the entry
        delete pEntry;
    }
}

ThreadError
otNcpSetPropV(
    otInstance *aInstance,
    uint32_t Command,
    spinel_prop_key_t Key,
    const char *pack_format,
    va_list args
    )
{
    ThreadError status;
    spinel_tid_t tid;

    // Create the context structure
    SpinelSetPropContext Context;
    Context.Key = Key;
    Context.Status = kThreadError_None;
    Context.Expire = otPlatAlarmGetNow() + 1000;

    if (Command == SPINEL_CMD_PROP_VALUE_SET)
    {
        Context.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_IS;
    }
    else if (Command == SPINEL_CMD_PROP_VALUE_INSERT)
    {
        Context.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_INSERTED;
    }
    else if (Command == SPINEL_CMD_PROP_VALUE_REMOVE)
    {
        Context.ExpectedResultCommand = SPINEL_CMD_PROP_VALUE_REMOVED;
    }
    else
    {
        // TODO ASSERT(false);
    }

    // Send the request transaction
    status =
        otNcpSendAsyncV(
            aInstance,
            otNcpSetPropHandler,
            &Context,
            &tid,
            Command,
            Key,
            8,
            pack_format,
            args);

    if (status == kThreadError_None)
    {
        // TODO Timer to cancel command
    }
    else
    {
        Context.Status = status;
    }

    return Context.Status;
}

ThreadError
otNcpSetProp(
    otInstance *aInstance,
    spinel_prop_key_t Key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otNcpSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_SET, Key, pack_format, args);
    va_end(args);
    return status;
}

ThreadError
otNcpInsertProp(
    otInstance *aInstance,
    spinel_prop_key_t Key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otNcpSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_INSERT, Key, pack_format, args);
    va_end(args);
    return status;
}

ThreadError
otNcpRemoveProp(
    otInstance *aInstance,
    spinel_prop_key_t Key,
    const char *pack_format,
    ...
    )
{
    va_list args;
    va_start(args, pack_format);
    ThreadError status = otNcpSetPropV(aInstance, SPINEL_CMD_PROP_VALUE_REMOVE, Key, pack_format, args);
    va_end(args);
    return status;
}
ThreadError
otNcpGetProp(
    otInstance *aInstance,
    void *DataBuffer,
    spinel_prop_key_t Key,
    const char *pack_format,
    ...
    )
{
    ThreadError status;
    spinel_tid_t tid;

    // Create the context structure
    SpinelGetPropContext Context;
    Context.Key = Key;
    Context.DataBuffer = DataBuffer;
    Context.Format = pack_format;
    Context.Status = kThreadError_None;
    Context.Expire = otPlatAlarmGetNow() + 1000;
    va_start(Context.Args, pack_format);

    // Send the request transaction
    status =
        otNcpSendAsyncV(
            aInstance,
            otNcpGetPropHandler,
            &Context,
            &tid,
            SPINEL_CMD_PROP_VALUE_GET,
            Key,
            0,
            NULL,
            NULL);
    if (status == kThreadError_None)
    {
    }
    else
    {
        Context.Status = status;
    }

    va_end(Context.Args);

    return Context.Status;
}

#ifdef __cplusplus
}  // extern "C"
#endif

