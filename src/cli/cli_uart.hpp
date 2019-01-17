/*
 *  Copyright (c) 2016, The OpenThread Authors.
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

/**
 * @file
 *   This file contains definitions for a CLI server on the UART service.
 */

#ifndef CLI_UART_HPP_
#define CLI_UART_HPP_

#include "openthread-core-config.h"

#include "cli/cli_server.hpp"
#include "common/instance.hpp"
#include "common/logging.hpp"
#include "common/tasklet.hpp"

#include <openthread/platform/uart.h>

#if OPENTHREAD_CONFIG_ENABLE_DEBUG_UART
#include <openthread/platform/debug_uart.h>
#endif

#ifdef OT_CLI_UART_LOCK_HDR_FILE

#include OT_CLI_UART_LOCK_HDR_FILE

#else

/**
 * Macro to acquire an exclusive lock of uart cli output
 * Default implementation does nothing
 *
 */
#ifndef OT_CLI_UART_OUTPUT_LOCK
#define OT_CLI_UART_OUTPUT_LOCK() \
    do                            \
    {                             \
    } while (0)
#endif

/**
 * Macro to release the exclusive lock of uart cli output
 * Default implementation does nothing
 *
 */
#ifndef OT_CLI_UART_OUTPUT_UNLOCK
#define OT_CLI_UART_OUTPUT_UNLOCK() \
    do                              \
    {                               \
    } while (0)
#endif

#endif // OT_CLI_UART_LOCK_HDR_FILE

namespace ot {
namespace Cli {

/**
 * Initialize the CLI UART module.
 *
 * @param[in]  aInstance  The OpenThread instance structure.
 *
 */
void otCliUartInit(void);

/**
 * This class implements the CLI server on top of the UART platform abstraction.
 *
 */
template <typename kReader> class Uart
{
public:
    /**
     * Constructor
     *
     * @param[in]  aInstance  The OpenThread instance structure.
     *
     */
    Uart(void)
    {
        mTxHead     = 0;
        mTxLength   = 0;
        mSendLength = 0;

        otCliUartInit();
    }

    /**
     * This method delivers raw characters to the client.
     *
     * @param[in]  aBuf        A pointer to a buffer.
     * @param[in]  aBufLength  Number of bytes in the buffer.
     *
     * @returns The number of bytes placed in the output queue.
     *
     */
    int Output(const char *aBuf, uint16_t aBufLength)
    {
        OT_CLI_UART_OUTPUT_LOCK();
        uint16_t remaining = kTxBufferSize - mTxLength;
        uint16_t tail;

        if (aBufLength > remaining)
        {
            aBufLength = remaining;
        }

        for (int i = 0; i < aBufLength; i++)
        {
            tail            = (mTxHead + mTxLength) % kTxBufferSize;
            mTxBuffer[tail] = *aBuf++;
            mTxLength++;
        }

        Send();
        OT_CLI_UART_OUTPUT_UNLOCK();

        return aBufLength;
    }

    void ReceiveTask(const uint8_t *aBuf, uint16_t aBufLength)
    {
        static_cast<kReader *>(this)->Input(aBuf, aBufLength);
    }

    void SendDoneTask(void)
    {
        mTxHead = (mTxHead + mSendLength) % kTxBufferSize;
        mTxLength -= mSendLength;
        mSendLength = 0;

        Send();
    }

private:
    enum
    {
        kTxBufferSize = OPENTHREAD_CONFIG_CLI_UART_TX_BUFFER_SIZE,
    };

    otError ProcessCommand(void);
    void    Send(void)
    {
        VerifyOrExit(mSendLength == 0);

        if (mTxLength > kTxBufferSize - mTxHead)
        {
            mSendLength = kTxBufferSize - mTxHead;
        }
        else
        {
            mSendLength = mTxLength;
        }

        if (mSendLength > 0)
        {
#if OPENTHREAD_CONFIG_ENABLE_DEBUG_UART
            /* duplicate the output to the debug uart */
            otPlatDebugUart_write_bytes(reinterpret_cast<uint8_t *>(mTxBuffer + mTxHead), mSendLength);
#endif
            otPlatUartSend(reinterpret_cast<uint8_t *>(mTxBuffer + mTxHead), mSendLength);
        }

    exit:
        return;
    }

    char     mTxBuffer[kTxBufferSize];
    uint16_t mTxHead;
    uint16_t mTxLength;

    uint16_t mSendLength;
};

} // namespace Cli
} // namespace ot

#endif // CLI_UART_HPP_
