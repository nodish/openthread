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

/**
 * @file
 *   This file includes definitions for the HDLC interface to radio (RCP).
 */

#ifndef POSIX_APP_HDLC_INTERFACE_HPP_
#define POSIX_APP_HDLC_INTERFACE_HPP_

#include "openthread-posix-config.h"
#include "platform-posix.h"
#include "posix/platform/radio.hpp"

#include <openthread/radio_driver.h>

#if OPENTHREAD_POSIX_CONFIG_RCP_UART_ENABLE

namespace ot {
namespace Posix {

class FileDescriptor : public otPosixRadioInstance
{
public:
    FileDescriptor(int aFd);

    ~FileDescriptor(void);

    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]    aMainloop   A reference to the mainloop context.
     *
     */
    void Poll(otSysMainloopContext &aMainloop);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]       aMainloop   A reference to the mainloop context.
     *
     */
    void Process(const otSysMainloopContext &aMainloop);

    otError Wait(uint32_t aTimeout);

    otError Write(const uint8_t *aBuffer, uint16_t aLength);

    static otPosixRadioDataFuncs sDataFuncs;
    static otPosixRadioPollFuncs sPollFuncs;

private:
    enum
    {
        kMaxFrameSize = 2048, ///< Maximum frame size (number of bytes).
        kMaxWaitTime  = 2000, ///< Maximum wait time in Milliseconds for socket to become writable (see `SendFrame`).
    };

    otPosixDataCallback mDataCallback;
    void *              mDataContext;

    /**
     * This method instructs `HdlcInterface` to read and decode data from radio over the socket.
     *
     * If a full HDLC frame is decoded while reading data, this method invokes the `HandleReceivedFrame()` (on the
     * `aCallback` object from constructor) to pass the received frame to be processed.
     *
     */
    void    Read(void);
    otError WaitForWritable(void);

    int mSockFd;
};
} // namespace Posix
} // namespace ot

#endif // OPENTHREAD_POSIX_CONFIG_RCP_UART_ENABLE
#endif // POSIX_APP_HDLC_INTERFACE_HPP_
