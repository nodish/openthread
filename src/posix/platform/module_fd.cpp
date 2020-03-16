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
 *   This file includes the implementation for the HDLC interface to radio (RCP).
 */

#include "module_fd.hpp"

#include "platform-posix.h"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <syslog.h>
#include <unistd.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"

namespace ot {
namespace Posix {

FileDescriptor::FileDescriptor(int aFd)
    : mSockFd(aFd)
{
}

FileDescriptor::~FileDescriptor(void)
{
    VerifyOrExit(mSockFd != -1);

    VerifyOrExit(0 == close(mSockFd), perror("close RCP"));
    VerifyOrExit(-1 != wait(NULL) || errno == ECHILD, perror("wait RCP"));

    mSockFd = -1;

exit:
    return;
}

otError FileDescriptor::Write(const uint8_t *aFrame, uint16_t aLength)
{
    otError error = OT_ERROR_NONE;
    while (aLength)
    {
        ssize_t rval = write(mSockFd, aFrame, aLength);

        if (rval > 0)
        {
            aLength -= static_cast<uint16_t>(rval);
            aFrame += static_cast<uint16_t>(rval);
            continue;
        }

        if ((rval < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR))
        {
            DieNow(OT_EXIT_ERROR_ERRNO);
        }

        SuccessOrExit(error = WaitForWritable());
    }

exit:
    return error;
}

void FileDescriptor::Poll(otSysMainloopContext &aMainloop)
{
    FD_SET(mSockFd, &aMainloop.mReadFdSet);

    if (aMainloop.mMaxFd < mSockFd)
    {
        aMainloop.mMaxFd = mSockFd;
    }
}

void FileDescriptor::Process(const otSysMainloopContext &aMainloop)
{
    if (FD_ISSET(mSockFd, &aMainloop.mReadFdSet))
    {
        Read();
    }
}

otError FileDescriptor::Wait(uint32_t aTimeout)
{
    otError        error   = OT_ERROR_NONE;
    struct timeval timeout = {aTimeout / 1000000, aTimeout % 1000000};
    fd_set         read_fds;
    fd_set         error_fds;
    int            rval;

    FD_ZERO(&read_fds);
    FD_ZERO(&error_fds);
    FD_SET(mSockFd, &read_fds);
    FD_SET(mSockFd, &error_fds);

    rval = select(mSockFd + 1, &read_fds, NULL, &error_fds, &timeout);

    if (rval > 0)
    {
        if (FD_ISSET(mSockFd, &read_fds))
        {
            Read();
        }
        else if (FD_ISSET(mSockFd, &error_fds))
        {
            DieNowWithMessage("NCP error", OT_EXIT_FAILURE);
        }
        else
        {
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else if (rval == 0)
    {
        ExitNow(error = OT_ERROR_RESPONSE_TIMEOUT);
    }
    else if (errno != EINTR)
    {
        DieNowWithMessage("wait response", OT_EXIT_FAILURE);
    }

exit:
    return error;
}

otError FileDescriptor::WaitForWritable(void)
{
    otError        error   = OT_ERROR_NONE;
    struct timeval timeout = {kMaxWaitTime / 1000, (kMaxWaitTime % 1000) * 1000};
    uint64_t       now     = platformGetTime();
    uint64_t       end     = now + kMaxWaitTime * US_PER_MS;
    fd_set         writeFds;
    fd_set         errorFds;
    int            rval;

    while (true)
    {
        FD_ZERO(&writeFds);
        FD_ZERO(&errorFds);
        FD_SET(mSockFd, &writeFds);
        FD_SET(mSockFd, &errorFds);

        rval = select(mSockFd + 1, NULL, &writeFds, &errorFds, &timeout);

        if (rval > 0)
        {
            if (FD_ISSET(mSockFd, &writeFds))
            {
                ExitNow();
            }
            else if (FD_ISSET(mSockFd, &errorFds))
            {
                DieNow(OT_EXIT_FAILURE);
            }
            else
            {
                assert(false);
            }
        }
        else if ((rval < 0) && (errno != EINTR))
        {
            DieNow(OT_EXIT_ERROR_ERRNO);
        }

        now = platformGetTime();

        if (end > now)
        {
            uint64_t remain = end - now;

            timeout.tv_sec  = static_cast<time_t>(remain / US_PER_S);
            timeout.tv_usec = static_cast<suseconds_t>(remain % US_PER_S);
        }
        else
        {
            break;
        }
    }

    error = OT_ERROR_FAILED;

exit:
    return error;
}

void FileDescriptor::Read(void)

{
    uint8_t buffer[kMaxFrameSize];
    ssize_t rval;

    rval = read(mSockFd, buffer, sizeof(buffer));

    if (rval > 0)
    {
        mDataCallback(mDataContext, buffer, static_cast<uint16_t>(rval));
    }
    else if ((rval < 0) && (errno != EAGAIN) && (errno != EINTR))
    {
        DieNow(OT_EXIT_ERROR_ERRNO);
    }
}

} // namespace Posix
} // namespace ot
