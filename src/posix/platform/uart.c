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

#include "platform-posix.h"

#include "openthread-core-config.h"

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <openthread/platform/uart.h>

#include "code_utils.h"

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
#define SOCKET_NAME "/tmp/ot-daemon.sock"
static int  sUartSocket    = -1;
static int  sSessionSocket = -1;
static bool sDaemon        = false;
#endif // OPENTHREAD_ENABLE_POSIX_APP_DAEMON

static const uint8_t *sWriteBuffer = NULL;
static uint16_t       sWriteLength = 0;

otError otPlatUartEnable(void)
{
    otError error = OT_ERROR_NONE;

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    if (otSysGetConfig()->mDaemon)
    {
        struct sockaddr_un sockname;
        int                ret;

        sUartSocket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
        if (sUartSocket == -1)
        {
            perror("socket");
            exit(OT_EXIT_FAILURE);
        }

        memset(&sockname, 0, sizeof(struct sockaddr_un));

        sockname.sun_family = AF_UNIX;
        strncpy(sockname.sun_path, SOCKET_NAME, sizeof(sockname.sun_path) - 1);

        ret = bind(sUartSocket, (const struct sockaddr *)&sockname, sizeof(struct sockaddr_un));
        if (ret == -1)
        {
            perror("bind");
            exit(OT_EXIT_FAILURE);
        }

        //
        // only accept 1 connection.
        //
        ret = listen(sUartSocket, 1);
        if (ret == -1)
        {
            perror("listen");
            exit(OT_EXIT_FAILURE);
        }
    }
#endif // OPENTHREAD_ENABLE_POSIX_APP_DAEMON

    return error;
}

otError otPlatUartDisable(void)
{
    otError error = OT_ERROR_NONE;

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    if (otSysGetConfig()->mDaemon)
    {
        if (sSessionSocket != -1)
        {
            close(sSessionSocket);
            sSessionSocket = -1;
        }

        if (sUartSocket != -1)
        {
            close(sUartSocket);
            sUartSocket = -1;
        }
    }
#endif // OPENTHREAD_ENABLE_POSIX_APP_DAEMON

    return error;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sWriteLength == 0, error = OT_ERROR_BUSY);

    sWriteBuffer = aBuf;
    sWriteLength = aBufLength;

exit:
    return error;
}

void platformUartUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, fd_set *aErrorFdSet, int *aMaxFd)
{
    int fd;

    assert(aReadFdSet != NULL);
    assert(aWriteFdSet != NULL);
    assert(aMaxFd != NULL);

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    if (otSysGetConfig()->mDaemon)
    {
        fd = (sSessionSocket == -1 ? sUartSocket : sSessionSocket);
    }
    else
#endif
    {
        fd = STDIN_FILENO;
    }

    FD_SET(fd, aReadFdSet);
    FD_SET(fd, aErrorFdSet);

    if (*aMaxFd < fd)
    {
        *aMaxFd = fd;
    }

    if (sWriteLength > 0)
    {
#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
        if (otSysGetConfig()->mDaemon)
        {
            fd = (sSessionSocket == -1 ? sUartSocket : sSessionSocket);
        }
        else
#endif
        {
            fd = STDOUT_FILENO;
        }

        FD_SET(fd, aWriteFdSet);

        if (aErrorFdSet != NULL)
        {
            FD_SET(fd, aErrorFdSet);
        }

        if (aMaxFd != NULL && *aMaxFd < fd)
        {
            *aMaxFd = fd;
        }
    }
}

void platformUartProcess(const fd_set *aReadFdSet, const fd_set *aWriteFdSet, const fd_set *aErrorFdSet)
{
    ssize_t rval;
    int     fd;

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    if (otSysGetConfig()->mDaemon)
    {
        if (FD_ISSET(sUartSocket, aErrorFdSet))
        {
            perror("socket error");
            exit(OT_EXIT_FAILURE);
        }
        else if (FD_ISSET(sUartSocket, aReadFdSet))
        {
            sSessionSocket = accept(sUartSocket, NULL, NULL);
        }

        otEXPECT(sSessionSocket != -1);

        if (FD_ISSET(sSessionSocket, aErrorFdSet))
        {
            close(sSessionSocket);
            sSessionSocket = -1;
        }

        otEXPECT(sSessionSocket != -1);

        fd = sSessionSocket;
    }
    else
#endif // OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    {
        if (FD_ISSET(STDIN_FILENO, aErrorFdSet))
        {
            perror("stdin");
            exit(OT_EXIT_FAILURE);
        }

        if (FD_ISSET(STDOUT_FILENO, aErrorFdSet))
        {
            perror("stdout");
            exit(OT_EXIT_FAILURE);
        }

        fd = STDIN_FILENO;
    }

    if (FD_ISSET(fd, aReadFdSet))
    {
        uint8_t buffer[256];

        rval = read(fd, buffer, sizeof(buffer));

        if (rval > 0)
        {
            otPlatUartReceived(buffer, (uint16_t)rval);
        }
        else if (rval <= 0)
        {
            perror("UART read");
#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
            if (otSysGetConfig()->mDaemon)
            {
                close(sSessionSocket);
                sSessionSocket = -1;
                otEXIT_NOW();
            }
            else
#endif
            {
                exit(OT_EXIT_FAILURE);
            }
        }
    }

#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
    if (!otSysGetConfig()->mDaemon)
#endif
    {
        fd = STDOUT_FILENO;
    }

    if ((sWriteLength > 0) && (FD_ISSET(fd, aWriteFdSet)))
    {
        rval = write(fd, sWriteBuffer, sWriteLength);

        if (rval < 0)
        {
            perror("UART write");
#if OPENTHREAD_ENABLE_POSIX_APP_DAEMON
            if (otSysGetConfig()->mDaemon)
            {
                close(sSessionSocket);
                sSessionSocket = -1;
                otEXIT_NOW();
            }
            else
#endif
            {
                exit(OT_EXIT_FAILURE);
            }
        }

        otEXPECT(rval > 0);

        sWriteBuffer += (uint16_t)rval;
        sWriteLength -= (uint16_t)rval;

        if (sWriteLength == 0)
        {
            otPlatUartSendDone();
        }
    }

exit:
    return;
}
