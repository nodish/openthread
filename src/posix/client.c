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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define SOCKET_NAME "/tmp/ot-daemon.sock"

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    int ret;
    int session;

    session = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (session == -1)
    {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    {
        struct sockaddr_un sockname;

        memset(&sockname, 0, sizeof(struct sockaddr_un));
        sockname.sun_family = AF_UNIX;
        strncpy(sockname.sun_path, SOCKET_NAME, sizeof(sockname.sun_path) - 1);

        ret = connect(session, (const struct sockaddr *)&sockname, sizeof(struct sockaddr_un));
    }

    if (ret == -1)
    {
        fprintf(stderr, "OpenThread daemon is not running.\n");
        exit(EXIT_FAILURE);
    }

    while (1)
    {
        char   buffer[512];
        fd_set readFdSet;
        int    maxFd = -1;

        FD_ZERO(&readFdSet);

        FD_SET(STDIN_FILENO, &readFdSet);
        maxFd = STDIN_FILENO;

        FD_SET(session, &readFdSet);
        maxFd = session > maxFd ? session : maxFd;

        ret = select(maxFd + 1, &readFdSet, NULL, NULL, NULL);

        if (ret < 0)
        {
            perror("select");
            break;
        }
        else if (ret == 0)
        {
            continue;
        }

        if (FD_ISSET(STDIN_FILENO, &readFdSet))
        {
            if (fgets(buffer, sizeof(buffer), stdin) == NULL)
            {
                break;
            }

            ret = write(session, buffer, strlen(buffer));
            if (ret == -1)
            {
                perror("write");
                break;
            }
        }

        if (FD_ISSET(session, &readFdSet))
        {
            ret = read(session, buffer, sizeof(buffer));
            if (ret == -1)
            {
                perror("read");
                exit(EXIT_FAILURE);
            }
            else if (ret == 0)
            {
                continue;
            }

            buffer[ret] = 0;
            printf("%s", buffer);
            fflush(stdout);
        }
    }

    close(session);

    return 0;
}
