
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#if defined(__APPLE__) || defined(__NetBSD__)
#include <util.h>
#else
#include <pty.h>
#endif

#include "common/code_utils.hpp"
#include "posix/platform/module_fd.hpp"

#ifndef SOCKET_UTILS_DEFAULT_SHELL
#define SOCKET_UTILS_DEFAULT_SHELL "/bin/sh"
#endif

namespace ot {
namespace Posix {

static otPosixRadioInstance *Create(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    int fd   = -1;
    int pid  = -1;
    int rval = -1;

    VerifyOrExit(aNext == NULL);

    {
        struct termios tios;

        memset(&tios, 0, sizeof(tios));
        cfmakeraw(&tios);
        tios.c_cflag = CS8 | HUPCL | CREAD | CLOCAL;

        VerifyOrExit((pid = forkpty(&fd, NULL, &tios, NULL)) != -1, perror("forkpty()"));
    }

    if (0 == pid)
    {
        const int kMaxCommand = 255;
        char      cmd[kMaxCommand];

        rval = snprintf(cmd, sizeof(cmd), "exec %s %s", otPosixRadioArgumentsGetPath(aArguments),
                        otPosixRadioArgumentsGetValue(aArguments, "args"));
        VerifyOrExit(rval > 0 && static_cast<size_t>(rval) < sizeof(cmd),
                     fprintf(stderr, "NCP file and configuration is too long!");
                     rval = -1);

        VerifyOrExit((rval = execl(SOCKET_UTILS_DEFAULT_SHELL, SOCKET_UTILS_DEFAULT_SHELL, "-c", cmd,
                                   static_cast<char *>(NULL))) != -1,
                     perror("execl(OT_RCP)"));
    }
    else
    {
        VerifyOrExit((rval = fcntl(fd, F_GETFL)) != -1, perror("fcntl(F_GETFL)"));
        VerifyOrExit((rval = fcntl(fd, F_SETFL, rval | O_NONBLOCK | O_CLOEXEC)) != -1, perror("fcntl(F_SETFL)"));
    }

exit:
    return new FileDescriptor(fd);
}

static otError Delete(otPosixRadioInstance *aInstance)
{
    delete static_cast<FileDescriptor *>(aInstance);
}

static otPosixRadioDriver sForkptyDriver = {
    .mNext       = NULL,
    .mName       = "tty",
    .Create      = Create,
    .Delete      = Delete,
    .mOperations = NULL,
    .mData       = &FileDescriptor::sDataFuncs,
    .mPoll       = &FileDescriptor::sPollFuncs,
};

} // namespace Posix
} // namespace ot

#if OPENTHREAD_POSIX_SPINEL_MODULE_ENABLE
extern "C" otError otPosixModuleInit(void)
#else
otError platformModuleInitForkpty(void)
#endif
{
    return otPosixRadioDriverRegister(&ot::Posix::sForkptyDriver);
}
