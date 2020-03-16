#include "posix/platform/module_tty.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "common/code_utils.hpp"
#include "posix/platform/module_fd.hpp"

namespace ot {
namespace Posix {

#ifdef __APPLE__

#ifndef B230400
#define B230400 230400
#endif

#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B576000
#define B576000 576000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

#ifndef B1152000
#define B1152000 1152000
#endif

#ifndef B1500000
#define B1500000 1500000
#endif

#ifndef B2000000
#define B2000000 2000000
#endif

#ifndef B2500000
#define B2500000 2500000
#endif

#ifndef B3000000
#define B3000000 3000000
#endif

#ifndef B3500000
#define B3500000 3500000
#endif

#ifndef B4000000
#define B4000000 4000000
#endif

#endif // __APPLE__

static otPosixRadioInstance *Create(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    int fd   = -1;
    int rval = 0;

    VerifyOrExit(aNext == NULL);

    fd = open(otPosixRadioArgumentsGetPath(aArguments), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd == -1)
    {
        perror("open uart failed");
        ExitNow();
    }

    if (isatty(fd))
    {
        const char *   value;
        struct termios tios;

        int  speed  = 115200;
        int  cstopb = 1;
        char parity = 'N';
        char flow   = 'N';

        VerifyOrExit((rval = tcgetattr(fd, &tios)) == 0);

        cfmakeraw(&tios);

        tios.c_cflag = CS8 | HUPCL | CREAD | CLOCAL;

        value = otPosixRadioArgumentsGetValue(aArguments, "speed");
        if (value != NULL)
        {
            speed = atoi(value);
        }

        value = otPosixRadioArgumentsGetValue(aArguments, "parity");
        if (value != NULL && strlen(value) != 1)
        {
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
        }

        switch (parity)
        {
        case 'N':
            break;
        case 'E':
            tios.c_cflag |= PARENB;
            break;
        case 'O':
            tios.c_cflag |= (PARENB | PARODD);
            break;
        default:
            // not supported
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        value = otPosixRadioArgumentsGetValue(aArguments, "cstopb");

        if (value != NULL)
        {
            cstopb = atoi(value);
        }

        switch (cstopb)
        {
        case 1:
            tios.c_cflag &= static_cast<unsigned long>(~CSTOPB);
            break;
        case 2:
            tios.c_cflag |= CSTOPB;
            break;
        default:
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        value = otPosixRadioArgumentsGetValue(aArguments, "flow");

        if (value != NULL && strlen(value) != 1)
        {
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
        }
        else
        {
            flow = value[0];
        }

        switch (speed)
        {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
#ifdef B230400
        case 230400:
            speed = B230400;
            break;
#endif
#ifdef B460800
        case 460800:
            speed = B460800;
            break;
#endif
#ifdef B500000
        case 500000:
            speed = B500000;
            break;
#endif
#ifdef B576000
        case 576000:
            speed = B576000;
            break;
#endif
#ifdef B921600
        case 921600:
            speed = B921600;
            break;
#endif
#ifdef B1000000
        case 1000000:
            speed = B1000000;
            break;
#endif
#ifdef B1152000
        case 1152000:
            speed = B1152000;
            break;
#endif
#ifdef B1500000
        case 1500000:
            speed = B1500000;
            break;
#endif
#ifdef B2000000
        case 2000000:
            speed = B2000000;
            break;
#endif
#ifdef B2500000
        case 2500000:
            speed = B2500000;
            break;
#endif
#ifdef B3000000
        case 3000000:
            speed = B3000000;
            break;
#endif
#ifdef B3500000
        case 3500000:
            speed = B3500000;
            break;
#endif
#ifdef B4000000
        case 4000000:
            speed = B4000000;
            break;
#endif
        default:
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        switch (flow)
        {
        case 'N':
            break;
        case 'H':
            tios.c_cflag |= CRTSCTS;
            break;
        default:
            // not supported
            DieNow(OT_EXIT_INVALID_ARGUMENTS);
            break;
        }

        VerifyOrExit((rval = cfsetspeed(&tios, static_cast<speed_t>(speed))) == 0, perror("cfsetspeed"));
        VerifyOrExit((rval = tcsetattr(fd, TCSANOW, &tios)) == 0, perror("tcsetattr"));
        VerifyOrExit((rval = tcflush(fd, TCIOFLUSH)) == 0);
    }

exit:
    if (rval != 0)
    {
        DieNow(OT_EXIT_ERROR_ERRNO);
    }

    return new FileDescriptor(fd);
}

static otError Delete(otPosixRadioInstance *aInstance)
{
    delete static_cast<FileDescriptor *>(aInstance);
}

static otPosixRadioDriver sTeletypeDriver = {
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
otError platformModuleInitTeletype(void)
#endif
{
    return otPosixRadioDriverRegister(&ot::Posix::sTeletypeDriver);
}
