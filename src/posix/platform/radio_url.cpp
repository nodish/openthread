#include <stdio.h>
#include <string.h>

#include <openthread/radio_driver.h>

#include "core/common/code_utils.hpp"
#include "posix/platform/platform-posix.h"

namespace ot {
namespace Posix {

struct otPosixRadioArguments
{
    const char *mDevice;
    char *      mStart;
    char *      mEnd;
};

class Arguments : public otPosixRadioArguments
{
public:
    Arguments(char *aUrl)
    {
        aUrl = strstr(aUrl, "://");
        VerifyOrDie(aUrl != NULL, OT_EXIT_INVALID_ARGUMENTS);
        aUrl[0] = '\0';
        aUrl += sizeof("://") - 1;
        mDevice = aUrl;

        mStart = strstr(aUrl, "?");

        if (mStart != NULL)
        {
            mStart[0] = '\0';

            mStart += sizeof("?") - 1;

            mEnd = mStart + strlen(mStart);
        }
        else
        {
            mEnd = aUrl + strlen(aUrl);
        }
    }

    const char *GetPath(void) const { return mDevice; }

    const char *GetValue(const char *aName, const char *aLastValue = NULL)
    {
        const char * rval  = NULL;
        const size_t len   = strlen(aName);
        char *       start = (aLastValue == NULL ? mStart : (const_cast<char *>(aLastValue) + strlen(aLastValue) + 1));

        while (start < mEnd)
        {
            char *last = NULL;

            for (char *cur = strtok(start, "&"); cur != NULL; cur = strtok(NULL, "&"))
            {
                if (!strncmp(aName, cur, len))
                {
                    if (cur[len] == '=')
                    {
                        ExitNow(rval = &cur[len + 1]);
                    }
                    else if (cur[len] == '&' || cur[len] == '\0')
                    {
                        ExitNow(rval = "");
                    }
                }
                last = cur;
            }

            start = last + strlen(last) + 1;
        }

    exit:
        return rval;
    }
};

const char *otPosixRadioArgumentsGetPath(otPosixRadioArguments *aArguments)
{
    return static_cast<ot::Posix::Arguments *>(aArguments)->GetPath();
}

const char *otPosixRadioArgumentsGetValue(otPosixRadioArguments *aArguments, const char *aName, const char *aLast)
{
    return static_cast<ot::Posix::Arguments *>(aArguments)->GetValue(aName, aLast);
}

} // namespace Posix
} // namespace ot

#if OT_UNIT_TEST
#include <assert.h>

void TestSimple()
{
    char                 url[] = "spinel:///dev/ttyUSB0?baudrate=115200";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));
    assert(!strcmp(args.GetValue("baudrate"), "115200"));
}

void TestSimpleNoArguments()
{
    char                 url[] = "spinel:///dev/ttyUSB0";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));
}

void TestMultipleProtocols()
{
    char                 url[] = "spinel+spi:///dev/ttyUSB0?baudrate=115200";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));
    assert(!strcmp(args.GetValue("baudrate"), "115200"));
}

void TestMultipleProtocolsAndDuplicateParameters()
{
    char                 url[] = "spinel+exec:///path/to/ot-rcp?arg=1&arg=arg2&arg=3";
    ot::Posix::Arguments args(url);
    const char *         arg = NULL;

    assert(!strcmp(args.GetPath(), "/path/to/ot-rcp"));

    arg = args.GetValue("arg");
    assert(!strcmp(arg, "1"));

    arg = args.GetValue("arg", arg);
    assert(!strcmp(arg, "arg2"));

    arg = args.GetValue("arg", arg);
    assert(!strcmp(arg, "3"));
}

int main()
{
    TestSimple();
    TestSimpleNoArguments();
    TestMultipleProtocols();
    TestMultipleProtocolsAndDuplicateParameters();
    printf("All tests passed!\r\n");
}
#endif
