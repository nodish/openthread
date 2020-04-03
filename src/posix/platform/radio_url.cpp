#include "posix/platform/radio_url.hpp"

#include <stdio.h>
#include <string.h>

#include "core/common/code_utils.hpp"
#include "posix/platform/platform-posix.h"

namespace ot {
namespace Posix {

Arguments::Arguments(char *aUrl)
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

const char *Arguments::GetValue(const char *aName, const char *aLastValue)
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

} // namespace Posix
} // namespace ot

const char *otPosixRadioArgumentsGetPath(otPosixRadioArguments *aArguments)
{
    return static_cast<ot::Posix::Arguments *>(aArguments)->GetPath();
}

const char *otPosixRadioArgumentsGetValue(otPosixRadioArguments *aArguments, const char *aName, const char *aLast)
{
    return static_cast<ot::Posix::Arguments *>(aArguments)->GetValue(aName, aLast);
}

#if OT_UNIT_TEST_RADIO_URL
#include <assert.h>

void TestSimple()
{
    char                 url[] = "spinel:///dev/ttyUSB0?baudrate=115200";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));
    assert(!strcmp(args.GetValue("baudrate"), "115200"));

    printf("PASS %s\r\n", __func__);
}

void TestSimpleNoArguments()
{
    char                 url[] = "spinel:///dev/ttyUSB0";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));

    printf("PASS %s\r\n", __func__);
}

void TestMultipleProtocols()
{
    char                 url[] = "spinel+spi:///dev/ttyUSB0?baudrate=115200";
    ot::Posix::Arguments args(url);

    assert(!strcmp(args.GetPath(), "/dev/ttyUSB0"));
    assert(!strcmp(args.GetValue("baudrate"), "115200"));

    printf("PASS %s\r\n", __func__);
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

    printf("PASS %s\r\n", __func__);
}

void TestPublicAPI()
{
    char                 url[] = "spinel+exec:///path/to/ot-rcp?arg=1&arg=arg2&arg=3";
    ot::Posix::Arguments args(url);
    const char *         arg = NULL;

    assert(!strcmp(otPosixRadioArgumentsGetPath(&args), "/path/to/ot-rcp"));

    arg = otPosixRadioArgumentsGetValue(&args, "arg", NULL);
    assert(!strcmp(arg, "1"));

    arg = otPosixRadioArgumentsGetValue(&args, "arg", arg);
    assert(!strcmp(arg, "arg2"));

    arg = otPosixRadioArgumentsGetValue(&args, "arg", arg);
    assert(!strcmp(arg, "3"));

    printf("PASS %s\r\n", __func__);
}

int main()
{
    TestSimple();
    TestSimpleNoArguments();
    TestMultipleProtocols();
    TestMultipleProtocolsAndDuplicateParameters();
    TestPublicAPI();

    return 0;
}
#endif // OT_UNIT_TEST_RADIO_URL
