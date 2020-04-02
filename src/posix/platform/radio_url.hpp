#ifndef POSIX_PLATFORM_RADIO_URL_HPP_
#define POSIX_PLATFORM_RADIO_URL_HPP_

#include <openthread/radio_driver.h>

struct otPosixRadioArguments
{
    const char *mDevice;
    char *      mStart;
    char *      mEnd;
};

namespace ot {
namespace Posix {
class Arguments : public otPosixRadioArguments
{
public:
    Arguments(char *aUrl);
    const char *GetPath(void) const { return mDevice; }
    const char *GetValue(const char *aName, const char *aLastValue = NULL);
};

} // namespace Posix
} // namespace ot

#endif // POSIX_PLATFORM_RADIO_URL_HPP_
