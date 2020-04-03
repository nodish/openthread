#ifndef OT_POSIX_PLATFORM_MODULE_MANAGER_HPP_
#define OT_POSIX_PLATFORM_MODULE_MANAGER_HPP_

#include "posix/platform/radio_url.hpp"

#define OT_POSIX_MODULE_DISABLE 0
#define OT_POSIX_MODULE_BUILTIN 1
#define OT_POSIX_MODULE_DYNAMIC 2

namespace ot {
namespace Posix {

class ModuleManager
{
public:
    ModuleManager(void);

    otError Register(otPosixRadioDriver *aDriver);

    otPosixRadioInstance *CreateInstance(char *aUrl);

    otPosixRadioInstance *CreateInstance(char *aProto, Arguments &aArguments);

private:
    otPosixRadioDriver *Find(const char *aName);

    otPosixRadioDriver *mDriverList;
    otPosixRadioDriver *mDriverUnused;
};

} // namespace Posix
} // namespace ot

#endif // OT_POSIX_PLATFORM_MODULE_MANAGER_HPP_

