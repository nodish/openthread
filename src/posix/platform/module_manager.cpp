#include <string.h>

#include "core/common/code_utils.hpp"
#include "posix/platform/platform-posix.h"
#include "posix/platform/radio_url.hpp"

namespace ot {
namespace Posix {

class ModuleManager
{
public:
    ModuleManager()
        : mDriverList(NULL)
        , mDriverUnused(NULL)
    {
#if 0
        // Try built-in drivers first
        if (!strcmp(aProto, kDriverSpinel))
        {
            driver = Driver::Create<kDriverSpinel>(aDevice, aArguments, lower);
        }
        else if (!strcmp(aProto, kDriverHdlc))
        {
            driver = Driver::Create<kDriverHdlc>(aDevice, aArguments, lower);
        }
        else if (!strcmp(aProto, kDriverSpi))
        {
            driver = Driver::Create<kDriverSpi>(aDevice, aArguments, lower);
        }
        else if (!strcmp(aProto, kDriverUart))
        {
            driver = Driver::Create<kDriverUart>(aDevice, aArguments, lower);
        }
        else if (!strcmp(aProto, kDriverForkpty))
        {
            driver = Driver::Create<kDriverForkpty>(aDevice, aArguments, lower);
        }
        else
        {
            driver = NULL;
        }

        if (nextProto == NULL)
        {
            sBottomDriver = static_cast<BottomDriver *>(driver);
        }

        return driver;
#endif
    }

    otError Register(otPosixRadioDriver *aDriver)
    {
        otError rval = OT_ERROR_NONE;

        for (otPosixRadioDriver *driver = mDriverList; driver != NULL; driver = driver->mNext)
        {
            if (driver == aDriver)
            {
                ExitNow(rval = OT_ERROR_ALREADY);
            }
        }

        aDriver->mNext = mDriverList;
        mDriverList    = aDriver;
        mDriverUnused  = aDriver;

    exit:
        return rval;
    }

    otPosixRadioInstance *CreateInstance(char *aUrl)
    {
        ot::Posix::Arguments args(aUrl);

        return CreateInstance(aUrl, args);
    }

    otPosixRadioInstance *CreateInstance(char *aProto, Arguments &aArguments)
    {
        char *                nextProto = strstr(aProto, "+");
        otPosixRadioInstance *instance  = NULL;
        otPosixRadioInstance *next      = NULL;
        otPosixRadioDriver *  driver    = NULL;

        if (nextProto != NULL)
        {
            nextProto[0] = '\0';
            nextProto += sizeof("+") - 1;
            next = CreateInstance(nextProto, aArguments);
        }

        driver = Find(aProto);
        VerifyOrDie(driver != NULL, OT_EXIT_INVALID_ARGUMENTS);

        instance = driver->Create(&aArguments, next);
        VerifyOrDie(instance != NULL && instance->mDriver == driver, OT_EXIT_INVALID_ARGUMENTS);

        if (mDriverUnused == driver)
        {
            mDriverUnused = mDriverUnused->mNext;
        }
        else
        {
            for (otPosixRadioDriver *prev = mDriverUnused; prev->mNext != NULL; prev = prev->mNext)
            {
                if (prev->mNext == driver)
                {
                    prev->mNext   = driver->mNext;
                    driver->mNext = mDriverList;
                    mDriverList   = driver;
                    break;
                }
            }
        }

        return instance;
    }

private:
    otPosixRadioDriver *Find(const char *aName)
    {
        otPosixRadioDriver *rval = NULL;

        for (otPosixRadioDriver *driver = mDriverList; driver != NULL; driver = driver->mNext)
        {
            if (!strcmp(aName, driver->mName))
            {
                rval = driver;
                break;
            }
        }

        return rval;
    }

    otPosixRadioDriver *mDriverList;
    otPosixRadioDriver *mDriverUnused;
};

otError otPosixRadioDriverRegister(void *aContext, otPosixRadioDriver *aDriver)
{
    return static_cast<ModuleManager *>(aContext)->Register(aDriver);
}

} // namespace Posix
} // namespace ot

#if OT_UNIT_TEST_MODULE_MANAGER

#include <assert.h>
#include <stdio.h>

static otPosixRadioInstance *CreateDummy1(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext);
static otPosixRadioInstance *CreateDummy2(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext);

static otPosixRadioDriver sDummy1 =
    {.mName = "dummy1", .Create = CreateDummy1, .Delete = NULL, .mOperations = NULL, .mData = NULL, .mPoll = NULL};
static otPosixRadioDriver sDummy2 =
    {.mName = "dummy2", .Create = CreateDummy2, .Delete = NULL, .mOperations = NULL, .mData = NULL, .mPoll = NULL};

otPosixRadioInstance *CreateDummy1(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    otPosixRadioInstance *instance = new otPosixRadioInstance;

    instance->mDriver = &sDummy1;
    return instance;
}

otPosixRadioInstance *CreateDummy2(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext)
{
    otPosixRadioInstance *instance = new otPosixRadioInstance;

    instance->mDriver = &sDummy2;
    return instance;
}

void TestSingle()
{
    ot::Posix::ModuleManager manager;
    char                     url[] = "dummy1:///dev/ttyUSB0";

    assert(OT_ERROR_NONE == otPosixRadioDriverRegister(&manager, &sDummy1));

    otPosixRadioInstance *instance = manager.CreateInstance(url);
    assert(instance != NULL);
    assert(!strcmp(instance->mDriver->mName, "dummy1"));

    printf("PASS %s\r\n", __func__);
}

void TestDual()
{
    ot::Posix::ModuleManager manager;
    char                     url[] = "dummy1+dummy2:///dev/ttyUSB0";

    assert(OT_ERROR_NONE == otPosixRadioDriverRegister(&manager, &sDummy1));
    assert(OT_ERROR_NONE == otPosixRadioDriverRegister(&manager, &sDummy2));

    otPosixRadioInstance *instance = manager.CreateInstance(url);
    assert(instance != NULL);
    assert(!strcmp(instance->mDriver->mName, "dummy1"));

    printf("PASS %s\r\n", __func__);
}

int main()
{
    TestSingle();
    TestDual();
    return 0;
}
#endif // OT_UNIT_TEST_MODULE_MANAGER
