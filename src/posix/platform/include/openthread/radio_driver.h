#ifndef OT_POSIX_RADIO_DRIVER_H_
#define OT_POSIX_RADIO_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include <openthread/error.h>

#include "openthread-system.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct otPosixRadioInstance otPosixRadioInstance;

typedef struct otPosixRadioOperations
{
    bool (*IsPromiscuous)(otPosixRadioInstance *aInstance);
} otPosixRadioOperations;

typedef otError (*otPosixDataCallback)(void *aContext, const uint8_t *aBuffer, uint16_t aLength);

typedef struct otPosixRadioDataFuncs
{
    void (*SetCallback)(otPosixRadioInstance *aInstance, otPosixDataCallback aCallback, void *aContext);
    otError (*Write)(otPosixRadioInstance *aInstance, const uint8_t *aBuffer, uint16_t aLength);
    otError (*Wait)(otPosixRadioInstance *aContext, uint32_t aTimeout);
} otPosixRadioDataFuncs;

typedef struct otPosixRadioPollFuncs
{
    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]    aMainloop   A reference to the mainloop context.
     *
     */
    void (*mPoll)(otPosixRadioInstance *aInstance, otSysMainloopContext *aMainloop);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]       aMainloop   A reference to the mainloop context.
     *
     */
    void (*Process)(otPosixRadioInstance *aInstance, const otSysMainloopContext *aMainloop);
} otPosixRadioPollFuncs;

typedef struct otPosixRadioArguments otPosixRadioArguments;

const char *otPosixRadioArgumentsGetPath(otPosixRadioArguments *aArguments);

const char *otPosixRadioArgumentsGetValue(otPosixRadioArguments *aArguments, const char *aName);
const char *otPosixRadioArgumentsGetValues(otPosixRadioArguments *aArguments, const char *aName, const char *aLast);

typedef struct otPosixRadioDriver
{
    struct otPosixRadioDriver *mNext;

    const char *mName;

    otPosixRadioInstance *(*Create)(otPosixRadioArguments *aArguments, otPosixRadioInstance *aNext);
    otError (*Delete)(otPosixRadioInstance *aInstance);

    const otPosixRadioOperations *mOperations;
    const otPosixRadioDataFuncs * mData;
    const otPosixRadioPollFuncs * mPoll;
} otPosixRadioDriver;

struct otPosixRadioInstance
{
    otPosixRadioDriver *mDriver;
};

otError otPosixModuleInit(void);

extern otError otPosixRadioDriverRegister(otPosixRadioDriver *aDriver);

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif // OT_POSIX_RADIO_DRIVER_H_
