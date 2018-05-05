/*
 *  Copyright (c) 2017, The OpenThread Authors.
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

/**
 * @file
 *   This file implements the OpenThread platform abstraction for GPIO.
 *
 */

#include <common/code_utils.hpp>
#include <openthread/types.h>
#include <openthread/platform/gpio.h>
#include <utils/code_utils.h>

#include "platform-cc2650.h"

#include "driverlib/gpio.h"
#include "driverlib/ioc.h"
#include "driverlib/prcm.h"

static uint8_t mOutPut = GPIO_LOGIC_LOW;

void cc2650GpioInit(void)
{
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);

    while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);

    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_GPIO);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();

    while (!PRCMLoadGet());

    IOCPinTypeGpioOutput(IOID_25);
    IOCPinTypeGpioOutput(IOID_26);
    IOCPinTypeGpioOutput(IOID_27);

    // clear output first
    GPIO_writeDio(RED_LED_PIN, GPIO_LOGIC_LOW);
    GPIO_writeDio(GREEN_LED_PIN, GPIO_LOGIC_LOW);
    GPIO_writeDio(BLUE_LED_PIN, GPIO_LOGIC_LOW);
}

void otPlatGpioSet(uint32_t port, uint8_t pin)
{
    (void)port;

    otEXPECT(pin == GREEN_LED_PIN);

    mOutPut = GPIO_LOGIC_HIGH;
    GPIO_writeDio(pin, GPIO_LOGIC_HIGH);

exit:
    return;
}

void otPlatGpioClear(uint32_t port, uint8_t pin)
{
    (void)port;

    otEXPECT(pin == GREEN_LED_PIN);

    mOutPut = GPIO_LOGIC_LOW;
    GPIO_writeDio(pin, GPIO_LOGIC_LOW);

exit:
    return;
}

void otPlatGpioToggle(uint32_t port, uint8_t pin)
{
    (void)port;

    otEXPECT(pin == GREEN_LED_PIN);

    mOutPut = mOutPut ? GPIO_LOGIC_LOW : GPIO_LOGIC_HIGH;
    GPIO_toggleDio(pin);

exit:
    return;
}

uint8_t otPlatGpioGet(uint32_t port, uint8_t pin)
{
    (void)port;

    // for cc2650, only GREEN LED would be used
    otEXPECT_ACTION(pin == GREEN_LED_PIN, mOutPut = GPIO_LOGIC_LOW);

exit:
    return mOutPut;
}
