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

#include "openthread-core-config.h"

#include <openthread/types.h>
#include <openthread/platform/gpio.h>

#include "em_gpio.h"
#include "em_cmu.h"
#include "platform-efr32.h"

#ifdef OPENTHREAD_ENABLE_JOINER
#include "bspconfig.h"
#include "gpiointerrupt.h"
#include <openthread/instance.h>

void gpioCallback(uint8_t pin);
#endif

void efr32GpioInit(void)
{
    // enable GPIO clock first if not
    CMU_ClockEnable(cmuClock_GPIO, true);

#ifdef EFR32MG12P432F1024GL125 // Wireless Started Kit
    // configure the gpio mode: output or input
    GPIO_PinModeSet(gpioPortD, RED_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
    GPIO_PinModeSet(gpioPortD, GREEN_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
    GPIO_PinModeSet(gpioPortD, BLUE_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
#else                         // Thunderboard Sense2
    // configure the gpio mode: output or input
    GPIO_PinModeSet(RGB_ENABLE_PORT, RGB_ENABLE_GPIO, gpioModePushPull, GPIO_LOGIC_HIGH);
    GPIO_PinModeSet(RGB_SELECT_PORT, RGB_SELECT_GPIO0, gpioModePushPull, GPIO_LOGIC_LOW);
    GPIO_PinModeSet(RGB_SELECT_PORT, RGB_SELECT_GPIO1, gpioModePushPull, GPIO_LOGIC_HIGH);
    GPIO_PinModeSet(RGB_SELECT_PORT, RGB_SELECT_GPIO2, gpioModePushPull, GPIO_LOGIC_HIGH);
    GPIO_PinModeSet(RGB_SELECT_PORT, RGB_SELECT_GPIO3, gpioModePushPull, GPIO_LOGIC_LOW);

    GPIO_PinModeSet(gpioPortD, RED_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
    GPIO_PinModeSet(gpioPortD, GREEN_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
    GPIO_PinModeSet(gpioPortD, BLUE_LED_PIN, gpioModePushPull, GPIO_LOGIC_LOW);
#endif

#ifdef OPENTHREAD_ENABLE_JOINER
    // configure PF6 as button interrupt
    GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInputPull, 1);

    GPIOINT_Init();
    GPIOINT_CallbackRegister(BSP_GPIO_PB0_PIN, gpioCallback);
    GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);
#endif
}

#ifdef OPENTHREAD_ENABLE_JOINER
void gpioCallback(uint8_t pin)
{
    // factoryreset
//    otInstanceFactoryReset(sInstance);
}
#endif

void otPlatGpioSet(uint32_t port, uint8_t pin)
{
    // must configure mode first

    GPIO_PinOutSet(port, pin);
}

void otPlatGpioClear(uint32_t port, uint8_t pin)
{
    // must configure mode first

    GPIO_PinOutClear(port, pin);
}

void otPlatGpioToggle(uint32_t port, uint8_t pin)
{
    // must configure mode filst
    GPIO_PinOutToggle(port, pin);
}

uint8_t otPlatGpioGet(uint32_t port, uint8_t pin)
{
    return GPIO_PinOutGet(port, pin);
}
