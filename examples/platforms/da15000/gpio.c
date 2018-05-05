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

#include <openthread/types.h>
#include <openthread/platform/gpio.h>

#include "platform-da15000.h"
#include "hw_gpio.h"

static uint8_t mRedLedOutput   = GPIO_LOGIC_LOW;
static uint8_t mGreenLedOutput = GPIO_LOGIC_LOW;
static uint8_t mBlueLedOutput  = GPIO_LOGIC_LOW;

void da15000GpioInit()
{
    // configure GPIO mode: output and initial pin state as low
    hw_gpio_set_pin_function(LED_GPIO_PORT, RED_LED_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
    hw_gpio_set_pin_function(LED_GPIO_PORT, GREEN_LED_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);
    hw_gpio_set_pin_function(LED_GPIO_PORT, BLUE_LED_PIN, HW_GPIO_MODE_OUTPUT, HW_GPIO_FUNC_GPIO);

    hw_gpio_set_inactive(LED_GPIO_PORT, RED_LED_PIN);
    hw_gpio_set_inactive(LED_GPIO_PORT, GREEN_LED_PIN);
    hw_gpio_set_inactive(LED_GPIO_PORT, BLUE_LED_PIN);

    mRedLedOutput   = GPIO_LOGIC_LOW;
    mGreenLedOutput = GPIO_LOGIC_LOW;
    mBlueLedOutput  = GPIO_LOGIC_LOW;
}

void otPlatGpioSet(uint32_t port, uint8_t pin)
{
    hw_gpio_set_active(port, pin);
}

void otPlatGpioClear(uint32_t port, uint8_t pin)
{
    hw_gpio_set_inactive(port, pin);
}
