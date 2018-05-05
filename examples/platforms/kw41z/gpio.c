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

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "MKW41Z4.h"
#include "platform-kw41z.h"

otPlatGpioIntCallback mHandler;

static uint8_t mRedLedOutput   = GPIO_LOGIC_LOW;
static uint8_t mGreenLedOutput = GPIO_LOGIC_LOW;
static uint8_t mBlueLedOutput  = GPIO_LOGIC_LOW;

void kw41zGpioInit(void)
{
    gpio_pin_config_t config =
    {
        kGPIO_DigitalOutput,
        GPIO_LOGIC_HIGH,
    };

    // set pin mux as gpio
    PORT_SetPinMux(PORTC, RED_LED_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, GREEN_LED_PIN, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, BLUE_LED_PIN, kPORT_MuxAsGpio);

    // init gpio
    GPIO_PinInit(GPIOC, RED_LED_PIN, &config);
    GPIO_PinInit(GPIOA, GREEN_LED_PIN, &config);
    GPIO_PinInit(GPIOA, BLUE_LED_PIN, &config);

    mRedLedOutput   = GPIO_LOGIC_HIGH;
    mGreenLedOutput = GPIO_LOGIC_HIGH;
    mBlueLedOutput  = GPIO_LOGIC_HIGH;
}

void otPlatGpioSet(uint32_t port, uint8_t pin)
{
    (void)port;

    switch (pin)
    {
    case RED_LED_PIN:
        GPIO_WritePinOutput(GPIOC, pin, GPIO_LOGIC_LOW);
        mRedLedOutput = GPIO_LOGIC_LOW;
        break;

    case GREEN_LED_PIN:
        GPIO_WritePinOutput(GPIOA, pin, GPIO_LOGIC_LOW);
        mGreenLedOutput = GPIO_LOGIC_LOW;
        break;

    case BLUE_LED_PIN:
        GPIO_WritePinOutput(GPIOA, pin, GPIO_LOGIC_LOW);
        mBlueLedOutput = GPIO_LOGIC_LOW;
        break;

    default:
        break;
    }
}

void otPlatGpioClear(uint32_t port, uint8_t pin)
{
    (void)port;

    switch (pin)
    {
    case RED_LED_PIN:
        GPIO_WritePinOutput(GPIOC, pin, GPIO_LOGIC_HIGH);
        mRedLedOutput = GPIO_LOGIC_HIGH;
        break;

    case GREEN_LED_PIN:
        GPIO_WritePinOutput(GPIOA, pin, GPIO_LOGIC_HIGH);
        mGreenLedOutput = GPIO_LOGIC_HIGH;
        break;

    case BLUE_LED_PIN:
        GPIO_WritePinOutput(GPIOA, pin, GPIO_LOGIC_HIGH);
        mBlueLedOutput = GPIO_LOGIC_HIGH;
        break;

    default:
        break;
    }
}

uint8_t otPlatGpioGet(uint32_t port, uint8_t pin)
{
    (void)port;
    uint8_t rval = 0;

    switch (pin)
    {
    case RED_LED_PIN:
        rval = mRedLedOutput;
        break;

    case GREEN_LED_PIN:
        rval = mGreenLedOutput;
        break;

    case BLUE_LED_PIN:
        rval = mBlueLedOutput;
        break;

    default:
        break;
    }

    return rval;
}

void otPlatGpioIntEnable(uint32_t port, uint8_t pin)
{
    // PTC4: SW3 as interrupt input
    (void)port;

    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config =
    {
        kGPIO_DigitalInput,
        GPIO_LOGIC_LOW,
    };

    port_pin_config_t config;
    config.pullSelect = kPORT_PullUp;
    config.mux = kPORT_MuxAsGpio;
    PORT_SetPinConfig(PORTC, INTERRUPT_PIN, &config);

    PORT_SetPinInterruptConfig(PORTC, INTERRUPT_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(PORTB_PORTC_IRQn);
    GPIO_PinInit(GPIOC, INTERRUPT_PIN, &sw_config);
}

void otPlatGpioRegisterCallback(uint32_t port, otPlatGpioIntCallback aCallback)
{
    (void)port;

    mHandler = aCallback;
}

void otPlatGpioIntClear(uint32_t port, uint8_t pin)
{
    (void)port;

    GPIO_ClearPinsInterruptFlags(GPIOC, 1 << pin);
}

void PORTB_PORTC_IRQHandler(void)
{
    mHandler();
}
