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

#include <assert.h>
#include <string.h>

#include <common/code_utils.hpp>
#include <openthread/types.h>
#include <openthread/platform/gpio.h>

#include "platform-cc2538.h"
#include "cc2538-reg.h"
#include "hw_gpio.h"
#include "hw_ioc.h"

#define IOC_OVERRIDE_DIS  0x00000000    // PAD Config Override Disabled
#define IOC_OVERRIDE_ANA  0x00000001    // PAD Config Override Analog Enable
#define IOC_OVERRIDE_PDE  0x00000002    // PAD Config Override Pull Down Enable
#define IOC_OVERRIDE_PUE  0x00000004    // PAD Config Override Pull Up Enable
#define IOC_OVERRIDE_OE   0x00000008    // PAD Config Override Output Enable

static bool GPIOBaseValid(uint32_t ui32Port);

static const uint32_t g_pui32IOCPortAOverrideReg[] =
{
    IOC_PA0_OVER, IOC_PA1_OVER, IOC_PA2_OVER, IOC_PA3_OVER,
    IOC_PA4_OVER, IOC_PA5_OVER, IOC_PA6_OVER, IOC_PA7_OVER
};

static const uint32_t g_pui32IOCPortBOverrideReg[] =
{
    IOC_PB0_OVER, IOC_PB1_OVER, IOC_PB2_OVER, IOC_PB3_OVER,
    IOC_PB4_OVER, IOC_PB5_OVER, IOC_PB6_OVER, IOC_PB7_OVER
};

static const uint32_t g_pui32IOCPortCOverrideReg[] =
{
    IOC_PC0_OVER, IOC_PC1_OVER, IOC_PC2_OVER, IOC_PC3_OVER,
    IOC_PC4_OVER, IOC_PC5_OVER, IOC_PC6_OVER, IOC_PC7_OVER
};

static const uint32_t g_pui32IOCPortDOverrideReg[] =
{
    IOC_PD0_OVER, IOC_PD1_OVER, IOC_PD2_OVER, IOC_PD3_OVER,
    IOC_PD4_OVER, IOC_PD5_OVER, IOC_PD6_OVER, IOC_PD7_OVER
};

static void SysCtrlIOClockSet(uint32_t ui32IODiv)
{
    uint32_t ui32RegVal;

    // check input parameters
    assert(ui32IODiv == SYS_CTRL_SYSDIV_32MHZ  ||
           ui32IODiv == SYS_CTRL_SYSDIV_16MHZ  ||
           ui32IODiv == SYS_CTRL_SYSDIV_8MHZ   ||
           ui32IODiv == SYS_CTRL_SYSDIV_4MHZ   ||
           ui32IODiv == SYS_CTRL_SYSDIV_2MHZ   ||
           ui32IODiv == SYS_CTRL_SYSDIV_1MHZ   ||
           ui32IODiv == SYS_CTRL_SYSDIV_500KHZ ||
           ui32IODiv == SYS_CTRL_SYSDIV_250KHZ);

    ui32RegVal = HWREG(SYS_CTRL_CLOCK_CTRL);
    ui32RegVal &= ~SYS_CTRL_CLOCK_CTRL_IO_DIV_M;
    ui32RegVal |= (ui32IODiv << SYS_CTRL_CLOCK_CTRL_IO_DIV_S);
    HWREG(SYS_CTRL_CLOCK_CTRL) = ui32RegVal;
} // SysCtrlIOClockSet


static void IOCPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins,
                            uint32_t ui32PinDrive)
{
    uint32_t ui32OverrideRegAddr;
    uint32_t ui32PinNo;
    uint32_t ui32PinBit;

    // Check the arguments
    assert((ui32Port == GPIO_A_BASE) || (ui32Port == GPIO_B_BASE) ||
           (ui32Port == GPIO_C_BASE) || (ui32Port == GPIO_D_BASE));
    assert(ui8Pins != 0);
    assert((ui32PinDrive == IOC_OVERRIDE_OE)  ||
           (ui32PinDrive == IOC_OVERRIDE_PUE) ||
           (ui32PinDrive == IOC_OVERRIDE_PDE) ||
           (ui32PinDrive == IOC_OVERRIDE_ANA) ||
           (ui32PinDrive == IOC_OVERRIDE_DIS));
    // PC0-PC3 does not support on-die pullup, pulldown or analog connectivity.
    assert(!((ui32Port == GPIO_C_BASE) && ((ui8Pins & 0xf) > 0) &&
             ((ui32PinDrive == IOC_OVERRIDE_PUE) ||
              (ui32PinDrive == IOC_OVERRIDE_PDE) ||
              (ui32PinDrive == IOC_OVERRIDE_ANA))));

    // Initialize to default value
    ui32OverrideRegAddr = IOC_PA0_SEL;

    // Look for specified port pins to be configured, multiple pins are allowed
    for (ui32PinNo = 0; ui32PinNo < 8; ui32PinNo++)
    {
        ui32PinBit = (ui8Pins >> ui32PinNo) & 0x00000001;

        if (ui32PinBit != 0)
        {
            // Find register addresses for configuring specified port pin
            switch (ui32Port)
            {
            case GPIO_A_BASE:
                ui32OverrideRegAddr = g_pui32IOCPortAOverrideReg[ui32PinNo];
                break;

            case GPIO_B_BASE:
                ui32OverrideRegAddr = g_pui32IOCPortBOverrideReg[ui32PinNo];
                break;

            case GPIO_C_BASE:
                ui32OverrideRegAddr = g_pui32IOCPortCOverrideReg[ui32PinNo];
                break;

            case GPIO_D_BASE:
                ui32OverrideRegAddr = g_pui32IOCPortDOverrideReg[ui32PinNo];
                break;

            default:
                // Default to port A pin 0
                ui32OverrideRegAddr = IOC_PA0_OVER;
                break;
            }

            // Set desired pin drive for the desired port pin
            HWREG(ui32OverrideRegAddr) = ui32PinDrive;
        }
    }
}

static bool GPIOBaseValid(uint32_t ui32Port)
{
    return ((ui32Port == GPIO_A_BASE) || (ui32Port == GPIO_B_BASE) ||
            (ui32Port == GPIO_C_BASE) || (ui32Port == GPIO_D_BASE));
}

static void GPIODirModeSet(uint32_t ui32Port, uint8_t ui8Pins,
                           uint32_t ui32PinIO)
{

    // Check the arguments.
    assert(GPIOBaseValid(ui32Port));
    assert((ui32PinIO == GPIO_DIR_MODE_IN) || (ui32PinIO == GPIO_DIR_MODE_OUT) ||
           (ui32PinIO == GPIO_DIR_MODE_HW));

    // Set the pin direction and mode.
    HWREG(ui32Port + GPIO_O_DIR)   = ((ui32PinIO & GPIO_DIR_MODE_OUT) ?
                                      (HWREG(ui32Port + GPIO_O_DIR) | ui8Pins) :
                                      (HWREG(ui32Port + GPIO_O_DIR) & ~(ui8Pins)));
    HWREG(ui32Port + GPIO_O_AFSEL) = ((ui32PinIO & GPIO_DIR_MODE_HW) ?
                                      (HWREG(ui32Port + GPIO_O_AFSEL) | ui8Pins) :
                                      (HWREG(ui32Port + GPIO_O_AFSEL) & ~(ui8Pins)));
}

static uint32_t GPIOPinRead(uint32_t ui32Port, uint8_t ui8Pins)
{
    // Check the arguments.
    assert(GPIOBaseValid(ui32Port));

    // Return the pin value(s).
    return (HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))));
}

static void GPIOPinWrite(uint32_t ui32Port, uint8_t ui8Pins, uint8_t ui8Val)
{
    // Check the arguments.
    assert(GPIOBaseValid(ui32Port));

    // Write the pins.
    HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = ui8Val;
}

static void GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
{
    // Check the arguments.
    assert(GPIOBaseValid(ui32Port));

    // Make the pin(s) be outputs.
    GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_OUT);

    // Set the pad(s) no override of the drive type.
    IOCPadConfigSet(ui32Port, ui8Pins, IOC_OVERRIDE_DIS);
}

void cc2538GpioInit(void)
{
    // Set IO clock to the same as system clock
    SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);

    // configure the mode of GPIO: input or output
    // enable pull-push mode
    GPIOPinTypeGPIOOutput(LED_GPIO_PORT, RED_LED_PIN);
    IOCPadConfigSet(LED_GPIO_PORT, RED_LED_PIN, IOC_OVERRIDE_PUE);

    GPIOPinTypeGPIOOutput(LED_GPIO_PORT, BLUE_LED_PIN);
    IOCPadConfigSet(LED_GPIO_PORT, BLUE_LED_PIN, IOC_OVERRIDE_PUE);

    GPIOPinTypeGPIOOutput(LED_GPIO_PORT, GREEN_LED_PIN);
    IOCPadConfigSet(LED_GPIO_PORT, GREEN_LED_PIN, IOC_OVERRIDE_PUE);

    // clear LEDs
    otPlatGpioClear(LED_GPIO_PORT, RED_LED_PIN);
    otPlatGpioClear(LED_GPIO_PORT, GREEN_LED_PIN);
    otPlatGpioClear(LED_GPIO_PORT, BLUE_LED_PIN);
}

void otPlatGpioSet(uint32_t port, uint8_t pin)
{
    int8_t value = (uint8_t)GPIOPinRead(port, pin);

    value |= pin;

    GPIOPinWrite(port, pin, value);
}

void otPlatGpioClear(uint32_t port, uint8_t pin)
{
    int8_t value = (uint8_t)GPIOPinRead(port, pin);

    value &= ~pin;

    GPIOPinWrite(port, pin, value);
}

uint8_t otPlatGpioGet(uint32_t port, uint8_t pin)
{
    uint8_t value = (uint8_t)GPIOPinRead(port, pin);

    return value > 0 ? GPIO_LOGIC_HIGH : GPIO_LOGIC_LOW;
}
