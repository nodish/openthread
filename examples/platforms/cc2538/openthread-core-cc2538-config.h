/*
 *  Copyright (c) 2016, The OpenThread Authors.
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
 *   This file includes cc2538 compile-time configuration constants for OpenThread.
 */

#ifndef OPENTHREAD_CORE_CC2538_CONFIG_H_
#define OPENTHREAD_CORE_CC2538_CONFIG_H_

/**
 * @def OPENTHREAD_CONFIG_PLATFORM_INFO
 *
 * The platform-specific string to insert into the OpenThread version string.
 *
 */
#define OPENTHREAD_CONFIG_PLATFORM_INFO                        "CC2538"

/**
 * @def OPENTHREAD_CONFIG_ENABLE_SOFTWARE_ACK_TIMEOUT
 *
 * Define to 1 if you want to enable software ACK timeout logic.
 *
 */
#define OPENTHREAD_CONFIG_ENABLE_SOFTWARE_ACK_TIMEOUT          1

/**
 * @def OPENTHREAD_CONFIG_ENABLE_SOFTWARE_RETRANSMIT
 *
 * Define to 1 if you want to enable software retransmission logic.
 *
 */
#define OPENTHREAD_CONFIG_ENABLE_SOFTWARE_RETRANSMIT           1

/**
 * @def OPENTHREAD_CONFIG_ENABLE_SOFTWARE_ENERGY_SCAN
 *
 * Define to 1 if you want to enable software energy scanning logic.
 *
 */
#define OPENTHREAD_CONFIG_ENABLE_SOFTWARE_ENERGY_SCAN          1

/**
 * @def SETTINGS_CONFIG_BASE_ADDRESS
 *
 * The actual physical address used for the cc2538 is set by the
 * linker file, the value here is "relative to the base address" set
 * in the linker file.
 */
#define SETTINGS_CONFIG_BASE_ADDRESS                           0

/**
 * @def The CC2538 linker script sets aside 2 pages.
 */
#define SETTINGS_CONFIG_PAGE_NUM                               2

/**
 * @def The page size of settings, 2K bytes
 */
#define SETTINGS_CONFIG_PAGE_SIZE                              2048

/* GPIO configuration for demo */
#define GPIO_FALLING_EDGE       0x00000000  // Interrupt on falling edge
#define GPIO_RISING_EDGE        0x00000004  // Interrupt on rising edge
#define GPIO_BOTH_EDGES         0x00000001  // Interrupt on both edges
#define GPIO_LOW_LEVEL          0x00000002  // Interrupt on low level
#define GPIO_HIGH_LEVEL         0x00000007  // Interrupt on high level

#define GPIO_A_BASE     0x400D9000  // GPIO_A
#define GPIO_B_BASE     0x400DA000  // GPIO_B
#define GPIO_C_BASE     0x400DB000  // GPIO_C 
#define GPIO_D_BASE     0x400DC000  // GPIO_D

#define LED_GPIO_PORT   GPIO_D_BASE // GPIO_PORT_D
#define RED_LED_PIN     0x00000020  // pin 5
#define GREEN_LED_PIN   0x00000010  // pin 4
#define BLUE_LED_PIN    0x00000008  // pin 3

#endif  // OPENTHREAD_CORE_CC2538_CONFIG_H_
