/*
 *  Copyright (c) 2018, The OpenThread Authors.
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
 * @brief
 *   This file defines the platform netif interface for OpenThread.
 *
 */

#ifndef OPENTHREAD_PLATFORM_NETIF_H_
#define OPENTHREAD_PLATFORM_NETIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <openthread/instance.h>
#include <openthread/ip6.h>

typedef enum {
    OT_PLAT_NETIF_EVENT_UP,           ///< The interface is up.
    OT_PLAT_NETIF_EVENT_DOWN,         ///< The interface is down.
    OT_PLAT_NETIF_EVENT_ADDR_ADDED,   ///< An IPv6 address is added.
    OT_PLAT_NETIF_EVENT_ADDR_REMOVED, ///< An IPv6 address is removed.
} otPlatNetifEventType;

typedef struct
{
    otPlatNetifEventType mType;
    union
    {
        otNetifAddress mNetifAddress;
    } mData;
} otPlatNetifEvent;

/**
 * A function pointer called on a network interface event happened.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 * @param [in]  aEvent      A pointer to the event just happened.
 *
 */
typedef void (*otPlatNetifEventHandler)(otInstance *aInstance, const otPlatNetifEvent *aEvent);

/**
 * This function initializes the platform network interface.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 * @param[in]   aHandler    The function pointer used to notify network interface event.
 *
 */
void otPlatNetifInit(otInstance *aInstance, otPlatNetifEventHandler aHandler);

/**
 * This function turns up the platform network interface.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 *
 * @retval  OT_ERROR_NONE   The platform network interface is successfully set up.
 * @retval  OT_ERROR_FAILED Failed to set up the network interface.
 *
 */
otError otPlatNetifUp(otInstance *aInstance);

/**
 * This function turns down the platform network interface.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 *
 * @retval  OT_ERROR_NONE   The platform network interface is successfully turned down.
 * @retval  OT_ERROR_FAILED Failed to turn down the network interface.
 *
 */
otError otPlatNetifDown(otInstance *aInstance);

/**
 * This function adds @p aNetifAddress to the platform network interface.
 *
 * @param[in]   aInstance       A pointer to the OpenThread interface.
 * @param[in]   aNetifAddress   A pointer to the address to be added.
 *
 * @retval  OT_ERROR_NONE   The platform network interface is successfully turned down.
 * @retval  OT_ERROR_FAILED Failed to turn down the network interface.
 *
 */
otError otPlatNetifAddAddress(otInstance *aInstance, const otNetifAddress *aNetifAddress);

/**
 * This function removes @p aNetifAddress from the platform network interface.
 *
 * @param[in]   aInstance       A pointer to the OpenThread interface.
 * @param[in]   aNetifAddress   A pointer to the address to be added.
 *
 * @retval  OT_ERROR_NONE   The platform network interface is successfully turned down.
 * @retval  OT_ERROR_FAILED Failed to turn down the network interface.
 *
 */
otError otPlatNetifRemoveAddress(otInstance *aInstance, const otNetifAddress *aNetifAddress);

/**
 * This function delivers a IPv6 packet to the platform network interface.
 *
 * @note This function takes ownership of @p aMessage.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 * @param[in]   aMessage    A pointer to the IPv6 packet.
 *
 */
void otPlatNetifReceive(otInstance *aInstance, otMessage *aMessage);

/**
 * This function sets the link address of the platform network interface.
 *
 * @param[in]   aInstance   A pointer to the OpenThread interface.
 * @param[in]   aExtAddress A pointer to link address.
 *
 */
void otPlatNetifSetLinkAddress(otInstance *aInstance, const otExtAddress *aExtAddress);

#ifdef __cplusplus
} // end of extern "C"
#endif

#endif // OPENTHREAD_PLATFORM_NETIF_H_
