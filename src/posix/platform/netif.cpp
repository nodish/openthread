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
 *   This file implements the spinel based radio transceiver.
 */

#include "platform-posix.h"

#include <arpa/inet.h>
#include <assert.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <linux/ipv6.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <openthread/message.h>
#include <openthread/platform/netif.h>

#include "common/code_utils.hpp"
#include "common/logging.hpp"

#if OPENTHREAD_ENABLE_PLATFORM_NETIF
static otPlatNetifEventHandler sEventHandler = NULL;
static otInstance *            sInstance     = NULL;
static int                     sTunFd        = -1;
static int                     sTunIndex     = 0;
static int                     sIpFd         = -1;
static int                     sNetlinkFd    = -1;
static char                    sTunName[IFNAMSIZ];
static const size_t            kMaxIp6Size   = 1536;
static otPlatNetifEvent *      sCurrentEvent = NULL;
static char                    ip6string[INET6_ADDRSTRLEN];

#ifndef OPENTHREAD_POSIX_TUN_DEVICE
#define OPENTHREAD_POSIX_TUN_DEVICE "/dev/net/tun"
#endif // OPENTHREAD_TUN_DEVICE

void otPlatNetifInit(otInstance *aInstance, otPlatNetifEventHandler aHandler)
{
    struct ifreq ifr;

    fprintf(stderr, "init plat netif\r\n");
    sIpFd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_IP);
    VerifyOrExit(sIpFd > 0);

    sNetlinkFd = socket(AF_NETLINK, SOCK_DGRAM, NETLINK_ROUTE);
    VerifyOrExit(sNetlinkFd > 0);

    {
        struct sockaddr_nl sa;

        memset(&sa, 0, sizeof(sa));
        sa.nl_family = AF_NETLINK;
        sa.nl_groups = RTMGRP_LINK | RTMGRP_IPV6_IFADDR;
        VerifyOrExit(bind(sNetlinkFd, reinterpret_cast<struct sockaddr *>(&sa), sizeof(sa)) == 0);
    }

    sTunFd = open(OPENTHREAD_POSIX_TUN_DEVICE, O_RDWR);
    VerifyOrExit(sTunFd > 0, otLogCritPlat(aInstance, "Unable to open tun device %s", OPENTHREAD_POSIX_TUN_DEVICE));

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TUN | IFF_NO_PI;
    strncpy(ifr.ifr_name, "wpan%d", IFNAMSIZ);

    VerifyOrExit(ioctl(sTunFd, TUNSETIFF, static_cast<void *>(&ifr)) == 0,
                 otLogCritPlat(aInstance, "Unable to configure tun device %s", OPENTHREAD_POSIX_TUN_DEVICE));
#if defined(ARPHRD_6LOWPAN)
    VerifyOrExit(ioctl(sTunFd, TUNSETLINK, ARPHRD_6LOWPAN) == 0,
                 otLogCritPlat(aInstance, "Unable to set link type of tun device %s", OPENTHREAD_POSIX_TUN_DEVICE));
#else
    VerifyOrExit(ioctl(sTunFd, TUNSETLINK, ARPHRD_ETHER) == 0,
                 otLogCritPlat(aInstance, "Unable to set link type of tun device %s", OPENTHREAD_POSIX_TUN_DEVICE));
#endif

    sTunIndex = if_nametoindex(ifr.ifr_name);
    VerifyOrExit(sTunIndex > 0);

    strncpy(sTunName, ifr.ifr_name, sizeof(sTunName));
    // set for platform UDP
    setenv("PLATFORM_NETIF", sTunName, 1);

    sInstance     = aInstance;
    sEventHandler = aHandler;

exit:
    if (sTunIndex == 0)
    {
        if (sTunFd != -1)
        {
            close(sTunFd);
            sTunFd = -1;
        }

        if (sIpFd != -1)
        {
            close(sIpFd);
            sIpFd = -1;
        }

        if (sNetlinkFd != -1)
        {
            close(sNetlinkFd);
            sNetlinkFd = -1;
        }
        exit(EXIT_FAILURE);
    }
}

otError otPlatNetifUp(otInstance *aInstance)
{
    otError      error = OT_ERROR_NONE;
    struct ifreq ifr   = {};

    OT_UNUSED_VARIABLE(aInstance);

    VerifyOrExit(sIpFd > 0);
    strncpy(ifr.ifr_name, sTunName, sizeof(ifr.ifr_name));
    VerifyOrExit(ioctl(sIpFd, SIOCGIFFLAGS, &ifr) == 0, error = OT_ERROR_FAILED);

    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
    VerifyOrExit(ioctl(sIpFd, SIOCSIFFLAGS, &ifr) == 0, error = OT_ERROR_FAILED);

exit:
    if (error != OT_ERROR_NONE)
    {
        perror("ifconfig up");
    }

    return error;
}

otError otPlatNetifDown(otInstance *aInstance)
{
    otError      error = OT_ERROR_NONE;
    struct ifreq ifr   = {};

    OT_UNUSED_VARIABLE(aInstance);

    VerifyOrExit(sIpFd > 0);
    strncpy(ifr.ifr_name, sTunName, sizeof(ifr.ifr_name));
    VerifyOrExit(ioctl(sIpFd, SIOCGIFFLAGS, &ifr) == 0, error = OT_ERROR_FAILED);

    ifr.ifr_flags &= ~IFF_UP;
    VerifyOrExit(ioctl(sIpFd, SIOCSIFFLAGS, &ifr) == 0, error = OT_ERROR_FAILED);

exit:
    if (error != OT_ERROR_NONE)
    {
        perror("ifconfig down");
    }

    return error;
}

otError otPlatNetifAddAddress(otInstance *aInstance, const otNetifAddress *aNetifAddress)
{
    struct in6_ifreq ifr6;
    otError          error = OT_ERROR_NONE;

    OT_UNUSED_VARIABLE(aInstance);

    VerifyOrExit(sIpFd > 0);

    VerifyOrExit(sCurrentEvent == NULL || sCurrentEvent->mType != OT_PLAT_NETIF_EVENT_ADDR_ADDED);
    memcpy(&ifr6.ifr6_addr, &aNetifAddress->mAddress, sizeof(ifr6.ifr6_addr));
    fprintf(stderr, "add address %s\r\n", inet_ntop(AF_INET6, &ifr6.ifr6_addr, ip6string, sizeof(ip6string)));

    ifr6.ifr6_ifindex   = sTunIndex;
    ifr6.ifr6_prefixlen = aNetifAddress->mPrefixLength;

    VerifyOrExit(ioctl(sIpFd, SIOCSIFADDR, &ifr6) == 0, error = OT_ERROR_FAILED);

exit:
    if (error != OT_ERROR_NONE)
    {
        perror("add address");
    }

    return error;
}

otError otPlatNetifRemoveAddress(otInstance *aInstance, const otNetifAddress *aNetifAddress)
{
    struct in6_ifreq ifr6;
    otError          error = OT_ERROR_NONE;

    OT_UNUSED_VARIABLE(aInstance);

    VerifyOrExit(sIpFd > 0);
    VerifyOrExit(sCurrentEvent == NULL || sCurrentEvent->mType != OT_PLAT_NETIF_EVENT_ADDR_REMOVED);
    memcpy(&ifr6.ifr6_addr, &aNetifAddress->mAddress, sizeof(ifr6.ifr6_addr));
    fprintf(stderr, "del address %s\r\n", inet_ntop(AF_INET6, &ifr6.ifr6_addr, ip6string, sizeof(ip6string)));
    ifr6.ifr6_ifindex   = sTunIndex;
    ifr6.ifr6_prefixlen = aNetifAddress->mPrefixLength;

    VerifyOrExit(ioctl(sIpFd, SIOCDIFADDR, &ifr6) == 0, error = OT_ERROR_FAILED);

exit:
    if (error != OT_ERROR_NONE)
    {
        perror("remove address");
    }

    return error;
}

void otPlatNetifReceive(otInstance *aInstance, otMessage *aMessage)
{
    char     packet[kMaxIp6Size];
    otError  error  = OT_ERROR_NONE;
    uint16_t length = otMessageGetLength(aMessage);

    OT_UNUSED_VARIABLE(aInstance);

    VerifyOrExit(sTunFd > 0);

    VerifyOrExit(otMessageRead(aMessage, 0, packet, sizeof(packet)) == length, error = OT_ERROR_NO_BUFS);

    VerifyOrExit(write(sTunFd, packet, length) == length, error = OT_ERROR_FAILED);

exit:
    if (error != OT_ERROR_NONE)
    {
        switch (error)
        {
        case OT_ERROR_FAILED:
            perror("netif receive");
            break;
        default:
            otLogWarnPlat(aInstance, "failed to deliver netif: %s", otThreadErrorToString(error));
            break;
        }
    }

    otMessageFree(aMessage);
}

static void processTransmit(otInstance *aInstance)
{
    char       packet[kMaxIp6Size];
    otError    error   = OT_ERROR_NONE;
    otMessage *message = NULL;

    {
        ssize_t rval;
        rval = read(sTunFd, packet, sizeof(packet));

        VerifyOrExit(rval > 0, error = OT_ERROR_FAILED);

        message = otIp6NewMessage(aInstance, false);
        VerifyOrExit(message != NULL, error = OT_ERROR_NO_BUFS);

        SuccessOrExit(error = otMessageAppend(message, packet, rval));

        error   = otIp6Send(aInstance, message);
        message = NULL;
    }

exit:
    if (error != OT_ERROR_NONE)
    {
        if (message != NULL)
        {
            otMessageFree(message);
        }

        otLogWarnPlat(aInstance, "Failed to transmit IPv6 packet: %s", otThreadErrorToString(error));
    }
}

static void processNetifAddrEvent(otInstance *aInstance, struct nlmsghdr *aNetlinkMessage)
{
    struct ifaddrmsg *ifaddr = reinterpret_cast<struct ifaddrmsg *>(NLMSG_DATA(aNetlinkMessage));
    size_t            rtaLength;
    otPlatNetifEvent  event;

    VerifyOrExit(ifaddr->ifa_index == static_cast<unsigned int>(sTunIndex) && ifaddr->ifa_family == AF_INET6);

    rtaLength = IFA_PAYLOAD(aNetlinkMessage);

    for (struct rtattr *rta = reinterpret_cast<struct rtattr *>(IFA_RTA(ifaddr)); RTA_OK(rta, rtaLength);
         rta                = RTA_NEXT(rta, rtaLength))
    {
        otNetifAddress &address = event.mData.mNetifAddress;

        switch (rta->rta_type)
        {
        case IFA_ADDRESS:
        case IFA_LOCAL:
        case IFA_BROADCAST:
        case IFA_ANYCAST:
            memcpy(&address.mAddress, RTA_DATA(rta), sizeof(address.mAddress));
            address.mPrefixLength = ifaddr->ifa_prefixlen;

            if (aNetlinkMessage->nlmsg_type == RTM_NEWADDR)
            {
                event.mType = OT_PLAT_NETIF_EVENT_ADDR_ADDED;
                fprintf(stderr, "address added %s\r\n",
                        inet_ntop(AF_INET6, &address.mAddress, ip6string, sizeof(ip6string)));
            }
            else if (aNetlinkMessage->nlmsg_type == RTM_DELADDR)
            {
                event.mType = OT_PLAT_NETIF_EVENT_ADDR_REMOVED;
                fprintf(stderr, "address removed %s\r\n",
                        inet_ntop(AF_INET6, &address.mAddress, ip6string, sizeof(ip6string)));
            }
            else
            {
                continue;
            }

            sCurrentEvent = &event;
            sEventHandler(aInstance, &event);
            sCurrentEvent = NULL;
            break;
        default:
            break;
        }
    }

exit:
    return;
}

static void processNetifLinkEvent(otInstance *aInstance, struct nlmsghdr *aNetlinkMessage)
{
    struct ifinfomsg *ifinfo = reinterpret_cast<struct ifinfomsg *>(NLMSG_DATA(aNetlinkMessage));
    otPlatNetifEvent  event;

    VerifyOrExit(ifinfo->ifi_index == sTunIndex);
    event.mType   = (ifinfo->ifi_flags & IFF_UP) ? OT_PLAT_NETIF_EVENT_UP : OT_PLAT_NETIF_EVENT_DOWN;
    sCurrentEvent = &event;
    sEventHandler(aInstance, &event);
    sCurrentEvent = NULL;

exit:
    return;
}

static void processNetifEvent(otInstance *aInstance)
{
    char    buffer[8192];
    ssize_t length;

    length = recv(sNetlinkFd, buffer, sizeof(buffer), 0);

    VerifyOrExit(length > 0);

    for (struct nlmsghdr *msg = reinterpret_cast<struct nlmsghdr *>(buffer); NLMSG_OK(msg, length);
         msg                  = NLMSG_NEXT(msg, length))
    {
        switch (msg->nlmsg_type)
        {
        case RTM_NEWADDR:
        case RTM_DELADDR:
            processNetifAddrEvent(aInstance, msg);
            break;
        case RTM_NEWLINK:
        case RTM_DELLINK:
            processNetifLinkEvent(aInstance, msg);
            break;
        default:
            break;
        }
    }

exit:
    return;
}

void platformNetifUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, fd_set *aErrorFdSet, int *aMaxFd)
{
    OT_UNUSED_VARIABLE(aWriteFdSet);

    VerifyOrExit(sTunIndex > 0);

    assert(sTunFd > 0);
    assert(sNetlinkFd > 0);
    assert(sIpFd > 0);

    FD_SET(sTunFd, aReadFdSet);
    FD_SET(sTunFd, aErrorFdSet);
    FD_SET(sNetlinkFd, aReadFdSet);
    FD_SET(sNetlinkFd, aErrorFdSet);

    if (sTunFd > *aMaxFd)
    {
        *aMaxFd = sTunFd;
    }

    if (sNetlinkFd > *aMaxFd)
    {
        *aMaxFd = sNetlinkFd;
    }

exit:
    return;
}

void platformNetifProcess(otInstance *  aInstance,
                          const fd_set *aReadFdSet,
                          const fd_set *aWriteFdSet,
                          const fd_set *aErrorFdSet)
{
    OT_UNUSED_VARIABLE(aWriteFdSet);
    VerifyOrExit(sTunIndex > 0);

    if (FD_ISSET(sTunFd, aErrorFdSet))
    {
        close(sTunFd);
        exit(EXIT_FAILURE);
    }

    if (FD_ISSET(sNetlinkFd, aErrorFdSet))
    {
        close(sNetlinkFd);
        exit(EXIT_FAILURE);
    }

    if (FD_ISSET(sTunFd, aReadFdSet))
    {
        processTransmit(aInstance);
    }

    if (FD_ISSET(sNetlinkFd, aReadFdSet))
    {
        processNetifEvent(aInstance);
    }

exit:
    return;
}
#endif // OPENTHREAD_ENABLE_PLATFORM_NETIF
