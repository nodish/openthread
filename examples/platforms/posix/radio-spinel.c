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

#include "platform-posix.h"
#include "ncp.h"

#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>

#include "utils/code_utils.h"

#if OPENTHREAD_ENABLE_POSIX_RADIO_SPINEL

enum
{
    IEEE802154_MIN_LENGTH         = 5,
    IEEE802154_MAX_LENGTH         = 127,
    IEEE802154_ACK_LENGTH         = 5,

    IEEE802154_BROADCAST          = 0xffff,

    IEEE802154_FRAME_TYPE_ACK     = 2 << 0,
    IEEE802154_FRAME_TYPE_MACCMD  = 3 << 0,
    IEEE802154_FRAME_TYPE_MASK    = 7 << 0,

    IEEE802154_SECURITY_ENABLED   = 1 << 3,
    IEEE802154_FRAME_PENDING      = 1 << 4,
    IEEE802154_ACK_REQUEST        = 1 << 5,
    IEEE802154_PANID_COMPRESSION  = 1 << 6,

    IEEE802154_DST_ADDR_NONE      = 0 << 2,
    IEEE802154_DST_ADDR_SHORT     = 2 << 2,
    IEEE802154_DST_ADDR_EXT       = 3 << 2,
    IEEE802154_DST_ADDR_MASK      = 3 << 2,

    IEEE802154_SRC_ADDR_NONE      = 0 << 6,
    IEEE802154_SRC_ADDR_SHORT     = 2 << 6,
    IEEE802154_SRC_ADDR_EXT       = 3 << 6,
    IEEE802154_SRC_ADDR_MASK      = 3 << 6,

    IEEE802154_DSN_OFFSET         = 2,
    IEEE802154_DSTPAN_OFFSET      = 3,
    IEEE802154_DSTADDR_OFFSET     = 5,

    IEEE802154_SEC_LEVEL_MASK     = 7 << 0,

    IEEE802154_KEY_ID_MODE_0      = 0 << 3,
    IEEE802154_KEY_ID_MODE_1      = 1 << 3,
    IEEE802154_KEY_ID_MODE_2      = 2 << 3,
    IEEE802154_KEY_ID_MODE_3      = 3 << 3,
    IEEE802154_KEY_ID_MODE_MASK   = 3 << 3,

    IEEE802154_MACCMD_DATA_REQ    = 4,
};

enum
{
    POSIX_RECEIVE_SENSITIVITY = -100,  // dBm
};

static void radioProcessFrame(otInstance *aInstance);

static uint8_t sReceivePsdu[OT_RADIO_FRAME_MAX_SIZE];
static uint8_t sTransmitPsdu[OT_RADIO_FRAME_MAX_SIZE];
static uint8_t sAckPsdu[OT_RADIO_FRAME_MAX_SIZE];
static otRadioFrame sReceiveFrame;
static otRadioFrame sTransmitFrame;
static otRadioFrame sAckFrame;

static uint8_t sExtendedAddress[OT_EXT_ADDRESS_SIZE];
static uint16_t sShortAddress;
static uint16_t sPanid;
static int sSockFd;
static uint8_t sChannel;
static bool sPromiscuous = false;
static otRadioState sState = OT_RADIO_STATE_DISABLED;

bool sLastTransmitFramePending;
otError sLastTransmitError;
bool sLastTransmitDone;
otRadioFrame *sLastTransmitPacket;

static inline otPanId getDstPan(const uint8_t *frame)
{
    return (otPanId)((frame[IEEE802154_DSTPAN_OFFSET + 1] << 8) | frame[IEEE802154_DSTPAN_OFFSET]);
}

static inline otShortAddress getShortAddress(const uint8_t *frame)
{
    return (otShortAddress)((frame[IEEE802154_DSTADDR_OFFSET + 1] << 8) | frame[IEEE802154_DSTADDR_OFFSET]);
}

static inline void getExtAddress(const uint8_t *frame, otExtAddress *address)
{
    size_t i;

    for (i = 0; i < sizeof(otExtAddress); i++)
    {
        address->m8[i] = frame[IEEE802154_DSTADDR_OFFSET + (sizeof(otExtAddress) - 1 - i)];
    }
}


void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    printf("calling %s\r\n", __func__);
    ncpGetProp(aInstance, SPINEL_PROP_HWADDR, SPINEL_DATATYPE_EUI64_S, &aIeeeEui64);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    printf("calling %s\r\n", __func__);
    sPanid = panid;
    ncpSetProp(aInstance, SPINEL_PROP_MAC_15_4_PANID, SPINEL_DATATYPE_UINT16_S, panid);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
    printf("calling %s\r\n", __func__);
    // TODO reverse or not?
    for (size_t i = 0; i < sizeof(sExtendedAddress); i++)
    {
        sExtendedAddress[i] = address[sizeof(sExtendedAddress) - 1 - i];
    }
    ncpSetProp(aInstance, SPINEL_PROP_MAC_15_4_LADDR, SPINEL_DATATYPE_EUI64_S, sExtendedAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(aInstance, SPINEL_PROP_MAC_15_4_SADDR, SPINEL_DATATYPE_UINT16_S, address);
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(aInstance, SPINEL_PROP_MAC_PROMISCUOUS_MODE, SPINEL_DATATYPE_UINT8_S,
            aEnable != 0 ? SPINEL_MAC_PROMISCUOUS_MODE_NETWORK : SPINEL_MAC_PROMISCUOUS_MODE_OFF);
}

void platformRadioInit(void)
{
    printf("calling %s\r\n", __func__);
    sSockFd = ncpOpen();

    sReceiveFrame.mPsdu = sReceivePsdu;
    sTransmitFrame.mPsdu = sTransmitPsdu;
    sAckFrame.mPsdu = sAckPsdu;
}

void platformRadioDeinit(void)
{
    close(sSockFd);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    bool enabled;
    ncpGetProp(aInstance, SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, &enabled);
    return enabled;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    otError error = ncpSetProp(aInstance, SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, true);
    if (error == OT_ERROR_NONE)
    {
        sState = OT_RADIO_STATE_SLEEP;
    }
    return error;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    otError error = ncpSetProp(aInstance, SPINEL_PROP_PHY_ENABLED, SPINEL_DATATYPE_BOOL_S, false);
    if (error == OT_ERROR_NONE)
    {
        sState = OT_RADIO_STATE_DISABLED;
    }
    return error;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;
    printf("calling %s\r\n", __func__);

    error = ncpSetProp(aInstance, SPINEL_PROP_MAC_RAW_STREAM_ENABLED,
            SPINEL_DATATYPE_BOOL_S, false);

    otEXPECT(error == OT_ERROR_NONE);
    sState = OT_RADIO_STATE_SLEEP;

exit:
    return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    printf("%s\r\n", __func__);
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION(sState != OT_RADIO_STATE_DISABLED, error = OT_ERROR_INVALID_STATE);

    if (sChannel != aChannel)
    {
        error = ncpSetProp(
                aInstance,
                SPINEL_PROP_PHY_CHAN,
                SPINEL_DATATYPE_UINT8_S,
                aChannel);
        otEXPECT(error == OT_ERROR_NONE);
        sChannel = aChannel;
    }

    if (sState == OT_RADIO_STATE_SLEEP)
    {
        error = ncpSetProp(
                aInstance,
                SPINEL_PROP_MAC_RAW_STREAM_ENABLED,
                SPINEL_DATATYPE_BOOL_S,
                true);
        otEXPECT(error == OT_ERROR_NONE);
        sState = OT_RADIO_STATE_RECEIVE;
    }

exit:

    return error;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aFrame)
{
    printf("calling %s\r\n", __func__);
    otError error = OT_ERROR_INVALID_STATE;
    otEXPECT(sState == OT_RADIO_STATE_RECEIVE);

    sLastTransmitDone = false;
    error = ncpSendPacket(aInstance, aFrame, &sAckFrame);
    otEXPECT(error == OT_ERROR_NONE);
    sState = OT_RADIO_STATE_TRANSMIT;

exit:
    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    return OT_RADIO_CAPS_NONE;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    uint8_t mode;
    ncpGetProp(aInstance, SPINEL_PROP_MAC_PROMISCUOUS_MODE, SPINEL_DATATYPE_UINT8_S, &mode);
    return mode == SPINEL_MAC_PROMISCUOUS_MODE_FULL;
}

void radioReceive(otInstance *aInstance)
{
    ncpReceive(aInstance, &sReceiveFrame);
}

void radioReceiveFrame(otInstance *aInstance)
{
    radioProcessFrame(aInstance);
}

void platformRadioUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, int *aMaxFd)
{
    if (aReadFdSet != NULL)
    {
        FD_SET(sSockFd, aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }

    if (aWriteFdSet != NULL)
    {
        FD_SET(sSockFd, aWriteFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }
}

static inline bool isAckRequested(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_ACK_REQUEST) != 0;
}

void platformRadioProcess(otInstance *aInstance)
{
    const int flags = POLLIN | POLLRDNORM | POLLERR | POLLNVAL | POLLHUP;
    struct pollfd pollfd = { sSockFd, flags, 0 };

    if (POLL(&pollfd, 1, 0) > 0 && (pollfd.revents & flags) != 0)
    {
        radioReceive(aInstance);
    }

    if (sState == OT_RADIO_STATE_TRANSMIT)
    {
        if (!isAckRequested(sTransmitFrame.mPsdu))
        {
            sState = OT_RADIO_STATE_RECEIVE;

#if OPENTHREAD_ENABLE_DIAG

            if (otPlatDiagModeGet())
            {
                otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, OT_ERROR_NONE);
            }
            else
#endif
            {
                otPlatRadioTxDone(aInstance, &sTransmitFrame, NULL, OT_ERROR_NONE);
            }
        }
        else if (sLastTransmitDone)
        {
            otPlatRadioTxDone(aInstance, &sTransmitFrame, &sAckFrame, sLastTransmitError);
            sLastTransmitDone = false;
            sState = OT_RADIO_STATE_RECEIVE;
        }
    }
}

void radioTransmitDone(otInstance *aInstance)
{
    (void)aInstance;
    sLastTransmitDone = true;
}


void radioProcessFrame(otInstance *aInstance)
{
    otError error = OT_ERROR_NONE;
    printf("%s\r\n", __func__);
    otPanId dstpan;
    otShortAddress short_address;
    otExtAddress ext_address;

    otEXPECT_ACTION(sPromiscuous == false, error = OT_ERROR_NONE);

    switch (sReceiveFrame.mPsdu[1] & IEEE802154_DST_ADDR_MASK)
    {
    case IEEE802154_DST_ADDR_NONE:
        break;

    case IEEE802154_DST_ADDR_SHORT:
        dstpan = getDstPan(sReceiveFrame.mPsdu);
        short_address = getShortAddress(sReceiveFrame.mPsdu);
        otEXPECT_ACTION((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                        (short_address == IEEE802154_BROADCAST || short_address == sShortAddress),
                        error = OT_ERROR_ABORT);
        break;

    case IEEE802154_DST_ADDR_EXT:
        dstpan = getDstPan(sReceiveFrame.mPsdu);
        getExtAddress(sReceiveFrame.mPsdu, &ext_address);
        otEXPECT_ACTION((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                        memcmp(&ext_address, sExtendedAddress, sizeof(ext_address)) == 0,
                        error = OT_ERROR_ABORT);
        break;

    default:
        error = OT_ERROR_ABORT;
        goto exit;
    }

    sReceiveFrame.mPower = -20;
    sReceiveFrame.mLqi = OT_RADIO_LQI_NONE;

exit:

#if OPENTHREAD_ENABLE_DIAG

    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioReceiveDone(aInstance, error == OT_ERROR_NONE ? &sReceiveFrame : NULL, error);
    }
    else
#endif
    {
        otPlatRadioReceiveDone(aInstance, error == OT_ERROR_NONE ? &sReceiveFrame : NULL, error);
    }
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_ENABLED,
            SPINEL_DATATYPE_BOOL_S,
            aEnable);
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    printf("calling %s\r\n", __func__);
    return ncpInsertProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            SPINEL_DATATYPE_UINT16_S,
            aShortAddress);
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    printf("calling %s\r\n", __func__);
    return ncpInsertProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            SPINEL_DATATYPE_EUI64_S,
            aExtAddress);
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    printf("calling %s\r\n", __func__);
    return ncpRemoveProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            SPINEL_DATATYPE_UINT16_S,
            aShortAddress);
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    printf("calling %s\r\n", __func__);
    return ncpRemoveProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            SPINEL_DATATYPE_EUI64_S,
            aExtAddress);
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            NULL);
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            NULL);
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    printf("calling %s\r\n", __func__);
    otError error;

    (error = ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_MASK,
            SPINEL_DATATYPE_UINT8_S,
            aScanChannel)) ||
    (error = ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_PERIOD,
            SPINEL_DATATYPE_UINT16_S,
            aScanDuration)) ||
    (error = ncpSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_STATE,
            SPINEL_DATATYPE_UINT8_S,
            SPINEL_SCAN_STATE_ENERGY));

    return error;
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
    printf("calling %s\r\n", __func__);
    ncpSetProp(
            aInstance,
            SPINEL_PROP_PHY_TX_POWER,
            SPINEL_DATATYPE_INT8_S,
            aPower);
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    (void)aInstance;
    return POSIX_RECEIVE_SENSITIVITY;
}

#endif // OPENTHREAD_ENABLE_POSIX_RADIO_SPINEL
