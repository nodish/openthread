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

#include <stdarg.h>

#include "platform-posix.h"
#include "otc.h"

#include "openthread/platform/diag.h"
#include "openthread/platform/radio.h"

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

//void saveHwAddr(otInstance *aInstance,
//        spinel_prop_key_t key,
//        const uint8_t *data,
//        spinel_size_t dataLength);

//typedef struct PropertyHandlerEntry
//{
//    spinel_prop_key_t mPropKey;
//    PropertyHandler   mHandler;
//} PropertyHandlerEntry;
//
//const PropertyHandlerEntry PropertyHandlerTable[] = {
//    {SPINEL_PROP_HWADDR, saveHwAddr},
//};
//
OT_TOOL_PACKED_BEGIN
struct RadioMessage
{
    uint8_t mPsdu[kMaxPHYPacketSize];
} OT_TOOL_PACKED_END;

static ThreadError radioTransmit(otInstance *aInstance, const struct RadioPacket *pkt);
static void radioTransmitDone(otInstance *aInstance);
static void radioSendMessage(otInstance *aInstance);
static void radioSendAck(otInstance *aInstance);
static void radioProcessFrame(otInstance *aInstance);

static PhyState sState = kStateDisabled;
static struct RadioMessage sReceiveMessage;
static struct RadioMessage sTransmitMessage;
static struct RadioMessage sAckMessage;
static RadioPacket sReceiveFrame;
static RadioPacket sTransmitFrame;
static RadioPacket sAckFrame;

static uint8_t sExtendedAddress[OT_EXT_ADDRESS_SIZE];
static uint16_t sShortAddress;
static uint16_t sPanid;
static int sSockFd;
static bool sPromiscuous = false;
static bool sAckWait = false;

bool sLastTransmitFramePending;
ThreadError sLastTransmitError;
RadioPacket *sLastTransmitPacket;

void handleMacFrame(void* aContext, const uint8_t *aBuf, uint16_t aLength);

static inline bool isFrameTypeAck(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_ACK;
}

static inline bool isFrameTypeMacCmd(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_TYPE_MASK) == IEEE802154_FRAME_TYPE_MACCMD;
}

static inline bool isSecurityEnabled(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_SECURITY_ENABLED) != 0;
}

static inline bool isFramePending(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_FRAME_PENDING) != 0;
}

static inline bool isAckRequested(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_ACK_REQUEST) != 0;
}

static inline bool isPanIdCompressed(const uint8_t *frame)
{
    return (frame[0] & IEEE802154_PANID_COMPRESSION) != 0;
}

static inline bool isDataRequest(const uint8_t *frame)
{
    const uint8_t *cur = frame;
    uint8_t securityControl;
    bool rval;

    // FCF + DSN
    cur += 2 + 1;

    VerifyOrExit(isFrameTypeMacCmd(frame), rval = false);

    // Destination PAN + Address
    switch (frame[1] & IEEE802154_DST_ADDR_MASK)
    {
    case IEEE802154_DST_ADDR_SHORT:
        cur += sizeof(otPanId) + sizeof(otShortAddress);
        break;

    case IEEE802154_DST_ADDR_EXT:
        cur += sizeof(otPanId) + sizeof(otExtAddress);
        break;

    default:
        ExitNow(rval = false);
    }

    // Source PAN + Address
    switch (frame[1] & IEEE802154_SRC_ADDR_MASK)
    {
    case IEEE802154_SRC_ADDR_SHORT:
        if (!isPanIdCompressed(frame))
        {
            cur += sizeof(otPanId);
        }

        cur += sizeof(otShortAddress);
        break;

    case IEEE802154_SRC_ADDR_EXT:
        if (!isPanIdCompressed(frame))
        {
            cur += sizeof(otPanId);
        }

        cur += sizeof(otExtAddress);
        break;

    default:
        ExitNow(rval = false);
    }

    // Security Control + Frame Counter + Key Identifier
    if (isSecurityEnabled(frame))
    {
        securityControl = *cur;

        if (securityControl & IEEE802154_SEC_LEVEL_MASK)
        {
            cur += 1 + 4;
        }

        switch (securityControl & IEEE802154_KEY_ID_MODE_MASK)
        {
        case IEEE802154_KEY_ID_MODE_0:
            cur += 0;
            break;

        case IEEE802154_KEY_ID_MODE_1:
            cur += 1;
            break;

        case IEEE802154_KEY_ID_MODE_2:
            cur += 5;
            break;

        case IEEE802154_KEY_ID_MODE_3:
            cur += 9;
            break;
        }
    }

    // Command ID
    rval = cur[0] == IEEE802154_MACCMD_DATA_REQ;

exit:
    return rval;
}

static inline uint8_t getDsn(const uint8_t *frame)
{
    return frame[IEEE802154_DSN_OFFSET];
}

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

uint8_t sFactoryAddress[8] = {0};

//void handleGetPropHwAddr(otInstance *aInstance,
//        void* aContext,
//        uint32_t command,
//        spinel_prop_key_t key,
//        const uint8_t *data,
//        spinel_size_t dataLength)
//{
//    (void)aContext;
//    uint8_t *hwaddr = NULL;
//    otcGetPropHandler(aInstance, command, key, data, dataLength, SPINEL_DATATYPE_EUI64_S, &hwaddr);
//    if (hwaddr)
//    {
//        memcpy(sFactoryAddress, hwaddr, sizeof(sFactoryAddress));
//    }
//}

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    printf("calling %s\r\n", __func__);
    otcGetProp(aInstance, SPINEL_PROP_HWADDR, SPINEL_DATATYPE_EUI64_S, &aIeeeEui64);
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    printf("calling %s\r\n", __func__);
    sPanid = panid;
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_15_4_PANID,
            SPINEL_DATATYPE_UINT16_S,
            panid);
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
    printf("calling %s\r\n", __func__);
    for (size_t i = 0; i < sizeof(sExtendedAddress); i++)
    {
        sExtendedAddress[i] = address[sizeof(sExtendedAddress) - 1 - i];
    }
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_15_4_LADDR,
            SPINEL_DATATYPE_EUI64_S,
            &sExtendedAddress);
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
    printf("calling %s\r\n", __func__);
    sShortAddress = address;
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_15_4_SADDR,
            SPINEL_DATATYPE_UINT16_S,
            address
            );
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    printf("calling %s\r\n", __func__);
    sPromiscuous = aEnable;
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_PROMISCUOUS_MODE,
            SPINEL_DATATYPE_UINT8_S,
            aEnable != 0 ? SPINEL_MAC_PROMISCUOUS_MODE_NETWORK : SPINEL_MAC_PROMISCUOUS_MODE_OFF
            );
}

void platformRadioInit(void)
{
    printf("calling %s\r\n", __func__);
    sSockFd = otcOpen(handleMacFrame, radioTransmitDone);

    sReceiveFrame.mPsdu = sReceiveMessage.mPsdu;
    sTransmitFrame.mPsdu = sTransmitMessage.mPsdu;
    sAckFrame.mPsdu = sAckMessage.mPsdu;
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    return (sState != kStateDisabled) ? true : false;
}

ThreadError otPlatRadioEnable(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    if (!otPlatRadioIsEnabled(aInstance))
    {
        sState = kStateSleep;
    }

    printf("-----------enable radio\r\n");
    otcSetProp(
            aInstance,
            SPINEL_PROP_PHY_ENABLED,
            SPINEL_DATATYPE_BOOL_S,
            true);

    return kThreadError_None;
}

ThreadError otPlatRadioDisable(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    if (otPlatRadioIsEnabled(aInstance))
    {
        sState = kStateDisabled;
    }

    otcSetProp(
            aInstance,
            SPINEL_PROP_PHY_ENABLED,
            SPINEL_DATATYPE_BOOL_S,
            false
            );
    return kThreadError_None;
}

ThreadError otPlatRadioSleep(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    ThreadError error = kThreadError_InvalidState;

    if (sState == kStateSleep || sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateSleep;
    }

    return otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_RAW_STREAM_ENABLED,
            SPINEL_DATATYPE_BOOL_S,
            false
            );
}

ThreadError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    printf("%s\r\n", __func__);
    ThreadError error = kThreadError_InvalidState;

    printf("%s called sState %d\r\n", __func__, sState);

    VerifyOrExit(sState != kStateDisabled, );

    error = kThreadError_None;

    if (sReceiveFrame.mChannel != aChannel)
    {
        sAckWait = false;
        error = otcSetProp(
                aInstance,
                SPINEL_PROP_PHY_CHAN,
                SPINEL_DATATYPE_UINT8_S,
                aChannel);
        sReceiveFrame.mChannel = aChannel;
    }

    if (error == kThreadError_None)
    {
        if (sState == kStateSleep)
        {
            error = otcSetProp(
                    aInstance,
                    SPINEL_PROP_MAC_RAW_STREAM_ENABLED,
                    SPINEL_DATATYPE_BOOL_S,
                    true);
        }
        sState = kStateReceive;
    }

exit:

    return error;
}

int sprint_hex(char *out, const uint8_t* buf, uint16_t len);

ThreadError otPlatRadioTransmit(otInstance *aInstance, RadioPacket *aPacket)
{
    printf("%s called state=%d ack=%d\r\n", __func__, sState, sAckWait);
    char hex[512];
    sprint_hex(hex, aPacket->mPsdu, aPacket->mLength);
    printf("%s packet=[%s]\r\n", __func__, hex);

    ThreadError error = kThreadError_InvalidState;
    (void)aInstance;
    (void)aPacket;

    if (sState == kStateReceive)
    {
        error = kThreadError_None;
        sState = kStateTransmit;
        sLastTransmitPacket = aPacket;
    }

    return error;
}

RadioPacket *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
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
    return kRadioCapsNone;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    return sPromiscuous;
}

void radioReceiveFrame(otInstance *aInstance)
{
    char hex[512];
    sprint_hex(hex, sReceiveFrame.mPsdu, sReceiveFrame.mLength);
    printf("%s length=%u packet=[%s]\r\n", __func__, sReceiveFrame.mLength, hex);

    if (sAckWait &&
        isFrameTypeAck(sReceiveFrame.mPsdu) &&
        getDsn(sReceiveFrame.mPsdu) == getDsn(sTransmitFrame.mPsdu))
    {
        sState = kStateReceive;
        sAckWait = false;

#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
        }
        else
#endif
        {
            otPlatRadioTransmitDone(aInstance, &sTransmitFrame, isFramePending(sReceiveFrame.mPsdu), kThreadError_None);
        }
    }
    else if ((sState == kStateReceive || sState == kStateTransmit))
    {
        radioProcessFrame(aInstance);
    }
}

void handleMacFrame(void* aContext, const uint8_t *aBuf, uint16_t aLength)
{
    uint16_t packetLength = 0;
    if (!(try_spinel_datatype_unpack(
                    aBuf,
                    aLength,
                    SPINEL_DATATYPE_UINT16_S,
                    &packetLength) &&
                packetLength <= sizeof(sReceiveMessage.mPsdu) &&
                aLength > sizeof(uint16_t) + packetLength))
    {
        printf("Invalid mac frame packet\n");
        return;
    }

    sReceiveFrame.mLength = (uint8_t)packetLength;

    uint8_t offset = 2; // spinel header
    uint8_t length = (uint8_t)(aLength - 2);

    if (packetLength != 0)
    {
        memcpy(sReceiveMessage.mPsdu, aBuf + offset, packetLength);
        offset += sReceiveFrame.mLength;
        length -= sReceiveFrame.mLength;
    }

    uint16_t flags = 0;
    int8_t noiseFloor = -128;
    if (try_spinel_datatype_unpack(
                aBuf + offset,
                length,
                SPINEL_DATATYPE_INT8_S
                SPINEL_DATATYPE_INT8_S
                SPINEL_DATATYPE_UINT16_S,
                &sReceiveFrame.mPower,
                &noiseFloor,
                &flags))
    {
        radioReceiveFrame((otInstance*)aContext);
    }

}

void radioReceive(otInstance *aInstance)
{
    otcReceive(aInstance);
}

void radioSendMessage(otInstance *aInstance)
{
    printf("%s\r\n", __func__);

    bool ackWait = isAckRequested(sTransmitFrame.mPsdu);
    sAckWait = ackWait;
    printf("%s sAckWait=%d\r\n", __func__, sAckWait);

    ThreadError error = radioTransmit(aInstance, &sTransmitFrame);

    if (!ackWait || kThreadError_None != error)
    {
        sState = kStateReceive;

#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagRadioTransmitDone(aInstance, &sTransmitFrame, false, error);
        }
        else
#endif
        {
            otPlatRadioTransmitDone(aInstance, &sTransmitFrame, false, error);
        }
    }
}

void platformRadioUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, int *aMaxFd)
{
    if (aReadFdSet != NULL && (sState != kStateTransmit || sAckWait))
    {
        FD_SET(sSockFd, aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }

    if (aWriteFdSet != NULL && sState == kStateTransmit && !sAckWait)
    {
        FD_SET(sSockFd, aWriteFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }
}

void platformRadioProcess(otInstance *aInstance)
{
    const int flags = POLLIN | POLLRDNORM | POLLERR | POLLNVAL | POLLHUP;
    struct pollfd pollfd = { sSockFd, flags, 0 };

    if (POLL(&pollfd, 1, 0) > 0 && (pollfd.revents & flags) != 0)
    {
        radioReceive(aInstance);
    }

    if (sState == kStateTransmit && !sAckWait)
    {
        printf("%s\r\n", __func__);
        radioSendMessage(aInstance);
    }
}

void radioTransmitDone(otInstance *aInstance)
{
    printf("%s begin sAckWait=%d\r\n", __func__, sAckWait);
    if (sState == kStateTransmit)
    {
        printf("%s in %d %d\r\n", __func__, sLastTransmitFramePending, sLastTransmitError);
        sState = kStateReceive;
        sAckWait = false;
        otPlatRadioTransmitDone(aInstance, sLastTransmitPacket, sLastTransmitFramePending, sLastTransmitError);
    }
    printf("%s done sAckWait=%d\r\n", __func__, sAckWait);
}

ThreadError radioTransmit(otInstance *aInstance, const struct RadioPacket *pkt)
{
    printf("%s\r\n", __func__);
    sLastTransmitFramePending = false;
    sLastTransmitError = kThreadError_None;
    return otcSendPacket(aInstance, pkt, isAckRequested(pkt->mPsdu));
}

void radioSendAck(otInstance *aInstance)
{
    printf("%s\r\n", __func__);
    sAckFrame.mLength = IEEE802154_ACK_LENGTH;
    sAckMessage.mPsdu[0] = IEEE802154_FRAME_TYPE_ACK;

    if (isDataRequest(sReceiveFrame.mPsdu))
    {
        sAckMessage.mPsdu[0] |= IEEE802154_FRAME_PENDING;
    }

    sAckMessage.mPsdu[1] = 0;
    sAckMessage.mPsdu[2] = getDsn(sReceiveFrame.mPsdu);

    radioTransmit(aInstance, &sAckFrame);
}

void radioProcessFrame(otInstance *aInstance)
{
    printf("%s\r\n", __func__);
    ThreadError error = kThreadError_None;
    otPanId dstpan;
    otShortAddress short_address;
    otExtAddress ext_address;

    VerifyOrExit(sPromiscuous == false, error = kThreadError_None);

    switch (sReceiveFrame.mPsdu[1] & IEEE802154_DST_ADDR_MASK)
    {
        case IEEE802154_DST_ADDR_NONE:
            break;

        case IEEE802154_DST_ADDR_SHORT:
            dstpan = getDstPan(sReceiveFrame.mPsdu);
            short_address = getShortAddress(sReceiveFrame.mPsdu);
            VerifyOrExit((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                    (short_address == IEEE802154_BROADCAST || short_address == sShortAddress),
                    error = kThreadError_Abort);
            break;

        case IEEE802154_DST_ADDR_EXT:
            dstpan = getDstPan(sReceiveFrame.mPsdu);
            getExtAddress(sReceiveFrame.mPsdu, &ext_address);
            VerifyOrExit((dstpan == IEEE802154_BROADCAST || dstpan == sPanid) &&
                    memcmp(&ext_address, sExtendedAddress, sizeof(ext_address)) == 0,
                    error = kThreadError_Abort);
            break;

        default:
            ExitNow(error = kThreadError_Abort);
    }

    sReceiveFrame.mLqi = kPhyNoLqi;

    // generate acknowledgment
    if (isAckRequested(sReceiveFrame.mPsdu))
    {
        radioSendAck(aInstance);
    }

exit:

#if OPENTHREAD_ENABLE_DIAG

    if (otPlatDiagModeGet())
    {
        otPlatDiagRadioReceiveDone(aInstance, error == kThreadError_None ? &sReceiveFrame : NULL, error);
    }
    else
#endif
    {
        otPlatRadioReceiveDone(aInstance, error == kThreadError_None ? &sReceiveFrame : NULL, error);
    }
}

void otPlatRadioEnableSrcMatch(otInstance *aInstance, bool aEnable)
{
    printf("calling %s\r\n", __func__);
    (void)aInstance;
    (void)aEnable;
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_ENABLED,
            SPINEL_DATATYPE_BOOL_S,
            (aEnable ? true : false));
}

ThreadError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    printf("calling %s\r\n", __func__);
    otcInsertProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            SPINEL_DATATYPE_UINT16_S,
            aShortAddress
            );
    return kThreadError_None;
}

ThreadError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    printf("calling %s\r\n", __func__);
    otcInsertProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            SPINEL_DATATYPE_EUI64_S,
            aExtAddress
            );
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    printf("calling %s\r\n", __func__);
    otcRemoveProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            SPINEL_DATATYPE_UINT16_S,
            aShortAddress
            );
    return kThreadError_None;
}

ThreadError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    printf("calling %s\r\n", __func__);
    otcRemoveProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            SPINEL_DATATYPE_EUI64_S,
            aExtAddress
            );
    return kThreadError_None;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_SHORT_ADDRESSES,
            NULL
            );
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    printf("calling %s\r\n", __func__);
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SRC_MATCH_EXTENDED_ADDRESSES,
            NULL
            );
}

ThreadError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    printf("calling %s\r\n", __func__);
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_MASK,
            SPINEL_DATATYPE_UINT8_S,
            aScanChannel
            );
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_PERIOD,
            SPINEL_DATATYPE_UINT16_S,
            aScanDuration
            );
    otcSetProp(
            aInstance,
            SPINEL_PROP_MAC_SCAN_STATE,
            SPINEL_DATATYPE_UINT8_S,
            SPINEL_SCAN_STATE_ENERGY
            );
    return kThreadError_None;
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
    printf("calling %s\r\n", __func__);
    otcSetProp(
            aInstance,
            SPINEL_PROP_PHY_TX_POWER,
            SPINEL_DATATYPE_INT8_S,
            aPower
            );
}
