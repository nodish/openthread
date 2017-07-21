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

#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/radio.h>

#include "utils/code_utils.h"
#include "utils/radio_utils.h"

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

OT_TOOL_PACKED_BEGIN
struct RadioMessage
{
    uint8_t mChannel;
    uint8_t mPsdu[OT_RADIO_FRAME_MAX_SIZE];
} OT_TOOL_PACKED_END;

static void radioTransmit(struct RadioMessage *msg, struct otRadioFrame *pkt);
static void radioSendMessage(otInstance *aInstance);
static void radioSendAck(void);
static void radioProcessFrame(otInstance *aInstance);

static otRadioState sState = OT_RADIO_STATE_DISABLED;
static struct RadioMessage sReceiveMessage;
static struct RadioMessage sTransmitMessage;
static struct RadioMessage sAckMessage;
static otRadioFrame sReceiveFrame;
static otRadioFrame sTransmitFrame;
static otRadioFrame sAckFrame;

static uint8_t sExtendedAddress[OT_EXT_ADDRESS_SIZE];
static uint16_t sShortAddress;
static uint16_t sPanid;
static int sSockFd;
static bool sPromiscuous = false;
static bool sAckWait = false;
static uint16_t sPortOffset = 0;

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

    otEXPECT_ACTION(isFrameTypeMacCmd(frame), rval = false);

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
        rval = false;
        goto exit;
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
        rval = false;
        goto exit;
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

void otPlatRadioGetIeeeEui64(otInstance *aInstance, uint8_t *aIeeeEui64)
{
    (void)aInstance;
    aIeeeEui64[0] = 0x18;
    aIeeeEui64[1] = 0xb4;
    aIeeeEui64[2] = 0x30;
    aIeeeEui64[3] = 0x00;
    aIeeeEui64[4] = (NODE_ID >> 24) & 0xff;
    aIeeeEui64[5] = (NODE_ID >> 16) & 0xff;
    aIeeeEui64[6] = (NODE_ID >> 8) & 0xff;
    aIeeeEui64[7] = NODE_ID & 0xff;
}

void otPlatRadioSetPanId(otInstance *aInstance, uint16_t panid)
{
    (void)aInstance;
    sPanid = panid;
}

void otPlatRadioSetExtendedAddress(otInstance *aInstance, uint8_t *address)
{
    (void)aInstance;

    for (size_t i = 0; i < sizeof(sExtendedAddress); i++)
    {
        sExtendedAddress[i] = address[sizeof(sExtendedAddress) - 1 - i];
    }
}

void otPlatRadioSetShortAddress(otInstance *aInstance, uint16_t address)
{
    (void)aInstance;
    sShortAddress = address;
}

void otPlatRadioSetPromiscuous(otInstance *aInstance, bool aEnable)
{
    (void)aInstance;
    sPromiscuous = aEnable;
}

void platformRadioInit(void)
{
    struct sockaddr_in sockaddr;
    char *offset;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;

    offset = getenv("PORT_OFFSET");

    if (offset)
    {
        char *endptr;

        sPortOffset = (uint16_t)strtol(offset, &endptr, 0);

        if (*endptr != '\0')
        {
            fprintf(stderr, "Invalid PORT_OFFSET: %s\n", offset);
            exit(1);
        }

        sPortOffset *= WELLKNOWN_NODE_ID;
    }

    if (sPromiscuous)
    {
        sockaddr.sin_port = htons(9000 + sPortOffset + WELLKNOWN_NODE_ID);
    }
    else
    {
        sockaddr.sin_port = htons(9000 + sPortOffset + NODE_ID);
    }

    sockaddr.sin_addr.s_addr = INADDR_ANY;

    sSockFd = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (sSockFd == -1)
    {
        perror("socket");
        exit(EXIT_FAILURE);
    }

    if (bind(sSockFd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1)
    {
        perror("bind");
        exit(EXIT_FAILURE);
    }

    sReceiveFrame.mPsdu = sReceiveMessage.mPsdu;
    sTransmitFrame.mPsdu = sTransmitMessage.mPsdu;
    sAckFrame.mPsdu = sAckMessage.mPsdu;
}

void platformRadioDeinit(void)
{
    close(sSockFd);
}

bool otPlatRadioIsEnabled(otInstance *aInstance)
{
    (void)aInstance;
    return (sState != OT_RADIO_STATE_DISABLED) ? true : false;
}

otError otPlatRadioEnable(otInstance *aInstance)
{
    if (!otPlatRadioIsEnabled(aInstance))
    {
        sState = OT_RADIO_STATE_SLEEP;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioDisable(otInstance *aInstance)
{
    if (otPlatRadioIsEnabled(aInstance))
    {
        sState = OT_RADIO_STATE_DISABLED;
    }

    return OT_ERROR_NONE;
}

otError otPlatRadioSleep(otInstance *aInstance)
{
    otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;

    if (sState == OT_RADIO_STATE_SLEEP || sState == OT_RADIO_STATE_RECEIVE)
    {
        error = OT_ERROR_NONE;
        sState = OT_RADIO_STATE_SLEEP;
    }

    return error;
}

otError otPlatRadioReceive(otInstance *aInstance, uint8_t aChannel)
{
    otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;

    if (sState != OT_RADIO_STATE_DISABLED)
    {
        error = OT_ERROR_NONE;
        sState = OT_RADIO_STATE_RECEIVE;
        sAckWait = false;
        sReceiveFrame.mChannel = aChannel;
    }

    return error;
}

otError otPlatRadioTransmit(otInstance *aInstance, otRadioFrame *aRadio)
{
    otError error = OT_ERROR_INVALID_STATE;
    (void)aInstance;
    (void)aRadio;

    if (sState == OT_RADIO_STATE_RECEIVE)
    {
        error = OT_ERROR_NONE;
        sState = OT_RADIO_STATE_TRANSMIT;
    }

    return error;
}

otRadioFrame *otPlatRadioGetTransmitBuffer(otInstance *aInstance)
{
    (void)aInstance;
    return &sTransmitFrame;
}

int8_t otPlatRadioGetRssi(otInstance *aInstance)
{
    (void)aInstance;
    return 0;
}

otRadioCaps otPlatRadioGetCaps(otInstance *aInstance)
{
    (void)aInstance;
    return OT_RADIO_CAPS_NONE;
}

bool otPlatRadioGetPromiscuous(otInstance *aInstance)
{
    (void)aInstance;
    return sPromiscuous;
}

void radioReceive(otInstance *aInstance)
{
    ssize_t rval = recvfrom(sSockFd, (char *)&sReceiveMessage, sizeof(sReceiveMessage), 0, NULL, NULL);

    if (rval < 0)
    {
        perror("recvfrom");
        exit(EXIT_FAILURE);
    }

#if OPENTHREAD_ENABLE_RAW_LINK_API
    // Timestamp
    sReceiveFrame.mMsec = otPlatAlarmMilliGetNow();
    sReceiveFrame.mUsec = 0;  // Don't support microsecond timer for now.
#endif

    sReceiveFrame.mLength = (uint8_t)(rval - 1);

    if (sAckWait &&
        sTransmitFrame.mChannel == sReceiveMessage.mChannel &&
        isFrameTypeAck(sReceiveFrame.mPsdu) &&
        getDsn(sReceiveFrame.mPsdu) == getDsn(sTransmitFrame.mPsdu))
    {
        sState = OT_RADIO_STATE_RECEIVE;
        sAckWait = false;

        otPlatRadioTxDone(aInstance, &sTransmitFrame, &sReceiveFrame, OT_ERROR_NONE);
    }
    else if ((sState == OT_RADIO_STATE_RECEIVE || sState == OT_RADIO_STATE_TRANSMIT) &&
             (sReceiveFrame.mChannel == sReceiveMessage.mChannel))
    {
        radioProcessFrame(aInstance);
    }
}

void radioSendMessage(otInstance *aInstance)
{
    sTransmitMessage.mChannel = sTransmitFrame.mChannel;

    otPlatRadioTxStarted(aInstance, &sTransmitFrame);
    radioTransmit(&sTransmitMessage, &sTransmitFrame);

    sAckWait = isAckRequested(sTransmitFrame.mPsdu);

    if (!sAckWait)
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
}

void platformRadioUpdateFdSet(fd_set *aReadFdSet, fd_set *aWriteFdSet, int *aMaxFd)
{
    if (aReadFdSet != NULL && (sState != OT_RADIO_STATE_TRANSMIT || sAckWait))
    {
        FD_SET(sSockFd, aReadFdSet);

        if (aMaxFd != NULL && *aMaxFd < sSockFd)
        {
            *aMaxFd = sSockFd;
        }
    }

    if (aWriteFdSet != NULL && sState == OT_RADIO_STATE_TRANSMIT && !sAckWait)
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

    if (sState == OT_RADIO_STATE_TRANSMIT && !sAckWait)
    {
        radioSendMessage(aInstance);
    }
}

void radioTransmit(struct RadioMessage *aMessage, struct otRadioFrame *aFrame)
{
    uint32_t i;
    struct sockaddr_in sockaddr;

    otRadioFrameFillCRC(aFrame);

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &sockaddr.sin_addr);

    for (i = 1; i <= WELLKNOWN_NODE_ID; i++)
    {
        ssize_t rval;

        if (NODE_ID == i)
        {
            continue;
        }

        sockaddr.sin_port = htons(9000 + sPortOffset + i);
        rval = sendto(sSockFd, (const char *)aMessage, 1 + aFrame->mLength,
                      0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));

        if (rval < 0)
        {
            perror("recvfrom");
            exit(EXIT_FAILURE);
        }
    }
}

void radioSendAck(void)
{
    sAckFrame.mLength = IEEE802154_ACK_LENGTH;
    sAckMessage.mPsdu[0] = IEEE802154_FRAME_TYPE_ACK;

    if (isDataRequest(sReceiveFrame.mPsdu))
    {
        sAckMessage.mPsdu[0] |= IEEE802154_FRAME_PENDING;
    }

    sAckMessage.mPsdu[1] = 0;
    sAckMessage.mPsdu[2] = getDsn(sReceiveFrame.mPsdu);

    sAckMessage.mChannel = sReceiveFrame.mChannel;

    radioTransmit(&sAckMessage, &sAckFrame);
}

void radioProcessFrame(otInstance *aInstance)
{
    otError error = OT_ERROR_NONE;
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

    // generate acknowledgment
    if (isAckRequested(sReceiveFrame.mPsdu))
    {
        radioSendAck();
    }

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
    (void)aInstance;
    (void)aEnable;
}

otError otPlatRadioAddSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return OT_ERROR_NONE;
}

otError otPlatRadioAddSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchShortEntry(otInstance *aInstance, const uint16_t aShortAddress)
{
    (void)aInstance;
    (void)aShortAddress;
    return OT_ERROR_NONE;
}

otError otPlatRadioClearSrcMatchExtEntry(otInstance *aInstance, const uint8_t *aExtAddress)
{
    (void)aInstance;
    (void)aExtAddress;
    return OT_ERROR_NONE;
}

void otPlatRadioClearSrcMatchShortEntries(otInstance *aInstance)
{
    (void)aInstance;
}

void otPlatRadioClearSrcMatchExtEntries(otInstance *aInstance)
{
    (void)aInstance;
}

otError otPlatRadioEnergyScan(otInstance *aInstance, uint8_t aScanChannel, uint16_t aScanDuration)
{
    (void)aInstance;
    (void)aScanChannel;
    (void)aScanDuration;
    return OT_ERROR_NOT_IMPLEMENTED;
}

void otPlatRadioSetDefaultTxPower(otInstance *aInstance, int8_t aPower)
{
    (void)aInstance;
    (void)aPower;
}

int8_t otPlatRadioGetReceiveSensitivity(otInstance *aInstance)
{
    (void)aInstance;
    return POSIX_RECEIVE_SENSITIVITY;
}
