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
 *   This file includes definitions for the spinel based radio transceiver.
 */

#ifndef RADIO_SPINEL_HPP_
#define RADIO_SPINEL_HPP_

#include "frame_queue.hpp"
#include "hdlc.hpp"
#include "spinel.h"

namespace ot {

class RadioSpinel
{
public:
    /**
     * This constructor initializes the spinel based OpenThread transceiver.
     *
     */
    RadioSpinel(void);

    /**
     * Initialize this radio transceiver.
     *
     * @param[in]   aRadioFile    The path to either a uart device or an executable.
     * @param[in]   aRadioConfig  Parameters given to the device or executable.
     *
     */
    void Init(const char *aRadioFile, const char *aRadioConfig);

    /**
     * Deinitialize this radio transceiver.
     *
     */
    void Deinit(void);

    /**
     * This method gets the status of promiscuous mode.
     *
     * @retval true   Promiscuous mode is enabled.
     * @retval false  Promiscuous mode is disabled.
     *
     */
    bool GetPromiscuous(void) const { return mPromiscuous; }

    /**
     * This method sets the status of promiscuous mode.
     *
     * @param[in]   aEnable     Whether to enable or disable promiscuous mode.
     *
     */
    void SetPromiscuous(bool aEnable);

    /**
     * Set the Short Address for address filtering.
     *
     * @param[in] aShortAddress  The IEEE 802.15.4 Short Address.
     *
     */
    void SetShortAddress(uint16_t aAddress);

    /**
     * Set the Extended Address for address filtering.
     *
     * @param[in] aExtAddress  A pointer to the IEEE 802.15.4 Extended Address stored in little-endian byte order.
     *
     */
    void SetExtendedAddress(const otExtAddress &aAddress);

    /**
     * Set the PAN ID for address filtering.
     *
     * @param[in]   aPanId  The IEEE 802.15.4 PAN ID.
     *
     */
    void SetPanId(uint16_t aPanId);

    /**
     * This method returns the radio receive sensitivity value.
     *
     * @returns The radio receive sensitivity value in dBm.
     *
     */
    uint8_t GetReceiveSensitivity(void) const { return mRxSensitivity; }

    /**
     * This method returns a reference to the transmit buffer.
     *
     * The caller forms the IEEE 802.15.4 frame in this buffer then calls otPlatRadioTransmit() to request transmission.
     *
     * @returns A reference to the transmit buffer.
     *
     */
    otRadioFrame &GetTransmitFrame(void) { return mTxRadioFrame; }

    /**
     * This method switches the radio from Receive to Transmit.
     *
     * @param[in] aFrame     A reference to the transmitted frame.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Transmit.
     * @retval OT_ERROR_INVALID_STATE The radio was not in the Receive state.
     */
    otError Transmit(otRadioFrame &aFrame);

    /**
     * This method swithes the radio from Sleep to Receive.
     *
     * @param[in]  aChannel   The channel to use for receiving.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Receive.
     * @retval OT_ERROR_INVALID_STATE The radio was disabled or transmitting.
     *
     */
    otError Receive(uint8_t aChannel);

    /**
     * This method switches the radio from Receive to Sleep.
     *
     * @retval OT_ERROR_NONE          Successfully transitioned to Sleep.
     * @retval OT_ERROR_BUSY          The radio was transmitting
     * @retval OT_ERROR_INVALID_STATE The radio was disabled
     *
     */
    otError Sleep(void);

    /**
     * Enable the radio.
     *
     * @param[in]   aInstance           A pointer to the OpenThread instance.
     *
     * @retval OT_ERROR_NONE     Successfully enabled.
     * @retval OT_ERROR_FAILED   The radio could not be enabled.
     *
     */
    otError Enable(otInstance *aInstance);

    /**
     * Disable the radio.
     *
     * @retval OT_ERROR_NONE  Successfully transitioned to Disabled.
     *
     */
    otError Disable(void);

    /**
     * Check whether radio is enabled or not.
     *
     * @returns TRUE if the radio is enabled, FALSE otherwise.
     *
     */
    bool IsEnabled(void) const { return mState != OT_RADIO_STATE_DISABLED; }

    /**
     * This method updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[inout]  aReadFdSet   A reference to the read file descriptors.
     * @param[inout]  aWriteFdSet  A reference to the write file descriptors.
     * @param[inout]  aMaxFd       A reference to the max file descriptor.
     * @param[inout]  aTimeout     A reference to the timeout.
     *
     */
    void UpdateFdSet(fd_set &aReadFdSet, fd_set &aWriteFdSet, int &aMaxFd, struct timeval &aTimeout);

    /**
     * This method performs radio driver processing.
     *
     * @param[in]   aReadFdSet      A reference to the read file descriptors.
     * @param[in]   aWriteFdSet     A reference to the write file descriptors.
     *
     */
    void Process(const fd_set &aReadFdSet, const fd_set &aWriteFdSet);

    /**
     * This method returns whether there are pending spinel frames.
     *
     * @retval  true    There is at least one frame queued.
     * @retval  false   There no frames queued.
     *
     */
    bool IsFrameQueued(void) const;

    /**
     * This method tries to retrieve a spinel property from OpenThread transceiver.
     *
     * @param[in]   aKey        Spinel property key.
     * @param[in]   aFormat     Spinel formatter to unpack property value.
     * @param[out]  ...         Variable arguments list.
     *
     * @retval  OT_ERROR_NONE               Successfully got the property.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Get(spinel_prop_key_t aKey, const char *aFormat, ...);

    /**
     * This method tries to update a spinel property of OpenThread transceiver.
     *
     * @param[in]   aKey        Spinel property key.
     * @param[in]   aFormat     Spinel formatter to pack property value.
     * @param[in]   ...         Variable arguments list.
     *
     * @retval  OT_ERROR_NONE               Successfully set the property.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Set(spinel_prop_key_t aKey, const char *aFormat, ...);

    /**
     * This method tries to insert a item into a spinel list property of OpenThread transceiver.
     *
     * @param[in]   aKey        Spinel property key.
     * @param[in]   aFormat     Spinel formatter to pack the item.
     * @param[in]   ...         Variable arguments list.
     *
     * @retval  OT_ERROR_NONE               Successfully insert item into the property.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Insert(spinel_prop_key_t aKey, const char *aFormat, ...);

    /**
     * This method tries to remove a item from a spinel list property of OpenThread transceiver.
     *
     * @param[in]   aKey        Spinel property key.
     * @param[in]   aFormat     Spinel formatter to pack the item.
     * @param[in]   ...         Variable arguments list.
     *
     * @retval  OT_ERROR_NONE               Successfully removed item from the property.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Remove(spinel_prop_key_t aKey, const char *aFormat, ...);

    /**
     * This method transmits a radio frame through OpenThread transceiver.
     *
     * @retval  OT_ERROR_NONE               Successfully transmitted the radio frame.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError ProcessTransmit(void);

private:
    enum
    {
        kMaxSpinelFrame = 2048, ///< Max size in bytes for transfering spinel frames.
        kMaxWaitTime    = 2000, ///< Max time to wait for response in milliseconds.
    };

    void         Receive(void);
    spinel_tid_t GetNextTid(void);
    void         FreeTid(spinel_tid_t tid) { mCmdTidsInUse &= ~(1 << tid); }
    otError      RequestV(bool aWait, uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, va_list aArgs);
    otError      Request(bool aWait, uint32_t aCommand, spinel_prop_key_t aKey, const char *aFormat, ...);
    otError      WaitResponse(void);
    otError      SendAll(const uint8_t *aBuffer, uint16_t aLength);
    otError      SendReset(void);
    otError      SendCommand(uint32_t          command,
                             spinel_prop_key_t key,
                             spinel_tid_t      tid,
                             const char *      pack_format,
                             va_list           args);

    static void HandleHdlcError(void *aContext, otError aError, uint8_t *aBuffer, uint16_t aLength);

    static void HandleSpinelFrame(void *aContext, uint8_t *aBuffer, uint16_t aLength)
    {
        static_cast<RadioSpinel *>(aContext)->HandleSpinelFrame(aBuffer, aLength);
    }
    void HandleSpinelFrame(const uint8_t *aBuffer, uint16_t aLength);

    otError ParseRadioFrame(otRadioFrame &aFrame, const uint8_t *aBuffer, uint16_t aLength);
    void    ProcessCommand(uint32_t aCommand, const uint8_t *aBuffer, uint16_t aLength);
    void    ProcessValueIs(spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void    HandleResult(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void    HandleTransmitDone(uint32_t aCommand, spinel_prop_key_t aKey, const uint8_t *aBuffer, uint16_t aLength);
    void    ProcessNotification(const uint8_t *aBuffer, uint16_t aLength);
    void    ProcessResponse(const uint8_t *aBuffer, uint16_t aLength);
    void    ProcessQueue(void);
    void    ProcessRadioFrame(void);

    otInstance *mInstance;

    uint16_t          mCmdTidsInUse;    ///< Used transaction ids.
    spinel_tid_t      mCmdNextTid;      ///< Next available transaction id.
    spinel_tid_t      mTxRadioTid;      ///< The transaction id used to send a radio frame.
    spinel_tid_t      mWaitingTid;      ///< The transaction id of current transaction.
    spinel_prop_key_t mWaitingKey;      ///< The property key of current transaction.
    const char *      mPropertyFormat;  ///< The spinel property format of current transaction.
    va_list           mPropertyArgs;    ///< The arguments pack or unpack spinel property of current transcation.
    uint32_t          mExpectedCommand; ///< Expected response command of current transaction.
    otError           mError;           ///< The result of current transaction.

    uint8_t       mHdlcBuffer[kMaxSpinelFrame];
    Hdlc::Decoder mHdlcDecoder;
    Hdlc::Encoder mHdlcEncoder;
    FrameQueue    mFrameQueue;

    uint8_t       mRxPsdu[OT_RADIO_FRAME_MAX_SIZE];
    uint8_t       mTxPsdu[OT_RADIO_FRAME_MAX_SIZE];
    otRadioFrame  mRxRadioFrame;
    otRadioFrame  mTxRadioFrame;
    otRadioFrame *mTransmitFrame; ///< Points to the frame to send

    uint8_t  mExtendedAddress[OT_EXT_ADDRESS_SIZE];
    uint16_t mShortAddress;
    uint16_t mPanid;
    uint8_t  mChannel;
    int8_t   mRxSensitivity;
    uint8_t  mTxState;
    otError  mTxError;

    int          mSockFd;
    otRadioState mState;
    bool         mAckWait;
    bool         mPromiscuous;
    bool         mIsReady;
};

} // namespace ot

#endif // RADIO_SPINEL_HPP_
