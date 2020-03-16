
#if OPENTHREAD_POSIX_VIRTUAL_TIME
/**
 * This method process read data (decode the data).
 *
 * This method is intended only for virtual time simulation. Its behavior is similar to `Read()` but instead of
 * reading the data from the radio socket, it uses the given data in the buffer `aBuffer`.
 *
 * @param[in] aBuffer  A pointer to buffer containing data.
 * @param[in] aLength  The length (number of bytes) in the buffer.
 *
 */
void Input(const uint8_t *aBuffer, uint16_t aLength)
{
    Decode(aBuffer, aLength);
}

otError FileDescriptor::Write(const uint8_t *aFrame, uint16_t aLength)
{
    virtualTimeSendRadioSpinelWriteEvent(aFrame, aLength);
}

otError Hdlc::WaitForFrame(uint32_t aTimeout)
{
    struct Event event;
    uint64_t     delay = aTimeout;

    virtualTimeSendSleepEvent(aTimeout);
    virtualTimeReceiveEvent(&event);

    switch (event.mEvent)
    {
    case OT_SIM_EVENT_RADIO_SPINEL_WRITE:
        mHdlcDecoder.Decode(event.mData, event.mDataLength);
        break;

    case OT_SIM_EVENT_ALARM_FIRED:
        VerifyOrExit(event.mDelay <= delay, error = OT_ERROR_RESPONSE_TIMEOUT);
        break;

    default:
        OT_ASSERT(false);
        break;
    }
}
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME
