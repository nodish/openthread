#ifndef OPENTHREAD_LINE_READER_H_
#define OPENTHREAD_LINE_READER_H_

#include "openthread-core-config.h"

#include <stdint.h>
#include "common/instance.hpp"

template <typename kStream> class LineReader
{
public:
    LineReader(void)
        : mRxLength(0)
    {
    }

    void Input(const uint8_t *aBuf, uint16_t aBufLength)
    {
        static const char sCommandPrompt[] = {'>', ' '};
        static const char sEraseString[]   = {'\b', ' ', '\b'};
        static const char CRNL[]           = {'\r', '\n'};
        const uint8_t *   end;

        end = aBuf + aBufLength;

        for (; aBuf < end; aBuf++)
        {
            switch (*aBuf)
            {
            case '\r':
            case '\n':
                static_cast<kStream *>(this)->Output(CRNL, sizeof(CRNL));

                if (mRxLength > 0)
                {
                    mRxBuffer[mRxLength] = '\0';

                    if (mRxBuffer[mRxLength - 1] == '\n')
                    {
                        mRxBuffer[--mRxLength] = '\0';
                    }

                    if (mRxBuffer[mRxLength - 1] == '\r')
                    {
                        mRxBuffer[--mRxLength] = '\0';
                    }

#if OPENTHREAD_CONFIG_LOG_OUTPUT != OPENTHREAD_CONFIG_LOG_OUTPUT_NONE
                    /*
                     * Note this is here for this reason:
                     *
                     * TEXT (command) input ... in a test automation script occurs
                     * rapidly and often without gaps between the command and the
                     * terminal CR
                     *
                     * In contrast as a human is typing there is a delay between the
                     * last character of a command and the terminal CR which executes
                     * a command.
                     *
                     * During that human induced delay a tasklet may be scheduled and
                     * the LOG becomes confusing and it is hard to determine when
                     * something happened.  Which happened first? the command-CR or
                     * the tasklet.
                     *
                     * Yes, while rare it is a race condition that is hard to debug.
                     *
                     * Thus this is here to affirmatively LOG exactly when the CLI
                     * command is being executed.
                     */
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
                    /* TODO: how exactly do we get the instance here? */
#else
                    otLogInfoCli("execute command: %s", mRxBuffer);
#endif
#endif
                    if (mRxLength > 0)
                    {
                        static_cast<kStream *>(this)->ProcessLine(mRxBuffer, mRxLength);
                    }

                    mRxLength = 0;
                }

                static_cast<kStream *>(this)->Output(sCommandPrompt, sizeof(sCommandPrompt));

                break;

#if OPENTHREAD_POSIX

            case 0x04: // ASCII for Ctrl-D
                exit(EXIT_SUCCESS);
                break;
#endif

            case '\b':
            case 127:
                if (mRxLength > 0)
                {
                    static_cast<kStream *>(this)->Output(sEraseString, sizeof(sEraseString));
                    mRxBuffer[--mRxLength] = '\0';
                }

                break;

            default:
                if (mRxLength < kRxBufferSize)
                {
                    static_cast<kStream *>(this)->Output(reinterpret_cast<const char *>(aBuf), 1);
                    mRxBuffer[mRxLength++] = static_cast<char>(*aBuf);
                }

                break;
            }
        }
    }

private:
    enum
    {
        kRxBufferSize = OPENTHREAD_CONFIG_CLI_UART_RX_BUFFER_SIZE,
    };

    char     mRxBuffer[kRxBufferSize];
    uint16_t mRxLength;
};

#endif // OPENTHREAD_LINE_READER_H_
