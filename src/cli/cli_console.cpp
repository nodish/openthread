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
 *   This file implements the CLI server on the CONSOLE service.
 */

#include "cli_console.hpp"

#if OPENTHREAD_CONFIG_CLI_TRANSPORT == OT_CLI_TRANSPORT_CONSOLE

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "cli/cli.hpp"
#include "common/instance.hpp"
#include "common/new.hpp"

namespace ot {
namespace Cli {

static OT_DEFINE_ALIGNED_VAR(sCliConsoleRaw, sizeof(Console), uint64_t);

extern "C" void otCliConsoleInit(otInstance *aInstance, otCliConsoleOutputCallback aCallback, void *aContext)
{
    Console::Initialize(aInstance, aCallback, aContext);
}

extern "C" void otCliConsoleInputLine(char *aBuf, uint16_t aBufLength)
{
    Interpreter::GetInterpreter().ProcessLine(aBuf, aBufLength);
}

// Add stubs for simulation
extern "C" void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    OT_UNUSED_VARIABLE(aBuf);
    OT_UNUSED_VARIABLE(aBufLength);
}

extern "C" void otPlatUartSendDone(void)
{
}

Console *Console::sConsole = nullptr;

Interpreter &Interpreter::GetInterpreter(void)
{
    return *Console::sConsole;
}

void Console::Initialize(otInstance *aInstance, otCliConsoleOutputCallback aCallback, void *aContext)
{
    Instance *instance = static_cast<Instance *>(aInstance);

    sConsole = new (&sCliConsoleRaw) Console(instance, aCallback, aContext);
}

Console::Console(Instance *aInstance, otCliConsoleOutputCallback aCallback, void *aContext)
    : Interpreter(aInstance)
    , mCallback(aCallback)
    , mContext(aContext)
{
}

int Interpreter::Output(const char *aBuf, uint16_t aBufLength)
{
    return static_cast<Console *>(this)->Output(aBuf, aBufLength);
}

int Console::Output(const char *aBuf, uint16_t aBufLength)
{
    return mCallback(aBuf, aBufLength, mContext);
}

} // namespace Cli
} // namespace ot

#endif // OPENTHREAD_CONFIG_CLI_TRANSPORT == OT_CLI_TRANSPORT_CONSOLE
