#
#  Copyright (c) 2016, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

CPPFLAGS_COMMON                                                        = \
    -I$(top_srcdir)/include                                              \
    -I$(top_srcdir)/src/core                                             \
    -I$(top_srcdir)/examples/platforms                                   \
    $(NULL)

LDADD_COMMON                                                           = \
    $(NULL)

LDFLAGS_COMMON                                                         = \
    $(NULL)

SOURCES_COMMON                                                         = \
    @top_srcdir@/src/core/coap/coap_base.cpp                \
    @top_srcdir@/src/core/coap/coap_server.cpp              \
    @top_srcdir@/src/core/coap/coap_client.cpp              \
    @top_srcdir@/src/core/coap/coap_header.cpp              \
    @top_srcdir@/src/core/coap/secure_coap_server.cpp              \
    @top_srcdir@/src/core/coap/secure_coap_client.cpp              \
    @top_srcdir@/src/core/common/logging.cpp                \
    @top_srcdir@/src/core/common/message.cpp                \
    @top_srcdir@/src/core/common/tasklet.cpp                  \
    @top_srcdir@/src/core/common/timer.cpp                  \
    @top_srcdir@/src/core/common/tlvs.cpp                  \
    @top_srcdir@/src/core/crypto/mbedtls.cpp                  \
    @top_srcdir@/src/core/crypto/sha256.cpp                  \
    @top_srcdir@/src/core/meshcop/dtls.cpp                 \
    @top_srcdir@/src/core/meshcop/border_agent.cpp                 \
    @top_srcdir@/src/core/net/udp6.cpp                  \
    @top_srcdir@/src/core/net/ip6_address.cpp                  \
    @top_srcdir@/examples/platforms/posix/alarm.c                  \
    @top_srcdir@/examples/platforms/posix/random.c                  \
    @top_srcdir@/examples/platforms/posix/logging.c                  \
    platform_udp.c                                                \
    main.cpp                                                \
    api.cpp                                                 \
    $(NULL)

if OPENTHREAD_ENABLE_BUILTIN_MBEDTLS
LDADD_COMMON                                                          += \
    $(top_builddir)/third_party/mbedtls/libmbedcrypto.a                  \
    $(NULL)
endif # OPENTHREAD_ENABLE_BUILTIN_MBEDTLS

if OPENTHREAD_EXAMPLES_POSIX
LDADD_COMMON                                                          += \
    -lstdc++                                                             \
    $(NULL)
endif # OPENTHREAD_EXAMPLES_POSIX

bin_PROGRAMS                                                             = \
    border-agent

border_agent_CPPFLAGS                                                    = \
    -DPLATFORM_UDP                                                      \
    $(CPPFLAGS_COMMON)                                                   \
    $(NULL)

border_agent_LDADD                                                       = \
    $(LDADD_COMMON)                                                      \
    $(NULL)

border_agent_LDFLAGS                                                     = \
    $(LDFLAGS_COMMON)                                                    \
    $(NULL)

border_agent_SOURCES                                                     = \
    $(SOURCES_COMMON)                                                    \
    $(NULL)

if OPENTHREAD_BUILD_COVERAGE
CLEANFILES                                                             = $(wildcard *.gcda *.gcno)
endif # OPENTHREAD_BUILD_COVERAGE

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
