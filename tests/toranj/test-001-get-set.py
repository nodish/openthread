#!/usr/bin/env python3
#
#  Copyright (c) 2018, The OpenThread Authors.
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

from wpan import verify
import wpan

# -----------------------------------------------------------------------------------------------------------------------
# Test description: simple wpanctl get and set commands

test_name = __file__[:-3] if __file__.endswith('.py') else __file__
print('-' * 120)
print('Starting \'{}\''.format(test_name))

# -----------------------------------------------------------------------------------------------------------------------
# Creating `wpan.Nodes` instances

node = wpan.Node()

# -----------------------------------------------------------------------------------------------------------------------
# Init all nodes

wpan.Node.init_all_nodes()

# -----------------------------------------------------------------------------------------------------------------------
# Test implementation

verify(node.get(wpan.WPAN_STATE) == wpan.STATE_OFFLINE)

# set some of properties and check and verify that the value is indeed
# changed...

node.set(wpan.WPAN_NAME, 'test-network')
verify(node.get(wpan.WPAN_NAME) == '"test-network"')

node.set(wpan.WPAN_NAME, 'a')
verify(node.get(wpan.WPAN_NAME) == '"a"')

node.set(wpan.WPAN_PANID, '0xABBA')
verify(node.get(wpan.WPAN_PANID) == '0xABBA')

node.set(wpan.WPAN_XPANID, '1020031510006016', binary_data=True)
verify(node.get(wpan.WPAN_XPANID) == '0x1020031510006016')

node.set(wpan.WPAN_KEY, '0123456789abcdeffecdba9876543210', binary_data=True)
verify(node.get(wpan.WPAN_KEY) == '[0123456789ABCDEFFECDBA9876543210]')

node.set(wpan.WPAN_MAC_WHITELIST_ENABLED, '1')
verify(node.get(wpan.WPAN_MAC_WHITELIST_ENABLED) == 'true')

node.set(wpan.WPAN_MAC_WHITELIST_ENABLED, '0')
verify(node.get(wpan.WPAN_MAC_WHITELIST_ENABLED) == 'false')

node.set(wpan.WPAN_MAC_WHITELIST_ENABLED, 'true')
verify(node.get(wpan.WPAN_MAC_WHITELIST_ENABLED) == 'true')

node.set(wpan.WPAN_THREAD_ROUTER_UPGRADE_THRESHOLD, '100')
verify(int(node.get(wpan.WPAN_THREAD_ROUTER_UPGRADE_THRESHOLD), 0) == 100)

node.set(wpan.WPAN_THREAD_ROUTER_DOWNGRADE_THRESHOLD, '40')
verify(int(node.get(wpan.WPAN_THREAD_ROUTER_DOWNGRADE_THRESHOLD), 0) == 40)

verify(int(node.get(wpan.WPAN_THREAD_ROUTER_DOWNGRADE_THRESHOLD), 0) == 40)

all_posix_app_gettable_props = [wpan.WPAN_RCP_VERSION]

# note: partitionid only takes effect after forming one Thread network.
node.set(wpan.WPAN_PARTITION_ID, '12345678')

node.form('get-set')

# verify that partitionid property is indeed changed.
verify(int(node.get(wpan.WPAN_PARTITION_ID), 0) == 12345678)

# Ensure `get` is successful with all gettable properties
for prop in wpan.ALL_GETTABLE_PROPS:
    node.get(prop)

if node.using_posix_app_with_rcp:
    for prop in all_posix_app_gettable_props:
        node.get(prop)

# -----------------------------------------------------------------------------------------------------------------------
# Test finished

wpan.Node.finalize_all_nodes()

print('\'{}\' passed.'.format(test_name))
