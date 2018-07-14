#!/bin/sh
rm -rf tmp

case "$BUILD_TARGET" in
posix-cli)
    VERBOSE=1 top_builddir=../../../build/x86_64-unknown-linux-gnu python $@
    ;;
posix-ncp)
    VERBOSE=1 NODE_TYPE=ncp-sim top_builddir=../../../build/x86_64-unknown-linux-gnu python $@
    ;;
posix-app-cli)
    sudo truncate -s 0 /var/log/syslog
    VERBOSE=1 RADIO_DEVICE=../../../build/x86_64-unknown-linux-gnu/examples/apps/ncp/ot-ncp-radio OT_CLI_PATH=../../../build/posix/x86_64-unknown-linux-gnu/src/posix/ot-cli python $@ 2>&1 | tee current.log
    sudo cp /var/log/syslog .
    tail current.log | grep OK
    ;;
*)
    false
    ;;
esac
