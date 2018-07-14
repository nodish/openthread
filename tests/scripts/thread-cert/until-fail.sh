#!/bin/sh
set -e
while true; do
    VIRTUAL_TIME=1 BUILD_TARGET=posix-ncp ./run.sh $@
done
