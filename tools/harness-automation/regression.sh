#!/bin/sh
set -ex

NAME=`date +%Y%m%d`
# prepare files
[ -d results ] || mkdir results
[ -d output ] || mkdir output
cp autothreadharness/settings_daily.py autothreadharness/settings.py
find -name '*.pyc' -delete

# patch settings
echo >> autothreadharness/settings.py
echo DUT_VERSION=\'$DUT_VERSION\' >> autothreadharness/settings.py
echo >> autothreadharness/settings.py
[ ! -f settings_custom ] || ( cat settings_custom >> autothreadharness/settings.py )
find -name '*.pyc' -delete

# save meta data
HARNESS_VERSION=`grep 'Version' /c/GRL/Thread1.1/info.ini | cut -d= -f2`
echo '{"firmware": "'$DUT_VERSION'",' > output/meta.json
echo '"harness": "'$HARNESS_VERSION'"}' >> output/meta.json

# start running
./start.sh -d -k efp $@ || true
./start.sh -d -k p $@ || true
./start.sh -d -k p $@ || true
./start.sh -d -k p $@ || true
mv output results/$NAME
