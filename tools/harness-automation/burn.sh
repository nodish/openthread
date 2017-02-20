#!/bin/sh
set -e
ARGS=`getopt vsni:f: $*`
set -- $ARGS

cd `dirname $0`
INPUT_FILE=
FIRMWARE_FILE=
DRY_RUN=
BURN_PREFIX=nohup
BURN_SUFFIX='> /dev/null &'

if [ -z "$FIRMWARE_URL" ]; then
  FIRMWARE_URL='http://nest:e620a193761e6c30b8aec31bc51292f1@192.168.1.2:8080/job/OpenThread/job/firmware/ws'
fi

while true; do
  case "$1" in
    -i)
      INPUT_FILE=$2
      shift
      ;;
    -f)
      FIRMWARE_FILE=$2
      shift
      ;;
    -h)
      echo "$0 - tool to burn firmware to cc2538"
      echo "   -f FIRMWARE_FILE, if not specified, latest will be download if possible"
      echo "   -i INPUT_FILE, one device name per line"
      echo "   -n dry run, only show what will be performed"
      echo "   -s disable concurrent burning"
      echo "   -v verify version after burning"
      echo "   -h show me help"
      exit 0
      ;;
    -s)
      BURN_PREFIX=
      BURN_SUFFIX=
      ;;
    -v)
      VERIFY_FIRMWARE=1
      ;;
    -n)
      DRY_RUN=echo
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "Internal error!"
      exit 1
      ;;
  esac
  shift
done

do_burn()
{
  $DRY_RUN eval $BURN_PREFIX python cc2538-burner/cc2538-bsl.py -b 460800 -q -e -w -v -p $1 -a 0x00200000 $FIRMWARE_FILE $BURN_SUFFIX
}

do_verify()
{
  VERSION=`./start.sh -l $1`
  echo "$VERSION"
  $DRY_RUN expr "$VERSION" : '.\+'$LATEST_VERSION > /dev/null
  if [ "$?" != "0" ]; then
    echo 'Firmware not updated'
    exit 1
  fi
}

if [ -z "$FIRMWARE_FILE" ]; then
	LATEST_VERSION=`curl -q $FIRMWARE_URL/LATEST_FIRMWARE`
	FIRMWARE_FILE="arm-none-eabi-ot-cli-$LATEST_VERSION.bin"
	$DRY_RUN curl -q -O "$FIRMWARE_URL/outputs/$FIRMWARE_FILE"
  echo 'Latest version is' $LATEST_VERSION
fi

if [ -z "$INPUT_FILE" ]; then
  while [ "$1" ]; do
    echo "$1"
    shift
  done > tmp_devices
  INPUT_FILE=tmp_devices
fi

while read port; do
  do_burn $port
done < $INPUT_FILE

echo -n 'Waiting jobs to complete..'
wait
echo 'OK'

if [ -n "$VERIFY_FIRMWARE" ]; then
  echo -n 'Start verifying..'
  while read port; do
    do_verify $port
  done < $INPUT_FILE
  echo 'OK'
fi
echo 'Done'
