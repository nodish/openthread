#!/bin/sh
DUT_VERSION=`./start.sh -l COM10 | cut -d' ' -f2-` ./regression.sh -b blacklist-rfshield.txt
./gencsv.py output/result.json
mv result.csv output
cp -r output "/c/Users/nest/Google Drive/results/OT-NXP/`date +%Y%m%d`"
