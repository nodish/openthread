#!/bin/sh
DUT_VERSION=`./start.sh -l COM29 | cut -d' ' -f2-` ./regression.sh
./gencsv.py output/result.json
mv result.csv output
cp -r output "/c/Users/nest/Google 云端硬盘/results/NXP-OT/`date +%Y%m%d`"
