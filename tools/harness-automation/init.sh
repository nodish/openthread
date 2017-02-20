#!/bin/sh
sed -i -b 's/ThreadChannel = .\{2\}/ThreadChannel = '$1'/'  /c/GRL/Thread1.1/Config/Configuration.ini
sed -i -b 's/THREAD_CHANNEL = .\{2\}/THREAD_CHANNEL = '$1'/' autothreadharness/settings.py
sed -i -b 's/ThreadSecondChannel = .\{2\}/ThreadSecondChannel = '$2'/'  /c/GRL/Thread1.1/Config/Configuration.ini
