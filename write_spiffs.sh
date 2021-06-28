#!/bin/bash

set -e

SKIP_TESTER="0"

if [ $# -eq "1" ]; then
  SKIP_TESTER="$1"
fi

. ${0%/*}/build.env

GREEN="\033[0;32m"
NC="\033[0m"
SERIAL_PORT=${SERIAL_PORT:-"/dev/ttyUSB1"}

echo -e "Creating spiffs.bin partition:"
rm -rf spiffs_fs_gz
mkdir -p spiffs_fs_gz
for i in $(ls ./spiffs_fs); do
  gzip -c "./spiffs_fs/$i" >> "./spiffs_fs_gz/$i"
  echo -e "Created spiffs_fs_gz/$i: ${GREEN}Done${NC}"
done

if [ $SKIP_TESTER -eq "0" ]; then
  echo tester >> spiffs_fs_gz/tester.html
fi

mkspiffs -c spiffs_fs_gz/ -b 4096 -p 256 -s 0x8000 spiffs.bin
echo -e "Created spiffs.bin: ${GREEN}Done${NC}"

python $IDF_PATH/components/esptool_py/esptool/esptool.py --chip esp32 --port ${SERIAL_PORT} --baud 115200 write_flash -z 0x3f0000 spiffs.bin
rm spiffs.bin
rm -rf spiffs_fs_gz
