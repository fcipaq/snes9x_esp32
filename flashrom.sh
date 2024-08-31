#!/bin/bash
# . ${IDF_PATH}/add_path.sh
esptool.py --chip esp32p4 --baud 921600 write_flash -fs detect --flash_freq 40m --flash_mode qio 0x400000 "$1"
