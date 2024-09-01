# . ${IDF_PATH}/add_path.sh
# esptool.py --chip esp32p4 --port "/dev/ttyUSB0" --baud 921600 write_flash -fs detect --flash_freq 40m --flash_mode qio 0x0110000 build/snes-retrogo.bin
esptool.py --chip esp32p4 --port "/dev/ttyUSB0" --baud 921600 write_flash -fs detect --flash_freq 40m --flash_mode qio 0x10000 build/snes-retrogo.bin
