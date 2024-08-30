# This is Snes9x running on the brand new ESP32-p4

This is based on [this](https://github.com/ducalex/retro-go/) repo.

The ESP-IDF MIPI driver allocates a frame buffer in PSRAM. That is great for content where most of the screen basically won't change from frame to frame but is very slow when using dynamic content (like video games) where the screen content is usually regenerated every single frame.
Therefore the MIPI driver must be hacked to use an IRAM line buffer instead. You need to replace `esp-idf/components/esp_lcd/dsi/esp_lcd_panel_dpi.c` with a hacked version. (Rember to switch back as this will break other stuff.)

You can watch a comparison video on [Youtube](https://youtu.be/osw1QMM4Avs)

In order to run a ROM, you need to flash the ROM to `0x400000` of the spi flash using esptool:

`esptool.py -p /dev/ttyUSB0 write_flash 0x400000 YourROM.smc`

Then, in `main.cpp`, adjust `Memory.ROM_Size` accordingly. 

I built it using ESP-IDF v.5.4.0:

`idf.py build flash`

Partially unrelated: Snes9x also runs on the ESP32-S3 with approx. 45 fps. You can watch it [here](https://www.youtube.com/watch?v=lVLDIexSZ18).