# todo: split esp32 part in seperate CMakeLists.txt file
idf_component_register(SRCS ./spc700.c ./memmap.c ./apu.c ./globals.c ./c4emu.c ./cpuops.c ./dsp1emu.c ./srtc.c ./apu_blargg.c ./clip.c ./cpu.c ./gfx.c ./soundux.c ./cpuexec.c ./obc1.c ./dsp2emu.c ./tile.c ./ppu.c ./dma.c ./save.c ./c4.c ./dsp1.c ./getset.c
                       INCLUDE_DIRS ./..
                       REQUIRES main esp_event esp_partition esp_adc nvs_flash fatfs spiffs driver # anything except for main is only reqired for esp32
                       PRIV_REQUIRES )

# only required for esp32
#set (CMAKE_CXX_FLAGS "-Wno-misleading-indentation -Wno-narrowing -Wno-format -mlongcalls -fpermissive")
set (CMAKE_C_FLAGS "-march=rv32imafc -mabi=ilp32f -Wno-format -Wno-unused-const-variable")
