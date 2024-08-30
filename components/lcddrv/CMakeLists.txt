# todo: split esp32 part in seperate CMakeLists.txt file
idf_component_register(SRCS ./esp_lcd_ek79007.c
                       INCLUDE_DIRS ./.. ./include
                       REQUIRES main esp_event driver # anything except for main is only reqired for esp32
                       PRIV_REQUIRES )

# only required for esp32
set (CMAKE_CXX_FLAGS "-march=rv32imafc -mabi=ilp32f") #"-Wno-misleading-indentation -Wno-narrowing -Wno-format -mlongcalls -fpermissive")
set (CMAKE_C_FLAGS "-march=rv32imafc -mabi=ilp32f -Wno-format -Wno-unused-const-variable")
