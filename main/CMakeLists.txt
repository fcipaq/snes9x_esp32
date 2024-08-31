idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS ./.. ./../components/lcddrv ./../components/snes9x "."
                       REQUIRES
                       PRIV_REQUIRES )

target_compile_options(${COMPONENT_LIB} PRIVATE -DCOMPILEDATE="$(COMPILEDATE)" -DGITREV="$(GITREV)")

#set (CMAKE_CXX_FLAGS "")
#set (CMAKE_C_FLAGS "-mlongcalls")
