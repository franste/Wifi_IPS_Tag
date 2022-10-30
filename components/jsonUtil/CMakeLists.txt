idf_component_register(SRCS "jsonUtil.c"
                    INCLUDE_DIRS "include"
                    REQUIRES json wifiSTA)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format") 
