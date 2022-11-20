idf_component_register(SRCS "spi_bmp390.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver driver_bmp390)
target_compile_options(${COMPONENT_LIB} PRIVATE  -Wno-format)                    
                    
