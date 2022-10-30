idf_component_register(SRCS "wifiSTA.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi esp_timer json lwip)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format") 