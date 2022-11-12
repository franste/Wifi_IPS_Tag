idf_component_register(SRCS "wifiSTA.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi esp_timer json lwip esp_http_client esp-tls wifi_packets
                    )
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format") 