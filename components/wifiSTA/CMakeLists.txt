idf_component_register(SRCS "wifiSTA.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi wpa_supplicant esp_timer json lwip esp_http_client esp_websocket_client storage json esp_system
                    )
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format") 