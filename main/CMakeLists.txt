idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES jsonUtil storage repl wifiSTA espNowMaster)
