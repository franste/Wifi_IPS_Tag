idf_component_register(SRCS "repl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash json storage console cmd_system)
