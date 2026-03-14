idf_component_register(SRCS "esp32_hx8347.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES 
                        esp_driver_gpio
                        driver
                        esp_rom
                        soc 
                        freertos)
