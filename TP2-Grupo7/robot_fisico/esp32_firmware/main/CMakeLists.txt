idf_component_register(SRCS "main.c" "motor_driver.c"
                       INCLUDE_DIRS "."
                    PRIV_REQUIRES micro_ros_espidf_component
                    REQUIRES driver esp_timer esp_rom)
target_link_libraries(${COMPONENT_LIB} PRIVATE m)