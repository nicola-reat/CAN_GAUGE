idf_component_register(SRCS "gauge_alarm.c" "gauge_cfg.c" "gauge_display" "adc_sensor.c" "main.c" "can_node.c"
                       INCLUDE_DIRS "."
                       REQUIRES lcd_st77916 ui driver esp_timer esp_hw_support driver esp_timer esp_hw_support app_update esp_rom esp_system spi_flash esp_adc nvs_flash)

# Short git hash of the source code, sent to the PC in the DETAILS answer.
# It is read when CMake runs: it is updated at every reconfigure / full build.
execute_process(COMMAND git rev-parse --short=7 HEAD
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                OUTPUT_VARIABLE GIT_HASH OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
execute_process(COMMAND git status --porcelain
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                OUTPUT_VARIABLE GIT_STATUS OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if("${GIT_HASH}" STREQUAL "")
    set(GIT_HASH "0000000")       # git not available
endif()
if("${GIT_STATUS}" STREQUAL "")
    set(GIT_DIRTY 0)
else()
    set(GIT_DIRTY 1)              # there are changes not committed yet
endif()
idf_build_set_property(COMPILE_OPTIONS "-DGIT_HASH_HEX=0x${GIT_HASH}ul" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DGIT_DIRTY=${GIT_DIRTY}" APPEND)