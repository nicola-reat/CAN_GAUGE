file(GLOB_RECURSE UI_SRCS
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

idf_component_register(
    SRCS ${UI_SRCS}
    INCLUDE_DIRS "."
    REQUIRES lvgl__lvgl
)