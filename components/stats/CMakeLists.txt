idf_component_register(
    SRCS        "stats.c"
    INCLUDE_DIRS "include"    # we’ll move stats.h into an include/ dir
    REQUIRES    freertos
)
