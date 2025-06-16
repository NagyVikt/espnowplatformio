# main/CMakeLists.txt

idf_component_register(
    SRCS        "app_main.c"      # or "main.c" / "main.cpp" if you named it differently
    INCLUDE_DIRS "."              # look here for station.h and stats.h
    REQUIRES    station            # your station component
                stats              # your stats component
)
