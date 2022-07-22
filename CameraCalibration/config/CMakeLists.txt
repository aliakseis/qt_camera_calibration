# Target is INTERFACE library
add_library(config INTERFACE)
target_include_directories(config INTERFACE ${CMAKE_CURRENT_BINARY_DIR})

configure_file(config.h.in config.h)

# Target only for display header/source files in IDE
add_custom_target(configSrc SOURCES config.h.in)
