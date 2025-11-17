# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES spi_bus esp_driver_gpio esp_rom
)
