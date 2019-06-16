file(GLOB TFTESPI_SRC
        "libraries/TFT_eSPI/*.cpp"
        )

set(GLOBALINCLUDES
        "./"
        PARENT_SCOPE
        )

set(LIBRARY_SRC
        ${TFTESPI_SRC}
        )

set(COMPONENT_SRCS
        "main.cpp"
        "meter.cpp"
        ${LIBRARY_SRC}
        )

set_property(SOURCE ${LIBRARY_SRC} PROPERTY COMPILE_FLAGS -Wno-all)

set(COMPONENT_ADD_INCLUDEDIRS
        "."
        "libraries/TFT_eSPI/"
        )

register_component()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DESP32")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DESP32")

option(M5STACK "Compile for M5Stack" ON) # Enabled by default

if (M5STACK)
    message(STATUS "Compiling for M5Stack")
    list(APPEND compile_definitions "M5Stack")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DM5Stack")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DM5Stack")
endif (M5STACK)

spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
