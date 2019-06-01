set(GLOBALINCLUDES
    "./"
    PARENT_SCOPE
)

set(COMPONENT_SRCS "main.cpp")
set(COMPONENT_ADD_INCLUDEDIRS
    "."
)

register_component()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DESP32")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DESP32")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTOUCH_CS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTOUCH_CS")

option(M5STACK "Compile for M5Stack" ON) # Enabled by default

if(M5STACK)
    message(STATUS "Compiling for M5Stack")
    list(APPEND compile_definitions "M5Stack")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DM5Stack")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DM5Stack")
endif(M5STACK)

spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
