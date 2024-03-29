#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the 
# src/ directory, compile them and link them into lib(subdirectory_name).a 
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
COMPONENT_ADD_INCLUDEDIRS+=BME680_driver
COMPONENT_ADD_INCLUDEDIRS+=include
COMPONENT_SRCS+=BME680_driver

## Uncomment the following line if we have an implementation of libcurl available to us.
##CXXFLAGS+=-DESP_HAVE_CURL

#COMPONENT_ADD_LDFLAGS=-lstdc++ -l$(COMPONENT_NAME)

## Uncomment the following line to enable exception handling
#CXXFLAGS+=-fexceptions
#CXXFLAGS+= -std=c++11
