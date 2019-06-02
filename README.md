# M5 Environment Probe

An environment probe based on the [M5Stack](https://m5stac.com) platform.

## Requirements

There are many release branches of [ESP-IDF](https://github.com/espressif/esp-idf) available.
This project is based on the master branch which is the 4.0 stream at this time.

## Building

In order to build the project, you need the [ESP-IDF SDK](https://github.com/espressif/esp-idf).  
Installation is fairly straight forward, see [Setting up the Toolchain](https://docs.espressif.com/projects/esp-idf/en/latest/get-started-cmake/index.html#step-1-set-up-the-toolchain) in the Espressif Documentation.
```
git clone https://www.github.com/roamingthings/m5-env-probe.git
cd m5-env-probe
git submodule update --init --recursive
mkdir build && cd build
cmake ../
cmake --build . --target spiffs_spiffs_bin
cmake --build . --target flash
```

## References

Many thanks to Matt Sieren for the [Homepoint](https://github.com/sieren/Homepoint) which has been an inspiration
and source for some parts of this project.
