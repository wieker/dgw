cmake_minimum_required(VERSION 3.16)
project(SAMD11_COM C ASM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE 1)

set(OPENOCD_BINARY "/home/wieker/Projects/linux/hackrf/openocd-bin/bin/openocd")
set(CMAKE_C_STANDARD 99)

add_executable(samd11_cli
        main.c
        test.c
        version.c
        i2c.c
        spi.c
        adc.c
        dac.c
        pwm.c
        gpio.c
        dgw_lin.c)

TARGET_LINK_LIBRARIES(samd11_cli udev)
