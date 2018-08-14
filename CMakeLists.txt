cmake_minimum_required(VERSION 2.8)

project (libs826_xenomai)

set(XENOMAI_DIR "/usr/xenomai")

execute_process(COMMAND ${XENOMAI_DIR}/bin/xeno-config --skin posix --cflags OUTPUT_VARIABLE xenomai_cflags OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${XENOMAI_DIR}/bin/xeno-config --skin posix --ldflags --no-auto-init OUTPUT_VARIABLE xenomai_ldflags OUTPUT_STRIP_TRAILING_WHITESPACE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${xenomai_cflags}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${xenomai_ldflags}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(s826_xeno SHARED src/826lib.c src/826drv_linux_xenomai.c src/platform_linux_xenomai.c)

install(TARGETS s826_xeno LIBRARY DESTINATION /usr/lib/)

add_custom_target(driver COMMAND make WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
add_custom_target(driver_install COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
add_custom_target(driver_uninstall COMMAND make uninstall WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
add_custom_target(driver_load COMMAND make load WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
add_custom_target(driver_unload COMMAND make unload WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
add_custom_target(driver_clean COMMAND make clean WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/driver)
