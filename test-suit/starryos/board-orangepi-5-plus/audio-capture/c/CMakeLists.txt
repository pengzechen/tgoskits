cmake_minimum_required(VERSION 3.20)
project(audio-capture C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(audio-capture src/main.c)
target_compile_options(audio-capture PRIVATE -Wall -Wextra -Werror)
target_link_options(audio-capture PRIVATE -static)

install(TARGETS audio-capture RUNTIME DESTINATION bin)
