cmake_minimum_required(VERSION 3.9)
project(rtweekend CXX) # PROJECT_NAME is now "hello"
set(CMAKE_CXX_STANDARD 20)

# This will come in handy for LSP servers such as clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Decent compile options
add_compile_options(-Wall -Wextra -Werror -pedantic -gdwarf-4)

# Conan is important
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

# Make the "hello" executable target. Use globbing for src
file(GLOB_RECURSE src_cpp CONFIGURE_DEPENDS "src/*.cpp")
add_executable(${PROJECT_NAME} ${src_cpp})
