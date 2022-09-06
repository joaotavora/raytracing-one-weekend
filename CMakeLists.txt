cmake_minimum_required(VERSION 3.9)
project(rtweekend CXX)
set(CMAKE_CXX_STANDARD 20)

# This will come in handy for LSP servers such as clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

# Decent compile options
add_compile_options(-Wall -Wextra -Werror -pedantic)

# Conan is important
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(main_exec ${PROJECT_NAME})
file(GLOB_RECURSE src_cpp CONFIGURE_DEPENDS "src/*.cpp")
add_executable(${main_exec} ${src_cpp})
target_link_libraries(${PROJECT_NAME} PRIVATE ${CONAN_LIBS})
