cmake_minimum_required(VERSION 3.16)
project(flightsim2 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra)
endif()

# MinGW: link the runtime statically so executables don't depend on which
# libstdc++/libgcc DLLs happen to be on PATH.
if(MINGW)
    add_link_options(-static)
endif()

# Core library: everything except main().
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(REMOVE_ITEM CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(flightsim_core STATIC ${CORE_SOURCES})
target_include_directories(flightsim_core PUBLIC src)

# Main executable: flightsim <scenario.json>
add_executable(flightsim src/main.cpp)
target_link_libraries(flightsim PRIVATE flightsim_core)

# Tests (assert-based, one executable per test file).
enable_testing()
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS tests/test_*.cpp)
foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE flightsim_core)
    target_include_directories(${test_name} PRIVATE tests)
    add_test(NAME ${test_name} COMMAND ${test_name}
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

# Golden regression gate: byte-exact CSV comparison of every golden scenario
# (tools/check_golden.py). Goldens are toolchain-baselined -- regenerate with
# --update <names> when a change is intentional.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME golden_gate
             COMMAND ${Python3_EXECUTABLE} tools/check_golden.py
                     --flightsim $<TARGET_FILE:flightsim>
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    set_tests_properties(golden_gate PROPERTIES TIMEOUT 300)
else()
    message(WARNING "Python3 not found -- golden_gate test not registered")
endif()
