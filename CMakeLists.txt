cmake_minimum_required(VERSION 3.11)

file(READ "VERSION" version)
project(franky VERSION ${version} LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

list(APPEND CMAKE_PREFIX_PATH "/usr/local")

include(GNUInstallDirs)

option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_PYTHON_STUBS "Build stubs" ON)
option(BUILD_TESTS "Build tests" ON)

find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)

if (${Franka_VERSION} VERSION_GREATER_EQUAL "0.8.0")
    add_compile_definitions(FRANKA_0_8)
endif ()

if (${Franka_VERSION} VERSION_GREATER_EQUAL "0.9.0")
    add_compile_definitions(FRANKA_0_9)
endif ()

message("Found Eigen Version: ${Eigen3_VERSION}")
message("Found Franka Version: ${Franka_VERSION}")

add_subdirectory(ruckig)

file(GLOB_RECURSE SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(franky ${SRC})
target_include_directories(franky PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(franky PUBLIC Franka::Franka Eigen3::Eigen ruckig)


if (BUILD_PYTHON_MODULE)
    # Force CMake to use Python from the current conda environment
    set(PYBIND11_FINDPYTHON ON)
    set(Python3_FIND_STRATEGY LOCATION)
    set(Python3_ROOT_DIR $ENV{CONDA_PREFIX})

    # Find Python first
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

    # Then find pybind11 (it will use the Python we just found)
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/pybind11)
        add_subdirectory(pybind11)
    else ()
        find_package(pybind11 REQUIRED)
    endif ()

    # Build Python module
    file(GLOB_RECURSE PYTHON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/python/*.cpp")
    pybind11_add_module(_franky-lib ${PYTHON_SRC})
    target_link_libraries(_franky-lib PUBLIC franky)
    set_target_properties(_franky-lib PROPERTIES OUTPUT_NAME "_franky")


    # Set include directories explicitly
    target_include_directories(_franky-lib PRIVATE
        ${Python3_INCLUDE_DIRS}
        ${pybind11_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    if (BUILD_PYTHON_STUBS)
        add_custom_target(
            franky-stubs
            COMMAND ${CMAKE_SOURCE_DIR}/generate_stubs.bash
                    ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    $<TARGET_FILE:_franky-lib>
            DEPENDS _franky-lib
        )
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/stubs/_franky-stubs/__init__.pyi DESTINATION lib OPTIONAL)
        add_custom_target(_franky
                DEPENDS _franky-lib
                DEPENDS franky-stubs)
    else ()
        add_custom_target(_franky
                DEPENDS _franky-lib)
    endif ()
endif ()


if (BUILD_EXAMPLES)
    foreach (example IN ITEMS linear)
        add_executable(${example} "examples/${example}.cpp")
        target_link_libraries(${example} PRIVATE franky)
    endforeach ()
endif ()


if (BUILD_TESTS)
    enable_testing()

    find_package(Catch2 REQUIRED)

    foreach (test IN ITEMS kinematics-test path-test)
        add_executable(${test} "test/${test}.cpp")
        target_link_libraries(${test} PRIVATE franky Catch2::Catch2)
        add_test(NAME ${test} COMMAND ${test})
    endforeach ()
endif ()


install(TARGETS franky
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Find gperftools - specify exact paths
find_library(GPERFTOOLS_PROFILER profiler PATHS /usr/lib/x86_64-linux-gnu REQUIRED)
if(NOT GPERFTOOLS_PROFILER)
    message(FATAL_ERROR "gperftools profiler not found. Please install google-perftools and libgoogle-perftools-dev")
endif()

# Add benchmarks
add_executable(ik-benchmark "benchmarks/inverse_kinematics_benchmark.cpp")
target_include_directories(ik-benchmark PRIVATE ${GPERFTOOLS_INCLUDE_DIR})
target_compile_options(ik-benchmark PRIVATE -g)  # Add debug symbols
target_link_options(ik-benchmark PRIVATE -Wl,--no-as-needed)  # Force linking
target_link_libraries(ik-benchmark PRIVATE
    franky
    ${GPERFTOOLS_PROFILER}
)
