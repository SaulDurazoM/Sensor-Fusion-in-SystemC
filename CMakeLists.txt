cmake_minimum_required(VERSION 3.16)
project(sensor_fusion_systemc LANGUAGES CXX)

set(CMAKE_PREFIX_PATH "/usr/local/systemc" CACHE PATH "SystemC install prefix")

find_package(SystemCLanguage CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(sensor_fusion
    src/main.cpp
    src/top.cpp
)

target_include_directories(sensor_fusion PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(sensor_fusion PRIVATE
    SystemC::systemc
    Eigen3::Eigen
    pthread
)
