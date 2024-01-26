cmake_minimum_required(VERSION 3.27) # for debugging
project(smart_meter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTest REQUIRED)

set(MODULE_NAME smart_meter_test)

add_executable(${MODULE_NAME})

target_include_directories(${MODULE_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_sources(${MODULE_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/test/modbus_server_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/sunspec_meter_model_test.cpp
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
        GTest::gtest
        GTest::gtest_main
)
