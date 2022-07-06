cmake_minimum_required(VERSION 3.5)
project(observers_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif ()


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
endif ()

ament_auto_package(INSTALL_TO_SHARE
config
launch
)
