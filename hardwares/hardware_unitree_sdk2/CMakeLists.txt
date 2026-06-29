cmake_minimum_required(VERSION 3.8)
project(hardware_unitree_sdk2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(Threads REQUIRED)

foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach ()

include(GNUInstallDirs)

add_library(
        ${PROJECT_NAME}
        SHARED
        src/HardwareUnitree.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE src
)

target_link_libraries(${PROJECT_NAME} unitree_sdk2 Threads::Threads)

ament_target_dependencies(
        ${PROJECT_NAME}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface HardwareUnitree.xml)

# ---- UDP MCU mock executable for local verification ----
add_executable(udp_mcu_mock src/udp_mcu_mock.cpp)
target_include_directories(udp_mcu_mock PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        PRIVATE src
)
target_link_libraries(udp_mcu_mock unitree_sdk2 Threads::Threads)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME} 
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(
  TARGETS udp_mcu_mock
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# 更稳妥的 export（你原来用的 ${dependencies} 变量并不存在）
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} unitree_sdk2)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
