cmake_minimum_required(VERSION 3.19)
project(BoxBoxApplication LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Network Sql)

# When Qt Creator is launched without sourcing the ROS 2 setup script, CMAKE_PREFIX_PATH
# may not include the default ROS 2 install roots. Add common locations so rclcpp can be found.
set(_ros_common_prefixes
    /opt/ros/humble
)
foreach(_ros_prefix IN LISTS _ros_common_prefixes)
    if(EXISTS "${_ros_prefix}/share/rclcpp/cmake/rclcppConfig.cmake")
        list(APPEND CMAKE_PREFIX_PATH "${_ros_prefix}")
        file(GLOB _ros_python_site "${_ros_prefix}/lib/python*/site-packages")
        if(_ros_python_site)
            list(APPEND _ros_python_paths "${_ros_python_site}")
        endif()
        list(APPEND _ros_detected_prefixes "${_ros_prefix}")
    endif()
endforeach()
if(_ros_detected_prefixes)
    list(REMOVE_DUPLICATES _ros_detected_prefixes)
    if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
        list(APPEND _ros_detected_prefixes "$ENV{AMENT_PREFIX_PATH}")
    endif()
    string(JOIN ":" _ros_ament_joined ${_ros_detected_prefixes})
    set(ENV{AMENT_PREFIX_PATH} "${_ros_ament_joined}")

    if(DEFINED ENV{COLCON_PREFIX_PATH} AND NOT "$ENV{COLCON_PREFIX_PATH}" STREQUAL "")
        list(APPEND _ros_detected_prefixes "$ENV{COLCON_PREFIX_PATH}")
        list(REMOVE_DUPLICATES _ros_detected_prefixes)
        string(JOIN ":" _ros_colcon_joined ${_ros_detected_prefixes})
        set(ENV{COLCON_PREFIX_PATH} "${_ros_colcon_joined}")
    else()
        set(ENV{COLCON_PREFIX_PATH} "${_ros_ament_joined}")
    endif()
endif()
if(_ros_python_paths)
    list(REMOVE_DUPLICATES _ros_python_paths)
    if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
        list(APPEND _ros_python_paths "$ENV{PYTHONPATH}")
    endif()
    string(JOIN ":" _ros_python_joined ${_ros_python_paths})
    set(ENV{PYTHONPATH} "${_ros_python_joined}")
endif()
unset(_ros_common_prefixes)
unset(_ros_python_paths)
unset(_ros_detected_prefixes)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

qt_standard_project_setup()

qt_add_executable(BoxBoxApplication
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow_network.cpp
    mainwindow_image.cpp
    mainwindow_database.cpp
    mainwindow_utils.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(BoxBoxApplication
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::Network
        Qt::Sql
        rclcpp::rclcpp
        ${std_msgs_LIBRARIES}
)

if(std_msgs_INCLUDE_DIRS)
    target_include_directories(BoxBoxApplication PRIVATE ${std_msgs_INCLUDE_DIRS})
endif()

include(GNUInstallDirs)

install(TARGETS BoxBoxApplication
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET BoxBoxApplication
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
