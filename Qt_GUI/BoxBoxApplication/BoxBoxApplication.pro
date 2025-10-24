QT       += core gui network sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG -= c++11
CONFIG += c++17

ROS_ROOT = /opt/ros/humble

INCLUDEPATH += \
    $$ROS_ROOT/include \
    $$ROS_ROOT/include/rclcpp \
    $$ROS_ROOT/include/rcl \
    $$ROS_ROOT/include/rcl_yaml_param_parser \
    $$ROS_ROOT/include/rcutils \
    $$ROS_ROOT/include/rmw \
    $$ROS_ROOT/include/rcl_interfaces \
    $$ROS_ROOT/include/rosidl_runtime_cpp \
    $$ROS_ROOT/include/rosidl_runtime_c \
    $$ROS_ROOT/include/rosidl_typesupport_cpp \
    $$ROS_ROOT/include/rosidl_typesupport_c \
    $$ROS_ROOT/include/rosidl_typesupport_introspection_cpp \
    $$ROS_ROOT/include/rosidl_typesupport_interface \
    $$ROS_ROOT/include/std_msgs \
    $$ROS_ROOT/include/ament_index_cpp \
    $$ROS_ROOT/include/rcpputils \
    $$files($$ROS_ROOT/include/*, directories)

LIBS += -L$$ROS_ROOT/lib \
    -lrclcpp \
    -lrcutils \
    -lrcpputils \
    -lrosidl_typesupport_cpp \
    -lrosidl_typesupport_c \
    -lrosidl_typesupport_introspection_cpp \
    -lrmw_implementation \
    -lrmw \
    -lrcl \
    -lrcl_yaml_param_parser \
    -lrcl_logging_interface \
    -lrosgraph_msgs__rosidl_typesupport_cpp \
    -lstatistics_msgs__rosidl_typesupport_cpp \
    -lstd_msgs__rosidl_typesupport_cpp \
    -lrosidl_typesupport_fastrtps_cpp \
    -lrosidl_typesupport_introspection_c \
    -llibstatistics_collector \
    -ltracetools \
    -lament_index_cpp \
    -lrcl_interfaces__rosidl_typesupport_cpp \
    -lrcl_logging_spdlog \
    -lrcl_interfaces__rosidl_typesupport_c \
    -lrcl_interfaces__rosidl_generator_c \
    -lrosidl_runtime_c \
    -lbuiltin_interfaces__rosidl_generator_c

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    mainwindow_database.cpp \
    mainwindow_image.cpp \
    mainwindow_network.cpp \
    mainwindow_utils.cpp

HEADERS += \
    mainwindow.h \
    mainwindow_helpers.h

FORMS += \
    mainwindow.ui

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
