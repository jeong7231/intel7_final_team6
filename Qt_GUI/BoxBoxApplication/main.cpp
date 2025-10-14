#include "mainwindow.h"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(6);
    rclcpp::init(argc, argv, init_options);

    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    const int result = app.exec();
    rclcpp::shutdown();
    return result;
}
