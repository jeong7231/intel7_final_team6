#include "mainwindow.h"

#include <QApplication>
#include <QPalette>
#include <QColor>
#include <QStyleFactory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::InitOptions init_options;
    init_options.set_domain_id(6);
    rclcpp::init(argc, argv, init_options);

    QApplication app(argc, argv);
    QApplication::setStyle(QStyleFactory::create(QStringLiteral("Fusion")));

    QPalette palette = app.palette();
    palette.setColor(QPalette::Window, QColor(245, 247, 250));
    palette.setColor(QPalette::WindowText, QColor(31, 45, 61));
    palette.setColor(QPalette::Base, QColor(255, 255, 255));
    palette.setColor(QPalette::AlternateBase, QColor(247, 249, 252));
    palette.setColor(QPalette::ToolTipBase, QColor(255, 255, 255));
    palette.setColor(QPalette::ToolTipText, QColor(31, 45, 61));
    palette.setColor(QPalette::Text, QColor(31, 45, 61));
    palette.setColor(QPalette::Button, QColor(255, 255, 255));
    palette.setColor(QPalette::ButtonText, QColor(31, 45, 61));
    palette.setColor(QPalette::BrightText, QColor(255, 255, 255));
    palette.setColor(QPalette::Highlight, QColor(45, 140, 240));
    palette.setColor(QPalette::HighlightedText, QColor(255, 255, 255));
    app.setPalette(palette);
    MainWindow window;
    window.show();

    const int result = app.exec();
    rclcpp::shutdown();
    return result;
}
