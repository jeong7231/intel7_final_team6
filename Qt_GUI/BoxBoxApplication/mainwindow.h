#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPair>
#include <QStandardItemModel>
#include <QStringList>
#include <QTcpSocket>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class QPushButton;
class QAbstractSocket;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onPublishButtonClicked();
    void onConnectButtonClicked();
    void onSendButtonClicked();
    void onSocketConnected();
    void onSocketDisconnected();
    void onSocketReadyRead();
    void onSocketErrorOccurred(QAbstractSocket::SocketError socket_error);

private:
    void appendLog(const QString &text);
    void updateConnectionState(bool connected);
    QString buildLoginPayload() const;
    QByteArray buildMessagePayload(const QString &rawMessage, const QString &target) const;
    void addTableRow(QStandardItemModel &model, const QStringList &values);
    QString currentTimestamp() const;
    QPair<QString, QString> splitBracketMessage(const QString &text) const;

    Ui::MainWindow *ui;
    QPushButton *publish_button_{nullptr};
    QTcpSocket *socket_{nullptr};
    QStandardItemModel ros_table_model_;
    QStandardItemModel sent_table_model_;
    QStandardItemModel received_table_model_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    bool servo_open_{false};
};
#endif // MAINWINDOW_H
