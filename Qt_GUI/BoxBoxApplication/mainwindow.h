#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QByteArray>
#include <QHash>
#include <QMainWindow>
#include <QPair>
#include <QPixmap>
#include <QStandardItemModel>
#include <QStringList>
#include <QTcpSocket>
#include <QTcpServer>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <cstdint>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>      // ✅ 비상정지/해제 Bool
#include <std_msgs/msg/int32.hpp>

namespace rclcpp {
namespace executors {
class SingleThreadedExecutor;
}
}

class QPushButton;
class QAbstractSocket;
class QLabel;
class QStackedWidget;
class QWidget;
class QResizeEvent;
class QModelIndex;
class QImage;

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

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    // ✅ 비상정지 토글 버튼
    void onEmergencyToggleClicked();

    // TCP
    void onConnectButtonClicked();
    void onSendButtonClicked();
    void onSocketConnected(QTcpSocket *socket);
    void onSocketDisconnected(QTcpSocket *socket);
    void onSocketReadyRead(QTcpSocket *socket);
    void onSocketErrorOccurred(QTcpSocket *socket, QAbstractSocket::SocketError socket_error);
    void onServerNewConnection();
    void onServerAcceptError(QAbstractSocket::SocketError socket_error);

private:
    void appendLog(const QString &text);
    void updateConnectionState(bool connected);
    QByteArray buildMessagePayload(const QString &rawMessage, const QString &target) const;
    void addTableRow(QStandardItemModel &model, const QStringList &values, const QString &imagePath = QString());
    QString currentTimestamp() const;
    QPair<QString, QString> splitBracketMessage(const QString &text) const;
    void displayImage(const QByteArray &data);
    void sendEmergencyTcpMessage(const QString &message);
    void resetImageViewer();
    void resetImageReceptionState();
    bool initializeDatabase();
    void loadDatabaseSnapshot();
    void populateModelFromDatabase(QStandardItemModel &model, const QString &region);
    void populateAllModelsFromDatabase();
    qint64 insertRecordForRegion(const QString &region, const QStringList &fields, const QString &imagePath);
    bool tableHasWeightColumn(const QString &table_name);
    QString tableForRegion(const QString &region) const;
    QStandardItemModel *modelForRegion(const QString &region);
    QString determineRegionForSource(const QString &source) const;
    QString normalizeRegion(const QString &value) const;
    void handleStructuredMessage(const QString &source,
                                 const QString &body,
                                 int zone_hint = -1,
                                 const QString &zone_literal = QString());
    struct ClientSession;
    bool processDelimitedMessage(const QString &message, const QString &source_hint, const QString &region_hint, QTcpSocket *origin_socket);
    void handleQtDelimitedMessage(const QStringList &tokens, const QString &source_hint, const QString &region_hint, QTcpSocket *origin_socket);
    void handleRosDelimitedMessage(const QStringList &tokens, const QString &source_hint, const QString &region_hint, QTcpSocket *origin_socket);
    void handleStmIr2Message(const QStringList &tokens, QTcpSocket *origin_socket);
    void broadcastMessageToClients(const QString &message, QTcpSocket *exclude = nullptr);
    void broadcastDestinationZone(const QString &zone_text, int zone_numeric = -1);
    void publishDestinationTopic(int zone);
    void sendStmTurMessage();
    void updatePendingZoneLabel();
    void processIr2Activation(int zone_hint, const QString &source_label);
    void forwardMessageToClient(const QString &raw_message, QTcpSocket *exclude = nullptr);
    ClientSession *sessionForSocket(QTcpSocket *socket) const;
    void rememberTextForRegion(const QString &region,
                               const QStringList &fields,
                               int zone_hint = -1,
                               const QString &zone_literal = QString());
    void rememberImageForRegion(const QString &region, const QString &imagePath);
    void tryCommitPendingRecord(const QString &region);
    void updatePendingRecordWeight(const QString &region, const QString &weight);
    bool ensureImageDirectoryExists();
    QString saveImageToDisk(const QString &region, const QImage &image);
    void setupTableInteractions();
    void handleTableActivation(const QModelIndex &index);
    void openImageFromPath(const QString &imagePath);
    void applyModernStyle();

    // ✅ 비상정지 상태 UI 반영
    void setEmergencyUiState(bool active);

    int zoneNumberForRegion(const QString &region) const;

    struct PendingRecord {
        QStringList fields;
        QString image_path;
        bool has_text{false};
        bool has_image{false};
        int zone_hint{-1};
        QString zone_text;
    };

    Ui::MainWindow *ui;

    // ✅ 토글 버튼
    QPushButton *stop_button_{nullptr};
    bool emergency_active_{false};

    struct ClientSession {
        QTcpSocket *socket{nullptr};
        QByteArray incoming_buffer;
        QByteArray image_buffer;
        qint64 pending_image_bytes{0};
        QString pending_image_region;
        QString client_id;
    };

    QTcpServer *server_{nullptr};
    QHash<QTcpSocket *, ClientSession *> sessions_;
    QStackedWidget *stacked_widget_{nullptr};
    QWidget *connect_page_{nullptr};
    QWidget *main_page_{nullptr};
    QLabel *image_label_{nullptr};

    QStandardItemModel ros_table_model_;
    QStandardItemModel sent_table_model_;
    QStandardItemModel received_table_model_;
    QStandardItemModel unclassified_table_model_;

    // ✅ ROS2 노드/퍼브/섭
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr destination_pub_;

    // ✅ 토픽명
    std::string stop_topic_   = "/emergency_stop";
    std::string resume_topic_ = "/emergency_resume";
    std::string status_topic_ = "/emergency/state";
    std::string unload_destination_topic_ = "/unload/destination";

    QPixmap current_image_pixmap_;

    QSqlDatabase database_;
    QString database_connection_name_;
    bool database_ready_{false};

    QHash<QString, PendingRecord> pending_records_;
    QString pending_image_region_;
    QString pending_weight_value_;
    QString pending_weight_region_;
    int pending_zone_{-1};
    std::uint64_t pending_zone_token_{0};
    QString image_storage_dir_;
    bool image_dir_ready_{false};

    QString table_name_seoul_;
    QString table_name_busan_;
    QString table_name_daejeon_;
    QString table_name_unclassified_;
    QString column_destination_;
    QString column_id_;
    QString column_weight_;
    QString column_notes_;
    QString column_thumbnail_;
    QString column_region_;
    QString column_created_at_;
    bool use_region_column_{true};
    bool use_shared_table_{true};
    QString order_by_clause_;
    QHash<QString, bool> table_weight_column_cache_;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

#endif // MAINWINDOW_H
