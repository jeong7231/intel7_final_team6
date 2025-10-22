#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QAbstractItemView>
#include <QAbstractSocket>
#include <QAbstractItemModel>
#include <QDateTime>
#include <QImage>
#include <QDebug>
#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHeaderView>
#include <QList>
#include <QLabel>
#include <QLineEdit>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QResizeEvent>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QStandardPaths>
#include <QMetaType>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QUrl>
#include <QVariant>
#include <QStringList>
#include <QStandardItem>
#include <QTableView>
#include <QBoxLayout>
#include <QHBoxLayout>
#include <QSizePolicy>
#include <algorithm>
#include <exception>

#include <std_msgs/msg/bool.hpp>  // ✅
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "mainwindow_helpers.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    if (ui->centralLayout && ui->stackedWidget) {
        ui->centralLayout->removeWidget(ui->stackedWidget);
        auto *centerLayout = new QHBoxLayout;
        centerLayout->setContentsMargins(0, 0, 0, 0);
        centerLayout->addStretch(1);
        centerLayout->addWidget(ui->stackedWidget, 10);
        centerLayout->addStretch(1);
        centerLayout->setStretch(1, 10);
        ui->centralLayout->addLayout(centerLayout);
        ui->stackedWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    // ✅ 버튼 교체: publishButton → 비상정지 토글 버튼으로 사용
    stop_button_ = ui->publishButton;            // UI에서 기존 버튼 재활용

    server_ = new QTcpServer(this);
    stacked_widget_ = ui->stackedWidget;
    connect_page_ = ui->connectPage;
    main_page_ = ui->mainPage;
    image_label_ = ui->imageLabel;

    if (stop_button_) {
        stop_button_->setText(tr("비상정지"));
        stop_button_->setStyleSheet(QStringLiteral("QPushButton { background-color: #FF7043; color: #ffffff; }"));
    }
    if (image_label_) {
        image_label_->setAlignment(Qt::AlignCenter);
        image_label_->setText(tr("이미지가 없습니다."));
    }
    if (stacked_widget_ && connect_page_) {
        stacked_widget_->setCurrentWidget(connect_page_);
    }
    if (ui->mainPageLayout) {
        ui->mainPageLayout->setStretch(0, 0);
        ui->mainPageLayout->setStretch(1, 1);
    }
    if (ui->leftLayout) {
        ui->leftLayout->setStretch(0, 0);
        ui->leftLayout->setStretch(1, 2);
        ui->leftLayout->setStretch(2, 1);
    }
    if (ui->connectServerLineEdit && ui->serverLineEdit) {
        ui->connectServerLineEdit->setText(ui->serverLineEdit->text());
    }
    if (ui->connectPortLineEdit && ui->portLineEdit) {
        ui->connectPortLineEdit->setText(ui->portLineEdit->text());
    }
    if (ui->connectPageStatusLabel) {
        ui->connectPageStatusLabel->setText(tr("연결되지 않음"));
    }

    updatePendingZoneLabel();

    const QStringList table_headers = {
        tr("번호"),
        tr("수령지"),
        tr("특이사항"),
        tr("사진"),
        tr("시간")
    };
    ros_table_model_.setHorizontalHeaderLabels(table_headers);
    sent_table_model_.setHorizontalHeaderLabels(table_headers);
    received_table_model_.setHorizontalHeaderLabels(table_headers);
    unclassified_table_model_.setHorizontalHeaderLabels(table_headers);

    if (ui->tab1TableView) {
        ui->tab1TableView->setModel(&ros_table_model_);
    }
    if (ui->tab2TableView) {
        ui->tab2TableView->setModel(&sent_table_model_);
    }
    if (ui->tab3TableView) {
        ui->tab3TableView->setModel(&received_table_model_);
    }
    if (ui->tab4TableView) {
        ui->tab4TableView->setModel(&unclassified_table_model_);
    }

    const auto configureView = [](QTableView *view) {
        if (!view) {
            return;
        }
        view->setSelectionBehavior(QAbstractItemView::SelectRows);
        view->setSelectionMode(QAbstractItemView::SingleSelection);
        view->setEditTriggers(QAbstractItemView::NoEditTriggers);
        view->horizontalHeader()->setStretchLastSection(true);
        view->verticalHeader()->setVisible(false);
    };

    configureView(ui->tab1TableView);
    configureView(ui->tab2TableView);
    configureView(ui->tab3TableView);
    configureView(ui->tab4TableView);
    setupTableInteractions();

    QString fallbackImageDir;
    const QString picturesRoot = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    if (!picturesRoot.isEmpty()) {
        fallbackImageDir = QDir(picturesRoot).filePath(QStringLiteral("BoxBoxImages"));
    } else {
        const QString appDataRoot = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        fallbackImageDir = appDataRoot.isEmpty()
                               ? QDir::current().filePath(QStringLiteral("BoxBoxImages"))
                               : QDir(appDataRoot).filePath(QStringLiteral("BoxBoxImages"));
    }
    image_storage_dir_ = envOrDefault("BOXBOX_IMAGE_DIR", fallbackImageDir);
    image_storage_dir_ = QDir(image_storage_dir_).absolutePath();
    image_dir_ready_ = ensureImageDirectoryExists();
    if (!image_dir_ready_) {
        appendLog(tr("이미지 저장 폴더를 준비할 수 없습니다: %1").arg(image_storage_dir_));
    }

    table_name_seoul_ = envOrDefault("BOXBOX_DB_TABLE_SEOUL", QStringLiteral("package"));
    table_name_busan_ = envOrDefault("BOXBOX_DB_TABLE_BUSAN", QStringLiteral("package"));
    table_name_daejeon_ = envOrDefault("BOXBOX_DB_TABLE_DAEJEON", QStringLiteral("package"));
    table_name_unclassified_ = envOrDefault("BOXBOX_DB_TABLE_UNCLASSIFIED", table_name_seoul_);
    column_destination_ = envOrDefault("BOXBOX_DB_COLUMN_DESTINATION", QStringLiteral("destination"));
    column_weight_ = envOrDefault("BOXBOX_DB_COLUMN_WEIGHT", QStringLiteral("weight"));
    column_notes_ = envOrDefault("BOXBOX_DB_COLUMN_NOTES", QStringLiteral("notes"));
    column_thumbnail_ = envOrDefault("BOXBOX_DB_COLUMN_THUMBNAIL", QStringLiteral("image_path"));
    column_region_ = envOrDefault("BOXBOX_DB_COLUMN_REGION", QStringLiteral("region"));
    column_id_ = envOrDefault("BOXBOX_DB_COLUMN_ID", QStringLiteral("id"));
    column_created_at_ = envOrDefault("BOXBOX_DB_COLUMN_CREATED_AT", QStringLiteral("Timestamp"));
    const QString default_order = column_created_at_.isEmpty()
                                      ? QString()
                                      : QStringLiteral("%1 DESC").arg(column_created_at_);
    order_by_clause_ = envOrDefault("BOXBOX_DB_ORDER_BY", default_order);
    const QString region_flag = envOrDefault("BOXBOX_DB_USE_REGION_COLUMN", QStringLiteral("0")).toLower();
    use_region_column_ = !(region_flag == QStringLiteral("0") || region_flag == QStringLiteral("false"));
    use_shared_table_ = (table_name_seoul_ == table_name_busan_) &&
                        (table_name_seoul_ == table_name_daejeon_) &&
                        (table_name_seoul_ == table_name_unclassified_);

    database_connection_name_ = QStringLiteral("boxbox_mysql_connection");
    database_ready_ = initializeDatabase();
    if (database_ready_) {
        loadDatabaseSnapshot();
        appendLog(tr("데이터베이스에 연결되었습니다."));
    } else {
        appendLog(tr("데이터베이스 연결에 실패했습니다. 환경 변수를 확인하세요."));
    }

    // ✅ ROS2 노드 & 퍼블리셔/서브스크라이버 생성
    node_ = rclcpp::Node::make_shared("boxbox_qt_emergency_panel");
    stop_pub_   = node_->create_publisher<std_msgs::msg::Bool>(stop_topic_,   rclcpp::QoS(10));
    resume_pub_ = node_->create_publisher<std_msgs::msg::Bool>(resume_topic_, rclcpp::QoS(10));
    destination_pub_ = node_->create_publisher<std_msgs::msg::Int32>(unload_destination_topic_, rclcpp::QoS(10));
    state_sub_  = node_->create_subscription<std_msgs::msg::Bool>(
        status_topic_, rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool &msg) {
            const bool active = msg.data;
            // UI 스레드로 전달
            QMetaObject::invokeMethod(this, [this, active]() {
                setEmergencyUiState(active);
            }, Qt::QueuedConnection);
        }
        );

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    if (executor_) {
        executor_->add_node(node_);
        const auto logger = node_->get_logger();
        executor_thread_ = std::thread([executor = executor_, logger]() {
            try {
                executor->spin();
            } catch (const std::exception &ex) {
                RCLCPP_ERROR(logger, "ROS2 executor terminated: %s", ex.what());
            }
        });
    }

    // ✅ 버튼 시그널 연결
    connect(stop_button_, &QPushButton::clicked, this, &MainWindow::onEmergencyToggleClicked);

    // TCP 서버 시그널 연결
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    connect(ui->connectPageConnectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::onSendButtonClicked);

    if (server_) {
        connect(server_, &QTcpServer::newConnection, this, &MainWindow::onServerNewConnection);
        connect(server_, &QTcpServer::acceptError, this, &MainWindow::onServerAcceptError);
    }

    // 초기 상태
    setEmergencyUiState(false);     // 일단 해제 상태 가정
    updateConnectionState(false);
}

MainWindow::~MainWindow()
{
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        ClientSession *session = it.value();
        if (!socket) {
            delete session;
            continue;
        }
        socket->disconnect();
        socket->disconnectFromHost();
        socket->close();
        socket->deleteLater();
        delete session;
    }
    sessions_.clear();
    if (server_) {
        server_->close();
    }
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (executor_ && node_) {
        executor_->remove_node(node_);
    }
    executor_.reset();

    if (database_.isValid()) {
        if (database_.isOpen()) {
            database_.close();
        }
        database_ = QSqlDatabase();
        if (!database_connection_name_.isEmpty() && QSqlDatabase::contains(database_connection_name_)) {
            QSqlDatabase::removeDatabase(database_connection_name_);
        }
    }
    node_.reset();
    delete ui;
}

// ====================== ✅ 비상정지/재개 구현 ======================

void MainWindow::onEmergencyToggleClicked()
{
    std_msgs::msg::Bool msg;
    msg.data = true;   // 규약: true 한 번만 발행

    const bool activating = !emergency_active_;
    const QString stmPayload = QStringLiteral("stm|es|%1")
                                   .arg(activating ? QStringLiteral("t") : QStringLiteral("f"));

    if (activating) {
        if (!stop_pub_) {
            appendLog(tr("비상정지 퍼블리셔가 초기화되지 않았습니다."));
            return;
        }

        stop_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Emergency STOP requested -> publish true on %s",
                    stop_topic_.c_str());
        appendLog(tr("ROS2: 비상정지 요청 전송 (topic: %1)").arg(QString::fromStdString(stop_topic_)));
        sendEmergencyTcpMessage(QStringLiteral("STOP"));
        forwardMessageToClient(stmPayload);

        setEmergencyUiState(true);
    } else {
        if (!resume_pub_) {
            appendLog(tr("비상정지 해제 퍼블리셔가 초기화되지 않았습니다."));
            return;
        }

        resume_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Emergency RESUME requested -> publish true on %s",
                    resume_topic_.c_str());
        appendLog(tr("ROS2: 비상정지 해제 요청 전송 (topic: %1)").arg(QString::fromStdString(resume_topic_)));
        sendEmergencyTcpMessage(QStringLiteral("RESUME"));
        forwardMessageToClient(stmPayload);

        setEmergencyUiState(false);
    }
}

void MainWindow::setEmergencyUiState(bool active)
{
    // active == true  : 비상정지 상태(모터 OFF)
    // active == false : 정상 상태(모터 ON)
    const bool changed = (active != emergency_active_);
    emergency_active_ = active;

    if (stop_button_) {
        if (active) {
            stop_button_->setText(tr("정지 해제"));
            stop_button_->setStyleSheet(QStringLiteral("QPushButton { background-color: #66BB6A; color: #ffffff; }"));
        } else {
            stop_button_->setText(tr("비상정지"));
            stop_button_->setStyleSheet(QStringLiteral("QPushButton { background-color: #FF7043; color: #ffffff; }"));
        }
        stop_button_->setEnabled(true);
    }

    if (changed) {
        appendLog(active ? tr("상태: 비상정지 활성(모터 전원 OFF)")
                         : tr("상태: 비상정지 해제(모터 전원 ON)"));
    }
}

// ====================== TCP/이미지/DB 이하 기존 로직 ======================

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    if (image_label_ && !current_image_pixmap_.isNull()) {
        image_label_->setPixmap(current_image_pixmap_.scaled(
            image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::setupTableInteractions()
{
    const auto connectView = [this](QTableView *view) {
        if (!view) {
            return;
        }
        connect(view, &QTableView::doubleClicked, this, &MainWindow::handleTableActivation);
    };

    connectView(ui->tab1TableView);
    connectView(ui->tab2TableView);
    connectView(ui->tab3TableView);
    connectView(ui->tab4TableView);
}
