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
#include <QFont>
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
#include <QTabWidget>
#include <QTabBar>
#include <QStyle>
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
    if (stop_button_) {
        stop_button_->setText(tr("비상정지"));
        stop_button_->setProperty("accentButton", true);
    }

    applyModernStyle();

    server_ = new QTcpServer(this);
    stacked_widget_ = ui->stackedWidget;
    connect_page_ = ui->connectPage;
    main_page_ = ui->mainPage;
    image_label_ = ui->imageLabel;
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

void MainWindow::applyModernStyle()
{
    if (!ui) {
        return;
    }

    setAttribute(Qt::WA_StyledBackground, true);
    if (QWidget *center = centralWidget()) {
        center->setAttribute(Qt::WA_StyledBackground, true);
    }

    QFont baseFont = font();
    const int desiredPointSize = baseFont.pointSize() > 0 ? baseFont.pointSize() : 10;
    baseFont.setPointSize(std::max(desiredPointSize, 11));
    setFont(baseFont);

    if (ui->centralLayout) {
        ui->centralLayout->setContentsMargins(32, 24, 32, 24);
        ui->centralLayout->setSpacing(24);
    }
    if (ui->connectPageLayout) {
        ui->connectPageLayout->setContentsMargins(0, 32, 0, 32);
        ui->connectPageLayout->setSpacing(24);
    }
    if (ui->connectPageGroupBoxLayout) {
        ui->connectPageGroupBoxLayout->setSpacing(18);
    }
    if (ui->mainPageLayout) {
        ui->mainPageLayout->setSpacing(24);
    }
    if (ui->leftLayout) {
        ui->leftLayout->setSpacing(16);
    }
    if (ui->rightPanelLayout) {
        ui->rightPanelLayout->setSpacing(16);
    }

    const auto markStatusLabel = [](QLabel *label) {
        if (!label) {
            return;
        }
        label->setProperty("stateLabel", true);
    };
    markStatusLabel(ui->statusLabel);
    markStatusLabel(ui->connectPageStatusLabel);
    if (ui->zoneCountLabel) {
        ui->zoneCountLabel->setWordWrap(true);
        ui->zoneCountLabel->setProperty("stateLabel", true);
    }
    if (ui->connectPageSubtitleLabel) {
        ui->connectPageSubtitleLabel->setProperty("subtitleLabel", true);
    }

    if (stop_button_) {
        stop_button_->setProperty("accentButton", true);
        stop_button_->setProperty("stopState", QStringLiteral("stop"));
        stop_button_->setMinimumHeight(42);
        stop_button_->setCursor(Qt::PointingHandCursor);
    }

    const QString stylesheet = QStringLiteral(
        "QMainWindow { background-color: #f5f7fa; }\n"
        "QWidget#centralwidget { background-color: transparent; }\n"
        "QGroupBox { background-color: #ffffff; border: 1px solid #dcdfe6; border-radius: 12px; margin-top: 18px; }\n"
        "QGroupBox::title { subcontrol-origin: margin; left: 18px; padding: 0 6px; font-weight: 600; color: #1f2d3d; }\n"
        "QPushButton { background-color: #2d8cf0; border: none; border-radius: 8px; color: #ffffff; padding: 9px 18px; font-weight: 600; }\n"
        "QPushButton:hover { background-color: #1a73e8; }\n"
        "QPushButton:pressed { background-color: #1765c0; }\n"
        "QPushButton:disabled { background-color: #d3dce6; color: #8a97a6; }\n"
        "QPushButton[accentButton=\"true\"] { background-color: #ff5b5b; }\n"
        "QPushButton[accentButton=\"true\"]:hover { background-color: #ff4343; }\n"
        "QPushButton[accentButton=\"true\"]:pressed { background-color: #e62e2e; }\n"
        "QPushButton[accentButton=\"true\"][stopState=\"resume\"] { background-color: #34c759; }\n"
        "QPushButton[accentButton=\"true\"][stopState=\"resume\"]:hover { background-color: #2dab4f; }\n"
        "QPushButton[accentButton=\"true\"][stopState=\"resume\"]:pressed { background-color: #20863c; }\n"
        "QLineEdit, QPlainTextEdit { border: 1px solid #dcdfe6; border-radius: 8px; padding: 8px 10px; background-color: #ffffff; }\n"
        "QLineEdit:focus, QPlainTextEdit:focus { border-color: #2d8cf0; }\n"
        "QPlainTextEdit { min-height: 180px; }\n"
        "QTabWidget::pane { border: 1px solid #dcdfe6; border-radius: 12px; background: #ffffff; margin-top: 12px; }\n"
        "QTabBar::tab { background: transparent; border: none; padding: 8px 20px; margin: 0 4px; border-radius: 20px; color: #5c677d; font-weight: 600; }\n"
        "QTabBar::tab:selected { background: #2d8cf0; color: #ffffff; }\n"
        "QTableView { background: #ffffff; border: none; gridline-color: #e0e6ed; selection-background-color: rgba(45, 140, 240, 80); selection-color: #1f2d3d; }\n"
        "QHeaderView::section { background: #f0f2f5; border: none; padding: 6px 8px; font-weight: 600; color: #475b77; }\n"
        "QLabel[stateLabel=\"true\"] { padding: 4px 14px; border-radius: 14px; background-color: #e6f1ff; color: #1a73e8; font-weight: 600; }\n"
        "QLabel[subtitleLabel=\"true\"] { color: #5c677d; }\n"
        "QStatusBar { background: #ffffff; border-top: 1px solid #dcdfe6; }\n"
        "QScrollBar:vertical { background: transparent; width: 12px; margin: 4px 0; }\n"
        "QScrollBar::handle:vertical { background: #c0c6d1; border-radius: 6px; min-height: 30px; }\n"
        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }\n"
        "QScrollBar:horizontal { background: transparent; height: 12px; margin: 0 4px; }\n"
        "QScrollBar::handle:horizontal { background: #c0c6d1; border-radius: 6px; min-width: 30px; }\n"
        "QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }\n"
        "QLabel#imageLabel { background: #fafbfc; border: 2px dashed #dcdfe6; border-radius: 12px; color: #8a97a6; }\n"
    );
    setStyleSheet(stylesheet);

    if (ui->connectPageTitleLabel) {
        QFont titleFont = baseFont;
        titleFont.setPointSize(baseFont.pointSize() + 8);
        titleFont.setBold(true);
        ui->connectPageTitleLabel->setFont(titleFont);
        ui->connectPageTitleLabel->setAlignment(Qt::AlignCenter);
        ui->connectPageTitleLabel->setText(tr("BoxBox Control Center"));
    }
    if (ui->connectPageSubtitleLabel) {
        QFont subtitleFont = baseFont;
        subtitleFont.setPointSize(baseFont.pointSize() + 1);
        ui->connectPageSubtitleLabel->setFont(subtitleFont);
    }

    if (ui->dataTabWidget) {
        ui->dataTabWidget->setDocumentMode(true);
        ui->dataTabWidget->setElideMode(Qt::ElideRight);
        if (QTabBar *bar = ui->dataTabWidget->tabBar()) {
            bar->setExpanding(false);
        }
    }

    if (ui->logTextEdit) {
        ui->logTextEdit->setPlaceholderText(tr("시스템 로그가 이곳에 표시됩니다."));
    }

    if (ui->messageLineEdit) {
        ui->messageLineEdit->setPlaceholderText(tr("보낼 메시지를 입력하세요"));
    }

    if (ui->imageLabel) {
        ui->imageLabel->setMinimumSize(380, 260);
        ui->imageLabel->setAlignment(Qt::AlignCenter);
        ui->imageLabel->setWordWrap(true);
    }
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
        stop_button_->setText(active ? tr("정지 해제") : tr("비상정지"));
        stop_button_->setProperty("stopState", active ? QStringLiteral("resume") : QStringLiteral("stop"));
        stop_button_->setEnabled(true);
        stop_button_->style()->unpolish(stop_button_);
        stop_button_->style()->polish(stop_button_);
        stop_button_->update();
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
