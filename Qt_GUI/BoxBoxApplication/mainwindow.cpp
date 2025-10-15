#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QAbstractItemView>
#include <QAbstractSocket>
#include <QDateTime>
#include <QHeaderView>
#include <QList>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>
#include <QStandardItem>
#include <QTableView>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    publish_button_ = ui->publishButton;
    socket_ = new QTcpSocket(this);

    ros_table_model_.setHorizontalHeaderLabels({tr("시간"), tr("내용")});
    sent_table_model_.setHorizontalHeaderLabels({tr("시간"), tr("대상"), tr("내용")});
    received_table_model_.setHorizontalHeaderLabels({tr("시간"), tr("출처"), tr("내용")});

    if (ui->tab1TableView) {
        ui->tab1TableView->setModel(&ros_table_model_);
    }
    if (ui->tab2TableView) {
        ui->tab2TableView->setModel(&sent_table_model_);
    }
    if (ui->tab3TableView) {
        ui->tab3TableView->setModel(&received_table_model_);
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

    node_ = rclcpp::Node::make_shared("boxbox_qt_publisher");
    publisher_ = node_->create_publisher<std_msgs::msg::Float32>(
        "turtlebot3_dumpbox/servo/pwm", rclcpp::QoS(10));

    connect(publish_button_, &QPushButton::clicked, this, &MainWindow::onPublishButtonClicked);
    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    connect(ui->sendButton, &QPushButton::clicked, this, &MainWindow::onSendButtonClicked);

    connect(socket_, &QTcpSocket::connected, this, &MainWindow::onSocketConnected);
    connect(socket_, &QTcpSocket::disconnected, this, &MainWindow::onSocketDisconnected);
    connect(socket_, &QTcpSocket::readyRead, this, &MainWindow::onSocketReadyRead);
    connect(socket_, &QTcpSocket::errorOccurred, this, &MainWindow::onSocketErrorOccurred);

    updateConnectionState(false);
}

MainWindow::~MainWindow()
{
    if (socket_) {
        socket_->disconnectFromHost();
        socket_->close();
    }
    delete ui;
}

void MainWindow::onPublishButtonClicked()
{
    if (!publisher_) {
        appendLog(tr("서보 퍼블리셔가 초기화되지 않았습니다."));
        return;
    }

    constexpr float kOpenPwm = 1000.0F;
    constexpr float kClosePwm = 1500.0F;

    servo_open_ = !servo_open_;
    const float pwm_value = servo_open_ ? kOpenPwm : kClosePwm;

    std_msgs::msg::Float32 message;
    message.data = pwm_value;
    publisher_->publish(message);

    const QString action = servo_open_ ? tr("OPEN") : tr("CLOSE");
    RCLCPP_INFO(node_->get_logger(), "Sent servo command %s (PWM %.1f)",
                servo_open_ ? "OPEN" : "CLOSE", pwm_value);
    appendLog(tr("ROS2: 서보 %1 명령 전송 (PWM %2)").arg(action).arg(pwm_value, 0, 'f', 1));
    addTableRow(ros_table_model_, {currentTimestamp(),
                                   tr("Servo %1 (PWM %2)").arg(action).arg(pwm_value, 0, 'f', 1)});
}

void MainWindow::onConnectButtonClicked()
{
    if (!socket_) {
        appendLog(tr("소켓이 준비되지 않았습니다."));
        return;
    }

    if (socket_->state() == QAbstractSocket::ConnectedState ||
        socket_->state() == QAbstractSocket::ConnectingState) {
        appendLog(tr("서버 연결 해제를 시도합니다."));
        socket_->disconnectFromHost();
        return;
    }

    const QString host = ui->serverLineEdit->text().trimmed();
    bool port_ok = false;
    const quint16 port = ui->portLineEdit->text().toUShort(&port_ok);
    const QString id = ui->idLineEdit->text().trimmed();
    const QString password = ui->passwordLineEdit->text();

    if (host.isEmpty()) {
        appendLog(tr("서버 주소를 입력하세요."));
        return;
    }
    if (!port_ok || port == 0) {
        appendLog(tr("올바른 포트 번호를 입력하세요."));
        return;
    }
    if (id.isEmpty()) {
        appendLog(tr("ID를 입력하세요."));
        return;
    }
    if (password.isEmpty()) {
        appendLog(tr("비밀번호를 입력하세요."));
        return;
    }

    appendLog(tr("서버 %1:%2 접속을 시도합니다...").arg(host).arg(port));
    socket_->connectToHost(host, port);

    ui->statusLabel->setText(tr("연결 중..."));
    ui->connectButton->setText(tr("연결 해제"));
    ui->connectButton->setEnabled(true);
    ui->sendButton->setEnabled(false);
    ui->serverLineEdit->setReadOnly(true);
    ui->portLineEdit->setReadOnly(true);
    ui->idLineEdit->setReadOnly(true);
    ui->passwordLineEdit->setReadOnly(true);
}

void MainWindow::onSendButtonClicked()
{
    if (!socket_ || socket_->state() != QAbstractSocket::ConnectedState) {
        appendLog(tr("서버에 연결되어 있지 않습니다."));
        return;
    }

    const QString raw_message = ui->messageLineEdit->text();
    const QString target = ui->targetLineEdit->text().trimmed();

    if (raw_message.isEmpty()) {
        appendLog(tr("보낼 메시지를 입력하세요."));
        return;
    }

    const QByteArray payload = buildMessagePayload(raw_message, target);

    if (socket_->write(payload) == -1) {
        appendLog(tr("메시지 전송에 실패했습니다: %1").arg(socket_->errorString()));
        return;
    }

    socket_->flush();

    const QString trimmedPayload = QString::fromUtf8(payload).trimmed();
    const auto parts = splitBracketMessage(trimmedPayload);
    const QString destinationUsed = parts.first.isEmpty() ? QStringLiteral("ALLMSG") : parts.first;
    const QString body = parts.second.isEmpty() ? trimmedPayload : parts.second;

    addTableRow(sent_table_model_, {currentTimestamp(), destinationUsed, body});
    appendLog(tr("송신: %1").arg(trimmedPayload));
    ui->messageLineEdit->clear();
}

void MainWindow::onSocketConnected()
{
    appendLog(tr("서버와 연결되었습니다. 로그인 정보를 전송합니다."));
    const QString login_payload = buildLoginPayload();
    if (login_payload.isEmpty()) {
        appendLog(tr("ID 또는 비밀번호가 비어 있어 로그인 메시지를 전송하지 못했습니다."));
        return;
    }
    socket_->write(login_payload.toUtf8());
    updateConnectionState(true);
}

void MainWindow::onSocketDisconnected()
{
    appendLog(tr("서버 연결이 종료되었습니다."));
    updateConnectionState(false);
}

void MainWindow::onSocketReadyRead()
{
    if (!socket_) {
        return;
    }

    const QByteArray data = socket_->readAll();
    if (data.isEmpty()) {
        return;
    }

    QString text = QString::fromUtf8(data);
    text.replace("\r\n", "\n");
    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty()) {
        const QString rawLine = QString::fromUtf8(data).trimmed();
        appendLog(QStringLiteral("수신: %1").arg(rawLine));
        addTableRow(received_table_model_, {currentTimestamp(), tr("서버"), rawLine});
        return;
    }

    for (const QString &line : lines) {
        const QString trimmedLine = line.trimmed();
        appendLog(tr("수신: %1").arg(trimmedLine));
        const auto parts = splitBracketMessage(trimmedLine);
        const QString source = parts.first.isEmpty() ? tr("서버") : parts.first;
        const QString body = parts.second.isEmpty() ? trimmedLine : parts.second;
        addTableRow(received_table_model_, {currentTimestamp(), source, body});
    }
}

void MainWindow::onSocketErrorOccurred(QAbstractSocket::SocketError /*socket_error*/)
{
    if (!socket_) {
        return;
    }
    appendLog(tr("소켓 오류: %1").arg(socket_->errorString()));
    updateConnectionState(false);
}

void MainWindow::appendLog(const QString &text)
{
    if (!ui || !ui->logTextEdit) {
        return;
    }
    ui->logTextEdit->appendPlainText(text);
}

void MainWindow::updateConnectionState(bool connected)
{
    if (!ui) {
        return;
    }

    ui->connectButton->setText(connected ? tr("연결 해제") : tr("서버 연결"));
    ui->sendButton->setEnabled(connected);
    ui->statusLabel->setText(connected ? tr("연결됨") : tr("연결되지 않음"));

    ui->serverLineEdit->setReadOnly(connected);
    ui->portLineEdit->setReadOnly(connected);
    ui->idLineEdit->setReadOnly(connected);
    ui->passwordLineEdit->setReadOnly(connected);
}

QString MainWindow::buildLoginPayload() const
{
    if (!ui) {
        return {};
    }
    const QString id = ui->idLineEdit->text().trimmed();
    const QString password = ui->passwordLineEdit->text();
    if (id.isEmpty() || password.isEmpty()) {
        return {};
    }
    return QStringLiteral("[%1:%2]").arg(id, password);
}

QByteArray MainWindow::buildMessagePayload(const QString &rawMessage, const QString &target) const
{
    QString message = rawMessage;
    QString destination = target.isEmpty() ? QStringLiteral("ALLMSG") : target;

    if (!message.startsWith('[')) {
        message = QStringLiteral("[%1]%2").arg(destination, message);
    }

    if (!message.endsWith('\n')) {
        message.append('\n');
    }

    return message.toUtf8();
}

void MainWindow::addTableRow(QStandardItemModel &model, const QStringList &values)
{
    QList<QStandardItem *> items;
    items.reserve(values.size());
    for (const QString &value : values) {
        auto *item = new QStandardItem(value);
        item->setEditable(false);
        items.append(item);
    }
    model.appendRow(items);
}

QString MainWindow::currentTimestamp() const
{
    return QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss"));
}

QPair<QString, QString> MainWindow::splitBracketMessage(const QString &text) const
{
    const QString trimmed = text.trimmed();
    if (!trimmed.startsWith('[')) {
        return {QString(), trimmed};
    }

    const int closingIndex = trimmed.indexOf(']');
    if (closingIndex <= 1) {
        return {QString(), trimmed};
    }

    const QString bracketContent = trimmed.mid(1, closingIndex - 1).trimmed();
    QString remainder = trimmed.mid(closingIndex + 1).trimmed();
    return {bracketContent, remainder};
}
