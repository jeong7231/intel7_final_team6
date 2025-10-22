#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mainwindow_helpers.h"

#include <QAbstractSocket>
#include <QMetaObject>
#include <QTimer>
#include <algorithm>
#include <QHostAddress>
#include <QList>
#include <QRegularExpression>
#include <QLineEdit>
#include <QPushButton>

void MainWindow::onConnectButtonClicked()
{
    if (!server_) {
        appendLog(tr("TCP 서버가 초기화되지 않았습니다."));
        return;
    }

    const auto *sender_button = qobject_cast<QPushButton *>(sender());
    const bool from_connect_page = sender_button && ui->connectPageConnectButton
                                   && sender_button == ui->connectPageConnectButton;

    QLineEdit *server_edit = from_connect_page && ui->connectServerLineEdit
                                 ? ui->connectServerLineEdit
                                 : ui->serverLineEdit;
    QLineEdit *port_edit = from_connect_page && ui->connectPortLineEdit
                               ? ui->connectPortLineEdit
                               : ui->portLineEdit;

    if (!server_edit || !port_edit) {
        appendLog(tr("서버 설정 위젯을 찾을 수 없습니다."));
        return;
    }

    if (server_->isListening()) {
        appendLog(tr("TCP 서버를 중지합니다."));
        for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
            if (!it.key()) {
                delete it.value();
                continue;
            }
            QTcpSocket *socket = it.key();
            socket->disconnect();
            socket->disconnectFromHost();
            socket->deleteLater();
            delete it.value();
        }
        sessions_.clear();
        server_->close();
        if (stacked_widget_ && connect_page_) {
            stacked_widget_->setCurrentWidget(connect_page_);
        }
        updateConnectionState(false);
        return;
    }

    const QString host = server_edit->text().trimmed();
    bool port_ok = false;
    const quint16 port = port_edit->text().toUShort(&port_ok);

    if (host.isEmpty()) {
        appendLog(tr("호스트가 비어 있어 모든 인터페이스에서 대기합니다."));
    }
    if (!port_ok || port == 0) {
        appendLog(tr("올바른 포트 번호를 입력하세요."));
        return;
    }

    QHostAddress address = QHostAddress::AnyIPv4;
    if (!host.isEmpty()) {
        if (host.compare(QStringLiteral("localhost"), Qt::CaseInsensitive) == 0) {
            address = QHostAddress::LocalHost;
        } else if (host.compare(QStringLiteral("any"), Qt::CaseInsensitive) == 0 ||
                   host == QStringLiteral("*")) {
            address = QHostAddress::AnyIPv4;
        } else {
            QHostAddress parsed;
            if (!parsed.setAddress(host)) {
                appendLog(tr("유효한 IP 주소를 입력하세요. (예: 0.0.0.0 또는 192.168.x.x)"));
                return;
            }
            address = parsed;
        }
    }

    if (!server_->listen(address, port)) {
        appendLog(tr("TCP 서버 시작 실패: %1").arg(server_->errorString()));
        return;
    }

    const QString boundAddress = server_->serverAddress().isNull()
                                     ? address.toString()
                                     : server_->serverAddress().toString();
    appendLog(tr("TCP 서버 시작: %1:%2").arg(boundAddress).arg(server_->serverPort()));

    const QString port_text = QString::number(port);
    if (ui->serverLineEdit && ui->serverLineEdit != server_edit) {
        ui->serverLineEdit->setText(host);
    }
    if (ui->portLineEdit && ui->portLineEdit != port_edit) {
        ui->portLineEdit->setText(port_text);
    }
    if (ui->connectServerLineEdit && ui->connectServerLineEdit != server_edit) {
        ui->connectServerLineEdit->setText(host);
    }
    if (ui->connectPortLineEdit && ui->connectPortLineEdit != port_edit) {
        ui->connectPortLineEdit->setText(port_text);
    }

    if (stacked_widget_ && main_page_) {
        stacked_widget_->setCurrentWidget(main_page_);
    }

    updateConnectionState(false);
}


void MainWindow::onSendButtonClicked()
{
    if (sessions_.isEmpty()) {
        appendLog(tr("연결된 클라이언트가 없습니다."));
        return;
    }

    const QString raw_message = ui->messageLineEdit->text().trimmed();

    if (raw_message.isEmpty()) {
        appendLog(tr("보낼 메시지를 입력하세요."));
        return;
    }

    QByteArray payload = buildMessagePayload(raw_message, QString());
    qDebug() << "[DEBUG] Sending payload:" << payload;

    int success_count = 0;
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
            continue;
        }
        if (socket->write(payload) == -1) {
            appendLog(tr("메시지 전송 실패(%1): %2")
                          .arg(socket->peerAddress().toString(), socket->errorString()));
            continue;
        }
        socket->flush();
        ++success_count;
    }
    if (success_count == 0) {
        appendLog(tr("메시지를 전송할 수 있는 클라이언트가 없습니다."));
    } else {
        appendLog(tr("송신(%1개): %2")
                      .arg(success_count)
                      .arg(QString::fromUtf8(payload).trimmed()));
    }
    ui->messageLineEdit->clear();
}



void MainWindow::onSocketConnected(QTcpSocket *socket)
{
    if (!socket) {
        return;
    }

    ClientSession *session = sessionForSocket(socket);
    if (!session) {
        return;
    }

    session->client_id.clear();
    session->incoming_buffer.clear();
    session->image_buffer.clear();
    session->pending_image_bytes = 0;
    session->pending_image_region.clear();

    const QString endpoint = QStringLiteral("%1:%2")
                                 .arg(socket->peerAddress().toString())
                                 .arg(socket->peerPort());
    appendLog(tr("클라이언트 연결됨: %1").arg(endpoint));

    resetImageViewer();

    appendLog(tr("클라이언트 메시지 교환을 허용합니다."));

    updateConnectionState(!sessions_.isEmpty());
}



void MainWindow::onSocketDisconnected(QTcpSocket *socket)
{
    if (!socket) {
        return;
    }

    auto it = sessions_.find(socket);
    QString client_label;
    if (it != sessions_.end() && it.value()) {
        client_label = it.value()->client_id;
    }

    appendLog(client_label.isEmpty()
                  ? tr("클라이언트 연결이 종료되었습니다.")
                  : tr("클라이언트 연결이 종료되었습니다: %1").arg(client_label));

    if (it != sessions_.end()) {
        delete it.value();
        sessions_.erase(it);
    }

    socket->deleteLater();
    updateConnectionState(!sessions_.isEmpty());
}



void MainWindow::onServerNewConnection()
{
    if (!server_) {
        return;
    }

    while (server_->hasPendingConnections()) {
        QTcpSocket *client = server_->nextPendingConnection();
        if (!client) {
            continue;
        }

        const QHostAddress new_address = client->peerAddress();
        QList<QTcpSocket *> duplicates;
        for (auto it = sessions_.cbegin(); it != sessions_.cend(); ++it) {
            QTcpSocket *existing = it.key();
            if (!existing) {
                continue;
            }
            if (existing->peerAddress() == new_address) {
                duplicates.append(existing);
            }
        }
        for (QTcpSocket *existingSocket : duplicates) {
            const QString existing_endpoint = QStringLiteral("%1:%2")
                                                  .arg(existingSocket->peerAddress().toString())
                                                  .arg(existingSocket->peerPort());
            appendLog(tr("동일 IP 재연결 감지 → 기존 클라이언트를 종료합니다: %1").arg(existing_endpoint));
            onSocketDisconnected(existingSocket);
            if (existingSocket->state() != QAbstractSocket::UnconnectedState) {
                existingSocket->disconnectFromHost();
            }
        }

        client->setParent(this);

        connect(client, &QTcpSocket::disconnected, this, [this, client]() {
            onSocketDisconnected(client);
        });
        connect(client, &QTcpSocket::readyRead, this, [this, client]() {
            onSocketReadyRead(client);
        });
        connect(client, &QTcpSocket::errorOccurred, this, [this, client](QAbstractSocket::SocketError error) {
            onSocketErrorOccurred(client, error);
        });

        auto *session = new ClientSession();
        session->socket = client;
        sessions_.insert(client, session);

        onSocketConnected(client);
    }
}



void MainWindow::onServerAcceptError(QAbstractSocket::SocketError socket_error)
{
    Q_UNUSED(socket_error);
    if (!server_) {
        return;
    }
    appendLog(tr("TCP 서버 오류: %1").arg(server_->errorString()));
}


void MainWindow::onSocketReadyRead(QTcpSocket *socket)
{
    ClientSession *session = sessionForSocket(socket);
    if (!session || !session->socket) {
        return;
    }

    session->incoming_buffer.append(socket->readAll());

    const QString kTokImgBrace = QStringLiteral("{IMG}");
    const auto inferPayloadSource = [](const QString &payload) -> QString {
        const QStringList parts = payload.split(QLatin1Char('|'), Qt::KeepEmptyParts);
        for (const QString &part : parts) {
            const QString candidate = part.trimmed();
            if (!candidate.isEmpty()) {
                return candidate;
            }
        }
        return {};
    };

    while (true) {
        if (session->pending_image_bytes > 0) {
            if (session->incoming_buffer.isEmpty()) {
                break;
            }

            const qint64 take = std::min(session->pending_image_bytes,
                                         static_cast<qint64>(session->incoming_buffer.size()));
            session->image_buffer.append(session->incoming_buffer.left(take));
            session->incoming_buffer.remove(0, take);
            session->pending_image_bytes -= take;

            if (session->pending_image_bytes == 0) {
                pending_image_region_ = session->pending_image_region;
                displayImage(session->image_buffer);
                session->image_buffer.clear();
                session->pending_image_region.clear();
            }
            continue;
        }

        const int nl = session->incoming_buffer.indexOf('\n');
        if (nl == -1) {
            break;
        }

        QByteArray line = session->incoming_buffer.left(nl);
        session->incoming_buffer.remove(0, nl + 1);

        QString s = QString::fromUtf8(line).trimmed();
        if (s.isEmpty()) {
            continue;
        }

        if (session->client_id.isEmpty()) {
            session->client_id = s;
            appendLog(tr("클라이언트 ID 설정: %1").arg(session->client_id));
            continue;
        }

        const auto parts = splitBracketMessage(s);
        const QString bracket = parts.first;
        const QString remainder = parts.second;

        auto findImgHeader = [&](const QString &text, qint64 &sizeOut) -> bool {
            int pos = text.indexOf(kTokImgBrace);
            if (pos < 0) {
                return false;
            }

            const QString prefix = text.left(pos).trimmed();
            if (!prefix.isEmpty()) {
                appendLog(tr("수신: %1").arg(prefix));
                const auto prefix_parts = splitBracketMessage(prefix);
                const QString prefix_token = prefix_parts.first;
                const QString prefix_body = prefix_parts.second.isEmpty()
                                                ? prefix
                                                : prefix_parts.second;
                QString prefix_source = prefix_token;
                QString region_source = prefix_token;

                if (prefix_source.isEmpty()) {
                    const QString candidate = inferPayloadSource(prefix_body);
                    if (!candidate.isEmpty()) {
                        prefix_source = candidate;
                        region_source = candidate;
                    } else {
                        prefix_source = session->client_id.isEmpty()
                                             ? tr("클라이언트")
                                             : session->client_id;
                        region_source = session->client_id;
                    }
                }

                QString region;
                if (!region_source.isEmpty()) {
                    region = determineRegionForSource(region_source);
                    session->pending_image_region = region;
                    pending_image_region_ = region;
                }

                if (prefix_body.contains(QLatin1Char('|'))) {
                    if (!processDelimitedMessage(prefix_body, prefix_source, region, socket)) {
                        if (!region.isEmpty()) {
                            pending_image_region_ = region;
                        }
                        handleStructuredMessage(prefix_source, prefix_body);
                    }
                }
            }

            const QString after = text.mid(pos + kTokImgBrace.size()).trimmed();
            bool ok = false;
            const qint64 n = after.toLongLong(&ok);
            if (!ok || n <= 0) {
                return false;
            }
            sizeOut = n;
            return true;
        };

        qint64 declared = 0;
        if (findImgHeader(s, declared)) {
            session->pending_image_bytes = declared;
            session->image_buffer.clear();
            appendLog(tr("이미지 데이터 수신 중... (%1 bytes)").arg(declared));
            continue;
        }

        appendLog(tr("수신: %1").arg(s));
        const QString source_token = bracket;
        const QString body = remainder.isEmpty() ? s : remainder;
        const QString payload_source = inferPayloadSource(body);

        QString source = source_token;
        if (source.isEmpty()) {
            if (!payload_source.isEmpty()) {
                source = payload_source;
            } else {
                source = session->client_id.isEmpty() ? tr("클라이언트") : session->client_id;
            }
        }

        QString region_source = source_token;
        if (region_source.isEmpty()) {
            if (!payload_source.isEmpty()) {
                region_source = payload_source;
            } else {
                region_source = session->client_id;
            }
        }

        const QString region = region_source.isEmpty()
                                   ? QString()
                                   : determineRegionForSource(region_source);
        if (body.contains(QLatin1Char('|'))) {
            if (!region.isEmpty()) {
                session->pending_image_region = region;
                pending_image_region_ = region;
            }
            if (!processDelimitedMessage(body, source, region, socket)) {
                handleStructuredMessage(source, body);
            }
        } else if (session->pending_image_region.isEmpty()) {
            session->pending_image_region = region;
            pending_image_region_ = region;
        }
    }
}



void MainWindow::onSocketErrorOccurred(QTcpSocket *socket, QAbstractSocket::SocketError /*socket_error*/)
{
    if (!socket) {
        return;
    }
    appendLog(tr("클라이언트 소켓 오류: %1").arg(socket->errorString()));
    if (socket->state() != QAbstractSocket::UnconnectedState) {
        socket->disconnectFromHost();
    }
}



void MainWindow::sendEmergencyTcpMessage(const QString &message)
{
    if (sessions_.isEmpty()) {
        appendLog(tr("TCP: 비상정지 메시지를 전송할 수 없습니다. 연결된 클라이언트가 없습니다."));
        return;
    }
    QString formatted = message;
    if (!formatted.contains(QLatin1Char('|'))) {
        formatted = QStringLiteral("server|EMERGENCY|%1").arg(message);
    }
    const QByteArray payload = buildMessagePayload(formatted, QString());

    int success_count = 0;
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
            continue;
        }
        if (socket->write(payload) == -1) {
            appendLog(tr("TCP: 비상정지 메시지 전송 실패(%1): %2")
                          .arg(socket->peerAddress().toString(), socket->errorString()));
            continue;
        }
        socket->flush();
        ++success_count;
    }

    if (success_count == 0) {
        appendLog(tr("TCP: 비상정지 메시지를 전송할 수 있는 클라이언트가 없습니다."));
    } else {
        appendLog(tr("TCP: 비상정지 메시지 전송(%1) → %2")
                      .arg(success_count)
                      .arg(QString::fromUtf8(payload).trimmed()));
    }
}



bool MainWindow::processDelimitedMessage(const QString &message,
                                         const QString &source_hint,
                                         const QString &region_hint,
                                         QTcpSocket *origin_socket)
{
    QStringList raw_parts = message.split(QLatin1Char('|'), Qt::KeepEmptyParts);
    if (raw_parts.isEmpty()) {
        return false;
    }

    QStringList parts;
    parts.reserve(raw_parts.size());
    for (const QString &part : raw_parts) {
        parts << part.trimmed();
    }

    int device_index = -1;
    for (int i = 0; i < parts.size(); ++i) {
        if (!parts.at(i).isEmpty()) {
            device_index = i;
            break;
        }
    }
    if (device_index == -1) {
        return false;
    }

    const QString device = parts.at(device_index);
    QStringList payload_tokens = parts.mid(device_index + 1);

    QString effective_source = source_hint;
    if (effective_source.isEmpty()) {
        effective_source = device;
    }

    QString region = region_hint;
    if (region.isEmpty()) {
        region = determineRegionForSource(device);
    }

        if (device.compare(QStringLiteral("qt"), Qt::CaseInsensitive) == 0) {
            handleQtDelimitedMessage(payload_tokens, effective_source, region, origin_socket);
            return true;
        }

        if (device.compare(QStringLiteral("stm"), Qt::CaseInsensitive) == 0) {
            if (!payload_tokens.isEmpty()) {
                const QString command = payload_tokens.first().trimmed().toLower();
                if (command == QStringLiteral("ir2")) {
                    handleStmIr2Message(payload_tokens, origin_socket);
                }
            }
        }

    if (device.compare(QStringLiteral("ros"), Qt::CaseInsensitive) == 0) {
        handleRosDelimitedMessage(payload_tokens, effective_source, region, origin_socket);
        return true;
    }

    if (!region.isEmpty()) {
        pending_image_region_ = region;
    }

    appendLog(tr("장치 %1 → 외부 클라이언트 전달 시도").arg(device));
    forwardMessageToClient(message, origin_socket);

    if (!payload_tokens.isEmpty()) {
        const QString payload = payload_tokens.join(QStringLiteral("|"));
        handleStructuredMessage(effective_source, payload);
    } else {
        appendLog(tr("장치 %1 에서 비어 있는 명령이 도착했습니다.").arg(device));
    }
    return true;
}



void MainWindow::handleQtDelimitedMessage(const QStringList &tokens,
                                          const QString &source_hint,
                                          const QString &region_hint,
                                          QTcpSocket *origin_socket)
{
    if (!region_hint.isEmpty()) {
        pending_image_region_ = region_hint;
    } else {
        const QString inferred = determineRegionForSource(QStringLiteral("qt"));
        if (!inferred.isEmpty()) {
            pending_image_region_ = inferred;
        }
    }

    QStringList workingTokens;
    workingTokens.reserve(tokens.size());
    for (const QString &token : tokens) {
        workingTokens << token.trimmed();
    }
    while (!workingTokens.isEmpty() && workingTokens.first().isEmpty()) {
        workingTokens.removeFirst();
    }

    if (workingTokens.isEmpty()) {
        appendLog(tr("qt 장치에서 비어 있는 명령을 수신했습니다."));
        return;
    }

    int zoneHint = -1;
    QString zoneLiteral;
    const QString primary = workingTokens.first().toLower();
    if (primary == QStringLiteral("det")) {
        QString zoneTokenRaw = workingTokens.size() > 1 ? workingTokens.at(1).trimmed() : QString();
        QString zoneTokenStripped = zoneTokenRaw;
        if (!zoneTokenStripped.isEmpty()) {
            static const QRegularExpression quoteStrip(QStringLiteral("^[\"']+|[\"']+$"));
            zoneTokenStripped.remove(quoteStrip);
        }
        const QString numericZone = extractNumericToken(zoneTokenStripped);
        if (!numericZone.isEmpty()) {
            bool ok = false;
            const int candidate = numericZone.toInt(&ok);
            if (ok) {
                zoneHint = candidate;
            }
        }
        if (!numericZone.isEmpty()) {
            workingTokens[1] = numericZone;
            zoneLiteral = numericZone;
        } else if (!zoneTokenStripped.isEmpty()) {
            workingTokens[1] = zoneTokenStripped;
            zoneLiteral = zoneTokenStripped;
        } else if (!zoneTokenRaw.isEmpty()) {
            workingTokens[1] = zoneTokenRaw;
            zoneLiteral = zoneTokenRaw;
        }
    }
    if (primary == QStringLiteral("ir2")) {
        QString state_token = workingTokens.size() > 1 ? workingTokens.at(1).trimmed().toLower()
                                                       : QStringLiteral("t");
        if (state_token.isEmpty()) {
            state_token = QStringLiteral("t");
        }
        const bool activated = state_token == QStringLiteral("t")
                               || state_token == QStringLiteral("true")
                               || state_token == QStringLiteral("1")
                               || state_token == QStringLiteral("on");
        if (!activated) {
            appendLog(tr("qt IR2 신호 무시: 상태(%1)").arg(workingTokens.value(1)));
            return;
        }
        processIr2Activation(zoneHint, tr("QT"));
        return;
    }
    if (primary == QStringLiteral("es")) {
        QString state_token = workingTokens.size() > 1 ? workingTokens.at(1).trimmed().toLower() : QString();
        if (state_token.isEmpty()) {
            state_token = QStringLiteral("t");
        }

        const auto publishBool = [this](const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub,
                                        const QString &topic_name,
                                        bool value,
                                        const QString &log_text) {
            if (!pub) {
                appendLog(tr("ROS 퍼블리셔(%1)가 초기화되지 않았습니다.").arg(topic_name));
                return;
            }
            std_msgs::msg::Bool msg;
            msg.data = value;
            pub->publish(msg);
            if (node_) {
                const QByteArray topicUtf8 = topic_name.toUtf8();
                RCLCPP_INFO(node_->get_logger(),
                            "Emergency command from qt -> publish %s on %s",
                            value ? "true" : "false",
                            topicUtf8.constData());
            }
            appendLog(log_text);
        };

        const bool requestStop = state_token == QStringLiteral("t")
                                 || state_token == QStringLiteral("true")
                                 || state_token == QStringLiteral("1")
                                 || state_token == QStringLiteral("on");
        const bool requestResume = state_token == QStringLiteral("f")
                                   || state_token == QStringLiteral("false")
                                   || state_token == QStringLiteral("0")
                                   || state_token == QStringLiteral("off");

        bool handled = false;
        QString normalizedState = state_token;

        if (requestStop) {
            publishBool(stop_pub_,
                        QString::fromStdString(stop_topic_),
                        true,
                        tr("ROS2: 비상정지 요청 전송 (topic: %1)").arg(QString::fromStdString(stop_topic_)));
            setEmergencyUiState(true);
            sendEmergencyTcpMessage(QStringLiteral("STOP"));
            normalizedState = QStringLiteral("t");
            handled = true;
        } else if (requestResume) {
            publishBool(resume_pub_,
                        QString::fromStdString(resume_topic_),
                        true,
                        tr("ROS2: 비상정지 해제 요청 전송 (topic: %1)").arg(QString::fromStdString(resume_topic_)));
            setEmergencyUiState(false);
            sendEmergencyTcpMessage(QStringLiteral("RESUME"));
            normalizedState = QStringLiteral("f");
            handled = true;
        } else {
            appendLog(tr("qt 명령: 알 수 없는 비상정지 상태 토큰(%1)").arg(workingTokens.value(1)));
        }

        if (handled) {
            QStringList broadcastParts;
            broadcastParts << QString() << QStringLiteral("es") << normalizedState;
            if (workingTokens.size() > 2) {
                broadcastParts.append(workingTokens.mid(2));
            }
            const QString broadcastPayload = broadcastParts.join(QStringLiteral("|"));
            broadcastMessageToClients(broadcastPayload, origin_socket);
        }
        return;
    }

    if (primary == QStringLiteral("weight") ||
        primary == QStringLiteral("wgt") ||
        primary == QStringLiteral("wt")) {
        QString numericValue;
        for (int i = 1; i < workingTokens.size(); ++i) {
            const QString candidate = workingTokens.at(i);
            const QString numeric = extractNumericToken(candidate);
            if (!numeric.isEmpty()) {
                numericValue = numeric;
                break;
            }
        }

        if (numericValue.isEmpty()) {
            appendLog(tr("qt 무게 명령: 저장할 값이 없습니다."));
            return;
        }

        QString region = region_hint;
        if (region.isEmpty() && !pending_image_region_.isEmpty()) {
            region = pending_image_region_;
        }
        if (region.isEmpty()) {
            const QString sourceCandidate = source_hint.isEmpty() ? QStringLiteral("qt") : source_hint;
            region = determineRegionForSource(sourceCandidate);
        }

        pending_weight_value_ = numericValue;

        QString resolvedRegion = region;
        if (!resolvedRegion.isEmpty()) {
            updatePendingRecordWeight(resolvedRegion, numericValue);
        } else {
            for (auto it = pending_records_.begin(); it != pending_records_.end(); ++it) {
                if (it.value().has_text && !it.value().has_image) {
                    resolvedRegion = it.key();
                    updatePendingRecordWeight(resolvedRegion, numericValue);
                    break;
                }
            }
        }

        pending_weight_region_ = resolvedRegion;

        if (resolvedRegion.isEmpty()) {
            appendLog(tr("무게 저장: %1 kg (지역 정보 없음)").arg(numericValue));
        } else {
            appendLog(tr("무게 저장(%1): %2 kg").arg(resolvedRegion, numericValue));
        }
        return;
    }

    auto mapOcrCodeToRegion = [](const QString &code) -> QString {
        bool ok = false;
        const int value = code.toInt(&ok);
        if (!ok) {
            return {};
        }
        switch (value) {
        case 0:
            return QStringLiteral("미분류");
        case 1:
            return QStringLiteral("서울");
        case 2:
            return QStringLiteral("대전");
        case 3:
            return QStringLiteral("부산");
        default:
            return {};
        }
    };

    auto detectionLabelForCode = [](QChar code) -> QString {
        switch (code.toLower().unicode()) {
        case 'a':
            return QStringLiteral("fragile");
        case 'b':
            return QStringLiteral("care");
        case 'c':
            return QStringLiteral("water");
        case 'd':
            return QStringLiteral("upside");
        default:
            return {};
        }
    };

    QStringList effectiveTokens = workingTokens;

    if (!workingTokens.isEmpty() && workingTokens.first().compare(QStringLiteral("det"), Qt::CaseInsensitive) == 0) {
        QString combined = workingTokens.value(1).trimmed();
        QString ocrRaw;
        QString detRaw;
        QStringList extras;

        if (!combined.isEmpty()) {
            const QStringList pair = combined.split(QLatin1Char(','), Qt::KeepEmptyParts);
            if (!pair.isEmpty()) {
                ocrRaw = pair.at(0).trimmed();
            }
            if (pair.size() > 1) {
                detRaw = pair.at(1).trimmed();
            }
        }

        if (detRaw.isEmpty() && workingTokens.size() > 2) {
            detRaw = workingTokens.at(2).trimmed();
            extras = workingTokens.mid(3);
        } else {
            extras = workingTokens.mid(2);
        }

        if (!ocrRaw.isEmpty()) {
            static const QRegularExpression quoteStrip(QStringLiteral("^[\"']+|[\"']+$"));
            ocrRaw.remove(quoteStrip);
            bool ok = false;
            const int candidate = ocrRaw.toInt(&ok);
            if (ok && candidate >= 0) {
                zoneHint = candidate;
            }
        }

        const QString numericZone = extractNumericToken(ocrRaw);
        const QString mappedRegion = mapOcrCodeToRegion(numericZone.isEmpty() ? ocrRaw : numericZone);
        const QString regionToken = mappedRegion.isEmpty()
                                        ? (numericZone.isEmpty() ? (ocrRaw.isEmpty() ? QStringLiteral("미분류")
                                                                                     : ocrRaw)
                                                                 : numericZone)
                                        : mappedRegion;

        QStringList detectionLabels;
        for (QChar ch : detRaw) {
            if (ch.isSpace() || ch == QLatin1Char(',')) {
                continue;
            }
            const QString label = detectionLabelForCode(ch);
            if (!label.isEmpty()) {
                detectionLabels << label;
            }
        }

        if (detectionLabels.isEmpty() && !detRaw.isEmpty()) {
            const QStringList fallbackParts = detRaw.split(QRegularExpression(QStringLiteral("\\s+")),
                                                           Qt::SkipEmptyParts);
            detectionLabels << fallbackParts;
        }

        QStringList rebuilt;
        rebuilt << QStringLiteral("OCR")
                << regionToken
                << QStringLiteral("DET");
        if (!detectionLabels.isEmpty()) {
            rebuilt.append(detectionLabels);
        }
        if (!extras.isEmpty()) {
            rebuilt.append(extras);
        }

        effectiveTokens = rebuilt;
    }

    if (zoneLiteral.isEmpty() && zoneHint >= 0) {
        zoneLiteral = QString::number(zoneHint);
    }

    const QString payload = effectiveTokens.join(QStringLiteral("|"));
    const QString effective_source = source_hint.isEmpty() ? QStringLiteral("qt") : source_hint;
    handleStructuredMessage(effective_source, payload, zoneHint, zoneLiteral);
}



void MainWindow::handleRosDelimitedMessage(const QStringList &tokens,
                                           const QString &source_hint,
                                           const QString &region_hint,
                                           QTcpSocket *origin_socket)
{
    Q_UNUSED(source_hint);
    Q_UNUSED(origin_socket);

    if (!region_hint.isEmpty()) {
        pending_image_region_ = region_hint;
    }

    if (tokens.isEmpty()) {
        appendLog(tr("ROS 명령이 비어 있습니다."));
        return;
    }

    const auto publishBool = [this](const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub,
                                    const QString &topic_name,
                                    bool value,
                                    const QString &log_text) {
        if (!pub) {
            appendLog(tr("ROS 퍼블리셔(%1)가 초기화되지 않았습니다.").arg(topic_name));
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = value;
        pub->publish(msg);
        appendLog(log_text);
    };

    const QString primary = tokens.first().trimmed().toLower();

    if (primary == QStringLiteral("stop") || primary == QStringLiteral("emergency_stop")) {
        publishBool(stop_pub_, QString::fromStdString(stop_topic_), true,
                    tr("ROS 명령 수신 → 비상정지 요청"));
        setEmergencyUiState(true);
        return;
    }

    if (primary == QStringLiteral("resume") || primary == QStringLiteral("emergency_resume")) {
        publishBool(resume_pub_, QString::fromStdString(resume_topic_), true,
                    tr("ROS 명령 수신 → 비상정지 해제 요청"));
        setEmergencyUiState(false);
        return;
    }

    if (primary == QStringLiteral("state") && tokens.size() > 1) {
        const QString state_token = tokens.at(1).trimmed().toLower();
        const bool active = (state_token == QStringLiteral("on") ||
                             state_token == QStringLiteral("1") ||
                             state_token == QStringLiteral("true"));
        setEmergencyUiState(active);
        appendLog(tr("ROS 상태 업데이트 → %1").arg(active ? tr("정지") : tr("정상")));
        return;
    }

    appendLog(tr("알 수 없는 ROS 명령: %1").arg(tokens.join(QStringLiteral("|"))));
}



void MainWindow::broadcastMessageToClients(const QString &message, QTcpSocket *exclude)
{
    if (sessions_.isEmpty()) {
        appendLog(tr("브로드캐스트 실패: 연결된 클라이언트가 없습니다."));
        return;
    }

    QString payload = message;
    if (!payload.endsWith(QLatin1Char('\n'))) {
        payload.append(QLatin1Char('\n'));
    }
    const QByteArray data = payload.toUtf8();

    int success_count = 0;
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        if (!socket || socket == exclude) {
            continue;
        }
        if (socket->state() != QAbstractSocket::ConnectedState) {
            continue;
        }
        if (socket->write(data) == -1) {
            appendLog(tr("브로드캐스트 실패(%1): %2")
                          .arg(socket->peerAddress().toString(), socket->errorString()));
            continue;
        }
        socket->flush();
        ++success_count;
    }

    if (success_count == 0) {
        appendLog(tr("브로드캐스트 대상이 없습니다. → %1")
                      .arg(QString::fromUtf8(data).trimmed()));
    } else {
        appendLog(tr("브로드캐스트 완료(%1) → %2")
                      .arg(success_count)
                      .arg(QString::fromUtf8(data).trimmed()));
    }
}


void MainWindow::updatePendingZoneLabel()
{
    if (!ui || !ui->zoneCountLabel) {
        return;
    }

    if (pending_zone_ > 0) {
        ui->zoneCountLabel->setText(tr("IR2 대기 존: %1").arg(pending_zone_));
    } else {
        ui->zoneCountLabel->setText(tr("IR2 대기 없음"));
    }
}


void MainWindow::processIr2Activation(int zone_hint, const QString &source_label)
{
    int zone = zone_hint;
    if (zone <= 0) {
        zone = pending_zone_;
    }
    if (zone <= 0) {
        zone = zoneNumberForRegion(pending_image_region_);
    }

    if (zone <= 0) {
        appendLog(tr("%1: IR2 신호 처리 실패 → 유효한 존 정보가 없습니다.").arg(source_label));
        updatePendingZoneLabel();
        return;
    }

    appendLog(tr("%1: IR2 신호 수신 → 존 %2 출발 대기 (3초 후 TUR 전송)")
                  .arg(source_label)
                  .arg(zone));

    if (ui && ui->zoneCountLabel) {
        ui->zoneCountLabel->setText(tr("IR2 진행: 존 %1 (3초 대기)").arg(zone));
    }

    const int pending_before = pending_zone_;
    const std::uint64_t token_snapshot = pending_zone_token_;

    QTimer::singleShot(3000, this, [this, zone, source_label, pending_before, token_snapshot]() {
        appendLog(tr("%1: 대기 완료 → 존 %2 TUR 송신").arg(source_label).arg(zone));
        sendStmTurMessage();
        publishDestinationTopic(zone);
        if (pending_zone_token_ == token_snapshot && pending_zone_ == pending_before) {
            ++pending_zone_token_;
            pending_zone_ = -1;
            updatePendingZoneLabel();
        }
    });
}



void MainWindow::broadcastDestinationZone(const QString &zone_text, int zone_numeric)
{
    // 1) 우선순위: zone_numeric(명시적 힌트) > zone_text(숫자) > zone_text(지역명 매핑)
    auto toZoneNumber = [this](const QString &s) -> int {
        const QString t = s.trimmed();
        if (t.isEmpty()) return -1;
        bool ok = false;
        int z = t.toInt(&ok);
        if (ok) return z;                         // "1" 같은 숫자 문자열
        return zoneNumberForRegion(t);            // "서울"/"대전"/"부산" → 1/2/3
    };

    int finalZone = -1;

    // (A) det 파싱에서 넘어온 숫자 힌트가 가장 정확하므로 최우선 사용
    if (zone_numeric > 0) {
        finalZone = zone_numeric;
    } else {
        // (B) zone_text가 숫자면 그대로, 아니면 지역명 매핑
        const int fromText = toZoneNumber(zone_text);
        if (fromText > 0) {
            finalZone = fromText;
        }
    }

    // 유효성 체크: 1,2,3만 허용(필요 시 0/기타 처리 규칙에 맞춰 조정)
    if (finalZone <= 0) {
        appendLog(tr("STM 존 정보 전송 실패: 유효한 존이 없습니다. (text=%1, hint=%2)")
                      .arg(zone_text)
                      .arg(zone_numeric));
        return;
    }

    // 최종적으로 숫자 존만 송신
    const QString message = QStringLiteral("stm|zon|%1").arg(finalZone);

    if (sessions_.isEmpty()) {
        appendLog(tr("STM 존 정보 전송 실패: 연결된 클라이언트가 없습니다. (%1)").arg(message));
        return;
    }

    QString payload = message;
    if (!payload.endsWith(QLatin1Char('\n'))) {
        payload.append(QLatin1Char('\n'));
    }
    const QByteArray data = payload.toUtf8();

    int success_count = 0;
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
            continue;
        }
        if (socket->write(data) == -1) {
            appendLog(tr("STM 존 정보 전송 실패(%1): %2")
                          .arg(socket->peerAddress().toString(), socket->errorString()));
            continue;
        }
        socket->flush();
        ++success_count;
    }

    if (success_count == 0) {
        appendLog(tr("브로드캐스트 대상이 없습니다. → %1")
                      .arg(QString::fromUtf8(data).trimmed()));
    } else {
        appendLog(tr("브로드캐스트 완료(%1) → %2")
                      .arg(success_count)
                      .arg(QString::fromUtf8(data).trimmed()));
    }
}



void MainWindow::handleStmIr2Message(const QStringList &tokens, QTcpSocket *origin_socket)
{
    Q_UNUSED(origin_socket);

    QString stateToken = tokens.size() > 1 ? tokens.at(1).trimmed().toLower() : QStringLiteral("t");
    if (stateToken.isEmpty()) {
        stateToken = QStringLiteral("t");
    }
    const bool activated = stateToken == QStringLiteral("t")
                           || stateToken == QStringLiteral("true")
                           || stateToken == QStringLiteral("1")
                           || stateToken == QStringLiteral("on");
    if (!activated) {
        appendLog(tr("STM IR2 신호 무시: 상태(%1)").arg(tokens.value(1)));
        return;
    }
    processIr2Activation(-1, tr("STM"));
}

void MainWindow::publishDestinationTopic(int zone)
{
    if (!destination_pub_) {
        appendLog(tr("ROS 퍼블리셔(%1)가 초기화되지 않았습니다.")
                      .arg(QString::fromStdString(unload_destination_topic_)));
        return;
    }

    std_msgs::msg::Int32 msg;
    msg.data = zone;
    destination_pub_->publish(msg);

    if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "Publish unload destination %d on %s",
                    zone,
                    unload_destination_topic_.c_str());
    }

    appendLog(tr("ROS: /unload/destination 전송 → %1").arg(zone));
}


void MainWindow::sendStmTurMessage()
{
    broadcastMessageToClients(QStringLiteral("stm|tur|t"));
}



void MainWindow::forwardMessageToClient(const QString &raw_message, QTcpSocket *exclude)
{
    if (sessions_.isEmpty()) {
        appendLog(tr("포워드 실패: 연결된 클라이언트가 없습니다."));
        return;
    }

    QStringList parts = raw_message.split(QLatin1Char('|'), Qt::KeepEmptyParts);
    QString target_id;
    int target_index = -1;
    for (int i = 0; i < parts.size(); ++i) {
        const QString trimmed = parts.at(i).trimmed();
        if (!trimmed.isEmpty()) {
            target_id = trimmed;
            target_index = i;
            break;
        }
    }

    if (target_id.isEmpty()) {
        appendLog(tr("포워드 실패: 대상 ID를 찾을 수 없습니다."));
        return;
    }

    const QString loweredTarget = target_id.toLower();
    const bool broadcastRequested =
        loweredTarget == QStringLiteral("broadcast")
        || loweredTarget == QStringLiteral("all")
        || loweredTarget == QStringLiteral("@all")
        || loweredTarget == QStringLiteral("*")
        || target_id == QStringLiteral("전체");

    if (broadcastRequested) {
        QStringList remainingParts = parts;
        if (target_index >= 0 && target_index < remainingParts.size()) {
            remainingParts.removeAt(target_index);
        }
        QString broadcastPayload = remainingParts.join(QStringLiteral("|"));
        if (broadcastPayload.isEmpty()) {
            appendLog(tr("브로드캐스트 실패: 보낼 내용이 없습니다."));
            return;
        }
        if (!broadcastPayload.startsWith(QLatin1Char('|'))) {
            broadcastPayload.prepend(QLatin1Char('|'));
        }
        broadcastMessageToClients(broadcastPayload, exclude);
        return;
    }

    QString payload = raw_message;
    if (!payload.endsWith('\n')) {
        payload.append('\n');
    }
    const QByteArray data = payload.toUtf8();

    int success_count = 0;
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        QTcpSocket *socket = it.key();
        ClientSession *session = it.value();
        if (!socket || socket == exclude) {
            continue;
        }
        if (session && session->client_id.compare(target_id, Qt::CaseInsensitive) != 0) {
            continue;
        }
        if (socket->state() != QAbstractSocket::ConnectedState) {
            continue;
        }
        if (socket->write(data) == -1) {
            appendLog(tr("포워드 실패(%1): %2")
                          .arg(session && !session->client_id.isEmpty() ? session->client_id
                                                                      : socket->peerAddress().toString(),
                               socket->errorString()));
            continue;
        }
        socket->flush();
        ++success_count;
    }

    if (success_count == 0) {
        appendLog(tr("포워드 대상(%1)을 찾지 못했습니다.").arg(target_id));
    } else {
        appendLog(tr("포워드 완료(%1) → %2").arg(success_count).arg(QString::fromUtf8(data).trimmed()));
    }
}



MainWindow::ClientSession *MainWindow::sessionForSocket(QTcpSocket *socket) const
{
    if (!socket) {
        return nullptr;
    }
    auto it = sessions_.find(socket);
    if (it == sessions_.end()) {
        return nullptr;
    }
    return it.value();
}
