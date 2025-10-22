#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDateTime>
#include <QFileInfo>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStandardItem>
#include <QTableView>
#include <QStringList>

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

    const bool listening = server_ && server_->isListening();

    if (!connected) {
        resetImageReceptionState();
        resetImageViewer();
    }

    if (stacked_widget_) {
        QWidget *target = listening ? main_page_ : connect_page_;
        if (target) {
            stacked_widget_->setCurrentWidget(target);
        } else {
            const int fallback = stacked_widget_->indexOf(listening ? main_page_ : connect_page_);
            if (fallback >= 0) {
                stacked_widget_->setCurrentIndex(fallback);
            } else {
                stacked_widget_->setCurrentIndex(listening ? 1 : 0);
            }
        }
    }

    const QString toggle_text = listening ? tr("서버 중지") : tr("서버 시작");
    if (ui->connectButton) {
        ui->connectButton->setText(toggle_text);
    }
    if (ui->connectPageConnectButton) {
        ui->connectPageConnectButton->setText(toggle_text);
    }

    const int client_count = sessions_.size();
    if (ui->sendButton) {
        ui->sendButton->setEnabled(client_count > 0);
    }

    QString status_text;
    if (listening) {
        if (client_count > 0) {
            QStringList labels;
            labels.reserve(client_count);
            for (auto it = sessions_.cbegin(); it != sessions_.cend(); ++it) {
                const ClientSession *session = it.value();
                if (!session) {
                    continue;
                }
                if (!session->client_id.isEmpty()) {
                    labels << session->client_id;
                }
            }
            if (labels.isEmpty()) {
                status_text = tr("클라이언트 연결됨 (%1명)").arg(client_count);
            } else {
                status_text = tr("클라이언트 연결됨 (%1명): %2")
                                  .arg(client_count)
                                  .arg(labels.join(QStringLiteral(", ")));
            }
        } else {
            status_text = tr("클라이언트 대기 중");
        }
    } else {
        status_text = tr("서버 중지됨");
    }

    if (ui->statusLabel) {
        ui->statusLabel->setText(status_text);
    }
    if (ui->connectPageStatusLabel) {
        ui->connectPageStatusLabel->setText(status_text);
    }

    if (ui->serverLineEdit) {
        ui->serverLineEdit->setReadOnly(listening);
    }
    if (ui->portLineEdit) {
        ui->portLineEdit->setReadOnly(listening);
    }
    if (ui->connectServerLineEdit) {
        ui->connectServerLineEdit->setReadOnly(listening);
    }
    if (ui->connectPortLineEdit) {
        ui->connectPortLineEdit->setReadOnly(listening);
    }
}



QByteArray MainWindow::buildMessagePayload(const QString &rawMessage, const QString &target) const
{
    QString message = rawMessage;
    Q_UNUSED(target);
    if (!message.endsWith('\n')) {
        message.append('\n');
    }

    return message.toUtf8();
}



void MainWindow::addTableRow(QStandardItemModel &model, const QStringList &values, const QString &imagePath)
{
    const int column_count = model.columnCount();
    if (column_count <= 0) {
        return;
    }

    const int imageColumn = column_count >= 2 ? column_count - 2 : column_count - 1;

    QList<QStandardItem *> items;
    items.reserve(column_count);
    for (int column = 0; column < column_count; ++column) {
        const QString value = column < values.size() ? values.at(column) : QString();
        auto *item = new QStandardItem(value);
        item->setEditable(false);
        if (column == imageColumn && !imagePath.isEmpty()) {
            const QString displayName = value.isEmpty()
                                            ? QFileInfo(imagePath).fileName()
                                            : value;
            item->setText(displayName);
            item->setData(imagePath, Qt::UserRole + 1);
            item->setToolTip(imagePath);
        }
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
