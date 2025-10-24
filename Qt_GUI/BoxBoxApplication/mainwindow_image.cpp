#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mainwindow_helpers.h"

#include <QDesktopServices>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QDateTime>
#include <QImage>
#include <QMetaObject>
#include <QPixmap>
#include <QRegularExpression>
#include <QTableView>
#include <QUrl>
#include <algorithm>

void MainWindow::displayImage(const QByteArray &data)
{
    QImage image;
    if (!image.loadFromData(data)) {
        appendLog(tr("이미지 디코딩 실패 (%1 bytes)").arg(data.size()));
        return;
    }

    current_image_pixmap_ = QPixmap::fromImage(image);
    if (image_label_) {
        const QSize target_size = image_label_->size();
        if (target_size.isValid()) {
            image_label_->setPixmap(current_image_pixmap_.scaled(
                target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            image_label_->setPixmap(current_image_pixmap_);
        }
        image_label_->setText(QString());
    }

    appendLog(tr("이미지 수신 완료 (%1 bytes)").arg(data.size()));

    QString region = pending_image_region_;
    auto ensureRegionHasPendingRecord = [this](const QString &candidate) -> bool {
        if (candidate.isEmpty()) {
            return false;
        }
        const auto it = pending_records_.find(candidate);
        if (it == pending_records_.end()) {
            return false;
        }
        const PendingRecord &record = it.value();
        return record.has_text && !record.has_image;
    };

    if (!ensureRegionHasPendingRecord(region)) {
        for (auto it = pending_records_.cbegin(); it != pending_records_.cend(); ++it) {
            if (it.value().has_text && !it.value().has_image) {
                region = it.key();
                break;
            }
        }
    }

    if (region.isEmpty()) {
        region = QStringLiteral("대전");
    }
    pending_image_region_ = region;
    QString saved_path;
    if (image_dir_ready_ || ensureImageDirectoryExists()) {
        saved_path = saveImageToDisk(region, image);
        if (saved_path.isEmpty()) {
            appendLog(tr("이미지를 저장하지 못했습니다."));
        }
    } else {
        appendLog(tr("이미지를 저장할 폴더가 준비되지 않았습니다."));
    }
    rememberImageForRegion(region, saved_path);
    tryCommitPendingRecord(region);
    pending_image_region_.clear();
}



void MainWindow::resetImageViewer()
{
    current_image_pixmap_ = QPixmap();
    if (image_label_) {
        image_label_->setPixmap(QPixmap());
        image_label_->setText(tr("이미지가 없습니다."));
    }
}



void MainWindow::resetImageReceptionState()
{
    for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        ClientSession *session = it.value();
        if (!session) {
            continue;
        }
        session->incoming_buffer.clear();
        session->image_buffer.clear();
        session->pending_image_bytes = 0;
        session->pending_image_region.clear();
    }
    pending_image_region_.clear();
}



bool MainWindow::ensureImageDirectoryExists()
{
    if (image_storage_dir_.isEmpty()) {
        return false;
    }
    QDir dir(image_storage_dir_);
    if (dir.exists()) {
        image_dir_ready_ = true;
        return true;
    }
    if (!dir.mkpath(QStringLiteral("."))) {
        image_dir_ready_ = false;
        appendLog(tr("이미지 폴더 생성 실패: %1").arg(image_storage_dir_));
        return false;
    }
    image_dir_ready_ = true;
    return true;
}



QString MainWindow::saveImageToDisk(const QString &region, const QImage &image)
{
    if (image.isNull()) {
        return {};
    }
    if (!ensureImageDirectoryExists()) {
        return {};
    }

    QDir dir(image_storage_dir_);
    QString safeRegion = region.isEmpty() ? QStringLiteral("unknown") : region;
    safeRegion.replace(QRegularExpression(QStringLiteral("[\\s/\\\\:]+")), QStringLiteral("_"));
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmsszzz"));
    QString baseName = QStringLiteral("%1_%2").arg(safeRegion, timestamp);
    QString filePath = dir.filePath(baseName + QStringLiteral(".png"));

    if (!image.save(filePath, "PNG")) {
        appendLog(tr("이미지 저장 실패: %1").arg(filePath));
        return {};
    }

    return QFileInfo(filePath).absoluteFilePath();
}



void MainWindow::handleTableActivation(const QModelIndex &index)
{
    if (!index.isValid()) {
        return;
    }
    const QAbstractItemModel *model = index.model();
    if (!model) {
        return;
    }
    const int columnCount = model->columnCount();
    if (columnCount <= 0) {
        return;
    }
    const int pathColumn = columnCount >= 2 ? columnCount - 2 : columnCount - 1;
    if (pathColumn < 0) {
        return;
    }

    const QModelIndex pathIndex = model->index(index.row(), pathColumn);
    QString imagePath = model->data(pathIndex, Qt::UserRole + 1).toString();
    if (imagePath.isEmpty()) {
        imagePath = model->data(pathIndex, Qt::DisplayRole).toString();
    }

    openImageFromPath(imagePath);
}



void MainWindow::openImageFromPath(const QString &imagePath)
{
    const QString path = imagePath.trimmed();
    if (path.isEmpty()) {
        appendLog(tr("열 수 있는 이미지가 없습니다."));
        return;
    }
    if (!QFile::exists(path)) {
        appendLog(tr("이미지 파일을 찾을 수 없습니다: %1").arg(path));
        return;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(path))) {
        appendLog(tr("이미지를 열 수 없습니다: %1").arg(path));
    }
}
