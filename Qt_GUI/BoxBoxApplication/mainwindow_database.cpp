#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mainwindow_helpers.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QMetaObject>
#include <QSqlError>
#include <QSqlRecord>
#include <QStandardItem>
#include <QVariant>
#include <algorithm>

namespace {
inline int variantMetaTypeId(const QVariant &variant)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return variant.metaType().id();
#else
    return static_cast<int>(variant.type());
#endif
}
}

bool MainWindow::initializeDatabase()
{
    if (!QSqlDatabase::isDriverAvailable(QStringLiteral("QMYSQL"))) {
        appendLog(tr("QMYSQL 드라이버를 찾을 수 없습니다. Qt MySQL 드라이버를 설치하세요."));
        return false;
    }

    if (!database_connection_name_.isEmpty() && QSqlDatabase::contains(database_connection_name_)) {
        QSqlDatabase::removeDatabase(database_connection_name_);
    }

    database_ = QSqlDatabase::addDatabase(QStringLiteral("QMYSQL"), database_connection_name_);
    database_.setHostName(envOrDefault("BOXBOX_DB_HOST", QStringLiteral("127.0.0.1")));
    database_.setPort(envOrDefaultInt("BOXBOX_DB_PORT", 3306));
    database_.setDatabaseName(envOrDefault("BOXBOX_DB_NAME", QStringLiteral("BoxBoxMySql")));
    database_.setUserName(envOrDefault("BOXBOX_DB_USER", QStringLiteral("boxbox1")));
    database_.setPassword(envOrDefault("BOXBOX_DB_PASSWORD", QStringLiteral("PASSWD")));

    if (!database_.open()) {
        appendLog(tr("데이터베이스 연결 실패: %1").arg(database_.lastError().text()));
        return false;
    }

    table_weight_column_cache_.clear();

    return true;
}



void MainWindow::loadDatabaseSnapshot()
{
    if (!database_ready_) {
        return;
    }

    const QStringList regions = {
        QStringLiteral("서울"),
        QStringLiteral("부산"),
        QStringLiteral("대전"),
        QStringLiteral("미분류")
    };
    for (const QString &region : regions) {
        if (auto *model = modelForRegion(region)) {
            model->removeRows(0, model->rowCount());
        }
    }

    if (use_shared_table_) {
        populateAllModelsFromDatabase();
    } else {
        for (const QString &region : regions) {
            if (auto *model = modelForRegion(region)) {
                populateModelFromDatabase(*model, region);
            }
        }
    }
}



void MainWindow::populateModelFromDatabase(QStandardItemModel &model, const QString &region)
{
    if (!database_ready_) {
        return;
    }

    const QString table_name = tableForRegion(region);
    if (table_name.isEmpty()) {
        appendLog(tr("'%1' 지역에 대응하는 테이블명이 설정되지 않았습니다.").arg(region));
        return;
    }

    const bool hasTimestamp = !column_created_at_.isEmpty();

    const bool hasWeightColumn = tableHasWeightColumn(table_name);

    QStringList selectColumns;
    selectColumns << column_id_ << column_destination_;
    if (hasWeightColumn) {
        selectColumns << column_weight_;
    }
    selectColumns << column_notes_ << column_thumbnail_;
    if (hasTimestamp) {
        selectColumns << column_created_at_;
    }

    QString sql = QStringLiteral("SELECT %1 FROM %2")
                      .arg(selectColumns.join(QStringLiteral(", ")), table_name);

    const bool applyFilter = use_shared_table_ && use_region_column_;
    if (applyFilter) {
        sql.append(QStringLiteral(" WHERE %1 = :region").arg(column_region_));
    }
    if (!order_by_clause_.isEmpty()) {
        sql.append(QStringLiteral(" ORDER BY %1").arg(order_by_clause_));
    }

    QSqlQuery query(database_);
    query.prepare(sql);
    if (applyFilter) {
        query.bindValue(QStringLiteral(":region"), region);
    }

    if (!query.exec()) {
        appendLog(tr("데이터 조회 실패 (%1): %2").arg(region, query.lastError().text()));
        return;
    }

    while (query.next()) {
        int columnIndex = 0;
        const qint64 id = query.value(columnIndex++).toLongLong();
        const QString destinationValue = query.value(columnIndex++).toString();
        QString weightValue;
        if (hasWeightColumn) {
            weightValue = query.value(columnIndex++).toString();
        }
        const QString normalizedDestination = normalizeRegion(destinationValue);
        const QString resolvedRegion = normalizedDestination.isEmpty() ? region : normalizedDestination;
        if (resolvedRegion != region) {
            continue;
        }

        const QString notesValue = query.value(columnIndex++).toString();
        const QVariant imageVariant = query.value(columnIndex++);
        QString timestampValue;
        if (hasTimestamp) {
            const QVariant tsVariant = query.value(columnIndex++);
            if (variantMetaTypeId(tsVariant) == QMetaType::QDateTime) {
                timestampValue = tsVariant.toDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
            } else {
                timestampValue = tsVariant.toString();
            }
        }
        QString imagePath;
        if (variantMetaTypeId(imageVariant) == QMetaType::QByteArray) {
            const QByteArray blob = imageVariant.toByteArray();
            if (!blob.isEmpty()) {
                imagePath = QString::fromUtf8(blob);
            }
        } else {
            imagePath = imageVariant.toString();
        }
        const QString displayName = imagePath.isEmpty()
                                        ? QString()
                                        : QFileInfo(imagePath).fileName();

        addTableRow(model, {
                               id > 0 ? QString::number(id) : QString(),
                               resolvedRegion,
                               notesValue,
                               displayName,
                               timestampValue
                           }, imagePath);
    }
}



void MainWindow::populateAllModelsFromDatabase()
{
    if (!database_ready_) {
        return;
    }

    const QString table_name = table_name_seoul_;
    if (table_name.isEmpty()) {
        appendLog(tr("공유 테이블명이 설정되지 않았습니다."));
        return;
    }

    const bool hasTimestamp = !column_created_at_.isEmpty();

    const bool hasWeightColumn = tableHasWeightColumn(table_name);

    QStringList selectColumns;
    selectColumns << column_id_ << column_destination_;
    if (hasWeightColumn) {
        selectColumns << column_weight_;
    }
    selectColumns << column_notes_ << column_thumbnail_;
    if (hasTimestamp) {
        selectColumns << column_created_at_;
    }

    QString sql = QStringLiteral("SELECT %1 FROM %2")
                      .arg(selectColumns.join(QStringLiteral(", ")), table_name);

    if (!order_by_clause_.isEmpty()) {
        sql.append(QStringLiteral(" ORDER BY %1").arg(order_by_clause_));
    }

    QSqlQuery query(database_);
    if (!query.exec(sql)) {
        appendLog(tr("데이터 조회 실패: %1").arg(query.lastError().text()));
        return;
    }

    while (query.next()) {
        int columnIndex = 0;
        const qint64 id = query.value(columnIndex++).toLongLong();
        const QString destinationValue = query.value(columnIndex++).toString();
        QString weightValue;
        if (hasWeightColumn) {
            weightValue = query.value(columnIndex++).toString();
        }
        const QString region = normalizeRegion(destinationValue);
        if (region.isEmpty()) {
            continue;
        }

        const QString notesValue = query.value(columnIndex++).toString();
        const QVariant imageVariant = query.value(columnIndex++);
        QString timestampValue;
        if (hasTimestamp) {
            const QVariant tsVariant = query.value(columnIndex++);
            if (variantMetaTypeId(tsVariant) == QMetaType::QDateTime) {
                timestampValue = tsVariant.toDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss"));
            } else {
                timestampValue = tsVariant.toString();
            }
        }
        QString imagePath;
        if (variantMetaTypeId(imageVariant) == QMetaType::QByteArray) {
            const QByteArray blob = imageVariant.toByteArray();
            if (!blob.isEmpty()) {
                imagePath = QString::fromUtf8(blob);
            }
        } else {
            imagePath = imageVariant.toString();
        }
        const QString displayName = imagePath.isEmpty()
                                        ? QString()
                                        : QFileInfo(imagePath).fileName();

        if (auto *model = modelForRegion(region)) {
            addTableRow(*model, {
                                    id > 0 ? QString::number(id) : QString(),
                                    region,
                                    notesValue,
                                    displayName,
                                    timestampValue
                                }, imagePath);
        }
    }
}



qint64 MainWindow::insertRecordForRegion(const QString &region, const QStringList &fields, const QString &imagePath)
{
    if (!database_ready_) {
        return -1;
    }

    const QString table_name = tableForRegion(region);
    if (table_name.isEmpty()) {
        appendLog(tr("'%1' 지역에 대한 테이블명이 설정되지 않았습니다.").arg(region));
        return -1;
    }

    if (fields.size() < 3) {
        appendLog(tr("DB에 저장할 데이터가 부족합니다. (%1)").arg(region));
        return -1;
    }

    QStringList columns;
    QStringList values;

    if (use_region_column_ && !column_region_.isEmpty()) {
        columns << column_region_;
        values << QStringLiteral(":region");
    }

    const bool hasWeightColumn = tableHasWeightColumn(table_name);

    columns << column_destination_;
    values << QStringLiteral(":destination");
    if (hasWeightColumn) {
        columns << column_weight_;
        values << QStringLiteral(":weight");
    }
    columns << column_notes_ << column_thumbnail_;
    values << QStringLiteral(":notes") << QStringLiteral(":image_path");

    if (!column_created_at_.isEmpty()) {
        columns << column_created_at_;
        values << QStringLiteral("NOW()");
    }

    const QString sql = QStringLiteral("INSERT INTO %1 (%2) VALUES (%3)")
                            .arg(table_name,
                                 columns.join(QStringLiteral(", ")),
                                 values.join(QStringLiteral(", ")));

    QSqlQuery query(database_);
    query.prepare(sql);
    if (use_region_column_ && !column_region_.isEmpty()) {
        query.bindValue(QStringLiteral(":region"), region);
    }
    query.bindValue(QStringLiteral(":destination"), fields.value(0));

    if (hasWeightColumn) {
        const QString weightField = fields.value(1).trimmed();
        if (weightField.isEmpty()) {
            query.bindValue(QStringLiteral(":weight"), QVariant());
        } else {
            bool ok = false;
            const double weightNumber = weightField.toDouble(&ok);
            if (ok) {
                query.bindValue(QStringLiteral(":weight"), weightNumber);
            } else {
                query.bindValue(QStringLiteral(":weight"), weightField);
            }
        }
    }

    query.bindValue(QStringLiteral(":notes"), fields.value(2));
    query.bindValue(QStringLiteral(":image_path"), imagePath);

    if (!query.exec()) {
        appendLog(tr("DB 저장 실패 (%1): %2").arg(region, query.lastError().text()));
        return -1;
    }

    const QVariant inserted = query.lastInsertId();
    return inserted.isValid() ? inserted.toLongLong() : 0;
}



bool MainWindow::tableHasWeightColumn(const QString &table_name)
{
    if (!database_ready_ || table_name.isEmpty() || column_weight_.isEmpty()) {
        return false;
    }

    auto it = table_weight_column_cache_.find(table_name);
    if (it != table_weight_column_cache_.end()) {
        return it.value();
    }

    const QSqlRecord record = database_.record(table_name);
    const bool hasColumn = record.isEmpty() ? false : record.indexOf(column_weight_) != -1;
    table_weight_column_cache_.insert(table_name, hasColumn);
    if (!hasColumn) {
        appendLog(tr("테이블 '%1'에 무게 컬럼 '%2'이 없어 생략합니다.").arg(table_name, column_weight_));
    }
    return hasColumn;
}



QString MainWindow::tableForRegion(const QString &region) const
{
    if (region == QStringLiteral("서울")) {
        return table_name_seoul_;
    }
    if (region == QStringLiteral("부산")) {
        return table_name_busan_;
    }
    if (region == QStringLiteral("대전")) {
        return table_name_daejeon_;
    }
    if (region == QStringLiteral("미분류")) {
        return table_name_unclassified_;
    }
    return QString();
}



QStandardItemModel *MainWindow::modelForRegion(const QString &region)
{
    if (region == QStringLiteral("서울")) {
        return &ros_table_model_;
    }
    if (region == QStringLiteral("부산")) {
        return &sent_table_model_;
    }
    if (region == QStringLiteral("대전")) {
        return &received_table_model_;
    }
    if (region == QStringLiteral("미분류")) {
        return &unclassified_table_model_;
    }
    return nullptr;
}



QString MainWindow::determineRegionForSource(const QString &source) const
{
    const QString normalized = normalizeRegion(source);
    if (!normalized.isEmpty()) {
        return normalized;
    }
    const QString lower = normalizedSourceName(source);
    if (lower.contains(QStringLiteral("qt"))) {
        return QStringLiteral("미분류");
    }
    return QStringLiteral("미분류");
}



QString MainWindow::normalizeRegion(const QString &value) const
{
    const QString lower = normalizedSourceName(value);
    if (lower.isEmpty()) {
        return {};
    }
    if (lower.contains(QStringLiteral("seoul")) || lower.contains(QStringLiteral("서울"))) {
        return QStringLiteral("서울");
    }
    if (lower.contains(QStringLiteral("busan")) || lower.contains(QStringLiteral("부산"))) {
        return QStringLiteral("부산");
    }
    if (lower.contains(QStringLiteral("daejeon")) || lower.contains(QStringLiteral("대전")) ||
        lower.contains(QStringLiteral("jetson"))) {
        return QStringLiteral("대전");
    }
    if (lower.contains(QStringLiteral("err")) ||
        lower.contains(QStringLiteral("미분류")) ||
        lower.contains(QStringLiteral("unclassified")) ||
        lower.contains(QStringLiteral("unknown"))) {
        return QStringLiteral("미분류");
    }
    return {};
}



int MainWindow::zoneNumberForRegion(const QString &region) const
{
    const QString normalized = normalizeRegion(region);
    if (normalized == QStringLiteral("미분류")) {
        return 0;
    }
    if (normalized == QStringLiteral("서울")) {
        return 1;
    }
    if (normalized == QStringLiteral("대전")) {
        return 2;
    }
    if (normalized == QStringLiteral("부산")) {
        return 3;
    }
    return -1;
}



void MainWindow::handleStructuredMessage(const QString &source,
                                         const QString &body,
                                         int zone_hint,
                                         const QString &zone_literal)
{
    QStringList tokens = body.split(QLatin1Char('|'), Qt::KeepEmptyParts);
    for (QString &token : tokens) {
        token = token.trimmed();
    }

    bool isOcrError = false;
    QString destinationValue;
    QStringList detParts;
    QString weightCandidate;

    for (int i = 0; i < tokens.size(); ++i) {
        const QString token = tokens.at(i);
        const QString upper = token.toUpper();

        if (upper == QStringLiteral("OCR")) {
            const QString nextToken = tokens.value(i + 1);
            if (nextToken.compare(QStringLiteral("Err"), Qt::CaseInsensitive) == 0) {
                isOcrError = true;
            } else if (!nextToken.isEmpty() && !isKeywordToken(nextToken)) {
                destinationValue = nextToken;
            }
            continue;
        }

        if (upper == QStringLiteral("DET")) {
            detParts.clear();
            for (int j = i + 1; j < tokens.size(); ++j) {
                const QString candidate = tokens.at(j);
                if (isKeywordToken(candidate)) {
                    break;
                }
                if (!candidate.isEmpty()) {
                    detParts << candidate;
                }
            }
            continue;
        }

        if (upper == QStringLiteral("WGT") ||
            upper == QStringLiteral("WEIGHT") ||
            upper == QStringLiteral("WT")) {
            for (int j = i + 1; j < tokens.size(); ++j) {
                const QString candidate = tokens.at(j);
                if (isKeywordToken(candidate)) {
                    break;
                }
                const QString numeric = extractNumericToken(candidate);
                if (!numeric.isEmpty()) {
                    weightCandidate = numeric;
                    break;
                }
            }
            continue;
        }
    }

    if (destinationValue.isEmpty()) {
        for (int i = 0; i < tokens.size(); ++i) {
            if (!isKeywordToken(tokens.at(i))) {
                destinationValue = tokens.at(i);
                break;
            }
        }
    }

    const bool hasDet = !detParts.isEmpty();
    QString notesValue = detParts.join(QStringLiteral(" | "));
    if (!hasDet) {
        notesValue = QStringLiteral("-");
    }

    QString region = isOcrError
                         ? QStringLiteral("미분류")
                         : normalizeRegion(destinationValue.isEmpty() ? tokens.value(0) : destinationValue);
    if (region.isEmpty()) {
        region = determineRegionForSource(source);
    }
    if (region.isEmpty()) {
        appendLog(tr("지역을 판별할 수 없는 메시지: %1").arg(body));
        return;
    }

    QString destinationForDb = destinationValue.trimmed();
    if (destinationForDb.isEmpty()) {
        destinationForDb = region;
    }
    QString notesForDb = notesValue.trimmed();
    if (notesForDb.isEmpty() && tokens.size() > 2) {
        notesForDb = tokens.mid(2).join(QStringLiteral(" | "));
    }

    QString weightValue = weightCandidate;
    const bool pendingRegionMatches = pending_weight_region_.isEmpty()
                                      || region.isEmpty()
                                      || pending_weight_region_.compare(region, Qt::CaseInsensitive) == 0;
    bool usedPendingWeight = false;
    bool shouldClearPendingWeight = false;

    if (hasDet && !pending_weight_value_.isEmpty() && pendingRegionMatches) {
        shouldClearPendingWeight = true;
        if (weightValue.isEmpty()) {
            weightValue = pending_weight_value_;
            usedPendingWeight = true;
        }
    }

    if (weightValue.isEmpty()) {
        for (const QString &token : tokens) {
            const QString numeric = extractNumericToken(token);
            if (!numeric.isEmpty()) {
                weightValue = numeric;
                break;
            }
        }
    }

    if (usedPendingWeight) {
        if (region.isEmpty()) {
            appendLog(tr("임시 무게 적용: %1 kg").arg(weightValue));
        } else {
            appendLog(tr("임시 무게 적용(%1): %2 kg").arg(region, weightValue));
        }
    }
    if (shouldClearPendingWeight) {
        pending_weight_value_.clear();
        pending_weight_region_.clear();
    }

    QStringList fields;
    fields << destinationForDb
           << weightValue.trimmed()
           << notesForDb;

    while (fields.size() < 3) {
        fields << QString();
    }

    rememberTextForRegion(region, fields, zone_hint, zone_literal);
    tryCommitPendingRecord(region);
}



void MainWindow::rememberTextForRegion(const QString &region,
                                       const QStringList &fields,
                                       int zone_hint,
                                       const QString &zone_literal)
{
    PendingRecord &record = pending_records_[region];
    record.fields = fields;
    record.has_text = true;
    if (zone_hint >= 0) {
        record.zone_hint = zone_hint;
    }
    if (!zone_literal.trimmed().isEmpty()) {
        record.zone_text = zone_literal.trimmed();
    } else if (zone_hint >= 0) {
        record.zone_text = QString::number(zone_hint);
    }
    pending_image_region_ = region;
}



void MainWindow::updatePendingRecordWeight(const QString &region, const QString &weight)
{
    const QString trimmedRegion = region.trimmed();
    const QString trimmedWeight = weight.trimmed();
    if (trimmedRegion.isEmpty() || trimmedWeight.isEmpty()) {
        return;
    }

    auto it = pending_records_.find(trimmedRegion);
    if (it == pending_records_.end()) {
        return;
    }

    PendingRecord &record = it.value();
    if (!record.has_text) {
        return;
    }

    QStringList fields = record.fields;
    while (fields.size() < 3) {
        fields << QString();
    }
    fields[1] = trimmedWeight;
    record.fields = fields;

    tryCommitPendingRecord(trimmedRegion);
}

void MainWindow::rememberImageForRegion(const QString &region, const QString &imagePath)
{
    PendingRecord &record = pending_records_[region];
    record.image_path = imagePath;
    record.has_image = !imagePath.isEmpty();
}

void MainWindow::tryCommitPendingRecord(const QString &region)
{
    PendingRecord &record = pending_records_[region];
    if (!record.has_text || !record.has_image) {
        return;
    }

    QStringList fields = record.fields;
    if (fields.size() < 3) {
        while (fields.size() < 3) {
            fields << QString();
        }
    }

    const qint64 inserted_id = insertRecordForRegion(region, fields, record.image_path);
    if (inserted_id >= 0) {
        appendLog(tr("DB 저장 완료 (%1): ID=%2").arg(region, QString::number(inserted_id)));
        const int zone_from_region = zoneNumberForRegion(region);
        int zone_numeric = -1;
        if (record.zone_hint >= 0) {
            zone_numeric = record.zone_hint;
            if (zone_from_region >= 0 && zone_from_region != zone_numeric) {
                appendLog(tr("STM 존 정보 불일치 감지 → 힌트 %1 / 지역 %2 (%3) 중 힌트 값을 우선 사용합니다.")
                              .arg(record.zone_hint)
                              .arg(region)
                              .arg(zone_from_region));
            }
        } else {
            zone_numeric = zone_from_region;
        }

        QString zone_literal = record.zone_text.trimmed();
        if (zone_literal.isEmpty() && zone_numeric >= 0) {
            zone_literal = QString::number(zone_numeric);
        }

        if (zone_literal.isEmpty()) {
            appendLog(tr("STM 존 정보 전송 실패: 존 정보가 비어 있습니다. (%1)").arg(region));
        } else {
            broadcastDestinationZone(zone_literal, zone_numeric);
        }

        QMetaObject::invokeMethod(this, [this]() {
            loadDatabaseSnapshot();
        }, Qt::QueuedConnection);
    }

    record = PendingRecord{};
}
