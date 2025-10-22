#ifndef MAINWINDOW_HELPERS_H
#define MAINWINDOW_HELPERS_H

#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QString>

inline QString envOrDefault(const char *name, const QString &fallback)
{
    const QString value = qEnvironmentVariable(name);
    return value.isEmpty() ? fallback : value;
}

inline int envOrDefaultInt(const char *name, int fallback)
{
    bool ok = false;
    const int value = qEnvironmentVariableIntValue(name, &ok);
    return ok ? value : fallback;
}

inline QString normalizedSourceName(const QString &source)
{
    return source.trimmed().toLower();
}

inline bool isKeywordToken(const QString &token)
{
    const QString upper = token.trimmed().toUpper();
    return upper == QStringLiteral("OCR")
        || upper == QStringLiteral("DET")
        || upper == QStringLiteral("WGT")
        || upper == QStringLiteral("WEIGHT")
        || upper == QStringLiteral("WT");
}

inline QString extractNumericToken(const QString &text)
{
    static const QRegularExpression re(QStringLiteral("([-+]?\\d+(?:\\.\\d+)?)"));
    const auto match = re.match(text);
    if (match.hasMatch()) {
        return match.captured(1);
    }
    return {};
}

#endif // MAINWINDOW_HELPERS_H
