#ifndef LOG_H
#define LOG_H

#pragma once

#include <qapplication.h>
#include <stdio.h>
#include <stdlib.h>
#include <QtMsgHandler>
#include <QString>
#include <QMutex>
#include <QDateTime>
#include <QIODevice>
#include <QFile>
#include <QTextStream>

#include "public/config.h"

class Log
{
public:
    Log();
    void static outputMessage(QtMsgType type, const QMessageLogContext &context, const QString &msg);

};

#endif  //LOG_H
