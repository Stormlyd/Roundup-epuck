#include <QApplication>
#include <QDesktopWidget>
#include <QFile>
#include <QDebug>

#include "public/style.h"
#include "public/config.h"

#include "log/log.h"

#if ENABLE_MEMWATCH
#include "module/memwatch.h"
#endif

#include "homePage.h"

void setStyle(const QString &qssFile)
{
    QFile qss(qssFile);
    qss.open(QFile::ReadOnly);
    qApp->setStyleSheet(qss.readAll());
    qss.close();
}


int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    qDebug()<<"--------------------------------------------";
    QString runtime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss ddd");
    qDebug()<<"App run time:"<<runtime;
    setStyle(":/new/ofapp/res/qss/ofapp.qss");
    HomePage home;
    home.show();

    return app.exec();
}
