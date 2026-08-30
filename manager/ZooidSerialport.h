
#ifndef ZooidSerialPort_H
#define ZooidSerialPort_H

#include <QObject>
#include <QSemaphore>
#include <QMutex>
#include <QRegExp>
#include <QThread>
#include <QSerialPort>

#include <iostream>

using namespace std;

class QThread;
class QSerialPort;
class ZooidSerialPort : public QObject
{
    Q_OBJECT
public:
    explicit ZooidSerialPort(const QString strComName = "");
    ~ZooidSerialPort();
    bool isOpen() const;
    bool setup(const QString strCOM = "", const int iBautRate = 9600, const int iDataBits = 8, const char chParity = 'N', const char chStopBits = 1);
    bool writeBytes(const QByteArray &byteArray);
    bool writeBytes(const char *data, qint64 maxSize = -1);
    void close();
    void clear();
    int readBufferLen();
    int readBuffer(char *data);

protected:
    QString m_strComName;
    QThread *m_pThread;
    QSerialPort *m_pCom;
    //write
    QMutex m_lockWrite;
    QSemaphore m_semWrite;
    QMutex m_lockWriteLen;
    qint64 m_iLen;
    //openCom
    QMutex m_lockSetCOM;
    QSemaphore m_semSetCOM;
    bool m_bOpen;
    //close
    QMutex m_lockClose;
    QSemaphore m_semClose;
    //clear
    QMutex m_lockClear;
    QSemaphore m_semClear;
    //readBytes
    QMutex m_lockReadBytes;
    QSemaphore m_semReadBytes;

    QMutex m_lockInBuffer;
    QByteArray m_strInBuffer;

private:

signals:
    void sigDataReady(QByteArray);
    void sigWriteBytes(const char *, qint64);
    void sigSetup(const QString, const int, const int, const char, const char);
    void sigClose();
    void sigClear();
    void sigReadBufferLen();
private slots:
    void slotWriteBytes(const char *pch, qint64);
    void slotSetup(const QString strCOM, const int iBautRate, const int iDataBits, const char chParity, const char chStopBits);
    void slotClose();
    void slotClear();
    void slotReadBufferLen();
};

#endif // ZooidSerialPort_H
