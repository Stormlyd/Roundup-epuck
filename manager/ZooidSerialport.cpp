#include "ZooidSerialPort.h"

/**
 * 重新用信号槽机制封装SerialPort
 * @brief ZooidSerialPort::ZooidSerialPort
 */
#include <QDebug>

ZooidSerialPort::ZooidSerialPort(const QString strComName)
: m_strComName(strComName),m_pThread(new QThread()), m_pCom(new QSerialPort()),m_iLen(-1), m_bOpen(false)
{
    m_pCom->moveToThread(m_pThread);
    this->moveToThread(m_pThread);
    m_pThread->start();

    connect(this, &ZooidSerialPort::sigSetup, this, &ZooidSerialPort::slotSetup);
    connect(this, &ZooidSerialPort::sigClose, this, &ZooidSerialPort::slotClose);
    connect(this, &ZooidSerialPort::sigClear, this, &ZooidSerialPort::slotClear);
    connect(this, &ZooidSerialPort::sigReadBufferLen, this, &ZooidSerialPort::slotReadBufferLen);
    connect(this, &ZooidSerialPort::sigWriteBytes, this, &ZooidSerialPort::slotWriteBytes);
}

// --------------------------------------------------------------------------
ZooidSerialPort::~ZooidSerialPort()
{
    close();
    m_pThread->quit();
    m_pThread->wait();

    delete m_pCom;
    m_pCom = nullptr;
    delete m_pThread;
    m_pThread = nullptr;
}

// --------------------------------------------------------------------------
bool ZooidSerialPort::isOpen() const
{
    return m_bOpen;
}

// --------------------------------------------------------------------------
bool ZooidSerialPort::setup(const QString strCOM,const int iBautRate,const int iDataBits,const char chParity,const char chStopBits )
{
    m_lockSetCOM.lock();
    const int nAvlb = m_semSetCOM.available();
    if(nAvlb > 0)
    {
        m_semSetCOM.tryAcquire(nAvlb);
    }
    emit sigSetup(strCOM, iBautRate, iDataBits, chParity, chStopBits);
    const bool bWait = m_semSetCOM.tryAcquire(1, 5000);// bool bWait = m_waitSetCOM.wait(&m_lockSetCOM, 5000);
    m_lockSetCOM.unlock();
    return bWait ? m_bOpen : false;

}

// --------------------------------------------------------------------------
bool ZooidSerialPort::writeBytes(const QByteArray &byteArray)
{
    return writeBytes(byteArray.data(), byteArray.size());
}

// --------------------------------------------------------------------------
bool ZooidSerialPort::writeBytes(const char *data, qint64 maxSize/* = -1*/)
{

    if(!m_bOpen)
    {
        return false;
    }

    m_lockWrite.lock();
    const int nAvlb = m_semWrite.available();
    if(nAvlb > 0)
    {
        m_semWrite.tryAcquire(nAvlb);
    }
    emit sigWriteBytes(data, maxSize);
    const bool bWait = m_semWrite.tryAcquire(1, 5000);
    m_lockWrite.unlock();
    QMutexLocker lk(&m_lockWriteLen);
    if(bWait)
    {
        return (m_iLen != -1);
    }
    else
    {
        return false;
    }
}

// --------------------------------------------------------------------------
int ZooidSerialPort::readBufferLen()
{
    m_lockReadBytes.lock();
    const int nAvlb = m_semReadBytes.available();
    if(nAvlb > 0)
    {
        m_semReadBytes.tryAcquire(nAvlb);
    }
    emit sigReadBufferLen();
    const bool bWait = m_semReadBytes.tryAcquire(1, 5000);
    m_lockReadBytes.unlock();

    QMutexLocker lk(&m_lockInBuffer);
    int iRet = bWait ? m_strInBuffer.size() : 0;
    return iRet;
}

// --------------------------------------------------------------------------
int ZooidSerialPort::readBuffer(char *data)
{
    m_lockInBuffer.lock();
    QByteArray buffer(m_strInBuffer);
    m_strInBuffer.clear();
    m_lockInBuffer.unlock();
    memcpy( data, buffer, buffer.size());
    return buffer.size();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::close()
{
   m_lockClose.lock();
   const int nAvlb = m_semClose.available();
   if(nAvlb > 0)
   {
       m_semClose.tryAcquire(nAvlb);
   }
   emit sigClose();
   m_semClose.tryAcquire(1, 5000);
   m_lockClose.unlock();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::clear()
{
    m_lockClear.lock();
    const int nAvlb = m_semClear.available();
    if(nAvlb > 0)
    {
        m_semClear.tryAcquire(nAvlb);
    }
    emit sigClear();
    m_semClear.tryAcquire(1, 5000);
    m_lockClear.unlock();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::slotWriteBytes(const char *pch, qint64 maxSize)
{
    m_lockWriteLen.lock();
    m_iLen = -1;

    if(m_pCom->isOpen())
    {
        m_iLen =  m_pCom->write(pch, maxSize);
        if(m_iLen == -1)
        {
            qDebug()<<"Message: serialport send error.";
        }
//        QString aa;
//        for(int i=0; i<maxSize; i++)
//        {
//            aa += QString::number(static_cast<uint8_t>(pch[i]), 16) + " ";
//        }
//        qDebug()<<aa;
    }
    m_lockWriteLen.unlock();
    m_semWrite.release();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::slotReadBufferLen()
{
    m_lockInBuffer.lock();
    m_strInBuffer.clear();
    if(m_pCom->isOpen())
    {
        m_strInBuffer = m_pCom->readAll();
    }
    m_lockInBuffer.unlock();
    m_semReadBytes.release();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::slotSetup(const QString strCOM, const int iBautRate, const int iDataBits, const char chParity, const char chStopBits)
{

    m_pCom->close();
    m_bOpen = false;
    if(strCOM.length() > 0)
    {
        m_strComName = strCOM;
    }
    if(m_strComName.length() < 1)
    {
        m_semSetCOM.release();
        return;
    }
    m_pCom->setPortName(m_strComName);

    if(false == m_pCom->isOpen())
    {
        if(false == m_pCom->open(QIODevice::ReadWrite))
        {
            m_semSetCOM.release();
        }
    }
    QSerialPort::BaudRate eBaudRate = QSerialPort::Baud9600;
    switch(iBautRate)
    {
    case 115200:
        eBaudRate = QSerialPort::Baud115200;
        break;
    case 9600:
        eBaudRate = QSerialPort::Baud9600;
        break;
    case 2400:
        eBaudRate = QSerialPort::Baud2400;
        break;
    case 1200:
        eBaudRate = QSerialPort::Baud1200;
        break;
    case 4800:
        eBaudRate = QSerialPort::Baud4800;
        break;
    case 19200:
        eBaudRate = QSerialPort::Baud19200;
        break;
    case 38400:
        eBaudRate = QSerialPort::Baud38400;
        break;
    case 57600:
        eBaudRate = QSerialPort::Baud57600;
        break;
    default:
        break;
    }
    m_pCom->setBaudRate(eBaudRate);

    QSerialPort::DataBits eDataBits = QSerialPort::Data8;
    switch(iDataBits)
    {
    case 8:
        eDataBits = QSerialPort::Data8;
        break;
    case 7:
        eDataBits = QSerialPort::Data7;
        break;
    case 6:
        eDataBits = QSerialPort::Data6;
        break;
    case 5:
        eDataBits = QSerialPort::Data5;
        break;
    default:
        break;
    }
    m_pCom->setDataBits(eDataBits);

    QSerialPort::Parity eParity = QSerialPort::NoParity;
    switch(chParity)
    {
    case 'N':
        eParity = QSerialPort::NoParity;
        break;
    case 'E':
        eParity = QSerialPort::EvenParity;
        break;
    case 'O':
        eParity = QSerialPort::OddParity;
        break;
    case 'S':
        eParity = QSerialPort::SpaceParity;
        break;
    case 'M':
        eParity = QSerialPort::MarkParity;
        break;
    default:
        break;
    }
    m_pCom->setParity(eParity);

    QSerialPort::StopBits eStopBits = QSerialPort::OneStop;
    switch(chStopBits)
    {
    case 1:
        eStopBits = QSerialPort::OneStop;
        break;
    case 2:
        eStopBits = QSerialPort::TwoStop;
        break;
    case 3:
        eStopBits = QSerialPort::OneAndHalfStop;
        break;
    default:
        break;
    }
    m_pCom->setStopBits(eStopBits);

    m_pCom->clear();
    m_bOpen = true;
    m_semSetCOM.release();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::slotClose()
{
    m_pCom->close();
    m_bOpen = false;
    m_semClose.release();
}

// --------------------------------------------------------------------------
void ZooidSerialPort::slotClear()
{
    m_pCom->clear();
    m_lockInBuffer.lock();
    m_strInBuffer.clear();
    m_lockInBuffer.unlock();
    m_semClear.release();
}

