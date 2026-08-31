#include "ZooidReceiver.h"

#include <chrono>
#include <cstring>
#include <fcntl.h>

namespace
{

uint64_t receiverSteadyNowMs()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<
        std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

} // namespace

ZooidReceiver::ZooidReceiver()
{
    receiverId = 0;
    threadsRunning = true;
    readyToSend = false;
    bytesToSend = 0;
    initialized = false;
    dataReceived = false;
    receiverSequence = 0;
    bufferReceivedAtMs = 0;
}

ZooidReceiver::ZooidReceiver(unsigned int id)
{
    receiverId = id;
    threadsRunning = true;
    readyToSend = false;
    bytesToSend = 0;
    initialized = false;
    dataReceived = false;
    receiverSequence = 0;
    bufferReceivedAtMs = 0;
}

ZooidReceiver::ZooidReceiver(string descriptor)
{
    receiverId = 0;
    threadsRunning = true;
    readyToSend = false;

    bytesToSend = 0;
    initialized = false;
    dataReceived = false;
    receiverSequence = 0;
    bufferReceivedAtMs = 0;
    init(descriptor);
}

ZooidReceiver::~ZooidReceiver()
{
    if(isInitialized())
    {
        disconnect();
    }
}

bool ZooidReceiver::init()
{
    //获取串口列表
    vector<string> devices;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        return connect(info.portName().toStdString(), 115200);
    }

    return false;
}

bool ZooidReceiver::init(string descriptor)
{
    return connect(descriptor, 115200);
}

bool ZooidReceiver::isInitialized()
{
    return initialized;
}

bool ZooidReceiver::connect(string description, int baudrate)
{
    if(serialPort.setup(QString::fromStdString(description), baudrate))
    {
        initialized = true;
        qDebug()<<"Message: Connecting to "<<QString::fromStdString(description);
        //启动接收线程
        receivingThread = std::thread(&ZooidReceiver::usbReceivingRoutine, this);
        //处理线程
        processingThread = std::thread(&ZooidReceiver::processIncomingData, this);
        //发送线程
        sendingThread = std::thread(&ZooidReceiver::usbSendingRoutine, this);
        //发送到usb设备 握手请求
        sendUSB(TYPE_HANDSHAKE_REQUEST, RECEIVER_RECIPIENT, sizeof(HANDSHAKE_REQUEST), (unsigned char*)HANDSHAKE_REQUEST, true);
        setReadyToSend();

        long timeOut = clock();
        ZooidMessage lastMessage;

        //等待握手数据 1s
        while (lastMessage.getType() != TYPE_HANDSHAKE_REPLY)
        {
            Sleep(1);
            long runtime = clock();
            lastMessage = getLastMessage();
            long t = runtime - timeOut;
            //握手超时
            if(t > 1000)
            {
                qDebug()<<"Message: handshake request timeout...";
                disconnect();
                return false;
            }
        }
        //握手应答成功 => 配置接收板
        if (lastMessage.getLength() == sizeof(HANDSHAKE_REPLY) &&
            lastMessage.getPayload() != nullptr &&
            std::memcmp(lastMessage.getPayload(), HANDSHAKE_REPLY,
                        sizeof(HANDSHAKE_REPLY)) == 0)
        {
            ReceiverConfigMessage config{};
            config.receiverId = receiverId;                         //配置接收板ID
            config.numZooids = NUM_ZOOIDS_PER_RECEIVER;             //配置x个Zooid使用一个接收板
            config.updateFrequency = int(SYSTEM_UPDATE_FREQUENCY);  //配置系统刷新频率
            sendUSB(TYPE_RECEIVER_CONFIG, RECEIVER_RECIPIENT, sizeof(config), (uint8_t*)&config, true);
            setReadyToSend();
            return true;
        }
        disconnect();
    }
    return false;
}

void ZooidReceiver::usbReceivingRoutine()
{
    while (threadsRunning)
    {
        Sleep(1);
        if (initialized && serialPort.isOpen())
        {
            const int bytesToRead = serialPort.readBufferLen();
            if (bytesToRead > 0)
            {
                std::vector<uint8_t> readData(
                    static_cast<std::size_t>(bytesToRead));
                const int bytesRead = serialPort.readBuffer(
                    reinterpret_cast<char*>(readData.data()));
                if (bytesRead > 0 && bytesRead <= bytesToRead)
                {
                    const uint64_t receivedAtMs = receiverSteadyNowMs();
                    unique_lock<mutex> lock(dataInMutex);
                    if (bufferIn.empty())
                        bufferReceivedAtMs = receivedAtMs;
                    bufferIn.insert(
                        bufferIn.end(), readData.begin(),
                        readData.begin() + bytesRead);
                    dataReceived = true;
                    lock.unlock();
                    processCond.notify_one();
                }
            }
        }
    }
}

void ZooidReceiver::processIncomingData()
{
    while (threadsRunning)
    {
        unique_lock<mutex> lock(dataInMutex);
        processCond.wait(lock, [this]() {
            return dataReceived || !threadsRunning;
        });
        if (!threadsRunning) break;
        dataReceived = false;

        uint8_t sourceAddr = 0;
        uint8_t messageType = 0;
        std::vector<uint8_t> payload;
        while (extractZooidFrame(
                   bufferIn, sourceAddr, messageType, payload))
        {
            const uint64_t sequence = ++receiverSequence;
            if (messageType == TYPE_RECEIVER_INFO)
            {
                receiverId = sourceAddr;
                qDebug()<<"Message: ZooidReceiver #"<<receiverId
                        <<" succesfully connected";
            }
            else
            {
                addMessage(ZooidMessage(
                    sourceAddr, messageType, payload.data(), payload.size(),
                    bufferReceivedAtMs, sequence));
            }
        }
        if (bufferIn.empty()) bufferReceivedAtMs = 0;
    }
}

void ZooidReceiver::usbSendingRoutine()
{
    while (threadsRunning)
    {
        if (initialized && serialPort.isOpen())
        {
            unique_lock<mutex> lock(dataOutMutex);
            {
                sendingCond.wait(lock, [this]() { return (bytesToSend > 0) | !threadsRunning; });
                if (bytesToSend>0)
                {
                    if(bytesToSend < 63)
                    {
                        try
                        {
                            if(serialPort.writeBytes(bufferOut.data(), bytesToSend))
                            {
                                bufferOut.erase(bufferOut.begin(), bufferOut.begin() + bytesToSend);
                                bytesToSend = 0;
                                readyToSend = false;
                            }
                            else
                            {
                                writeFailure.store(true);
                            }
                        }
                        catch(int e)
                        {
                            Q_UNUSED(e);
                            writeFailure.store(true);
                            initialized = false;
                        }
                    }
                    else
                    {
                        try
                        {
                            if(serialPort.writeBytes(bufferOut.data(), 63))
                            {
                                bufferOut.erase(bufferOut.begin(), bufferOut.begin() + 63);
                                bytesToSend -= 63;
                            }
                            else
                            {
                                writeFailure.store(true);
                            }
                        }
                        catch(int e)
                        {
                            Q_UNUSED(e);
                            writeFailure.store(true);
                            initialized = false;
                        }
                    }
                }
            }
            lock.unlock();
        }
    }
}

void ZooidReceiver::setReadyToSend()
{
    sendingCond.notify_one();
}

bool ZooidReceiver::consumeWriteFailure()
{
    return writeFailure.exchange(false);
}

void ZooidReceiver::clearPendingOutput()
{
    unique_lock<mutex> lock(dataOutMutex);
    bufferOut.clear();
    bytesToSend = 0;
    readyToSend = false;
}

void ZooidReceiver::sendUSB(uint8_t type, uint8_t dest, uint8_t length, uint8_t *data, bool sendNow)
{
    //线程在运行中 && 发送数据不为空
    if (threadsRunning && length > 0 && data != nullptr)
    {
        //锁定发送数据
        unique_lock<mutex> lock(dataOutMutex);
        {
            if (bufferOut.size() > 1024)
            {
                bufferOut.erase(bufferOut.begin(), bufferOut.begin() + length + 5);
            }

            bufferOut.push_back('~');   //帧头0x7e
            bufferOut.push_back(type);  //帧类型
            bufferOut.push_back(dest);  //接收者 ---就是把数据发送给谁
            bufferOut.push_back(length);//数据长度
            bufferOut.insert(bufferOut.end(), data, data + length);//数据
            bufferOut.push_back('!');   //帧尾0x21
            bytesToSend += length + 5;  //帧长度
        }
        lock.unlock();
        if (sendNow)
            setReadyToSend();
    }
}

void ZooidReceiver::addMessage(const ZooidMessage& m)
{
    lock_guard<mutex> lock(valuesMutex);
    incomingMessages.push(m);
}

ZooidMessage ZooidReceiver::getLastMessage()
{
    lock_guard<mutex> lock(valuesMutex);
    return incomingMessages.pop();
}

int ZooidReceiver::getId()
{
    return receiverId;
}

unsigned int ZooidReceiver::availableMessages()
{
    lock_guard<mutex> lock(valuesMutex);
    return static_cast<unsigned int>(incomingMessages.size());
}

bool ZooidReceiver::isDataReadyToSend()
{
    lock_guard<mutex> lock(valuesMutex);
    return !incomingMessages.empty();
}

void ZooidReceiver::reset()
{
    //serialPort.flush();
    unique_lock<mutex> lock(dataInMutex);
    bufferIn.clear();
    dataReceived = false;
    receiverSequence = 0;
    bufferReceivedAtMs = 0;
    lock.unlock();

    unique_lock<mutex> lock2(valuesMutex);
    incomingMessages.clear();
    lock2.unlock();
}

void ZooidReceiver::disconnect()
{
    qDebug()<<"Message: disconnect...";
    if(serialPort.isOpen() && initialized )
    {
       // initialized = false;
        sendUSB(TYPE_HANDSHAKE_LEAVE, RECEIVER_RECIPIENT, sizeof(HANDSHAKE_LEAVE), (unsigned char*)HANDSHAKE_LEAVE, true);
//        while(bufferOut.size() > 0)
//        {
//            Sleep(1);
//        }
    }

    //Sleep(1000);
    Sleep(100);

    //关闭线程标记
    threadsRunning = false;

    receivingThread.join();
    processCond.notify_one();
    processingThread.join();
    sendingCond.notify_one();
    sendingThread.join();

    unique_lock<mutex> lock(dataInMutex);
    bufferIn.clear();
    dataReceived = false;
    bufferReceivedAtMs = 0;
    lock.unlock();

    Sleep(200);

    unique_lock<mutex> lock1(dataOutMutex);
    bufferOut.clear();
    lock1.unlock();

    unique_lock<mutex> lock2(valuesMutex);
    incomingMessages.clear();
    lock2.unlock();
    serialPort.close();
    initialized = false;
}
