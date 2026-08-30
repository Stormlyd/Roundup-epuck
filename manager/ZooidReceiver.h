#ifndef ZOOIDRECEIVER_H
#define ZOOIDRECEIVER_H

#pragma once

#include <iostream>
#include <thread>
#include <iostream>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <windows.h>
#include <condition_variable>
#include <atomic>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>

#include "public/config.h"
#include "ZooidMessage.h"
#include "ZooidSerialport.h"

using namespace std;

#define MINIMUM_BYTES_TO_READ           5
#define NB_MAX_VALUES                   20

#define HANDSHAKE_REQUEST "Are you?"            //握手请求数据
#define HANDSHAKE_REPLY "You are?"              //握手应答数据
#define HANDSHAKE_LEAVE "Bye!"                  //握手离开数据

#define TYPE_UPDATE                             0x03    //更新
#define TYPE_STATUS                             0x04    //状态
#define TYPE_MOTORS_VELOCITY                    0x05    //电机速度
#define TYPE_ROBOT_POSITION                     0x06    //位置
#define TYPE_RECEIVER_INFO                      0x10    //接收信息
#define TYPE_RECEIVER_CONFIG                    0x11    //接收配置
#define TYPE_HANDSHAKE_REQUEST                  0x12    //握手请求
#define TYPE_HANDSHAKE_REPLY                    0x13    //握手应答
#define TYPE_HANDSHAKE_LEAVE                    0x14    //握手离开


#define RECEIVER_RECIPIENT                      250     //接收者
#define MANAGER_ID                              251     //管理器Id
#define BATON_ID                                0x11    //指挥棒


typedef struct {
    uint16_t positionX;     //x
    uint16_t positionY;     //y
    uint8_t colorRed;       //红LEd
    uint8_t colorGreen;     //绿LEd
    uint8_t colorBlue;      //蓝LEd
    uint8_t preferredSpeed; //速度
    int16_t orientation;    //方向
    bool isFinalGoal;       //是最终目标
    uint8_t empty;
    uint8_t controlMode;    //控制模式

}PositionControlMessage;//位置控制信息

typedef struct {
    uint16_t positionX;     //x
    uint16_t positionY;     //y
    int16_t orientation;    //方向
    uint8_t state;          //状态
    uint8_t batteryLevel;   //电量
}StatusMessage;//状态信息

typedef struct {
    uint8_t receiverId;     //接受id
    uint8_t numZooids;      //zooids数量
    uint8_t updateFrequency;//更新频率
    uint8_t empty;          //空(扩展)
}ReceiverConfigMessage;     //接受配置信息


class ZooidReceiver: public QObject
{
    Q_OBJECT
public:
    /**
     * @brief ZooidReceiver
     */
    ZooidReceiver();

    /**
     * @brief ZooidReceiver
     * @param id
     */
    ZooidReceiver(unsigned int id);

    /**
     * @brief ZooidReceiver
     * @param descriptor
     */
    ZooidReceiver(string descriptor);

    ~ZooidReceiver();

    /**
     * @brief 初始化接收器
     * @return 返回初始化状态, true成功; false失败
     */
    bool init();

    /**
     * @brief 初始化接收器
     * @param descriptor    要设置的串口名
     * @return  返回初始化状态, true成功; false失败
     */
    bool init(string descriptor);

    /**
     * @brief 连接串口
     * @param description   要连接的串口名
     * @param baudrate      要连接串口的波特率
     * @return              返回连接状态, true成功; false失败
     */
    bool connect(string description, int baudrate);

    /**
     * @brief 是否初始化状态
     * @return  返回初始化状态, true成功; false失败
     */
    bool isInitialized();

    /**
     * @brief 添加一条消息到消息缓冲区中
     * @param m 要添加的信息
     */
    void addMessage(ZooidMessage m);

    /**
     * @brief 获取消息缓冲区中最后一条信息
     * @return  返回之后一条数据
     */
    ZooidMessage getLastMessage();

    /**
     * @brief 返回当前消息缓冲区的数量
     * @return  返回消息数
     */
    unsigned int availableMessages();

    /**
     * @brief 时候有数据可发送
     * @return  返回结果
     */
    bool isDataReadyToSend();

    /**
     * @brief 获取当前接收器的ID
     * @return 返回ID
     */
    int getId();

    /**
     * @brief 获取最后一条数据帧
     * @return  字节
     */
    uint8_t* getLastData();

    /**
     * @brief 接收器复位
     */
    void reset();

    /**
     * @brief 接收器断开连接
     */
    void disconnect();

    /**
     * @brief 通过USB发送数据
     * @param type      发送数据类型
     * @param dest      目标地址
     * @param length    内容数据长度
     * @param data      数据内容
     * @param sendNow   是否现在发送数据
     */
    void sendUSB(uint8_t type, uint8_t dest, uint8_t length, uint8_t* data, bool sendNow = false);

    /**
     * @brief 设置准备发送
     */
    void setReadyToSend();

    /**
     * @brief 返回并清除异步串口写失败标记
     */
    bool consumeWriteFailure();

    /** 清除尚未写入串口的旧命令，用于紧急零速前去除过期非零帧。 */
    void clearPendingOutput();

private:
    /**
     * @brief usb数据接收线程服务程序
     */
    void usbReceivingRoutine();

    /**
     * @brief usb数据处理线程服务程序
     */
    void processIncomingData();

    /**
     * @brief usb数据发送线程服务程序
     */
    void usbSendingRoutine();

private:
    int receiverId;                                     //接收id
    ZooidSerialPort serialPort;                         //串口
    mutex dataInMutex, valuesMutex, dataOutMutex;       //并发互斥锁  输入数据 值 输出数据
    std::condition_variable sendingCond, processCond;   //并发编程-条件变量
    vector<char> bufferIn;                              //输入缓冲buffer
    vector<char> bufferOut;                             //输出缓冲buffer
    vector<ZooidMessage> incomingMessages;              //消息列表buffer
    std::thread processingThread;                       //数据处理线程
    std::thread receivingThread;                        //数据接收线程
    std::thread sendingThread;                          //数据发送线程
    unsigned int bytesToSend;                           //发送数据长度
    bool dataReceived;                                  //数据接收标记
    bool threadsRunning;                                //线程运行标记
    bool readyToSend;                                   //准备发送标记
    bool initialized;
    std::atomic<bool> writeFailure{false};
};


#endif // ZOOIDRECEIVER_H
