#ifndef ZOOIDMESSAGE_H
#define ZOOIDMESSAGE_H

#pragma once

#include <iostream>
#include <string>
#include <cstdio>
#include <iomanip>
#include <sstream>

using namespace std;

class ZooidMessage
{

public:
    /**
     * @brief ZooidMessage
     */
    ZooidMessage();

    /**
     * @brief ZooidMessage
     * @param _senderId
     * @param _type
     * @param _payload
     */
    ZooidMessage(uint8_t _senderId, uint8_t _type, uint8_t* _payload);

    ~ZooidMessage();

    /**
     * @brief 设置一个信息包的类型位
     * @param _type 要设置的类型
     */
    void setType(uint8_t _type);

    /**
     * @brief 获取一个信息包类型位
     * @return  返回当前类型
     */
    uint8_t getType();

    /**
     * @brief 设置一个信息包源地址ID
     * @param _senderId 发送者的Id
     */
    void setSenderId(uint8_t _senderId);

    /**
     * @brief 获取一个信息包源地址ID
     * @return 返回ID
     */
    uint8_t getSenderId();

    /**
     * @brief 设置一个信息包中的数据内容
     * @param _payload  有效数据
     */
    void setPayload(uint8_t* _payload);

    /**
     * @brief 获取一个信息包中的数据内容
     * @return  返回数据内容
     */
    uint8_t* getPayload();

    /**
     * @brief 制作一个信息包
     * @return  返回信息报字节数组
     */
    uint8_t* ToByteArray();

    /**
     * @brief 转到字符串
     * @return
     */
    string IntoString();

    /**
     * @brief 转到16进制
     * @return
     */
    string ToHexString();

private:
    uint8_t type;           //类型
    uint8_t senderId;       //发送者Id
    uint8_t* payload;       //有效数据
};

#endif // ZOOIDMESSAGE_H
