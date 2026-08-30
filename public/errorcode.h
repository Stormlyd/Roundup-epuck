#ifndef ERRORCODE_H
#define ERRORCODE_H

#include <QString>

//分配模式
enum ErrorCode{
    NotError = 0,                //无错误
    NotHavePlanError,            //未选择方案错误
    ZooidNullError,              //在线机器人为空错误
    NumberError,                 // 机器人数量小于表演要求的最下数量
};


#endif // ERRORCODE_H
