#ifndef ZOOIDCHARGE_H
#define ZOOIDCHARGE_H

#pragma once

#include <algorithm>
#include <vector>

#include <QObject>
#include <QDebug>
#include <QFile>

#include "public/config.h"
#include "orca/Vector2.h"
#include "ZooidManager.h"

using namespace lzm;

class ZooidManager;


typedef struct {
    int id;                 //充电桩ID
    bool has;               //是否被占用，False为空闲
    Vector2 first;          //机器人准备充电位置
    Vector2 second;         //充电桩位置
}ChargeMessage;

class ZooidCharge : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief ZooidCharge
     */
    ZooidCharge(ZooidManager *_manager);

    /**
     * @brief 加载资源
     */
    void loadParameters();

    /**
     * @brief 获取充电位置
     * @return  返回所有空闲第一位置的充电桩
     */
    vector<Vector2> getFreeChargeFirstPosition();

    /**
     * @brief 根据当前位置与充电第一位置匹配
     * @param goalPos 根据当前位置计算出当前充电桩id
     * @return  返回充电桩ID -1表示附近没有充电桩
     */
    int getNearFirstChargeId(Vector2 position);

    /**
     * @brief 根据当前位置与充电第二位置匹配
     * @param position 据当前位置计算出当前充电桩id
     * @return 返回充电桩ID -1表示附近没有充电桩
     */
    int getNearSecondChargeId(Vector2 position);

    /**
     * @brief 释放机器人所在的充电桩
     * @param zooidId
     */
    void releaseCharge(unsigned int id);


    /**
     * @brief 释放机器人所在的充电桩
     * @param zooidId
     */

    void releaseCharge(Vector2 position);

    /**
     * @brief 设置机器人的在充电桩充电
     * @param id        充电桩Id
     */
    void setChargeZooid(int id);

    /**
     * @brief 设置机器人的在充电桩充电
     * @param id        充电桩Id
     */
    void setChargeZooid(Vector2 position);


    /**
     * @brief 获取第一位置
     * @param id    充电桩Id
     * @return      返回第一位置
     */
    Vector2 getFirstPos(int id);

    /**
     * @brief 获取第二位置
     * @param id    充电桩Id
     * @return      返回第二位置
     */
    Vector2 getSecondPos(int id);


private:
    ZooidManager *manager;

    vector<ChargeMessage> chargeMsg;

    friend ZooidManager;
};

#endif // ZOOIDCHARGE_H
