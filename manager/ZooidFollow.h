#ifndef ZOOIDFOLLOW_H
#define ZOOIDFOLLOW_H

#pragma once

#include <algorithm>
#include <vector>

#include <QTimer>
#include <QObject>
#include <QDebug>

#include "public/config.h"
#include "orca/Vector2.h"
#include "ZooidManager.h"
#include "ZooidVoronoi.h"

using namespace lzm;

class ZooidManager;


class ZooidFollow : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief 状态码
     */
    enum StateCode{
        Running = 0,
        Free,
        MoveFast,
        PosError,
    };
    /**
     * @brief ZooidFollow
     */
    ZooidFollow(ZooidManager *_mamger);


    /**
     * @brief 设置表演时长
     * @param _time  要设置的时间s
     */
    void setTime(unsigned int _time);

    /**
     * @brief 保存需要表演的机器人id
     * @param ids
     */
    void setZooids(vector<unsigned int> ids);

    vector<unsigned int>getZooids();

    /**
     * @brief 开始跟随
     */
    void begin();

    /**
     * @brief end
     */
    void end();

    /**
     * @brief 重置数据
     */
    void reset();

    /**
     * @brief 获取状态
     * @return  返回当前状态
     */
    int getState();

    /**
     * @brief 返回指挥棒位置
     * @return
     */
    Vector2 getBatonPosition();

    /**
     * @brief 设置指挥棒的位置
     * @param position
     */
    void setBatonPosition(Vector2 position);

    void setVoronoiController(ZooidVoronoi* controller) { voronoiController = controller; }

public slots:
    /**
     * @brief 定时器服务程序
     */
    void followTimerRun();

public:
    Vector2 baton;                //定义指挥棒,是一个位置

private:
    ZooidManager *mamger;
    unsigned int time;          //表演时长, 单位秒

    QTimer *followTimer;        //跟随器定时器
    int timeCount;              //计数器
    float performTime;

    int stateCode;              //跟随状态码

    vector<unsigned int> zooids;

    friend ZooidManager;

    ZooidVoronoi* voronoiController = nullptr;
};

#endif // ZOOIDFOLLOW_H

