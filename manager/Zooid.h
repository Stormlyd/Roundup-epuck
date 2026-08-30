#ifndef ZOOID_H
#define ZOOID_H
#pragma once

#include <iostream>

#include <QGraphicsItem>
#include <QColor>
#include <QObject>
#include <QPainter>
#include <QGraphicsSceneMouseEvent>

#include "public/config.h"
#include "ZooidGoal.h"
#include "orca/Vector2.h"

#define NO_TOUCH        0

using namespace lzm;
using namespace std;

class Zooid : public QObject, public QGraphicsItem
{
    Q_OBJECT //宏，启动qt对象元系统

public:
    /**
     * @brief Constructor
     */
    Zooid();

    /**
     * @brief Constructor
     * @param _radius       机器人半径
     * @param _position     机器人初始位置
     * @param _batteryShow  机器人电量显示
     */
    Zooid(float _radius, Vector2 _position, bool _batteryShow);  //构造函数

     ~Zooid();  //析构函数，清理这个函数

    enum RobotType
    {
        ZooidRobot = 0,                      // Zooid
        BatonRobot,                          // 指挥棒
        AgentRobot,                          // 代理机器人
    };

    enum RobotControlMode
    {
        PositionControl=0,               //位置控制
        SpeedControl                     //速度控制
    };

    // PID控制器结构体
    struct PIDController
    {
        float kp, ki, kd;     // PID参数
        float target;         // 目标值
        float integral;       // 积分项
        float prevError;      // 上一次误差
        unsigned long prevTime; // 上一次计算时间

        void init(float p, float i, float d, float targetAngle)
        {
            kp = p;
            ki = i;
            kd = d;
            target = targetAngle;
            integral = 0.0f;
            prevError = 0.0f;
            prevTime = clock();
        }

        // 计算PID输出
        float compute(float current)
        {
            // 计算角度误差（考虑360°循环）
            float error = target - current;

            // 将误差归一化到[-180, 180]范围
            if (error > 180.0f)
                error -= 360.0f;
            else if (error < -180.0f)
                error += 360.0f;

            // 获取当前时间和时间间隔
            unsigned long currentTime = clock();
            float dt = (currentTime - prevTime) / 1000.0f; // 转换为秒
            if (dt <= 0.0f) dt = 0.01f; // 避免除零

            // 积分项（带抗饱和）
            integral += error * dt;
            // 限制积分项，防止积分饱和
            integral = constrain(integral, -100.0f, 100.0f);

            // 微分项
            float derivative = (error - prevError) / dt;

            // 计算PID输出
            float output = kp * error + ki * integral + kd * derivative;

            // 保存误差和时间
            prevError = error;
            prevTime = currentTime;

            return output;
        }
    };

    /**
     * @brief 机器人旋转PID
     */
    PIDController rotationPID;

    //以下为定义运算符，逻辑判断和赋值等
    /**
     * @brief operator ==
     * @param r
     * @return
     */
    bool operator == (const Zooid& r);

    /**
     * @brief operator >
     * @param r
     * @return
     */
    bool operator > (const Zooid& r);

    /**
     * @brief operator <
     * @param r
     * @return
     */
    bool operator < (const Zooid& r);

    /**
     * @brief operator !=
     * @param r
     * @return
     */
    bool operator != (const Zooid& r);

    /**
     * @brief operator =
     * @param z
     */
    void operator = (const Zooid &z);

    /**
     * @brief 设置机器人位置
     * @param _pos  要设置当前的位置
     */
    void setPosition(Vector2 _pos);

    /**
     * @brief 设置机器人位置
     * @param _x    要设置当前的位置x
     * @param _y    要设置当前的位置y
     */
    void setPosition(float _x, float _y);

    /**
     * @brief 设置机器人的目标位置
     * @param _pos  要设置当前目标位置
     */
    void setGoalPosition(Vector2 _pos);

    /**
     * @brief 设置机器人的目标位置
     * @param _x    要设置当前目标位置x
     * @param _y    要设置当前目标位置y
     */
    void setGoalPosition(float _x, float _y);

    /**
     * @brief 设置机器人的半径
     * @param _radius   当前机器人的半径
     */
    void setRadius(float _radius);

    /**
     * @brief 设置机器人的角度
     * @param _angle    要设置的角度(角度)
     */
    void setOrientation(float _angle);

    /**
     * @brief 设置机器人颜色
     * @param _color    要设置的颜色
     */
    void setColor(QColor _color);

    /**
     * @brief 设置机器人状态
     * @param _state    要设置的状态
     */
    void setState(unsigned int _state);

    /**
     * @brief 设置机器人ID
     * @param _id   要设置的ID编号
     */
    void setId(unsigned int _id);

    /**
     * @brief 设置机器人的电量
     * @param _battery  要设置的电量值
     */
    void setBatteryLevel(unsigned int _battery);

    /**
     * @brief 设置电量显示状态
     * @param _show 设置电量显示值, 显示true; 不显示false
     */
    void setBatteryShow(bool _show);

    /**
     * @brief 设置机器人类型
     * @param robotType 要设置的类型值,默认为ZooidRobot
     */
    void setRobotType(RobotType _robotType = ZooidRobot);



    void setRobotControlMode(RobotControlMode _robotControlMode);
    /**
     * @brief 设置机器人控制模式
     * @param robotType 要设置的模式
     */

    /**
     * @brief 设置机器人为活动状态
     */
    void activate();

    /**
     * @brief 设置 机器人为停用状态
     */
    void deactivate();

    /**
     * @brief 设置机器人的速度
     * @param _speed    要设置的速度值
     */
    void setSpeed(unsigned int _speed);

    /**
     * @brief 设置机器人到达目标的状态
     * @param _goalReached  设置到达目标点的状态值, 到达true; 未到达false
     */
    void setGoalReached(bool _goalReached);

    /**
     * @brief 设置最后更新数据的时间戳
     * @param time  时间值
     */
    void setLastUpdate(long time);

    /**
     * @brief 获取当前机器人的位置
     * @return  返回当前机器人的位置值
     */
    Vector2 getPosition();

    /**
     * @brief 获取当前机器人的目标位置
     * @return  返回当前机器人的目标位置值
     */
    Vector2 getGoalPosition();

    /**
     * @brief 获取当前机器人的方向
     * @return  返回当前机器人的角度值
     */
    float getOrientation();

    /**
     * @brief 获取机器人的状态
     * @return  返回机器人的状态值
     */
    unsigned int getState();

    /**
     * @brief 获取机器人的颜色
     * @return 返回当前机器人的颜色值
     */
    QColor getColor();

    /**
     * @brief 获取机器人的ID
     * @return  返回当前的ID值
     */
    unsigned int getId();

    /**
     * @brief 获取机器人的控制模式
     * @return  返回当前的控制模式
     */
    RobotControlMode getRobotControlMode();

    /**
     * @brief 获取机器人的半径
     * @return  返回当前机器人的半径值
     */
    float getRadius();

    /**
     * @brief 获取当前机器人的电量值
     * @return  返回电量
     */
    unsigned int getBatteryLevel();

    /**
     * @brief 获取机器人的活动状态
     * @return  返回活动状态值; 活动true, 停用false
     */
    bool isActivated();

    /**
     * @brief 获取机器人的充电状态
     * @return 0表演 1要回去的 2已经在充电桩
     */
    int getCharge();

    /**
     * @brief setCharge
     * @param chagre
     */
    void setCharge(int chagre);

    /**
     * @brief 获取机器人的速度值
     * @return  返回速度
     */
    unsigned int getSpeed();

    /**
     * @brief 看门狗
     */
    void tickWatchdog();

    /**
     * @brief 重置看门狗
     */
    void resetWatchdog();

    /**
     * @brief 获取机器人的连接状态
     * @return
     */
    bool isConnected();

    /**
     * @brief 获取几人的触摸状态
     * @return
     */
    bool isTouched();

    /**
     * @brief 获取机器人的失去状态
     * @return
     */
    bool isBlinded();

    /**
     * @brief 获取机器人的轻拍状态
     * @return
     */
    bool isTapped();

    /**
     * @brief 获取机器人的摇晃状态
     * @return
     */
    bool isShaken();

    /**
     * @brief 获取机器人到达目标状态
     * @return
     */
    bool isGoalReached();

    /**
     * @brief 获取电量显示状态
     * @return
     */
    bool isShowBattery();

    /**
     * @brief boundingRect
     * @return
     */
    QRectF boundingRect() const Q_DECL_OVERRIDE;

    /**
     * @brief shape
     * @return
     */
    QPainterPath shape() const Q_DECL_OVERRIDE;

    /**
     * @brief paint
     * @param painter
     * @param item
     * @param widget
     */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget) Q_DECL_OVERRIDE;//重绘

    /**
     * @brief 绘制Zooid机器人样式
     * @param painter
     */
    void drawZooidRobot(QPainter *painter);

    /**
     * @brief 绘制指挥棒机器人样式
     * @param painter
     */
    void drawBatonRobot(QPainter *painter);

    /**
     * @brief 绘制代理机器人样式
     * @param painter
     */
    void drawAgentRobot(QPainter *painter);

protected:
    /**
     * @brief mousePressEvent
     * @param event
     */
    void mousePressEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief mouseMoveEvent
     * @param event
     */
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief mouseReleaseEvent
     * @param event
     */
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) Q_DECL_OVERRIDE;

private:

    /**
     * @brief 机器人的唯一标识ID
     */
    unsigned int id;

    /**
     * @brief 机器人的半径
     */
    float radius;

    /**
     * @brief 机器人的角度
     */
    float orientation;

    /**
     * @brief 机器人的位置
     */
    Vector2 position ;

    /**
     * @brief 机器人的目标位置
     */
    Vector2 goalPosition;

    /**
     * @brief 机器人的变量值
     */
    unsigned int batteryLevel;      //电量

    /**
     * @brief 机器人的速度
     */
    unsigned int speed;

    /**
     * @brief 机器人的颜色
     */
    QColor color;

    /**
     * @brief 机器人的活动状态  1:运行中 0:停用
     */
    bool activated;

    /**
     * @brief 机器人是否要充电
     */
    int charge;

    /**
     * @brief 机器人的状态
     */
    unsigned int state;

    /**
     * @brief 机器人电量显示状态  1:显示 0:不显示
     */
    bool batteryShow;

    /**
     * @brief 机器人到达目标点状态  1:到达 0:未到达
     */
    bool goalReached;               //到达目标点

    /**
     * @brief 看门狗计数器
     */
    unsigned short watchdogCounter;

    /**
     * @brief 机器人最后更新数据时间戳
     */
    long lastUpdate;

    /**
     * @brief 机器人类型
     */
    RobotType robotType;

    /**
     * @brief 机器人控制模式
     */
    RobotControlMode robotControlMode;

};

#endif // ZOOID_H
