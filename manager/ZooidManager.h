#ifndef ZOOIDMANAGER_H
#define ZOOIDMANAGER_H
#pragma once

#include <iostream>
#include <algorithm>
#include <vector>
#include <mutex>
#include <thread>
#include <queue>
#include <map>
#include <cmath>
#include <string.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>


#include <QSettings>
#include <QDebug>
#include <QCoreApplication>
#include <QTime>
#include <QObject>

#include "public/config.h"
#include "public/errorcode.h"
#include "public/pf.h"
#include "ZooidGoal.h"
#include "Zooid.h"
#include "ZooidSimulator.h"
#include "ZooidAlgorithm.h"
#include "ZooidReceiver.h"
#include "ZooidDraw.h"
#include "ZooidMessage.h"
#include "Zooidinfo.h"
#include "ZooidFollow.h"
#include "ZooidCharge.h"
#include "ZooidVoronoi.h"
#include "ZooidTestMode.h"
#include "ZooidTestTargets.h"

using namespace std;
using namespace lzm;

// 前向声明
class LVoronoiViewBox;


class ZooidFollow;
class ZooidCharge;

enum class TestModeStatus
{
    Idle,
    StartPending,
    Running,
    Completed,
    Stopped,
    NoActiveRobots,
    AllTargetsLost,
    FeedbackStale,
    InvalidFeedback,
    InvalidGeometry,
    ReceiverError
};

class ZooidManager: public QObject
{
   Q_OBJECT

public:

    /**
     * @brief Constructor
     */
    ZooidManager();

    ~ZooidManager();

    /**
     * @brief 管理器初始化, 必须要调用初始化
     */
    void init();

    /**
     * @brief 获取模拟器运行状态
     * @return  返回当前模拟器运行的状态 Off关 On开 NoPlanning无方案
     */
    SimulationMode getSimulationMode();

    /**
     * @brief 设置模拟器的运行模式
     * @param mode  用来设置当前模拟器的状态 Off关 On开 NoPlanning无方案
     */
    void setSimulationMode(SimulationMode mode);

    /**
     * @brief 获取当前位置匹配模式
     * @return  返回当前匹配模式 OptimalAssignment; NaiveAssignment
     */
    AssignmentMode getAssignmentMode();

    /**
     * @brief 设置位置匹配模式
     * @param mode  需要设置的匹配模式
     */
    void setAssignmentMode(AssignmentMode mode);

    /**
     * @brief 分配机器人与目标位置的匹配关系
     */
    void assignRobots();

    /**
     * @brief 分配机器人与充电桩位置的匹配关系
     */
    void assignCharge();

    /**
     * @brief 获取当前的运行方案
     * @return  返回当前的运行方案
     */
    PlanningMode getPlanningMode();

    /**
     * @brief 设置当前的运行方案
     * @param mode          设置的方案模式
     * @param nbUseZooid    设置方案使用的机器人个数, 默认使用10个
     */
    void setPlanningMode(PlanningMode mode, int nb = 20);

    /**
     * @brief 设置自绘路径点的集合
     * @param _drawPathPoints   要设置的集合值
     */
    void setDrawPathPoints(vector<Vector2> _drawPathPoints);

    /**
     * @brief 运行生成的方案
     * @return  返回错误码
     */
    ErrorCode runPlanning();

    /**
     * @brief 选择合适的机器人从充电桩
     */
    void selectSuitableZooidFromCharge(int nbZooid);

    /**
     * @brief 设置模拟器显示电量
     * @param show  设置显现状态 true显示 false不显示
     */
    void setBatteryShow(bool show);

    /**
     * @brief 设置目标位置显示
     * @param show 设置显现状态 true显示 false不显示
     */
    void setGoalShow(bool show);

    /**
     * @brief 获取当前的机器人数
     * @return  返回机器人的个数
     */
    int getNbZooids();

    /**
     * @brief 获取足够多电量的机器人数
     * @return  返回满足条件的个数
     */
    int getEnoughBatteryNbZooids();

    /**
     * @brief 获取当前所有机器人的信息
     * @param allInfo   存放获取到的信息
     * @return          返回获取状态true
     */
    bool getAllZooidInfo(vector<ZooidInfo> &allInfo);

    /**
     * @brief 返回指定Id的zooid状态
     * @param zooidId   要检索的Zooid Id
     * @return          如果有，则为true;否则为false。
     */
    bool haveZooid(unsigned int zooidId);

    /**
     * @brief 获取机器人ID所在的编货
     * @param zooidId   要检索的机器人呢ID
     * @return  返回索引,没有找到返回-1
     */
    int getZooidIndex(unsigned int zooidId);

    /**
     * @brief 新添加一个机器人到管理器中
     * @param zooidId   配置新机器人的Id
     */
    void addZooid(unsigned int zooidId);

    /**
     * @brief 新添加一个机器人到管理器中
     * @param zooidId   配置新机器人的Id
     * @param position  配置新机器人的位置
     */
    void addZooid(unsigned int zooidId, Vector2 position);

    /**
     * @brief removeZooid
     * @param zooidId   要移除Id的机器人
     */
    void removeZooid(unsigned int zooidId);

    /**
     * @brief 设置给定编号机器人的角度
     * @param zooidId   要设置机器人角度的ID
     * @param angle     要设置的角度(角度)
     */
    void rotateZooid(unsigned int zooidId, float angle) ;

    /**
     * @brief 移动给定编号的目标
     * @param index   要移动目标的编号
     * @param position  要移动目标的机器人位置
     */
    void moveGoal(unsigned int zooidId, Vector2 position);

    /**
     * @brief 移动给定编号的目标
     * @param index 要移动目标的编号
     * @param x     要移动目标的位置x
     * @param y     要移动目标的位置y
     */
    void moveGoal(unsigned int zooidId, float x, float y);

    /**
     * @brief 获取当前连接器的数量
     * @return  返回连接器的数量
     */
    int getNbConnectedReceivers();

    /**
     * @brief 获取所有机器人时的充电状态
     * @return  返回到达的状态 true到达 false未到达
     */
    bool isChargeZooidAll();

    /**
     * @brief 设置电量阈值
     * @param batteryValue 要设置的电量值
     */
    void setBatteryLimit(unsigned int batteryValue);

    /**
     * @brief 设置的表演次数
     * @param showCountValue 要设置的表演次数
     */
    void setShowCount(unsigned int showCountValue);


    /**
     * @brief 获取当前电量限制值
     * @return
     */
    unsigned int getBatteryLimit();

    /**
     * @brief getShowCount
     * @return
     */
    unsigned int getShowCount();


    vector<Vector2>getFreeChargeFirstPosition();

    /**
     * @brief 获取二维空间坐标系最大宽度
     * @return  返回最大宽度
     */
    float getWorldWidth();
    /**
     * @brief 获取二维空间坐标系最大高度
     * @return  返回最大高度
     */
    float getWorldHeight();
    bool isReachedGoalAll();

    /** 启动一轮硬件同步测试。重复启动返回 false。 */
    bool startTestMode();

    /** 请求安全停止；管理线程会连续发送三轮零速。 */
    void stopTestMode();

    TestModeStatus getTestModeStatus() const;
    std::vector<unsigned int> getTestModeLostRobotIds() const;
    PursuitStatusSnapshot getTestModeSnapshot() const;
private slots:

    /**
     * @brief 管理器线程运行
     */
    void managerThreadRun();

    /**
     * @brief 管理器定时器运行
     */
    void managerTimerRun();

    /**
     * @brief 旋转到某一角度
     */
    void rotateToAngle(Zooid *z,float targetAngle, float currentAngle);

private:

    /**
     * @brief 更新Zooid在线状态
     */
    void onlineZooidUpdate();

    /**
     * @brief 更新机器人位置
     */
    void zooidPosUpdate();


    /**
     * @brief 更新充电方案
     */
    void chargeUpdate();


    /**
     * @brief 设置二维空间坐标系最大宽度
     * @param width 要设置的宽度
     */
    void setWorldWidth(float width);

    /**
     * @brief 设置二维空间坐标系最大高度
     * @param height 要设置的高度
     */
    void setWorldHeight(float height);

    /**
     * @brief 设置二维空间坐标系的大小
     * @param width     设置坐标系的最大宽度
     * @param height    设置坐标系的最大高度
     */
    void setWorldDimensions(float width, float height);

    /**
     * @brief 初始化模拟器
     */
    void initSimulation();

    /**
     * @brief 初始化跟随模式
     */
    void initFollow();

    /**
     * @brief 初始化充电模式
     */
    void initCharge();

    /**
     * @brief 初始化Vornoi算法控制
     */
    void initVoronoi();

    /**
     * @brief 加载配置参数
     */
    void loadParameters();

    /**
     * @brief 保存配置参数
     */
    void saveParameters();

    /**
     * @brief 运行模拟器
     * @return  返回运行状态,成功为true; 失败为false
     */
    bool runSimulation();

    /**
     * @brief 生成三角形方案
     * @return  返回错误码
     */
    ErrorCode generateTriangle();

    /**
     * @brief 生成一个矩形方案
     * @return  返回错误码
     */
    ErrorCode generateReact();

    /**
     * @brief 生成一个充电桩方案(对应充电桩位置)
     * @return  返回错误码
     */
    ErrorCode generateCharge();

    /**
     * @brief 生成一个圆形方案
     * @return  返回错误码
     */
    ErrorCode generateCircul();

    /**
     * @brief 生成自绘方案
     * @return  返回错误码
     */
    ErrorCode generateDrawpath();

    /**
     * @brief 生成跟随方案
     * @return  返回错误码
     */
    ErrorCode genrateFollow();

    /**
     * @brief 更新跟随
     */
    void updateFollow();

    /**
     * @brief 生成五角星方案
     * @return  返回错误码
     */
    ErrorCode genrateFivepointed();

    /**
     * @brief 生成六边形方案
     * @return  返回错误码
     */
    ErrorCode  genrateHexagon();

    /**
     * @brief 生成十字方案
     * @return  返回错误码
     */
    ErrorCode genrateCross();

    /**
     * @brief 初始化机器人的接收器(通信)
     * @return
     */
    bool initZooidReceivers();
    /**
     * @brief 获取当前在线的接收机串口名称
     * @return  返回所有的串口号
     */
    vector<string> getAvailableZooidReceivers();

    /**
     * @brief 处理接收到的数据
     */
    void processReceiversData();

    /**
     * @brief 设置给定Id的机器人的交互状态值
     * @param zooidId   要设置的机器人ID
     * @param touched   触摸状态
     * @param blinded   失去状态
     * @param tapped    轻拍状态
     * @param shaken    摇晃状态
     */
    void setZooidInteraction(unsigned int zooidId, bool touched, bool blinded, bool tapped, bool shaken);

    /**
     * @brief 设置机器人ID的位置信息
     * @param zooidId   要设置的机器人Id
     * @param motor1    电机1的速度
     * @param motor2    电机2的速度
     * @param color     颜色
     */
    void controlRobotSpeed(int zooidId, int16_t motor1, int16_t motor2, QColor color);

    /**
     * @brief 设置机器人ID的位置信息
     * @param zooidId           要设置的机器人Id
     * @param x                 位置x
     * @param y                 位置y
     * @param color             颜色
     * @param orientation       方向角(弧度)
     * @param preferredSpeed    速度
     * @param isFinalGoal       是否到达目标
     */
    void controlRobotPosition(uint8_t zooidId, float x, float y, QColor color, float orientation, float preferredSpeed, bool isFinalGoal);

    /**
     * @brief 根据id来检索对应的接收器
     * @param zooidId   要检索的机器人的zooidId
     * @return          返回机器人对应的接收器
     */
    ZooidReceiver* retrieveReceiver(unsigned int zooidId);

    /**
     * @brief 向机器人发送命令
     */
    void sendRobotsOrders();

    uint64_t steadyNowMs() const;
    std::vector<unsigned int> snapshotActiveZooidIds();
    std::vector<PursuitRobotState> snapshotTestRobots(uint64_t nowMs);
    void serviceTestMode(uint64_t nowMs);
    void sendTestCommand(const std::vector<unsigned int>& ids, WheelCommand command);
    void sendTestCommands(const std::map<unsigned int, WheelCommand>& commands);
    void flushReceiversForIds(const std::vector<unsigned int>& ids);
    void clearReceiverQueuesForIds(const std::vector<unsigned int>& ids);
    void beginSafeTestStop(TestModeStatus finalStatus);
    void serviceZeroStopBurst();
    bool anyReceiverWriteFailed();


public:
    /**
     * @brief 管理器的模拟器, 主要实现算法个显示
     */
    ZooidSimulator *zooidSimulator;

    /**
     * @brief 自绘路径点集合
     */
    vector<Vector2> drawPathPoints;

    /**
     * @brief 跟随模式的管理器
     */
    ZooidFollow *zooidFollow;

    /**
     * @brief 充电桩管理器
     */
    ZooidCharge *zooidCharge;
public slots:
    /**
     * @brief 点击模拟器
     */
    void setBattonPosition(int x, int y);

public:
    /**
     * @brief 启动Voronoi覆盖控制模式
     */
    void startVoronoiMode();
    void stopVoronoiMode();
    void stopAllZooids();                     // 停止所有机器人（发送零速）

    /**
     * @brief 更新Voronoi模式
     */
    void updateVoronoi();

    /**
     * @brief 生成Voronoi覆盖控制方案
     * @return  返回错误码
     */
    ErrorCode genrateVoronoi();

    /**
     * @brief 直接更新 Voronoi 显示（用于定时器回调）
     */
    void updateVoronoiDisplay();

    /**
     * @brief 启用/禁用静态障碍物的OAVC约束
     * @param enabled true=启用
     */
    void setObstacleEnabled(bool enabled);
    void setObstacleStatic();                  // 强制使用静态障碍物（速度=0）
    void setFormationMode(bool enabled);
    void setDualObstacleMode(bool enabled);
    void setPushWaveRobotMode(bool enabled);   // 推波机器人障碍物模式

private:
    unsigned int numZooids;                     //Zooid数量
    vector<Zooid*> myZooids;                    //Zooid
    vector<ZooidGoal*> myGoals;                 //Zooid目标位置
    lzm::Vector2 worldDimensions;               //坐标系

    Zooid* agentZooid;                          //代理测试

    vector<ZooidReceiver*> myReceivers;         //接收器
    int nbRequiredReceivers;                    //接收器数量

   // CenterControlComm *myCenter;                //中控传输串口
    std::thread managerThread;                  //管理器线程
    std::atomic<bool> updating;                 //管理器线程运行标记
    std::mutex valuesMutex;                     //信号量

    float currentUpdatePeriod;                  //当前更新周期
    QTimer * managerTimer;                      //管理器定时器

    SimulationMode simulationMode;              //模拟器模式
    AssignmentMode assignmentMode;              //分配模式
    PlanningMode planningMode;                  //分配方案

    float kSpeed;                               //保持速度
    float prefSpeed;                            //期望速度
    float uncertaintyOffset;                    //补偿

    bool showBattery;                           //显示电量
    bool showGoal;                              //目标点显示

    unsigned int batteryLimit;                  //电量限制
    unsigned int showCount;                     //表演次数

    int nbUseZooid;                             //使用机器人的个数

    std::vector<ZooidInfo>zooidRegisterBuff;    //记录新的机器人信息

    friend class ZooidFollow;
    friend class ZooidCharge;

    uint16_t cntOrder;                          //用于控制指令降频发送

    //Voronoi算法
    ZooidVoronoi* voronoiController = nullptr;
    bool voronoiRunning = false;
    float voronoiTime = 0.0f;

    // Voronoi 可视化
    class LVoronoiViewBox* voronoiViewBox = nullptr;
    QTimer* voronoiUpdateTimer = nullptr;  // Voronoi 更新定时器

    vector<Zooid*> voronoiActiveZooids;          // Voronoi模式下参与控制的真实机器人

    mutable std::mutex testModeMutex;
    std::condition_variable testModeStoppedCond;
    ZooidTestMode testMode;
    ZooidTestTargets testTargets;
    TestModeStatus testModeStatus;
    TestModeStatus pendingStopStatus;
    bool testStopRequested;
    int zeroStopCyclesRemaining;
    std::vector<unsigned int> testStopIds;
    std::vector<unsigned int> testLostIds;
    std::map<unsigned int, uint64_t> testFeedbackMs;
    uint64_t testSequence = 0;
};

#endif // ZOOIDMANAGER_H
