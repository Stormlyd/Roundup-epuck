#include "ZooidManager.h"
#include "../component/LVoronoiViewBox.h"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <chrono>
#include <set>
#include <iterator>

#include "ZooidCoordinates.h"
#include "ZooidSpeedCodec.h"

ZooidManager::ZooidManager()
{
    worldDimensions = Vector2(0.0f, 0.0f);
    showBattery = false;
    showGoal = false;
    updating = true;

    batteryLimit = 50;

#if DEBUG_MODE_SOFTWARA
    numZooids = INIT_ROBOT_NUMBER;
#else
    numZooids = 0;
#endif
    nbRequiredReceivers = 6;
    nbUseZooid = 0;
    managerTimer = nullptr;
    testModeStatus = TestModeStatus::Idle;
    pendingStopStatus = TestModeStatus::Stopped;
    testStopRequested = false;
    zeroStopCyclesRemaining = 0;
}

ZooidManager::~ZooidManager()
{
    // 接收器仍在线时先完成安全零速，再关闭管理线程和串口。
    stopTestMode();
    bool stopTimedOut = false;
    {
        unique_lock<mutex> lock(testModeMutex);
        const bool stopped = testModeStoppedCond.wait_for(lock, std::chrono::milliseconds(250), [this]() {
            return !testStopRequested && zeroStopCyclesRemaining == 0;
        });
        stopTimedOut = !stopped;
    }

    if (stopTimedOut)
    {
        const std::vector<unsigned int> activeIds = snapshotActiveZooidIds();
        clearReceiverQueuesForIds(activeIds);
        sendTestCommand(activeIds, {0, 0});
        flushReceiversForIds(activeIds);
    }

    updating.store(false);
    if (managerThread.joinable())
        managerThread.join();

    //释放场景刷新定时器
    delete managerTimer;
    managerTimer = nullptr;

    //释放接收器
    for (int i = 0; i < myReceivers.size(); i++)
    {
        delete(myReceivers[i]);
        myReceivers[i] = nullptr;
    }
    myReceivers.clear();

    //delete myCenter;
    //myCenter = nullptr;

    //释放指挥棒
    delete zooidFollow;
    zooidFollow = nullptr;

    //释放充电桩
    delete zooidCharge;
    zooidCharge = nullptr;

    //释放模拟器
    delete zooidSimulator;
    zooidSimulator = nullptr;

    //释放中控
   // delete myCenter;
    //myCenter=nullptr;

    myZooids.clear();
    myGoals.clear();
}

void ZooidManager::init()
{
    //加载参数
    loadParameters();
    //初始化模拟器
    initSimulation();
    //初始化跟随模式管理器
    initFollow();
    //初始化充电桩
    initCharge();
    //初始化接收器
    initZooidReceivers();
    //初始化Voronoi
    initVoronoi();

    //生成numZooids个Zooid与ZooidGoal
    for (int i = 0; i < numZooids; i++)
    {
        Vector2 position(pfRandFloat(0.1f, getWorldWidth() - 0.1f), pfRandFloat(0.1f, getWorldHeight() - 0.1f));
        Zooid *tmpRobot = new Zooid(ROBOT_RADIUS, position, showBattery);
        ZooidGoal *tmpGoal = new ZooidGoal(position, QColor(pfRandInt(50, 255),pfRandInt(50, 255),pfRandInt(50, 255)), showGoal);

        unsigned int goalId = (unsigned int)zooidSimulator->addGoal(position);
        unsigned int agentId = (unsigned int)zooidSimulator->addAgent(position, goalId);

        tmpRobot->setGoalPosition(position);
        tmpRobot->setBatteryLevel( 100 - i);
        tmpRobot->setColor(tmpGoal->getColor());
        tmpRobot->setId(agentId);
        tmpRobot->setCharge(0);
        tmpGoal->setAssociatedZooid(agentId);

        myZooids.push_back(tmpRobot);
        myGoals.push_back(tmpGoal);

        zooidSimulator->addZooid(tmpRobot);
        zooidSimulator->addZooidGoal(tmpGoal);
    }

    //分配机器人
    assignRobots();

#if AGENT_DEBUG
    agentZooid = new Zooid(ROBOT_RADIUS, Vector2(0,0), false);
    agentZooid->setColor(QColor("#ffffff"));
    agentZooid->setId(255);
    agentZooid->setRobotType(Zooid::AgentRobot);
    zooidSimulator->addZooid(agentZooid);
#endif

    //启动管理器线程
    managerThread = std::thread(&ZooidManager::managerThreadRun, this);

    //启动管理器定时器
    managerTimer = new QTimer();
    QObject::connect(managerTimer, SIGNAL(timeout()), this, SLOT(managerTimerRun()));
    managerTimer->start(static_cast<int>(SYSTEM_UPDATE_PERIOD));
}

void ZooidManager::initVoronoi()
{
    voronoiController = new ZooidVoronoi(this);

    // 设置场地（根据你的实际场地，单位：米）
    // 你的代码中 worldDimensions 是 1.460 x 0.914（米）
    voronoiController->setFieldBounds(0, 0, getWorldWidth(), getWorldHeight());

    // 设置机器人半径（Zooid直径约26mm，半径0.013m）
    voronoiController->setRobotRadius(0.013f);

    // 设置控制参数（与MATLAB一致）    //CVT系数，斥力系数，速度长度比例，最大速度
    voronoiController->setControlParams(25.0f, 0.5f, 20.0f, 5.0f);  //25,0.3,20.0,5.0

    // 设置障碍物参数（推波机制：从左外侧启动，低速向右穿越）
    voronoiController->setObstacleParams(
        0.3f,                          // 边长 0.20m (200mm)
        Vector2(-0.2f, 0.457f),         // 初始位置（左外侧）
        Vector2(1.0f, 0.0f),            // 向右运动
        0.003f,                          // 初速度 0.05 m/s（原0.1的一半）
        0.001f,                          // 加速度
        0.01f                            // 最大速度 0.1 m/s
    );

    // 添加静态障碍物备用（默认禁用）
    float cx = getWorldWidth() / 2.0f;
    float cy = getWorldHeight() / 2.0f;
    voronoiController->addStaticObstacle(
        Vector2(cx, cy),                // 场地中央
        0.2f,                           // 边长 0.2m (200mm)
        3                               // 每条边 3 个圆
    );
    voronoiController->setObstacleEnabled(false);  // 默认禁用，由按钮启用

    // 配置编队推波参数（编队左边贴紧全局左边界，初始无障碍）
    float fcy = getWorldHeight() / 2.0f;
    float formW = 0.55f;                               // 编队宽度（米）
    float formH = 0.65f;                               // 编队高度（米）
    voronoiController->setFormationParams(
        formW, formH,                                 // 编队宽高
        0.005f,                                       // 前馈速度（m/s）
        Vector2(formW/2.0f, fcy),                     // 编队起点（左边贴x=0）
        Vector2(getWorldWidth() - formW/2.0f, fcy)    // 编队目标（右边贴右边界）
    );
    // 编队内5个机器人相对偏移（适配缩小的编队）
    voronoiController->setFormationOffsets({
        Vec2(-0.18f,  0.18f),
        Vec2(-0.18f, -0.18f),
        Vec2( 0.18f,  0.18f),
        Vec2( 0.18f, -0.18f),
        Vec2( 0.0f,  -0.18f)
    });

    // 配置双障碍物模式参数
    voronoiController->setObstacleRobotParams(
        0.2f,                          // 障碍物机器人速度（m/s），原0.2
        0.1f,                           // 障碍物边长 0.1m=100mm
        Vec2(-1.0f, 0.0f)               // 从右向左
    );
    // 第二个静态障碍物（场地正中心下方）
    voronoiController->addStaticObstacle2(
        Vector2(getWorldWidth() / 2.0f, 0.1f),  // 中心, 下端贴底 (y=side/2)
        0.2f,                           // 边长 0.2m=200mm
        3
    );

    // 连接速度命令信号到硬件接口
    connect(voronoiController, &ZooidVoronoi::speedCommand,
            this, [this](int id, int16_t left, int16_t right, QColor color){
                this->controlRobotSpeed(id, left, right, color);
            });

    // 传给 ZooidFollow 用于初始化
    zooidFollow->setVoronoiController(voronoiController);
}

void ZooidManager::initSimulation()
{
    //创建模拟器
    zooidSimulator = new ZooidSimulator();

    //设置模拟器大小 单位mm 对应实际场地 1460mm × 914mm
    zooidSimulator->setSize(1460, 914);

    //开启OpenGl
    zooidSimulator->setOpenGlView();

    //打开模拟器
    simulationMode = On;

    //使用最优分配 NaiveAssignment OptimalAssignment
    assignmentMode = OptimalAssignment;

    //设置分配方案
    planningMode = NullPlanning;

    //设置运动参数
    kSpeed = 1.0f;
    prefSpeed = 1.0f * MaxSpeed;
    uncertaintyOffset = 0.05f * prefSpeed;

    //设置模拟器更新频率
    zooidSimulator->setTimeStep(SYSTEM_UPDATE_FREQUENCY / 1000.0f);

    //设置默认代理参数
    zooidSimulator->setAgentDefaults(NeighborDist, MaxNeighbors, ROBOT_RADIUS, GoalRadius, prefSpeed, MaxSpeed,TIME_TO_ORIENTATION, WheelTrack, uncertaintyOffset, MaxAccel, lzm::Vector2(0.0f, 0.0f),0.0f);

    QObject::connect(zooidSimulator, SIGNAL(sendClickPosition(int, int)), this, SLOT(setBattonPosition(int,int)));
}

void ZooidManager::setBattonPosition(int x, int y)
{
    float clickX = pfMap((float)x, 0, 960 , 0.0f,getWorldWidth() - 0);
    float clickY = pfMap((float)y,0, 610 , 0.0f,getWorldHeight()- 0);

    if(clickY < 0.2 || clickY > 0.7 || clickX < 0.2 || clickX > 1.2){
        return ;
    }

    zooidFollow->setBatonPosition(Vector2(clickX, clickY));
}

void ZooidManager::initFollow()
{
    zooidFollow = new ZooidFollow(this);
    // voronoiController 移到 initVoronoi() 中统一创建，避免重复
}

void ZooidManager::initCharge()
{
    zooidCharge = new ZooidCharge(this);
}

float ZooidManager::getWorldWidth()
{
    return worldDimensions.getX();
}

float ZooidManager::getWorldHeight()
{
    return worldDimensions.getY();
}

void ZooidManager::setWorldWidth(float width)
{
    if(width > 0.0f)
    {
        worldDimensions.setX(width);
    }
}

void ZooidManager::setWorldHeight(float height)
{
    if(height > 0.0f)
    {
        worldDimensions.setY(height);
    }
}

void ZooidManager::setWorldDimensions(float width, float height)
{
    if(width > 0.0f && height >0.0f)
    {
        worldDimensions.setX(width);
        worldDimensions.setY(height);
    }
}

void ZooidManager::loadParameters()
{
    setWorldDimensions(1.460f, 0.914f);
}

void ZooidManager::saveParameters()
{
    //预留
}

void ZooidManager::managerTimerRun()
{
    //更新机器人位置
    zooidPosUpdate();
}

void ZooidManager::managerThreadRun(){
    uint64_t previousTimestep = steadyNowMs();

    while (updating.load())
    {
        processReceiversData();
        const uint64_t nowMs = steadyNowMs();
        const uint64_t elapsedTime = nowMs - previousTimestep;

        if (elapsedTime >= SYSTEM_UPDATE_PERIOD)
        {
#if !DEBUG_MODE_SOFTWARA
            onlineZooidUpdate();
#endif
            serviceTestMode(nowMs);
            previousTimestep = nowMs;

            unique_lock<mutex> lock(valuesMutex);
            {
                currentUpdatePeriod = static_cast<float>(elapsedTime);
            }
        }
        else
        {
            Sleep(1);
        }
    }
}

uint64_t ZooidManager::steadyNowMs() const
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

bool ZooidManager::startTestMode()
{
    unique_lock<mutex> lock(testModeMutex);
    if (testModeStatus == TestModeStatus::StartPending ||
        testModeStatus == TestModeStatus::Running ||
        zeroStopCyclesRemaining > 0)
    {
        return false;
    }

    testMode.stop();
    testTargets.clear();
    testStopIds.clear();
    testLostIds.clear();
    testStopRequested = false;
    testModeStatus = TestModeStatus::StartPending;
    return true;
}

void ZooidManager::stopTestMode()
{
    unique_lock<mutex> lock(testModeMutex);
    if (zeroStopCyclesRemaining > 0)
        return;
    testStopRequested = true;
}

TestModeStatus ZooidManager::getTestModeStatus() const
{
    unique_lock<mutex> lock(testModeMutex);
    return testModeStatus;
}

std::vector<unsigned int> ZooidManager::getTestModeLostRobotIds() const
{
    unique_lock<mutex> lock(testModeMutex);
    return testLostIds;
}

PursuitStatusSnapshot ZooidManager::getTestModeSnapshot() const
{
    unique_lock<mutex> lock(testModeMutex);
    return testMode.statusSnapshot();
}

std::vector<unsigned int> ZooidManager::snapshotActiveZooidIds()
{
    std::vector<unsigned int> ids;
    unique_lock<mutex> lock(valuesMutex);
    for (Zooid* zooid : myZooids)
    {
        if (zooid != nullptr && zooid->isConnected() && zooid->isActivated())
            ids.push_back(zooid->getId());
    }
    return ids;
}

std::vector<PursuitRobotState> ZooidManager::snapshotTestRobots(uint64_t nowMs)
{
    std::vector<PursuitRobotState> robots;
    unique_lock<mutex> lock(valuesMutex);
    robots.reserve(myZooids.size());
    for (Zooid* zooid : myZooids)
    {
        if (zooid == nullptr)
            continue;
        PursuitRobotState state;
        state.id = zooid->getId();
        state.pose.x = zooid->getPosition().getX();
        state.pose.y = zooid->getPosition().getY();
        state.pose.yaw = receiverHeadingToYaw(zooid->getOrientation());
        const auto stamp = testFeedbackMs.find(state.id);
        state.feedbackMs = stamp == testFeedbackMs.end() ? nowMs - 500 : stamp->second;
        state.connected = zooid->isConnected();
        state.activated = zooid->isActivated();
        robots.push_back(state);
    }
    return robots;
}

void ZooidManager::sendTestCommand(const std::vector<unsigned int>& ids, WheelCommand command)
{
    for (unsigned int id : ids)
    {
        QColor color(Qt::white);
        {
            unique_lock<mutex> lock(valuesMutex);
            auto zooid = find_if(myZooids.begin(), myZooids.end(), [id](Zooid* value) {
                return value != nullptr && value->getId() == id;
            });
            if (zooid != myZooids.end())
                color = (*zooid)->getColor();
        }
        controlRobotSpeed(static_cast<int>(id), command.left, command.right, color);
    }
}

void ZooidManager::sendTestCommands(const std::map<unsigned int, WheelCommand>& commands)
{
    for (const auto& entry : commands)
        sendTestCommand({entry.first}, entry.second);
}

void ZooidManager::flushReceiversForIds(const std::vector<unsigned int>& ids)
{
    std::set<int> receiverIds;
    for (unsigned int id : ids)
        receiverIds.insert(static_cast<int>(id / NUM_ZOOIDS_PER_RECEIVER));

    for (ZooidReceiver* receiver : myReceivers)
    {
        if (receiver != nullptr && receiver->isInitialized() &&
            receiverIds.count(receiver->getId()) != 0)
        {
            receiver->setReadyToSend();
        }
    }
}

void ZooidManager::clearReceiverQueuesForIds(const std::vector<unsigned int>& ids)
{
    std::set<int> receiverIds;
    for (unsigned int id : ids)
        receiverIds.insert(static_cast<int>(id / NUM_ZOOIDS_PER_RECEIVER));

    for (ZooidReceiver* receiver : myReceivers)
    {
        if (receiver != nullptr && receiverIds.count(receiver->getId()) != 0)
            receiver->clearPendingOutput();
    }
}

bool ZooidManager::anyReceiverWriteFailed()
{
    for (ZooidReceiver* receiver : myReceivers)
    {
        if (receiver != nullptr && receiver->consumeWriteFailure())
            return true;
    }
    return false;
}

void ZooidManager::beginSafeTestStop(TestModeStatus finalStatus)
{
    unique_lock<mutex> lock(testModeMutex);
    if (zeroStopCyclesRemaining > 0)
    {
        if (finalStatus == TestModeStatus::ReceiverError)
        {
            pendingStopStatus = finalStatus;
            testModeStatus = finalStatus;
        }
        return;
    }

    if (finalStatus == TestModeStatus::Stopped)
        testMode.stop();
    pendingStopStatus = finalStatus;
    testModeStatus = finalStatus;
    testStopRequested = false;

    testStopIds = testTargets.activeIds();
    testStopIds.insert(testStopIds.end(), testTargets.lostIds().begin(), testTargets.lostIds().end());
    std::sort(testStopIds.begin(), testStopIds.end());
    testStopIds.erase(std::unique(testStopIds.begin(), testStopIds.end()), testStopIds.end());
    zeroStopCyclesRemaining = testStopIds.empty() ? 0 : 3;

    if (!testStopIds.empty())
        clearReceiverQueuesForIds(testStopIds);

    if (zeroStopCyclesRemaining == 0)
        testModeStoppedCond.notify_all();
}

void ZooidManager::serviceZeroStopBurst()
{
    std::vector<unsigned int> ids;
    {
        unique_lock<mutex> lock(testModeMutex);
        if (zeroStopCyclesRemaining <= 0)
            return;

        ids = testStopIds;
    }

    sendTestCommand(ids, {0, 0});
    flushReceiversForIds(ids);

    bool finished = false;
    {
        unique_lock<mutex> lock(testModeMutex);
        if (zeroStopCyclesRemaining > 0)
            --zeroStopCyclesRemaining;
        finished = zeroStopCyclesRemaining == 0;
    }

    if (finished)
    {
        unique_lock<mutex> lock(testModeMutex);
        testTargets.clear();
        testStopIds.clear();
        testModeStatus = pendingStopStatus;
        testModeStoppedCond.notify_all();
    }
}

void ZooidManager::serviceTestMode(uint64_t nowMs)
{
    bool sessionActive = false;
    {
        unique_lock<mutex> lock(testModeMutex);
        sessionActive = testModeStatus == TestModeStatus::StartPending ||
                        testModeStatus == TestModeStatus::Running ||
                        zeroStopCyclesRemaining > 0;
    }
    const bool receiverFailed = anyReceiverWriteFailed();
    if (receiverFailed && sessionActive)
    {
        beginSafeTestStop(TestModeStatus::ReceiverError);
        serviceZeroStopBurst();
        return;
    }

    bool stopRequested = false;
    {
        unique_lock<mutex> lock(testModeMutex);
        stopRequested = testStopRequested;
    }

    if (stopRequested)
    {
        bool needsSnapshot = false;
        {
            unique_lock<mutex> lock(testModeMutex);
            needsSnapshot = testTargets.activeIds().empty() && testTargets.lostIds().empty();
        }
        if (needsSnapshot)
        {
            const std::vector<unsigned int> ids = snapshotActiveZooidIds();
            unique_lock<mutex> lock(testModeMutex);
            testTargets.startSnapshot(ids);
        }
        beginSafeTestStop(TestModeStatus::Stopped);
        serviceZeroStopBurst();
        return;
    }

    {
        unique_lock<mutex> lock(testModeMutex);
        if (zeroStopCyclesRemaining > 0)
        {
            lock.unlock();
            serviceZeroStopBurst();
            return;
        }
    }

    bool startPending = false;
    {
        unique_lock<mutex> lock(testModeMutex);
        startPending = testModeStatus == TestModeStatus::StartPending;
    }

    if (startPending)
    {
        const std::vector<PursuitRobotState> robots = snapshotTestRobots(nowMs);
        unique_lock<mutex> lock(testModeMutex);
        if (testModeStatus == TestModeStatus::StartPending)
        {
            if (!testMode.start(robots, nowMs))
            {
                testModeStatus = TestModeStatus::NoActiveRobots;
                testModeStoppedCond.notify_all();
                return;
            }
            testTargets.startSnapshot(testMode.roleMap().participantIds());
            testLostIds.clear();
            testSequence = 0;
            testModeStatus = TestModeStatus::Running;
        }
    }

    {
        unique_lock<mutex> lock(testModeMutex);
        if (testModeStatus != TestModeStatus::Running)
            return;
    }

    const std::vector<PursuitRobotState> robots = snapshotTestRobots(nowMs);
    PursuitControlOutput output;
    {
        unique_lock<mutex> lock(testModeMutex);
        if (testStopRequested || testModeStatus != TestModeStatus::Running)
        {
            lock.unlock();
            beginSafeTestStop(TestModeStatus::Stopped);
            serviceZeroStopBurst();
            return;
        }
        output = testMode.update(robots, nowMs, getWorldWidth(), getWorldHeight(), ++testSequence);
        testLostIds.clear();
        const std::vector<unsigned int> participants = testMode.roleMap().participantIds();
        for (unsigned int id : participants)
        {
            const auto found = std::find_if(robots.begin(), robots.end(), [id](const PursuitRobotState& robot) {
                return robot.id == id && robot.connected && robot.activated;
            });
            if (found == robots.end()) testLostIds.push_back(id);
        }
    }

    TestModeStatus stopReason = TestModeStatus::Idle;
    switch (output.fault)
    {
    case PursuitFault::None: break;
    case PursuitFault::ParticipantMissing: stopReason = TestModeStatus::AllTargetsLost; break;
    case PursuitFault::FeedbackStale: stopReason = TestModeStatus::FeedbackStale; break;
    case PursuitFault::InvalidGeometry: stopReason = TestModeStatus::InvalidGeometry; break;
    case PursuitFault::ReceiverError: stopReason = TestModeStatus::ReceiverError; break;
    default: stopReason = TestModeStatus::InvalidFeedback; break;
    }
    if (output.phase == PursuitPhase::Captured && output.fault == PursuitFault::None)
        stopReason = TestModeStatus::Completed;

    std::vector<unsigned int> commandIds;
    for (const auto& entry : output.commands) commandIds.push_back(entry.first);
    {
        unique_lock<mutex> lock(testModeMutex);
        testTargets.recordCommanded(commandIds);
    }
    sendTestCommands(output.commands);
    flushReceiversForIds(commandIds);

    if (stopReason != TestModeStatus::Idle)
    {
        beginSafeTestStop(stopReason);
        serviceZeroStopBurst();
    }
}

//检测机器人是否在线
void ZooidManager::onlineZooidUpdate()
{
    //删除掉线
    for(int i=0; i<myZooids.size(); )
    {
        if(!myZooids[i]->isConnected())
        {
            removeZooid(myZooids[i]->getId());
        }
        else
        {
            i++;
        }
    }

    //上线注册
    for(int i=0; i<zooidRegisterBuff.size(); i++)
    {
        addZooid(zooidRegisterBuff[i].id,zooidRegisterBuff[i].position );
    }
    if(zooidRegisterBuff.size())
        zooidRegisterBuff.clear();
}

void ZooidManager::zooidPosUpdate()
{
    unique_lock<mutex> lock(valuesMutex);
    //位置刷新
    int myZooidsSize = static_cast<int>(myZooids.size());
    for(int i=0; i<myZooidsSize; i++)
    {
        //将获取的原始坐标进行转化
        myZooids[i]->setPos(myZooids[i]->getPosition().getX() * 1000 - 35, (getWorldHeight() - myZooids[i]->getPosition().getY()) * 1000 - 35);
        myGoals[i]->setPos(myGoals[i]->getPosition().getX() * 1000 - 35, (getWorldHeight() - myGoals[i]->getPosition().getY()) * 1000 - 35);
    }
#if AGENT_DEBUG
    //更新代理调试机器人位置
    //agentZooid->setPos(agentZooid->getPosition().getX() * 1000 -35, agentZooid->getPosition().getY() * 1000 -35);
#endif
}

void ZooidManager::chargeUpdate()
{
    //遍历所有机器人的充电状态
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        //必须加这个判断, 否则机器人会跑到其他充电桩
        if (lzm::absSq(myZooids[i]->getPosition() - myGoals[i]->getPosition()) < GoalRadius * GoalRadius)
        {
            // 防止正在表演的机器人路过第一位置进入充电桩
            if(myZooids[i]->getCharge() == 0)
            {
                continue;
            }

            if(myZooids[i]->getCharge() == 1)
            {
               //根据机器人当前位置找到距离附近的充电桩(第一位置)
               int zooidChargeId = zooidCharge->getNearFirstChargeId(myZooids[i]->getPosition());
               // 机器人周围有id的充电桩第一位置
               if(zooidChargeId != -1){
                   // 设置当前充电桩被使用
                   if(zooidCharge->chargeMsg[zooidChargeId].has == false){  //充电桩空闲
                       zooidCharge->setChargeZooid(zooidChargeId);   //占用此充电桩
                       // 让机器人去第二位置（充电桩位置）
                       moveGoal(getZooidIndex(myZooids[i]->getId()), zooidCharge->getSecondPos(zooidChargeId));
                   }
                   else{
                       myZooids[i]->setCharge(0);
                       //如果该充电桩被占用了就先继续寻找下一个充电桩
                   }

               }

                //检查是否到达充电位置
               zooidChargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
               if(zooidChargeId != -1){
                    myZooids[i]->setCharge(2);
                    // 设置当前充电桩被使用
                    zooidCharge->setChargeZooid(zooidChargeId);
               }
            }
        }
    }
}

void ZooidManager::setSimulationMode(SimulationMode mode)
{
    unique_lock<mutex> lock(valuesMutex);
    {
        simulationMode = mode;
    }
    lock.unlock();
}

SimulationMode ZooidManager::getSimulationMode()
{
    SimulationMode mode;

    unique_lock<mutex> lock(valuesMutex);
    {
        mode = simulationMode;
    }
    return mode;
}

void ZooidManager::setAssignmentMode(AssignmentMode mode) {
    unique_lock<mutex> lock(valuesMutex);
    {
        assignmentMode = mode;
    }
    lock.unlock();
    assignRobots();
}

AssignmentMode ZooidManager::getAssignmentMode()
{
    AssignmentMode mode;

    unique_lock<mutex> lock(valuesMutex);
    {
        mode = assignmentMode;
    }
    lock.unlock();

    return mode;
}

void ZooidManager::setPlanningMode(PlanningMode mode, int nb){
    unique_lock<mutex> lock(valuesMutex);
    {
        planningMode = mode;
        nbUseZooid = nb;
    }
    lock.unlock();
}

void ZooidManager::setDrawPathPoints(vector<Vector2> _drawPathPoints)
{
    drawPathPoints = _drawPathPoints;
}

PlanningMode ZooidManager::getPlanningMode(){

    PlanningMode mode;

    unique_lock<mutex> lock(valuesMutex);
    {
        mode = planningMode;
    }

    return mode;
}

//充电
ErrorCode ZooidManager::generateCharge()
{
    unsigned int nbZooid = static_cast<unsigned int>(myZooids.size());

    if(!nbZooid)
    {
        return ZooidNullError;
    }

    //获取到所有空闲充电桩第一位置

    for(unsigned int i = 0; i< nbZooid ; i++)
    {
        //判断当前机器人是否在充电桩(第二位置)，在则返回充电桩Id，否则返回-1
        int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
        // 周围没有充电桩
        if(chargeId == -1){
            //设置不在充电桩的机器人位置
            myZooids[i]->setCharge(1);
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//三角形编队
ErrorCode ZooidManager:: generateTriangle()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 3)
    {
        return NumberError;
    }

    if(zooidNumber >= 3 && zooidNumber < 6){
        zooidNumber = 3;
    }else if(zooidNumber >= 6 && zooidNumber < 10){
        zooidNumber = 6;
    }else {
        zooidNumber = 10;
    }

    //三角稀疏度
    float k = 1.40;

    //计算阶数
    int Order = 1;
    while(zooidNumber >  ((1 + Order) * Order) / 2)
    {
        Order++;
    }

    //定义杨辉三角
    Vector2 yanghui[20][20];
    yanghui[0][0] = Vector2(getWorldWidth() / 2.0f, (getWorldHeight() - Order * ROBOT_DIAMETER * (k + 0.3) +ROBOT_DIAMETER) / 2 );

    //对角线处理
    for(int i=1; i<= Order; i++){
        yanghui[i][i].setX(yanghui[i-1][i-1].getX() + ROBOT_DIAMETER * k);
        yanghui[i][i].setY(yanghui[i-1][i-1].getY() + ROBOT_DIAMETER * (k + 0.3));
    }

    //计算杨辉三角
    for(int i=1; i <Order; i++)
    {
        for(int j=0; j < i; j++)
        {
            yanghui[i][j].setX(yanghui[i-1][j].getX() - ROBOT_DIAMETER * k);
            yanghui[i][j].setY(yanghui[i-1][j].getY() + ROBOT_DIAMETER * (k + 0.3));
        }
    }

    //定义生成结果
    vector<Vector2> result;

    //首先添加边
    for(int i=0; i<Order; i++)
    {
        for(int j=0; j<Order; j++)
        {
            if(j == 0 || i == Order -1 || i == j){
                result.push_back(yanghui[i][j]);
            }
        }
    }

    //添加内部点
    for(int i=1; i<Order-1; i++)
    {
        for(int j=1; j<i; j++)
        {
            result.push_back(yanghui[i][j]);
        }
    }

    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否在表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                // 已经在充电桩
                myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//矩形编队
ErrorCode ZooidManager:: generateReact()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 4)
    {
        return NumberError;
    }

    if(zooidNumber >= 4 && zooidNumber < 9){
        zooidNumber = 4;
    }else if(zooidNumber >= 9 && zooidNumber < 16){
        zooidNumber = 9;
    }else {
        zooidNumber = 16;
    }

    //矩形稀疏度
    float k = 1.35;

    //计算阶数
    int Order = static_cast<int>(std::sqrt(zooidNumber));
    if(Order * Order < zooidNumber){
        Order ++;
    }

    //计算矩形长宽
    int edgea,edgeb;
    edgea = edgeb = Order;

    if(Order * (Order- 1) >= zooidNumber){
        edgeb --;
    }

    //定义矩形
    Vector2 react[10][10];

    float aLen =  ROBOT_DIAMETER * (k + 1) * (edgea - 1);
    float bLen =  ROBOT_DIAMETER * (k + 1) * (edgeb - 1);

    Vector2 first((getWorldWidth() - aLen) / 2.0f, (getWorldHeight() - bLen) / 2.0f);

    //计算矩形
    for(int i=0; i<edgea; i++)
    {
        for(int j=0; j<edgeb; j++)
        {
            react[i][j].setX(first.getX() + i * ROBOT_DIAMETER * (k + 1));
            react[i][j].setY(first.getY() + j * ROBOT_DIAMETER * (k + 1));
        }
    }

    //生成结果
    vector<Vector2> result;

    //首先添加边
    for(int i=0; i<edgea; i++)
    {
        for(int j=0; j<edgeb; j++)
        {
            if(i ==0 || j ==0 || i==edgea-1 || j==edgeb-1)
            {
                result.push_back(react[i][j]);
            }
        }
    }

    //添加矩形内部点
    for(int i=1; i<edgea-1; i++){
        for(int j=1; j<edgeb-1; j++)
        {
            result.push_back(react[i][j]);
        }
    }

    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//圆形编队
ErrorCode ZooidManager:: generateCircul()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 6)
    {
        return NumberError;
    }
    //圆形稀疏度
    float k = 1.0f;
    if(zooidNumber >= 6 && zooidNumber < 9){
        zooidNumber = 6;
        k = 1.2f;
    }else if(zooidNumber >= 10 && zooidNumber < 16){
        zooidNumber = 9;
        k = 0.9f;
    }else {
        zooidNumber = 16;
    }

    vector<Vector2> result;

    int c = 1;          //每圈的个数(周长)
    qreal r = 0.12 * k; //每圈的半径
    int i=0;
    while(++i)
    {
        if(result.size() >= zooidNumber)
        {
            break;
        }
        //剩余的Zooid
        int sum = zooidNumber - (int)result.size();
        int need = c;
        int remain = zooidNumber- (int)result.size() - c;
        if(remain > 0)
        {
            if(remain <= need )
            {
                c = sum;
                if(zooidNumber < 20)//这里的20是测试出来的
                {
                    r += 0.1 * k;
                }else
                {
                    c = sum / 2;
                }
            }
        }else
        {
            c = sum;
        }

        //计算弧度
        qreal radians = (PI/180) * (360 / c);

        for(int j=0; j < c; j++)
        {
            float theta = radians * j;
            qreal x = getWorldWidth() / 2.0f + r * (i-1) * cos(theta);
            qreal y = getWorldHeight() / 2.0f +r * (i-1) * sin(theta);
            result.push_back(Vector2(x,y));
        }
        c = (2 * PI * r * i) / (ROBOT_DIAMETER * 2.3 * k);
    }


    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                 myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//五角星编队
ErrorCode ZooidManager::genrateFivepointed()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 10)
    {
        return NumberError;
    }
    zooidNumber = 10;

    vector<Vector2> result;

    for(int i=0; i < 5; i++)
    {
        float theta =  i * (PI / 180 * 72) + 0.3;

        qreal x = getWorldWidth() / 2.0f + cos(theta) * 0.13;
        qreal y = getWorldHeight() / 2.0f + sin(theta) * 0.13;

        result.push_back(Vector2(x,y));
    }

    for(int i=0; i < 5; i++)
    {
        float theta =  i * (PI / 180 * 72) + 0.94;

        qreal x = getWorldWidth() / 2.0f + cos(theta) * 0.25;
        qreal y = getWorldHeight() / 2.0f + 0.01 + sin(theta) * 0.25;
        result.push_back(Vector2(x,y));
    }

    //更新位置
    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//六边形
ErrorCode ZooidManager::genrateHexagon()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 6)
    {
        return NumberError;
    }
    zooidNumber = 6;
    //矩形稀疏度
    float k = 2.0;

    vector<Vector2> result;

    for(int i=0; i < 6; i++)
    {
        float theta =  i * PI / 3 ;

        qreal x = getWorldWidth() / 2.0f + cos(theta) * 0.2;
        qreal y = getWorldHeight() / 2.0f + sin(theta) * 0.2;

        result.push_back(Vector2(x,y));
    }

    //更新位置
    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }
    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();


    return NotError;
}
//十字
ErrorCode ZooidManager::genrateCross()
{
    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber < 5 )
    {
        return NumberError;
    }
    //矩形稀疏度
    float k = 1.5;

    if(zooidNumber >= 5 && zooidNumber < 9){
        zooidNumber = 5;
        k =2.2;
    }else if(zooidNumber >= 9 && zooidNumber < 13){
        zooidNumber = 9;
    }else {
        zooidNumber = 13;
    }


    int len = 0;
    Vector2 conterPoint = Vector2(getWorldWidth() / 2.0f, getWorldHeight()  / 2.0f );

    vector<Vector2> result;
    if(zooidNumber%2)
    {
        result.push_back(conterPoint);
    }

    len = (zooidNumber + 2) / 4;

    float margin = k * ROBOT_DIAMETER;

    for(int i=0; i<len; i++)
    {
        Vector2 lp = Vector2(conterPoint.getX() - margin *(i+1), conterPoint.getY());
        Vector2 rp = Vector2(conterPoint.getX() + margin *(i+1), conterPoint.getY());
        Vector2 tp = Vector2(conterPoint.getX(), conterPoint.getY() - margin *(i+1));
        Vector2 bp = Vector2(conterPoint.getX(), conterPoint.getY() + margin *(i+1));
        result.push_back(lp);
        result.push_back(rp);
        result.push_back(tp);
        result.push_back(bp);
    }

    //更新位置

    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    return NotError;
}
//绘画模式
ErrorCode ZooidManager::generateDrawpath(){

    int pSize = drawPathPoints.size();

    //state defualt full zeros
    bool *state = new bool[pSize];
    for(int i=0; i<pSize; i++)
    {
        state[i] = false;
    }

    //去除相邻Zooid有相交的
    for(int i=0; i<pSize; i++)
    {
        if(state[i])
        {
            continue;
        }
        for(int j=i+1; j<pSize; j++)
        {
            if(lzm::abs(drawPathPoints[i] - drawPathPoints[j]) <= ROBOT_DIAMETER * 2)
            {
                state[j] = true;
            }
        }
    }

    vector<Vector2> result;

    for(int i=0; i<pSize; i++)
    {
        if(!state[i])
        {
            result.push_back(drawPathPoints[i]);
        }
    }

    int zooidNumber = getEnoughBatteryNbZooids();
    if(zooidNumber <result.size()){
        return NumberError;
    }else{
        zooidNumber = result.size();
    }

    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();    //创造一个二维数组队列，储存每个机器人的id和电量
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    int nb = 0;
    // 遍历当前所有机器人
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
            //每确定一个机器人空闲则添加一个目标
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                myZooids[i]->setCharge(1);
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    delete [] state;

    return NotError;
}
//第一次进入跟随模式时进入这个函数
ErrorCode ZooidManager::genrateFollow()
{
    // 获取可表演机器人个数
    int zooidNumber = getEnoughBatteryNbZooids();
    // 没有满足的
    if(zooidNumber < 1){
        return NumberError;
    }else if(zooidNumber >= 9){
        // 最多要9个
        zooidNumber = 9;
    }

    //getFollowPos
    Vector2 centerPosition = zooidFollow->getBatonPosition();

    float DLEN = 3 * ROBOT_RADIUS;
    int dx = 1, dy = 1;
    // no up
    if(centerPosition.getX() - DLEN * 2 <= 0){
        dy = 0;
    }

    // no down
    if(centerPosition.getX() + DLEN * 2 >= getWorldWidth()){
        dy = 2;
    }

    // no left
    if(centerPosition.getY() - DLEN * 2 <= 0){
        dx = 0;
    }

    // no right
    if(centerPosition.getY() + DLEN * 2  >= getWorldHeight()){
        dx = 2;
    }

    Vector2 pos[3][3];
    // 生成绝对坐标
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            pos[i][j].setX(j * DLEN);
            pos[i][j].setY(i * DLEN);
        }
    }

    // 计算相对坐标
    Vector2 dDistance = centerPosition - pos[dx][dy];

    // 坐标转换
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            pos[i][j] = pos[i][j] + dDistance;
        }
    }

    // 计算pos中与centerPosition 点距离最近的5个点就是跟随的位置
    vector<Vector2> result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            bool isInsert = false;
            for(int k=0; k<result.size(); k++){
                // 比较距离差
                if (lzm::abs(pos[i][j] - pos[dx][dy]) <= lzm::abs(result[k] - pos[dx][dy])){
                    result.insert(result.begin()+k,pos[i][j]);
                    isInsert = true;
                    break;
                }
            }
            if(!isInsert){
                result.push_back(pos[i][j]);
            }
        }
    }
    int x = pfMap(pos[dx][dy].getX(),0.0f,getWorldWidth() ,0, 1460 );
    int y = pfMap(pos[dx][dy].getY(),0.0f,getWorldHeight(), 0, 914);
    zooidSimulator->setFollowImagePos(x, y);

    // 筛选电量最高的
    unsigned int highBatteryQueue[100][2];
    int count = 0;
    memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
    for(unsigned int i=0; i< myZooids.size(); i++){

        if(count < zooidNumber){
             highBatteryQueue[count][0] = myZooids[i]->getId();
             highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
             count++;
        }else{
            unsigned int id = myZooids[i]->getId();
            unsigned int battery = myZooids[i]->getBatteryLevel();
            unsigned int minBattery = 9999;
            int index = -1;
            for(int j=0; j<zooidNumber; j++){
                if(highBatteryQueue[j][1] < minBattery){
                    minBattery = highBatteryQueue[j][1];
                    index = j;
                }
            }

            if(index != -1 && battery > highBatteryQueue[index][1]){
                highBatteryQueue[index][0] = id;
                highBatteryQueue[index][1] = battery;
            }
        }
    }

    // 保存使用的ids
    vector <unsigned int>ids;
    for(unsigned int i=0; i< zooidNumber; i++){
        ids.push_back(highBatteryQueue[i][0]);
    }
    zooidFollow->setZooids(ids);

    int nb = 0;
    // 遍历当前所有机器人
    vector<Vector2>freeCharge = getFreeChargeFirstPosition();
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        // 判断当前机器人是否再表演队列中
        bool isHas = false;
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == highBatteryQueue[j][0]){
                isHas = true;
                break;
            }
        }

        if(isHas){
            //可以继续表演， 分配图案
            myZooids[i]->setCharge(0);
            moveGoal(i,result[nb++]);
        }
        else{
            // 表演区里的低电量
            int chargeId = zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition());
            // 判断当前机器人不用表演时候的位置
            if(chargeId == -1){
                // 回去充电桩
                // 由于跟随模式方案实时计算，防止在充电桩第一位置抖动
                if(myZooids[i]->getCharge() != 1){
                    myZooids[i]->setCharge(1);
                }
            }else{
                myZooids[i]->setCharge(2);
            }
        }
    }
    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();
    return NotError;
}

// Voronoi 覆盖控制方案初始化
ErrorCode ZooidManager::genrateVoronoi()
{
        // 获取可表演机器人个数
        int zooidNumber = getEnoughBatteryNbZooids();
        if (zooidNumber < 1) {
            return NumberError;
        }
        if (zooidNumber >= 9) {
            zooidNumber = 9;
        }

        // 筛选电量最高的机器人
        unsigned int highBatteryQueue[100][2];
        int count = 0;
        memset(highBatteryQueue, 0, sizeof(highBatteryQueue));
        for (unsigned int i = 0; i < myZooids.size(); i++) {
            if (count < zooidNumber) {
                highBatteryQueue[count][0] = myZooids[i]->getId();
                highBatteryQueue[count][1] = myZooids[i]->getBatteryLevel();
                count++;
            } else {
                unsigned int id = myZooids[i]->getId();
                unsigned int battery = myZooids[i]->getBatteryLevel();
                unsigned int minBattery = 9999;
                int index = -1;
                for (int j = 0; j < zooidNumber; j++) {
                    if (highBatteryQueue[j][1] < minBattery) {
                        minBattery = highBatteryQueue[j][1];
                        index = j;
                    }
                }
                if (index != -1 && battery > highBatteryQueue[index][1]) {
                    highBatteryQueue[index][0] = id;
                    highBatteryQueue[index][1] = battery;
                }
            }
        }

        // 保存 robot ids 并构建 voronoiActiveZooids 列表
        voronoiActiveZooids.clear();
        vector<unsigned int> ids;
        for (unsigned int i = 0; i < zooidNumber; i++) {
            ids.push_back(highBatteryQueue[i][0]);
        }
        zooidFollow->setZooids(ids);

        // 设置所有机器人为非充电状态，并填充 voronoiActiveZooids
        for (unsigned int i = 0; i < myZooids.size(); i++) {
            bool isHas = false;
            for (int j = 0; j < zooidNumber; j++) {
                if (myZooids[i]->getId() == highBatteryQueue[j][0]) {
                    isHas = true;
                    break;
                }
            }
            if (isHas) {
                myZooids[i]->setCharge(0);
                voronoiActiveZooids.push_back(myZooids[i]);
            }
        }

        assignCharge();
        setAssignmentMode(OptimalAssignment);
        assignRobots();

    return NotError;
}

void ZooidManager::setObstacleEnabled(bool enabled)
{
    if (voronoiController) {
        voronoiController->setObstacleEnabled(enabled);
        qDebug() << "ZooidManager: obstacle" << (enabled ? "enabled" : "disabled");
    }
}

void ZooidManager::setObstacleStatic()
{
    if (voronoiController) {
        voronoiController->setObstacleSpeedZero();
    }
}

void ZooidManager::setDualObstacleMode(bool enabled)
{
    if (voronoiController) {
        voronoiController->setDualObstacleMode(enabled);
        voronoiController->setFormationMode(enabled);     // 复用编队逻辑
        if (enabled) voronoiController->setObstacleEnabled(true);
        qDebug() << "ZooidManager: dual obstacle mode" << (enabled ? "enabled" : "disabled");
    }
}

void ZooidManager::setPushWaveRobotMode(bool enabled)
{
    if (voronoiController) {
        voronoiController->setPushWaveRobotMode(enabled);
    }
}

void ZooidManager::setFormationMode(bool enabled)
{
    if (voronoiController) {
        voronoiController->setFormationMode(enabled);
        if (enabled) voronoiController->setObstacleEnabled(true);  // 编队模式需要OAVC
        // 注意：false时不关闭obstacle，让其他模式自行管理
        qDebug() << "ZooidManager: formation mode" << (enabled ? "enabled" : "disabled");
    }
}

void ZooidManager::startVoronoiMode()
{
    if (myZooids.empty()) {
        qWarning() << "No zooids available for Voronoi mode";
        return;
    }

    voronoiRunning = true;
    voronoiTime = 0.0f;
    voronoiController->reset();

    qDebug() << "startVoronoiMode: voronoiRunning=" << voronoiRunning
             << ", activeZooids.size()=" << voronoiActiveZooids.size();

    // 初始化 Voronoi 可视化（每次都重新创建，避免问题）
    if (voronoiViewBox != nullptr) {
        delete voronoiViewBox;
        voronoiViewBox = nullptr;
    }
    voronoiViewBox = new LVoronoiViewBox();
    voronoiViewBox->setFieldBounds(0, 0, getWorldWidth(), getWorldHeight());
    voronoiViewBox->setScale(600.0f);
    voronoiViewBox->setWindowFlags(Qt::Window);
    voronoiViewBox->setWindowTitle("Voronoi Coverage Control Visualization");
    voronoiViewBox->resize(800, 500);
    voronoiViewBox->show();
    voronoiViewBox->raise();

    // 创建定时器直接更新 Voronoi 显示（避免跨线程信号槽问题）
    // 延迟 200ms 启动，等待窗口初始化完成
    if (voronoiUpdateTimer) {
        voronoiUpdateTimer->stop();
        delete voronoiUpdateTimer;
    }
    voronoiUpdateTimer = new QTimer(this);
    connect(voronoiUpdateTimer, &QTimer::timeout, this, [this]() {
        static int count = 0;
        this->updateVoronoiDisplay();
    });
    QTimer::singleShot(200, this, [this]() {
        voronoiUpdateTimer->start(100);  // 每 100ms 更新一次
    });
}

void ZooidManager::stopVoronoiMode()
{
    voronoiRunning = false;
    voronoiController->reset();

    // 停止定时器
    if (voronoiUpdateTimer) {
        voronoiUpdateTimer->stop();
    }

    // 关闭可视化窗口
    if (voronoiViewBox) {
        voronoiViewBox->close();
    }

    // 所有活动机器人发送零速
    for (auto* z : voronoiActiveZooids) {
        controlRobotSpeed(z->getId(), 0, 0, z->getColor());
    }

    qDebug() << "Voronoi mode stopped by user";
}

void ZooidManager::stopAllZooids()
{
    const std::vector<unsigned int> ids = snapshotActiveZooidIds();
    sendTestCommand(ids, {0, 0});
    flushReceiversForIds(ids);
}

// 更新 Voronoi 模式（无障碍简化版）
void ZooidManager::updateVoronoi()
{
    if (!voronoiRunning) return;
    if (!voronoiController) {
        qWarning() << "updateVoronoi: voronoiController is null!";
        return;
    }

    // 获取活动机器人列表
    if (voronoiActiveZooids.empty()) {
        qWarning() << "updateVoronoi: no active zooids!";
        return;
    }

    voronoiTime += 0.1f;

    // 执行控制（无障碍物）
    bool shouldStop = voronoiController->computeControlStep(voronoiActiveZooids, voronoiTime);

    // 更新可视化
    if (voronoiViewBox != nullptr) {
        std::vector<VoronoiPolygon> regions;
        std::vector<QColor> colors;
        std::vector<Vec2> positions;
        std::vector<Vec2> centroids;

        for (size_t i = 0; i < voronoiActiveZooids.size(); ++i) {
            VoronoiPolygon poly = voronoiController->getRobotRegion(i);
            regions.push_back(poly);
            colors.push_back(voronoiActiveZooids[i]->getColor());
            positions.push_back(Vec2(voronoiActiveZooids[i]->getPosition().getX(),
                                     voronoiActiveZooids[i]->getPosition().getY()));
            if (!poly.isEmpty()) {
                centroids.push_back(poly.centroid());
            } else {
                centroids.push_back(Vec2(0, 0));
            }
        }

        // 障碍物 & 危险三角形数据（仅在启用时传递）
        std::vector<Vec2> obsVerts;
        std::vector<CoverCircle> coverCircles = voronoiController->getObstacleCoverCircles();
        DangerTriangle danger;
        if (!coverCircles.empty()) {
            if (voronoiController->getDualObstacleMode() || voronoiController->getPushWaveRobotMode()) {
                float h = 0.1f / 2.0f;  // 100mm
                Vec2 rp = voronoiController->getObstacleRobotPos();
                obsVerts = {Vec2(rp.x-h,rp.y-h), Vec2(rp.x+h,rp.y-h), Vec2(rp.x+h,rp.y+h), Vec2(rp.x-h,rp.y+h)};
            } else if (voronoiController->getFormationMode()) {
                obsVerts = voronoiController->getStaticObstacleBoundary();
            } else if (voronoiController->getObstacle().speed < 0.001f) {
                obsVerts = voronoiController->getStaticObstacleBoundary();
            } else {
                obsVerts = voronoiController->getObstacle().getVertices();
            }
            danger = voronoiController->getDangerTriangle();
            if (voronoiController->getDualObstacleMode()) {
                voronoiViewBox->setSecondDanger(voronoiController->getStaticObs2Danger());
                float sh = 0.2f / 2.0f;
                Vec2 sc = Vec2(getWorldWidth()/2.0f, 0.1f);
                voronoiViewBox->setObstacleVerts2({Vec2(sc.x-sh,sc.y-sh),Vec2(sc.x+sh,sc.y-sh),Vec2(sc.x+sh,sc.y+sh),Vec2(sc.x-sh,sc.y+sh)});
            }
        }

        // 编队显示
        if (voronoiController->getFormationMode() || voronoiController->getDualObstacleMode()) {
            float fw, fh;
            voronoiController->getFormationBounds(fw, fh);
            voronoiViewBox->setFormationDisplay(true, voronoiController->getFormationCenter(), fw, fh);
        }

        // 更新显示
        voronoiViewBox->setMetrics(voronoiController->getMinDistToObstacle(),
                                    voronoiController->getAvgRecoveryTime(),
                                    voronoiController->getCoverageVariance());
        voronoiViewBox->pushCostValue(voronoiController->getCostFunction());
        voronoiViewBox->pushMinDistValue(voronoiController->getFrameMinDist());
        voronoiViewBox->pushInterRobotDistValue(voronoiController->getFrameInterRobotMinDist());
        voronoiViewBox->updateVoronoiDisplay(
            regions, colors, positions, centroids,
            obsVerts, coverCircles, danger,
            voronoiController->getCoverageRatio(), voronoiController->getIteration()
        );
    } else {
        qWarning() << "updateVoronoi: voronoiViewBox is null!";
    }

    if (shouldStop) {
        // 导出覆盖率数据
        if (voronoiViewBox) {
            voronoiViewBox->showCoverageChart();
        }
        voronoiRunning = false;
        qDebug() << "Voronoi mode stopped";
        for (auto* z : voronoiActiveZooids) {
            controlRobotSpeed(z->getId(), 0, 0, z->getColor());
        }
        // 停止定时器
        if (voronoiUpdateTimer) {
            voronoiUpdateTimer->stop();
        }
    }
}

// 直接更新 Voronoi 显示（虚拟模式：计算+显示；真实模式：仅刷新显示）
void ZooidManager::updateVoronoiDisplay()
{
    if (!voronoiRunning) return;
    if (!voronoiController) return;
    if (!voronoiViewBox) return;

    if (voronoiActiveZooids.empty()) return;

    // 仅刷新显示（控制计算由 updateVoronoi 在 60Hz 循环中驱动）
    // 收集可视化数据
    std::vector<VoronoiPolygon> regions;
    std::vector<QColor> colors;
    std::vector<Vec2> positions;
    std::vector<Vec2> centroids;

    for (size_t i = 0; i < voronoiActiveZooids.size(); ++i) {
        VoronoiPolygon poly = voronoiController->getRobotRegion(i);
        regions.push_back(poly);
        colors.push_back(voronoiActiveZooids[i]->getColor());
        positions.push_back(Vec2(voronoiActiveZooids[i]->getPosition().getX(),
                                 voronoiActiveZooids[i]->getPosition().getY()));
        if (!poly.isEmpty()) {
            centroids.push_back(poly.centroid());
        } else {
            centroids.push_back(Vec2(0, 0));
        }
    }

    // 更新显示（仅在障碍物启用时传递障碍物数据）
    std::vector<Vec2> obsVerts;
    std::vector<CoverCircle> coverCircles = voronoiController->getObstacleCoverCircles();
    DangerTriangle danger;
    if (!coverCircles.empty()) {
        if (voronoiController->getDualObstacleMode()) {
            // 双障碍物模式：显示动态障碍物（6号机器人）的边界
            float h = 0.1f / 2.0f;  // 100mm障碍物
            Vec2 rp = voronoiController->getObstacleRobotPos();
            obsVerts = {Vec2(rp.x-h,rp.y-h), Vec2(rp.x+h,rp.y-h), Vec2(rp.x+h,rp.y+h), Vec2(rp.x-h,rp.y+h)};
        } else if (voronoiController->getFormationMode()) {
            obsVerts = voronoiController->getStaticObstacleBoundary();
        } else {
            obsVerts = voronoiController->getObstacle().getVertices();
        }
        danger = voronoiController->getDangerTriangle();
    }
    // 编队显示
    if (voronoiController->getFormationMode() || voronoiController->getDualObstacleMode()) {
        float fw, fh;
        voronoiController->getFormationBounds(fw, fh);
        voronoiViewBox->setFormationDisplay(true, voronoiController->getFormationCenter(), fw, fh);
    }

    voronoiViewBox->updateVoronoiDisplay(
        regions, colors, positions, centroids,
        obsVerts, coverCircles, danger,
        0.0f, voronoiController->getIteration()
    );

    // 无时间限制，持续运行
}

// 旋转到指定角度（0-360°）
void ZooidManager::rotateToAngle(Zooid *z,float targetAngle, float currentAngle)
{
    static float lastTargetAngle = 0.0f;

    // 如果目标角度变化，重新初始化PID
    if (abs(targetAngle - lastTargetAngle) > 0.1f)
    {
        z->rotationPID.target = targetAngle;
        z->rotationPID.integral = 0.0f;
        z->rotationPID.prevError = 0.0f;
        z->rotationPID.prevTime = clock();
        lastTargetAngle = targetAngle;
    }

    // 计算角度误差（绝对值）
    float error = targetAngle - currentAngle;
    // 归一化误差到[-180, 180]
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }

    // 计算PID输出
    float pidOutput = z->rotationPID.compute(currentAngle);

    // 限制输出速度范围
    pidOutput = constrain(pidOutput, -500.0f, 500.0f);

    // 应用旋转：左右轮反向运动
    // 如果pidOutput为正，向左旋转；为负，向右旋转
    controlRobotSpeed(z->getId(), pidOutput, -pidOutput, z->getColor());

}

//在zooidfollow的回调函数中被调用，每0.1s调用一次
void ZooidManager::updateFollow()
{
    // 获取可表演机器人个数
    vector<unsigned int> zooids = zooidFollow->getZooids();
    unsigned int zooidNumber = zooids.size();

    //getFollowPos
    Vector2 centerPosition = zooidFollow->getBatonPosition();

    float DLEN = 3 * ROBOT_RADIUS;
    int dx = 1, dy = 1;
    // no up
    if(centerPosition.getX() - DLEN * 2 <= 0){
        dy = 0;
    }

    // no down
    if(centerPosition.getX() + DLEN * 2 >= getWorldWidth()){
        dy = 2;
    }

    // no left
    if(centerPosition.getY() - DLEN * 2 <= 0){
        dx = 0;
    }

    // no right
    if(centerPosition.getY() + DLEN * 2  >= getWorldHeight()){
        dx = 2;
    }

    Vector2 pos[3][3];
    // 生成绝对坐标
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            pos[i][j].setX(j * DLEN);
            pos[i][j].setY(i * DLEN);
        }
    }

    // 计算相对坐标
    Vector2 dDistance = centerPosition - pos[dx][dy];

    // 坐标转换
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            pos[i][j] = pos[i][j] + dDistance;
        }
    }

    // 计算pos中与centerPosition 点距离最近的5个点就是跟随的位置
    vector<Vector2> result;
    for(int i=0; i<3; i++){
        for(int j=0; j<3; j++){
            bool isInsert = false;
            for(int k=0; k<result.size(); k++){
                // 比较距离差
                if (lzm::abs(pos[i][j] - pos[dx][dy]) <= lzm::abs(result[k] - pos[dx][dy])){
                    result.insert(result.begin()+k,pos[i][j]);
                    isInsert = true;
                    break;
                }
            }
            if(!isInsert){
                result.push_back(pos[i][j]);
            }
        }
    }
    int x = pfMap(pos[dx][dy].getX(),0.0f,getWorldWidth() ,0, 1460 );
    int y = pfMap(pos[dx][dy].getY(),0.0f,getWorldHeight(), 0, 914);
    zooidSimulator->setFollowImagePos(x, y);

    int nb = 0;
    // 遍历当前所有机器人
    vector<Vector2>freeCharge = getFreeChargeFirstPosition();
    for(unsigned int i=0; i<myZooids.size(); i++)
    {

        if(myZooids[i]->getRobotControlMode() == Zooid::SpeedControl)
        {
//            controlRobotSpeed(myZooids[i]->getId(), 2500, 2500, myZooids[i]->getColor());
            continue;
            //少一个表演中的机器人就少一个目标
        }

        // 判断当前机器人是否在表演队列中
        for(int j=0; j<zooidNumber; j++){
            if(myZooids[i]->getId() == zooids[j]){
                myZooids[i]->setCharge(0);
                moveGoal(i,result[nb++]);
                break;
            }
        }
    }

    assignCharge();
    setAssignmentMode(OptimalAssignment);
    assignRobots();

    // 以下为算法接口
    // 外轮速度 / 内轮速度 = (R + L/2) / (R - L/2)
    // 轮距 = 50mm
    float wheelDistance = 50.0f;
    float baseSpeed = 500.0f;
    float time = zooidFollow->performTime;

//    //原地转圈
//    rotateToAngle(1, 90, myZooids[0]->getOrientation());

//    if(time <= 5)
//        controlRobotSpeed(myZooids[0]->getId(), 300, 300, myZooids[0]->getColor());
//    else if(time <= 10)
//        controlRobotSpeed(myZooids[0]->getId(), -300, -300, myZooids[0]->getColor());
//    else
//        controlRobotSpeed(myZooids[0]->getId(), 0, 0, myZooids[0]->getColor());

    //转圈模式，设置圆的半径
    float circleRadius = 100.0f; // 毫米
    if(time <= 10)
    {
        // 画半圆
        float leftSpeed = baseSpeed;
        float rightSpeed = baseSpeed * ((circleRadius - wheelDistance/2) / (circleRadius + wheelDistance/2));
        for(unsigned int i=0; i<myZooids.size(); i++)
            controlRobotSpeed(myZooids[i]->getId(), leftSpeed, rightSpeed, myZooids[i]->getColor());
    }
    else if(time <= 20)
    {
        // 画另外半圆
        float leftSpeed = -baseSpeed;
        float rightSpeed = -baseSpeed * ((circleRadius - wheelDistance/2) / (circleRadius + wheelDistance/2));
        for(unsigned int i=0; i<myZooids.size(); i++)
        controlRobotSpeed(myZooids[i]->getId(), leftSpeed, rightSpeed, myZooids[i]->getColor());
    }


//    // 更新障碍物
//    voronoiController->updateObstacle(0.1f);

//    // 执行控制（传入6个机器人的指针）
//    bool shouldStop = voronoiController->computeControlStep(myZooids, time);

//    if (shouldStop)
//    {
//        voronoiRunning = false;
//        // 停止所有机器人
//        for (auto* z : myZooids)
//            controlRobotSpeed(z->getId(), 0, 0, z->getColor());
//   }


}

ErrorCode ZooidManager::runPlanning(){
    switch(planningMode)
    {
        case ChargePlanning:
            return generateCharge();
        case TrianglePlanning:
            return generateTriangle();
        case ReactPlanning:
            return generateReact();
        case CirculPlanning:
            return generateCircul();
        case DrawpathPlanning:
            return generateDrawpath();
        case FollowPlanning:
            return genrateFollow();
        case VoronoiPlanning:
            return genrateVoronoi();
        case CrossPlanning:
            return genrateCross();
        case HexagonPlanning:
            return genrateHexagon();
        case FivepointedPlanning:
            return genrateFivepointed();
        default:
            break;
    }

    return NotHavePlanError;
}

//根据代价矩阵分配机器人目标
void ZooidManager::assignRobots()
{

    if(!myZooids.size())
    {
        return ;
    }

    AssignmentMode mode;
    unique_lock<mutex> lock(valuesMutex);
    {
        mode = assignmentMode;
    }
    lock.unlock();

    //非优分配分配
    if (mode == NaiveAssignment)
    {
        for (unsigned int i = 0; i < myZooids.size(); i++) {
            unique_lock<mutex> lock(valuesMutex);
            {
                zooidSimulator->setAgentGoal(i, i);
                myZooids[i]->setGoalPosition(myGoals[i]->getPosition());;
            }
            lock.unlock();
        }
    }
    else if (mode == OptimalAssignment)
    {
        unsigned long size = min(myGoals.size(), myZooids.size()); //目标数量和机器人数量jian3最小值
        vector<vector<double>> Cost(size, vector<double>(size));
        // 遍历所有机器人
        for (unsigned int i = 0; i < size; i++)
        {
            // 遍历所有位置
            for (unsigned int j = 0; j < size; j++)
            {
               // 计算邻接矩阵
               Cost[i][j] = lzm::absSq(myZooids[i]->getPosition() - myGoals[j]->getPosition()) * 1000.0f;

               if( myZooids[j]->getCharge()){
                   if( i == j){
                        Cost[i][j] /= 1000;
                   }else{
                       Cost[i][j] *= 1000000000.0;
                   }
               }
            }
        }
        vector<int> assignment;
        AssignmentProblemSolver APS;   //匈牙利算法求解
        APS.Solve(Cost, assignment);

        for (unsigned int i = 0; i < size; i++)
        {
            unique_lock<mutex> lock(valuesMutex);
            {
                zooidSimulator->setAgentGoal(i, i);
                myZooids[i]->setGoalPosition(myGoals[assignment[i]]->getPosition());
            }
            lock.unlock();
        }
    }

    //更新目标位置
    for(unsigned int i = 0; i<myZooids.size(); i++)
    {
        myGoals[i]->setPosition(myZooids[i]->getGoalPosition());
    }
}

void ZooidManager::assignCharge()
{
    // 遍历当前所有机器人
    vector<Vector2>freeCharge = getFreeChargeFirstPosition();
    vector<unsigned int> needChargeZooids;

    for(unsigned int i=0; i< myZooids.size(); i++){
        if(myZooids[i]->getCharge() == 1){
            needChargeZooids.push_back(myZooids[i]->getId());
        }
    }

    if(!freeCharge.size() || !needChargeZooids.size()){
        return;
    }

    vector<vector<double>>Cost(needChargeZooids.size(), vector<double>(freeCharge.size()));
    // 遍历所有机器人
    for (unsigned int i = 0; i < needChargeZooids.size(); i++)
    {
        // 定位机器人
        unsigned int index = -1;
        for(unsigned int k=0; k< myZooids.size(); k++){
            if(myZooids[k]->getId() == needChargeZooids[i]){
                index = k;
                break;
            }
        }
        // 遍历所有位置
        for (unsigned int j = 0; j < freeCharge.size(); j++)
        {
            Cost[i][j] = lzm::absSq(myZooids[index]->getPosition() - freeCharge[j]) * 1000.0f;
        }
    }

    vector<int> assignment;
    AssignmentProblemSolver APS;
    APS.Solve(Cost, assignment);

    for (unsigned int i = 0; i < needChargeZooids.size(); i++)
    {
        // 定位机器人
        unsigned int index = -1;
        for(unsigned int k=0;k< myZooids.size(); k++){
            if(myZooids[k]->getId() == needChargeZooids[i]){
                index = k;
                moveGoal(index,freeCharge[assignment[i]]);
                break;
            }
        }
    }
}

bool ZooidManager::runSimulation()
{
    if(!myZooids.size())
    {
        return false;
    }

    SimulationMode mode;
    unique_lock<mutex> lock(valuesMutex);
    {
        mode = simulationMode;
    }

    lock.unlock();
    if (mode == On)
    {
        //更新目标位置
        for (int i = 0; i < myGoals.size(); i++)
        {
            unique_lock<mutex> lock(valuesMutex);
            {
                zooidSimulator->setGoalPosition(i, lzm::Vector2(myGoals[i]->getPosition().getX(), myGoals[i]->getPosition().getY()));
            }
            lock.unlock();
        }

        unique_lock<mutex> lock(valuesMutex);
        {
            zooidSimulator->doStep();
        }
        lock.unlock();

        //设置属性
        for (int i = 0; i < myZooids.size(); i++)
        {
            unique_lock<mutex> lock(valuesMutex);
            {
                lzm::Vector2 distance = zooidSimulator->getAgentPosition(i) - zooidSimulator->getGoalPosition(i);

                float k = pow(abs(distance), 2.0f) * 2000.0f + 0.000001f;
                if (k >= 1.0f)
                {
                    k = 1.0f;
                }

                zooidSimulator->setAgentPrefSpeed(i, prefSpeed * k * (float)myZooids[i]->getSpeed()/100.0f);
                zooidSimulator->setAgentMaxSpeed(i, 1.1f * prefSpeed * k);

                if(myZooids[i]->getState() == 0)
                {
                    //模拟器运动
                    if(myZooids[i]->isActivated() && !myZooids[i]->isConnected())
                    {
                        if(zooidSimulator->getAgentPosition(i).getX() >= 0.0f && zooidSimulator->getAgentPosition(i).getY() >= 0.0f)
                        {
                            myZooids[i]->setPosition(zooidSimulator->getAgentPosition(i).getX(), zooidSimulator->getAgentPosition(i).getY());
                            float angle = zooidSimulator->getAgentOrientation(i) * 180.0f / PI;   //angle是弧度制，值域是R
                            angle = (angle>=0.0f) ? fmod(angle, 360.0f) : fmod(angle, -360.0f);  //限制在-360~0和0~360
                            myZooids[i]->setOrientation(angle);
                            myZooids[i]->setGoalReached(zooidSimulator->getAgentReachedGoal(i));
                        }
                        myZooids[i]->setGoalReached(zooidSimulator->getAgentReachedGoal(i));

                    }
                }
            }
            lock.unlock();
        }

        return true;


    }
    return false;
}

bool ZooidManager::isReachedGoalAll()
{
    if( !myZooids.size())
    {
        return false;
    }
    for (int i = 0; i < myZooids.size(); i++)
    {
        if(!myZooids[i]->isGoalReached())
        {
            return false;
        }
    }
    return true;
}

bool ZooidManager::isChargeZooidAll()
{
    for(unsigned int i=0; i<myZooids.size(); i++)
    {
        if(zooidCharge->getNearSecondChargeId(myZooids[i]->getPosition()) == -1)
        {
            return false;
        }
    }

    return true;
}

void ZooidManager::setBatteryLimit(unsigned int batteryValue)
{
    batteryLimit = batteryValue;
}

void ZooidManager::setShowCount(unsigned int showCountValue)
{
    showCount = showCountValue;
}

unsigned int ZooidManager::getBatteryLimit()
{
    return batteryLimit;
}

unsigned int ZooidManager::getShowCount(){
    return showCount;
}

vector<Vector2> ZooidManager::getFreeChargeFirstPosition()
{
    vector<Vector2> chargePosition;
    for(unsigned int i=0; i<zooidCharge->chargeMsg.size(); i++)
    {
        bool isHas = false;
        for(unsigned int j=0; j<myZooids.size(); j++){
            if (lzm::absSq(myZooids[j]->getPosition() - zooidCharge->chargeMsg[i].second) < GoalRadius * GoalRadius){
                zooidCharge->chargeMsg[i].has = true;
                isHas = true;
                break;
            }
        }

        if(!isHas){
           chargePosition.push_back(zooidCharge->chargeMsg[i].first);
           zooidCharge->chargeMsg[i].has = false;
        }
    }

    // 第一位置vector
    return chargePosition;
}

void ZooidManager::setBatteryShow(bool show)
{
    showBattery = show;
    if(!myZooids.size())
    {
        return ;
    }

    for(int i=0; i<myZooids.size(); i++)
    {
        myZooids[i]->setBatteryShow(showBattery);
    }
}

void ZooidManager::setGoalShow(bool show)
{
    showGoal = show;
    if(!myGoals.size())
    {
        return ;
    }

    for(int i=0; i<myGoals.size(); i++)
    {
        myGoals[i]->setGoalShow(showGoal);
    }
}

int ZooidManager::getNbZooids()
{
    return static_cast<int>(myZooids.size());
}

int ZooidManager::getEnoughBatteryNbZooids()
{
    int number = 0;
    for(int i=0; i<myZooids.size(); i++)
    {
        if(myZooids[i]->getBatteryLevel() >= getBatteryLimit())
        {
            number ++;
        }
    }

    return number;
}

bool ZooidManager::haveZooid(unsigned int zooidId)
{
    auto it = find_if(myZooids.begin(), myZooids.end(), [&zooidId](Zooid *z) { return z->getId() == zooidId; });
    if (it != myZooids.end())
    {
        return true;
    }
    return false;
}

int ZooidManager::getZooidIndex(unsigned int zooidId)
{
    for(int i=0; i<myZooids.size(); i++)
    {
        if(myZooids[i]->getId() == zooidId)
        {
            return i;
        }
    }
    return -1;
}

void ZooidManager::addZooid(unsigned int zooidId)
{
    Vector2 position(pfRandFloat(0.1f, getWorldWidth() - 0.1f), pfRandFloat(0.1f, getWorldHeight() - 0.1f));
    addZooid(zooidId, position);
}

void ZooidManager::addZooid(unsigned int zooidId, Vector2 position)
{
    if(haveZooid(zooidId) || zooidId  >= MAX_NB_ZOOIDS)
    {
        return ;
    }

    Zooid *newZooid = new Zooid(ROBOT_RADIUS, position, showBattery);
    ZooidGoal *newGoal = new ZooidGoal(position, QColor(pfRandInt(50, 255),pfRandInt(50, 255),pfRandInt(50, 255)), showGoal);

    unsigned int goalId = static_cast<unsigned int>(zooidSimulator->addGoal(position));
    unsigned int agentId = static_cast<unsigned int>(zooidSimulator->addAgent(position, goalId));

    newZooid->setGoalPosition(position);
    newZooid->setColor(newGoal->getColor());
    newZooid->setId(zooidId);
    newZooid->setCharge(0);
    newZooid->resetWatchdog();


    if (zooidId >= 1 && zooidId <= 6)
    {
        newZooid->setRobotControlMode(Zooid::SpeedControl);
    }
    else
    {
        newZooid->setRobotControlMode(Zooid::PositionControl);  // 其他机器人使用位置控制
    }


    newGoal->setAssociatedZooid(zooidId);
    qDebug()<<"Message: Add Zooid is"<<zooidId<<"success.";

    //新上线的机器人就在充电桩
    int chargeId = zooidCharge->getNearSecondChargeId(position);
    if(chargeId != -1)
    {
        zooidCharge->setChargeZooid(chargeId);
        newZooid->setCharge(2);
    }

    unique_lock<mutex> lock(valuesMutex);
    {
        myZooids.push_back(newZooid);
        myGoals.push_back(newGoal);

        zooidSimulator->addZooid(newZooid);
        zooidSimulator->addZooidGoal(newGoal);
    }
    lock.unlock();

    assignRobots();
}

void ZooidManager::removeZooid(unsigned int zooidId)
{
    if(!haveZooid(zooidId))
    {
        qDebug()<<"Message: zooid id"<<zooidId<<"not has, remove zooid fail";
        return ;
    }

    unique_lock<mutex> lock(valuesMutex);
    {
        int index = getZooidIndex(zooidId);
        if(index != -1 )
        {
            //释放充电桩
            zooidCharge->releaseCharge(myZooids[index]->getPosition());

            //删除当前Zooid的代理
            zooidSimulator->eraseAgent(index);

            //删除当前Zooid的代理目标
            zooidSimulator->eraseGoal(index);

            delete(myZooids[index]);
            myZooids.erase(myZooids.begin() + index);

            delete(myGoals[index]);
            myGoals.erase(myGoals.begin() + index);
            testFeedbackMs.erase(zooidId);

            qDebug()<<"Message: Remove zooid id"<<zooidId<<"success.";
        }
    }
    lock.unlock ();

    assignRobots();
}

void ZooidManager::rotateZooid(unsigned int zooidId, float angle)
{
    unique_lock<mutex> lock(valuesMutex);
    {
        auto zooid = find_if(myZooids.begin(), myZooids.end(), [&zooidId](Zooid *z) { return z->getId() == zooidId; });
        if(zooid != myZooids.end())
        {
            (*zooid)->setOrientation((*zooid)->getOrientation() + angle);
        }
    }
    lock.unlock();
}

void ZooidManager::moveGoal(unsigned int index, Vector2 position)
{
    unique_lock<mutex> lock(valuesMutex);
    {
        if (index < myGoals.size())
        {
            myGoals[index]->setPosition(position);
        }
    }
    lock.unlock();
}

void ZooidManager::moveGoal(unsigned int index, float x, float y)
{
    unique_lock<mutex> lock(valuesMutex);
    {
        if (index < myGoals.size())
        {
            myGoals[index]->setPosition(x, y);
        }
    }
    lock.unlock();
}

void ZooidManager::setZooidInteraction(unsigned int zooidId, bool touched, bool blinded, bool tapped, bool shaken)
{
    unique_lock<mutex> lock(valuesMutex);
    {
        auto zooid = find_if(myZooids.begin(), myZooids.end(), [&zooidId](Zooid *z) { return z->getId() == zooidId; });
        if(zooid != myZooids.end())
        {
            int tmpState = 0;
            if (touched) tmpState |= 1;
            if (blinded) tmpState |= 2;
            if (tapped)  tmpState |= 4;
            if (shaken)  tmpState |= 8;
            (*zooid)->setState(tmpState);
        }
    }

    lock.unlock();
}

bool ZooidManager::getAllZooidInfo(vector<ZooidInfo> &allInfo)
{
    for(int i=0; i<myZooids.size(); i++)
    {
        ZooidInfo tempInfo;
        tempInfo.id = myZooids[i]->getId();
        tempInfo.color = myZooids[i]->getColor();
        tempInfo.batteryLevel = myZooids[i]->getBatteryLevel();
        tempInfo.orientation = myZooids[i]->getOrientation();
        tempInfo.speed = myZooids[i]->getSpeed();
        tempInfo.position = myZooids[i]->getPosition();
        allInfo.push_back(tempInfo);
    }
    return true;
}


bool ZooidManager::initZooidReceivers()
{
    bool initOk = true;
    //获取串口列表
    vector<string> serialPorts = getAvailableZooidReceivers();
    //取接收连接数
    int receiversToConnect = min(nbRequiredReceivers, (int)serialPorts.size());
    //遍历在线串口
    for (int i = 0; i < serialPorts.size(); i++)
    {
        if(strcmp(serialPorts[i].c_str(),"COM18")==0||strcmp(serialPorts[i].c_str(),"COM2")==0)
            continue;
        if (myReceivers.size() < receiversToConnect)
        {
            //创建接收器
            ZooidReceiver* z = new ZooidReceiver(myReceivers.size());
            //打开串口
            if (z->init(serialPorts[i]) == true)
            {
                myReceivers.push_back(z);
                //需要这样的睡眠才能收到握手的回应 保证串口有效连接  延时1ms
                Sleep(1);
            }
        }
        else
        {
            break;
        }
    }
    return initOk;
}

vector<string> ZooidManager::getAvailableZooidReceivers()
{
    //获取串口列表
    vector<string> descriptors;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        string currentDevice = info.portName().toStdString();
        if (currentDevice.find("COM") != -1)
        {
            descriptors.push_back(currentDevice.substr(currentDevice.find('(') + 1,currentDevice.length() - (currentDevice.find('(') + 1)));
        }
    }

    return descriptors;
}

void ZooidManager::processReceiversData()
{
    unique_lock<mutex> lock(valuesMutex);
    {
        for(auto z:myZooids)
        {
            z->tickWatchdog();
        }
    }
    lock.unlock();

    for (unsigned int i = 0; i < myReceivers.size(); i++)
    {
        while (myReceivers[i]->availableMessages() > 0)
        {
            ZooidMessage msg = myReceivers[i]->getLastMessage();
            unsigned int zooidId = msg.getSenderId() + myReceivers[i]->getId() * NUM_ZOOIDS_PER_RECEIVER;
            //接收到状态类型
            if (msg.getType() == TYPE_STATUS)
            {
                DecodedStatusMessage status;
                if (!decodeStatusMessage(msg, status))
                    continue;

                ZooidWorldPoint world;
                if (!zooidRawToWorld(
                        status.positionX, status.positionY, world))
                    continue;

                float robotA = (float)(status.orientation) / 100.0f;
                float robotX = static_cast<float>(world.x);
                float robotY = static_cast<float>(world.y);

                unique_lock<mutex> lock(valuesMutex);
                {
                    testFeedbackMs[zooidId] = msg.getReceivedAtMs();
                    auto zooid = find_if(myZooids.begin(), myZooids.end(), [&zooidId](Zooid *z) { return z->getId() == zooidId; });
                    if(zooid != myZooids.end())
                    {
                        //更新状态
                        (*zooid)->setLastUpdate(clock());
                        (*zooid)->setState(status.state);
                        (*zooid)->setPosition(robotX, robotY);
                        (*zooid)->setOrientation(robotA);
                        (*zooid)->setBatteryLevel(status.batteryLevel);
                        (*zooid)->resetWatchdog();
                       // qDebug()<<robotX<<","<<robotY;
                    }
                    else
                    {
                        //注册新Zooid
                        ZooidInfo temp;
                        temp.id = zooidId;
                        temp.position.setX(robotX);
                        temp.position.setY(robotY);
                        zooidRegisterBuff.push_back(temp);
                    }
                }
                lock.unlock();
            }
        }
    }
}

//void ZooidManager::controlRobotSpeed(int zooidId, int8_t motor1, int8_t motor2, QColor color)
//{
//    uint8_t buffer[5] = { 0 };

//    buffer[0] = (motor1 <= 100) ? motor1 : 100;
//    buffer[1] = (motor2 <= 100) ? motor2 : 100;
//    buffer[2] = color.red();
//    buffer[3] = color.green();
//    buffer[4] = color.blue();

//    ZooidReceiver* r = retrieveReceiver(zooidId);
//    if (r && r->isInitialized())
//    {
//        r->sendUSB(TYPE_MOTORS_VELOCITY,zooidId - r->getId() * NUM_ZOOIDS_PER_RECEIVER, sizeof(buffer), (uint8_t *)&buffer[0]);
//    }
//}

void ZooidManager::controlRobotSpeed(int zooidId, int16_t motor1, int16_t motor2, QColor color)
{
    const EncodedWheelSpeeds encoded = encodeWheelSpeeds(motor1, motor2);
    PositionControlMessage msg{};
    msg.positionX = encoded.positionX;
    msg.positionY = encoded.positionY;
    msg.colorRed = static_cast<uint8_t>(color.red());
    msg.colorGreen = static_cast<uint8_t>(color.green());
    msg.colorBlue = static_cast<uint8_t>(color.blue());
    msg.preferredSpeed = static_cast<uint8_t>(ZOOID_RUN_SPEED);
    msg.orientation = static_cast<int16_t>(static_cast<uint16_t>(52000));
    msg.isFinalGoal = true;
    msg.empty = (planningMode == ChargePlanning?0xfe : 0xff);
    msg.controlMode = 1; // Zooid::SpeedControl 的串口协议值

    //向指定的Receiver发送数据
    ZooidReceiver* r = retrieveReceiver(zooidId);
    if (r && r->isInitialized())
    {
        r->sendUSB(TYPE_ROBOT_POSITION, zooidId - r->getId()*NUM_ZOOIDS_PER_RECEIVER, sizeof(msg), (uint8_t *)&msg);
    }
}

ZooidReceiver* ZooidManager::retrieveReceiver(unsigned int zooidId)
{
    int receiverId = zooidId / NUM_ZOOIDS_PER_RECEIVER;
    if (receiverId < myReceivers.size())
    {
        for (int i = 0; i < myReceivers.size(); i++)
        {
            if (myReceivers[i]->getId() == receiverId)
            {
                return myReceivers[i];
            }
        }
    }
    return nullptr;
}

//position即可发送机器人位置也可以发送目标位置
void ZooidManager::controlRobotPosition(uint8_t zooidId, float x, float y, QColor color, float orientation, float preferredSpeed, bool isFinalGoal)
{
    PositionControlMessage msg{};

    if (!std::isfinite(x) || !std::isfinite(y))
        return;

    ZooidWorldPoint world = {
        static_cast<double>(x), static_cast<double>(y)
    };
    if (world.x < 0.0) world.x = 0.0;
    if (world.x > ZooidFieldWidth) world.x = ZooidFieldWidth;
    if (world.y < 0.0) world.y = 0.0;
    if (world.y > ZooidFieldHeight) world.y = ZooidFieldHeight;
    if (!zooidWorldToRaw(world, msg.positionX, msg.positionY))
        return;

    msg.colorRed = color.red();
    msg.colorGreen = color.green();
    msg.colorBlue = color.blue();
    msg.orientation = (uint16_t)(orientation * 100.0f);  //提高精度发送
    msg.preferredSpeed = (uint8_t)preferredSpeed;
    msg.isFinalGoal = isFinalGoal;
    msg.empty = (planningMode == ChargePlanning?0xfe : 0xff);
    msg.controlMode = 0; // Zooid::PositionControl 的串口协议值

    //向指定的Receiver发送数据
    ZooidReceiver* r = retrieveReceiver(zooidId);
    if (r && r->isInitialized())
    {
        r->sendUSB(TYPE_ROBOT_POSITION, zooidId - r->getId()*NUM_ZOOIDS_PER_RECEIVER, sizeof(msg), (uint8_t *)&msg);
    }
}

void ZooidManager::sendRobotsOrders()
{
    SimulationMode mode;

    unique_lock<mutex> lock(valuesMutex);
    {
        mode = simulationMode;
    }
    lock.unlock();

    cntOrder++;
    if(cntOrder >= 5)
        cntOrder = 0;

    for (int i = 0; i < myZooids.size(); i++)
    {
        if (!myZooids[i]->isActivated())
        {
            controlRobotSpeed(myZooids[i]->getId(), 0, 0, myZooids[i]->getColor());//将这个机器人的速度为0 0
        }
        else if (mode == On)
        {

            // 速度控制模式
            if(myZooids[i]->getRobotControlMode() == Zooid::SpeedControl)
            {
                if(cntOrder == 0)
                {
                    if(zooidFollow->stateCode == ZooidFollow::Free)
                        controlRobotSpeed(myZooids[i]->getId(), 0, 0, myZooids[i]->getColor());
                }

//                controlRobotPosition(
//                    myZooids[i]->getId(),
//                    2500,
//                    2500,
//                    myZooids[i]->getColor(),
//                    520, //本来想用angle == 520 作为标志位
//                    ZOOID_RUN_SPEED,
//                    true);

                continue;
            }

            // 检查是否到达目标
            bool isGoalReached = false;
            float preferredSpeed = ZOOID_RUN_SPEED;
           // preferredSpeed = 100;
            if (lzm::absSq(myZooids[i]->getPosition() - myGoals[i]->getPosition()) < GoalRadius * GoalRadius)
            {
                  isGoalReached = true;
            }

#if AGENT_DEBUG
            agentZooid->setPosition(zooidSimulator->getAgentPosition(0));
#endif
            controlRobotPosition(
                myZooids[i]->getId(),
                zooidSimulator->getAgentPosition(i).getX(),
                zooidSimulator->getAgentPosition(i).getY(),
                myZooids[i]->getColor(),
                myZooids[i]->getOrientation(),
                preferredSpeed,
                isGoalReached);
        }
        else if (mode == NoPlanning)
        {
            controlRobotPosition(
                myZooids[i]->getId(),
                myZooids[i]->getPosition().getX(),
                myZooids[i]->getPosition().getY(),
                myZooids[i]->getColor(),
                myZooids[i]->getOrientation(),
                myZooids[i]->getSpeed(),
                myZooids[i]->isGoalReached());
        }
    }

    //一次性发送所有数据
    for (int i = 0; i < myReceivers.size(); i++)
    {
        myReceivers[i]->setReadyToSend();
    }
}

int ZooidManager::getNbConnectedReceivers()
{
    return myReceivers.size();
}
