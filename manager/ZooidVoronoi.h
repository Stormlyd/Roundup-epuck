#ifndef ZOOIDVORONOI_H
#define ZOOIDVORONOI_H

#pragma once

#include <QObject>
#include <vector>
#include <QColor>
#include <cmath>

#include "Zooid.h"

// 使用 lzm::Vector2（与现有代码一致）
using namespace lzm;

// ==================== 几何结构体 ====================

/**
 * @brief 2D向量工具（兼容 lzm::Vector2 的扩展运算）
 */
struct Vec2 {
    float x, y;
    Vec2(float _x = 0.0f, float _y = 0.0f) : x(_x), y(_y) {}
    Vec2(const Vector2& v) : x(v.getX()), y(v.getY()) {}

    operator Vector2() const { return Vector2(x, y); }

    Vec2 operator+(const Vec2& o) const { return Vec2(x + o.x, y + o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x - o.x, y - o.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    Vec2 operator/(float s) const { return Vec2(x / s, y / s); }

    float norm() const { return std::sqrt(x * x + y * y); }
    float normSq() const { return x * x + y * y; }
    float dot(const Vec2& o) const { return x * o.x + y * o.y; }
    Vec2 normalized() const { float n = norm(); return n > 1e-6f ? Vec2(x / n, y / n) : Vec2(0, 0); }
};

/**
 * @brief 多边形（替代MATLAB polyshape）
 */
struct VoronoiPolygon {
    vector<Vec2> vertices;   // 逆时针，首尾不重复
    float area = 0.0f;

    bool isEmpty() const { return vertices.size() < 3; }
    void computeArea();
    Vec2 centroid() const;
};

/**
 * @brief 障碍物覆盖圆（对应MATLAB中的 z_obs, R_obs）
 */
struct CoverCircle {
    Vec2 center;
    float radius;
};

/**
 * @brief 动态障碍物
 */
struct DynamicObstacle {
    Vec2 center;
    float side = 0.4f;           // 边长（米）
    float speed = 0.0f;          // 当前速度
    Vec2 moveDir;                // 运动方向（单位向量）
    float accel = 0.0f;          // 加速度
    float maxSpeed = 0.0f;       // 最大速度

    vector<Vec2> getVertices() const;
    vector<CoverCircle> getCoverCircles() const;
};

/**
 * @brief 危险三角形（前进边避障）
 */
struct DangerTriangle {
    Vec2 P1, P2;        // 前进边两端
    Vec2 apex;          // 顶点
    Vec2 mid;           // 边中点
    float length = 0.0f; // 三角形长度
};

/**
 * @brief OAVC区域信息
 */
struct OAVCRegion {
    VoronoiPolygon poly;
    Vec2 centroid;
    bool valid = false;
};

/**
 * @brief 机器人控制状态
 */
struct RobotControlState {
    int id = 0;
    Vec2 position;
    Vec2 velocityCmd;
    Vec2 centroid;
    VoronoiPolygon region;
    int convergedSteps = 0;
    QColor color;
};

/**
 * @brief ZooidVoronoi 多机器人覆盖控制器
 * @details 实现 OAVC + CVT 变速障碍避障覆盖算法
 */
class ZooidVoronoi : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父对象
     */
    explicit ZooidVoronoi(QObject *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~ZooidVoronoi();

    // ==================== 参数设置 ====================

    /**
     * @brief 设置场地边界（米）
     * @param x0 左边界
     * @param y0 下边界
     * @param width 宽度
     * @param height 高度
     */
    void setFieldBounds(float x0, float y0, float width, float height);

    /**
     * @brief 设置机器人半径（米）
     * @param radius 半径
     */
    void setRobotRadius(float radius);

    /**
     * @brief 设置控制参数
     * @param kp CVT增益
     * @param kRep 斥力增益
     * @param lengthSpeedRatio 危险三角形长度/速度比
     * @param maxVel 最大速度（m/s）
     */
    void setControlParams(float kp, float kRep, float lengthSpeedRatio, float maxVel);

    /**
     * @brief 设置障碍物参数
     * @param side 边长
     * @param startPos 初始位置
     * @param moveDir 运动方向
     * @param v0 初速度
     * @param accel 加速度
     * @param maxSpd 最大速度
     */
    void setObstacleParams(float side, Vector2 startPos, Vector2 moveDir,
                          float v0, float accel, float maxSpd);

    /**
     * @brief 添加静态障碍物（离散圆拟合）
     * @param center 障碍物中心位置
     * @param side 边长（米）
     * @param circlesPerSide 每条边的离散圆数量（默认3）
     */
    void addStaticObstacle(Vector2 center, float side, int circlesPerSide = 3);

    /**
     * @brief 启用/禁用静态障碍物的OAVC约束
     * @param enabled true=启用
     */
    void setObstacleEnabled(bool enabled) { obstacleEnabled = enabled; }
    void setObstacleSpeedZero() { obstacle.speed = 0; }  // 强制静态障碍物模式

    /**
     * @brief 设置编队模式
     * @param enabled true=编队推波模式
     */
    void setFormationMode(bool enabled) { isFormationMode = enabled; }

    /**
     * @brief 设置编队参数
     */
    void setFormationParams(float width, float height, float speed,
                           Vector2 startCenter, Vector2 targetCenter);

    /**
     * @brief 设置编队内机器人偏移
     */
    void setFormationOffsets(const vector<Vec2>& offsets) { formationOffsets = offsets; }

    /**
     * @brief 设置双障碍物模式
     */
    void setDualObstacleMode(bool enabled) { isDualObstacleMode = enabled; }
    void setPushWaveRobotMode(bool enabled) { isPushWaveRobotMode = enabled; }

    /**
     * @brief 添加第二个静态障碍物
     */
    void addStaticObstacle2(Vector2 center, float side, int circlesPerSide = 3);

    /**
     * @brief 设置障碍物机器人参数
     */
    void setObstacleRobotParams(float speed, float side, Vec2 moveDir);

    /**
     * @brief 获取当前覆盖圆（动态或静态，用于可视化）
     * @return 覆盖圆列表
     */
    const vector<CoverCircle>& getObstacleCoverCircles() const { return currentCoverCircles; }

    /**
     * @brief 静态障碍物边界顶点（用于可视化）
     * @return 矩形四个顶点
     */
    vector<Vec2> getStaticObstacleBoundary() const;

    // ==================== 运行时接口 ====================

    /**
     * @brief 更新障碍物状态（每周期调用）
     * @param dt 时间步长（秒）
     */
    void updateObstacle(float dt);

    /**
     * @brief 获取当前障碍物位置
     * @return 障碍物中心位置
     */
    Vector2 getObstacleCenter() const;

    /**
     * @brief 判断障碍物是否已离开场地
     * @return true=已离开
     */
    bool isObstacleLeftField() const;

    /**
     * @brief 判断障碍物是否在场地内
     * @return true=在
     */
    bool isObstacleInField() const;

    /**
     * @brief 执行单步控制计算（对应MATLAB主循环体）
     * @param robots 机器人指针数组（6个）
     * @param time 当前时间（秒）
     * @return 是否达到收敛停止条件
     */
    bool computeControlStep(vector<Zooid*>& robots, float time);

    /**
     * @brief 计算所有机器人的速度命令
     * @param robots 机器人指针数组
     * @param time 当前时间
     * @details 这是主入口，每0.1s调用一次
     */
    void computeAllVelocities(vector<Zooid*>& robots, float time);

    /**
     * @brief 检查所有机器人是否收敛
     * @param robots 机器人指针数组
     * @return true=全部收敛
     */
    bool checkAllConverged(const vector<Zooid*>& robots) const;

    /**
     * @brief 重置控制器状态
     */
    void reset();

    // ==================== 数据获取（调试用）====================

    /**
     * @brief 获取机器人区域
     * @param robotIndex 机器人索引
     * @return 区域多边形
     */
    VoronoiPolygon getRobotRegion(int robotIndex) const;

    /**
     * @brief 获取覆盖率（0~1）
     * @return 覆盖率
     */
    float getCoverageRatio() const { return coverageRatio; }
    float getMinDistToObstacle() const { return minDistToObstacle; }
    float getFrameMinDist() const { return frameMinDist; }
    float getFrameInterRobotMinDist() const { return interRobotMinDist; }
    float getAvgRecoveryTime() const { return avgRecoveryTime; }
    float getCoverageVariance() const { return coverageVariance; }
    float getCostFunction() const { return costFunctionValue; }
    float getCostMaxDisturb() const { return costMaxDisturb; }
    float getInterRobotMinDist() const { return interRobotMinDist; }

    /**
     * @brief 获取当前迭代次数
     * @return 迭代次数
     */
    int getIteration() const { return iteration; }

    /**
     * @brief 获取障碍物对象
     * @return 障碍物
     */
    const DynamicObstacle& getObstacle() const { return obstacle; }

    /**
     * @brief 获取障碍物顶点
     * @return 顶点数组
     */
    vector<Vec2> getObstacleVertices() const { return obstacle.getVertices(); }

    /**
     * @brief 获取当前危险三角形
     * @return 危险三角形
     */
    const DangerTriangle& getDangerTriangle() const { return currentDanger; }
    const DangerTriangle& getStaticObs2Danger() const { return staticObs2Danger; }

    /**
     * @brief 获取编队中心（用于可视化）
     */
    Vec2 getFormationCenter() const { return formationCenter; }

    /**
     * @brief 获取编队边界（宽高）
     */
    void getFormationBounds(float& w, float& h) const { w = formationWidth; h = formationHeight; }

    /**
     * @brief 是否编队模式
     */
    bool getFormationMode() const { return isFormationMode; }
    bool getDualObstacleMode() const { return isDualObstacleMode; }
    bool getPushWaveRobotMode() const { return isPushWaveRobotMode; }
    Vec2 getObstacleRobotPos() const { return dualObsStartPos; }  // 当前障碍物位置
    const vector<CoverCircle>& getStaticObs2Circles() const { return staticObs2Circles; }

signals:
    /**
     * @brief 速度命令信号（连接到ZooidManager::controlRobotSpeed）
     */
    void speedCommand(int zooidId, int16_t leftSpeed, int16_t rightSpeed, QColor color);

private:
    // ==================== 核心算法函数 ====================

    /**
     * @brief 计算OAVC Voronoi分区
     * @param positions 所有机器人位置
     * @param obsCircles 障碍物覆盖圆
     * @return 每个机器人的区域数组
     */
    vector<OAVCRegion> computeOAVCRegions(const vector<Vec2>& positions,
                                           const vector<CoverCircle>& obsCircles,
                                           const float* xr_override = nullptr,
                                           const float* yr_override = nullptr);

    /**
     * @brief 计算单个OAVC区域
     * @param myPos 当前机器人位置
     * @param allPositions 所有机器人位置
     * @param myIndex 当前机器人索引
     * @param obsCircles 障碍物覆盖圆
     * @return OAVC区域
     */
    OAVCRegion computeSingleOAVC(const Vec2& myPos, const vector<Vec2>& allPositions,
                                  int myIndex, const vector<CoverCircle>& obsCircles);

    /**
     * @brief 半平面交集计算（对应MATLAB halfplane）
     */
    VoronoiPolygon halfplane(float A, float B, float C, float xr[2], float yr[2]);

    /**
     * @brief 无界半平面相交
     */
    VoronoiPolygon unboundedHalfplaneIntersect(const VoronoiPolygon& poly, float A, float B, float C);

    /**
     * @brief 多边形交集（简化版）
     */
    VoronoiPolygon VoronoiPolygonIntersect(const VoronoiPolygon& p1, const VoronoiPolygon& p2);

    /**
     * @brief 计算多边形质心
     */
    Vec2 computeVoronoiPolygonCentroid(const VoronoiPolygon& poly) const;

    /**
     * @brief 计算危险三角形
     */
    DangerTriangle computeForwardEdgeDangerTriangle(const Vec2& obsCenter,
                                                     const vector<Vec2>& obsVerts,
                                                     const Vec2& dir, float speed);

    /**
     * @brief 计算斥力
     */
    Vec2 computeRepulsiveForce(const Vec2& robotPos, const DangerTriangle& danger);

    /**
     * @brief 将速度向量转换为左右轮速（差速模型）
     * @param robotIndex 机器人索引
     * @param vCmd 速度命令（vx, vy）
     * @param orientation 当前朝向（度）
     * @return [leftSpeed, rightSpeed]（mm/s）
     */
    vector<int16_t> velocityToWheelSpeeds(int robotIndex, const Vec2& vCmd, float orientation);

    /**
     * @brief 限幅函数
     */
    float clamp(float value, float min, float max) const;

    /**
     * @brief 计算两点距离
     */
    float distance(const Vec2& a, const Vec2& b) const;

    // ==================== 成员变量 ====================

    // 场地参数
    float fieldX0 = 0.0f;
    float fieldY0 = 0.0f;
    float fieldW = 1.460f;      // 默认1460mm = 1.46m
    float fieldH = 0.914f;      // 默认914mm = 0.914m

    // 机器人参数
    float robotRadius = 0.03f;   // 30mm = 0.03m（根据Zooid实际修改）
    int numRobots = 0;      //运行时动态获取

    // 控制参数
    float kP = 1.2f;
    float kRep = 45.0f;
    float lengthSpeedRatio = 2.5f;
    float maxVelocity = 2.0f;    // m/s
    float dt = 0.1f;

    // 障碍物
    DynamicObstacle obstacle;
    Vec2 obstacleInitialCenter;     // 障碍物初始位置（用于reset）
    float obstacleInitialSpeed = 0.0f; // 障碍物初始速度（用于reset）

    // 静态障碍物（OAVC离散圆拟合）
    vector<CoverCircle> staticObsCoverCircles;
    Vec2 staticObsCenter;
    float staticObsSide = 0.0f;
    bool obstacleEnabled = false;

    // 当前覆盖圆（用于可视化，动态或静态）
    vector<CoverCircle> currentCoverCircles;

    // 双障碍物编队模式
    bool isDualObstacleMode = false;          // 双障碍物编队模式
    bool isPushWaveRobotMode = false;         // 推波机器人障碍物模式
    int obstacleRobotIndex = -1;              // 充当障碍物的机器人索引
    float obstacleRobotSpeed = 0.02f;         // 障碍物速度（m/s）
    float obstacleRobotSide = 0.15f;          // 障碍物边长（m）
    Vec2 obstacleRobotMoveDir = Vec2(-1, 0);  // 从右向左
    Vec2 dualObsStartPos;                     // 障碍物初始位置
    Vec2 staticObs2Center;                    // 第二静止障碍物中心
    float staticObs2Side = 0.15f;             // 第二静止障碍物边长
    vector<CoverCircle> staticObs2Circles;    // 第二静止障碍物覆盖圆

    // 状态
    int iteration = 0;
    float coverageRatio = 0.0f;
    float minDistToObstacle = 1e9f;       // 全程最近距离
    vector<float> recoveryTimes;           // 每次恢复时间
    float avgRecoveryTime = 0;             // 平均恢复时间
    float coverageVariance = 0;            // 覆盖面积方差
    float costFunctionValue = 0;           // 成本函数 J=Σ(dist²)
    float costBaseline = 0;                // 扰动前基线
    float costRecoveryTime = 0;            // 成本函数恢复时间
    float costMaxDisturb = 0;              // 最大扰动幅度
    bool costDisturbed = false;            // 是否受扰动
    int costDisturbCount = 0;              // 扰动持续时间计数
    float frameMinDist = 1e9f;             // 本帧最近距离
    float interRobotMinDist = 1e9f;        // 机器人间最小间距
    vector<bool> wasDisturbed;             // 每机器人上一步是否受斥力
    vector<int> recoveryCounters;          // 每机器人恢复计时器
    bool obsHasLeft = false;
    bool obsHasEntered = false;  // 障碍物是否曾进入场地
    int waitStepsAfterObsLeft = 0;
    int convergedSteps = 0;           // 连续收敛步数
    DangerTriangle currentDanger;  // 当前危险三角形
    DangerTriangle staticObs2Danger; // 静态障碍物2危险三角
    bool isFormationMode = false; // 编队推波模式

    // 编队推波参数
    Vec2 formationCenter;               // 编队中心当前坐标（米）
    Vec2 formationMoveDir = Vec2(1, 0); // 编队移动方向（默认向右）
    float formationWidth = 0.8f;        // 编队宽（米）
    float formationHeight = 0.5f;       // 编队高（米）
    float formationSpeed = 0.05f;       // 编队前进速度（m/s）
    static constexpr float FEEDFORWARD_MULT = 1.8f; // 前馈速度倍率
    vector<Vec2> formationOffsets;      // 每个机器人在编队内的相对偏移
    Vec2 formationTargetCenter;         // 编队目标中心
    vector<Vec2> staticObsVerts;        // 静止障碍物顶点
    Vec2 formationStartCenter;          // 编队初始中心（用于reset）
    int formationPhase = 0;             // 0=收敛阶段不移动, 1=运动阶段

    // 常量
    static constexpr float CONVERGENCE_THRESHOLD = 0.02f;
    static constexpr float K_turn = 7.0f;  //转向系数
    static constexpr float SPIN_THRESHOLD = 40.0f;  // 偏差>40°原地自转
    static constexpr float SPIN_SPEED = 20.0f;  // 自转角速度 (rad/s)，约 0.13s/rad
    static constexpr int CONVERGENCE_STABLE_STEPS = 10;
    static constexpr int MAX_WAIT_AFTER_OBS_LEFT = 200;
    static constexpr float WHEEL_DISTANCE = 0.05f;  // 轮距50mm = 0.05m
    static constexpr int16_t MAX_WHEEL_SPEED = 1000; // 最大轮速mm/s
    static constexpr int16_t MIN_WHEEL_SPEED = 0;  // 最小启动速度mm/s（克服静摩擦）

    // 缓存
    vector<OAVCRegion> lastRegions;
};

#endif // ZOOIDVORONOI_H
