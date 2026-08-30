#include "ZooidVoronoi.h"
#include <QtDebug>
#include <algorithm>

// ==================== 前向声明 ====================
vector<Vec2> convexHull(vector<Vec2> pts);
VoronoiPolygon sutherlandHodgmanClip(const VoronoiPolygon& subject, const VoronoiPolygon& clipPoly);

// ==================== Vec2 与 Vector2 转换辅助 ====================
static Vec2 toVec2(const Vector2& v) { return Vec2(v.getX(), v.getY()); }
static Vector2 toVector2(const Vec2& v) { return Vector2(v.x, v.y); }

// ==================== VoronoiPolygon 方法实现 ====================
void VoronoiPolygon::computeArea()
{
    if (vertices.size() < 3) { area = 0; return; }
    float s = 0;
    int n = vertices.size();
    for (int i = 0; i < n; ++i) {
        const Vec2& p1 = vertices[i];
        const Vec2& p2 = vertices[(i + 1) % n];
        s += p1.x * p2.y - p2.x * p1.y;
    }
    area = std::abs(s) * 0.5f;
}

Vec2 VoronoiPolygon::centroid() const
{
    if (vertices.empty()) return Vec2(0, 0);
    if (vertices.size() < 3) {
        float cx = 0, cy = 0;
        for (const auto& v : vertices) { cx += v.x; cy += v.y; }
        return Vec2(cx / vertices.size(), cy / vertices.size());
    }

    int n = vertices.size();
    float A = 0, Cx = 0, Cy = 0;
    for (int i = 0; i < n; ++i) {
        const Vec2& p1 = vertices[i];
        const Vec2& p2 = vertices[(i + 1) % n];
        float cr = p1.x * p2.y - p2.x * p1.y;
        A += cr;
        Cx += (p1.x + p2.x) * cr;
        Cy += (p1.y + p2.y) * cr;
    }
    A *= 0.5f;
    if (std::abs(A) < 1e-8f) {
        float cx = 0, cy = 0;
        for (const auto& v : vertices) { cx += v.x; cy += v.y; }
        return Vec2(cx / n, cy / n);
    }
    return Vec2(Cx / (6 * A), Cy / (6 * A));
}

// ==================== DynamicObstacle 方法 ====================
vector<Vec2> DynamicObstacle::getVertices() const
{
    float h = side / 2.0f;
    return {
        Vec2(center.x - h, center.y - h),
        Vec2(center.x + h, center.y - h),
        Vec2(center.x + h, center.y + h),
        Vec2(center.x - h, center.y + h)
    };
}

vector<CoverCircle> DynamicObstacle::getCoverCircles() const
{
    vector<CoverCircle> circles;
    float m = 3.0f;
    float s = side / m;
    float a = s / 2.0f;
    float d = a;
    float R = std::sqrt(a * a + d * d);

    float h = side / 2.0f;
    float ox = center.x, oy = center.y;
    float x1 = ox - h + a;
    float x2 = ox + h - a;
    float y1 = oy - h + a;
    float y2 = oy + h - a;

    // 四条边各3个圆
    for (int i = 0; i < 3; ++i) {
        float t = i / 2.0f;
        circles.push_back({Vec2(x1 + t * (x2 - x1), oy - h + d), R});
        circles.push_back({Vec2(ox + h - d, y1 + t * (y2 - y1)), R});
        circles.push_back({Vec2(x2 - t * (x2 - x1), oy + h - d), R});
        circles.push_back({Vec2(ox - h + d, y2 - t * (y2 - y1)), R});
    }
    return circles;
}

// ==================== ZooidVoronoi 构造/析构 ====================
ZooidVoronoi::ZooidVoronoi(QObject *parent) : QObject(parent)
{
    obstacle.moveDir = Vec2(1, 0);  // 默认向右
}

ZooidVoronoi::~ZooidVoronoi()
{
}

// ==================== 参数设置 ====================
void ZooidVoronoi::setFieldBounds(float x0, float y0, float width, float height)
{
    fieldX0 = x0;
    fieldY0 = y0;
    fieldW = width;
    fieldH = height;
}

void ZooidVoronoi::setRobotRadius(float radius)
{
    robotRadius = radius;
}

void ZooidVoronoi::setControlParams(float kp, float kRep, float lengthSpeedRatio, float maxVel)
{
    this->kP = kp;
    this->kRep = kRep;
    this->lengthSpeedRatio = lengthSpeedRatio;
    this->maxVelocity = maxVel;
}

void ZooidVoronoi::setObstacleParams(float side, Vector2 startPos, Vector2 moveDir,
                                      float v0, float accel, float maxSpd)
{
    obstacle.side = side;
    obstacle.center = toVec2(startPos);
    obstacle.moveDir = toVec2(moveDir).normalized();
    obstacle.speed = v0;
    obstacle.accel = accel;
    obstacle.maxSpeed = maxSpd;
    // 保存初始值用于 reset
    obstacleInitialCenter = obstacle.center;
    obstacleInitialSpeed = v0;
}

void ZooidVoronoi::addStaticObstacle2(Vector2 center, float side, int circlesPerSide)
{
    staticObs2Center = toVec2(center);
    staticObs2Side = side;
    staticObs2Circles.clear();

    float m = static_cast<float>(circlesPerSide);
    float s = side / m;
    float a = s / 2.0f;
    float d = a;
    float R = std::sqrt(a * a + d * d);
    float h = side / 2.0f;
    float ox = staticObs2Center.x, oy = staticObs2Center.y;

    float x_start = ox - h + a;
    float y_fixed = oy - h + d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float xi = x_start + i * s;
        staticObs2Circles.push_back({Vec2(xi, y_fixed), R});
    }
    float y_start = oy - h + a;
    float x_fixed = ox + h - d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float yi = y_start + i * s;
        staticObs2Circles.push_back({Vec2(x_fixed, yi), R});
    }
    float x_start_r = ox + h - a;
    float y_fixed_r = oy + h - d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float xi = x_start_r - i * s;
        staticObs2Circles.push_back({Vec2(xi, y_fixed_r), R});
    }
    float y_start_d = oy + h - a;
    float x_fixed_d = ox - h + d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float yi = y_start_d - i * s;
        staticObs2Circles.push_back({Vec2(x_fixed_d, yi), R});
    }
}

void ZooidVoronoi::setObstacleRobotParams(float speed, float side, Vec2 moveDir)
{
    obstacleRobotSpeed = speed;
    obstacleRobotSide = side;
    obstacleRobotMoveDir = moveDir;
}

void ZooidVoronoi::setFormationParams(float width, float height, float speed,
                                       Vector2 startCenter, Vector2 targetCenter)
{
    formationWidth = width;
    formationHeight = height;
    formationSpeed = speed;
    formationStartCenter = toVec2(startCenter);
    formationTargetCenter = toVec2(targetCenter);
    // 保存静止障碍物顶点（用于可视化和OAVC）
    float h = staticObsSide / 2.0f;
    float cx = staticObsCenter.x;
    float cy = staticObsCenter.y;
    staticObsVerts = {
        Vec2(cx - h, cy - h),
        Vec2(cx + h, cy - h),
        Vec2(cx + h, cy + h),
        Vec2(cx - h, cy + h)
    };
}

// ==================== 静态障碍物（OAVC离散圆拟合）====================
void ZooidVoronoi::addStaticObstacle(Vector2 center, float side, int circlesPerSide)
{
    staticObsCenter = toVec2(center);
    staticObsSide = side;
    staticObsCoverCircles.clear();

    float m = static_cast<float>(circlesPerSide);
    float s = side / m;
    float a = s / 2.0f;
    float d = a;
    float R = std::sqrt(a * a + d * d);
    float h = side / 2.0f;
    float ox = staticObsCenter.x, oy = staticObsCenter.y;

    // 四条边 × m个圆
    // 下边（水平，从左到右）
    float x_start = ox - h + a;
    float y_fixed = oy - h + d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float xi = x_start + i * s;
        staticObsCoverCircles.push_back({Vec2(xi, y_fixed), R});
    }
    // 右边（垂直，从下到上）
    float y_start = oy - h + a;
    float x_fixed = ox + h - d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float yi = y_start + i * s;
        staticObsCoverCircles.push_back({Vec2(x_fixed, yi), R});
    }
    // 上边（水平，从右到左）
    float x_start_r = ox + h - a;
    float y_fixed_r = oy + h - d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float xi = x_start_r - i * s;
        staticObsCoverCircles.push_back({Vec2(xi, y_fixed_r), R});
    }
    // 左边（垂直，从上到下）
    float y_start_d = oy + h - a;
    float x_fixed_d = ox - h + d;
    for (int i = 0; i < circlesPerSide; ++i) {
        float yi = y_start_d - i * s;
        staticObsCoverCircles.push_back({Vec2(x_fixed_d, yi), R});
    }

    qDebug() << "ZooidVoronoi::addStaticObstacle: side=" << side << ", m=" << circlesPerSide
             << ", R=" << R << ", total circles=" << staticObsCoverCircles.size();
}

vector<Vec2> ZooidVoronoi::getStaticObstacleBoundary() const
{
    float h = staticObsSide / 2.0f;
    return {
        Vec2(staticObsCenter.x - h, staticObsCenter.y - h),
        Vec2(staticObsCenter.x + h, staticObsCenter.y - h),
        Vec2(staticObsCenter.x + h, staticObsCenter.y + h),
        Vec2(staticObsCenter.x - h, staticObsCenter.y + h)
    };
}

// ==================== 障碍物更新 ====================
void ZooidVoronoi::updateObstacle(float dt)
{
    obstacle.speed = std::min(obstacle.speed + obstacle.accel * dt, obstacle.maxSpeed);
    obstacle.center = obstacle.center + obstacle.moveDir * (obstacle.speed * dt);
}

Vector2 ZooidVoronoi::getObstacleCenter() const
{
    return toVector2(obstacle.center);
}

bool ZooidVoronoi::isObstacleLeftField() const
{
    float h = obstacle.side / 2.0f;
    float minX = obstacle.center.x - h;
    float maxX = obstacle.center.x + h;
    float minY = obstacle.center.y - h;
    float maxY = obstacle.center.y + h;

    bool left = maxX < fieldX0;           // 全在左边外
    bool right = minX > fieldX0 + fieldW;  // 全在右边外
    bool bottom = maxY < fieldY0;          // 全在下边外
    bool top = minY > fieldY0 + fieldH;   // 全在上边外

    return left || right || bottom || top;
}

bool ZooidVoronoi::isObstacleInField() const
{
    float h = obstacle.side / 2.0f;
    float minX = obstacle.center.x - h;
    float maxX = obstacle.center.x + h;
    float minY = obstacle.center.y - h;
    float maxY = obstacle.center.y + h;

    // 与场地有重叠 = 在场地内（部分或全部）
    bool overlapX = !(maxX < fieldX0 || minX > fieldX0 + fieldW);
    bool overlapY = !(maxY < fieldY0 || minY > fieldY0 + fieldH);

    return overlapX && overlapY;
}

// ==================== 主控制入口（含OAVC + 推波机制）====================
bool ZooidVoronoi::computeControlStep(vector<Zooid*>& robots, float time)
{
    if (robots.empty()) {
        qWarning() << "ZooidVoronoi: No robots provided";
        return false;
    }
    numRobots = (int)robots.size();

    iteration++;

    // 机器人障碍物模式：找ID最大的机器人作障碍物
    if ((isDualObstacleMode || isPushWaveRobotMode) && obstacleRobotIndex < 0) {
        int maxId = -1;
        for (int j = 0; j < numRobots; ++j) {
            int rid = robots[j]->getId();
            if (rid > maxId) { maxId = rid; obstacleRobotIndex = j; }
        }
        qDebug() << "DualObs: obstacle robot idx=" << obstacleRobotIndex
                 << " id=" << robots[obstacleRobotIndex]->getId();
    }

    // 1. 收集所有机器人位置
    vector<Vec2> positions;
    for (Zooid* r : robots) {
        positions.push_back(toVec2(r->getPosition()));
    }

    // 2. 障碍物覆盖圆 + 危险三角形（推波机制）
    vector<CoverCircle> obsCircles;
    currentDanger = DangerTriangle();  // 重置，无危险时不显示

    if (obstacleEnabled || isFormationMode || isDualObstacleMode || isPushWaveRobotMode) {
        if (isPushWaveRobotMode) {
            // 推波机器人模式：从机器人当前位置读取障碍物位置
            if (obstacleRobotIndex >= 0 && obstacleRobotIndex < numRobots)
                dualObsStartPos = toVec2(robots[obstacleRobotIndex]->getPosition());
            DynamicObstacle tmpObs;
            tmpObs.center = dualObsStartPos;
            tmpObs.side = obstacleRobotSide;
            auto tmpCircles = tmpObs.getCoverCircles();
            for (auto& c : tmpCircles) obsCircles.push_back(c);
            currentCoverCircles = obsCircles;

            float r6speed = std::abs(obstacleRobotSpeed);
            Vec2 relDir = obstacleRobotMoveDir * (obstacleRobotSpeed > 0 ? 1.0f : -1.0f);
            vector<Vec2> r6verts = tmpObs.getVertices();
            currentDanger = computeForwardEdgeDangerTriangle(dualObsStartPos, r6verts, relDir, r6speed);
        } else if (isDualObstacleMode) {
            // 双障碍物模式：仅一个静态障碍物（中央下方）+ 动态障碍物（6号机器人）
            obsCircles = staticObs2Circles;

            // 更新障碍物机器人位置（从机器人当前位置，推波模式也复用）
            if (obstacleRobotIndex >= 0 && obstacleRobotIndex < numRobots)
                dualObsStartPos = toVec2(robots[obstacleRobotIndex]->getPosition());
            // 动态障碍物覆盖圆：复用标准OAVC公式
            DynamicObstacle tmpObs;
            tmpObs.center = dualObsStartPos;
            tmpObs.side = obstacleRobotSide;
            auto tmpCircles = tmpObs.getCoverCircles();
            for (auto& c : tmpCircles) obsCircles.push_back(c);
            currentCoverCircles = obsCircles;

            // 动态障碍物(6号机器人)危险三角
            float r6speed = obstacleRobotSpeed / 24.0f;
            Vec2 relDir = formationMoveDir * -1.0f;
            vector<Vec2> r6verts = tmpObs.getVertices();
            currentDanger = computeForwardEdgeDangerTriangle(dualObsStartPos, r6verts, relDir, r6speed);

            // 静态障碍物危险三角：相对速度 = formationSpeed
            float sh = staticObs2Side / 2.0f;
            float sx = staticObs2Center.x, sy = staticObs2Center.y;
            vector<Vec2> sverts = {Vec2(sx-sh,sy-sh),Vec2(sx+sh,sy-sh),Vec2(sx+sh,sy+sh),Vec2(sx-sh,sy+sh)};
            staticObs2Danger = computeForwardEdgeDangerTriangle(staticObs2Center, sverts, relDir, formationSpeed * 1.5f);
        } else if (isFormationMode) {
            // 编队模式：使用静止障碍物覆盖圆，危险三角用相对速度
            obsCircles = staticObsCoverCircles;
            currentCoverCircles = obsCircles;
            // 相对速度：障碍物从编队视角向编队逼近，用放大后速度使斥力提早作用
            Vec2 relativeDir = formationMoveDir * -1.0f;
            currentDanger = computeForwardEdgeDangerTriangle(
                staticObsCenter, staticObsVerts, relativeDir,
                formationSpeed * FEEDFORWARD_MULT);
        } else if (obstacle.speed > 0.001f) {
            // 动态障碍物：更新位置，实时生成覆盖圆和危险三角形
            updateObstacle(dt);
            obsCircles = obstacle.getCoverCircles();

            // 跟踪障碍物穿越状态
            if (isObstacleInField()) {
                if (!obsHasEntered) {
                    obsHasEntered = true;
                    qDebug() << "Obstacle entered field at iteration" << iteration;
                }
                vector<Vec2> verts = obstacle.getVertices();
                currentDanger = computeForwardEdgeDangerTriangle(
                    obstacle.center, verts, obstacle.moveDir, obstacle.speed);
            }
            if (obsHasEntered && isObstacleLeftField() && !obsHasLeft) {
                obsHasLeft = true;
                qDebug() << "Obstacle left field at iteration" << iteration;
            }
        } else {
            // 静态障碍物：使用预计算覆盖圆
            obsCircles = staticObsCoverCircles;
        }
        currentCoverCircles = obsCircles;  // 保存用于可视化
    }

    // 3. 计算Voronoi分区（OAVC）— 始终用全局场地边界
    lastRegions = computeOAVCRegions(positions, obsCircles);

    // 编队模式：用编队矩形裁剪全局Voronoi区域
    if (isFormationMode || isDualObstacleMode) {
        float formXmin = formationCenter.x - formationWidth / 2.0f;
        float formXmax = formationCenter.x + formationWidth / 2.0f;
        float formYmin = formationCenter.y - formationHeight / 2.0f;
        float formYmax = formationCenter.y + formationHeight / 2.0f;
        VoronoiPolygon formBox;
        formBox.vertices = {
            Vec2(formXmin, formYmin),
            Vec2(formXmax, formYmin),
            Vec2(formXmax, formYmax),
            Vec2(formXmin, formYmax)
        };
        for (auto& reg : lastRegions) {
            if (!reg.valid || reg.poly.vertices.size() < 3) continue;
            VoronoiPolygon clipped = sutherlandHodgmanClip(reg.poly, formBox);
            if (clipped.vertices.size() >= 3) {
                reg.poly = clipped;
                reg.poly.computeArea();
                reg.centroid = reg.poly.centroid();
            } else {
                reg.valid = false;
                reg.poly.vertices.clear();
            }
        }
    }

    // 4. 计算每个机器人的CVT控制 + 推波斥力
    bool allConverged = true;
    bool anyValid = false;
    frameMinDist = 1e9f;

    for (int i = 0; i < numRobots; ++i) {
        Vec2 p_r = positions[i];

        // CVT吸引力：指向质心
        Vec2 F_cvt(0, 0);
        if (lastRegions[i].valid) {
            F_cvt = (lastRegions[i].centroid - p_r) * kP;
        }

        // 推波斥力
        Vec2 F_rep(0, 0);
        if (isPushWaveRobotMode) {
            F_rep = computeRepulsiveForce(p_r, currentDanger);
        } else if (isDualObstacleMode) {
            // 双障碍物：动态障碍物斥力 + 静态障碍物斥力
            F_rep = computeRepulsiveForce(p_r, currentDanger);
            Vec2 F_rep_s = computeRepulsiveForce(p_r, staticObs2Danger);
            F_rep = F_rep + F_rep_s;
        } else if (isFormationMode) {
            F_rep = computeRepulsiveForce(p_r, currentDanger);
        } else if (obstacleEnabled && obstacle.speed > 0.001f && isObstacleInField()) {
            F_rep = computeRepulsiveForce(p_r, currentDanger);
        }

        // 总力
        Vec2 F_total;
        if (i == obstacleRobotIndex && (isDualObstacleMode || isPushWaveRobotMode)) {
            F_total = obstacleRobotMoveDir * obstacleRobotSpeed;
        } else if (isPushWaveRobotMode) {
            F_total = F_cvt + F_rep;  // 纯CVT+斥力
        } else if (isFormationMode || isDualObstacleMode) {
            // 编队前馈速度
            float dx = formationTargetCenter.x - formationCenter.x;
            bool reachedTarget = (std::abs(dx) < 0.001f);
            Vec2 v_feedforward = reachedTarget ? Vec2(0, 0) :
                formationMoveDir * formationSpeed * FEEDFORWARD_MULT;
            F_total = F_cvt + F_rep + v_feedforward;

            // 软边界约束：保持速度方向，只缩放幅度防止越界
            float margin = robotRadius;
            float newX = p_r.x + F_total.x * dt;
            float newY = p_r.y + F_total.y * dt;
            float formXmin = formationCenter.x - formationWidth / 2.0f;
            float formXmax = formationCenter.x + formationWidth / 2.0f;
            float formYmin = formationCenter.y - formationHeight / 2.0f;
            float formYmax = formationCenter.y + formationHeight / 2.0f;
            // 只在越界时缩放速度，保持方向不变
            float scale = 1.0f;
            if (F_total.x > 0 && newX > formXmax - margin)
                scale = std::min(scale, (formXmax - margin - p_r.x) / (F_total.x * dt));
            if (F_total.x < 0 && newX < formXmin + margin)
                scale = std::min(scale, (formXmin + margin - p_r.x) / (F_total.x * dt));
            if (F_total.y > 0 && newY > formYmax - margin)
                scale = std::min(scale, (formYmax - margin - p_r.y) / (F_total.y * dt));
            if (F_total.y < 0 && newY < formYmin + margin)
                scale = std::min(scale, (formYmin + margin - p_r.y) / (F_total.y * dt));
            F_total = F_total * scale;
        } else {
            F_total = F_cvt + F_rep;
        }

        // 速度限幅
        float vNorm = F_total.norm();
        if (vNorm > maxVelocity) {
            F_total = F_total * (maxVelocity / vNorm);
        }

        // 发送速度命令到真实机器人
        {
            vector<int16_t> wheelSpeeds = velocityToWheelSpeeds(i, F_total, robots[i]->getOrientation());
            if (isDualObstacleMode && i == obstacleRobotIndex)
                qDebug() << "R6 wheelSpeeds:" << wheelSpeeds[0] << wheelSpeeds[1]
                         << "F=" << F_total.x << F_total.y << "orient=" << robots[i]->getOrientation();
            emit speedCommand(robots[i]->getId(), wheelSpeeds[0], wheelSpeeds[1], robots[i]->getColor());
        }

        // 性能指标：最近距离 + 恢复时间
        if (obstacleEnabled && (obstacle.speed > 0.001f || isFormationMode || isDualObstacleMode)) {
            float distToObs;
            if (isDualObstacleMode && i == obstacleRobotIndex) {
                distToObs = 1e9f;  // 跳过障碍物机器人自身
            } else if (isDualObstacleMode) {
                // 双障碍物：取距两个障碍物较近的距离
                float d1 = (p_r - dualObsStartPos).norm();
                float d2 = (p_r - staticObs2Center).norm();
                distToObs = std::min(d1, d2);
            } else {
                Vec2 obsCenter = isFormationMode ? staticObsCenter : obstacle.center;
                distToObs = (p_r - obsCenter).norm();
            }
            if (distToObs < minDistToObstacle) minDistToObstacle = distToObs;
            if ((i == 0 || (isDualObstacleMode && i == 0)) && distToObs < 1e8f) frameMinDist = distToObs;
            else if (distToObs < frameMinDist) frameMinDist = distToObs;

            // 扰动恢复计时
            if (i >= (int)wasDisturbed.size()) {
                wasDisturbed.resize(i + 1, false);
                recoveryCounters.resize(i + 1, 0);
            }
            bool disturbed = (F_rep.norm() > 0.01f);
            if (disturbed) {
                wasDisturbed[i] = true;
                recoveryCounters[i] = 0;
            } else if (wasDisturbed[i]) {
                recoveryCounters[i]++;
                // 检查是否回到质心（恢复完成）
                if (lastRegions[i].valid) {
                    float cdist = (p_r - lastRegions[i].centroid).norm();
                    if (cdist < CONVERGENCE_THRESHOLD) {
                        recoveryTimes.push_back(recoveryCounters[i] * dt);
                        wasDisturbed[i] = false;
                    }
                }
            }
        }

        // 收敛检测（双障碍物模式跳过障碍物机器人）
        if (lastRegions[i].valid && !(isDualObstacleMode && i == obstacleRobotIndex)) {
            anyValid = true;
            float dist = (p_r - lastRegions[i].centroid).norm();
            if (dist > CONVERGENCE_THRESHOLD) {
                allConverged = false;
            }
        }
    }
    if (!anyValid) allConverged = false;  // 没有有效区域 = 未收敛

    // 编队模式下：阶段0先收敛，阶段1开始运动
    if (isFormationMode || isDualObstacleMode) {
        if (formationPhase == 0) {
            // 收敛阶段：编队不移动，等待机器人收敛到框内
            if (convergedSteps >= CONVERGENCE_STABLE_STEPS) {
                formationPhase = 1;
                convergedSteps = 0;  // 清零，重新计数运动阶段收敛
                qDebug() << "Formation converged, starting movement at iteration" << iteration;
            }
        } else {
            // 运动阶段：编队中心向目标移动
            float dx = formationTargetCenter.x - formationCenter.x;
            float distToTarget = std::abs(dx);
            if (distToTarget > formationSpeed * dt) {
                formationCenter = formationCenter + formationMoveDir * formationSpeed * dt;
            } else {
                formationCenter = formationTargetCenter;
            }
        }
    }

    // 平均恢复时间
    if (!recoveryTimes.empty()) {
        float sum = 0;
        for (float t : recoveryTimes) sum += t;
        avgRecoveryTime = sum / recoveryTimes.size();
    }

    // 机器人间最小间距
    interRobotMinDist = 1e9f;
    for (int i = 0; i < numRobots; ++i) {
        for (int j = i+1; j < numRobots; ++j) {
            float d = (positions[i] - positions[j]).norm();
            if (d < interRobotMinDist) interRobotMinDist = d;
        }
    }

    // 覆盖率计算
    if (obstacleEnabled || isFormationMode || isDualObstacleMode) {
        float totalArea = 0;
        float fieldArea = fieldW * fieldH;
        for (auto& reg : lastRegions) {
            if (reg.valid && reg.poly.vertices.size() >= 3) {
                reg.poly.computeArea();  // 每次强制重算面积
                if (reg.poly.area > 1e-8f) totalArea += reg.poly.area;
            }
        }
        // 防止偶尔跳零：保持上次有效值
        if (totalArea > 1e-8f)
            coverageRatio = totalArea / fieldArea * 100.0f;
        // else: 不更新 coverageRatio，保持上一帧的值

        // 覆盖面积方差
        int n = 0; float mean = 0, sumSq = 0;
        for (auto& reg : lastRegions) {
            if (reg.valid && reg.poly.vertices.size() >= 3) {
                reg.poly.computeArea();
                if (reg.poly.area > 1e-8f) { mean += reg.poly.area; n++; }
            }
        }
        if (n > 1) {
            mean /= n;
            for (auto& reg : lastRegions)
                if (reg.valid && reg.poly.area > 1e-6f)
                    sumSq += (reg.poly.area - mean) * (reg.poly.area - mean);
            coverageVariance = sumSq / n;
        }
        // 成本函数 J = Σ(距质心距离²)
        costFunctionValue = 0;
        for (int i = 0; i < numRobots && i < (int)lastRegions.size(); ++i) {
            if (lastRegions[i].valid) {
                float d = (positions[i] - lastRegions[i].centroid).norm();
                costFunctionValue += d * d;
            }
        }
        // 障碍物穿越时的恢复时间：扰动后J回到基线的用时
        if (obstacleEnabled && (obsHasEntered || isFormationMode || isDualObstacleMode)) {
            if (costBaseline > 1e-6f) {
                float disturbance = costFunctionValue - costBaseline;
                if (disturbance > costMaxDisturb) costMaxDisturb = disturbance;
                if (!costDisturbed && disturbance > costBaseline) {
                    costDisturbed = true; costDisturbCount = 0;
                }
                if (costDisturbed) {
                    costDisturbCount++;
                    if (costFunctionValue < costBaseline * 1.3f) {
                        costRecoveryTime = costDisturbCount * dt;
                        costDisturbed = false;
                    }
                }
            }
        }
    } else {
        // 无障碍时更新基线
        costBaseline = costFunctionValue > 1e-6f ? costFunctionValue : costBaseline;
    }

    // 收敛判断：需连续稳定 CONVERGENCE_STABLE_STEPS 步才停止
    if (allConverged) {
        convergedSteps++;
    } else {
        convergedSteps = 0;
    }
    // 推波机器人模式：跟踪障碍物离开场地
    if (isPushWaveRobotMode && obstacleRobotIndex >= 0) {
        float halfSide = obstacleRobotSide / 2.0f;
        if (dualObsStartPos.x > fieldX0 && dualObsStartPos.x < fieldX0 + fieldW)
            obsHasEntered = true;
        if (obsHasEntered && dualObsStartPos.x + halfSide < fieldX0)
            obsHasLeft = true;
    }
    bool dynamicObstacle = (obstacleEnabled && obstacle.speed > 0.001f);
    bool obstacleCompleted = isFormationMode || isDualObstacleMode || isPushWaveRobotMode
        || !dynamicObstacle || (obsHasEntered && obsHasLeft);

    // 编队/双障碍物模式：编队到达目标 + 机器人收敛 → 停止
    bool formationCompleted = !(isFormationMode || isDualObstacleMode || isPushWaveRobotMode) ||
        (formationCenter.x >= formationTargetCenter.x - 0.01f &&
         formationCenter.y >= formationTargetCenter.y - 0.01f);

    if (convergedSteps >= CONVERGENCE_STABLE_STEPS && obstacleCompleted && formationCompleted) {
        qDebug() << "Voronoi converged at iteration" << iteration;
        return true;
    }

    return false;
}

void ZooidVoronoi::computeAllVelocities(vector<Zooid*>& robots, float time)
{
    computeControlStep(robots, time);
}

bool ZooidVoronoi::checkAllConverged(const vector<Zooid*>& robots) const
{
    // 简化版：检查所有机器人速度是否接近0且靠近质心
    // 完整版需要维护每个机器人的 convergedSteps 历史
    for (int i = 0; i < robots.size() && i < lastRegions.size(); ++i) {
        if (!lastRegions[i].valid) continue;
        Vec2 pos = toVec2(robots[i]->getPosition());
        float dist = (pos - lastRegions[i].centroid).norm();
        if (dist > CONVERGENCE_THRESHOLD) {
            return false;
        }
    }
    return true;
}

void ZooidVoronoi::reset()
{
    iteration = 0;
    coverageRatio = 0.0f;
    convergedSteps = 0;
    minDistToObstacle = 1e9f;
    costFunctionValue = 0;
    costBaseline = 0;
    costRecoveryTime = 0;
    costMaxDisturb = 0;
    costDisturbed = false;
    costDisturbCount = 0;
    interRobotMinDist = 1e9f;
    recoveryTimes.clear();
    avgRecoveryTime = 0;
    wasDisturbed.clear();
    recoveryCounters.clear();
    obsHasLeft = false;
    obsHasEntered = false;
    waitStepsAfterObsLeft = 0;
    lastRegions.clear();
    currentCoverCircles.clear();
    // 恢复障碍物初始状态
    obstacle.center = obstacleInitialCenter;
    obstacle.speed = obstacleInitialSpeed;
    // 恢复编队初始状态
    formationCenter = formationStartCenter;
    formationPhase = 0;
    isDualObstacleMode = false;
    isPushWaveRobotMode = false;
    obstacleRobotIndex = -1;
    obstacleRobotIndex = -1;
}

// ==================== OAVC 核心算法 ====================
vector<OAVCRegion> ZooidVoronoi::computeOAVCRegions(const vector<Vec2>& positions,
                                                      const vector<CoverCircle>& obsCircles,
                                                      const float* xr_override,
                                                      const float* yr_override)
{
    vector<OAVCRegion> regions;
    int n = positions.size();

    float xr[2] = {fieldX0, fieldX0 + fieldW};
    float yr[2] = {fieldY0, fieldY0 + fieldH};
    if (xr_override) { xr[0] = xr_override[0]; xr[1] = xr_override[1]; }
    if (yr_override) { yr[0] = yr_override[0]; yr[1] = yr_override[1]; }

    for (int i = 0; i < n; ++i) {
        regions.push_back(computeSingleOAVC(positions[i], positions, i, obsCircles));
    }

    return regions;
}

OAVCRegion ZooidVoronoi::computeSingleOAVC(const Vec2& myPos, const vector<Vec2>& allPositions,
                                            int myIndex, const vector<CoverCircle>& obsCircles)
{
    OAVCRegion result;

    // 步骤1：初始化为大的无界多边形（远超场地范围）
    // 这样半平面相交的结果就是"无界"的近似
    const float INF = 10000.0f;
    VoronoiPolygon cell;
    cell.vertices = {
        Vec2(-INF, -INF),
        Vec2(INF, -INF),
        Vec2(INF, INF),
        Vec2(-INF, INF)
    };
    cell.computeArea();

    // 步骤2：与所有其他机器人的Voronoi边相交
    for (int k = 0; k < allPositions.size(); ++k) {
        if (k == myIndex) continue;
        Vec2 pk = allPositions[k];

        // Voronoi 边是 myPos 和 pk 的垂直平分线
        float A = 2 * (pk.x - myPos.x);
        float B = 2 * (pk.y - myPos.y);
        float C = (myPos.x * myPos.x + myPos.y * myPos.y) -
                  (pk.x * pk.x + pk.y * pk.y);

        // 半平面相交
        cell = unboundedHalfplaneIntersect(cell, A, B, C);
        if (cell.isEmpty()) break;
    }

    // 步骤2.5：OAVC障碍物约束（离散圆半平面）
    for (const CoverCircle& obs : obsCircles) {
        float di = (myPos - obs.center).norm();
        float w = 2.0f * obs.radius * di - di * di;
        float A = 2.0f * (obs.center.x - myPos.x);
        float B = 2.0f * (obs.center.y - myPos.y);
        float C = (myPos.x * myPos.x + myPos.y * myPos.y) -
                  (obs.center.x * obs.center.x + obs.center.y * obs.center.y) + w;

        cell = unboundedHalfplaneIntersect(cell, A, B, C);
        if (cell.isEmpty()) break;
    }

    // 步骤3：用场地边界裁剪
    VoronoiPolygon boundary;
    boundary.vertices = {
        Vec2(fieldX0, fieldY0),
        Vec2(fieldX0 + fieldW, fieldY0),
        Vec2(fieldX0 + fieldW, fieldY0 + fieldH),
        Vec2(fieldX0, fieldY0 + fieldH)
    };
    boundary.computeArea();

    if (!cell.isEmpty()) {
        cell = sutherlandHodgmanClip(cell, boundary);
    }

    result.poly = cell;
    result.valid = !cell.isEmpty() && cell.area > 1e-4f;
    if (result.valid) {
        result.centroid = computeVoronoiPolygonCentroid(cell);
    } else {
        result.centroid = myPos;
    }

    return result;
}

// ==================== 半平面计算 ====================
VoronoiPolygon ZooidVoronoi::halfplane(float A, float B, float C, float xr[2], float yr[2])
{
    VoronoiPolygon result;

    // 使用网格采样找到半平面内的所有点
    vector<Vec2> allPoints;
    const int samples = 50;  // 增加采样数以获得更精确的边界

    for (int i = 0; i <= samples; ++i) {
        float x = xr[0] + (xr[1] - xr[0]) * i / samples;
        for (int j = 0; j <= samples; ++j) {
            float y = yr[0] + (yr[1] - yr[0]) * j / samples;
            if (A * x + B * y + C <= 1e-6f) {
                allPoints.push_back(Vec2(x, y));
            }
        }
    }

    // 添加边界线段与半平面的交点
    vector<Vec2> corners[4] = {
        {Vec2(xr[0], yr[0]), Vec2(xr[1], yr[0])},
        {Vec2(xr[1], yr[0]), Vec2(xr[1], yr[1])},
        {Vec2(xr[1], yr[1]), Vec2(xr[0], yr[1])},
        {Vec2(xr[0], yr[1]), Vec2(xr[0], yr[0])}
    };

    for (int e = 0; e < 4; ++e) {
        Vec2 p1 = corners[e][0];
        Vec2 p2 = corners[e][1];
        float den = A * (p2.x - p1.x) + B * (p2.y - p1.y);
        if (std::abs(den) > 1e-6f) {
            float t = -(A * p1.x + B * p1.y + C) / den;
            if (t >= 0 && t <= 1) {
                allPoints.push_back(Vec2(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y)));
            }
        }
    }

    if (allPoints.size() < 3) return result;

    // 使用凸包获得正确的多边形
    result.vertices = convexHull(allPoints);
    result.computeArea();

    return result;
}

// ==================== 无界半平面相交 ====================
VoronoiPolygon ZooidVoronoi::unboundedHalfplaneIntersect(const VoronoiPolygon& poly, float A, float B, float C)
{
    if (poly.isEmpty()) return poly;
    if (poly.vertices.empty()) return poly;

    VoronoiPolygon result;
    const auto& verts = poly.vertices;

    for (size_t i = 0; i < verts.size(); ++i) {
        Vec2 p1 = verts[i];
        Vec2 p2 = verts[(i + 1) % verts.size()];

        // 计算边的"内侧"判断：边中点在半平面内则保留
        Vec2 mid = (p1 + p2) * 0.5f;
        bool midInside = (A * mid.x + B * mid.y + C <= 1e-6f);

        // p1 在半平面内
        bool p1Inside = (A * p1.x + B * p1.y + C <= 1e-6f);
        // p2 在半平面内
        bool p2Inside = (A * p2.x + B * p2.y + C <= 1e-6f);

        if (p1Inside) {
            result.vertices.push_back(p1);
        }

        // 如果一个在内一个在外，求交点
        if (p1Inside != p2Inside) {
            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float den = A * dx + B * dy;
            if (std::abs(den) > 1e-6f) {
                float t = -(A * p1.x + B * p1.y + C) / den;
                result.vertices.push_back(Vec2(p1.x + t * dx, p1.y + t * dy));
            }
        }
    }

    result.computeArea();
    return result;
}

VoronoiPolygon ZooidVoronoi::VoronoiPolygonIntersect(const VoronoiPolygon& p1, const VoronoiPolygon& p2)
{
    if (p1.isEmpty()) return p1;
    if (p2.isEmpty()) return p2;

    // 简单的多边形裁剪
    VoronoiPolygon result = sutherlandHodgmanClip(p1, p2);

    // 确保结果是有效的多边形
    if (result.vertices.size() < 3 || result.area < 1e-6f) {
        return VoronoiPolygon();
    }

    return result;
}

Vec2 ZooidVoronoi::computeVoronoiPolygonCentroid(const VoronoiPolygon& poly) const
{
    return poly.centroid();
}

// ==================== 危险三角形与斥力 ====================
DangerTriangle ZooidVoronoi::computeForwardEdgeDangerTriangle(const Vec2& obsCenter,
                                                               const vector<Vec2>& obsVerts,
                                                               const Vec2& dir, float speed)
{
    DangerTriangle dt;
    int edges[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    float maxProj = -1e9f;
    int bestEdge = 0;

    for (int i = 0; i < 4; ++i) {
        Vec2 v1 = obsVerts[edges[i][0]];
        Vec2 v2 = obsVerts[edges[i][1]];
        Vec2 mid = (v1 + v2) * 0.5f;
        float proj = (mid - obsCenter).dot(dir);
        if (proj > maxProj) {
            maxProj = proj;
            bestEdge = i;
        }
    }

    dt.P1 = obsVerts[edges[bestEdge][0]];
    dt.P2 = obsVerts[edges[bestEdge][1]];
    dt.mid = (dt.P1 + dt.P2) * 0.5f;
    dt.length = speed * lengthSpeedRatio;
    dt.apex = dt.mid + dir * dt.length;

    return dt;
}

Vec2 ZooidVoronoi::computeRepulsiveForce(const Vec2& robotPos, const DangerTriangle& danger)
{
    Vec2 F_rep(0, 0);

    // 编队模式下用相对速度（障碍物相对编队向左逼近），否则用障碍物实际方向
    Vec2 dir = isFormationMode ? (formationMoveDir * -1.0f) : obstacle.moveDir;
    float D = danger.length;
    Vec2 startMid = danger.mid;
    float startHalfWidth = (danger.P1 - danger.P2).norm() / 2.0f;

    // 全局转局部坐标
    Vec2 dr = robotPos - startMid;
    float dxLocal = dir.x * dr.x + dir.y * dr.y;
    float dyLocal = -dir.y * dr.x + dir.x * dr.y;

    if (dxLocal > 0 && dxLocal <= D) {
        float currentHalfWidth = startHalfWidth * (1.0f - dxLocal / D);
        float dY = std::max(0.0f, std::abs(dyLocal) - currentHalfWidth);
        float mag = kRep / ((dY + 0.6f) * (dY + 0.6f));
        float signDy = (dyLocal > 0) ? 1.0f : -1.0f;

        // 局部力转全局
        float FrepLocalX = 0;
        float FrepLocalY = mag * signDy;
        F_rep.x = dir.x * FrepLocalX - dir.y * FrepLocalY;
        F_rep.y = dir.y * FrepLocalX + dir.x * FrepLocalY;
    }

    return F_rep;
}

// ==================== 速度转换 ====================
vector<int16_t> ZooidVoronoi::velocityToWheelSpeeds(int robotIndex, const Vec2& vCmd, float orientation)
{
    if (vCmd.norm() < 1e-6f) return {0, 0};

    // 约定：0°=正右，CCW为正，范围 [-180, 180]
    auto norm180 = [](float a) -> float {
        while (a > 180.0f) a -= 360.0f;
        while (a <= -180.0f) a += 360.0f;
        return a;
    };

    // 1. rawAngle：F_total方向（0°=右, CCW+）
    float rawAngle = norm180(std::atan2(vCmd.y, vCmd.x) * 180.0f / M_PI);

    // 2. 当前朝向：机器人约定(0°=上,CW)→CCW(0°=右,CCW)
    float currentAngle = norm180(90.0f - orientation);

    // 3. 选 rawAngle 或 rawAngle±180° 中离当前朝向近的
    float rawAngle_alt = norm180(rawAngle + 180.0f);
    float diff_raw = std::abs(norm180(rawAngle - currentAngle));
    float diff_alt = std::abs(norm180(rawAngle_alt - currentAngle));

    float targetAngle;
    float v_sign;
    if (diff_raw <= diff_alt) {
        targetAngle = rawAngle;
        v_sign = 1.0f;
    } else {
        targetAngle = rawAngle_alt;
        v_sign = -1.0f;
    }

    // 4. 朝向误差 → omega (CCW+)
    float headingError = norm180(targetAngle - currentAngle);
    float headingErrorAbs = std::abs(headingError);

    // 5. 线速度：大角度偏差自转时保留前向速度，边转边前进
    float v_mag = vCmd.norm();
    float omega, v;
    if (headingErrorAbs > SPIN_THRESHOLD) {
        v = v_mag * 0.3f * v_sign;  // 保留30%前向速度
        omega = (headingError > 0 ? 1.0f : -1.0f) * SPIN_SPEED;
    } else {
        omega = headingError * M_PI / 180.0f * K_turn;
        v = v_sign * v_mag;
    }

    // 6. 转换为左右轮速（m/s）
    float v_left = v - omega * (WHEEL_DISTANCE / 2.0f);
    float v_right = v + omega * (WHEEL_DISTANCE / 2.0f);

    // 7. 统一缩放到 MAX_WHEEL_SPEED 以内，保留差速比例
    float maxAbs = std::max(std::abs(v_left), std::abs(v_right));
    if (maxAbs * 1000.0f > MAX_WHEEL_SPEED) {
        float scale = MAX_WHEEL_SPEED / (maxAbs * 1000.0f);
        v_left *= scale;
        v_right *= scale;
    }

    // 8. 转换为 mm/s
    int16_t leftSpeed = static_cast<int16_t>(v_left * 1000.0f);
    int16_t rightSpeed = static_cast<int16_t>(v_right * 1000.0f);

    // 9. 最小速度死区（不抹平差速）
    if (leftSpeed > 0 && leftSpeed < MIN_WHEEL_SPEED)
        leftSpeed = MIN_WHEEL_SPEED;
    else if (leftSpeed < 0 && leftSpeed > -MIN_WHEEL_SPEED)
        leftSpeed = -MIN_WHEEL_SPEED;
    if (rightSpeed > 0 && rightSpeed < MIN_WHEEL_SPEED)
        rightSpeed = MIN_WHEEL_SPEED;
    else if (rightSpeed < 0 && rightSpeed > -MIN_WHEEL_SPEED)
        rightSpeed = -MIN_WHEEL_SPEED;

    return {leftSpeed, rightSpeed};
}

// ==================== 工具函数 ====================
float ZooidVoronoi::clamp(float value, float min, float max) const
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float ZooidVoronoi::distance(const Vec2& a, const Vec2& b) const
{
    return (a - b).norm();
}

// ==================== 凸包算法（Andrew's Monotone Chain）====================
vector<Vec2> convexHull(vector<Vec2> pts)
{
    if (pts.size() < 3) return pts;

    // 按 x 排序，相同时按 y
    sort(pts.begin(), pts.end(), [](const Vec2& a, const Vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });

    // 去重
    vector<Vec2> uniquePts;
    for (const auto& p : pts) {
        if (uniquePts.empty() ||
            std::abs(p.x - uniquePts.back().x) > 1e-6f ||
            std::abs(p.y - uniquePts.back().y) > 1e-6f) {
            uniquePts.push_back(p);
        }
    }
    pts = uniquePts;
    if (pts.size() < 3) return pts;

    vector<Vec2> lower, upper;
    for (const auto& p : pts) {
        while (lower.size() >= 2) {
            Vec2 a = lower[lower.size() - 2];
            Vec2 b = lower[lower.size() - 1];
            Vec2 c = p;
            float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (cross <= 0) lower.pop_back();
            else break;
        }
        lower.push_back(p);
    }

    for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
        const Vec2& p = *it;
        while (upper.size() >= 2) {
            Vec2 a = upper[upper.size() - 2];
            Vec2 b = upper[upper.size() - 1];
            Vec2 c = p;
            float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (cross <= 0) upper.pop_back();
            else break;
        }
        upper.push_back(p);
    }

    // 移除重复的端点
    lower.pop_back();
    upper.pop_back();

    vector<Vec2> hull = lower;
    hull.insert(hull.end(), upper.begin(), upper.end());
    return hull;
}

// ==================== Sutherland-Hodgman 多边形裁剪 ====================
VoronoiPolygon sutherlandHodgmanClip(const VoronoiPolygon& subject, const VoronoiPolygon& clipPoly)
{
    if (subject.isEmpty()) return subject;
    if (clipPoly.isEmpty()) return VoronoiPolygon();

    VoronoiPolygon output = subject;
    const auto& clipVerts = clipPoly.vertices;

    if (clipVerts.size() < 3) return output;

    for (size_t i = 0; i < clipVerts.size(); ++i) {
        if (output.isEmpty()) return output;

        Vec2 edgeStart = clipVerts[i];
        Vec2 edgeEnd = clipVerts[(i + 1) % clipVerts.size()];

        VoronoiPolygon input = output;
        output.vertices.clear();

        for (size_t j = 0; j < input.vertices.size(); ++j) {
            Vec2 current = input.vertices[j];
            Vec2 next = input.vertices[(j + 1) % input.vertices.size()];

            float currentSide = (edgeEnd.x - edgeStart.x) * (current.y - edgeStart.y) -
                               (edgeEnd.y - edgeStart.y) * (current.x - edgeStart.x);
            float nextSide = (edgeEnd.x - edgeStart.x) * (next.y - edgeStart.y) -
                            (edgeEnd.y - edgeStart.y) * (next.x - edgeStart.x);

            bool currentInside = currentSide >= 0;
            bool nextInside = nextSide >= 0;

            if (currentInside && nextInside) {
                // 两个点都在内部：只保留下一个点
                output.vertices.push_back(next);
            } else if (currentInside && !nextInside) {
                // 当前在内，下一个在外：求交点并保留
                float dx = next.x - current.x;
                float dy = next.y - current.y;
                float ex = edgeEnd.x - edgeStart.x;
                float ey = edgeEnd.y - edgeStart.y;
                float den = dx * ey - dy * ex;  // (E-S) × (E2-E1)
                if (std::abs(den) > 1e-6f) {
                    float t = ((edgeStart.x - current.x) * ey - (edgeStart.y - current.y) * ex) / den;
                    output.vertices.push_back(Vec2(current.x + t * dx, current.y + t * dy));
                }
            } else if (!currentInside && nextInside) {
                // 当前在外，下一个在内：求交点，保留交点和下一个点
                float dx = next.x - current.x;
                float dy = next.y - current.y;
                float ex = edgeEnd.x - edgeStart.x;
                float ey = edgeEnd.y - edgeStart.y;
                float den = dx * ey - dy * ex;  // (E-S) × (E2-E1)
                if (std::abs(den) > 1e-6f) {
                    float t = ((edgeStart.x - current.x) * ey - (edgeStart.y - current.y) * ex) / den;
                    output.vertices.push_back(Vec2(current.x + t * dx, current.y + t * dy));
                }
                output.vertices.push_back(next);
            }
            // 两个都在外：不保留任何点
        }
    }

    output.computeArea();
    return output;
}

VoronoiPolygon ZooidVoronoi::getRobotRegion(int robotIndex) const
{
    if (robotIndex >= 0 && robotIndex < lastRegions.size()) {
        return lastRegions[robotIndex].poly;
    }
    return VoronoiPolygon();
}
