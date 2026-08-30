#ifndef LVORONOIVIEWBOX_H
#define LVORONOIVIEWBOX_H

#pragma once

#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QColor>
#include <vector>

#include "../manager/ZooidVoronoi.h"

class LVoronoiViewBox : public QWidget
{
    Q_OBJECT

public:
    explicit LVoronoiViewBox(QWidget *parent = nullptr);
    ~LVoronoiViewBox();

    // 设置缩放比例（像素/米）
    void setScale(float scale);

    // 设置场地边界（米）
    void setFieldBounds(float x0, float y0, float width, float height);

    // 更新可视化数据
    void updateVoronoiDisplay(
        const std::vector<VoronoiPolygon>& regions,
        const std::vector<QColor>& robotColors,
        const std::vector<Vec2>& robotPositions,
        const std::vector<Vec2>& centroids,
        const std::vector<Vec2>& obstacleVerts,
        const std::vector<CoverCircle>& coverCircles,
        const DangerTriangle& dangerTriangle,
        float coverageRatio,
        int iteration
    );

    // 清除显示
    void clear();

    // 设置编队显示
    void setFormationDisplay(bool active, Vec2 center, float w, float h);

    // 设置性能指标
    void setMetrics(float minDist, float avgRecovery, float variance);
    void pushCostValue(float cost);
    void pushMinDistValue(float d);
    void pushInterRobotDistValue(float d);   // 推入本帧最近距离
    void setSecondDanger(const DangerTriangle& d);
    void setObstacleVerts2(const std::vector<Vec2>& v) { m_obstacleVerts2 = v; }

    // 导出覆盖率数据到文件
    void exportCoverageData(const QString& filepath);

    // 弹出覆盖率曲线窗口（白底明亮风格）
    void showCoverageChart();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    // 坐标转换：世界坐标 -> 屏幕坐标
    QPointF worldToScreen(float wx, float wy) const;

    // 绘制 Voronoi 区域
    void drawVoronoiRegions(QPainter& painter);

    // 绘制质心
    void drawCentroids(QPainter& painter);

    // 绘制障碍物
    void drawObstacle(QPainter& painter);

    // 绘制 OAVC 覆盖圆
    void drawCoverCircles(QPainter& painter);

    // 绘制危险三角形
    void drawDangerTriangle(QPainter& painter);

    // 绘制机器人
    void drawRobots(QPainter& painter);

    // 绘制编队边界
    void drawFormation(QPainter& painter);

    // 绘制覆盖率曲线
    void drawCoverageCurve(QPainter& painter);

    // 缩放比例
    float m_scale;

    // 场地边界
    float m_fieldX0, m_fieldY0, m_fieldW, m_fieldH;

    // 可视化数据
    std::vector<VoronoiPolygon> m_regions;
    std::vector<QColor> m_robotColors;
    std::vector<Vec2> m_robotPositions;
    std::vector<Vec2> m_centroids;
    std::vector<Vec2> m_obstacleVerts;
    std::vector<Vec2> m_obstacleVerts2;  // 第二障碍物（静态）
    std::vector<CoverCircle> m_coverCircles;
    DangerTriangle m_dangerTriangle;
    DangerTriangle m_dangerTriangle2;
    std::vector<float> m_interDistHistory;

    // 编队显示
    bool m_showFormation = false;
    Vec2 m_formationCenter;
    float m_formationW = 0;
    float m_formationH = 0;

    // 性能指标
    float m_minDist = 0;
    float m_avgRecovery = 0;
    float m_variance = 0;
    bool m_hasMetrics = false;

    // 覆盖率历史
    std::vector<float> m_coverageHistory;
    std::vector<float> m_varianceHistory;
    std::vector<float> m_costHistory;
    std::vector<float> m_minDistHistory;
    std::vector<int> m_iterationHistory;
    float m_currentCoverage = 0;
    int m_currentIteration = 0;

    bool m_hasData;
};

#endif // LVORONOIVIEWBOX_H
