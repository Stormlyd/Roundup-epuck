#include "LVoronoiViewBox.h"
#include "../manager/ZooidVoronoi.h"
#include <QPainter>
#include <QPainterPath>
#include <QFile>
#include <QTextStream>
#include <QDialog>
#include <QVBoxLayout>
#include <QtMath>
#include <QDebug>
#include <cmath>

LVoronoiViewBox::LVoronoiViewBox(QWidget *parent)
    : QWidget(parent)
    , m_scale(500.0f)  // 默认 500 像素/米
    , m_fieldX0(0), m_fieldY0(0), m_fieldW(1.46f), m_fieldH(0.914f)
    , m_currentCoverage(0)
    , m_currentIteration(0)
    , m_hasData(false)
{
    setAttribute(Qt::WA_OpaquePaintEvent);
    setMinimumSize(200, 200);
}

LVoronoiViewBox::~LVoronoiViewBox()
{
}

void LVoronoiViewBox::setScale(float scale)
{
    m_scale = scale;
    update();
}

void LVoronoiViewBox::setFieldBounds(float x0, float y0, float width, float height)
{
    m_fieldX0 = x0;
    m_fieldY0 = y0;
    m_fieldW = width;
    m_fieldH = height;
    update();
}

QPointF LVoronoiViewBox::worldToScreen(float wx, float wy) const
{
    float screenX = (wx - m_fieldX0) * m_scale;
    float screenY = (m_fieldY0 + m_fieldH - wy) * m_scale;
    return QPointF(screenX, screenY);
}

void LVoronoiViewBox::updateVoronoiDisplay(
    const std::vector<VoronoiPolygon>& regions,
    const std::vector<QColor>& robotColors,
    const std::vector<Vec2>& robotPositions,
    const std::vector<Vec2>& centroids,
    const std::vector<Vec2>& obstacleVerts,
    const std::vector<CoverCircle>& coverCircles,
    const DangerTriangle& dangerTriangle,
    float coverageRatio,
    int iteration)
{
    // qDebug() << "LVoronoiViewBox::updateVoronoiDisplay called: regions=" << regions.size();
    m_regions = regions;
    m_robotColors = robotColors;
    m_robotPositions = robotPositions;
    m_centroids = centroids;
    m_obstacleVerts = obstacleVerts;
    m_coverCircles = coverCircles;
    m_dangerTriangle = dangerTriangle;
    m_currentCoverage = coverageRatio;
    m_currentIteration = iteration;
    m_hasData = true;

    // 更新覆盖率历史（过滤跳零）
    float cov = coverageRatio;
    if (cov < 0.1f && !m_coverageHistory.empty())
        cov = m_coverageHistory.back();  // 用上一帧值替代零
    m_coverageHistory.push_back(cov);
    m_iterationHistory.push_back(iteration);

    // 限制历史长度
    if (m_coverageHistory.size() > 500) {
        m_coverageHistory.erase(m_coverageHistory.begin());
        m_iterationHistory.erase(m_iterationHistory.begin());
    }

    update();
}

void LVoronoiViewBox::setFormationDisplay(bool active, Vec2 center, float w, float h)
{
    m_showFormation = active;
    m_formationCenter = center;
    m_formationW = w;
    m_formationH = h;
}

void LVoronoiViewBox::setMetrics(float minDist, float avgRecovery, float variance)
{
    m_minDist = minDist;
    m_avgRecovery = avgRecovery;
    m_variance = variance;
    m_hasMetrics = true;
    m_varianceHistory.push_back(variance);
    if (m_varianceHistory.size() > 500) m_varianceHistory.erase(m_varianceHistory.begin());
}

void LVoronoiViewBox::setSecondDanger(const DangerTriangle& d) { m_dangerTriangle2 = d; }

void LVoronoiViewBox::pushCostValue(float cost)
{
    m_costHistory.push_back(cost);
    if (m_costHistory.size() > 500) m_costHistory.erase(m_costHistory.begin());
}
void LVoronoiViewBox::pushMinDistValue(float d)
{
    m_minDistHistory.push_back(d);
    if (m_minDistHistory.size() > 500) m_minDistHistory.erase(m_minDistHistory.begin());
}
void LVoronoiViewBox::pushInterRobotDistValue(float d)
{
    m_interDistHistory.push_back(d);
    if (m_interDistHistory.size() > 500) m_interDistHistory.erase(m_interDistHistory.begin());
}

void LVoronoiViewBox::exportCoverageData(const QString& filepath)
{
    QFile file(filepath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&file);
    out << "Iteration,Coverage%\n";
    for (size_t i = 0; i < m_coverageHistory.size(); ++i) {
        out << m_iterationHistory[i] << "," << m_coverageHistory[i] << "\n";
    }
    file.close();
    qDebug() << "Coverage data exported to" << filepath;
}

void LVoronoiViewBox::clear()
{
    m_hasData = false;
    m_showFormation = false;
    m_regions.clear();
    m_robotColors.clear();
    m_robotPositions.clear();
    m_centroids.clear();
    m_obstacleVerts.clear();
    m_coverCircles.clear();
    m_coverageHistory.clear();
    m_varianceHistory.clear();
    m_costHistory.clear();
    m_iterationHistory.clear();
    m_currentCoverage = 0;
    m_currentIteration = 0;
    update();
}

void LVoronoiViewBox::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    // qDebug() << "LVoronoiViewBox::paintEvent: m_hasData=" << m_hasData << ", size=" << width() << "x" << height();

    // 检查窗口是否有效
    if (width() <= 0 || height() <= 0) {
        // qDebug() << "LVoronoiViewBox::paintEvent: window not ready, skipping";
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.save();

    // 填充背景
    painter.fillRect(rect(), QColor("#101129"));

    if (!m_hasData) {
        // 无数据时显示提示
        // qDebug() << "LVoronoiViewBox::paintEvent: showing 'Waiting for Voronoi data...'";
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Waiting for Voronoi data...");
        painter.restore();
        return;
    }

    // 绘制场地边界
    QPen boundaryPen(Qt::white, 2);
    painter.setPen(boundaryPen);
    QRectF fieldRect = QRectF(worldToScreen(m_fieldX0, m_fieldY0),
                               QSizeF(m_fieldW * m_scale, m_fieldH * m_scale));
    painter.drawRect(fieldRect);

    // 绘制 Voronoi 覆盖控制可视化
    drawCoverCircles(painter);
    drawVoronoiRegions(painter);
    drawCentroids(painter);
    drawObstacle(painter);
    if (!m_obstacleVerts2.empty()) { using std::swap; swap(m_obstacleVerts, m_obstacleVerts2); drawObstacle(painter); swap(m_obstacleVerts, m_obstacleVerts2); }
    drawDangerTriangle(painter);
    if (m_dangerTriangle2.length > 0) {
        using std::swap; swap(m_dangerTriangle, m_dangerTriangle2);
        drawDangerTriangle(painter);
        swap(m_dangerTriangle, m_dangerTriangle2);
    }
    drawFormation(painter);
    drawRobots(painter);

    // 性能指标
    if (m_hasMetrics) {
        painter.setPen(QColor(255, 255, 100));
        QFont f = painter.font(); f.setPointSize(10); f.setBold(true); painter.setFont(f);
        float avgCov = 0; if(!m_coverageHistory.empty()){for(float v:m_coverageHistory)avgCov+=v;avgCov/=m_coverageHistory.size();}
        QString m = QString::fromUtf8("最短距离: %1mm | 恢复: %2s | 方差: %3cm² | 平均覆盖率: %4%")
            .arg(m_minDist * 1000, 0, 'f', 0)
            .arg(m_avgRecovery, 0, 'f', 1)
            .arg(m_variance * 10000, 0, 'f', 1)
            .arg(avgCov, 0, 'f', 1);
        painter.drawText(10, height() - 10, m);
    }

    painter.restore();
}

void LVoronoiViewBox::drawVoronoiRegions(QPainter& painter)
{
    painter.save();
    for (size_t i = 0; i < m_regions.size() && i < m_robotColors.size(); ++i) {
        const VoronoiPolygon& poly = m_regions[i];
        if (poly.vertices.size() < 3) continue;

        // 检查多边形是否有有效的坐标（不是无穷大或 NaN）
        bool hasValidCoords = true;
        for (const Vec2& v : poly.vertices) {
            if (!std::isfinite(v.x) || !std::isfinite(v.y) ||
                std::abs(v.x) > 1e6 || std::abs(v.y) > 1e6) {
                hasValidCoords = false;
                break;
            }
        }
        if (!hasValidCoords) {
            // qDebug() << "drawVoronoiRegions: skipping invalid polygon" << i;
            continue;
        }

        QPainterPath path;
        QPointF firstPt = worldToScreen(poly.vertices[0].x, poly.vertices[0].y);
        path.moveTo(firstPt);

        for (size_t j = 1; j < poly.vertices.size(); ++j) {
            QPointF pt = worldToScreen(poly.vertices[j].x, poly.vertices[j].y);
            path.lineTo(pt);
        }
        path.closeSubpath();

        // 半透明填充 (FaceAlpha=0.3)
        QColor fillColor = m_robotColors[i];
        fillColor.setAlpha(76);  // 0.3 * 255 ≈ 76
        painter.setBrush(fillColor);
        painter.setPen(QPen(m_robotColors[i], 1));
        painter.drawPath(path);
    }
    painter.restore();
}

void LVoronoiViewBox::drawCentroids(QPainter& painter)
{
    painter.save();
    QPen centroidPen(Qt::black, 2);
    painter.setPen(centroidPen);

    for (const Vec2& centroid : m_centroids) {
        if (centroid.x == 0 && centroid.y == 0) continue;

        // 检查质心是否在合理范围内（场地边界内或附近）
        float maxCoord = std::max(m_fieldW, m_fieldH) * 2;  // 允许一些超出
        if (std::abs(centroid.x) > maxCoord || std::abs(centroid.y) > maxCoord) {
            // qDebug() << "drawCentroids: skipping invalid centroid" << centroid.x << centroid.y;
            continue;
        }

        QPointF c = worldToScreen(centroid.x, centroid.y);

        // 绘制 X 标记
        float size = 6;
        painter.drawLine(QLineF(c.x() - size, c.y() - size, c.x() + size, c.y() + size));
        painter.drawLine(QLineF(c.x() - size, c.y() + size, c.x() + size, c.y() - size));
    }
    painter.restore();
}

void LVoronoiViewBox::drawObstacle(QPainter& painter)
{
    if (m_obstacleVerts.size() < 4) return;

    QPainterPath path;
    QPointF firstPt = worldToScreen(m_obstacleVerts[0].x, m_obstacleVerts[0].y);
    path.moveTo(firstPt);

    for (size_t i = 1; i < m_obstacleVerts.size(); ++i) {
        QPointF pt = worldToScreen(m_obstacleVerts[i].x, m_obstacleVerts[i].y);
        path.lineTo(pt);
    }
    path.closeSubpath();

    // 红色填充
    painter.setBrush(QColor(255, 100, 100, 50));
    QPen obsPen(Qt::red, 2.5);
    painter.setPen(obsPen);
    painter.drawPath(path);
}

void LVoronoiViewBox::drawCoverCircles(QPainter& painter)
{
    painter.save();
    // OAVC 圆 - 绿色 (0.2, 0.7, 0.3), FaceAlpha=0.15
    QColor circleColor = QColor(51, 179, 76, 38);  // 0.15 * 255 ≈ 38
    painter.setBrush(circleColor);
    painter.setPen(Qt::NoPen);

    for (const CoverCircle& circle : m_coverCircles) {
        QPointF center = worldToScreen(circle.center.x, circle.center.y);
        float radius = circle.radius * m_scale;
        painter.drawEllipse(center, radius, radius);
    }
    painter.restore();
}

void LVoronoiViewBox::drawDangerTriangle(QPainter& painter)
{
    if (m_dangerTriangle.length <= 0) return;

    const Vec2& P1 = m_dangerTriangle.P1;
    const Vec2& P2 = m_dangerTriangle.P2;
    const Vec2& apex = m_dangerTriangle.apex;

    QPainterPath path;
    path.moveTo(worldToScreen(P1.x, P1.y));
    path.lineTo(worldToScreen(P2.x, P2.y));
    path.lineTo(worldToScreen(apex.x, apex.y));
    path.closeSubpath();

    // 半透明填充 (1, 0.8, 0.8), FaceAlpha=0.2
    QColor fillColor = QColor(255, 204, 204, 51);  // 0.2 * 255 ≈ 51
    painter.setBrush(fillColor);

    // 红色虚线边框
    QPen triPen(Qt::red, 1.5, Qt::DashLine);
    painter.setPen(triPen);
    painter.drawPath(path);
}

void LVoronoiViewBox::drawFormation(QPainter& painter)
{
    if (!m_showFormation) return;

    QPointF tl = worldToScreen(m_formationCenter.x - m_formationW / 2.0f,
                                m_formationCenter.y + m_formationH / 2.0f);
    QPointF br = worldToScreen(m_formationCenter.x + m_formationW / 2.0f,
                                m_formationCenter.y - m_formationH / 2.0f);
    QRectF formRect(tl, br);

    QPen formPen(QColor(100, 150, 255), 2, Qt::DashLine);
    painter.setPen(formPen);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(formRect);
}

void LVoronoiViewBox::drawRobots(QPainter& painter)
{
    float robotScreenRadius = 0.013f * m_scale;  // 13mm 半径

    for (size_t i = 0; i < m_robotPositions.size() && i < m_robotColors.size(); ++i) {
        const Vec2& pos = m_robotPositions[i];
        QPointF center = worldToScreen(pos.x, pos.y);

        // 机器人本体
        painter.setBrush(m_robotColors[i]);
        painter.setPen(QPen(Qt::black, 1));
        painter.drawEllipse(center, robotScreenRadius, robotScreenRadius);

        // 机器人编号
        painter.setPen(Qt::white);
        QFont font = painter.font();
        font.setPointSize(8);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(center, QString::number(i + 1));
    }
}

void LVoronoiViewBox::drawCoverageCurve(QPainter& painter)
{
    if (m_coverageHistory.size() < 2) return;

    // 底部大幅覆盖率曲线图
    float curveWidth = width() * 0.95f;
    float curveHeight = height() * 0.25f;
    float margin = 10;
    QRectF curveRect(margin, height() - curveHeight - margin - 25, curveWidth, curveHeight);

    // 背景框
    painter.fillRect(curveRect, QColor(0, 0, 0, 150));
    QPen framePen(Qt::white, 1);
    painter.setPen(framePen);
    painter.drawRect(curveRect);

    // 绘制曲线
    if (m_coverageHistory.size() >= 2) {
        QPainterPath path;
        float xStep = curveWidth / qMax((float)m_coverageHistory.size() - 1, 1.0f);

        for (size_t i = 0; i < m_coverageHistory.size(); ++i) {
            float x = curveRect.left() + i * xStep;
            float y = curveRect.bottom() - (m_coverageHistory[i] / 100.0f) * curveHeight;
            if (i == 0) {
                path.moveTo(x, y);
            } else {
                path.lineTo(x, y);
            }
        }

        QPen curvePen(QColor(255, 0, 255), 1.5);  // 洋红色曲线
        painter.setPen(curvePen);
        painter.drawPath(path);
    }

    // 显示数值
    painter.setPen(Qt::white);
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);
    QString coverageText = QString("Coverage: %1%").arg(m_currentCoverage, 0, 'f', 1);
    painter.drawText(curveRect.left(), curveRect.bottom() + 15, coverageText);
}

void LVoronoiViewBox::showCoverageChart()
{
    if (m_coverageHistory.size() < 2) return;

    QDialog* dlg = new QDialog(nullptr);
    dlg->setWindowTitle(QString::fromUtf8("推波性能分析"));
    dlg->resize(900, 550);
    dlg->setStyleSheet("background-color:white;");

    class ChartWidget : public QWidget {
    public:
        const std::vector<float> *costH, *distH, *interH;
        ChartWidget(QWidget* p) : QWidget(p) { setMinimumSize(860, 500); }
        void paintEvent(QPaintEvent*) override {
            QPainter p(this); p.fillRect(rect(), Qt::white);
            if (!costH || !distH || costH->size()<2) return;
            float cx=80, cy=30, cw=width()-190, ch=height()-110;
            QRectF cr(cx,cy,cw,ch);
            p.setPen(QPen(QColor(200,200,200),1)); p.drawRect(cr);
            QFont f; f.setPointSize(9); p.setFont(f);
            int n=(int)costH->size();
            float xs=cw/std::max(n-1,1);

            // 左Y轴：方差 cm² (红)
            float jmax=0.001f; for(float v:*costH)if(v>jmax)jmax=v;
            p.setPen(QColor(220,50,50));
            for(float v=0;v<=jmax*1.05f;v+=jmax*0.25f){float y=cr.bottom()-(v/jmax)*ch;p.drawText(QRectF(0,y-8,cx-8,16),Qt::AlignRight|Qt::AlignVCenter,QString::number(v*10000,'f',1));}
            p.save();p.translate(12,cr.center().y());p.rotate(-90);p.drawText(QRectF(-60,-10,120,20),Qt::AlignCenter,QString::fromUtf8("方差 cm²"));p.restore();

            // 右Y轴：距离 mm（上限1.5m）
            float dmax=0.001f; for(float v:*distH)if(v>dmax&&v<10)dmax=v;
            if(interH)for(float v:*interH)if(v>dmax&&v<10)dmax=v;
            dmax *= 2.5f;  // 留出上方空间
            if(dmax<0.1f)dmax=1.5f;
            p.setPen(QColor(50,100,220));
            for(float v=0;v<=dmax*1.05f;v+=dmax*0.25f){float y=cr.bottom()-(v/dmax)*ch;p.drawText(QRectF(cr.right()+8,y-8,80,16),Qt::AlignLeft|Qt::AlignVCenter,QString::number((int)(v*1000)));}
            p.save();p.translate(width()-12,cr.center().y());p.rotate(90);p.drawText(QRectF(-50,-10,100,20),Qt::AlignCenter,QString::fromUtf8("距离 mm"));p.restore();

            // 图例
            p.setPen(QColor(50,100,220));
            p.drawText(cr.left()+10,cy-10,QString::fromUtf8("— 障碍物距离"));
            if(interH&&interH->size()>=2){
                p.setPen(QColor(0,180,80));
                p.drawText(cr.left()+120,cy-10,QString::fromUtf8("- - 机间距离"));
            }

            // X轴
            p.setPen(Qt::black);
            p.drawText(QRectF(cx,cr.bottom()+8,cw,20),Qt::AlignCenter,QString::fromUtf8("迭代步数"));
            int step=std::max(1,n/8);
            for(int i=0;i<n;i+=step){p.drawText(QRectF(cr.left()+i*xs-20,cr.bottom()+26,40,16),Qt::AlignCenter,QString::number(i+1));}

            // 成本函数曲线(红)
            QPainterPath jp; for(size_t i=0;i<costH->size();++i){float x=cx+i*xs,y=cr.bottom()-((*costH)[i]/jmax)*ch;if(i==0)jp.moveTo(x,y);else jp.lineTo(x,y);}
            p.setPen(QPen(QColor(220,50,50),2));p.setRenderHint(QPainter::Antialiasing);p.drawPath(jp);

            // 最近距离曲线(蓝)
            QPainterPath dp; for(size_t i=0;i<distH->size();++i){float x=cx+i*xs,y=cr.bottom()-((*distH)[i]/dmax)*ch;if(i==0)dp.moveTo(x,y);else dp.lineTo(x,y);}
            p.setPen(QPen(QColor(50,100,220),2));p.drawPath(dp);

            // 机器人间最小距离(绿虚线)
            if(interH&&interH->size()>=2){QPainterPath ip;for(size_t i=0;i<interH->size();++i){float x=cx+i*xs,y=cr.bottom()-((*interH)[i]/dmax)*ch;if(i==0)ip.moveTo(x,y);else ip.lineTo(x,y);}
            p.setPen(QPen(QColor(0,180,80),2,Qt::DashLine));p.drawPath(ip);}
        }
    };
    auto* cw = new ChartWidget(dlg);
    cw->costH = &m_varianceHistory;  // 方差代替成本函数
    cw->distH = &m_minDistHistory;
    cw->interH = &m_interDistHistory;
    QVBoxLayout* lay = new QVBoxLayout(dlg); lay->addWidget(cw);
    dlg->show();
}
