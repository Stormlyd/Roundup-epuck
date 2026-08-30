#ifndef ZOOIDSIMULATOR_H
#define ZOOIDSIMULATOR_H
#pragma once

#include <cstdlib>
#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <QFrame>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QTimer>
#include <QPoint>

#include "public/config.h"
#include <QtWidgets>
#include <QtOpenGL>
#include <stdexcept>

#include "orca/Simulator.h"
#include "Zooid.h"

using namespace std;

class ZooidSimulator:public QGraphicsView, public Simulator
{
    Q_OBJECT
public:
    /**
     * @brief ZooidSimulator
     */
    ZooidSimulator();

    ~ZooidSimulator();

    /**
     * @brief 设置模拟器尺寸大小
     * @param width     要设置的宽度
     * @param height    要设置的高度
     */
    void setSize(int _width = 640, int _height = 480);

    /**
     * @brief 设置模拟器背景颜色
     * @param color 要设置的颜色
     */
    void setBackBackground(QColor color);

    /**
     * @brief 设置模拟器背景图片
     * @param pix
     */
    void setBackgroundImage(QPixmap *pix);

    /**
     * @brief 设置选择图形
     * @param pix
     */
    void setGraphicalImage(QPixmap *pix);
    /**
     * @brief 清除模拟器背景图
     */
    void clearBackgroundImage();

    /**
     * @brief setFollowImagePos
     * @param x
     * @param y
     */
    void setFollowImagePos(int x, int y);
    /**
     * @brief 添加Zooid样式到当前模拟器中
     * @param zooid 要添加的机器人
     */
    void addZooid(Zooid *zooid);

    /**
     * @brief 添加Zooid目标样式到当前模拟器中
     * @param zooid 要添加的机器人目标
     */
    void addZooidGoal(ZooidGoal *zooidGoal);

    /**
     * @brief 将当前的机器人从模拟器中移除
     * @param zooid 要移除的机器人
     */
    void removeZooid(Zooid *zooid);

    /**
     * @brief 移除机器人的目标位置
     * @param zooidGoal 要移除的机器人目标
     */
    void removeZooidGoal(ZooidGoal *zooidGoal);

    /**
     * @brief 获取模拟器宽度
     * @return 返回机模拟器的宽度
     */
    int getWidth();

    /**
     * @brief 获取模拟器高度
     * @return 返回机模拟器的宽
     */
    int getheight();

    /**
     * @brief 清空模拟器中所有内容
     */
    void clearAll();

    /**
     * @brief 模拟器放大
     */
    void zoomIn();

    /**
     * @brief 模拟器缩小
     */
    void zoomOut();

    /**
     * @brief 模拟器缩放倍数
     * @param level 要设置的缩放倍数
     */
    void zoomlevel(qreal level);

    /**
     * @brief 获取当前模拟器的缩放倍数
     * @return  返回缩放倍数
     */
    double getZoom();

    /**
     * @brief 使用openGl渲染
     */
    void setOpenGlView();

    /**
     * @brief 设置画布拖拽
     */
    void setDragScroll();
protected:
    virtual void mousePressEvent(QMouseEvent *event);

signals:
    /**
     * @brief 点击模拟器发送点击位置
     * @param x
     * @param y
     */
    void sendClickPosition(int x, int y);
private:

    QGraphicsScene *simulatorScene; //定义模拟器场景
    qreal zoom;                     //缩放倍数
    qreal zoomOld;                  //缩放倍数
    int width;                      //模拟器宽度
    int height;                     //模拟器高度
    QTimer * updateSceneTimer;      //刷新场景定时器

    QMutex m_mutex;

    QGraphicsPixmapItem *bgPath;    //背景路径

    QGraphicsPixmapItem *graphicalImg;

    QGraphicsPixmapItem *followImg;
private slots:

    /**
     * @brief 更新模拟器
     */
    void updateSimulator();

};

#endif // ZOOIDSIMULATOR_H


