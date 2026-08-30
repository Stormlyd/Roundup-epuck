#ifndef ZOOIDDARW_H
#define ZOOIDDARW_H

#pragma once

#include <iostream>
#include <algorithm>

#include <QWidget>
#include <QPixmap>
#include <QPoint>
#include <QPainter>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QSize>
#include "public/config.h"
#include "orca/Vector2.h"

using namespace std;
using namespace lzm;

class PositionData
{
public:
    PositionData () : value(0), has(false) { }
    Vector2 position;
    int value;
    bool has;

    void operator = (const PositionData &p) {
        this->has = p.has;
        this->position = p.position;
        this->value = p.value;
    }

    bool operator == (const PositionData &p) {
       return (*this == p);
    }

    bool operator != (const PositionData &p) {
       return !(*this == p);
    }
};
class cmpPositionDataLess : public binary_function<PositionData, PositionData, bool>
{
public:
    bool operator()(const PositionData &p1, const PositionData &p2)
    {
        return p1.value < p2.value;
    }
};
class ZooidDraw : public QWidget
{
    Q_OBJECT
public:
    ZooidDraw(QWidget *parent= nullptr);
    ~ZooidDraw();

    //设置绘制路径尺寸大小
    void setSize(int width = 320, int height = 240);
    //设置绘制路径背景颜色
    void setBackBackground(QColor color);
    //获取绘制路径宽度
    int getWidth();
    //获取绘制路径高度
    int getheight();
    //设置画笔
    void setPen(QPen pen);
    //设置pix
    void setPixmap(QPixmap pixmap);
    void setPixmap(QPixmap pixmap, QColor fillColor);
    QPixmap getPixmap();
    void clearPixmap();
    //清除路径
    void clearPath();
    //计算
    void solve(int row, int col);
    void computvalue(int s, int e, int x, int y, int row, int col, int dep, int depth, int dir);
    void clearPositionData();
    //生成路径
    void generatePath();
    vector<Vector2> getPathPoints();
    QPoint getStartPoint();
    QPoint getEndPoint();
private:


    //路径画笔
    QPen pathPen;
    //显示路径
    QPixmap pathPixmap;
    //起始点
    QPoint startPoint;
    //结束点
    QPoint endPoint;
public:
    //位置矩阵
    PositionData posMatrix[100][100];
    //路径
    vector<Vector2> pathPoints;

protected:
    void paintEvent(QPaintEvent *event);
    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
};

#endif // ZOOIDDARW_H
