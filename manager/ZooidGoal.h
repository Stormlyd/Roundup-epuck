#ifndef ZOOIDGOAL_H
#define ZOOIDGOAL_H
#pragma once

#include <QColor>
#include <QObject>
#include <QPainter>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>

#include "orca/Vector2.h"

using namespace lzm;

class ZooidGoal : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     */
    ZooidGoal();

    /**
     * @brief Constructor
     * @param _position     目标位置
     * @param _color        目标颜色
     * @param _goalShow     目标显示状态
     */
    ZooidGoal(Vector2 _position, QColor _color, bool _goalShow);

    ~ZooidGoal();

    /**
     * @brief operator =
     * @param g
     */
    void operator = (const ZooidGoal &g);

    /**
     * @brief 设置位置
     * @param _position 要设置的位置值
     */
    void setPosition(Vector2 _position);

    /**
     * @brief 设置位置
     * @param _x    要设置的位置值x
     * @param _y    要设置的位置值y
     */
    void setPosition(float _x, float _y);

    /**
     * @brief 设置颜色
     * @param _color    要设置的颜色值(RGB)
     */
    void setColor(QColor _color);

    /**
     * @brief 设置目标显示状态
     * @param _show 显示状态值, true显示; false不显示
     */
    void setGoalShow(bool _show);


    /**
     * @brief 设置目标关联的机器人index
     * @param index 要关联的机器人index
     */
    void setAssociatedZooid(unsigned int index) {zooidIndex = index;}

    unsigned int getAssociatedZooid(){return zooidIndex;}

    /**
     * @brief 获取当前位置
     * @return  返回当前位置值
     */
    Vector2 getPosition();

    /**
     * @brief 获取当前颜色值
     * @return  返回颜色值
     */
    QColor getColor();

    /**
     * @brief 返回目标显示状态
     * @return
     */
    bool isGoalShow();

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

private:
    /**
     * @brief 位置信息
     */
    Vector2 position;

    /**
     * @brief 颜色
     */
    QColor color;

    /**
     * @brief 目标显示标记
     */
    bool goalShow;

    /**
     * @brief 目标关联的机器人编号
     */
    unsigned int zooidIndex;
};

#endif // ZOOIDGOAL_H
