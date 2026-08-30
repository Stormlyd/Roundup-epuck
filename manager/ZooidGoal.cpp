#include "ZooidGoal.h"

ZooidGoal::ZooidGoal()
{
    position = Vector2(0.0f, 0.0f);
    color = QColor(0,0,0); 
    goalShow = false;
    setZValue(0);
}

ZooidGoal::ZooidGoal(Vector2 _position, QColor _color, bool _show)
{
    position = _position;
    color = _color; 
    goalShow = _show;
    setZValue(0);
}

ZooidGoal::~ZooidGoal()
{
}

QRectF ZooidGoal::boundingRect() const
{
    return QRectF(0, 0, 70, 70);
}

QPainterPath ZooidGoal::shape() const
{
    QPainterPath path;
    path.addRect(0,0, 50, 50);
    return path;
}

void ZooidGoal::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    if(!goalShow)
    {
        return ;
    }

    Q_UNUSED(widget);
    QColor fillColor = color;
    fillColor.setAlpha(60);
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->translate(35,35);
    painter->setPen(fillColor);
    painter->setBrush(QBrush(fillColor));
    painter->drawEllipse(-25,-25,50,50);
}

void ZooidGoal::operator = (const ZooidGoal &g)
{
    position = g.position;
    color = g.color;
}

void ZooidGoal::setPosition(Vector2 _position)
{
    position = _position;
}

void ZooidGoal::setPosition(float _x, float _y)
{
    position.setX(_x);
    position.setY(_y);
}

void ZooidGoal::setColor(QColor _color)
{
    color = _color;
}

void ZooidGoal::setGoalShow(bool _show)
{
    goalShow = _show;
}

Vector2 ZooidGoal::getPosition()
{
    return position;
}

QColor ZooidGoal::getColor()
{
    return color;
}

bool ZooidGoal::isGoalShow()
{
    return goalShow;
}

