#include "Zooid.h"
#include <QDebug>

Zooid::Zooid()
{
    id = 100;
    position = Vector2(0.0f, 0.0f);
    goalPosition = Vector2(0.0f, 0.0f);
    radius = 1.0f;
    orientation = 0.0f;
    state = NO_TOUCH;
    color = QColor(0,0,0);
    batteryLevel = 100;
    speed = 100;
    batteryShow = false;
    charge = false;
    activated = true;
    goalReached = false;
    watchdogCounter = 0;
    setZValue(id + 1);
    //设置图形的深度值（z）：z越大在越上层
    robotType = ZooidRobot;
    robotControlMode = SpeedControl;
    rotationPID.init(2.0f, 0.1f, 0.5f, 0.0f);

}

Zooid::Zooid(float _radius, Vector2 _position, bool _batteryShow)
{
    id = 0;
    position = _position;
    goalPosition = _position;
    radius = _radius;
    orientation = 0.0f;
    state = NO_TOUCH;
    color = QColor(0,0,0);
    batteryLevel = 100;
    speed = 100;
    batteryShow = _batteryShow;
    activated = true;
    goalReached = false;
    charge = false;
    watchdogCounter = 0;
    setZValue(id + 1);
    robotType = ZooidRobot;
    robotControlMode = SpeedControl;
    rotationPID.init(2.0f, 0.1f, 0.5f, 0.0f);
}

Zooid::~Zooid()
{

}

bool Zooid::operator == (const Zooid &r)
{
    return this->id == r.id;
    //判断id是否相等
}

bool Zooid::operator > (const Zooid &r)
{
    return this->batteryLevel > r.batteryLevel;
    //判断电量大小
}

bool Zooid::operator < (const Zooid &r)
{
    return this->batteryLevel < r.batteryLevel;
}

bool Zooid::operator != (const Zooid &r)
{
    return !(*this == r);
}

void Zooid::operator = (const Zooid &z )
{
    id = z.id;
    radius = z.radius;
    orientation = z.orientation;
    position = z.position;
    color = z.color;
    batteryLevel = z.batteryLevel;
    goalReached = z.goalReached;
}

QRectF Zooid::boundingRect() const
{
    return QRectF(0, 0, 70, 70);
}

QPainterPath Zooid::shape() const
{
    QPainterPath path;
    path.addRect(0,0, 50, 50);
    return path;
    //50*50矩形，实际碰撞区域，比刚才的boundingRect更精确
}

void Zooid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);

    switch(robotType)
    {
    case ZooidRobot:
        drawZooidRobot(painter);
        break;
    case BatonRobot:
        drawBatonRobot(painter);
        break;
    case AgentRobot:
        drawAgentRobot(painter);
        break;
    }
}

//绘制一个圆形的机器人图形，包含方向指示、电量显示和ID标识
void Zooid::drawZooidRobot(QPainter *painter)
{
    QColor fillColor = color; //color为zooid类里自己定义的颜色，省略this->
    painter->setRenderHint(QPainter::Antialiasing, true); //开启抗锯齿
    float orientation = this->orientation + 90; //加this是因为前面有个orientation

    painter->translate(35,35);  //移动坐标系到70*70矩形的中心
    painter->rotate(orientation);  //旋转坐标系

    painter->setBrush(Qt::gray);
    painter->drawRoundRect(-25, 0, 5, 20, 50, 50);
    painter->drawRoundRect(20, -20, 5, 20, 50, 50);

    //绘制外壳
    painter->setPen(Qt::black);
    painter->setBrush(Qt::white);
    painter->drawEllipse(-25,-25,50,50);

    //绘制外壳颜色
    painter->setBrush(QBrush(fillColor));
    painter->drawEllipse(-20,-20,40,40);

    //绘制方向标
    painter->setBrush(QColor("#68d8fe"));
    painter->drawRect(-2,-19,4, 10);

    painter->rotate(-orientation);

    if(batteryShow)
    {
        int x = 0;
        if(this->position.getX() * 1000 < 50)
        {
            x = 80;
        }

        painter->setPen(Qt::black);
        //绘制电池
        painter->drawRect(-42 + x, -37, 7, 1);
        painter->drawLine(-45 + x, -35, -32 + x, -35);
        painter->drawLine(-45 + x, -15, -32 + x, -15);
        painter->drawLine(-45 + x, -35, -45 + x, -15);
        painter->drawLine(-32 + x, -35, -32 + x, -15);

        //绘制电量
        QColor batteryColor;
        if(this->batteryLevel >= BATTERY_HIGH){
            batteryColor = Qt::green;
        }else if(this->batteryLevel >= BATTERY_LOW){
             batteryColor = Qt::yellow;
        }else {
            batteryColor = Qt::red;
        }

        int batteryLevel = (this->batteryLevel) / 100.0 * 18;


        painter->setBrush(batteryColor);
        painter->drawRect(-44 + x, -34 + (18 - batteryLevel), 11, batteryLevel);
    }

    //绘制id 字体居中
    painter->setPen(Qt::black);
    QFont font("黑体", 12, QFont::Bold);
    painter->setFont(font);
    QString id = QString::number(this->id, 10);
    int widthId = painter->fontMetrics().width(id) ;
    painter->drawText(- widthId / 2, 7, id);
}

void Zooid::drawBatonRobot(QPainter *painter)
{

    QColor fillColor = color;
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->translate(35,35);

    //绘制外壳
    painter->setPen(Qt::black);
    painter->setBrush(Qt::white);
    painter->drawEllipse(-25,-25,50,50);

    //绘制外壳颜色
    painter->setBrush(QBrush(Qt::white));
    painter->drawEllipse(-20,-20,40,40);
}

void Zooid::drawAgentRobot(QPainter *painter)
{
    QColor fillColor = color;
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->translate(35,35);

    //绘制外壳
    painter->setPen(Qt::black);
    painter->setBrush(color);
    painter->drawEllipse(-25,-25,50,50);

    //绘制外壳颜色
    painter->setBrush(QBrush(color));
    painter->drawEllipse(-20,-20,40,40);
}

void Zooid::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mousePressEvent(event);
    update();
}

void Zooid::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->modifiers() & Qt::ShiftModifier)
    {
        update();
        return;
    }
    QGraphicsItem::mouseMoveEvent(event);
}

void Zooid::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    QGraphicsItem::mouseReleaseEvent(event);
    update();
}

Vector2 Zooid::getPosition()
{
    return position;
}

Vector2 Zooid::getGoalPosition()
{
    return goalPosition;
}

float Zooid::getOrientation()
{
    return orientation;
}

unsigned int Zooid::getState()
{
    return state;
}

QColor Zooid::getColor()
{
    return color;
}

unsigned int Zooid::getId()
{
    return id;
}

Zooid::RobotControlMode Zooid::getRobotControlMode()
{
    return robotControlMode;
}

float Zooid::getRadius()
{
    return radius;
}

unsigned int Zooid::getBatteryLevel()
{
    return batteryLevel;
}

bool Zooid::isConnected()
{
    return watchdogCounter > 0;
}

bool Zooid::isActivated()
{
    return activated;
}

int Zooid::getCharge()
{
    return charge;
}

void Zooid::setCharge(int chagre)
{
    this->charge = chagre;
}

void Zooid::tickWatchdog()
{
    if(watchdogCounter != 0)
    {
        watchdogCounter--;
    }
    else
    {
        //batteryLevel = 100;
    }
}

void Zooid::resetWatchdog()
{
    watchdogCounter = ZOOID_WATCHDOG_TIMEOUT;
}

void Zooid::setPosition(Vector2 _pos)
{
    position = _pos;
}

void Zooid::setPosition(float _x, float _y)
{
    position.setX(_x);
    position.setY(_y);
}

void Zooid::setGoalPosition(Vector2 _pos)
{
    goalPosition = _pos;
}

void Zooid::setGoalPosition(float _x, float _y)
{
    goalPosition.setX(_x);
    goalPosition.setY(_y);
}

void Zooid::setGoalReached(bool _goalReached)
{
    goalReached = _goalReached;
}

void Zooid::setLastUpdate(long time)
{
    lastUpdate = time;
}

void Zooid::setRadius(float _radius)
{
    radius = _radius;
}

void Zooid::setOrientation(float _orientation)
{
    orientation = _orientation;
}

void Zooid::setColor(QColor _color)
{
    color = _color;
}

void Zooid::setState(unsigned int _state)
{
    state = _state;
}

void Zooid::setId(unsigned int _id)
{
    id = _id;
}

void Zooid::setBatteryLevel(unsigned int _battery)
{
    if(_battery > 100){
        _battery = 100;
    }

    batteryLevel = _battery;

}

void Zooid::setBatteryShow(bool _show)
{
    batteryShow = _show;
}

void Zooid::setRobotType(RobotType _robotType)
{
    this->robotType = _robotType;
}

void Zooid::setRobotControlMode(RobotControlMode _robotControlMode)
{
    this->robotControlMode = _robotControlMode;
}

void Zooid::activate()
{
    activated = true;
}

void Zooid::deactivate()
{
    activated = false;
}

void Zooid::setSpeed(unsigned int _speed)
{
    speed = _speed;
    if(speed>100)
    {
        speed = 100;
    }
}

unsigned int Zooid::getSpeed()
{
    return speed;
}

bool Zooid::isGoalReached()
{
    return goalReached;
}

bool Zooid::isTouched()
{
    return (state & 1) > 0;
}

bool Zooid::isBlinded()
{
    return (state & 2) > 0;
}

bool Zooid::isTapped()
{
    return (state & 4) > 0;
}

bool Zooid::isShaken()
{
    return (state & 8) > 0;
}

bool Zooid::isShowBattery()
{
    return batteryShow;
}











