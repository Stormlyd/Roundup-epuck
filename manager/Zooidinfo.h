#ifndef ZOOIDINFO_H
#define ZOOIDINFO_H
#include <QColor>

#include "orca/Vector2.h"

using namespace lzm;

typedef struct ZooidInfoSut
{
    unsigned int id;                //id
    float radius;                   //半径
    float orientation;              //方向 0~360
    Vector2 position;               //位置
    unsigned int batteryLevel;      //电量
    unsigned int speed;             //速度
    QColor color;                   //Led颜色
    ZooidInfoSut()
    {
        id = 0;
        radius = 0.0f;
        orientation = 0.0f;
        position = lzm::Vector2(0.0f, 0.0f);
        batteryLevel = 0;
        speed = 0;
        color = Qt::black;
    }
} ZooidInfo;


#endif // ZOOIDINFO_H
