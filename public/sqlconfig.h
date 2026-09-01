#ifndef SQLCONFIG_H
#define SQLCONFIG_H
#include <QString>

typedef struct {
    bool EnabelFigureMode = false;  //图形模式表演
    bool EnabelDrawMode = false;    //自绘模式表演
    bool EnabelFollowMode = false;  //跟随模式表演
    bool goalDisplay = true;        //目标位置显示
    bool batteryDisplay = true;     //电量显示
    bool screensaverDisplay = false;//屏保显示
    int circularNumber = 8;         //圆形表演个数
    int triangleNumber = 3;         //三角形表演个数
    int reactNumber = 4;            //矩形表演个数
    int crossNumber = 5;            //十字表演个数
    int hexagonNumber = 6;          //六边形表演个数
    int fivepointedNumber = 10;     //五角星表演个数
    int FollowTime = 60;            //跟随表演时长
    int DrawTime = 60;              //自绘表演时长
    int FigureTime = 60;            //图形表演时长
    QString password = QStringLiteral("admin"); //后台登录密码
    float zoom = 1.0f;              //模拟器缩放倍数
    int useTime = 0;                //正常使用时间(s)
    int waitTime = 300;             //屏保等待时间(s)
    unsigned int battery = 20;      //表演时电量最小值
    unsigned int showCount = 0;     //表演次数
}SqlConfigure;

#endif // SQLCONFIG_H
