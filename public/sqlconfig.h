#ifndef SQLCONFIG_H
#define SQLCONFIG_H
#include <QString>

typedef struct {
    bool EnabelFigureMode;  //图形模式表演
    bool EnabelDrawMode;    //自绘模式表演
    bool EnabelFollowMode;  //跟随模式表演
    bool goalDisplay;       //目标位置显示
    bool batteryDisplay;    //电量显示
    bool screensaverDisplay;//屏保显示
    int circularNumber;     //圆形表演个数
    int triangleNumber;     //三角形表演个数
    int reactNumber;        //矩形表演个数
    int crossNumber;        //十字表演个数
    int hexagonNumber;      //六边形表演个数
    int fivepointedNumber;  //五角星表演个数
    int FollowTime;         //跟随表演时长
    int DrawTime;           //自绘表演时长
    int FigureTime;         //图形表演时长
    QString password;       //后台登录密码
    float zoom;             //模拟器缩放倍数
    int useTime;            //正常使用时间(s)
    int waitTime;           //屏保等待时间(s)
    unsigned int battery;   //表演时电量最小值
    unsigned int showCount; //表演次数
}SqlConfigure;

#endif // SQLCONFIG_H
