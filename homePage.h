#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QScroller>
#include <QSqlDatabase>
#include <QSqlError>

#include "public/style.h"

#include "component/LSwitchButton.h"
#include "component/LMessageBox.h"
#include "component/LNumberInputBox.h"
#include "component/LPanel.h"
#include "component/LSliderBox.h"
#include "component/LFigureModelBox.h"
#include "component/LDrawModelbox.h"
#include "component/LoginBox.h"
#include "component/LCountdownBox.h"
#include "manager/ZooidManager.h"
#include "adminPage.h"
#include "./public/sqlconfig.h"

class AdminPage;
//class LMainMenu;

class HomePage : public QWidget
{
    Q_OBJECT
public:

    /**
     * @brief HomePage
     * @param parent
     */
    HomePage();

    ~HomePage();

    /**
     * @brief 初始化设置
     */
    void setup();

    /**
     * @brief 初始化界面
     */
    void initGUI();

    /**
     * @brief 更新图形模式
     */
    void updateGraphical();


    /**
     * @brief connectDB
     * @return  是否连接成功 成功true
     */
    bool connectDB();

    /**
     * @brief 读取数据库配置信息
     */
    void readSqlConfigData();

    /**
     * @brief 设置当前App的界面
     * @param page 当前页值
     */
    void setCurrentWidget(AppWidget widget);

    /**
     * @brief 获取当前显示界面
     * @return  返回当前选中界面值
     */
    AppWidget getCurrentWidget();

protected:
    void paintEvent(QPaintEvent *);
private slots:

    /**
     * @brief 数据更新
     */
    void dataUpdate();

    /** 启动一轮多机器人同步硬件测试。 */
    void startHardwareTest();

    /**
     * @brief 生成一键充电方案
     */
    void generateCharge();

    /**
     * @brief 生成三角形方案
     */
    void generateTriangle();

    /**
     * @brief 生成矩形方案
     */
    void generateReact();

    /**
     * @brief 生成十字方案
     */
    void generateCross();

    /**
     * @brief 生成六边形
     */
    void generateHexagon();

    /**
     * @brief 生成圆形方案
     */
    void generateCircul();

    /**
     * @brief 生成五角星方案
     */
    void generateFivepointed();

    /**
     * @brief 生成自绘方案
     */
    void generateDrawpath();

    /**
     * @brief 生成跟随方案
     */
    void generateFollowpath();

    /**
     * @brief 生成Voronoi覆盖控制方案
     */
    void generateVoronoipath();

    /**
     * @brief 启用虚拟机器人模式（用于无实体机器人时测试）
     */
    void enablePushWaveVoronoiMode();
    void enableFormationVoronoiMode();
    void enableDualObstacleMode();
    void enableCoverageOptimalMode();

    /**
     * @brief 停止所有模式，发送零速
     */
    void stopAllModes();

    /**
     * @brief 运行方案
     */
    bool runPlanning();

    /**
     * @brief 选择图形窗口
     */
    void selectGraphicalModal();

    /**
     * @brief 登录按钮单击事件
     */
    void onAdminBtnClicked();



private:
    void updateTestModeUi();

    int operateNum;                 //操作次数 用于向中控上报
    int zooidNumber;                //机器人在线数量
   // int screensaverCount;           //屏保计数器
    //int screensaverFlag;            //屏保播放计数器false 不在屏保中

    AppWidget appWidget;            //选中当前界面

    QSqlDatabase  m_db;
    AdminPage *admin = nullptr;     //后台管理界面

    ZooidManager zooidManager;      //机器人管理器
    LCountdownBox      *coutdown = nullptr;   //倒计时组件
    //LMoviePlayer *screensaverPlayer;//屏保界面

    QPushButton* graphicalModeBtn = nullptr;
    QPushButton* drawModeBtn = nullptr;
    QPushButton* followModeBtn = nullptr;
    QPushButton* voronoiModeBtn = nullptr;
    QPushButton* backButton = nullptr;
    QPushButton* pushWaveVoronoiBtn = nullptr;
    QPushButton* formationVoronoiBtn = nullptr;
    QPushButton* dualObstacleBtn = nullptr;
    QPushButton* coverageOptimalBtn = nullptr;
    QPushButton* testModeBtn = nullptr;
    QPushButton* stopBtn = nullptr;
    QLabel* testModeStatusLabel = nullptr;

    QPushButton* testCharge = nullptr;

    bool isGraphical;

    QTimer * updateTimer = nullptr;

    SqlConfigure  sqlConfig;

    friend  class AdminPage;
   // friend  class LMainMenu;


};

#endif // MAINWINDOW_H






















