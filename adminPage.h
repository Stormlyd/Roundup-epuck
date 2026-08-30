#ifndef ADMINPAGE_H
#define ADMINPAGE_H

#pragma once

#include <QWidget>
#include <QDialog>
#include <QPaintEvent>
#include <QPainter>
#include <QListWidget>
#include <QStandardItemModel>
#include <QtCore/QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QPushButton>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QSqlQuery>
#include <QSqlError>

#include "public/style.h"
#include "manager/ZooidManager.h"
#include "component/LPanel.h"
#include "component/LSwitchButton.h"
#include "component/LMessageBox.h"
#include "homePage.h"

class HomePage;

class AdminPage : /*public QWidget*/public QDialog
{
    Q_OBJECT
public:
    /**
     * @brief AdminPage
     * @param parent
     */
    explicit AdminPage(HomePage *_homePage);

    /**
     * @brief 设置
     */
    void setup();

    /**
     * @brief initGUI
     */
    void initGUI();


    /**
     * @brief   将机器人的信息转换为text
     * @param info  要转换的机器人信息
     * @return      转换后的text
     */
    QString  zooidInfoToText(ZooidInfo info);

protected:
    void paintEvent(QPaintEvent *);


public slots:
    /**
     * @brief 取消按钮单击事件
     */
    void onCancelClicked();

    /**
     * @brief 点击到机器人信息页
     */
    void onRobotoInfoClicked();

    /**
     * @brief 点击到表演配置页事件
     */
    void onPerformConfigClicked();

    /**
     * @brief 点击到运行日志管理页事件
     */
    void onRunLogBtnClicked();

    /**
     * @brief 单击系统设置按钮事件
     */
    void onSystemSetBtnClicked();

    /**
     * @brief 点击到密码管理页事件
     */
    void onPassManagerClicked();

    /**
     * @brief 更新机器人信息事件
     */
    void updateZooidList();

    /**
     * @brief 选择组成圆形机器人个数事件
     */
    void onSelectCircNumClicked();

    /**
     * @brief 选择组三角形机器人个数事件
     */
    void onTriangleNumClicked();

    /**
     * @brief 选择组矩形机器人个数事件
     */
    void onRectangleNumClicked();

    /**
     * @brief 模拟器缩放按钮单击事件+
     */
    void onZoomInBtnClicked();

    /**
     * @brief 模拟器缩放按钮单击事件-
     */
    void onZoomOutBtnClicked();

    /**
     * @brief 选择组十字机器人个数事件
     */
    void onCrossNumClicked();

    /**
     * @brief 保存图形配置信息事件
     */
    void onSaveFigureBtnClicked();

    /**
     * @brief 保存跟随模式配置信息事件
     */
    void onSaveFollowModeBtnClicked();

    /**
     * @brief 保存自绘模式配置信息事件
     */
    void onSaveDrawBtnClicked();

    /**
     * @brief 选择跟随表演时长单击事件
     */
    void onChoiceFollowTimeBtnClicked();

    /**
     * @brief 图形模式表演时长
     */
    void onChoiceFigureTimeBtnClicked();

    /**
     * @brief 屏保时长
     */
    void onWaitTimeBtnClicked();

    /**
     * @brief 可运行电量按钮单击事件
     */
    void onBatteryLimitBtnBtnClicked();

    /**
     * @brief 选择自绘表演时长单击事件
     */
    void onChoiceDrawTimeBtnClicked();

    /**
     * @brief 密码保存事件
     */
    void onSavePassBtnClicked();

    /**
     * @brief 设置目标显示
     * @param state true显示; false不显示
     */
    void setGoalDisplay(bool state);

    /**
     * @brief 设置电量显示
     * @param state true显示; false不显示
     */
    void setBatteryDisplay(bool state);

    /**
     * @brief 退出应用程序单击事件
     */
    void onCloseAppBtnClicked();

private:
    HomePage        *homePage;
    LPanel          *leftBtnFrame;
    QStackedWidget  *stackedWidget;
    LPanel          *robotInfoPage;
    LPanel          *perfomConfigurPage;
    LPanel          *passManagePage;
    LPanel          *systemSetPage;
    QListWidget     *zooidListView;

    LSwitchButton   *figureSwitchBtn;
    LSwitchButton   *drawSwitchBtn;
    LSwitchButton   *followModeSwitchBtn;

    QPushButton     *selectCircBtn;
    QPushButton     *selectTriangleBtn;
    QPushButton     *selectRectangleBtn;
    QPushButton     *selectCrossBtn;

    QLineEdit *inputOrigPassEdit;
    QLineEdit *inputNewPassEdit;
    QLineEdit *aginInputPassEdit;

    QPushButton *followTimeBtn;
    QPushButton *drawTimeBtn;
    QPushButton *figureTimeBtn;

    QPushButton *followNumberBtn;

    LSwitchButton *goalSwitchBtn;
    LSwitchButton *batterySwitchBtn;

    QPushButton *zoomInBtn;
    QPushButton *zoomOutBtn;
    QLabel *zoomLab;
    QPushButton *batteryLimitBtn;

    QTimer * updateTimer;

    friend class HomePage;


};

#endif // ADMINPAGE_H
