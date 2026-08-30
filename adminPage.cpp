#include <QStandardItemModel>

#include "adminPage.h"
#include "component/LSwitchButton.h"
#include "public/style.h"
#include "component/LNumberInputBox.h"

AdminPage::AdminPage(HomePage *_homePage)
{
    if(_homePage == nullptr)
    {
        return ;
    }
    homePage = _homePage;
    setup();
    initGUI();
}

void AdminPage::setup()
{
    setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
    setMinimumSize(SCREEN_WIDTH,SCREEN_HEIGHT);
    setMaximumSize(SCREEN_WIDTH,SCREEN_HEIGHT);
}

void AdminPage::initGUI()
{
    //回到前台
    QPushButton *closeBtn = new QPushButton(QStringLiteral("回到前台"));
    closeBtn->setIcon(QIcon(":/new/ofapp/res/images/back.png"));
    closeBtn->setParent(this);
    closeBtn->setFixedSize(110,40);
    //closeBtn->setStyleSheet(border_info_2 + text_info + font_size_sm+font_weight_500 +"border-radius: 20px;");
   closeBtn->setStyleSheet(closeBtnStyle);
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->move(30, 40);

    //--左侧放按钮Frame
    leftBtnFrame = new LPanel(this,LPanel::Two);
    leftBtnFrame->setGeometry(30,100,230,637);

    //--对左侧Frame垂直布局
    QVBoxLayout *leftFramVLayout = new QVBoxLayout(leftBtnFrame);
    leftFramVLayout->setMargin(20);

    //--机器人信息按钮
    QPushButton *robotoInfoBtn = new QPushButton(leftBtnFrame);
    robotoInfoBtn->setText(QStringLiteral(" 详细信息"));
    robotoInfoBtn->setIcon(QIcon(":/new/ofapp/res/images/data.png"));
    robotoInfoBtn->setFixedHeight(48);
    robotoInfoBtn->setStyleSheet(adminPagebtnstyle);
    robotoInfoBtn->setCursor(Qt::PointingHandCursor);
    leftFramVLayout->addWidget(robotoInfoBtn);

    //--表演配置按钮
    QPushButton *performConfigureBtn = new QPushButton(leftBtnFrame);
    performConfigureBtn->setText(QStringLiteral(" 表演配置"));
    performConfigureBtn->setIcon(QIcon(":/new/ofapp/res/images/config.png"));
    performConfigureBtn->setFixedHeight(48);
    performConfigureBtn->setStyleSheet(adminPagebtnstyle);
    performConfigureBtn->setCursor(Qt::PointingHandCursor);
    leftFramVLayout->addWidget(performConfigureBtn);

    //--密码管理按钮
    QPushButton *passManagerBtn = new QPushButton(leftBtnFrame);
    passManagerBtn->setText(QStringLiteral(" 密码管理"));
    passManagerBtn->setIcon(QIcon(":/new/ofapp/res/images/password.png"));
    passManagerBtn->setFixedHeight(48);
    passManagerBtn->setStyleSheet(adminPagebtnstyle);
    passManagerBtn->setCursor(Qt::PointingHandCursor);
    leftFramVLayout->addWidget(passManagerBtn);

    //--运行日志按钮
    QPushButton *runLogBtn = new QPushButton(leftBtnFrame);
    runLogBtn->setText(QStringLiteral(" 运行日志"));
    runLogBtn->setIcon(QIcon(":/new/ofapp/res/images/log.png"));
    runLogBtn->setFixedHeight(48);
    runLogBtn->setStyleSheet(adminPagebtnstyle);
    runLogBtn->setCursor(Qt::PointingHandCursor);
    leftFramVLayout->addWidget(runLogBtn);

    //--系统设置按钮
    QPushButton *systemSetBtn = new QPushButton(leftBtnFrame);
    systemSetBtn->setText(QStringLiteral(" 系统设置"));
    systemSetBtn->setIcon(QIcon(":/new/ofapp/res/images/set.png"));
    systemSetBtn->setFixedHeight(48);
    systemSetBtn->setCursor(Qt::PointingHandCursor);
    systemSetBtn->setStyleSheet(adminPagebtnstyle);
    leftFramVLayout->addWidget(systemSetBtn);

    //--关于我们按钮
    QPushButton *aboutBtn = new QPushButton(leftBtnFrame);
    aboutBtn->setText(QStringLiteral(" 关于我们"));
    aboutBtn->setIcon(QIcon(":/new/ofapp/res/images/about.png"));
    aboutBtn->setFixedHeight(48);
    aboutBtn->setCursor(Qt::PointingHandCursor);
    aboutBtn->setStyleSheet(adminPagebtnstyle);
    leftFramVLayout->addWidget(aboutBtn);
    leftFramVLayout->addStretch();
    //--左侧放按钮Frame  end

    stackedWidget = new QStackedWidget(this);
    stackedWidget->setGeometry(280,100,1056,637);
    //--机器人信息页  Zooid在线列表
    robotInfoPage = new LPanel(this,LPanel::Two);  //机器人页面板
    QVBoxLayout * robotInfoVBoxLayout = new QVBoxLayout(robotInfoPage);
    QLabel* zooidListLab = new QLabel(QStringLiteral("在线列表"));
    zooidListLab->setStyleSheet(m_12 + transparent_bg + font_size_sm + text_white + font_weight_500);
    robotInfoVBoxLayout->addWidget(zooidListLab);
    QHBoxLayout *theaderLayout = new QHBoxLayout();
    robotInfoVBoxLayout->addLayout(theaderLayout);
    theaderLayout->setSpacing(0);
    QString theaderStyle(info_bg + text_info + font_size_sm + font_weight_600 + p_4);

    QLabel *idLabel = new QLabel(QStringLiteral("ID").append("\t\t\t"));
    idLabel->setStyleSheet(theaderStyle);
    theaderLayout->addWidget(idLabel);
    QLabel *batteryLabel = new QLabel(QStringLiteral("电量").append("\t\t\t"));
    batteryLabel->setStyleSheet(theaderStyle);
    theaderLayout->addWidget(batteryLabel);
    QLabel *speedLabel = new QLabel(QStringLiteral("速度").append("\t\t\t"));
    speedLabel->setStyleSheet(theaderStyle);
    theaderLayout->addWidget(speedLabel);
    QLabel *angleLabel = new QLabel(QStringLiteral("  角度").append("\t\t\t"));
    angleLabel->setStyleSheet(theaderStyle);
    theaderLayout->addWidget(angleLabel);
    QLabel *placeLabel = new QLabel(QStringLiteral("      位置"));
    placeLabel->setStyleSheet(theaderStyle);
    theaderLayout->addWidget(placeLabel);

    zooidListView = new QListWidget();
    zooidListView->setFrameShape(QListWidget::NoFrame);
    zooidListView->setViewMode(QListView::ListMode);
    zooidListView->setFlow(QListView::TopToBottom);
    zooidListView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    zooidListView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    zooidListView->setVerticalScrollMode(QListWidget::ScrollPerPixel);
    zooidListView->setStyleSheet(R"(
        QListWidget { outline: none;background:transparent;}
        QListWidget::Item:hover { background: #101129; color:#61a8ff; }
        QListWidget::item:selected { background: #101129; color:#61a8ff;  }
        QListWidget::item:selected:!active { background: 101129;color:#61a8ff;})");

    QScroller::grabGesture(zooidListView,QScroller::LeftMouseButtonGesture);
    robotInfoVBoxLayout->addWidget(zooidListView);
    stackedWidget->addWidget(robotInfoPage);
    //--机器人信息页--end

    //--表演配置页
    perfomConfigurPage =new LPanel(this,LPanel::Two);

    QVBoxLayout *perfomConfigurLayout = new QVBoxLayout(perfomConfigurPage);

    //--图形模式配置页
    QFrame *figureModeFrame = new QFrame();
    figureModeFrame->setLayoutDirection(Qt::LeftToRight);
    figureModeFrame->setFixedSize(1056, 200);
    perfomConfigurLayout->addWidget(figureModeFrame);

    QLabel *figureModeLabel = new QLabel(figureModeFrame);
    figureModeLabel->setText(QStringLiteral("图形模式"));
    figureModeLabel->setStyleSheet(text_info + font_weight_600 + font_size_md);
    figureModeLabel->move(10,10);

    //功能开启标签
    QLabel *functionOffOnLab = new QLabel(figureModeFrame);
    functionOffOnLab->setText(QStringLiteral("功能开启"));
    functionOffOnLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    functionOffOnLab->move(30,70);

    //功能开启按钮
    figureSwitchBtn = new LSwitchButton(figureModeFrame);
    figureSwitchBtn->setFixedSize(50, 24);
    figureSwitchBtn->move(100,70);

    if(homePage->sqlConfig.EnabelFigureMode)
    {
        figureSwitchBtn->setToggle(true);
    }
    else
    {
        figureSwitchBtn->setToggle(false);
    }

    QLabel *performRobotNumLab = new QLabel(figureModeFrame);
    performRobotNumLab->setText(QStringLiteral("表演数量"));
    performRobotNumLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    performRobotNumLab->move(30,120);

    //选择构成圆形机器人数量按钮
    selectCircBtn = new QPushButton(figureModeFrame);
    //selectCircBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    selectCircBtn->setStyleSheet(selectBtnStyle);
    selectCircBtn->setIcon(QIcon(":/new/ofapp/res/images/circul.png"));
    selectCircBtn->setFixedSize(70, 30);
    selectCircBtn->move(100, 115);
    selectCircBtn->setText(QString::number(homePage->sqlConfig.circularNumber) + QStringLiteral("台"));

    //选择构成三角形机器人数量按钮
    selectTriangleBtn = new QPushButton(figureModeFrame);
    //selectTriangleBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    selectTriangleBtn->setStyleSheet(selectBtnStyle);
    selectTriangleBtn->setIcon(QIcon(":/new/ofapp/res/images/triangle.png"));
    selectTriangleBtn->setFixedSize(70, 30);
    selectTriangleBtn->move(180, 115);
    selectTriangleBtn->setText(QString::number(homePage->sqlConfig.triangleNumber) + QStringLiteral("台"));

    //选择构成矩形机器人数量按钮
    selectRectangleBtn = new QPushButton(figureModeFrame);
    //selectRectangleBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    selectRectangleBtn->setStyleSheet(selectBtnStyle);
    selectRectangleBtn->setIcon(QIcon(":/new/ofapp/res/images/react.png"));
    selectRectangleBtn->setFixedSize(70, 30);
    selectRectangleBtn->move(260, 115);
    selectRectangleBtn->setText(QString::number(homePage->sqlConfig.reactNumber) + QStringLiteral("台"));

    selectCrossBtn = new QPushButton(figureModeFrame);
    //selectCrossBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    selectCrossBtn->setStyleSheet(selectBtnStyle);
    selectCrossBtn->setIcon(QIcon(":/new/ofapp/res/images/cross.png"));
    selectCrossBtn->setFixedSize(70, 30);
    selectCrossBtn->move(340, 115);
    selectCrossBtn->setText(QString::number(homePage->sqlConfig.crossNumber) + QStringLiteral("台"));

    QLabel *figureTimeLab = new QLabel(figureModeFrame);
    figureTimeLab->setText(QStringLiteral("表演时长"));
    figureTimeLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    figureTimeLab->move(30,170);

    figureTimeBtn = new QPushButton(figureModeFrame);
    //figureTimeBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    figureTimeBtn->setStyleSheet(selectBtnStyle);
    figureTimeBtn->setIcon(QIcon(":/new/ofapp/res/images/time.png"));
    figureTimeBtn->setFixedSize(100, 30);
    figureTimeBtn->move(100, 165);
    figureTimeBtn->setText(QString::number(homePage->sqlConfig.FigureTime) + QStringLiteral("秒"));

    //保存按钮
    QPushButton *saveFigureBtn = new QPushButton(figureModeFrame);
    saveFigureBtn->setText(QStringLiteral("保存设置"));
    saveFigureBtn->setFixedSize(100, 36);
    //saveFigureBtn->setStyleSheet(border_info_2 + text_info + rounded_sm + font_size_sm + font_weight_600);
    saveFigureBtn->setStyleSheet(saveFigureBtnStyle);
    saveFigureBtn->move(figureModeFrame->width() - saveFigureBtn->width() -30,
                        figureModeFrame->height() - saveFigureBtn->height() -20 );

    QFrame *line1 = new QFrame();
    line1->setFixedSize(1034, 2);
    line1->setStyleSheet("border-bottom: 2px solid #68d8fe; background-color: #101129;");
    perfomConfigurLayout->addWidget(line1);

    //自绘模式
    QFrame *drawModeFrame = new QFrame();
    drawModeFrame->setLayoutDirection(Qt::LeftToRight);
    drawModeFrame->setFixedSize(1056, 150);
    perfomConfigurLayout->addWidget(drawModeFrame);

    QLabel *drawtitleLab = new QLabel(drawModeFrame);
    drawtitleLab->setText(QStringLiteral("绘图模式"));
    drawtitleLab->setStyleSheet(text_info + font_weight_600 + font_size_md);
    drawtitleLab->move(10,10);

    QLabel *drawFunctionOffOnLab = new QLabel(drawModeFrame);
    drawFunctionOffOnLab->setText(QStringLiteral("功能开启"));
    drawFunctionOffOnLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    drawFunctionOffOnLab->move(30,70);

    //是否开启按钮
    drawSwitchBtn = new LSwitchButton(drawModeFrame);
    drawSwitchBtn->setFixedSize(50, 24);
    drawSwitchBtn->move(100,70);
    if(homePage->sqlConfig.EnabelDrawMode)
    {
        drawSwitchBtn->setToggle(true);
    }
    else
    {
        drawSwitchBtn->setToggle(false);
    }

    QLabel *drawTimeLab = new QLabel(drawModeFrame);
    drawTimeLab->setText(QStringLiteral("表演时长"));
    drawTimeLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    drawTimeLab->move(30,120);

    drawTimeBtn = new QPushButton(drawModeFrame);
    // drawTimeBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    drawTimeBtn->setStyleSheet(selectBtnStyle);
    drawTimeBtn->setIcon(QIcon(":/new/ofapp/res/images/time.png"));
    drawTimeBtn->setFixedSize(100, 30);
    drawTimeBtn->move(100, 115);
    drawTimeBtn->setText(QString::number(homePage->sqlConfig.DrawTime) + QStringLiteral("秒"));

    QPushButton *saveDrawBtn = new QPushButton(drawModeFrame);
    saveDrawBtn->setText(QStringLiteral("保存设置"));
    saveDrawBtn->setFixedSize(100, 36);
    //saveDrawBtn->setStyleSheet(border_info_2 + text_info + rounded_sm + font_size_sm + font_weight_600);
    saveDrawBtn->setStyleSheet(saveFigureBtnStyle);
    saveDrawBtn->move(drawModeFrame->width() - saveDrawBtn->width() -30,
                        drawModeFrame->height() - saveDrawBtn->height() -20 );

    QFrame *line2 = new QFrame();
    line2->setFixedSize(1034, 2);
    line2->setStyleSheet("border-bottom: 2px solid #68d8fe; background-color: #101129;");
    perfomConfigurLayout->addWidget(line2);

    //跟随模式Frame
    QFrame *followModeFrame = new QFrame();
    followModeFrame->setLayoutDirection(Qt::LeftToRight);
    followModeFrame->setFixedSize(1056, 200);
    perfomConfigurLayout->addWidget(followModeFrame);

    //跟随 Frame-title-Lab
    QLabel *followModeLab = new QLabel(followModeFrame);
    followModeLab->setText(QStringLiteral("跟随模式"));
    followModeLab->setStyleSheet(text_info + font_weight_600 + font_size_md);
    followModeLab->move(10,10);

    //跟随下功能是否开启
    QLabel *followModeFunLab = new QLabel(followModeFrame);
    followModeFunLab->setText(QStringLiteral("功能开启"));
    followModeFunLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    followModeFunLab->move(30,70);

    //跟随下功能是否开启按钮
    followModeSwitchBtn = new LSwitchButton(followModeFrame);
    followModeSwitchBtn->setFixedSize(50, 24);
    followModeSwitchBtn->move(100,70);
    if(homePage->sqlConfig.EnabelFollowMode)
    {
        followModeSwitchBtn->setToggle(true);
    }
    else
    {
        followModeSwitchBtn->setToggle(false);
    }

    QLabel *performTimeLab = new QLabel(followModeFrame);
    performTimeLab->setText(QStringLiteral("表演时长"));
    performTimeLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    performTimeLab->move(30,120);

    followTimeBtn = new QPushButton(followModeFrame);
    //followTimeBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    followTimeBtn->setStyleSheet(selectBtnStyle);
    followTimeBtn->setIcon(QIcon(":/new/ofapp/res/images/time.png"));
    followTimeBtn->setFixedSize(100, 30);
    followTimeBtn->move(100, 115);
    followTimeBtn->setText(QString::number(homePage->sqlConfig.FollowTime) + QStringLiteral("秒"));

    QLabel *followNumberLab = new QLabel(followModeFrame);
    followNumberLab->setText(QStringLiteral("表演数量"));
    followNumberLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    followNumberLab->move(30,170);

    followNumberBtn = new QPushButton(followModeFrame);
    //followNumberBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    followNumberBtn->setStyleSheet(selectBtnStyle);
    followNumberBtn->setFixedSize(70, 30);
    followNumberBtn->move(100, 165);
    followNumberBtn->setText(QString::number(homePage->sqlConfig.circularNumber) + QStringLiteral("台"));

    QPushButton *saveFollowModeBtn = new QPushButton(followModeFrame);
    saveFollowModeBtn->setText(QStringLiteral("保存设置"));
    saveFollowModeBtn->setFixedSize(100, 36);
    //saveFollowModeBtn->setStyleSheet(border_info_2 + text_info + rounded_sm + font_size_sm + font_weight_600);
    saveFollowModeBtn->setStyleSheet(saveFigureBtnStyle);
    saveFollowModeBtn->move(followModeFrame->width() - saveFollowModeBtn->width() -30,
                        followModeFrame->height() - saveFollowModeBtn->height() -20 );

    perfomConfigurLayout->addStretch();
    stackedWidget->addWidget(perfomConfigurPage);
    //--   表演配置页    --end

    //--密码管理页
    passManagePage = new LPanel(this,LPanel::Two);
    QWidget *passFrame = new QWidget(passManagePage);
    passFrame->setFixedSize(300, 360);
    passFrame->move(stackedWidget->width()/2 - passFrame->width()/2,
                    stackedWidget->height()/2 -passFrame->height()/2);

    QVBoxLayout *passManageVLayout = new QVBoxLayout(passFrame);

    QLabel *inputOrigPassTitLab = new QLabel(passFrame);
    inputOrigPassTitLab->setText(QStringLiteral("修改密码"));
    inputOrigPassTitLab->setAlignment(Qt::AlignCenter);
    inputOrigPassTitLab->setStyleSheet(text_info + font_size_md + font_weight_600);
    passManageVLayout->addWidget(inputOrigPassTitLab);

    QLabel *inputOrigPassLab = new QLabel();
    inputOrigPassLab->setText(QStringLiteral("输入原密码："));
    inputOrigPassLab->setStyleSheet(text_info + font_size_sm);
    passManageVLayout->addWidget(inputOrigPassLab);

    inputOrigPassEdit = new QLineEdit();
    inputOrigPassEdit->setEchoMode(QLineEdit::Password);
    inputOrigPassEdit->setStyleSheet(text_info + border_info_1 + rounded_sm + font_size_md +bg_info1 + pl_12 + pr_12);
    inputOrigPassEdit->setFixedHeight(40);
    passManageVLayout->addWidget(inputOrigPassEdit);

    QLabel *inputNewPassLab = new QLabel(passFrame);
    inputNewPassLab->setText(QStringLiteral("输入新密码："));
    inputNewPassLab->setStyleSheet(text_info + font_size_sm);
    passManageVLayout->addWidget(inputNewPassLab);

    inputNewPassEdit = new QLineEdit(passFrame);
    inputNewPassEdit->setEchoMode(QLineEdit::Password);
    inputNewPassEdit->setStyleSheet(text_info + border_info_1 + rounded_sm + font_size_md + bg_info1 + pl_12 + pr_12);
    inputNewPassEdit->setFixedHeight(40);
    passManageVLayout->addWidget(inputNewPassEdit);

    QLabel *aginInputPassLab = new QLabel(passFrame);
    aginInputPassLab->setText(QStringLiteral("再次输入密码"));
    aginInputPassLab->setStyleSheet(text_info + font_size_sm);
    passManageVLayout->addWidget(aginInputPassLab);

    aginInputPassEdit = new QLineEdit(passFrame);
    aginInputPassEdit->setEchoMode(QLineEdit::Password);
    aginInputPassEdit->setStyleSheet(text_info + border_info_1 + rounded_sm + font_size_md +bg_info1 + pl_12 + pr_12);
    aginInputPassEdit->setFixedHeight(40);
    passManageVLayout->addWidget(aginInputPassEdit);
    passManageVLayout->addStretch();

    QPushButton *savePassBtn = new QPushButton(passFrame);
    savePassBtn->setText(QStringLiteral("保存"));
    savePassBtn->setFixedHeight(40);
    //savePassBtn->setStyleSheet(border_info_1 + bg_info + text_white + font_size_sm + font_weight_500 + rounded_sm);
    savePassBtn->setStyleSheet(savePassBtnStyle);

    passManageVLayout->addWidget(savePassBtn);
    passManageVLayout->addStretch();

    stackedWidget->addWidget(passManagePage);

    //--密码管理页   --end

    //--系统设置页   --begin
    systemSetPage =new LPanel(this,LPanel::Two);
    stackedWidget->addWidget(systemSetPage);

    QLabel *systemSetLabel = new QLabel(systemSetPage);
    systemSetLabel->setText(QStringLiteral("系统设置"));
    systemSetLabel->setStyleSheet(text_info + font_weight_600 + font_size_md);
    systemSetLabel->move(20,20);

    //目标位置开启标签
    QLabel *goalPosOffOnLab = new QLabel(systemSetPage);
    goalPosOffOnLab->setText(QStringLiteral("目标显示"));
    goalPosOffOnLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    goalPosOffOnLab->move(30,70);

    //目标位置开启按钮
    goalSwitchBtn = new LSwitchButton(systemSetPage);
    goalSwitchBtn->setFixedSize(50, 24);
    goalSwitchBtn->move(100,70);

    if(homePage->sqlConfig.goalDisplay)
    {
        goalSwitchBtn->setToggle(true);
    }
    else
    {
        goalSwitchBtn->setToggle(false);
    }

    //电量显示开启标签
    QLabel *batteryOffOnLab = new QLabel(systemSetPage);
    batteryOffOnLab->setText(QStringLiteral("电量显示"));
    batteryOffOnLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    batteryOffOnLab->move(180,70);

    //目标位置开启按钮
    batterySwitchBtn = new LSwitchButton(systemSetPage);
    batterySwitchBtn->setFixedSize(50, 24);
    batterySwitchBtn->move(250,70);

    if(homePage->sqlConfig.batteryDisplay)
    {
        batterySwitchBtn->setToggle(true);
    }
    else
    {
        batterySwitchBtn->setToggle(false);
    }
    batterySwitchBtn->setToggle(false);

    //模拟器缩放标签
    zoomLab = new QLabel(systemSetPage);
    zoomLab->setText(QStringLiteral("模拟器缩放: ") + QString::number(static_cast<double>(homePage->sqlConfig.zoom)));
    zoomLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    zoomLab->move(30,120);


    //模拟器缩放按钮+
    zoomInBtn = new QPushButton(systemSetPage);
    zoomInBtn->setText(QStringLiteral("放大"));
    zoomInBtn->setFixedSize(64, 32);
    zoomInBtn->setStyleSheet(transparent_bg + border_info_2 + text_info + rounded_sm + font_weight_700);
    zoomInBtn->setCursor(Qt::PointingHandCursor);
    zoomInBtn->move(160,115);

    //模拟器缩放按钮-
    zoomOutBtn = new QPushButton(systemSetPage);
    zoomOutBtn->setText(QStringLiteral("缩小"));
    zoomOutBtn->setFixedSize(64, 32);
    zoomOutBtn->setStyleSheet(transparent_bg + border_info_2 + text_info + rounded_sm + font_weight_700);
    zoomOutBtn->setCursor(Qt::PointingHandCursor);
    zoomOutBtn->move(236,115);

    //机器人运行电量最小值
    QLabel *batteryLab = new QLabel(systemSetPage);
    batteryLab->setText(QStringLiteral("机器人可运行最小电量"));
    batteryLab->setStyleSheet(text_info + font_weight_600 + font_size_sm);
    batteryLab->move(30,165);

    batteryLimitBtn = new QPushButton(systemSetPage);
    batteryLimitBtn->setStyleSheet(transparent_bg + text_info + font_weight_600 + font_size_sm + text_left);
    batteryLimitBtn->setIcon(QIcon(":/new/ofapp/res/images/batterylimit.png"));
    batteryLimitBtn->setFixedSize(100, 30);
    batteryLimitBtn->move(200, 165);
    batteryLimitBtn->setText(QString::number(homePage->sqlConfig.battery));

    QPushButton *closeAppBtn = new QPushButton(systemSetPage);
    closeAppBtn->setFixedSize(150, 48);
    closeAppBtn->setText(QStringLiteral("退出应用程序"));
    closeAppBtn->setCursor(Qt::PointingHandCursor);
    closeAppBtn->setStyleSheet(border_danger_2 + rounded_sm + font_size_md + font_weight_500 + text_danger);
    closeAppBtn->move(stackedWidget->width() / 2 - closeAppBtn->width()/2,
                      stackedWidget->height() - 120);

    stackedWidget->setCurrentIndex(0);
    ///////////////////////////////////////////////////////////////////////////////////////////////
    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCancelClicked()));
    QObject::connect(robotoInfoBtn, SIGNAL(clicked()), this, SLOT(onRobotoInfoClicked()));
    QObject::connect(performConfigureBtn, SIGNAL(clicked()), this, SLOT(onPerformConfigClicked()));
    QObject::connect(passManagerBtn, SIGNAL(clicked()), this, SLOT(onPassManagerClicked()));
    QObject::connect(runLogBtn, SIGNAL(clicked()), this, SLOT(onRunLogBtnClicked()));
    QObject::connect(systemSetBtn, SIGNAL(clicked()), this, SLOT(onSystemSetBtnClicked()));

    QObject::connect(selectCircBtn, SIGNAL(clicked()), this, SLOT(onSelectCircNumClicked()));
    QObject::connect(selectTriangleBtn, SIGNAL(clicked()), this, SLOT(onTriangleNumClicked()));
    QObject::connect(selectRectangleBtn, SIGNAL(clicked()), this, SLOT(onRectangleNumClicked()));
    QObject::connect(selectCrossBtn, SIGNAL(clicked()), this, SLOT(onCrossNumClicked()));

    QObject::connect(zoomInBtn, SIGNAL(clicked()), this, SLOT(onZoomInBtnClicked()));
    QObject::connect(zoomOutBtn, SIGNAL(clicked()), this, SLOT(onZoomOutBtnClicked()));


    QObject::connect(saveFigureBtn, SIGNAL(clicked()), this, SLOT(onSaveFigureBtnClicked()));
    QObject::connect(figureTimeBtn, SIGNAL(clicked()), this, SLOT(onChoiceFigureTimeBtnClicked()));
    
    QObject::connect(saveDrawBtn, SIGNAL(clicked()), this, SLOT(onSaveDrawBtnClicked()));
    QObject::connect(drawTimeBtn, SIGNAL(clicked()), this, SLOT(onChoiceDrawTimeBtnClicked()));

    QObject::connect(followTimeBtn, SIGNAL(clicked()), this, SLOT(onChoiceFollowTimeBtnClicked()));
    QObject::connect(saveFollowModeBtn, SIGNAL(clicked()), this, SLOT(onSaveFollowModeBtnClicked()));

    QObject::connect(goalSwitchBtn, SIGNAL(toggled(bool)), this, SLOT(setGoalDisplay(bool)));
    QObject::connect(batterySwitchBtn, SIGNAL(toggled(bool)), this, SLOT(setBatteryDisplay(bool)));

    QObject::connect(savePassBtn, SIGNAL(clicked()), this, SLOT(onSavePassBtnClicked()));

    QObject::connect(closeAppBtn, SIGNAL(clicked()), this, SLOT(onCloseAppBtnClicked()));

    QObject::connect(batteryLimitBtn, SIGNAL(clicked()), this, SLOT(onBatteryLimitBtnBtnClicked()));


    updateTimer = new QTimer();
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(updateZooidList()));
    updateTimer->start(100);
}

void AdminPage::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    QPainterPath pathBack;
    pathBack.setFillRule(Qt::WindingFill);
    pathBack.addRect(QRect(0, 0, this->width(), this->height()));
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.fillPath(pathBack, QBrush(QColor("#101129")));
    painter.drawPixmap(QRect(0, 0, this->width(), this->width()/14.7), QPixmap(":/new/ofapp/res/images/bg_title.png"));
    painter.setPen(QColor("#68d8fe"));
    QFont font("Microsoft YaHei", 22, QFont::Bold);
    painter.setFont(font);
    QString titleName(QStringLiteral("后台管理"));
    int titleNameWidth = painter.fontMetrics().width(titleName) ;
    painter.drawText((1366 - titleNameWidth) / 2, 60, titleName);
}

void AdminPage::onRobotoInfoClicked()
{
    stackedWidget->setCurrentWidget(robotInfoPage);
}

void AdminPage::onPerformConfigClicked()
{
    stackedWidget->setCurrentWidget(perfomConfigurPage);
}

void AdminPage::onRunLogBtnClicked()
{
    //stackedWidget->setCurrentWidget(runLogBtn);
}

void AdminPage::onSystemSetBtnClicked()
{
    stackedWidget->setCurrentWidget(systemSetPage);
}

void AdminPage::onPassManagerClicked()
{
    inputOrigPassEdit->setFocus();
    stackedWidget->setCurrentWidget(passManagePage);
}

void AdminPage::onCancelClicked()
{
    homePage->setCurrentWidget(HomeWidget);
    close();
}

QString AdminPage::zooidInfoToText(ZooidInfo info)
{
    QString infoText = "  ";
    infoText += QString::number(info.id).append("\t\t\t");
    infoText += QString::number(info.batteryLevel).append("\t\t\t");
    infoText += QString::number(0).append("\t\t\t");
    infoText += QString::number(info.orientation,'g', 3) + QStringLiteral("°").append("\t\t\t");
    QString x = QString::number((int)(info.position.getX() * 1000));
    QString y = QString::number((int)(info.position.getY() * 1000));
    infoText += x + ", " + y;
    return infoText;
}

void AdminPage::updateZooidList(){

    vector<ZooidInfo> zooidInfoVec;
    homePage->zooidManager.getAllZooidInfo(zooidInfoVec);

    //先从列表中移除离线的Zooid
    for(int i=0; i<zooidListView->count(); i++)
    {
        unsigned int zooidId = (unsigned int)(zooidListView->item(i)->statusTip().toInt());
        auto it = find_if(zooidInfoVec.begin(), zooidInfoVec.end(), [&zooidId](ZooidInfo &info) { return info.id == zooidId; });
        if (it != zooidInfoVec.end())
        {
            zooidListView->item(i)->setText(zooidInfoToText(*it));
        }
        else
        {
            zooidListView->takeItem(i);
            i--;
        }
    }

    //新上线的Zooid添加到列表中
    for(int i=0; i <zooidInfoVec.size(); i++){

        QString zooidIdText =QString::number(zooidInfoVec[i].id);
        bool isNewZooid = true;
        for(int i=0; i<zooidListView->count(); i++)
        {
            if(zooidIdText == zooidListView->item(i)->statusTip())
            {
                isNewZooid = false;
                break;
            }
        }

        if(isNewZooid)
        {
            QListWidgetItem *item = new QListWidgetItem(zooidListView,zooidListView->count());
            item->setText(zooidInfoToText(zooidInfoVec[i]));
            item->setTextColor(QColor("#61a8ff"));
            item->setFont(QFont("Microsoft YaHei", 9, QFont::Bold));
            item->setSizeHint(QSize(item->sizeHint().width(),40));
            item->setStatusTip(zooidIdText);
        }
    }
}

void AdminPage::onSelectCircNumClicked()
{
    //设置图形模式圆形的表演数量
    LNumberInputBox selectNum(this);

    int state = selectNum.showBox(QStringLiteral("选择机器人个数"),true);
    int circularNumber = selectNum.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        selectCircBtn->setText(QString::number(circularNumber).append(QStringLiteral("台")));
        homePage->sqlConfig.circularNumber = circularNumber;
    }
}

void AdminPage::onTriangleNumClicked()
{
    //保存图形模式里的三角形表演个数
    LNumberInputBox selectNum(this);

    int state = selectNum.showBox(QStringLiteral("选择机器人个数"),true);
    int triangleNumber = selectNum.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(triangleNumber > 21)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("机器人数量不建议超过21台"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            selectTriangleBtn->setText(QString::number(triangleNumber).append(QStringLiteral("台")));
            homePage->sqlConfig.triangleNumber = triangleNumber;
        }

    }
}

void AdminPage::onRectangleNumClicked()
{
    //保存图形模式里的矩形表演个数
    LNumberInputBox  selectNum(this);

    int state = selectNum.showBox(QStringLiteral("选择机器人个数"),true);
    int reactNumber = selectNum.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(reactNumber > 20)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("机器人数量不建议超过20台"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            selectRectangleBtn->setText(QString::number(reactNumber).append(QStringLiteral("台")));
            homePage->sqlConfig.reactNumber = reactNumber;
        }
    }
}

void AdminPage::onZoomInBtnClicked()
{
    homePage->zooidManager.zooidSimulator->zoomIn();
    double zoomValue = homePage->zooidManager.zooidSimulator->getZoom();
    homePage->sqlConfig.zoom = static_cast<float>(zoomValue);
    zoomLab->setText(QStringLiteral("模拟器缩放: ") + QString::number(zoomValue));

    QSqlQuery  query;
    QString sql=QString("UPDATE `config` SET `zoom`=?  WHERE `id`=1000");
    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.zoom);
    query.exec();
    homePage->update();

}

void AdminPage::onZoomOutBtnClicked()
{
    homePage->zooidManager.zooidSimulator->zoomOut();
    double zoomValue = homePage->zooidManager.zooidSimulator->getZoom();
    homePage->sqlConfig.zoom = static_cast<float>(zoomValue);
    zoomLab->setText(QStringLiteral("模拟器缩放: ") + QString::number(zoomValue));

    QSqlQuery  query;
    QString sql=QString("UPDATE `config` SET `zoom`=?  WHERE `id`=1000");
    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.zoom);
    query.exec();
    homePage->update();
}

void AdminPage::onCrossNumClicked()
{
    //保存图形模式里的十字表演个数
    LNumberInputBox  selectNum(this);

    int state = selectNum.showBox(QStringLiteral("选择机器人个数"),true);
    int crossNumber = selectNum.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(crossNumber > 15)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("机器人数量不建议超过15台"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            selectCrossBtn->setText(QString::number(crossNumber).append(QStringLiteral("台")));
            homePage->sqlConfig.crossNumber = crossNumber;
        }

    }
}

void AdminPage::onChoiceFollowTimeBtnClicked()
{
    LNumberInputBox  followTimeBox(this);

    int state = followTimeBox.showBox(QStringLiteral("表演时长"),true);
    int followTime = followTimeBox.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(followTime < 10)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("表演时长最短10秒"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            followTimeBtn->setText(QString::number(followTime).append(QStringLiteral("秒")));
            homePage->sqlConfig.FollowTime = followTime;
        }
    }
}

void AdminPage::onChoiceFigureTimeBtnClicked()
{
    LNumberInputBox  figureTimeBox(this);

    int state = figureTimeBox.showBox(QStringLiteral("表演时长"),true);
    int figureTime = figureTimeBox.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(figureTime < 10)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("表演时长最短10秒"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            figureTimeBtn->setText(QString::number(figureTime).append(QStringLiteral("秒")));
            homePage->sqlConfig.FigureTime = figureTime;
        }
    }
}


void AdminPage::onWaitTimeBtnClicked()
{
    LNumberInputBox waitTimeBox(this);

    int state = waitTimeBox.showBox(QStringLiteral("屏保时长"), true);
    int waitTime = waitTimeBox.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
       // waitTimeBtn->setText(QString::number(waitTime).append(QStringLiteral("秒")));
        homePage->sqlConfig.waitTime = waitTime;
        QSqlQuery  query;
        QString sql=QString("UPDATE `config` SET `waitTime`=?  WHERE `id`=1000");
        query.prepare(sql);
        query.addBindValue(homePage->sqlConfig.waitTime);
        query.exec();

        //这里设置前台的逻辑...
    }
}

void AdminPage::onBatteryLimitBtnBtnClicked()
{
    LNumberInputBox batteryLimitBox(this);

    int state = batteryLimitBox.showBox(QStringLiteral("电量设置"), true);
    unsigned int battery = batteryLimitBox.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(battery > 100)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("设置电量值不能超过100"),LMessageBox::BUTTON_OK , true);
        }

        else
        {
            homePage->zooidManager.setBatteryLimit(battery);
            batteryLimitBtn->setText(QString::number(battery));
            homePage->sqlConfig.battery = battery;
            QSqlQuery  query;
            QString sql=QString("UPDATE `config` SET `battery`=?  WHERE `id`=1000");
            query.prepare(sql);
            query.addBindValue(homePage->sqlConfig.battery);
            query.exec();

            if(battery < 10)
            {
                LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("电量设置的较低可能会影响表演效果"),LMessageBox::BUTTON_OK , true);
            }
        }
    }
}

void AdminPage::onChoiceDrawTimeBtnClicked()
{
    LNumberInputBox drawTimeBox(this);

    int state = drawTimeBox.showBox(QStringLiteral("表演时长"),true);
    int drawTime = drawTimeBox.getValue();

    if(state == LNumberInputBox::ID_OK)
    {
        if(drawTime < 10)
        {
            LMessageBox::showBox(this, QStringLiteral("提示"),QStringLiteral("表演时长最短10秒"),LMessageBox::BUTTON_OK , true);
        }
        else
        {
            drawTimeBtn->setText(QString::number(drawTime).append(QStringLiteral("秒")));
            homePage->sqlConfig.DrawTime = drawTime;
        }
    }
}

void AdminPage::onSaveFigureBtnClicked()
{
    //更新图形模式数据
    homePage->sqlConfig.EnabelFigureMode = figureSwitchBtn->isToggled();

    QSqlQuery  query;
    QString sql=QString("UPDATE `config` SET `EnabelFigureMode`=?, `FigureTime` = ?, `circularNumber`=? ,`triangleNumber`=?, `reactNumber`=?, `crossNumber` = ? WHERE `id`=1000");

    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.EnabelFigureMode);
    query.addBindValue(homePage->sqlConfig.FigureTime);
    query.addBindValue(homePage->sqlConfig.circularNumber);
    query.addBindValue(homePage->sqlConfig.triangleNumber);
    query.addBindValue(homePage->sqlConfig.reactNumber);
    query.addBindValue(homePage->sqlConfig.crossNumber);
    query.exec();

    if(!query.exec())
    {
        qDebug()<<"Error: "<< query.lastError();
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存失败"),LMessageBox::BUTTON_OK , true);
    }
    else
    {
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存成功"),LMessageBox::BUTTON_OK , true);
        if(homePage->sqlConfig.EnabelFigureMode)
        {
            homePage->graphicalModeBtn->show();
        }
        else
        {
            homePage->graphicalModeBtn->hide();
        }
        homePage->update();
    }
}

void AdminPage::onSaveFollowModeBtnClicked()
{
    //更新跟随模式配置数据
    homePage->sqlConfig.EnabelFollowMode = followModeSwitchBtn->isToggled();

    QSqlQuery  query;
    QString sql = QString("UPDATE `config` SET EnabelFollowMode=?, FollowTime=? WHERE `id` = 1000");
    query.prepare(sql);

    query.addBindValue(homePage->sqlConfig.EnabelFollowMode);
    query.addBindValue(homePage->sqlConfig.FollowTime);
    query.exec();

    if(!query.exec())
    {
        qDebug()<<"Error: " << query.lastError();
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存失败"),LMessageBox::BUTTON_OK , true);
    }
    else
    {
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存成功"),LMessageBox::BUTTON_OK , true);
        if(homePage->sqlConfig.EnabelFollowMode)
        {
            homePage->followModeBtn->show();
        }
        else
        {
            homePage->followModeBtn->hide();
        }
        //更新界面
        homePage->update();
    }
}

void AdminPage::onSaveDrawBtnClicked()
{   
    //更新自绘模式数据
    homePage->sqlConfig.EnabelDrawMode = drawSwitchBtn->isToggled();

    QSqlQuery  query;
    QString sql = QString("UPDATE `config` SET `EnabelDrawMode`=?, `DrawTime` = ? WHERE `id`=1000");
    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.EnabelDrawMode);
    query.addBindValue(homePage->sqlConfig.DrawTime);
    query.exec();
    if(!query.exec())
    {
        qDebug()<<"Error: "<< query.lastError();
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存失败"),LMessageBox::BUTTON_OK , true);
    }
    else
    {
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("保存成功"),LMessageBox::BUTTON_OK , true);
        if(homePage->sqlConfig.EnabelDrawMode)
        {
            homePage->drawModeBtn->show();
        }
        else
        {
            homePage->drawModeBtn->hide();
        }
        homePage->update();
    }
}

void AdminPage::onSavePassBtnClicked()
{
     QString  OrigPass= inputOrigPassEdit->text();
     QSqlQuery query;
     if(OrigPass.isEmpty())
     {
         LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("原始密码不能为空"),LMessageBox::BUTTON_OK , true);
         return;
     }
     else
     {
         QString sql = QString("SELECT `password` FROM `config` WHERE `id` = 1000");
         query.exec(sql);
         query.first();
         QString password = query.value("password").toString();
         if(OrigPass != password)
         {
             LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("与原始密码不符，重新输入！"),LMessageBox::BUTTON_OK , true);
             return;
         }
     }

     QString inputNewPass=inputNewPassEdit->text();
     if(inputNewPass.isEmpty())
     {
        LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("新密码不能为空！"),LMessageBox::BUTTON_OK , true);
        return;
     }

     QString  aginPass=aginInputPassEdit->text();
     if(aginInputPassEdit->text().isEmpty())
     {
         LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("再次输入密码不能为空！"),LMessageBox::BUTTON_OK , true);
         return;
     }
     else
     {
         if(aginPass != inputNewPass)
         {
            LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("两次输入密码不一样！"),LMessageBox::BUTTON_OK , true);
            return;
         }
     }
     if(!query.exec("UPDATE `config` SET `password` = " + inputNewPass))
     {
         qDebug()<<"Error: "<< query.lastError();
     }
     else
     {
         LMessageBox::showBox(this,QStringLiteral("提示"),QStringLiteral("密码修改成功 ！"),LMessageBox::BUTTON_OK , true);
         inputOrigPassEdit->clear();
         aginInputPassEdit->clear();
         inputNewPassEdit->clear();
         qDebug()<<"Message: password modify success! new password is"<<inputNewPass;
     }
}

void AdminPage::setGoalDisplay(bool state)
{
    //更新目标位置显示
    homePage->sqlConfig.goalDisplay = state;
    QSqlQuery  query;
    QString sql=QString("UPDATE `config` SET `goalDisplay`=?  WHERE `id`=1000");
    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.goalDisplay);
    query.exec();
    homePage->zooidManager.setGoalShow(homePage->sqlConfig.goalDisplay);
    homePage->update();
}

void AdminPage::setBatteryDisplay(bool state)
{
    //电量显示
    homePage->sqlConfig.batteryDisplay = state;

    QSqlQuery  query;
    QString sql=QString("UPDATE `config` SET `batteryDisplay`=?  WHERE `id`=1000");
    query.prepare(sql);
    query.addBindValue(homePage->sqlConfig.batteryDisplay);
    query.exec();
    homePage->zooidManager.setBatteryShow(homePage->sqlConfig.batteryDisplay);
    homePage->update();
}

void AdminPage::onCloseAppBtnClicked()
{
    this->close();
    homePage->close();
}


