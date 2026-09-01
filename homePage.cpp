#include "homePage.h"
#include <QDir>
#include <QFileInfo>
#include <QSet>
#include <QSqlQuery>
#include <QStringList>

namespace
{
QString pursuitPhaseText(PursuitPhase phase)
{
    switch (phase)
    {
    case PursuitPhase::Idle: return QStringLiteral("IDLE");
    case PursuitPhase::Pursuit: return QStringLiteral("PURSUIT");
    case PursuitPhase::Surround: return QStringLiteral("SURROUND");
    case PursuitPhase::Capture: return QStringLiteral("CAPTURE");
    case PursuitPhase::Captured: return QStringLiteral("CAPTURED");
    }
    return QStringLiteral("UNKNOWN");
}
}

HomePage::HomePage()
{
    setup();
    initGUI();
}

HomePage::~HomePage()
{
    if (updateTimer != nullptr)
    {
        updateTimer->stop();
        delete updateTimer;
        updateTimer = nullptr;
    }
}

bool HomePage::connectDB()
{
    const QFileInfo databaseFile(SQL_PATH);
    if (!QDir().mkpath(databaseFile.absolutePath()))
    {
        qWarning() << "Unable to create database directory:" << databaseFile.absolutePath();
        return false;
    }

    m_db = QSqlDatabase::addDatabase("QSQLITE");
    m_db.setDatabaseName(SQL_PATH);
    if (!m_db.open())
    {
        qWarning() << "Unable to open database:" << m_db.lastError();
        return false;
    }

    QSqlQuery query(m_db);
    const QString createConfigTable = QStringLiteral(
        "CREATE TABLE IF NOT EXISTS `config` ("
        "`id` INTEGER PRIMARY KEY,"
        "`EnabelFigureMode` INTEGER NOT NULL DEFAULT 0,"
        "`EnabelDrawMode` INTEGER NOT NULL DEFAULT 0,"
        "`EnabelFollowMode` INTEGER NOT NULL DEFAULT 0,"
        "`goalDisplay` INTEGER NOT NULL DEFAULT 1,"
        "`batteryDisplay` INTEGER NOT NULL DEFAULT 1,"
        "`screensaverDisplay` INTEGER NOT NULL DEFAULT 0,"
        "`circularNumber` INTEGER NOT NULL DEFAULT 8,"
        "`triangleNumber` INTEGER NOT NULL DEFAULT 3,"
        "`reactNumber` INTEGER NOT NULL DEFAULT 4,"
        "`crossNumber` INTEGER NOT NULL DEFAULT 5,"
        "`hexagonNumber` INTEGER NOT NULL DEFAULT 6,"
        "`fivepointedNumber` INTEGER NOT NULL DEFAULT 10,"
        "`FollowTime` INTEGER NOT NULL DEFAULT 60,"
        "`DrawTime` INTEGER NOT NULL DEFAULT 60,"
        "`FigureTime` INTEGER NOT NULL DEFAULT 60,"
        "`password` TEXT NOT NULL DEFAULT 'admin',"
        "`zoom` REAL NOT NULL DEFAULT 1.0,"
        "`useTime` INTEGER NOT NULL DEFAULT 0,"
        "`waitTime` INTEGER NOT NULL DEFAULT 300,"
        "`battery` INTEGER NOT NULL DEFAULT 20,"
        "`showCount` INTEGER NOT NULL DEFAULT 0"
        ")");

    if (!query.exec(createConfigTable))
    {
        qWarning() << "Unable to create config table:" << query.lastError();
        return false;
    }

    struct ConfigColumn
    {
        const char* name;
        const char* definition;
    };
    static const ConfigColumn requiredColumns[] = {
        {"id", "INTEGER"},
        {"EnabelFigureMode", "INTEGER NOT NULL DEFAULT 0"},
        {"EnabelDrawMode", "INTEGER NOT NULL DEFAULT 0"},
        {"EnabelFollowMode", "INTEGER NOT NULL DEFAULT 0"},
        {"goalDisplay", "INTEGER NOT NULL DEFAULT 1"},
        {"batteryDisplay", "INTEGER NOT NULL DEFAULT 1"},
        {"screensaverDisplay", "INTEGER NOT NULL DEFAULT 0"},
        {"circularNumber", "INTEGER NOT NULL DEFAULT 8"},
        {"triangleNumber", "INTEGER NOT NULL DEFAULT 3"},
        {"reactNumber", "INTEGER NOT NULL DEFAULT 4"},
        {"crossNumber", "INTEGER NOT NULL DEFAULT 5"},
        {"hexagonNumber", "INTEGER NOT NULL DEFAULT 6"},
        {"fivepointedNumber", "INTEGER NOT NULL DEFAULT 10"},
        {"FollowTime", "INTEGER NOT NULL DEFAULT 60"},
        {"DrawTime", "INTEGER NOT NULL DEFAULT 60"},
        {"FigureTime", "INTEGER NOT NULL DEFAULT 60"},
        {"password", "TEXT NOT NULL DEFAULT 'admin'"},
        {"zoom", "REAL NOT NULL DEFAULT 1.0"},
        {"useTime", "INTEGER NOT NULL DEFAULT 0"},
        {"waitTime", "INTEGER NOT NULL DEFAULT 300"},
        {"battery", "INTEGER NOT NULL DEFAULT 20"},
        {"showCount", "INTEGER NOT NULL DEFAULT 0"}
    };

    QSet<QString> existingColumns;
    QSqlQuery schemaQuery(m_db);
    if (!schemaQuery.exec(QStringLiteral("PRAGMA table_info(`config`)")))
    {
        qWarning() << "Unable to inspect config table:" << schemaQuery.lastError();
        return false;
    }
    while (schemaQuery.next())
        existingColumns.insert(schemaQuery.value(1).toString().toLower());

    for (const ConfigColumn& column : requiredColumns)
    {
        if (existingColumns.contains(QString::fromLatin1(column.name).toLower()))
            continue;

        QSqlQuery migrationQuery(m_db);
        const QString migrationSql = QStringLiteral("ALTER TABLE `config` ADD COLUMN `%1` %2")
            .arg(QLatin1String(column.name), QLatin1String(column.definition));
        if (!migrationQuery.exec(migrationSql))
        {
            qWarning() << "Unable to migrate config column" << column.name
                       << migrationQuery.lastError();
            return false;
        }
    }

    QSqlQuery defaultRowQuery(m_db);
    if (!defaultRowQuery.exec(QStringLiteral("SELECT 1 FROM `config` WHERE `id` = 1000 LIMIT 1")))
    {
        qWarning() << "Unable to check default config row:" << defaultRowQuery.lastError();
        return false;
    }
    if (!defaultRowQuery.next() &&
        !defaultRowQuery.exec(QStringLiteral("INSERT INTO `config` (`id`) VALUES (1000)")))
    {
        qWarning() << "Unable to create default config row:" << defaultRowQuery.lastError();
        return false;
    }

    return true;
}

void HomePage::paintEvent(QPaintEvent *)
{
    //背景
    QPainter painter(this);
    QPainterPath pathBack;
    pathBack.setFillRule(Qt::WindingFill);
    pathBack.addRect(QRect(0, 0, this->width(), this->height()));
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.fillPath(pathBack, QBrush(QColor("#101129")));
    painter.drawPixmap(QRect(0, 0, this->width(), static_cast<int>(this->width()/14.7)), QPixmap(":/new/ofapp/res/images/bg_title.png"));

    //标题
    painter.setPen(QColor("#68d8fe"));
    QFont font("Microsoft YaHei", 22, QFont::Bold);
    painter.setFont(font);
    QString titleName(AppNameStr);
    int titleNameWidth = painter.fontMetrics().width(titleName) ;
    painter.drawText((SCREEN_WIDTH - titleNameWidth) / 2, 60, titleName);
}

void HomePage::dataUpdate()
{
    updateTestModeUi();
}

void HomePage::startHardwareTest()
{
    if (!zooidManager.startTestMode())
    {
        testModeStatusLabel->setText(QStringLiteral("测试已在启动、运行或安全停止中"));
        return;
    }
    testModeStatusLabel->setText(QStringLiteral("正在检查在线机器人…"));
}

void HomePage::updateTestModeUi()
{
    if (testModeBtn == nullptr || testModeStatusLabel == nullptr)
        return;

    const TestModeStatus status = zooidManager.getTestModeStatus();
    QString text;
    switch (status)
    {
    case TestModeStatus::Idle:
        text = QStringLiteral("等待开始测试");
        break;
    case TestModeStatus::StartPending:
        text = QStringLiteral("正在检查在线机器人…");
        break;
    case TestModeStatus::Running:
        text = QStringLiteral("围捕任务运行中");
        break;
    case TestModeStatus::Completed:
        text = QStringLiteral("测试完成，机器人已停止");
        break;
    case TestModeStatus::Stopped:
        text = QStringLiteral("已手动停止");
        break;
    case TestModeStatus::NoActiveRobots:
        text = QStringLiteral("需要至少四台反馈新鲜且已激活的机器人");
        break;
    case TestModeStatus::AllTargetsLost:
        text = QStringLiteral("参与机器人掉线，已执行安全停止");
        break;
    case TestModeStatus::FeedbackStale:
        text = QStringLiteral("参与机器人反馈超时，已执行安全停止");
        break;
    case TestModeStatus::InvalidFeedback:
        text = QStringLiteral("机器人反馈或控制数据无效，已执行安全停止");
        break;
    case TestModeStatus::InvalidGeometry:
        text = QStringLiteral("当前场地无法生成安全围捕槽位，已停止");
        break;
    case TestModeStatus::ReceiverError:
        text = QStringLiteral("接收器发送失败，已执行安全停止");
        break;
    }

    const PursuitStatusSnapshot snapshot = zooidManager.getTestModeSnapshot();
    if (snapshot.roles.valid)
    {
        text += QStringLiteral("\n阶段: ") + pursuitPhaseText(snapshot.phase);
        text += QStringLiteral("\n目标 ID: ") + QString::number(snapshot.roles.targetId);
        text += QStringLiteral("\n追捕 ID: %1, %2, %3")
            .arg(snapshot.roles.pursuerIds[0])
            .arg(snapshot.roles.pursuerIds[1])
            .arg(snapshot.roles.pursuerIds[2]);
        text += QStringLiteral("\n距离: %1, %2, %3 m")
            .arg(snapshot.targetDistances[0], 0, 'f', 3)
            .arg(snapshot.targetDistances[1], 0, 'f', 3)
            .arg(snapshot.targetDistances[2], 0, 'f', 3);
        text += QStringLiteral("\n捕获进度: %1%").arg(qRound(snapshot.captureProgress * 100.0));
    }

    const std::vector<unsigned int> lostIds = zooidManager.getTestModeLostRobotIds();
    if (!lostIds.empty())
    {
        QStringList values;
        for (unsigned int id : lostIds)
            values << QString::number(id);
        text += QStringLiteral("\n掉线 ID: ") + values.join(QStringLiteral(", "));
    }

    testModeStatusLabel->setText(text);
    testModeBtn->setEnabled(status != TestModeStatus::StartPending &&
                            status != TestModeStatus::Running);
}

void HomePage::setup()
{
    setWindowTitle(AppNameStr);
    setWindowFlags(Qt::FramelessWindowHint);
    setMinimumSize(SCREEN_WIDTH,SCREEN_HEIGHT);
    setMaximumSize(SCREEN_WIDTH,SCREEN_HEIGHT);

    //连接数据库
    !connectDB()?(qDebug()<<("Error: Qql connect failed!")):(qDebug()<<("Sql connect success!"));
    readSqlConfigData();

    appWidget = MenuWidget;

    operateNum=0;
    zooidNumber = 0;
    isGraphical = false;
    zooidManager.init();

    setCurrentWidget(MenuWidget);

    //初始化模拟器大小
    zooidManager.zooidSimulator->zoomlevel(static_cast<double>(sqlConfig.zoom));

    //初始化最低电量值
    zooidManager.setBatteryLimit(sqlConfig.battery);

    //初始化目标位置显示
    zooidManager.setGoalShow(sqlConfig.goalDisplay);

    //初始化电量显示
    zooidManager.setBatteryShow(sqlConfig.batteryDisplay);

    //后台管理
    admin = new AdminPage(this);

}

void HomePage::initGUI()
{
    //zooid模拟器
    LPanel *simulatorLPanel = new LPanel(this, LPanel::One);
    zooidManager.zooidSimulator->setParent(simulatorLPanel);
    zooidManager.zooidSimulator->setFixedSize(960,710);
    zooidManager.zooidSimulator->move(20,13);
    zooidManager.zooidSimulator->show();
    simulatorLPanel->setGeometry(30,100, 1000,740);

    //添加生成方案按钮
    LPanel *goalPlanPanel = new LPanel(this);
    goalPlanPanel->setGeometry(1050,100, 280,740);
    QVBoxLayout* selectPlanLayout = new QVBoxLayout();
    selectPlanLayout->setSpacing(0);
    goalPlanPanel->setLayout(selectPlanLayout);

    QLabel* selectPlanLab = new QLabel(PlangSelestStr);
    selectPlanLab->setStyleSheet(m_12 + transparent_bg + font_size_sm + text_white + font_weight_500);
    selectPlanLayout->addWidget(selectPlanLab);

    QVBoxLayout* selectPlanBtnsLayout = new QVBoxLayout();
    selectPlanLayout->addLayout(selectPlanBtnsLayout);

    testModeBtn = new QPushButton(QStringLiteral("开始测试"), goalPlanPanel);
    testModeBtn->setCursor(Qt::PointingHandCursor);
    testModeBtn->setFixedHeight(120);
    testModeBtn->setStyleSheet(mainbuttonstyle);
    selectPlanBtnsLayout->addWidget(testModeBtn);

    stopBtn = new QPushButton(QStringLiteral("停止测试"), goalPlanPanel);
    stopBtn->setStyleSheet("QPushButton{background-color:#cc3333;border-radius:5px;color:white;font-size:24px;font-weight:bold;}"
                           "QPushButton:hover{background-color:#ee4444;}");
    stopBtn->setFixedHeight(120);
    stopBtn->setCursor(Qt::PointingHandCursor);
    selectPlanBtnsLayout->addWidget(stopBtn);

    testModeStatusLabel = new QLabel(QStringLiteral("等待机器人上线"), goalPlanPanel);
    testModeStatusLabel->setWordWrap(true);
    testModeStatusLabel->setAlignment(Qt::AlignCenter);
    testModeStatusLabel->setStyleSheet(m_12 + transparent_bg + font_size_sm + text_white);
    selectPlanBtnsLayout->addWidget(testModeStatusLabel);

    selectPlanBtnsLayout->addStretch();

    QPushButton *adminBtn = new QPushButton(AdminManagerStr);
    adminBtn->setIcon(QIcon(":/new/ofapp/res/images/admin.png"));
    adminBtn->setParent(this);
    adminBtn->setFixedSize(110,40);
    adminBtn->setStyleSheet(adminBtnstyle);
    adminBtn->move(30, 40);

    backButton = new QPushButton(this);
    backButton->setStyleSheet(transparent_bg + "border-image:url(:/new/ofapp/res/images/back1.png);" );
    backButton->setFixedSize(120,40);
    backButton->setCursor(Qt::PointingHandCursor);
    backButton->move(width()-150, 40);
    backButton->hide();

    ///////////////////////////////////////////////////////////////////////////////////////////
    QObject::connect(adminBtn, SIGNAL(clicked()), this, SLOT(onAdminBtnClicked()));
    connect(testModeBtn, &QPushButton::clicked, this, &HomePage::startHardwareTest);
    connect(stopBtn, &QPushButton::clicked, this, &HomePage::stopAllModes);

    updateTimer = new QTimer();
    connect(updateTimer, SIGNAL(timeout()), this, SLOT(dataUpdate()));
    updateTimer->start(100);
}

void HomePage::generateCharge()
{
    isGraphical = false;
    zooidManager.zooidFollow->end();
    coutdown->endCountdown();
    zooidManager.setPlanningMode(ChargePlanning);
    zooidManager.zooidSimulator->clearBackgroundImage();
    runPlanning();
}

void HomePage::generateTriangle()
{
    zooidManager.setPlanningMode(TrianglePlanning, sqlConfig.triangleNumber);
}

void HomePage::generateReact()
{
    zooidManager.setPlanningMode(ReactPlanning, sqlConfig.reactNumber);
}

void HomePage::generateCross()
{
    zooidManager.setPlanningMode(CrossPlanning, sqlConfig.crossNumber);
}

void HomePage::generateHexagon()
{
    zooidManager.setPlanningMode(HexagonPlanning, sqlConfig.hexagonNumber);
}

void HomePage::generateCircul()
{
    zooidManager.setPlanningMode(CirculPlanning, sqlConfig.circularNumber);
}

void HomePage::generateFivepointed()
{
    zooidManager.setPlanningMode(FivepointedPlanning, sqlConfig.fivepointedNumber);
}

void HomePage::generateDrawpath()
{
    drawModeBtn->setStyleSheet(mainButtonSelect);

    //判断当前倒计时未完成
    if(coutdown->isCountDown())
    {
        LMessageBox::showBox(this, WarningStr, ShowingStr,  LMessageBox::BUTTON_OK, true);
        return ;
    }

    LDrawModelBox  drawModelBox(this);
    int state = drawModelBox.showBox();
    if(state == LDrawModelBox::Ok)
    {
        drawModelBox.zooidDraw->generatePath();
        zooidManager.setDrawPathPoints(drawModelBox.zooidDraw->getPathPoints());
        zooidManager.setPlanningMode(DrawpathPlanning);
        ErrorCode errorCode = zooidManager.runPlanning();
        if(errorCode == NumberError)
        {
            LMessageBox::showBox(this, WarningStr,DrawPathStr,  LMessageBox::BUTTON_OK, true);
        }
        else
        {
            zooidManager.setGoalShow(sqlConfig.goalDisplay);
            coutdown->startCountdown(sqlConfig.DrawTime);
            QPixmap pix = drawModelBox.zooidDraw->getPixmap();
            zooidManager.zooidSimulator->setBackgroundImage(&pix);
            isGraphical = true;
        }
    }
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainButtonSelect);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::generateFollowpath()
{
    followModeBtn->setStyleSheet(mainButtonSelect);
    if(coutdown->isCountDown())
    {
        LMessageBox::showBox(this,WarningStr, ShowingStr,  LMessageBox::BUTTON_OK, true);
        followModeBtn->setStyleSheet(mainbuttonstyle);//倒计时结束颜色变浅
        return ;
    }

    zooidManager.setPlanningMode(FollowPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if(errorCode == NumberError)
    {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr,  LMessageBox::BUTTON_OK, true);
        generateCharge();
    }
    else
    {

        zooidManager.setGoalShow(false);
        //设置跟随模式时长
        zooidManager.zooidFollow->setTime(static_cast<unsigned int>(sqlConfig.FollowTime));
        // 设置起始位置
        zooidManager.zooidFollow->setBatonPosition(
                    Vector2(zooidManager.getWorldWidth() / 2.0,
                            zooidManager.getWorldHeight() / 2.0));
        //开始表演
        zooidManager.zooidFollow->begin();
        //启动倒计时
        coutdown->startCountdown(sqlConfig.FollowTime);

        isGraphical = false;
    }
    followModeBtn->setStyleSheet(mainButtonSelect);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}


void HomePage::enablePushWaveVoronoiMode()
{
    pushWaveVoronoiBtn->setStyleSheet(mainButtonSelect);
    if (coutdown->isCountDown()) {
        LMessageBox::showBox(this, WarningStr, ShowingStr, LMessageBox::BUTTON_OK, true);
        pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
        return;
    }

    zooidManager.setObstacleEnabled(false);
    zooidManager.setFormationMode(false);
    zooidManager.setPlanningMode(VoronoiPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if (errorCode == NumberError) {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr, LMessageBox::BUTTON_OK, true);
        generateCharge();
    } else {
        zooidManager.setGoalShow(false);
        zooidManager.startVoronoiMode();
        zooidManager.setPushWaveRobotMode(true);  // 必须在reset()之后
        isGraphical = false;
    }
    // 恢复其他按钮样式
    voronoiModeBtn->setStyleSheet(mainbuttonstyle);
    pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
    formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
    dualObstacleBtn->setStyleSheet(mainbuttonstyle);
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::enableFormationVoronoiMode()
{
    formationVoronoiBtn->setStyleSheet(mainButtonSelect);
    if (coutdown->isCountDown()) {
        LMessageBox::showBox(this, WarningStr, ShowingStr, LMessageBox::BUTTON_OK, true);
        formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
        return;
    }

    zooidManager.setFormationMode(true);       // 启用编队推波
    zooidManager.setPlanningMode(VoronoiPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if (errorCode == NumberError) {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr, LMessageBox::BUTTON_OK, true);
        generateCharge();
    } else {
        zooidManager.setGoalShow(false);
        zooidManager.startVoronoiMode();
        // 编队推波模式无时间限制
        isGraphical = false;
    }
    // 恢复其他按钮样式
    voronoiModeBtn->setStyleSheet(mainbuttonstyle);
    pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
    formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
    dualObstacleBtn->setStyleSheet(mainbuttonstyle);
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::enableCoverageOptimalMode()
{
    coverageOptimalBtn->setStyleSheet(mainButtonSelect);
    if (coutdown->isCountDown()) {
        LMessageBox::showBox(this, WarningStr, ShowingStr, LMessageBox::BUTTON_OK, true);
        coverageOptimalBtn->setStyleSheet(mainbuttonstyle);
        return;
    }

    zooidManager.setObstacleEnabled(true);   // 启用静态障碍物OAVC
    zooidManager.setFormationMode(false);
    zooidManager.setPlanningMode(VoronoiPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if (errorCode == NumberError) {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr, LMessageBox::BUTTON_OK, true);
        generateCharge();
    } else {
        zooidManager.setGoalShow(false);
        zooidManager.startVoronoiMode();
        zooidManager.setObstacleStatic();  // 强制静态障碍物（速度=0）
        isGraphical = false;
    }
    voronoiModeBtn->setStyleSheet(mainbuttonstyle);
    pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
    formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
    dualObstacleBtn->setStyleSheet(mainbuttonstyle);
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::enableDualObstacleMode()
{
    dualObstacleBtn->setStyleSheet(mainButtonSelect);
    if (coutdown->isCountDown()) {
        LMessageBox::showBox(this, WarningStr, ShowingStr, LMessageBox::BUTTON_OK, true);
        dualObstacleBtn->setStyleSheet(mainbuttonstyle);
        return;
    }

    zooidManager.setPlanningMode(VoronoiPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if (errorCode == NumberError) {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr, LMessageBox::BUTTON_OK, true);
        generateCharge();
    } else {
        zooidManager.setGoalShow(false);
        zooidManager.startVoronoiMode();
        zooidManager.setDualObstacleMode(true);  // 必须在reset()之后设置
        isGraphical = false;
    }
    // 恢复其他按钮样式
    voronoiModeBtn->setStyleSheet(mainbuttonstyle);
    pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
    formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
    dualObstacleBtn->setStyleSheet(mainbuttonstyle);
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::stopAllModes()
{
    zooidManager.stopTestMode();
    if (testModeStatusLabel != nullptr)
        testModeStatusLabel->setText(QStringLiteral("正在安全停止…"));
}

void HomePage::generateVoronoipath()
{
    voronoiModeBtn->setStyleSheet(mainButtonSelect);
    if(coutdown->isCountDown())
    {
        LMessageBox::showBox(this, WarningStr, ShowingStr, LMessageBox::BUTTON_OK, true);
        voronoiModeBtn->setStyleSheet(mainbuttonstyle);
        return;
    }

    zooidManager.setObstacleEnabled(false);  // 标准理想环境，无障碍物
    zooidManager.setFormationMode(false);    // 非编队模式
    zooidManager.setPlanningMode(VoronoiPlanning, MAX_FOLLOW_ZOOID_COUNT);
    ErrorCode errorCode = zooidManager.runPlanning();
    if(errorCode == NumberError)
    {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr, LMessageBox::BUTTON_OK, true);
        generateCharge();
    }
    else
    {
        zooidManager.setGoalShow(false);
        zooidManager.startVoronoiMode();
        // Voronoi 模式无时间限制
        isGraphical = false;
    }
    pushWaveVoronoiBtn->setStyleSheet(mainbuttonstyle);
    formationVoronoiBtn->setStyleSheet(mainbuttonstyle);
    dualObstacleBtn->setStyleSheet(mainbuttonstyle);
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainbuttonstyle);
}

void HomePage::selectGraphicalModal()
{
    if(coutdown->isCountDown())
    {
        LMessageBox::showBox(this,WarningStr, ShowingStr,  LMessageBox::BUTTON_OK, true);
        return ;
    }
    if((!zooidManager.isReachedGoalAll())&&(coutdown->getCountTime()>1))
    {
        LMessageBox::showBox(this,WarningStr, ShowingStr,  LMessageBox::BUTTON_OK, true);
        return ;
    }

    //图案选择
    LFigureModelBox figureModelBox(this);
    int state = figureModelBox.showBox();
    if(state == LFigureModelBox::Ok)
    {
        zooidManager.setGoalShow(sqlConfig.goalDisplay);
        int figureType = figureModelBox.getChooseResult();
        switch(figureType)
        {
        case LFigureModelBox::Cirecul:
            generateCircul();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;
        case LFigureModelBox::Triangle:
            generateTriangle();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;
        case LFigureModelBox::React:
            generateReact();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;
        case LFigureModelBox::Cross:
            generateCross();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;

        case LFigureModelBox::Hexagon:
            generateHexagon();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;
        case LFigureModelBox::Fivepointed:
            generateFivepointed();
            if(runPlanning())
            {
                coutdown->startCountdown(sqlConfig.FigureTime);
                isGraphical = true;
            }
            break;
        default:
            break;
        }
        QPixmap pix = figureModelBox.getSelectPixmap();
        zooidManager.zooidSimulator->setGraphicalImage(&pix);
    }
    followModeBtn->setStyleSheet(mainbuttonstyle);
    drawModeBtn->setStyleSheet(mainbuttonstyle);
    graphicalModeBtn->setStyleSheet(mainButtonSelect);
}

bool HomePage::runPlanning()
{
    ErrorCode errorCode = zooidManager.runPlanning();
    if(errorCode == NumberError)
    {
        LMessageBox::showBox(this, WarningStr, NoHaveRobotStr,  LMessageBox::BUTTON_OK, true);
        return false;
    }

    return true;
}

void HomePage::onAdminBtnClicked()
{
    LoginBox login(this);
    int state= login.showBox();
    if(state == LoginBox::Ok)
    {
        setCurrentWidget(AdminWidget);
        admin->move(this->x(),this->y());
        admin->exec();
    }
}


void HomePage::updateGraphical(){
    // 在图形模式下 所以机器人到达目标点
    int time = 4;
    if(isGraphical && zooidManager.isReachedGoalAll()){
        isGraphical = false;
        // 已经道道目标位置，但是剩余时间还很多，让其时间缩短
        if(coutdown->getCountTime()>time){
            coutdown->startCountdown(time);
        }
    }
}


void HomePage::readSqlConfigData()
{
    QSqlQuery query(m_db);
    if (query.exec(QStringLiteral("SELECT * FROM `config` WHERE `id` = 1000")) && query.first())
    {
        //读取行
        sqlConfig.EnabelFigureMode = query.value("EnabelFigureMode").toInt();
        sqlConfig.EnabelDrawMode = query.value("EnabelDrawMode").toInt();
        sqlConfig.EnabelFollowMode = query.value("EnabelFollowMode").toInt();
        sqlConfig.circularNumber = query.value("circularNumber").toInt();
        sqlConfig.crossNumber = query.value("crossNumber").toInt();
        sqlConfig.hexagonNumber = query.value("hexagonNumber").toInt();
        sqlConfig.triangleNumber = query.value("triangleNumber").toInt();
        sqlConfig.reactNumber = query.value("reactNumber").toInt();
        sqlConfig.fivepointedNumber = query.value("fivepointedNumber").toInt();
        sqlConfig.password = query.value("password").toString();
        sqlConfig.FollowTime = query.value("FollowTime").toInt();
        sqlConfig.DrawTime = query.value("DrawTime").toInt();
        sqlConfig.FigureTime = query.value("FigureTime").toInt();
        sqlConfig.batteryDisplay = query.value("batteryDisplay").toInt();
        sqlConfig.goalDisplay = query.value("goalDisplay").toInt();
        sqlConfig.screensaverDisplay = query.value("screensaverDisplay").toInt();
        sqlConfig.zoom = query.value("zoom").toFloat();
        sqlConfig.useTime = query.value("useTime").toInt();
        sqlConfig.waitTime = query.value("waitTime").toInt();
        sqlConfig.battery = query.value("battery").toUInt();
        sqlConfig.showCount = query.value("showCount").toUInt();
    }
    else
    {
        qWarning() << "Using built-in defaults because the config row could not be read:"
                   << query.lastError();
    }
}

void HomePage::setCurrentWidget(AppWidget widget)
{
    appWidget = widget;
}

AppWidget HomePage::getCurrentWidget()
{
    return appWidget;
}
