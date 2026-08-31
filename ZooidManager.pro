#-------------------------------------------------
#
# Project created by QtCreator 2020-02-14T15:10:58
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ZooidManager
TEMPLATE = app

QT += serialport
QT += widgets
QT += opengl
QT += sql
QT += multimedia multimediawidgets


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++14

SOURCES += main.cpp\
    public/pf.cpp \
    orca/Agent.cpp \
    orca/Goal.cpp \
    orca/KdTree.cpp \
    orca/Simulator.cpp \
    orca/Vector2.cpp \
    component/LSwitchButton.cpp \
    component/LMessageBox.cpp \
    component/LPanel.cpp \
    component/LNumberInputBox.cpp \
    component/LSliderBox.cpp \
    module/memwatch.c \
    manager/Zooid.cpp \
    manager/ZooidAlgorithm.cpp \
    manager/ZooidDraw.cpp \
    manager/ZooidFollow.cpp \
    manager/ZooidGoal.cpp \
    manager/ZooidManager.cpp \
    manager/ZooidMessage.cpp \
    manager/ZooidReceiver.cpp \
    manager/ZooidSerialport.cpp \
    manager/ZooidSimulator.cpp \
    manager/ZooidCharge.cpp \
    manager/ZooidTestMode.cpp \
    manager/ZooidCoordinates.cpp \
    manager/ZooidCbfSafety.cpp \
    manager/ZooidSpeedCodec.cpp \
    manager/ZooidTestTargets.cpp \
    manager/ZooidPursuitRoles.cpp \
    manager/ZooidPursuitGeometry.cpp \
    manager/ZooidPursuitStateMachine.cpp \
    manager/ZooidPursuitControl.cpp \
    adminPage.cpp \
    homePage.cpp \
    component/LFigureModelBox.cpp \
    component/LFollowModeBox.cpp \
    component/LShowViewBox.cpp \
    component/LoginBox.cpp \
    component/LDrawModelbox.cpp \
    component/LCountdownBox.cpp \
    component/LTimerButton.cpp \
    component/LSlideSelector.cpp \
    log/log.cpp \
    manager/ZooidVoronoi.cpp \
    component/LVoronoiViewBox.cpp

HEADERS  += \
    public/config.h \
    public/pf.h \
    public/style.h \
    orca/Agent.h \
    orca/definitions.h \
    orca/Goal.h \
    orca/KdTree.h \
    orca/Simulator.h \
    orca/Vector2.h \
    component/LSwitchButton.h \
    component/LMessageBox.h \
    component/LPanel.h \
    component/LNumberInputBox.h \
    component/LSliderBox.h \
    module/memwatch.h \
    manager/Zooid.h \
    manager/ZooidAlgorithm.h \
    manager/ZooidDraw.h \
    manager/ZooidFollow.h \
    manager/ZooidGoal.h \
    manager/Zooidinfo.h \
    manager/ZooidManager.h \
    manager/ZooidMessage.h \
    manager/ZooidReceiver.h \
    manager/ZooidSerialport.h \
    manager/ZooidSimulator.h \
    manager/ZooidCharge.h   \
    manager/ZooidTestMode.h \
    manager/ZooidCoordinates.h \
    manager/ZooidCbfSafety.h \
    manager/ZooidSpeedCodec.h \
    manager/ZooidTestTargets.h \
    manager/ZooidWheelCommand.h \
    manager/ZooidPursuitTypes.h \
    manager/ZooidPursuitRoles.h \
    manager/ZooidPursuitGeometry.h \
    manager/ZooidPursuitStateMachine.h \
    manager/ZooidPursuitControl.h \
    adminPage.h \
    homePage.h \
    public/errorcode.h \
    component/LFigureModelBox.h \
    component/LFollowModeBox.h \
    component/LShowViewBox.h \
    public/config.h \
    public/errorcode.h \
    public/pf.h \
    public/style.h \
    component/LoginBox.h \
    component/LDrawModelbox.h \
    component/LCountdownBox.h \
    public/sqlconfig.h \
    component/LTimerButton.h \
    component/LSlideSelector.h \
    log/log.h \
    public/language.h \
    manager/ZooidVoronoi.h \
    component/LVoronoiViewBox.h

FORMS    +=

RC_ICONS = zooid.ico

RESOURCES += \
    image.qrc \
    qss.qrc

DISTFILES += \
    json/chargePosition.json
