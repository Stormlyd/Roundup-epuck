#ifndef LSLIDESELECTOR_H
#define LSLIDESELECTOR_H

#include <QWidget>
#include <QPropertyAnimation>
#include <QPainter>
#include <QLabel>
#include <QDebug>
#include <QPushButton>
#include <QWheelEvent>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>

#include "public/config.h"
#include "public/style.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

class LSlideSelector : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief 构造
     * @param parent
     */
    explicit LSlideSelector(QWidget *parent = nullptr);
   ~LSlideSelector();

    enum PosType {
        Type_LL,
        Type_L,
        Type_M,
        Type_R,
        Type_RR
    };

    /**
     * @brief 初始化设置
     */
    void setup();

    /**
     * @brief 初始化界面
     */
    void initGUI();

    /**
     * @brief 初始化, 一定要在additem后调用
     */
    void init();

    /**
     * @brief 添加子项
     * @param value     值
     * @param name      名称
     * @param iconPath  图标路径
     */
    void addItem(int value, QString name, QString iconPath);

    //----add by chenlu----
    /**
     * @brief 添加子项
     * @param iconPath  图标路径
     */
    void addItem(QString iconPath);


    /**
     * @brief 当前选中索引
     * @return
     */
    int getIndex(){return currentIndex;}

    /**
     * @brief 获取当前选中索引对应的值
     * @return
     */
    int getValue(){return values[currentIndex];}

    /**
     * @brief 上一页
     */
    void scrollPre();

    /**
     * @brief 下一页
     */
    void scrollNext();

    // ---------add by chenlu--------------
    /**
     * @brief 往左移
     */
    void scrollLeft();
    /**
     * @brief 往右移
     */
    void scrollRight();
    // ---------add by chenlu--------------
protected:

    /**
     * @brief wheelEvent
     * @param event
     */
    void wheelEvent(QWheelEvent *event) override;

    /**
     * @brief mousePressEvent
     * @param event
     */
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief mouseReleaseEvent
     * @param event
     */
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

public:

    QPoint getPos(PosType type);

    QWidget *getWidget(PosType type);
    QWidget *getFirstWidget(PosType type);

    void initFistSight();

    void initAnimation();


private:
    QLabel *indexNameLab;           //选中值名称标签

    int currentIndex;               //当前选中值
    QList<QString> names;           //图标名称
    QList<int> values;              //值列表

    QList<QWidget *> m_widgetList;
    QAnimationGroup *m_animationGroup;

    int posx;

    //------add by chenlu-----
     QList<QWidget *> m_widgetList2;
};

#endif // LSLIDESELECTOR_H
