#ifndef LDrawModelBox_H
#define LDrawModelBox_H

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QEventLoop>

#include "public/style.h"
#include "manager/ZooidDraw.h"


class LDrawModelBox : public  QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     */
    LDrawModelBox(QWidget *parent = nullptr);

    ~LDrawModelBox();

    /**
     * @brief 按钮类型枚举
     */
    enum ButtonType
    {
        Ok,                     //确定
        Close,                  //关闭
        Demonstration,          //演示按钮
    };


    /**
     * @brief 显示模态窗口
     * @param parent    设置当前父窗口
     * @return          返回当前选中的结果
     */
    int showBox();

    /**
     * @brief 初始化模态窗口
     */
    void init();

    /**
     * @brief 自绘器
     */
    ZooidDraw  *zooidDraw;

protected:
    /**
     * @brief paintEvent
     * @param event
     */
    void paintEvent(QPaintEvent *event);

    /**
     * @brief closeEvent
     * @param event
     */
    void closeEvent(QCloseEvent *event);

private slots:

    /**
     * @brief 关闭按钮单击事件
     */
    void onCloseClicked();

    /**
     * @brief 确定按钮单击事件
     */
    void onOkayClicked();

    /**
     * @brief 清除画板按钮单击事件
     */
    void onClearClicked();

    /**
     * @brief 演示按钮单击事件
     */
    void onDemonstrationClicked();

private:
    ButtonType m_buttonResult;
    QEventLoop *m_eventLoop;

    QLabel *titleLab;
    QPushButton *closeBtn;
    QPushButton* clearBtn;
    QPushButton* okayBtn;
    QPushButton* demonstrationBtn;

};

#endif // LDrawModelBox_H
