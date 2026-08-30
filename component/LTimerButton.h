#ifndef LTIMERBUTTON_H
#define LTIMERBUTTON_H


#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QTimer>

#include <algorithm>

class LTimerButton : public QWidget
{
    Q_OBJECT
public:
    explicit LTimerButton(QWidget *parent = nullptr);

    /**
     * @brief 设置开始倒计时
     * @param time  要设置的时间 单位:秒
     */
    void setTimeStart(int time);
protected:
    /**
     * @brief 重绘样式
     */
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 鼠标按下事件
     * @param event
     */
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 鼠标释放事件切换开关状态发射toggled()信号
     * @param event
     */
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

    /**
     * @brief 大小改变事件
     * @param event
     */
    void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;

public slots:
    /**
     * @brief 定时器事件
     */
    void countDownTimerRun();

signals:
    /**
     * @brief 倒计时完成发送信号
     */
    void countDownEvent();

    /**
     * @brief 鼠标单击信号
     */
    void clicked();

private:
    QTimer *timer;                  //倒计时定时器
    int countTimer;                 //倒计时计数器

    QColor backgroundColor;         //按钮背景色
    QString text;                   //按钮文本
    QColor textColor;               //文本颜色
};

#endif // LTIMERBUTTON_H
