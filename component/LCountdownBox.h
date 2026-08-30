#ifndef LCOUNTDOWNBOX_H
#define LCOUNTDOWNBOX_H

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QPainter>

#include "public/style.h"

class LCountdownBox:public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief LCountdownBox
     * @param parent
     */
    LCountdownBox(QWidget *parent = nullptr);
    ~LCountdownBox();

    /**
     * @brief 倒计时开始
     * @param time
     */
    void startCountdown(int time);

    /**
     * @brief 是否在倒计时中
     * @return  返回状态
     */
    bool isCountDown(){ return countTime > 0;}

    /**
     * @brief 获取当前倒计时
     * @return
     */
    int getCountTime();

    /**
     * @brief 获取倒计时文本
     * @return 时间文本值
     */
    QString getTimeText();

    /**
     * @brief 结束倒计时
     */
    void endCountdown();


public slots:
    /**
     * @brief countDownTimerRun
     */
    void countDownTimerRun();

signals:
    /**
     * @brief 倒计时完成发送信号
     */
    void countDownEvent();


private:
    QLabel *timeView;
    QLabel *secondLab;

int   countTime;
    QTimer *countDownTimer;
};

#endif // LCOUNTDOWNBOX_H
