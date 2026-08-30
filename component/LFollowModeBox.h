#pragma once
#ifndef   LFOLLOWMODEBOX_H
#define   LFOLLOWMODEBOX_H

#include <QWidget>
#include <QLabel>
#include <QEventLoop>
#include <QPushButton>
#include <QCloseEvent>

class  LFollowModelBox :public QWidget
{
     Q_OBJECT
public:
    LFollowModelBox(QWidget *parent = 0);
    ~LFollowModelBox();

    enum MessageBtnType
    {
        BUTTON_ZONE,                //初始化
        BUTTON_OK,                  //确定
        BUTTON_LOOK                 //查看
    };

    void setTitle(QString title);
    int showBox(const QString & titleText, bool isModelWindow);

private:
    void  init();
    int   exec();

protected:
    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *event);

private slots:
    /**
     * @brief 关闭按钮单击事件
     */
    void onCloseClicked();

    /**
     * @brief 确定按钮单击事件
     */
    void onOkBtnClicked();

    /**
     * @brief onLookClicked
     */
    void onLookClicked();

private:
    QEventLoop         *m_eventLoop;
    MessageBtnType     m_chooseResult;
    QPushButton        *closeBtn;
    QLabel             *titlelabel;
};

#endif     //LFOLLOWMODEBOX_H
