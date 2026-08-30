#ifndef LFIGUREMODELBOX_H
#define LFIGUREMODELBOX_H
#pragma once

#include <QWidget>
#include <QDialog>
#include <QEventLoop>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QDebug>
#include <QPainter>
#include <QMouseEvent>


#include "public/style.h"

#include "LSlideSelector.h"

class LFigureModelBox : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief Constructor
     */
    LFigureModelBox(QWidget *parent = nullptr);

    ~LFigureModelBox();

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
     * @brief 图形类型
     */
    enum FigureType
    {
        Cirecul,                //圆形
        Triangle,               //三角形
        React,                  //矩形
        Cross,                  //十字
        Hexagon,                //六边形
        Fivepointed,            //五角星
    };

    /**
     * @brief 设置当前选中
     * @param 当前选中的按钮类型
     */
    void setChooseResult(FigureType  chooseResult)
    {
        m_chooseResult = chooseResult;
    }

    /**
     * @brief 获取当前选中状态
     * @return  返回选中按钮的类型
     */
    FigureType  getChooseResult()
    {
        return m_chooseResult;
    }

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
     * @brief setButtonBackground
     * @param btn
     * @param url
     */
    void setButtonBackground(QPushButton *btn, QString url);

    /**
     * @brief getSelectPixmap
     * @param url
     * @return
     */
    QPixmap getSelectPixmap();

    /**
     * @brief setSelectPixmap
     * @param url
     */
    void setSelectPixmap(QString url);

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

    void onBtn1Clicked();
    void onBtn2Clicked();
    void onBtn3Clicked();
    void onBtn4Clicked();
    void onBtn5Clicked();
    void onBtn6Clicked();


private:
    FigureType m_chooseResult;
    ButtonType m_buttonResult;
    QEventLoop *m_eventLoop;

    QLabel *titleLab;
    QPushButton *closeBtn;
    QPushButton* okayBtn;

    QPushButton *btn1;
    QPushButton *btn2;
    QPushButton *btn3;
    QPushButton *btn4;
    QPushButton *btn5;
    QPushButton *btn6;

    QLabel *lab1;
    QLabel *lab2;
    QLabel *lab3;
    QLabel *lab4;
    QLabel *lab5;
    QLabel *lab6;

    QPixmap selectPixmap;

};

#endif // LFIGUREMODELBOX_H
