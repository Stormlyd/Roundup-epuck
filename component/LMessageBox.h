#ifndef LMESSAGE_BOX
#define LMESSAGE_BOX

#include <QWidget>
#include <QEventLoop>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>

#include "public/style.h"

class LMessageBox : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief LMessageBox
     * @param parent
     */
    LMessageBox(QWidget *parent = nullptr);

    ~LMessageBox();

    enum ChosseResult
    {
        ID_OK = 0,                      // 确定;
        ID_CANCEL                       // 取消;
    };

    enum MessageButtonType
    {
        BUTTON_OK = 0,                  // 确定
        BUTTON_CANCEL,                  // 取消;
        BUTTON_OK_AND_CANCEL,           // 确定取消
        BUTTON_CLOSE                    // 关闭
    };

    /**
     * @brief 设置标题
     * @param title 要设置的标题
     */
    void setTitleText(QString title);

    /**
     * @brief 设置内容
     * @param contentText   要设置的内容
     */
    void setContentText(QString contentText);

    /**
     * @brief 设置按钮类型
     * @param buttonType    按钮类型
     */
    void setButtonType(MessageButtonType buttonType);

public:
    /**
     * @brief 显示信息框
     * @param parent            设置当前父布局
     * @param titleText         设置标题
     * @param contentText       设置内容
     * @param messageButtonType 设置按钮类型
     * @param isModelWindow     设置模式 true阻塞 false非阻塞
     * @return
     */
    int static showBox(QWidget* parent,const QString &titleText,const QString &contentText ,MessageButtonType messageButtonType ,bool isModelWindow = false);

private:
    /**
     * @brief 初始化
     */
    void init();

    /**
     * @brief 阻塞显示
     * @return
     */
    int exec();

protected:
    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *event);

private slots:
    /**
     * @brief 确定按钮单击事件
     */
    void onOkClicked();

    /**
     * @brief 取消按钮单击事件
     */
    void onCancelClicked();

private:
    QEventLoop* m_eventLoop;
    ChosseResult m_chooseResult;
    QPushButton *okBtn;
    QPushButton *cancelBtn;
    QPushButton *closeBtn;
    QLabel *titleLab;
    QLabel *contentLab;

    QVBoxLayout *messageLayout;
    QHBoxLayout *topLayout;
    QHBoxLayout *contentLayout;
    QHBoxLayout *buttonLayout;

};


#endif // LMESSAGE_BOX

