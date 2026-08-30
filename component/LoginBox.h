#ifndef LOGINBOX_H
#define LOGINBOX_H

#pragma once

#include <QDialog>
#include <QLabel>
#include <QtSql/QSqlQuery>
#include <QPushButton>
#include <QPainter>
#include <QPushButton>
#include <QMessageBox>

#include "public/style.h"
#include "adminPage.h"


class LoginBox : public QWidget
{
    Q_OBJECT
public:
    explicit LoginBox(QWidget *parent=0);
    ~LoginBox();

    /**
     * @brief 按钮类型枚举
     */
    enum ButtonType
    {
        Ok,                     //确定
        Close,                  //关闭
    };

    /**
     * @brief 登录
     * @param pass
     * @return 登录成功 返回true  否则 false
     */
    bool onLogin(QString pass);

    /**
     * @brief 显示模态窗口
     * @param parent    设置当前父窗口
     * @return          返回当前选中的结果
     */
    int showBox();

protected:
    /**
     * @brief paintEvent
     * @param event
     */
    void paintEvent(QPaintEvent *);

    /**
     * @brief closeEvent
     * @param event
     */
    void closeEvent(QCloseEvent *event);

signals:
    void isLogin();

private slots:
    /**
     * @brie 关闭事件
     */
     void onCloseClicked();

     /**
      * @brief onOkayClicked
      */
     void  onOkayClicked();

private:
    QLineEdit  *passEdit;
    ButtonType m_buttonResult;
    QEventLoop *m_eventLoop;

    QLabel *titleLab;
    QPushButton *closeBtn;
    QLabel *passwordLab;
    QPushButton* okayBtn;
    QLabel *messageLab;

};
#endif // LOGINBOX_H
