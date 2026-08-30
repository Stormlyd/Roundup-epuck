#ifndef LPANELL_H
#define LPANELL_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include "manager/ZooidManager.h"
class LPanel : public QWidget
{
    Q_OBJECT
public:
    enum Style
    {
        Default = 0,
        One,
        Two,
    };

    explicit LPanel(QWidget *parent = nullptr, Style style=Default);
protected:
    void paintEvent(QPaintEvent *event);
private:
    Style m_style;

signals:

public slots:

public:


};

#endif // LPANELL_H
