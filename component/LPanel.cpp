#include "LPanel.h"
#include <QDebug>
#include <QStylePainter>
#include <QStyleOption>

/**
 * @brief LPanel::LPanel
 * @param parent
 */
LPanel::LPanel(QWidget *parent,  Style style) : QWidget(parent)
{

    m_style = style;
    setAttribute(Qt::WA_TranslucentBackground, true); //透明背景

}

void LPanel::paintEvent(QPaintEvent *event)
{
    //绘制背景 背景始终不变形
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);



    switch(m_style)
    {
        case One:
        {
            painter.drawPixmap(QRect(0, 0,42, 42), QPixmap(":/new/ofapp/res/images/border1_lt.png"));
            painter.drawPixmap(QRect(width()-42, 0,42, 42), QPixmap(":/new/ofapp/res/images/border1_rt.png"));
            painter.drawPixmap(QRect(0, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border1_lb.png"));
            painter.drawPixmap(QRect(width()-42, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border1_rb.png"));
            painter.drawPixmap(QRect(42, 0,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_t.png"));
            painter.drawPixmap(QRect(0, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_l.png"));
            painter.drawPixmap(QRect(width()-20, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_r.png"));
            painter.drawPixmap(QRect(42, height()-20,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_b.png"));
        }
        break;
        case Two:
        {
            QPainter painter(this);
            QPainterPath pathBack;
            pathBack.setFillRule(Qt::WindingFill);
            pathBack.addRect(QRect(0, 0, this->width(), this->height()));
            painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
            painter.fillPath(pathBack, QBrush(QColor("#101129")));
            painter.drawPixmap(QRect(0, 0,42, 42), QPixmap(":/new/ofapp/res/images/border2_lt.png"));
            painter.drawPixmap(QRect(width()-42, 0,42, 42), QPixmap(":/new/ofapp/res/images/border2_rt.png"));
            painter.drawPixmap(QRect(0, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border2_lb.png"));
            painter.drawPixmap(QRect(width()-42, height()-42,42, 42), QPixmap(":/new/ofapp/res/images/border2_rb.png"));
            painter.drawPixmap(QRect(42, 0,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_t.png"));
            painter.drawPixmap(QRect(0, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_l.png"));
            painter.drawPixmap(QRect(width()-20, 42,20, height()-84), QPixmap(":/new/ofapp/res/images/line_r.png"));
            painter.drawPixmap(QRect(42, height()-20,width()-84, 20), QPixmap(":/new/ofapp/res/images/line_b.png"));
        }
        break;
        default:
        {
            painter.drawPixmap(QRect(0, 0,138, 46), QPixmap(":/new/ofapp/res/images/border0_lt.png"));
            painter.drawPixmap(QRect(width()-100, 0,100, 60), QPixmap(":/new/ofapp/res/images/border0_rt.png"));
            painter.drawPixmap(QRect(0, height()-36,164, 36), QPixmap(":/new/ofapp/res/images/border0_lb.png"));
            painter.drawPixmap(QRect(width()-20, height()-23,20, 23), QPixmap(":/new/ofapp/res/images/border0_rb.png"));
            painter.drawPixmap(QRect(138, 0,width()-238, 16), QPixmap(":/new/ofapp/res/images/line0_t.png"));
            painter.drawPixmap(QRect(0, 46,19, height()-82), QPixmap(":/new/ofapp/res/images/line0_l.png"));
            painter.drawPixmap(QRect(164, height()-22,width()-184, 22), QPixmap(":/new/ofapp/res/images/line0_b.png"));
            painter.drawPixmap(QRect(width()-19, 60,19, height()-60), QPixmap(":/new/ofapp/res/images/line0_r.png"));
        }
    }

}


