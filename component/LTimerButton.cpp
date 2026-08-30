#include "LTimerButton.h"

LTimerButton::LTimerButton(QWidget *parent): QWidget(parent)
{
    setMinimumSize(110,40);
    setFixedSize(110,40);
    setCursor(Qt::PointingHandCursor);

    //按钮背景色
    backgroundColor = QColor(255,255,255,100);

    //文本颜色
    textColor =  QColor(255,255,255);

    //默认文本
    text = QString(QStringLiteral("关闭"));

    timer = new QTimer();
    timer->setInterval(1000);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(countDownTimerRun()));
}

void LTimerButton::setTimeStart(int time)
{
    countTimer = std::min(std::max(0,time), 1000);
    timer->start();
}

void LTimerButton::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);

    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.setBrush(backgroundColor);
    //绘制左半圆
    QRectF leftSemicircleRect(0, 0, height(), height());
    painter.drawPie(leftSemicircleRect,  90 * 16, 180 * 16);

    //绘制中间矩形
    painter.drawRect(static_cast<int>(height()/2.0f), 0, width()-height(),height());

    //绘制右半圆
    QRectF rightSemicircleRect(width() - height(), 0, height(), height());
    painter.drawPie(rightSemicircleRect, 90*16, -180*16);

    QString timeText = "";
    if(countTimer>0)
    {
        textColor =  QColor(255,255,255);
        timeText += QString::number(countTimer) +  QStringLiteral("秒 ");
    }
    else
    {
        textColor =  QColor("#00ff00");
    }

    timeText += text;

    //绘制文本
    painter.setPen(textColor);
    QFont font("Microsoft YaHei", 10, QFont::Bold);
    painter.setFont(font);
    int textWidth = painter.fontMetrics().width(timeText);
    painter.drawText((width() - textWidth) / 2, 26, timeText);

}

void LTimerButton::mousePressEvent(QMouseEvent *event)
{
    if (isEnabled())
    {
        if (event->buttons() & Qt::LeftButton)
        {
            event->accept();
        } else
        {
            event->ignore();
        }
    }
}

void LTimerButton::mouseReleaseEvent(QMouseEvent *event)
{
    if (isEnabled())
    {
        if ((event->type() == QMouseEvent::MouseButtonRelease) && (event->button() == Qt::LeftButton))
        {
            event->accept();
            if(countTimer == 0)
            {
                emit clicked();
            }
            update();
        }
        else
        {
            event->ignore();
        }
    }
}

void LTimerButton::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}



void LTimerButton::countDownTimerRun()
{
    countTimer--;
    if(countTimer <= 0)
    {
        countTimer = 0;
        timer->stop();
        emit countDownEvent();
    }
    update();
}
