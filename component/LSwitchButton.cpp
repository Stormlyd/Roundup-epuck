#include "LSwitchButton.h"

LSwitchButton::LSwitchButton(QWidget *parent): QWidget(parent)
{
    setCursor(Qt::PointingHandCursor);
    setMinimumSize(60,30);
    setFixedSize(60,30);
    _checked = false;
    _checkedColor = QColor(104, 216, 254);
    _thumbColor = Qt::white;
    _background = QColor(210, 210, 210);
    _disabledColor = QColor(190, 190, 190);
}

void LSwitchButton::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing);

    QPainterPath path;
    QColor background;
    QColor thumbColor;
    if (isEnabled())
    {
        if (_checked)
        {
            background = _checkedColor;
            thumbColor = _thumbColor;
        }
        else
        {
            background = _background;
            thumbColor = _thumbColor;
        }
    }
    else
    {
        background = _background;
        thumbColor = _disabledColor;
    }

    // 绘制滑轨
    painter.setBrush(background);
    painter.drawEllipse(0, 0, height(), height());
    painter.drawEllipse(width() - height(), 0, height(), height());
    painter.drawRect(height()/2.0f, 0, width()-height(),height());

    // 绘制滑块
    painter.setBrush(thumbColor);
    painter.drawEllipse((width() - height()) * _checked + 2, 2, height()-4, height()-4);

}


void LSwitchButton::mousePressEvent(QMouseEvent *event)
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

void LSwitchButton::mouseReleaseEvent(QMouseEvent *event)
{
    if (isEnabled())
    {
        if ((event->type() == QMouseEvent::MouseButtonRelease) && (event->button() == Qt::LeftButton))
        {
            event->accept();
            _checked = !_checked;
            emit toggled(_checked);
            update();
        }
        else
        {
            event->ignore();
        }
    }
}

void LSwitchButton::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}

bool LSwitchButton::isToggled() const
{
    return _checked;
}

void LSwitchButton::setToggle(bool checked)
{
    _checked = checked;
}

void LSwitchButton::setBackgroundColor(QColor color)
{
    _background = color;
}

void LSwitchButton::setCheckedColor(QColor color)
{
    _checkedColor = color;
}

void LSwitchButton::setDisbaledColor(QColor color)
{
    _disabledColor = color;
}
