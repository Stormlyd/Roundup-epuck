#include "LNumberInputBox.h"
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

LNumberInputBox::LNumberInputBox(QWidget *parent): QWidget(parent)
{
    maxValue = 9999;
    m_eventLoop = nullptr;
    m_chooseResult = ID_CLOSE;
    init();
}

LNumberInputBox::~LNumberInputBox()
{
    delete closeBtn;
    closeBtn = nullptr;

    delete m_eventLoop;
    m_eventLoop = nullptr;

    delete titleLab;
    titleLab = nullptr;

    delete closeBtn;
    closeBtn = nullptr;

    delete okayBtn;
    okayBtn = nullptr;

    delete backspaceBtn;
    backspaceBtn = nullptr;

    delete display;
    display = nullptr;

    for(int i=0; i<NumDigitButtons; i++)
    {
        delete(digitButtons[i]);
        digitButtons[i] = nullptr;
    }

}

void LNumberInputBox::init()
{
    setWindowFlags(Qt::Window|Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground, true);
    setFixedSize(270, 460);
    QVBoxLayout *layout = new QVBoxLayout(this);

    //标题
    QHBoxLayout *topLayout = new QHBoxLayout();
    layout->addLayout(topLayout);

    titleLab = new QLabel(QStringLiteral("提示"));
    titleLab->setStyleSheet(font_size_lg + font_weight_600 + text_info);

    //关闭按钮
    closeBtn = new QPushButton();
    closeBtn->setIcon(QIcon(":/new/ofapp/res/images/close.png"));
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(transparent_bg);

    topLayout->addWidget(titleLab);
    topLayout->addStretch();
    topLayout->addWidget(closeBtn);
    layout->addStretch();

    display = new QLabel("0");
    display->setAlignment(Qt::AlignCenter);
    display->setStyleSheet(font_size_28 + font_weight_600 + text_info + transparent_bg);
    display->setFixedHeight(60);

    layout->addWidget(display);
    layout->addStretch();

    QString btnStyle(font_size_lg + font_weight_600 + text_info + transparent_bg + border_info_2 + rounded_sm);

    for (int i = 0; i < NumDigitButtons; ++i)
    {
        digitButtons[i] = new QPushButton(QString::number(i));
        digitButtons[i]->setStyleSheet(btnStyle);
        digitButtons[i]->setFixedSize(80,80);
        QObject::connect(digitButtons[i], SIGNAL(clicked()), this, SLOT(onDigitClicked()));
    }

    okayBtn = new QPushButton(QStringLiteral("确定"));
    okayBtn->setStyleSheet(btnStyle);
    okayBtn->setFixedSize(80,80);

    backspaceBtn = new QPushButton(QStringLiteral("退格"));
    backspaceBtn->setStyleSheet(btnStyle);
    backspaceBtn->setFixedSize(80,80);

    //定义栅格布局 排列键盘按钮
    QGridLayout* keyLayout = new QGridLayout();
    keyLayout->addWidget(digitButtons[1],0,0,1,1);
    keyLayout->addWidget(digitButtons[2],0,1,1,1);
    keyLayout->addWidget(digitButtons[3],0,2,1,1);
    keyLayout->addWidget(digitButtons[4],1,0,1,1);
    keyLayout->addWidget(digitButtons[5],1,1,1,1);
    keyLayout->addWidget(digitButtons[6],1,2,1,1);
    keyLayout->addWidget(digitButtons[7],2,0,1,1);
    keyLayout->addWidget(digitButtons[8],2,1,1,1);
    keyLayout->addWidget(digitButtons[9],2,2,1,1);
    keyLayout->addWidget(okayBtn,3,0,1,1);
    keyLayout->addWidget(digitButtons[0],3,1,1,1);
    keyLayout->addWidget(backspaceBtn,3,2,1,1);

    layout->addLayout(keyLayout);
    layout->addStretch();

    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(okayBtn, SIGNAL(clicked()), this, SLOT(onOkayClicked()));
    QObject::connect(backspaceBtn, SIGNAL(clicked()), this, SLOT(onBackspaceClicked()));
}

void LNumberInputBox::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    QPainterPath pathBack;
    pathBack.setFillRule(Qt::WindingFill);
    pathBack.addRect(QRect(0, 0, this->width(), this->height()));
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

void LNumberInputBox::setTitleText(QString title)
{
    titleLab->setText(title);
}

int LNumberInputBox::showBox(const QString &titleText, bool isModelWindow)
{
    setTitleText(titleText);
    if (isModelWindow)
    {
        return exec();
    }
    else
    {
        show();
    }

    return -1;
}

int LNumberInputBox::exec()
{
    this->setWindowModality(Qt::WindowModal);
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_chooseResult;
}

void LNumberInputBox::onOkayClicked()
{
    m_chooseResult = ID_OK;
    close();
}

void LNumberInputBox::onCloseClicked()
{
    m_chooseResult = ID_CLOSE;
    close();
}

void LNumberInputBox::onDigitClicked()
{
    QPushButton* clickedButton = qobject_cast<QPushButton*>(sender());

    QString value = display->text() + clickedButton->text();
    if(value.toInt() > maxValue)
    {
        return ;
    }

    int digitValue = clickedButton->text().toInt();
    if (display->text() == "0")
    {
        display->setText("");
    }
    display->setText(display->text() + QString::number(digitValue));
}

void LNumberInputBox::onBackspaceClicked()
{
    QString text = display->text();
    text.chop(1);
    if (text.isEmpty())
    {
        text = "0";
    }
    display->setText(text);
}

int LNumberInputBox::getValue()
{
    return  display->text().toInt();
}

void LNumberInputBox::setMaxValue(int _maxValue)
{
    maxValue = _maxValue;
}

void LNumberInputBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }

    event->accept();
}
