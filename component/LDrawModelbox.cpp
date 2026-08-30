#include "LDrawModelBox.h"
#include <QDebug>

LDrawModelBox::LDrawModelBox(QWidget *parent) : QWidget(parent)
{
    init();
}

LDrawModelBox::~LDrawModelBox()
{
    delete zooidDraw;
    zooidDraw = nullptr;

    delete m_eventLoop;
    m_eventLoop = nullptr;

    delete titleLab;
    titleLab = nullptr;

    delete closeBtn;
    closeBtn = nullptr;

    delete clearBtn;
    clearBtn = nullptr;

    delete okayBtn;
    okayBtn = nullptr;

    delete demonstrationBtn;
    demonstrationBtn = nullptr;
}

int LDrawModelBox::showBox()
{
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_buttonResult;
}

void LDrawModelBox::init()
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setWindowModality(Qt::WindowModal);
    resize(960, 600);
    move(50,118);

    //标题
    titleLab = new QLabel(QStringLiteral("绘图模式"));
    titleLab->setParent(this);
    titleLab->setStyleSheet(font_size_lg + font_weight_600 + text_info);
    titleLab->move(18,15);

    //关闭按钮
    closeBtn = new QPushButton(this);
    closeBtn->setIcon(QIcon(":/new/ofapp/res/images/close.png"));
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(transparent_bg);
    closeBtn->move(width()-50,15);

    //绘图模式Frame
    zooidDraw = new ZooidDraw(this);
    zooidDraw->move(40,70);
    zooidDraw->setStyleSheet(border_info_1);

    //清空按钮
    clearBtn = new QPushButton(QStringLiteral("清空"));
    clearBtn->setParent(this);
    clearBtn->setCursor(Qt::PointingHandCursor);
    //clearBtn->setStyleSheet(border_info_2 + text_info + rounded_lg + font_size_lg + font_weight_600);
    clearBtn->setStyleSheet(definiteBtnStyle1);
    clearBtn->setFixedSize(128,48);
    clearBtn->move(width()/2 - 138, height()-80);

    //确定按钮
    okayBtn = new QPushButton(QStringLiteral("确定"));
    okayBtn->setParent(this);
    okayBtn->setCursor(Qt::PointingHandCursor);
    //okayBtn->setStyleSheet(border_info_2 + text_info + rounded_lg + font_size_lg + font_weight_600);
    okayBtn->setStyleSheet(definiteBtnStyle2);
    okayBtn->setFixedSize(128,48);
    okayBtn->move(width()/2 + 10, height()-80);

    //查看演示按钮
    demonstrationBtn = new QPushButton(QStringLiteral("查看演示"));
    demonstrationBtn->hide();
    demonstrationBtn->setCursor(Qt::PointingHandCursor);
    demonstrationBtn->setStyleSheet(transparent_bg + text_info + font_size_md + font_weight_700);
    demonstrationBtn->setParent(this);
    demonstrationBtn->move(width()-110, height()-50);

    //连接槽
    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(clearBtn, SIGNAL(clicked()), this, SLOT(onClearClicked()));
    QObject::connect(okayBtn, SIGNAL(clicked()), this, SLOT(onOkayClicked()));
    QObject::connect(demonstrationBtn, SIGNAL(clicked()), this, SLOT(onDemonstrationClicked()));
}

void LDrawModelBox::paintEvent(QPaintEvent *event)
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

void LDrawModelBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }
    event->accept();
}

void LDrawModelBox::onCloseClicked()
{
    m_buttonResult = Close;
    close();
}

void LDrawModelBox::onOkayClicked()
{
    m_buttonResult = Ok;
    close();
}

void LDrawModelBox::onClearClicked()
{
    zooidDraw->clearPath();
}

void LDrawModelBox::onDemonstrationClicked()
{
    m_buttonResult = Demonstration;
    close();
}

