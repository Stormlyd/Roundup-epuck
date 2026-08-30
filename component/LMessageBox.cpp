#include <QPainter>
#include <QMouseEvent>
#include "LMessageBox.h"
#include <QDebug>

LMessageBox::LMessageBox(QWidget *parent): QWidget(parent),
    m_eventLoop(nullptr),
    m_chooseResult(ID_CANCEL)
{
    init();
}

LMessageBox::~LMessageBox()
{
    delete okBtn;
    okBtn = nullptr;

    delete closeBtn;
    closeBtn = nullptr;

    delete cancelBtn;
    cancelBtn = nullptr;

    delete buttonLayout;
    buttonLayout = nullptr;

    delete topLayout;
    topLayout = nullptr;

    delete messageLayout;
    messageLayout = nullptr;
}

void LMessageBox::init()
{
    setWindowFlags(Qt::Window|Qt::FramelessWindowHint);
    setAttribute(Qt::WA_TranslucentBackground, true);

    setMinimumHeight(240);
    setMinimumWidth(480);
    messageLayout = new QVBoxLayout(this);

    //顶部布局
    topLayout = new QHBoxLayout();
    messageLayout->addLayout(topLayout);

    //标题
    titleLab = new QLabel(QStringLiteral("提示"));
    titleLab->setStyleSheet(font_size_lg + font_weight_600 + text_info);

    //关闭按钮
    closeBtn = new QPushButton();
    QIcon closeBtnIco(":/new/ofapp/res/images/close.png");
    closeBtn->setIcon(closeBtnIco);
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(transparent_bg);

    topLayout->addWidget(titleLab);
    topLayout->addStretch();
    topLayout->addWidget(closeBtn);
    messageLayout->addStretch();

    //内容
    contentLayout = new QHBoxLayout();
    contentLab = new QLabel();
    contentLab->setStyleSheet(text_info + font_weight_500 + font_size_md);
    contentLab->setAlignment(Qt::AlignCenter);
    contentLab->setMaximumWidth(400);
    contentLab->setMinimumWidth(400);
    contentLab->setWordWrap(true);
    contentLayout->addStretch();
    contentLayout->addWidget(contentLab);
    contentLayout->addStretch();
    messageLayout->addLayout(contentLayout);
    messageLayout->addStretch();

    //按钮布局
    buttonLayout = new QHBoxLayout();
    messageLayout->addLayout(buttonLayout);

    QString btnStyle(border_info_2 + rounded_sm + text_info + font_weight_400 + pl_12 + pt_4 + pr_12 + pb_4);

    //确定按钮
    okBtn = new QPushButton();
    okBtn->setStyleSheet(btnStyle);
    okBtn->setCursor(Qt::PointingHandCursor);

    //取消按钮
    cancelBtn = new QPushButton();
    cancelBtn->setStyleSheet(btnStyle);
    cancelBtn->setCursor(Qt::PointingHandCursor);

    buttonLayout->addStretch();
    buttonLayout->addWidget(okBtn);
    buttonLayout->addWidget(cancelBtn);
    buttonLayout->addStretch();
    messageLayout->addStretch();

    //连接槽
    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCancelClicked()));
    QObject::connect(okBtn, SIGNAL(clicked()), this, SLOT(onOkClicked()));
    QObject::connect(cancelBtn, SIGNAL(clicked()), this, SLOT(onCancelClicked()));
}

void LMessageBox::paintEvent(QPaintEvent *event)
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

void LMessageBox::setTitleText(QString title)
{
    titleLab->setText(title);
}

void LMessageBox::setContentText(QString contentText)
{
    contentLab->setText(contentText);
}

void LMessageBox::setButtonType(MessageButtonType buttonType)
{
    switch (buttonType)
    {
    case BUTTON_OK:
        okBtn->setText(QStringLiteral("确定"));
        cancelBtn->setVisible(false);
        break;
    case BUTTON_CANCEL:
        okBtn->setVisible(false);
        cancelBtn->setText(QStringLiteral("取消"));
        break;
    case BUTTON_OK_AND_CANCEL:
        okBtn->setText(QStringLiteral("确定"));
        cancelBtn->setText(QStringLiteral("取消"));
        break;
    case BUTTON_CLOSE:
        okBtn->setVisible(false);
        cancelBtn->setVisible(false);
        break;
    }
}

int LMessageBox::showBox(QWidget* parent, const QString &title, const QString &contentText,  MessageButtonType messageButtonType, bool isModelWindow)
{
    LMessageBox * messageBox = new LMessageBox(parent);
    messageBox->setTitleText(title);
    messageBox->setContentText(contentText);
    messageBox->setButtonType(messageButtonType);

    if (isModelWindow)
    {
        return messageBox->exec();
    }
    else
    {
        messageBox->show();
    }

    return -1;
}

int LMessageBox::exec()
{
    this->setWindowModality(Qt::WindowModal);
    show();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_chooseResult;
}

void LMessageBox::onOkClicked()
{
    m_chooseResult = ID_OK;
    close();
}

void LMessageBox::onCancelClicked()
{
    m_chooseResult = ID_CANCEL;
    close();
}

void LMessageBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }
    event->accept();
}

