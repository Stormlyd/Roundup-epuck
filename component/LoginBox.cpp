#include "LoginBox.h"

/**
 * @brief   LoginBox::LoginBox
 * @param   parent
 * @author  ZengXiang
 */
LoginBox::LoginBox(QWidget *parent): QWidget(parent)
{
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setWindowModality(Qt::WindowModal);
    resize(480,260);

    //标题
    titleLab = new QLabel(QStringLiteral("后台登录"));
    titleLab->setParent(this);
    titleLab->setStyleSheet(font_size_lg + font_weight_400 + text_info);
    titleLab->move(18,15);

    //关闭按钮
    closeBtn = new QPushButton(this);
    closeBtn->setIcon(QIcon(":/new/ofapp/res/images/close.png"));
    closeBtn->setCursor(Qt::PointingHandCursor);
    closeBtn->setStyleSheet(transparent_bg);
    closeBtn->move(width()-50,15);

    passwordLab = new QLabel(this);
    passwordLab->setText(QStringLiteral("请输入密码"));
    passwordLab->setStyleSheet(text_info + font_size_sm);
    passwordLab->move(60, 65);

    passEdit =new QLineEdit(this);
    passEdit->setFixedSize(360, 40);
    passEdit->setEchoMode(QLineEdit::Password);
    passEdit->setStyleSheet(text_info + border_info_1 + rounded_sm + font_size_md +bg_info1 + pl_12 + pr_12);
    passEdit->move(60,100);

    messageLab = new QLabel(QStringLiteral(""));
    messageLab->setParent(this);
    messageLab->setAlignment(Qt::AlignCenter);
    messageLab->setFixedSize(480, 40);
    messageLab->setStyleSheet(font_size_sm + font_weight_400 + text_danger);
    messageLab->move(0,140);

    //确定按钮
    okayBtn = new QPushButton(QStringLiteral("确定"));
    okayBtn->setParent(this);
    okayBtn->setCursor(Qt::PointingHandCursor);
    okayBtn->setStyleSheet(border_info_2 + text_info + rounded_lg + font_size_lg + font_weight_200);
    okayBtn->setFixedSize(128,48);
    okayBtn->move(176 ,height() - 78 );

    QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
    QObject::connect(okayBtn, SIGNAL(clicked()), this, SLOT(onOkayClicked()));
}

LoginBox::~LoginBox()
{
    delete passEdit;
    passEdit = nullptr;

    delete m_eventLoop;
    m_eventLoop = nullptr;

    delete titleLab;
    titleLab = nullptr;

    delete messageLab;
    messageLab = nullptr;

    delete closeBtn;
    closeBtn = nullptr;

    delete passwordLab;
    passwordLab = nullptr;

    delete okayBtn;
    okayBtn = nullptr;
}

void LoginBox::paintEvent(QPaintEvent *)
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

int LoginBox::showBox()
{
    show();
    passEdit->setFocus();
    m_eventLoop = new QEventLoop(this);
    m_eventLoop->exec();
    return m_buttonResult;
}

void LoginBox::closeEvent(QCloseEvent *event)
{
    if (m_eventLoop != nullptr)
    {
        m_eventLoop->exit();
    }
    event->accept();
}

void LoginBox::onCloseClicked()
{
    m_buttonResult = Close;
    close();
}

void LoginBox::onOkayClicked()
{
    QString  passWord = passEdit->text();
    if(passEdit->text().isEmpty())
    {
        messageLab->setText( QStringLiteral("密码不能为空"));
        return ;
    }

    if(onLogin(passWord))
    {
        m_buttonResult = Ok;
        messageLab->setText( QStringLiteral(""));
        close();
    }
    else
    {
        messageLab->setText( QStringLiteral("密码错误！"));
    }

}

bool LoginBox::onLogin(QString pass)
{
    QSqlQuery query;
    QString sql = QString("SELECT `password` FROM `config` WHERE id=1000");
    query.exec(sql);
    query.first();
    QString password = query.value("password").toString();
    if (pass == password)
    {
        //登录成功
        return true;
    }
    else
    {
        //密码不正确
        return false;
    }
}
