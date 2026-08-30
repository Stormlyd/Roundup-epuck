#include "LShowViewBox.h"
#include <QGridLayout>
#include <QMovie>

#define TITLE_LABEL_QSS "font-size:22px;font-weight: 600; color:#68d8fe; margin:10px 0px 0px 10px;"
#define TITLE_LABEL_QSS "font-size:22px;font-weight: 600; color:#68d8fe; margin:10px 0px 0px 10px;"
#define CLOSE_BUTTON_QSS "background:transparent;margin-right:20px;margin-top:10px;"


LShowViewBox::LShowViewBox(QDialog *parent) : QDialog(parent), movie (nullptr), widget(nullptr)
{
	 
	init();
}


LShowViewBox::~LShowViewBox()
{
    if (movie != nullptr)
    {
		delete movie;
	}

    if (widget != nullptr)
    {
		delete  widget;
	}
}



void LShowViewBox::init()
{
	setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
	//setAttribute(Qt::WA_TranslucentBackground, true);
	resize(500, 380);//显示大小	
	 
	movieLabel = new QLabel(this);
	movieLabel->setText(QStringLiteral("movieLabel"));
	movieLabel->setGeometry(QRect(15, 35, 461, 331)); 
	//movieLabel->setGeometry(QRect(10, 50, 481, 341));

	widget = new  QWidget(this);
	widget->setGeometry(QRect(10, 8, 481, 20));
	QHBoxLayout *horizontalLayout = new QHBoxLayout(widget); 
	horizontalLayout->setContentsMargins(0, 0, 0, 0);
	titleLab = new QLabel(widget);
	titleLab->setStyleSheet(TITLE_LABEL_QSS);
	titleLab->setText("演示");
	
	QPushButton *closeBtn = new QPushButton(widget); 
	QIcon closeBtnIco(":/new/ofapp/res/images/close.png");
	closeBtn->setIcon(closeBtnIco);
	closeBtn->setCursor(Qt::PointingHandCursor);
	closeBtn->setStyleSheet(CLOSE_BUTTON_QSS);


	horizontalLayout->addWidget(titleLab);
	horizontalLayout->addStretch();
	horizontalLayout->addWidget(closeBtn);

	QObject::connect(closeBtn, SIGNAL(clicked()), this, SLOT(onCloseClicked()));
}

//--slot
void LShowViewBox::onCloseClicked()
{
	close();
}

void LShowViewBox::setMoviePath(QString path)
{
	movie = new QMovie(path);
	movieLabel->setMovie(movie);
    movieLabel->setScaledContents(true);
}

//设置标题
void LShowViewBox::setShowViewTitle(QString title)
{
	titleLab->setText(title);
}

void LShowViewBox::movieStart()
{
	movie->start();
}
 
