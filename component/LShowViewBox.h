#pragma once

#ifndef LShowViewBox_H
#define LShowViewBox_H

#include <QEventLoop> 
#include <QPushbutton>
#include <QLabel>
#include <QPainter>
#include <QDialog>
#include <QWidget>

class LShowViewBox : public QDialog
{
	Q_OBJECT
public:
	LShowViewBox(QDialog *parent = 0);
	~LShowViewBox();

	//初始化界面
	void init();
	void setMoviePath(QString path);
	void movieStart();

	//设置标题
	void setShowViewTitle(QString title);

	//void paintEvent(QPaintEvent *event);
private slots:
    void onCloseClicked();

private:
	QMovie  *movie;
	QLabel  *movieLabel,*titleLab;
	QWidget *widget;
};

#endif  //LShowViewBox_H
