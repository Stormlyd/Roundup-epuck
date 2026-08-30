#include "ZooidDraw.h"
#include <QPushButton>
#include <QStylePainter>
#include <QStyleOption>
#include <QDebug>

/**
 * ZooidDraw路径绘制器
 * @brief ZooidDraw::ZooidDraw
 */
ZooidDraw::ZooidDraw(QWidget *parent): QWidget(parent)
{
    //初始化
    setAutoFillBackground (true);
    setBackBackground(QColor("#101129"));

    // 636 308
    //setSize(636,308);
    setSize(880,426);
    setPen(QPen(QColor("#68d8fe"), ROBOT_DIAMETER * 300 * 1.38, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    setPixmap(QPixmap(width(), height()), QColor("#101129"));
    setWindowFlags(windowFlags()&~Qt::WindowMaximizeButtonHint);
}

ZooidDraw::~ZooidDraw()
{

}

void ZooidDraw::paintEvent(QPaintEvent *event)
{
    //加载当前pathPix
    QPainter tempPainter(&pathPixmap);
    //打开反锯齿
    tempPainter.setRenderHint(QPainter::Antialiasing, true);
    //使用画笔
    tempPainter.setPen(pathPen);
    //画线
    tempPainter.drawLine(startPoint,endPoint);
    //更新位置
    startPoint = endPoint;
    //双缓冲
    QPainter painter(this);
    painter.drawPixmap (QPoint(0,0), pathPixmap);

    QStylePainter painter1(this);
    //用style画背景 (会使用setstylesheet中的内容)
    QStyleOption opt;
    opt.initFrom(this);
    opt.rect=rect();
    painter1.drawPrimitive(QStyle::PE_Widget, opt);
    QWidget::paintEvent(event);

}

void ZooidDraw::mousePressEvent(QMouseEvent *event)
{
    //鼠标左键按下
    if(event->button() == Qt::LeftButton)
        startPoint = event->pos();
    endPoint = startPoint;
}

void ZooidDraw::mouseMoveEvent(QMouseEvent *event)
{
    // 鼠标左键按下的同时移动鼠标
    if(event->buttons() & Qt::LeftButton)
    {
        endPoint = event->pos();
        update();
    }
}

void ZooidDraw::mouseReleaseEvent(QMouseEvent *event)
{
    //鼠标左键释放
    if(event->button() == Qt::LeftButton)
    {
        endPoint = event->pos();
    }
}

// --------------------------------------------------------------------------
void ZooidDraw::setSize(int width, int height){
    this->setMinimumSize(width,height);
    this->resize(width,height);
}


// --------------------------------------------------------------------------
void ZooidDraw::setBackBackground(QColor color){
    setPalette (QPalette(color));
}

// --------------------------------------------------------------------------
void ZooidDraw::setPen(QPen pen){
    pathPen = pen;
}

// --------------------------------------------------------------------------
void ZooidDraw::setPixmap(QPixmap  pixmap){
    pathPixmap = pixmap;
}

// --------------------------------------------------------------------------
void ZooidDraw::setPixmap(QPixmap pixmap, QColor fillColor ){
    pathPixmap = pixmap;
    pathPixmap.fill(fillColor);
}

QPixmap ZooidDraw::getPixmap(){
    QPixmap pixmap = pathPixmap;
    int width = pixmap.width();
    int height = pixmap.height();
    QImage tempImage = pixmap.toImage();

    // 设置模拟器中路径的颜色
    for(int i=0; i<height; i++)
    {
        for(int j=0; j< width; j++)
        {
            QColor color = tempImage.pixelColor(QPoint(j,i));
            if(color.red() < 0xe0 && color.green() < 0xe0 && color.blue() > 150)
            {
                // 设置颜色
                tempImage.setPixelColor(j, i, QColor(33,66,99));
            }
        }
    }
    pixmap = QPixmap::fromImage(tempImage);


    QSize picSize(1077,541);
    QPixmap scaledPixmap = pixmap.scaled(picSize, Qt::KeepAspectRatio);
    return scaledPixmap;
}


// --------------------------------------------------------------------------
void ZooidDraw::clearPath()
{
    QPixmap clearPix = QPixmap(size());
    clearPix.fill (QColor("#101129"));
    pathPixmap = clearPix;
    update();
    clearPositionData();
 }

// --------------------------------------------------------------------------
void ZooidDraw::clearPositionData(){
    //清空数据
    for(int i=0; i<100; i++)
    {
        for(int j=0; j<100; j++)
        {
            posMatrix[i][j].position = Vector2(0.0f, 0.0f);
            posMatrix[i][j].has = false;
            posMatrix[i][j].value = 0;
        }
    }

    pathPoints.clear();
}

void ZooidDraw::generatePath(){

    clearPositionData();
    QImage tempImage = pathPixmap.toImage();
    int width = pathPixmap.width();//636
    int height = pathPixmap.height();//308

    int k = (ROBOT_DIAMETER * 300);//分辨率系数

    int x,y;
    x = y = 0;
    for(int i=0; i<height; i++)
    {
        //降低分辨率
        if(i%k)continue;
        y = 0;
        for(int j=0; j< width; j++)
        {
            if(j%k)continue;
            //滤波
            QColor color = tempImage.pixelColor(QPoint(j,i));

            if(color.red() < 0x90 && color.green() < 0xe0 && color.blue() > 200)
            {
                float px = j/600.0f / 1.381 + 0.218f;//位置补偿x
                float py = i/600.0f / 1.381 + 0.2f;  //位置补偿y
                posMatrix[x][y].position = Vector2(px , py);
                posMatrix[x][y].has = true;
                posMatrix[x][y].value = 1;
                tempImage.setPixelColor(j, i, Qt::blue);
            }
            y++;
        }
        x++;
    }
    this->pathPixmap = QPixmap::fromImage(tempImage);
    update();

    //计算权重
    solve(height/k + 1, width/k + 1);
}

vector<Vector2> ZooidDraw::getPathPoints()
{
    return this->pathPoints;
}

// --------------------------------------------------------------------------
void ZooidDraw::solve(int row, int col){

    //计算每一个有效点的权重
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            if(posMatrix[i][j].has){
                for(int k=0; k<8; k++){
                    computvalue(i, j, i, j, row, col, 0, 3, k);
                }
            }
        }
    }

    PositionData *tempData = new PositionData [row *col];
    int cot = 0;
    for(int i=0; i<row; i++)
    {
        for(int j=0; j<col; j++)
        {
            if(posMatrix[i][j].has)
            {
                tempData[cot++] = posMatrix[i][j];
            }
        }
    }

    sort(tempData, tempData + cot, cmpPositionDataLess());

    for(int i=0; i<cot; i++)
    {
        pathPoints.push_back(Vector2(tempData[i].position.getX(), tempData[i].position.getY()));
    }
    delete [] tempData;
}

// --------------------------------------------------------------------------
// 计算以point为中心, 相邻深度为depth 的权重
// --------------------------------------------------------------------------
void ZooidDraw::computvalue(int s, int e, int x, int y, int row, int col, int dep, int depth, int dir){

    //剪枝
    if(x < 0 || y < 0 || y >= col || x >= row){
        return ;
    }

    if(dep >= depth){
        return;
    }

    //累加权重
    if(!(s == x && e == y) && posMatrix[x][y].has){
        posMatrix[s][e].value ++;
    }

    const int DIR[8][2] = {1,0, 0,1, -1,0, 0,-1, 1,1, 1,-1, -1,1, -1,-1};
    computvalue(s, e, x + DIR[dir][0], y + DIR[dir][1], row, col, ++dep, depth, dir);

    return ;
}

QPoint ZooidDraw::getStartPoint()
{
       return this->startPoint;
}
QPoint ZooidDraw::getEndPoint()
{
       return this->endPoint;
}
