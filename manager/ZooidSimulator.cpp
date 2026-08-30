#include "ZooidSimulator.h"

ZooidSimulator::ZooidSimulator()
{
    simulatorScene = new QGraphicsScene();
    width = 640;
    height = 480;
    zoom = 0.65;
    zoomOld = 1;

    // 新建背景子项 luzhimin-20201103
    bgPath = new QGraphicsPixmapItem;
    simulatorScene->addItem(bgPath);
    bgPath->setZValue(0);

    graphicalImg = new QGraphicsPixmapItem;
    simulatorScene->addItem(graphicalImg);
    graphicalImg->setZValue(0);

    followImg = new QGraphicsPixmapItem;
    simulatorScene->addItem(followImg);
    followImg->setZValue(99999);

    QPixmap pixmap(":/new/ofapp/res/images/flag.png");
    QSize picSize(150,150);
    QPixmap scaledPixmap = pixmap.scaled(picSize, Qt::KeepAspectRatio);
    followImg->setPixmap(scaledPixmap);
    followImg->hide();


    setRenderHint(QPainter::Antialiasing, true);
    setScene(simulatorScene);
    setFrameStyle(QFrame::NoFrame);

    setDragScroll();
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setBackBackground(QColor("#101129"));

    updateSceneTimer = new QTimer();
    connect(updateSceneTimer, SIGNAL(timeout()), this, SLOT(updateSimulator()));
    updateSceneTimer->start((int)SYSTEM_UPDATE_PERIOD);
}

ZooidSimulator::~ZooidSimulator()
{
    if (updateSceneTimer != nullptr)
    {
        delete updateSceneTimer;
    }

    if (simulatorScene != nullptr)
    {
        delete simulatorScene;
    }
}

void ZooidSimulator::setOpenGlView()
{
    this->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
}

void ZooidSimulator::setDragScroll()
{
    this->setDragMode(QGraphicsView::ScrollHandDrag);
}

void ZooidSimulator::setSize(int _width, int _height )
{
    if(_width <= 0 || _height <= 0)
    {
        return ;
    }

    width = _width;
    height = _height;
    resize(QSize(width , height));
    simulatorScene->setSceneRect(0, 0,width ,height);
}

void ZooidSimulator::setBackBackground(QColor color)
{
    setBackgroundBrush(QColor("#101129"));
}

void ZooidSimulator::setBackgroundImage(QPixmap *pix){
    clearBackgroundImage();
    bgPath->setPixmap(*pix);//将图添加进item
    bgPath->setPos(191,183);//设置item坐标为（0,0）
    bgPath->show();
}

void ZooidSimulator::setGraphicalImage(QPixmap *pix){
    clearBackgroundImage();
    graphicalImg->setPixmap(*pix);//将图添加进item
    graphicalImg->setPos(1280,40);//设置item坐标为（0,0）
    graphicalImg->show();
}

void ZooidSimulator::setFollowImagePos(int x, int y)
{
    clearBackgroundImage();
    followImg->setPos(x-67,y-145);//设置item坐标为（0,0）
    followImg->show();
}

void ZooidSimulator::clearBackgroundImage(){
    bgPath->hide();
    graphicalImg->hide();
    followImg->hide();
}


void ZooidSimulator::updateSimulator()
{
    if(zoom != zoomOld)
    {
        QMatrix matrix;
        matrix.scale(this->zoom, this->zoom);
        this->setMatrix(matrix);
    }
    this->viewport()->update();
}

void ZooidSimulator::addZooid(Zooid *zooid)
{
    zooid->setPos(zooid->getPosition().getX() * 1000 - 35, (0.914f - zooid->getPosition().getY()) * 1000 - 35);
    simulatorScene->addItem(zooid);
}

void ZooidSimulator::addZooidGoal(ZooidGoal *zooidGoal)
{
    zooidGoal->setPos(zooidGoal->getPosition().getX() *1000 - 35, (0.914f - zooidGoal->getPosition().getY())*1000 - 35);
    simulatorScene->addItem(zooidGoal);
}

void ZooidSimulator::removeZooid(Zooid *zooid)
{
    simulatorScene->removeItem(zooid);
	if(zooid != nullptr)
    {
        delete zooid;
    }
}

void ZooidSimulator::removeZooidGoal(ZooidGoal *zooidGoal)
{
    simulatorScene->removeItem(zooidGoal);
	if (zooidGoal != nullptr)
    {
        delete  zooidGoal;
    }
}

int ZooidSimulator::getWidth()
{
    return width;
}

int ZooidSimulator::getheight()
{
   return height;
}

void ZooidSimulator::clearAll()
{
    simulatorScene->clear();
}

void ZooidSimulator::zoomIn()
{
    zoom += 0.15;

    if(zoom > 2.75)
    {
        zoom = 2.75;
    }
}

void ZooidSimulator::zoomOut()
{
    zoom -= 0.15;

    if(zoom < 0.50)
    {
        zoom = 0.50;
    }
}

void ZooidSimulator::zoomlevel(qreal level)
{
    if(level >= 0.50 || level <= 2.75)
    {
        zoom = level;
    }
}

double ZooidSimulator::getZoom()
{
    return zoom;
}

void ZooidSimulator::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons()&Qt::LeftButton)
    {
        QMutexLocker locker(&m_mutex);
        locker.unlock();
        QString str;

        str = QString("%1 , %2").arg(event->pos().x()).arg(event->pos().y());
        emit sendClickPosition((int)event->pos().x(), (int)event->pos().y());
    }
}
