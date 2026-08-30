#include "ZooidFollow.h"

ZooidFollow::ZooidFollow(ZooidManager *_mamger):mamger(_mamger)
{
    time = 0;
    baton = Vector2(1, 0.5);
    stateCode = Free;
    followTimer = new QTimer();
    followTimer->setInterval(100);   //10Hz
    QObject::connect(followTimer, SIGNAL(timeout()), this, SLOT(followTimerRun()));
}

void ZooidFollow::begin()
{
    reset();
    followTimer->start();
}

void ZooidFollow::end()
{
    followTimer->stop();
    stateCode = Free;
    reset();
}

void ZooidFollow::reset()
{
    timeCount = 0;
    performTime = 0;
    //voronoiController->reset();
}

int ZooidFollow::getState()
{
    return stateCode;
}

void ZooidFollow::setBatonPosition(Vector2 position)
{
     baton = position;
}

Vector2 ZooidFollow::getBatonPosition()
{
    return baton;
}


void ZooidFollow::setTime(unsigned int _time)
{
    time = _time;
}

void ZooidFollow::setZooids(vector<unsigned int>ids)
{
    zooids = ids;
}

vector<unsigned int> ZooidFollow::getZooids()
{
    return zooids;
}

//回调函数,每0.1s进入一次
void ZooidFollow::followTimerRun()
{
    timeCount++;
    performTime += 0.1;
    if(timeCount / 10 > time)   //time为表演时长
    {
        end();
        return ;
    }

    stateCode = Running;
    mamger->updateFollow();
}
