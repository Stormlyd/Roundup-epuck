#include "ZooidCharge.h"

ZooidCharge::ZooidCharge(ZooidManager *_manager):manager(_manager)
{
    loadParameters();
}

void ZooidCharge::loadParameters()
{

    QFile loadFile(CHARGE_PATH);
    if(!loadFile.open(QIODevice::ReadOnly))
    {
       qDebug() << "Error: Charge json read failed";
       return;
    }

    QByteArray allData = loadFile.readAll();
    loadFile.close();

    QJsonParseError json_error;
    QJsonDocument jsonDoc(QJsonDocument::fromJson(allData, &json_error));

    if(json_error.error != QJsonParseError::NoError)
    {
        qDebug() << "Error: Charge json error!";
        return;
    }

    QJsonObject rootObj = jsonDoc.object();

    //解析位置信息
    if(rootObj.contains("position") && rootObj["position"].isArray())
    {
        QJsonArray subArray = rootObj.value("position").toArray();
        for(int i = 0; i< subArray.size(); i++)
        {
            ChargeMessage msg;
            msg.has = false;

            if(!subArray[i].isObject())
            {
                continue;
            }
            QJsonObject posObj = subArray[i].toObject();


            //解析第一位置
            if(posObj.contains("first") && posObj["first"].isArray())
            {
                QJsonArray posArray = posObj.value("first").toArray();
                if(posArray.size() == 2 && posArray[0].isDouble() && posArray[1].isDouble())
                {
                    msg.first = Vector2(static_cast<float>(posArray[0].toDouble()),static_cast<float>(posArray[1].toDouble()));
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            //解析第二位置
            if(posObj.contains("second") && posObj["second"].isArray())
            {
                QJsonArray posArray = posObj.value("second").toArray();
                if(posArray.size() == 2 && posArray[0].isDouble() && posArray[1].isDouble())
                {
                    msg.second = Vector2(static_cast<float>(posArray[0].toDouble()),static_cast<float>(posArray[1].toDouble()));
                }
                else
                {
                    continue;
                }
            }
            else
            {
                continue;
            }

            // 充电桩Id从0递增
            msg.id = static_cast<int>(chargeMsg.size());
            chargeMsg.push_back(msg);

           // qDebug()<<"charge id:"<<msg.id<<"first pos:"<<msg.first.getX()<<msg.first.getY()<<"second pos:"<<msg.second.getX()<<msg.second.getY();
        }

        qDebug() << "charge count is"<<subArray.size();
    }

    qDebug() << "Charge json file read success!";

    //解析其他信息TODO...
}

vector<Vector2> ZooidCharge::getFreeChargeFirstPosition()
{
    //获取空闲的充电位置， 返回的是第一位置
    vector<Vector2> chargePosition;
    for(unsigned int i=0; i<chargeMsg.size(); i++)
    {
        // 没有机器人则返回第一位置
        if(chargeMsg[i].has == false)
        {
            chargePosition.push_back(chargeMsg[i].first);
        }
    }

    // 第一位置vector
    return chargePosition;
}

int ZooidCharge::getNearFirstChargeId(Vector2 position)
{
    // 返回离第一位置最近的充电住桩ID
    for(unsigned int i = 0; i<chargeMsg.size(); i++){
        //在第一位置上
        if (lzm::absSq(position - chargeMsg[i].first) < GoalRadius * GoalRadius)
        {
              return static_cast<int>(chargeMsg[i].id);
        }
    }

    return -1;
}

int ZooidCharge::getNearSecondChargeId(Vector2 position)
{
    for(unsigned int i = 0; i<chargeMsg.size(); i++){
        //在充电桩上 (第二位置)
        if (lzm::absSq(position - chargeMsg[i].second) <= GoalRadius * GoalRadius)
        {
             return static_cast<int>(chargeMsg[i].id);
        }
    }
    return -1;
}

void ZooidCharge::releaseCharge(Vector2 position)
{
    for(unsigned int i = 0; i<chargeMsg.size(); i++){
        //在充电桩上 (第二位置)
        if (lzm::absSq(position - chargeMsg[i].second) <= GoalRadius * GoalRadius)
        {
              chargeMsg[i].has = false;
              return ;
        }
    }
}

void ZooidCharge::releaseCharge(unsigned int id)
{
    chargeMsg[id].has = false;
}

void ZooidCharge::setChargeZooid(int id)
{
    chargeMsg[static_cast<unsigned int>(id)].has = true;
}

void ZooidCharge::setChargeZooid(Vector2 position)
{
    for(unsigned int i = 0; i<chargeMsg.size(); i++){
        //在充电桩上 (第一位置)
        if (lzm::absSq(position - chargeMsg[i].first) <= GoalRadius * GoalRadius)
        {
              chargeMsg[i].has = false;
              return ;
        }
    }
}

Vector2 ZooidCharge::getSecondPos(int id)
{
    return chargeMsg[static_cast<unsigned int>(id)].second;
}

Vector2 ZooidCharge::getFirstPos(int id)
{
    return chargeMsg[static_cast<unsigned int>(id)].first;
}
