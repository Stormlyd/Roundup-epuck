#include "ZooidMessage.h"

ZooidMessage::ZooidMessage()
{
    senderId = 0;
    type = 0;
    payload = nullptr;
}

ZooidMessage::~ZooidMessage()
{

}

ZooidMessage::ZooidMessage(uint8_t _senderId, uint8_t _type, uint8_t* _payload)
{
    senderId = _senderId;
    type = _type;
    payload = _payload;
}

void ZooidMessage::setType(uint8_t _type)
{
    type = _type;
}

uint8_t ZooidMessage::getType()
{
    return type;
}

void ZooidMessage::setSenderId(uint8_t _senderId)
{
    senderId = _senderId;
}

uint8_t ZooidMessage::getSenderId()
{
    return senderId;
}

void ZooidMessage::setPayload(uint8_t* _payload)
{
    payload = _payload;
}

uint8_t* ZooidMessage::getPayload()
{
    return payload;
}

uint8_t* ZooidMessage::ToByteArray()
{
    uint8_t* array = new uint8_t[sizeof(payload)+2];
    //第一个字节 类型
    array[0] = type;
    //发送者id
    array[1] = senderId;
    //返回一帧数据
    for(int i=0;i<sizeof(payload);i++)
    {
        array[i+2]=payload[i];
    }

    return array;
}

string ZooidMessage::IntoString()
{
    if(payload != nullptr)
    {
        string s = string();

        if(senderId!='\n')
            s += (char)senderId;
        else
            s += "\\n";

        if(type!='\n')
            s += (char)type;
        else
            s += "\\n";

        for (int i = 0; i < sizeof(payload); i++)
        {
            if (payload[i] == '\n')
                s += "\\n";
            else
                s += (char)payload[i];
        }

        return s;
    }
    else
        return nullptr;
}

string ZooidMessage::ToHexString()
{
    std::stringstream stream;
    stream << "0x" << std::setfill ('0') << std::setw(2) << std::hex << senderId;
    stream << " 0x" << std::setfill ('0') << std::setw(2) << std::hex << type;

    for (int i = 0; i < sizeof(payload); i++)
    {
        if (payload[i] == '\n')
            stream << endl;
        else
            stream << " 0x" << std::setfill ('0') << std::setw(2) << std::hex << payload[i];
    }
    return stream.str();
}

