#include "ZooidMessage.h"

#include <algorithm>

ZooidMessage::ZooidMessage(uint8_t senderId,
                           uint8_t type,
                           const uint8_t* payload,
                           std::size_t length,
                           uint64_t receivedAtMs,
                           uint64_t sequence)
    : type_(type),
      senderId_(senderId),
      receivedAtMs_(receivedAtMs),
      sequence_(sequence)
{
    if (payload != nullptr && length > 0)
        payload_.assign(payload, payload + length);
}

uint8_t ZooidMessage::getType() const
{
    return type_;
}

uint8_t ZooidMessage::getSenderId() const
{
    return senderId_;
}

const uint8_t* ZooidMessage::getPayload() const
{
    return payload_.empty() ? nullptr : payload_.data();
}

std::size_t ZooidMessage::getLength() const
{
    return payload_.size();
}

uint64_t ZooidMessage::getReceivedAtMs() const
{
    return receivedAtMs_;
}

uint64_t ZooidMessage::getSequence() const
{
    return sequence_;
}

bool decodeStatusMessage(const ZooidMessage& message,
                         DecodedStatusMessage& status)
{
    if (message.getLength() != 8 || message.getPayload() == nullptr)
        return false;

    const uint8_t* payload = message.getPayload();
    const auto decodeUint16 = [payload](std::size_t offset) {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(payload[offset]) |
            static_cast<uint16_t>(
                static_cast<uint16_t>(payload[offset + 1]) << 8));
    };

    DecodedStatusMessage decoded;
    decoded.positionX = decodeUint16(0);
    decoded.positionY = decodeUint16(2);
    const uint16_t rawOrientation = decodeUint16(4);
    const int32_t signedOrientation = rawOrientation <= 0x7fff
        ? static_cast<int32_t>(rawOrientation)
        : static_cast<int32_t>(rawOrientation) - 0x10000;
    decoded.orientation = static_cast<int16_t>(signedOrientation);
    decoded.state = payload[6];
    decoded.batteryLevel = payload[7];
    status = decoded;
    return true;
}

ZooidMessageQueue::ZooidMessageQueue(std::size_t capacity)
    : capacity_(capacity)
{
}

void ZooidMessageQueue::push(const ZooidMessage& message)
{
    if (capacity_ == 0) return;
    if (messages_.size() == capacity_) messages_.pop_front();
    messages_.push_back(message);
}

ZooidMessage ZooidMessageQueue::pop()
{
    if (messages_.empty()) return {};
    ZooidMessage message = messages_.front();
    messages_.pop_front();
    return message;
}

std::size_t ZooidMessageQueue::size() const
{
    return messages_.size();
}

bool ZooidMessageQueue::empty() const
{
    return messages_.empty();
}

void ZooidMessageQueue::clear()
{
    messages_.clear();
}

bool extractZooidFrame(std::vector<uint8_t>& buffer,
                       uint8_t& senderId,
                       uint8_t& type,
                       std::vector<uint8_t>& payload)
{
    const uint8_t startMarker = static_cast<uint8_t>('~');
    const uint8_t endMarker = static_cast<uint8_t>('!');
    while (true) {
        const auto start = std::find(buffer.begin(), buffer.end(), startMarker);
        if (start == buffer.end()) {
            buffer.clear();
            return false;
        }
        if (start != buffer.begin()) buffer.erase(buffer.begin(), start);
        if (buffer.size() < 4) return false;

        const std::size_t payloadLength = buffer[3];
        if (payloadLength > ZooidMaximumPayloadSize) {
            buffer.erase(buffer.begin());
            continue;
        }
        const std::size_t frameLength = payloadLength + 5;
        if (buffer.size() < frameLength) return false;
        if (buffer[frameLength - 1] != endMarker) {
            buffer.erase(buffer.begin());
            continue;
        }

        type = buffer[1];
        senderId = buffer[2];
        payload.assign(buffer.begin() + 4, buffer.begin() + 4 + payloadLength);
        buffer.erase(buffer.begin(), buffer.begin() + frameLength);
        return true;
    }
}
