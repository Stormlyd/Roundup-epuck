#ifndef ZOOIDMESSAGE_H
#define ZOOIDMESSAGE_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

constexpr std::size_t ZooidMaximumPayloadSize = 32;

class ZooidMessage
{
public:
    ZooidMessage() = default;
    ZooidMessage(uint8_t senderId,
                 uint8_t type,
                 const uint8_t* payload,
                 std::size_t length,
                 uint64_t receivedAtMs,
                 uint64_t sequence);

    uint8_t getType() const;
    uint8_t getSenderId() const;

    const uint8_t* getPayload() const;
    std::size_t getLength() const;
    uint64_t getReceivedAtMs() const;
    uint64_t getSequence() const;

private:
    uint8_t type_ = 0;
    uint8_t senderId_ = 0;
    std::vector<uint8_t> payload_;
    uint64_t receivedAtMs_ = 0;
    uint64_t sequence_ = 0;
};

struct DecodedStatusMessage
{
    uint16_t positionX = 0;
    uint16_t positionY = 0;
    int16_t orientation = 0;
    uint8_t state = 0;
    uint8_t batteryLevel = 0;
};

bool decodeStatusMessage(const ZooidMessage& message,
                         DecodedStatusMessage& status);

class ZooidMessageQueue
{
public:
    explicit ZooidMessageQueue(std::size_t capacity);

    void push(const ZooidMessage& message);
    ZooidMessage pop();
    std::size_t size() const;
    bool empty() const;
    void clear();

private:
    std::size_t capacity_;
    std::deque<ZooidMessage> messages_;
};

bool extractZooidFrame(std::vector<uint8_t>& buffer,
                       uint8_t& senderId,
                       uint8_t& type,
                       std::vector<uint8_t>& payload);

#endif // ZOOIDMESSAGE_H
