#ifndef IMAGE_TRX_HPP
#define IMAGE_TRX_HPP

#include <gob_transceiver.hpp>
#include <memory>

//Unreliable communication if defined.
//#define USING_UNRELIABLE

using TRX = goblib::esp_now::Transceiver;

//
struct Payload
{
    constexpr static size_t SPLIT_SIZE = 230;
    uint8_t type{};  // 0:start transfer 1:data block
    union
    {
        // type == 0 (Senderto Receiver)
        struct
        {
            size_t   size{};
            uint32_t crc32{};
        };
        // type == 1 (Sender to Receiver)
        struct
        {
            uint8_t bufSize;
            uint8_t buf[SPLIT_SIZE];
        };
    };
};

// Transceiver for data transfer
class ImageTRX : public TRX
{
  public:
    enum Status : uint8_t { None, Send, Recv };
    using finished_callback_t = void(*)(ImageTRX* trx, const Status s);

    ImageTRX(const uint8_t tid);
    virtual ~ImageTRX();

    bool inProgress() const { return _state != None; }

    std::unique_ptr<uint8_t[]>& uptr() { return _buf; }
    size_t size() const { return _size; }
    
    Status status() const { return _state; }
    bool finished() const { return _state == None && _size && _progress >= _size; }
    unsigned long startTime() const { return _startTime; }
    unsigned long timeRequired() const { return _endTime - _startTime; }
    size_t transferedSize() const { return _progress; }
    float transferedRate() const { return (float)_progress / _size; }
    size_t averageSpeed() const { return _speed; }
    uint16_t crc32() const { return _crc32; }

    bool send(const goblib::esp_now::MACAddress& addr, uint8_t* buf, const size_t sz);
    void abort();
    void purge();

    void setFinishedCallback(finished_callback_t f) { _callback = f; }

#if !defined(NDEBUG)
    String debugInfo() const;
#endif
    
  protected:
    virtual void update(const unsigned long ms) override;
    virtual void onReceive(const goblib::esp_now::MACAddress& addr, const void* data, const uint8_t length) override;
    
  private:
    bool _send(const goblib::esp_now::MACAddress& addr, uint8_t* buf, const size_t sz);
    void update_send();
    void update_recv();

    goblib::esp_now::MACAddress _addr;
    std::unique_ptr<uint8_t[]> _buf{};
    size_t _size{};

    size_t _progress{};
    size_t _length{};
    size_t _speed{};
    uint32_t _crc32{};
    uint64_t _sequence{};

    unsigned long _startTime{}, _endTime{};

    finished_callback_t _callback{};
    Status _state{None};
    bool _valid{};
};

uint32_t calculateCRC32(const char* path);

#endif
