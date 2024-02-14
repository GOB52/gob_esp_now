#ifndef TRANSFER_TRX_HPP
#define TRANSFER_TRX_HPP

#include <SdFat.h>
#include <WString.h>
#include <gob_transceiver.hpp>

using TRX = goblib::esp_now::Transceiver;
using file_size_t = unsigned long; // type of File.size()

struct Payload
{
    constexpr static uint8_t SPLIT_SIZE = 230;
    uint8_t type{};  // 0:start transfer 1:data block  2:resend request
    union
    {
        // type == 0 (Senderto Receiver)
        struct
        {
            char name[128+1]{};
            file_size_t fileSize{};
            uint32_t crc32{};
        };
        // type == 1 (Sender to Receiver)
        struct
        {
            uint8_t bufSize;
            uint8_t buf[SPLIT_SIZE];
        };
        // type == 2 (Receiver to Sender)
        struct
        {
            file_size_t position; // File position to be sent again.
        };
    };
};

//
using File = FsFile;

// Transceiver for file transfer
class TransferTRX : public TRX
{
  public:
    enum Status : uint8_t { None, Send, Recv };

    TransferTRX(const uint8_t tid);
    virtual ~TransferTRX();

    bool inProgress() const { return _state != None; }

    Status status() const { return _state; }
    bool finished() const { return _state == None && _size && _progress >= _size; }
    unsigned long startTime() const { return _startTime; }
    unsigned long timeRequired() const { return _endTime - _startTime; }
    const char* fname() const { return _fname.c_str(); }
    const char* path() const { return _path.c_str(); }
    file_size_t fileSize() const { return _size; }
    file_size_t transferedSize() const { return _progress; }
    float transferedRate() const { return (float)_progress / _size; }
    file_size_t averageSpeed() const { return _speed; }
    uint32_t crc32() const { return _crc32; }

    // For receiver 
    void setDir(const char* d) { _dir = d; } // Download dir
    
    bool send(const goblib::esp_now::MACAddress& addr, const String& path);
    inline bool send(const goblib::esp_now::MACAddress& addr, const char* path) { return send(addr, String(path)); }
    void abort();

    // bus lock
    struct lock_guard_bs
    {
        explicit lock_guard_bs (SemaphoreHandle_t& s) : _sem(&s) { xSemaphoreTake(*_sem, portMAX_DELAY); }
        ~lock_guard_bs()                                         { xSemaphoreGive(*_sem); }
        lock_guard_bs() = delete;
        lock_guard_bs(const lock_guard_bs&) = delete;
        lock_guard_bs(lock_guard_bs&&) = delete;
        lock_guard_bs& operator=(const lock_guard_bs&) = delete;
        lock_guard_bs& operator=(lock_guard_bs&&) = delete;
      private:
        SemaphoreHandle_t* _sem{};
    };
    template<typename Func, typename... Args> auto bus_lock(Func func, Args&&... args)
            -> decltype(func(std::forward<Args>(args)...))
    {
        lock_guard_bs lock(_bus);
        return func(std::forward<Args>(args)...);
    }
    
  protected:
    virtual void update(const unsigned long ms) override;
    virtual void onReceive(const goblib::esp_now::MACAddress& addr, const void* data, const uint8_t length) override;
    
  private:
    void update_send();
    void update_recv();

    goblib::esp_now::MACAddress _addr;
    
    File _file;
    String _fname;
    file_size_t _size{};
    file_size_t _progress{};
    file_size_t _length{};
    file_size_t _speed{};
    uint32_t _crc32{};
    uint64_t _sequence{};

    unsigned long _startTime{}, _endTime{};

    Status _state{None};
    bool _retrunAck{};

    String _dir{"/"}; // for recv
    String _path;
    
    mutable SemaphoreHandle_t _bus{}; // For exclusive control of card access and lcd
};

uint32_t calculateCRC32(const char* path);

#endif
