
#include <SdFat.h>
#include <M5Unified.h>
#include "transferTRX.hpp"
#include <CRC32.h>

extern SdFs sd; // file_transfer_main.cpp

using goblib::esp_now::MACAddress;

uint32_t calculateCRC32(const char* path)
{
    CRC32 crc;
    File f = sd.open(path);
    if(!f) { M5_LOGE("Failed to open [%s]\n", path); return 0; }

    auto hsz = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    uint8_t* buf = new uint8_t[hsz];
    if(!buf || !hsz) { return 0; }

    while(f.available())
    {
        auto sz = std::min((int)hsz, f.available());
        if(f.read(buf, sz) != sz)
        {
            delete[] buf;
            return 0;
        }
        crc.add(buf, sz);
    }
    f.close();

    delete[] buf;

    return crc.calc();
}

//
TransferTRX::TransferTRX(const uint8_t tid) : TRX(tid)
{
    _bus = xSemaphoreCreateBinary();
    assert(_bus);
    xSemaphoreGive(_bus);
}

TransferTRX::~TransferTRX()
{
    bus_lock([this]()
    {
        this->_file.close();
    });
    vSemaphoreDelete(_bus);
}

// Start transfer (Sender)
bool TransferTRX::send(const MACAddress& addr, const String& path)
{
    if(_state != Status::None) { return false; }

    bus_lock([this, &path]()
    {
        this->_crc32 = calculateCRC32(path.c_str());
        this->_file = sd.open(path);
        this->_size = (bool)(this->_file) ? this->_file.available() : 0;
    });
    if(!_file) { M5_LOGE("Failed to open [%s]", path); return false; }

    _addr = addr;

    _path = path;
    _fname = path.substring(path.lastIndexOf('/') + 1);
    _progress = _length = 0;
    _state = Status::Send;
    _sequence = 0;

    M5_LOGI("Send file info sz:%lu CRC:%x", _size, _crc32);
    Payload pl;
    pl.type = 0;
    snprintf(pl.name, sizeof(pl.name), "%s", _fname.c_str());
    pl.fileSize = _size;
    pl.crc32 = _crc32;

    _startTime = millis();
    _sequence = postReliable(_addr, pl);
    if(!_sequence)
    {
        M5_LOGE("Failed to post");
        return false;
    }
    return true;
}

void TransferTRX::abort()
{
    _state = Status::None;
    bus_lock([this]()
    {
        this->_file.close();
    });
    _size = 0;
    _fname = "";
}
        
void TransferTRX::update(const unsigned long ms)
{
    switch(_state)
    {
    case Status::Send: update_send(); break;
    case Status::Recv: update_recv(); break;
    default: break;
    }
}

void TransferTRX::update_recv()
{
}

void TransferTRX::update_send()
{
    if(!_sequence) { return; }
    if(!delivered(_sequence, _addr)) { return; }

    M5_LOGV("deliverd %llu", _sequence);
    
    _progress += _length;
    auto now = millis();
    _speed = ((float)_progress / (now - _startTime)) * 1000.f;

    bool available = bus_lock([this, &now]()
    {
        if(!this->_file.available())
        {
            this->_file.close();
            this->_state = Status::None;
            this->_endTime = now;
            this->_speed = ((float)_size / timeRequired()) * 1000.f;
            M5_LOGI("Finished %lu (%f%%) <%lu>", timeRequired(), transferedRate() * 100, _speed);
            return false;
        }
        return true;
    });
    if(!available) { return; }
    
    Payload pl;
    auto read = std::min((int)Payload::SPLIT_SIZE, _file.available());
    auto len = bus_lock([this, &pl, &read]()
    {
        return _file.read(pl.buf, read);
    });
    if(len != read)
    {
        M5_LOGE("Failed to read");
        bus_lock([this, &len]
        {
            this->_file.seek(_file.position() - len); // rewind position
        });
        return;
    }

    pl.type = 1;
    pl.bufSize = len;
    auto prev = _sequence;
    _sequence = postReliable(_addr, pl);

    M5_LOGV("posted data: %llu", _sequence);

    if(!_sequence) { _sequence = prev; }
    _length = len;
}
    
void TransferTRX::onReceive(const MACAddress& addr, const void* data, const uint8_t length)
{
    const Payload* pl = (Payload*)data;

    //M5_LOGI("Recv:%d", pl->type);

    switch(pl->type)
    {
    case 0: // Start transfer (Receiver)
        {
            if(_state != None) { M5_LOGI("Failed to start transfer(R)"); break; }

            _addr = addr;
            _fname = pl->name;
            _path = _dir + _fname;
            _size = pl->fileSize;
            _crc32 = pl->crc32;
            _progress = _length = 0;

            M5_LOGI("Stanby for incomming [%s] %lu", _fname.c_str(), _size);
            
            _file = bus_lock([this]()
            {
                return sd.open(this->_path.c_str(), O_WRITE | O_CREAT | O_TRUNC);
            });

            if(_file)
            {
                _state = Status::Recv;
                _startTime = millis();
                post_ack(addr);
            }
            else
            {
                M5_LOGE("Failed to open [%s]", _path.c_str());
            }
        }
        break;
    case 1: // Receive data block
        {
            if(_state != Status::Recv || _progress + pl->bufSize > _size || _addr != addr)
            {
                M5_LOGE("Failed to receive"); break;
            }

            auto len = bus_lock([this, &pl]()
            {
                return this->_file.write(pl->buf, pl->bufSize);
            });
            if(len != pl->bufSize)
            {
                M5_LOGE("Failed to write %lu/%lu", len, pl->bufSize);
                // TODO;retry
            }
            post_ack(addr);
            
            _length = pl->bufSize;
            _progress += len;
            auto now = millis();
            _speed = ((float)_progress / (now - _startTime)) * 1000.f;
            if(_progress >= _size)
            {
                bus_lock([this]()
                {
                    this->_file.flush();
                    this->_file.close();
                });
                _state = Status::None;
                _endTime = now;
                _speed = ((float)_size / timeRequired()) * 1000.f;
                M5_LOGI("Finished %lu (%f) %lu CRC:%x", timeRequired(), transferedRate() * 100, _speed, _crc32);
            }
            //
        }
        break;
    }
}
