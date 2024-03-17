
#include <M5Unified.h>
#include "imageTRX.hpp"
#include <CRC32.h>

using goblib::esp_now::MACAddress;

constexpr size_t Payload::SPLIT_SIZE;
//
ImageTRX::ImageTRX(const uint8_t tid) : TRX(tid)
{
}

ImageTRX::~ImageTRX()
{
}

void ImageTRX::purge()
{
    with_lock([this]()
    {
        _buf.reset();
        this->_buf = nullptr;
        this->_size = 0;
    });
}

// Start transfer (Sender)
bool ImageTRX::send(const MACAddress& addr, uint8_t* buf, const size_t sz)
{
    if(inProgress()) { M5_LOGW("busy"); return false; }

    _buf.reset(buf);
    _size = sz;
    CRC32 crc;
    crc.add(buf, sz);
    _crc32 = crc.calc();
    _addr = addr;

    _progress = _length = 0;
    _state = Status::Send;
    _sequence = 0;

#if !defined(USING_UNRELIABLE)
    M5_LOGD("Send buffer sz:%lu CRC:%x", _size, _crc32);
    Payload pl;
    pl.type = 0;
    pl.size = _size;
    pl.crc32 = _crc32;

    _startTime = millis();

    _sequence = postReliable(_addr, pl);
    if(!_sequence)
    {
        M5_LOGE("Failed to post");
        return false;
    }
    return true;
#else
    _startTime = millis();
    _sequence = 1;
    return true;
#endif
}

void ImageTRX::abort()
{
    with_lock([this]()
    {
        this->_state = Status::None;
        purge();
    });
}
        
void ImageTRX::update(const unsigned long ms)
{
    switch(_state)
    {
    case Status::Send: update_send(); break;
    case Status::Recv: update_recv(); break;
    default: break;
    }
}

void ImageTRX::update_recv()
{
    if(_progress >= _size)
    {
        if(_callback && _valid) { _callback(this, _state); }
        _state = Status::None;
    }
}

void ImageTRX::update_send()
{
    if(!_sequence) { return; }
#if !defined(USING_UNRELIABLE)
    if(!delivered(_sequence, _addr)) { return; }
    M5_LOGV("deliverd %llu", _sequence);

#else
    if(_sequence == 1)
    {
        M5_LOGD("Send buffer sz:%lu CRC:%x", _size, _crc32);
        Payload pl;
        pl.type = 0;
        pl.size = _size;
        pl.crc32 = _crc32;

        if(postUnreliable(_addr, pl))
        {
            ++_sequence;
        }
        else
        {
            abort();
        }
        return;
    }
#endif

    _progress += _length;
    auto now = millis();
    _speed = ((float)_progress / (now - _startTime)) * 1000.f;

    if(_progress >= _size)
    {
        _endTime = now;
        M5_LOGD("Finished %lu (%f%%) <%zu>", timeRequired(), transferedRate() * 100, _speed);
        if(_callback) { _callback(this, _state); }
        _state = Status::None;
        return;
    }
    
    Payload pl;
    auto len = std::min(Payload::SPLIT_SIZE, _size - _progress);
    memcpy(pl.buf, _buf.get() + _progress, len);

    pl.type = 1;
    pl.bufSize = len;

#if !defined(USING_UNRELIABLE)
    _sequence = postReliable(_addr, pl);
    if(!_sequence) { abort(); return; }
#else
    if(!postUnreliable(_addr, pl))
    {
        abort();
        return;
    }
#endif
    _length = len;
}
    
void ImageTRX::onReceive(const MACAddress& addr, const void* data, const uint8_t length)
{
    const Payload* pl = (Payload*)data;

    //M5_LOGI("Recv:%d", pl->type);

    switch(pl->type)
    {
    case 0: // Start transfer (Receiver)
        {
            if(_state != None) { M5_LOGE("Failed to start transfer(R)"); break; }

            _buf.reset(new uint8_t[pl->size]);
            if(!_buf) { M5_LOGE("Failed to alloc %zu", pl->size); break; }
            
            _addr = addr;
            _size = pl->size;
            _crc32 = pl->crc32;
            _progress = _length = 0;
            _valid = false;

            M5_LOGD("Stanby for incomming %lu : %x", _size, _crc32);

            _state = Status::Recv;
            _startTime = millis();
#if !defined(USING_UNRELIABLE)
            post_ack(addr);
#endif
        }
        break;
    case 1: // Receive data block
        {
            if(_state != Status::Recv || _progress + pl->bufSize > _size || _addr != addr)
            {
                M5_LOGE("Failed to receive"); break;
            }

            memcpy(_buf.get() + _progress, pl->buf, pl->bufSize);
#if !defined(USING_UNRELIABLE)
            post_ack(addr);
#endif            
            _length = pl->bufSize;
            _progress += _length;

            auto now = millis();
            _speed = ((float)_progress / (now - _startTime)) * 1000.f;
            if(_progress >= _size)
            {
                _endTime = now;
                CRC32 crc;
                crc.add(_buf.get(), _size);
                auto rcrc32 = crc.calc();
                _valid = rcrc32 == _crc32;
                if(rcrc32 != _crc32 || _progress != _size)
                {
                    M5_LOGE("Invalid data %d/%d", rcrc32 != _crc32,  _progress != _size);
                }
                M5_LOGD("%d Finished %lu msec %lu bytes/sec CRC:%x", _valid, timeRequired(), _speed, rcrc32);
            }
        }
        break;
    }
}

#if !defined(NDEBUG)
String ImageTRX::debugInfo() const
{
    using namespace goblib::esp_now;
    
    String s = Transceiver::debugInfo();
    s += '\n';

    s += formatString("[%s] ptr:%p sz:%zu pr:%zu len:%zu sp:%zu\n"
                      "seq:%llu ST:%lu ET:%lu State:%u"
                      ,
                      _addr.toString().c_str(), _buf.get(), _size, _progress, _length, _speed,
                      _sequence, _startTime, _endTime, _state
                      );
    return s;
}
#endif
