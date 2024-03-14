/*
  input value TRX
 */
#ifndef INPUT_TRX_HPP
#define INPUT_TRX_HPP

#include <gob_transceiver.hpp>
#include <tuple>
#include <array>
#include <cassert>
#include <internal/gob_esp_now_log.hpp>

template <typename T, size_t Capacity> class RingBuffer
{
    static_assert(Capacity > 0, "Capacity must be > 0");
    static_assert(Capacity <= 255, "Capacity must be <= 255");
  public:
    RingBuffer() {}

    void push(const T& v)
    {
        if(!full())
        {
            _buf[_tail++] = v;
            _tail %= Capacity;
            ++_size;
        }
        else {  LIB_LOGE("Overflow"); abort(); }
    }
    void pop()
    {
        if(!empty())
        {
            _head = (_head + 1) % Capacity;
            --_size;
        }
        else { LIB_LOGE("Empty"); abort(); }
    }
    inline const T& front() { assert(_size > 0 && "Empty"); return _buf[_head]; }
    inline bool empty() const { return _size == 0; }
    inline bool full() const { return _size >= Capacity; }
    inline uint8_t size() const { return _size; }
    
  private:
    std::array<T, Capacity> _buf;
    uint8_t _head{}, _tail{}, _size{};
};


class InputTRX : public goblib::esp_now::Transceiver
{
  public:
    using MACAddress = goblib::esp_now::MACAddress;
    using goblib::esp_now::Transceiver::postReliable;

    explicit InputTRX(const uint8_t tid) : goblib::esp_now::Transceiver(tid) {}

    bool available() const { return delivered(_poped + 1); }
    
    std::pair<int, int> pop()
    {
        int s = _sent.front();
        int r = _received.front();
        _sent.pop();
        _received.pop();
        ++_poped;
        return std::make_pair(s, r);
    }
    
    bool post(const int val)
    {
        return with_lock([this,&val]()
        {
            if(this->postReliable(val) != 0)
            {
                this->_sent.push(val);
                return true;
            }
            LIB_LOGE("Failed to post");
            return false;
        });
    }

  protected:
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length)
    {
        // StarGame 後の通信で [Seq, acked]
        // A -- [1,0] -> B
        // A <- [1,0] -- B
        // となった時に双方 Acked 0 になるので 初回は明示的に ack 返す
        // Config で 強制 Ack を無効にしているための措置
        if(peerInfo(addr)->recvAck == 0) { post_ack(addr); } 
        _received.push(*(const int*)data);
    }

  private:
    uint64_t _poped{};
    RingBuffer<int, 16>  _sent, _received;
};
#endif
