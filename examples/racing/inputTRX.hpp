/*
  input value TRX
 */
#ifndef INPUT_TRX_HPP
#define INPUT_TRX_HPP

#include <gob_transceiver.hpp>
#include <tuple>
#include <array>
#include <internal/gob_esp_now_log.hpp>

// Since deque uses a lot of memory(Memory fragmentation also occurs), use a ring buffer instead
template <typename T, size_t Capacity> class RingBuffer
{
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
        else {  LIB_LOGE("Overflow"); }
    }
    void pop()
    {
        if(!empty())
        {
            _head = (_head + 1) % Capacity;
            --_size;
        }
        else { LIB_LOGE("Empty"); }
    }
    inline const T& front() { return _buf[_head]; }
    inline bool empty() const { return !_size; }
    inline bool full() const { return _size == Capacity; }
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

    uint64_t available() const { return _ackedSeq > _poped ? _ackedSeq - _poped : 0; }
    // Call if available
    std::pair<int, int> pop()
    {
        //LIB_LOGE("%u:%u", _sent.size(), _received.size());

        int s = _sent.front();
        int r = _received.front();
        _sent.pop();
        _received.pop();
        ++_poped;
        return std::make_pair(s, r);
    }
    
    bool post(const int val)
    {
        return with_lock([this, &val]()
        {
            if(this->postReliable(val) != 0)
            {
                this->_sent.push(val);
                return true;
            }
            return false;
        });
    }

  protected:
    virtual void onReceive(const MACAddress& addr, const void* data, const uint8_t length)
    {
        LIB_LOGE("RECV");
        _received.push(*(const int*)data);
        _ackedSeq = peerInfo(addr)->recvAck;
    }

  private:
    uint64_t _poped{}, _ackedSeq{};
    RingBuffer<int, 8>  _sent, _received;
};
#endif
