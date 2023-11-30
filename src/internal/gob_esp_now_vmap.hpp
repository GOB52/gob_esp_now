/*!
  @file gob_esp_now_vmap.hpp
  @brief Implementing a map with vector
  @note Pursuit of memory efficiency and iteration speed
 */
#ifndef GOB_ESP_NOW_VMAP_HPP
#define GOB_ESP_NOW_VMAP_HPP

#include <vector>

namespace goblib { namespace esp_now {

/*!
  @class vmap
  @brief std::map-like map class implemented with std::vector
  @note Adding an element causes the element to be sorted by the value of key (for binary search)
 */

template <typename Key, typename T> class vmap
{
  public:
    using value_type = std::pair<Key, T>;
    using container_type = std::vector<value_type>;
    using key_type = Key;
    using mapped_type = T;
    using size_type = typename container_type::size_type;
    using iterator = typename container_type::iterator;
    using const_iterator = typename container_type::const_iterator;
    using reverse_iterator = typename container_type::reverse_iterator;
    using const_reverse_iterator = typename container_type::const_reverse_iterator;
    
public:
    vmap() = default;

    ///@name
    ///@{
    //    inline T& at(const key_type& key) { return const_cast<T&>(static_cast<const decltype(*this)&>(*this).at(key)); }
    T& at(const key_type& key)
    {
        auto it = find(key);
        if (it != _v.end() && it->first == key) { return it->second; }
        return _v.front().second;
    }

    const T& at(const key_type& key) const
    {
        auto it = find(key);
        if (it != _v.cend() && it->first == key) { return it->second; }
        else
        {

#if 0
#if defined(__EXCEPTIONS)
        throw std::out_of_range("vmap.at");
#else
        assert(false && "out_of_range");
        abort();
#endif
#endif
        return _v.front().second;
        }
    }
    T& operator[](const key_type& key)
    {
        auto it = find2(key);
        if (it == _v.end() || it->first != key) { it = _v.insert(it, std::make_pair(key, T())); }
        return it->second;
    }
    T& operator[](key_type&& key)
    {
        auto it = find2(key);
        if (it == _v.end() || it->first != key) { it = _v.insert(it, std::make_pair(std::move(key), T())); }
        return it->second;
    }
    ///@}

    ///@name Iterators
    ///@{
    inline iterator begin() noexcept{ return _v.begin(); }
    inline const_iterator begin() const noexcept{ return _v.begin(); }
    inline const_iterator cbegin() const noexcept{ return _v.cbegin(); }
    inline iterator end() noexcept{ return _v.end(); }
    inline const_iterator end() const noexcept{ return _v.end(); }
    inline const_iterator cend() const noexcept{ return _v.cend(); }

    inline reverse_iterator rbegin() noexcept { return _v.rbegin(); }
    inline const_reverse_iterator rbegin() const noexcept { return _v.rbegin(); }
    inline const_reverse_iterator crbegin() const noexcept{ return _v.crbegin(); }
    inline reverse_iterator rend() noexcept { return _v.rend(); }
    inline const_reverse_iterator rend() const noexcept { return _v.rend(); }
    inline const_reverse_iterator crend() const noexcept { return _v.crend(); }
    ///@}

    ///@name Capacity
    ///@{
    inline bool empty() const noexcept { return _v.empty(); }
    inline size_type size() const noexcept { return _v.size(); }
    inline size_type max_size() const noexcept { return _v.max_size; }
    ///@}

    ///@name Modifiers
    ///@{
    inline void clear() noexcept { _v.clear(); }
    template <class... Args> std::pair<iterator, bool> emplace(Args&&... args)
    {
        value_type val(std::forward<Args>(args)...);
        auto it = std::lower_bound(_v.begin(), _v.end(), val, [](const value_type& a, const value_type& b)
        {
            return a.first < b.first;
        });
        if(it == _v.end() || it->first != val.first) { it = _v.insert(it, std::move(val));  return {it, true}; }
        return {it, false};
    }
    size_type erase(const key_type& key)
    {
#if 0
        auto it = std::remove_if(_v.begin(), _v.end(), [&key](const value_type& pair) { return pair.first == key; });
        size_type removed = std::distance(it, _v.end());
        _v.erase(it, _v.end());
        return removed;
#else
        auto it = find(key);
        if(it != _v.end() && it->first == key) { _v.erase(it, it + 1); return 1; }
        return 0;
#endif
    }
    inline iterator erase(const_iterator position) { return _v.erase(position); }
    inline iterator erase(const_iterator first, const_iterator last) { return _v.erase(first, last); }
    ///@}

    ///@name Lookup
    ///@{
    inline size_type count(const key_type& key) const { return (size_type)(find(key) != _v.end());  }
    inline const_iterator find(const key_type& key) const
    {
        auto it = std::lower_bound(_v.begin(), _v.end(), key, [](const value_type& pair, const key_type& k) { return pair.first < k; });
        return (it != _v.end() && it->first == key) ? it : _v.end();
    }
    inline iterator find(const key_type& key)
    {
        auto it = std::lower_bound(_v.begin(), _v.end(), key, [](const value_type& pair, const key_type& k) { return pair.first < k; });
        return (it != _v.end() && it->first == key) ? it : _v.end();
    }
    ///@}

  private:
    inline iterator find2(const key_type& key)
    {
        return std::lower_bound(_v.begin(), _v.end(), key, [](const value_type& pair, const key_type& k) { return pair.first < k; });
    }

    container_type _v;
};
//
}}
#endif
