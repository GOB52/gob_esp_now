#include <gtest/gtest.h>
#include "gob_mac_address.hpp"
#include "internal/gob_esp_now_vmap.hpp"
#include <esp_system.h>
#include <M5Unified.h>
#include <random>
#include <map>

using goblib::esp_now::vmap;

auto rng = std::default_random_engine {};

struct Foo
{
    explicit Foo(int v) : value(v) {}
    Foo(const Foo& b) { /*M5_LOGI("copy");*/ value = b.value; }
    Foo(Foo&& b) noexcept{ /*M5_LOGW("move");*/ value = b.value; }
    Foo& operator=(const Foo& b) { /*M5_LOGI("copy=");*/ if(this != &b) { value = b.value;}  return *this; }
    Foo& operator=(Foo&& b) { /*M5_LOGI("move=");*/ if(this != &b) { value = b.value;}  return *this; }

    bool operator==(const Foo& b) const { return value == b.value; }
    bool operator!=(const Foo& b) const { return !(value == b.value); }
    bool operator<(const Foo& b) const { return value < b.value; }

    int value{};
};


class profile
{
  public:
    explicit profile(int64_t& out) : _save(out) { _start_at = esp_timer_get_time(); }
    ~profile() {_save = esp_timer_get_time() - _start_at; }
  private:
    int64_t _start_at{};
    int64_t& _save;
};


TEST(vmap, compatibility)
{
    std::map<int,int> s_map;
    vmap<int,int> v_map;
    std::vector<int> v;

    for(int i=0;i<100;++i) { v.emplace_back(i); } 
    std::shuffle(v.begin(),v.end(), rng);    

    // empty
    EXPECT_EQ(s_map.empty(), v_map.empty());
    
    // [],at
    for(auto& e : v)
    {
        s_map[e] = e * 2;
        v_map[e] = e * 2;
    }
    for(auto& e : v) { EXPECT_EQ(s_map[e], v_map[e]); }
    for(auto& e : v) { EXPECT_EQ(s_map.at(e), v_map.at(e)); }

    // size
    EXPECT_EQ(s_map.size(), v_map.size());

    // erase
    // by key
    auto pos = v.size() - 2;
    auto se = s_map.erase(pos);
    auto ve = v_map.erase(pos);
    EXPECT_EQ(se, ve);
    EXPECT_EQ(s_map.size(), v_map.size());
    EXPECT_EQ(s_map.find(pos), s_map.end());
    EXPECT_EQ(v_map.find(pos), v_map.end());
    // by iterator
    auto sit = std::next(s_map.find(3), 1); // [4]
    auto vit = std::next(v_map.find(3), 1); // [4]
    EXPECT_EQ(sit->first, vit->first);
    sit = s_map.find(4);
    auto sit2 = s_map.erase(sit);
    vit = v_map.find(4);
    auto vit2 = v_map.erase(vit);
    EXPECT_EQ(s_map.size(), v_map.size());
    EXPECT_EQ(sit2->first, vit2->first);
    EXPECT_EQ(s_map.find(4), s_map.end());
    EXPECT_EQ(v_map.find(4), v_map.end());
    sit = std::next(s_map.find(3), 1); // [5]
    vit = std::next(v_map.find(3), 1); // [5]
    EXPECT_EQ(sit->first, vit->first);
    // by iterators
    sit = s_map.find(10);
    sit2 = s_map.find(20);
    sit2 = s_map.erase(sit, sit2);
    vit = v_map.find(10);
    vit2 = v_map.find(20);
    vit2 = v_map.erase(vit, vit2);
    EXPECT_EQ(sit2->first, vit2->first);
    EXPECT_EQ(s_map.size(), v_map.size());
    for(int i=10;i<20;++i)
    {
        EXPECT_EQ(s_map.find(i), s_map.end()) << "S) i:" << i;
        EXPECT_EQ(v_map.find(i), v_map.end()) << "V) i:" << i;
    }
    EXPECT_EQ(s_map.find(20)->first, v_map.find(20)->first);

    // clear
    s_map.clear();
    v_map.claer();
    EXPECT_EQ(s_map.empty(), v_map.empty());
}




#if 0
TEST(vmap,t)
{
    vmap<int,int> v_map;

    v_map[1000] = 1000;
    v_map[100] = 100;
    v_map[10] = 10;
    for(auto& e : v_map) { M5_LOGW("%d", e.first); }

    //v_map.sort();
    //for(auto& e : v_map) { M5_LOGW("%d", e.first); }
    
    auto it = v_map.find2(99);
    M5_LOGI("99: %d", std::distance(v_map.begin(),it)); // 0

    it = v_map.find(100);
    M5_LOGI("100 f: %d", std::distance(v_map.begin(),it)); //?
    it = v_map.find2(100);
    M5_LOGI("100 f2: %d", std::distance(v_map.begin(),it)); //?
    
    it = v_map.find2(101);
    M5_LOGI("101: %d", std::distance(v_map.begin(),it)); //1
}
#endif

#if 0

//#define RSIZE (1024 * 2)
#define RSIZE (20)

TEST(vmap, benchmark))
{
    int64_t tm;
    std::map<Foo, int> s_map;
    vmap<Foo, int> v_map;
    std::vector<int> rv;
    for(int i=0;i<RSIZE;++i) { rv.emplace_back(esp_random() & 0xFFFF); }
    std::shuffle(rv.begin(),rv.end(), rng);

//
    auto mem = esp_get_free_heap_size();
    {
        profile _(tm);
        for(auto& e : rv)
        {
            s_map[Foo(e)] = e * 2;
        }
    }
    M5_LOGI("std::map[]= tm:%lld mem:%u", tm, mem - esp_get_free_heap_size());

    mem = esp_get_free_heap_size();
    {
        profile _(tm);
        for(auto& e : rv)
        {
            v_map[Foo(e)] = e * 2;
        }
    }
    M5_LOGI("vmap[]= tm:%lld mem:%u", tm, mem - esp_get_free_heap_size());


    for(auto& e : rv)
    {
        Foo f(e);
        EXPECT_EQ(s_map[f],v_map[f]);
    }


    
    // find()
    {
        profile _(tm);
        for(int i=0;i<2000;++i)
        {
            auto it = s_map.find(Foo(i));
        }
    }
    M5_LOGI("std::map find:%lld", tm);
    {
        profile _(tm);
        for(int i=0;i<2000;++i)
        {
            auto it = v_map.find(Foo(i));
        }
    }
    M5_LOGI("vmap find:%lld", tm);

    // at()
    {
        profile _(tm);
        for(auto& e : rv)
        {
            auto a = s_map.at(Foo(e));
        }
    }
    M5_LOGI("std::map at:%lld", tm);
    {
        profile _(tm);
        for(auto& e : rv)
        {
            auto a = v_map.at(Foo(e));
        }
    }
    M5_LOGI("vmap at:%lld", tm);
    
    // iteration
    {
        profile _(tm);
        for(auto& e : s_map) { ++e.second; } 
    }
    M5_LOGI("std::map for-auto:%lld", tm);

    {
        profile _(tm);
        for(auto& e : v_map) { ++e.second; }
    }
    M5_LOGI("std::map for-auto:%lld", tm);

    // erace
    {
        profile _(tm);
        for(auto it = rv.crbegin();it != rv.crend(); ++it)
        {
            s_map.erase(Foo(*it));
        }
    }
    M5_LOGI("std::map erase:%lld", tm);

    {
        profile _(tm);
        for(auto it = rv.crbegin();it != rv.crend(); ++it)
        {
            v_map.erase(Foo(*it));
        }
    }
    M5_LOGI("std::map erase:%lld", tm);

    //    for(auto& e : s_map) { M5_LOGW("S:%5d", e.first.value); }
    //    for(auto& e : v_map) { M5_LOGW("V:%5d", e.first.value); }
}
#endif

/*
[   337][I][test_vmap.cpp:55] TestBody(): std::map[]= tm:14777 mem:40680
[   338][I][test_vmap.cpp:65] TestBody(): vmap[]= tm:1177 mem:8208
[   341][I][test_vmap.cpp:75] TestBody(): std::map find:976
[   348][I][test_vmap.cpp:83] TestBody(): vmap find:1883
[   352][I][test_vmap.cpp:110] TestBody(): std::map for-auto:204
[   359][I][test_vmap.cpp:116] TestBody(): std::map for-auto:53
[   371][I][test_vmap.cpp:126] TestBody(): std::map erase:5994
[   391][I][test_vmap.cpp:135] TestBody(): std::map erase:19424
*/
// かくかんすうがごかんせいあるか
/*


 */

