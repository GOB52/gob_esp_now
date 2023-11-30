#include <gtest/gtest.h>
#include "gob_mac_address.hpp"
#include "internal/gob_esp_now_vmap.hpp"
#include <esp_system.h>
#include <M5Unified.h>
#include <random>
#include <map>

using goblib::esp_now::vmap;

auto rng = std::default_random_engine {};

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

    // count
    EXPECT_EQ(s_map.count(1), s_map.count(1));
    EXPECT_EQ(s_map.count(15), s_map.count(15));
    EXPECT_EQ(s_map.count(-100), s_map.count(-100));

    // emplace
    // Not exists
    auto rs = s_map.emplace(15, 30);
    auto rv = v_map.emplace(15, 30);
    EXPECT_EQ(rs.first->first, rv.first->first);   // iterator
    EXPECT_EQ(rs.second, rv.second); // inserted? bool
    EXPECT_EQ(s_map.size(), v_map.size());
    // Exists
    rs = s_map.emplace(60, 120);
    rv = v_map.emplace(60, 120);
    EXPECT_EQ(rs.first->first, rv.first->first);   // iterator
    EXPECT_EQ(rs.second, rv.second); // inserted? bool
    EXPECT_EQ(s_map.size(), v_map.size());

    // iterator
    {
        auto sbeg = s_map.begin();
        auto send = s_map.end();
        --send;
        auto vbeg = v_map.begin();
        auto vend = v_map.end();
        --vend;
        EXPECT_EQ(sbeg->first, vbeg->first);
        EXPECT_EQ(send->first, vend->first);
    }
    {
        auto sbeg = s_map.rbegin();
        auto send = s_map.rend();
        --send;
        auto vbeg = v_map.rbegin();
        auto vend = v_map.rend();
        --vend;
        EXPECT_EQ(sbeg->first, vbeg->first);
        EXPECT_EQ(send->first, vend->first);
    }
    
    // clear
    s_map.clear();
    v_map.clear();
    EXPECT_EQ(s_map.empty(), v_map.empty());
}

#if 0
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

#define RSIZE (3000)
//#define RSIZE (20)

TEST(vmap, benchmark)
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
  On Core2

#define RSIZE (3000)

[   389][I][test_vmap.cpp:135] TestBody(): std::map[]= tm:56976 mem:117096
[   504][I][test_vmap.cpp:145] TestBody(): vmap[]= tm:114246 mem:32784    <<== Memory efficient, Add is slow.

[   512][I][test_vmap.cpp:161] TestBody(): std::map find:1606
[   514][I][test_vmap.cpp:169] TestBody(): vmap find:2061 <<== Search is moderately fast

[   517][I][test_vmap.cpp:179] TestBody(): std::map at:2457
[   523][I][test_vmap.cpp:187] TestBody(): vmap at:3076 <<== Search is moderately fast

[   526][I][test_vmap.cpp:194] TestBody(): std::map for-auto:592
[   532][I][test_vmap.cpp:200] TestBody(): std::map for-auto:147 <<== Iteration is fast

[   557][I][test_vmap.cpp:210] TestBody(): std::map erase:17743
[   670][I][test_vmap.cpp:219] TestBody(): std::map erase:113166 <<== Erase is very slow (due to high memory copy)

#define RSIZE (20)

[   329][I][test_vmap.cpp:135] TestBody(): std::map[]= tm:298 mem:800
[   335][I][test_vmap.cpp:145] TestBody(): vmap[]= tm:131 mem:272 <<==
[   343][I][test_vmap.cpp:161] TestBody(): std::map find:660
[   349][I][test_vmap.cpp:169] TestBody(): vmap find:1111 <==
[   354][I][test_vmap.cpp:179] TestBody(): std::map at:10
[   360][I][test_vmap.cpp:187] TestBody(): vmap at:12 <== 
[   366][I][test_vmap.cpp:194] TestBody(): std::map for-auto:9
[   373][I][test_vmap.cpp:200] TestBody(): std::map for-auto:2 <==
[   379][I][test_vmap.cpp:210] TestBody(): std::map erase:118
[   386][I][test_vmap.cpp:219] TestBody(): std::map erase:20 <==

The larger the number of elements, the more pronounced the difference in memory usage.
*/

