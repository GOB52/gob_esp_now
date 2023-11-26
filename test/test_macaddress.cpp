#include <gtest/gtest.h>
#include <gob_mac_address.hpp>

using goblib::esp_now::MACAddress;

TEST(MACAddress, Basic)
{
    uint8_t tmp[] = {1,2,3,4,5,7 };
    constexpr MACAddress a0;
    constexpr MACAddress a1((uint64_t)0x123456789abc); // little endian
    constexpr MACAddress a2(0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54);
    MACAddress a3({1,2,3,4,5,6});
    MACAddress a4(tmp);
    MACAddress a5("01:02:03:04:Ab:06");
    MACAddress a6(goblib::esp_now::BROADCAST);
    MACAddress a7,a8;
    MACAddress a9(tmp);

    a7 = a1; // copy
    a8 = std::move(a9); // move

    uint8_t* n8 = nullptr;
    const char* ns = nullptr;
    MACAddress an8(n8);
    MACAddress ans(ns);
    
    // OUI/NIC
    EXPECT_EQ(a0.OUI(), 0);
    EXPECT_EQ(a0.NIC(), 0);
    EXPECT_EQ(a1.OUI(), 0xbc9a78);
    EXPECT_EQ(a1.NIC(), 0x563412);
    EXPECT_EQ(a2.OUI(), 0xFEDCBA);
    EXPECT_EQ(a2.NIC(), 0x987654);
    EXPECT_EQ(a3.OUI(), 0x010203);
    EXPECT_EQ(a3.NIC(), 0x040506);
    EXPECT_EQ(a4.OUI(), 0x010203);
    EXPECT_EQ(a4.NIC(), 0x040507);
    EXPECT_EQ(a5.OUI(), 0x010203);
    EXPECT_EQ(a5.NIC(), 0x04ab06);
    EXPECT_EQ(a6.OUI(), 0xFFFFFF);
    EXPECT_EQ(a6.NIC(), 0xFFFFFF);
    EXPECT_EQ(a7.OUI(), 0xbc9a78);
    EXPECT_EQ(a7.NIC(), 0x563412);
    EXPECT_EQ(a8.OUI(), 0x010203);
    EXPECT_EQ(a8.NIC(), 0x040507);
    EXPECT_EQ(a9.OUI(), 0);
    EXPECT_EQ(a9.NIC(), 0);

    // Properties
    EXPECT_TRUE(a0.isUnicast());
    EXPECT_FALSE(a0.isMulticast());
    EXPECT_FALSE(a0.isBroadcast());
    EXPECT_FALSE(a0.isLocal());
    EXPECT_TRUE(a0.isUniversal());

    EXPECT_TRUE(a1.isUnicast());
    EXPECT_FALSE(a1.isMulticast());
    EXPECT_FALSE(a1.isBroadcast());
    EXPECT_FALSE(a1.isLocal());
    EXPECT_TRUE(a1.isUniversal());

    EXPECT_TRUE(a2.isUnicast());
    EXPECT_FALSE(a2.isMulticast());
    EXPECT_FALSE(a2.isBroadcast());
    EXPECT_TRUE(a2.isLocal());
    EXPECT_FALSE(a2.isUniversal());

    EXPECT_FALSE(a3.isUnicast());
    EXPECT_TRUE(a3.isMulticast());
    EXPECT_FALSE(a3.isBroadcast());
    EXPECT_FALSE(a3.isLocal());
    EXPECT_TRUE(a3.isUniversal());

    EXPECT_FALSE(a4.isUnicast());
    EXPECT_TRUE(a4.isMulticast());
    EXPECT_FALSE(a4.isBroadcast());
    EXPECT_FALSE(a4.isLocal());
    EXPECT_TRUE(a4.isUniversal());
    
    EXPECT_FALSE(a5.isUnicast());
    EXPECT_TRUE(a5.isMulticast());
    EXPECT_FALSE(a5.isBroadcast());
    EXPECT_FALSE(a5.isLocal());
    EXPECT_TRUE(a5.isUniversal());

    EXPECT_FALSE(a6.isUnicast());
    EXPECT_TRUE(a6.isMulticast());
    EXPECT_TRUE(a6.isBroadcast());
    EXPECT_TRUE(a6.isLocal());
    EXPECT_FALSE(a6.isUniversal());

    // Compare
    EXPECT_EQ(a0, an8);
    EXPECT_EQ(a0, ans);
    EXPECT_EQ(a0, a9);
    EXPECT_EQ(a4, a8);
    EXPECT_EQ(a7, a1);
    EXPECT_NE(a0, a1);
    EXPECT_NE(a0, a2);
    EXPECT_NE(a0, a3);
    EXPECT_NE(a0, a4);
    EXPECT_NE(a0, a5);
    EXPECT_NE(a0, a6);
    EXPECT_NE(a0, a7);
    EXPECT_NE(a0, a8);
                                
    EXPECT_GE(a1, a1);
    EXPECT_LE(a1, a1);

    EXPECT_LT(a3, a4);
    EXPECT_LT(a3, a5);
    EXPECT_LE(a3, a4);
    EXPECT_LE(a3, a5);
    EXPECT_GT(a4, a3);
    EXPECT_GT(a5, a4);
    EXPECT_GE(a4, a3);
    EXPECT_GE(a5, a4);

    EXPECT_GE(a6, a0);
    EXPECT_GE(a6, a1);
    EXPECT_GE(a6, a2);
    EXPECT_GE(a6, a3);
    EXPECT_GE(a6, a4);
    EXPECT_GE(a6, a5);
    EXPECT_GE(a6, a6);
    EXPECT_GE(a6, a7);
    EXPECT_GE(a6, a8);
    EXPECT_GE(a6, a9);

    // Element access
    EXPECT_EQ(a2[0], 0xFE);
    EXPECT_EQ(a2[1], 0xDC);
    EXPECT_EQ(a2[2], 0xBA);
    EXPECT_EQ(a2[3], 0x98);
    EXPECT_EQ(a2[4], 0x76);
    EXPECT_EQ(a2[5], 0x54);

    EXPECT_EQ(a2.data()[0], 0xFE);
    EXPECT_EQ(a2.data()[1], 0xDC);
    EXPECT_EQ(a2.data()[2], 0xBA);
    EXPECT_EQ(a2.data()[3], 0x98);
    EXPECT_EQ(a2.data()[4], 0x76);
    EXPECT_EQ(a2.data()[5], 0x54);
    
    // Cast
    uint64_t a64{};
    a64 = (uint64_t)a0;
    EXPECT_EQ(a64, 0ULL);
    a64 = (uint64_t)a1;
    EXPECT_EQ(a64, 0x123456789abc);
    a64 = (uint64_t)a2;
    EXPECT_EQ(a64, 0x547698badcfe);
    a64 = (uint64_t)a3;
    EXPECT_EQ(a64, 0x060504030201);
    a64 = (uint64_t)a4;
    EXPECT_EQ(a64, 0x070504030201);
    a64 = (uint64_t)a5;
    EXPECT_EQ(a64, 0x06AB04030201);
    a64 = (uint64_t)a6;
    EXPECT_EQ(a64, 0xFFFFFFFFFFFF);

    EXPECT_FALSE((bool)a0);
    EXPECT_TRUE(!a0);
    EXPECT_FALSE(!a1);
    EXPECT_TRUE((bool)a1);
}

TEST(MACAddress, methods)
{
    uint8_t tmp[] = {1,2,3,4,5,7 };
    constexpr MACAddress a0;
    constexpr MACAddress a1((uint64_t)0x123456789abc); // little endian
    constexpr MACAddress a2(0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54);
    MACAddress a3({1,2,3,4,5,6});
    MACAddress a4(tmp);
    MACAddress a5("01:02:03:04:Ab:06");
    MACAddress a6(goblib::esp_now::BROADCAST);

    struct Foo
    {
        const MACAddress* addr;
        const char* str;
    };
    Foo tbl[] = 
    {
        { &a0, "00:00:00:00:00:00" },
        { &a1, "bC:9A:78:56:34:12" },
        { &a2, "Fe:Dc:bA:98:76:54" },
        { &a3, "01:02:03:04:05:06" },
        { &a4, "01:02:03:04:05:07" },
        { &a5, "01:02:03:04:ab:06" },
        { &a6, "ff:ff:Ff:fF:FF:ff" }
    };
    // toString , parse
    for(auto it = std::begin(tbl); it != std::end(tbl); ++it)
    {
        //EXPECT_STREQ(it->addr->toString().c_str(), it->str) << it->str;
        EXPECT_TRUE(String(it->str).equalsIgnoreCase(it->addr->toString())) << it->str;
        MACAddress tmp;
        tmp.parse(it->str);
        EXPECT_EQ(*(it->addr), tmp) << it->str;
    }

    // Failed to parse
    {
        MACAddress fp;
        const char* ftbl[] =
                {
                    "",
                    "12-34#56/78*9A~BC",
                    "AB:CD:EF:GH:IJ:KL",
                    "12:34:56",
                    "0x123456789abc",
                };
        for(auto it = std::begin(ftbl); it != std::end(ftbl); ++it)
        {
            EXPECT_FALSE(fp.parse(*it)) << *it;
            EXPECT_EQ((uint64_t)fp, 0ULL) << *it;
        }
    }
    
    // get
    esp_mac_type_t mtTable[4] =
    {
        ESP_MAC_WIFI_STA,
        ESP_MAC_WIFI_SOFTAP,
        ESP_MAC_BT,
        ESP_MAC_ETH,
    };
    std::vector<MACAddress> ma;
    for(int i=0;i<4;++i)
    {
        MACAddress a(mtTable[i]);
        EXPECT_NE((uint64_t)a, 0ULL) << "mt:" << mtTable[i];
        ma.push_back(a);
    }
    // Unique address?
    auto it = std::unique(ma.begin(), ma.end());
    EXPECT_EQ(it, ma.end());
}
