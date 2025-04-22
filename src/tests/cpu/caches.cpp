#include <boost/test/unit_test.hpp>
#include <iostream>
#include <iomanip>
#include "nbus/cpu/cache.h"

using namespace sysnp::nbus::n16r;

uint16_t lineTestData[16] = {
    0x0123,0x4567,0x89ab,0xcdef,0xfedc,0xba98,0x7654,0x3210,
    0x0123,0x4567,0x89ab,0xcdef,0xfedc,0xba98,0x7654,0x3210
};

uint16_t altLineTestData[16] = {
    0x1032,0x5476,0x98ba,0xdcfe,0xefcd,0xab89,0x6745,0x2301,
    0x1032,0x5476,0x98ba,0xdcfe,0xefcd,0xab89,0x6745,0x2301
};

BOOST_AUTO_TEST_SUITE(Caches)

BOOST_AUTO_TEST_CASE(standardCache) {
    uint32_t asid = 45;

    uint8_t offset = 32;
    uint8_t binBits = 5;
    uint8_t flags = 0;
    Cache<uint32_t, uint8_t, uint32_t, uint8_t> cache(binBits, 4, 1, false, -1, 0);

    std::vector<uint8_t> lineData;
    for (int i = 0; i < 16; i++) {
        auto w = lineTestData[i];
        lineData.push_back(w >> 8);
        lineData.push_back(w & 0xff);
    }
    std::vector<uint8_t> altData;
    for (int i = 0; i < 16; i++) {
        auto w = altLineTestData[i];
        altData.push_back(w >> 8);
        altData.push_back(w & 0xff);
    }

    uint32_t way0 = 0x00000000;
    uint32_t way1 = 0x10000000;
    uint32_t way2 = 0x20000000;
    uint32_t way3 = 0x30000000;
    uint32_t way4 = 0x40000000;

    BOOST_CHECK(cache.contains(way0 | offset, 1, asid) == CacheContainsNone);

    cache.load(way0 | offset, asid, 0, lineData);
    BOOST_CHECK(cache.contains( way0 | offset     ,  1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains( way0 | offset     , 32, asid) == CacheContainsLower );
    BOOST_CHECK(cache.contains((way0 | offset) - 1,  1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way0 | offset) - 1,  2, asid) == CacheContainsUpper );

    BOOST_REQUIRE(cache.contains((way0 | offset) + 2, 2, asid) == CacheContainsSingle);

    auto d = cache.read((way0 | offset) + 2, 2, asid);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == lineData[2]);
    BOOST_CHECK(d[1] == lineData[3]);

    BOOST_CHECK(cache.contains((way0 | offset), 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way1 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way2 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way3 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way4 | offset), 1, asid) == CacheContainsNone  );

    cache.load(way1 | offset, asid, 0, altData);

    BOOST_CHECK(cache.contains((way0 | offset), 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way1 | offset), 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way2 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way3 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way4 | offset), 1, asid) == CacheContainsNone  );

    cache.load(way2 | offset, asid, 0, lineData);
    BOOST_CHECK(cache.contains((way0 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way1 | offset), 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way2 | offset), 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way3 | offset), 1, asid) == CacheContainsNone  );

    d = cache.read(way1 | offset + 1, 2, asid);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == altData[1]);
    BOOST_CHECK(d[1] == altData[2]);

    cache.flush(way1 | offset, asid);
    BOOST_CHECK(cache.contains((way1 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way2 | offset), 1, asid) == CacheContainsSingle);

    cache.load(way2 | offset + 16, asid, 0, lineData);
    BOOST_CHECK(cache.contains((way2 | offset)     , 1, asid) == CacheContainsSingle);
    BOOST_CHECK(cache.contains((way2 | offset) + 16, 1, asid) == CacheContainsSingle);

    BOOST_CHECK(cache.contains((way2 | offset) + 15, 2, asid) == CacheContainsSplit );

    cache.flush();
    BOOST_CHECK(cache.contains((way0 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way1 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way2 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way3 | offset), 1, asid) == CacheContainsNone  );
    BOOST_CHECK(cache.contains((way4 | offset), 1, asid) == CacheContainsNone  );
}

BOOST_AUTO_TEST_CASE(singleValueCache) {
    uint32_t asid = 45;

    Cache<uint32_t, uint16_t, uint32_t, uint16_t> cache(4, 12, 1, true, -1, 0);

    BOOST_CHECK(true);
}

/*
BOOST_AUTO_TEST_CASE(cacheMemory) {
    uint32_t asid = 45;

    std::vector<uint8_t> lineData;
    for (int i = 0; i < 16; i++) {
        auto w = lineTestData[i];
        lineData.push_back(w >> 8);
        lineData.push_back(w & 0xff);
    }
    std::vector<uint8_t> altData;
    for (int i = 0; i < 16; i++) {
        auto w = altLineTestData[i];
        altData.push_back(w >> 8);
        altData.push_back(w & 0xff);
    }

    Cache cache(32, 5, 5, 2);

    uint16_t d;

    // this should fill both ways of every bin, and set lru;
    for (int i = 0; i < 64; i++) {
        cache.load(i << 5, 45, 0, (i & 1) ? altData : lineData);
        d = cache.readWord(i << 5, 45);
    }
    // and this should load two more, evicting two blocks
    for (int i = 64; i < 66; i++) {
        cache.load(i << 5, 45, 0, (i & 1) ? altData : lineData);
        d = cache.readWord(i << 5, 45);
    }

    // check first way
    BOOST_CHECK(!cache.contains(0x000, 2, asid)); // evicted
    BOOST_CHECK(!cache.contains(0x020, 2, asid)); // evicted
    BOOST_CHECK( cache.contains(0x040, 2, asid)); // persists
    // check second way
    BOOST_CHECK( cache.contains(0x400, 2, asid)); // persists
    BOOST_CHECK( cache.contains(0x420, 2, asid)); // persists
    BOOST_CHECK( cache.contains(0x440, 2, asid)); // persists
    // check "third way"
    BOOST_CHECK( cache.contains(0x800, 2, asid)); // present
    BOOST_CHECK( cache.contains(0x820, 2, asid)); // present
    BOOST_CHECK(!cache.contains(0x840, 2, asid)); // never loaded
}

*/

BOOST_AUTO_TEST_SUITE_END()
