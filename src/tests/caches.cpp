#define BOOST_TEST_MODULE Cache
#include <boost/test/unit_test.hpp>
#include "../machine/nbus/cpu/cache.h"

using namespace sysnp::nbus::n16r;

uint16_t lineTestData[16] = {
    0x0123,0x4567,0x89ab,0xcdef,0xfedc,0xba98,0x7654,0x3210,
    0x0123,0x4567,0x89ab,0xcdef,0xfedc,0xba98,0x7654,0x3210
};

uint16_t altLineTestData[16] = {
    0x1032,0x5476,0x98ba,0xdcfe,0xefcd,0xab89,0x6745,0x2301,
    0x1032,0x5476,0x98ba,0xdcfe,0xefcd,0xab89,0x6745,0x2301
};

BOOST_AUTO_TEST_CASE(cacheLine) {
    CacheLine line(32, 22, 5);

    BOOST_CHECK(!line.valid);
    BOOST_CHECK(!line.contains(32, 45));

    std::vector<uint8_t> lineData;
    for (int i = 0; i < 16; i++) {
        auto w = lineTestData[i];
        lineData.push_back(w >> 8);
        lineData.push_back(w & 0xff);
    }

    line.load(32, 45, 0, lineData);
    BOOST_CHECK(line.valid);
    BOOST_CHECK(line.contains(32, 45));
    BOOST_CHECK(line.contains(34, 45));

    auto d = line.read(34, 45, 2);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == 0x45);
    BOOST_CHECK(d[1] == 0x67);

    line.invalidate();
    BOOST_CHECK(!line.valid);
}

BOOST_AUTO_TEST_CASE(cacheBin) {
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

    CacheBin bin(32, 22, 5, 4);
    BOOST_CHECK(!bin.contains(way0 | 32, 45));

    bin.load(way0 | 32, 45, 0, lineData);
    BOOST_CHECK  (bin.contains(way0 | 32, 45));
    BOOST_REQUIRE(bin.contains(way0 | 34, 45));

    auto d = bin.read(way0 | 34, 45, 2);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == lineData[2]);
    BOOST_CHECK(d[1] == lineData[3]);

    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK(!bin.contains(way1 | 32, 45));
    BOOST_CHECK(!bin.contains(way2 | 32, 45));
    BOOST_CHECK(!bin.contains(way3 | 32, 45));
    BOOST_CHECK(!bin.contains(way4 | 32, 45));

    bin.load(way1 | 32, 45, 0, altData);
    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK(!bin.contains(way2 | 32, 45));
    BOOST_CHECK(!bin.contains(way3 | 32, 45));
    BOOST_CHECK(!bin.contains(way4 | 32, 45));

    d = bin.read(way1 | 32, 45, 2);

    bin.load(way2 | 32, 45, 0, lineData);
    bin.load(way3 | 32, 45, 0, altData);
    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK( bin.contains(way2 | 32, 45));
    BOOST_CHECK( bin.contains(way3 | 32, 45));
    BOOST_CHECK(!bin.contains(way4 | 32, 45));

    d = bin.read(way2 | 32, 45, 2);
    d = bin.read(way3 | 32, 45, 2);

    bin.load(way4 | 32, 45, 0, altData);
    BOOST_CHECK(!bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK( bin.contains(way2 | 32, 45));
    BOOST_CHECK( bin.contains(way3 | 32, 45));
    BOOST_CHECK( bin.contains(way4 | 32, 45));

    d = bin.read(way1 | 32, 45, 2);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == altData[0]);
    BOOST_CHECK(d[1] == altData[1]);

    d = bin.read(way4 | 32, 45, 2);
    d = bin.read(way0 | 32, 45, 2);
    BOOST_CHECK(d.size() == 0);

    bin.load(way0 | 32, 45, 0, altData);
    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK(!bin.contains(way2 | 32, 45));
    BOOST_CHECK( bin.contains(way3 | 32, 45));
    BOOST_CHECK( bin.contains(way4 | 32, 45));
    
    d = bin.read(way0 | 32, 45, 2);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == altData[0]);
    BOOST_CHECK(d[1] == altData[1]);

    d = bin.read(way1 | 33, 45, 2);
    BOOST_CHECK(d.size() == 2);
    BOOST_CHECK(d[0] == altData[1]);
    BOOST_CHECK(d[1] == altData[2]);

    bin.invalidate(way4 | 32, 45);
    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK(!bin.contains(way2 | 32, 45));
    BOOST_CHECK( bin.contains(way3 | 32, 45));
    BOOST_CHECK(!bin.contains(way4 | 32, 45));

    bin.load(way2 | 32, 45, 0, lineData);
    d = bin.read(way2 | 33, 45, 4);
    BOOST_CHECK(d.size() == 4);
    BOOST_CHECK(d[0] == lineData[1]);
    BOOST_CHECK(d[1] == lineData[2]);
    BOOST_CHECK(d[2] == lineData[3]);
    BOOST_CHECK(d[3] == lineData[4]);

    // load data for an alternate asid, at an address that
    // matches existing data for the original asid
    bin.load(way1 | 32, 109, 0, lineData);
    BOOST_CHECK( bin.contains(way0 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 45));
    BOOST_CHECK( bin.contains(way2 | 32, 45));
    BOOST_CHECK(!bin.contains(way3 | 32, 45));
    BOOST_CHECK(!bin.contains(way4 | 32, 45));
    BOOST_CHECK( bin.contains(way1 | 32, 109));

    // verify the old data/asid is intact
    d = bin.read(way1 | 32, 45, 2);
    BOOST_CHECK(d[0] == altData[0]);
    BOOST_CHECK(d[1] == altData[1]);

    // verify the new data/asid is intact
    d = bin.read(way1 | 32, 109, 2);
    BOOST_CHECK(d[0] == lineData[0]);
    BOOST_CHECK(d[1] == lineData[1]);
}

BOOST_AUTO_TEST_CASE(cacheMemory) {
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
    BOOST_CHECK(!cache.contains(0x000, 45)); // evicted
    BOOST_CHECK(!cache.contains(0x020, 45)); // evicted
    BOOST_CHECK( cache.contains(0x040, 45)); // persists
    // check second way
    BOOST_CHECK( cache.contains(0x400, 45)); // persists
    BOOST_CHECK( cache.contains(0x420, 45)); // persists
    BOOST_CHECK( cache.contains(0x440, 45)); // persists
    // check "third way"
    BOOST_CHECK( cache.contains(0x800, 45)); // present
    BOOST_CHECK( cache.contains(0x820, 45)); // present
    BOOST_CHECK(!cache.contains(0x840, 45)); // never loaded
}

BOOST_AUTO_TEST_CASE(cacheController) {
    CacheController controller;

    controller.setCache(InstructionCache, 32, 5, 5, 2);
    controller.setCache(DataCache, 32, 5, 5, 4);

    BOOST_CHECK(!controller.contains(InstructionCache, 45, 45));

    auto readId = controller.queueRead(InstructionRead, 45, 45);
    BOOST_CHECK(readId == 1);
    BOOST_CHECK(controller.isOperationPrepared());

    auto op = controller.getOperation();
    BOOST_CHECK(op.operationId == readId);
    BOOST_CHECK(op.address == 32);
    BOOST_CHECK(op.asid == 45);
    BOOST_CHECK(op.bytes == 32);
    BOOST_CHECK(op.isReady());

    for (int i = 0; i < 16; i++) {
        controller.ingestWord(lineTestData[i]);
    }

    BOOST_CHECK( controller.contains(InstructionCache, 45, 45));
    BOOST_CHECK( controller.contains(InstructionCache, 32, 45));
    BOOST_CHECK(!controller.contains(InstructionCache, 64, 45));
}

