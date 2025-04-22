#include <boost/test/unit_test.hpp>
#include "machine.h"
#include "nbus/memory.h"

using namespace sysnp::nbus;

BOOST_AUTO_TEST_SUITE(MemoryDevice)

BOOST_AUTO_TEST_CASE(memoryModule) {
    MemoryModule module(0, 65536, false, "", 0, 0, "Ook");

    BOOST_CHECK(module.getName() == "Ook");
    BOOST_CHECK(module.getReadLatency() == 0);
    BOOST_CHECK(module.getWriteLatency() == 0);

    BOOST_CHECK(module.read(0x1ffff) == 0);

    module.write(0xff00, 0x55);
    module.write(0xff01, 0xaa);
    BOOST_CHECK(module.read(0xff00) == 0x55);
    BOOST_CHECK(module.read(0xff01) == 0xaa);
}

BOOST_AUTO_TEST_CASE(memory, * boost::unit_test::depends_on("NBus/nbus")) {
    char config[] = "module: memory\n\
device: 0x1f0010\n\
ioHole: 0xf00000\n\
ioHoleSize: 0x040000\n\
ioAddress: 0xf00000\n\
modules:\n\
  - {size: 4096, name: \"4MB RAM\", start: 0x000000, rom: false, readLatency: 0, writeLatency: 1}\n\
  - {size: 64, name: \"64KB ROM\", start: 0xfe0000, rom: true, file: \"bios.bin\", readLatency: 0, writeLatency: 0}";
    auto tree = ryml::parse_in_place(config);
    auto memoryConfig = tree.rootref();

    auto machine = std::make_shared<sysnp::Machine>();

    auto memory = std::make_shared<Memory>();
    memory->setMachine(machine);
    memory->init(memoryConfig);

    auto nbus = std::make_shared<NBus>();
    nbus->setMachine(machine);

    auto busInterface = std::make_shared<NBusInterface>(nbus, nbus);
    auto memInterface = std::make_shared<NBusInterface>(nbus, memory);
    nbus->addInterface(busInterface);
    nbus->addInterface(memInterface);
    nbus->postInit();

    memory->setInterface(memInterface);
    memory->postInit();


    BOOST_CHECK(busInterface->senseSignal(NBusSignal::NotReady) == 0);

    BOOST_TEST_CHECKPOINT("writing first test");

    busInterface->assertSignal(NBusSignal::Address, 0xff00);
    busInterface->assertSignal(NBusSignal::WriteEnable, 0b11);
    busInterface->assertSignal(NBusSignal::Data, 0xaa55);

    memory->clockDown();

    busInterface->deassertSignal(NBusSignal::Address);
    busInterface->deassertSignal(NBusSignal::WriteEnable);
    busInterface->deassertSignal(NBusSignal::Data);

    memory->clockUp();

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::NotReady) == 1);

    memory->clockDown();
    memory->clockUp();

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::Data) == 0);
    BOOST_CHECK(busInterface->senseSignal(NBusSignal::NotReady) == 0);

    memory->clockDown();
    memory->clockUp();

    BOOST_TEST_CHECKPOINT("reading first test");

    busInterface->assertSignal(NBusSignal::Address, 0xff00);
    busInterface->assertSignal(NBusSignal::ReadEnable, 0b01);

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::Address) == 0xff00);
    BOOST_CHECK(busInterface->senseSignal(NBusSignal::ReadEnable) == 0b01);

    memory->clockDown();
    
    busInterface->deassertSignal(NBusSignal::Address);
    busInterface->deassertSignal(NBusSignal::ReadEnable);

    memory->clockUp();

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::NotReady) == 0);
    BOOST_CHECK(busInterface->senseSignal(NBusSignal::Data) == 0xaa55);

    memory->clockDown();
    memory->clockUp();

    BOOST_TEST_CHECKPOINT("writing second test");

    busInterface->assertSignal(NBusSignal::Address, 0xff02);
    busInterface->assertSignal(NBusSignal::WriteEnable, 0b11);
    busInterface->assertSignal(NBusSignal::Data, 0x55aa);

    memory->clockDown();

    busInterface->deassertSignal(NBusSignal::Address);
    busInterface->deassertSignal(NBusSignal::WriteEnable);
    busInterface->deassertSignal(NBusSignal::Data);

    memory->clockUp();
    memory->clockDown();
    memory->clockUp();
    memory->clockDown();
    memory->clockUp();

    BOOST_TEST_CHECKPOINT("reading second test");

    busInterface->assertSignal(NBusSignal::Address, 0xff00);
    busInterface->assertSignal(NBusSignal::ReadEnable, 0b11);

    memory->clockDown();

    busInterface->deassertSignal(NBusSignal::Address);
    busInterface->deassertSignal(NBusSignal::ReadEnable);

    memory->clockUp();

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::Data) == 0xaa55);

    memory->clockDown();
    memory->clockUp();

    BOOST_CHECK(busInterface->senseSignal(NBusSignal::Data) == 0x55aa);
}

BOOST_AUTO_TEST_SUITE_END()
