#include <boost/test/unit_test.hpp>
#include "nbus/serial.h"

using namespace sysnp::nbus;

BOOST_AUTO_TEST_SUITE(SerialDevice)

BOOST_AUTO_TEST_CASE(serial, * boost::unit_test::depends_on("NBus/nbus")) {
    BOOST_CHECK(1);
}

BOOST_AUTO_TEST_SUITE_END()
