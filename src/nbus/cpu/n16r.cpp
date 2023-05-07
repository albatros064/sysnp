#include <libconfig.h++>

#include "n16r.h"

namespace sysnp {

namespace nbus {

namespace n16r {

N16R::N16R() {
}
N16R::~N16R() {
}

void N16R::init(const libconfig::Setting &setting) {
}

void N16R::postInit() {
}

void N16R::clockUp() {
}

void N16R::clockDown() {
}

std::string N16R::command(std::stringstream &input) {
    return "N16R. Ok.";
}

}; // namespace n16r

}; // namespace nbus

}; // namespace sysnp

