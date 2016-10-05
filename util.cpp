#include <sstream>
#include <string>

namespace sysnp {

std::string to_hex(uint32_t value, bool prefix =true) {
    std::stringstream stream;
    if (prefix) {
        stream << "0x";
    }
    stream << std::hex << value;
    return stream.str();
}

}; // namespace

