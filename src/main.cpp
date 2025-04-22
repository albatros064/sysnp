#include <iostream>
#include <fstream>

#include "machine/machine.h"

int main(int argc, char* argv[]) {
    std::ifstream file("hardware.yaml", std::ios::in|std::ios::binary|std::ios::ate);

    if (!file.is_open()) {
        std::cout << "Configuration file not found." << std::endl;
        return -1;
    }

    size_t size = file.tellg();
    file.seekg(0);

    char *fileContent = new char[size];
    file.read(fileContent, size);
    file.close();

    ryml::Tree tree = ryml::parse_in_place({fileContent, size});

    std::shared_ptr<sysnp::Machine> machine = std::make_shared<sysnp::Machine>();

    machine->load(tree.rootref());
    machine->run();
    
    return 0;
}

