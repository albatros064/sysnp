#include "cpu.h"
#include "machine.h"

int main(int argc, char* argv[]) {
	Machine machine(65536, argv[1]);
	machine.run();
	return 0;
}

