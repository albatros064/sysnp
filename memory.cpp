#include "memory.h"
#include "exceptions.h"
#include <stdlib.h>

Memory::Memory(uint32_t size):
_bytes(size) {
	_data = (uint8_t *) malloc(_bytes);

	if (!_data) {
		throw OutOfMemory(size);
	}
}

Data16_Value& Memory16::operator[](const uint32_t offset) {
	if (_data == 0) {
		throw EmptyArray(offset);
	}

	if (offset > _bytes) {
		throw ArrayIndexOutOfBounds(offset, _bytes);
	}
}

