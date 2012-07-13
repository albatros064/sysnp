#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

#include <stdio.h>

/* Exception Classes */
class GenericException {
public:
	GenericException(uint32_t val):_val(val) {}
	virtual void print() = 0;
protected:
	uint32_t _val;
};

class OutOfMemory : public GenericException {
public:
	OutOfMemory(uint32_t count):GenericException(count) {}

	void print() { printf("\nOutOfMemory Exception: Failed to allocate %d bytes.\n", _val); }
};

class ArrayIndexOutOfBounds : public GenericException {
public:
	ArrayIndexOutOfBounds(uint32_t offset, uint32_t max):GenericException(offset),_max(max) {}

	void print() { printf("\nArrayIndexOutOfBounds Exception: Attempted access array with index %d, but the maximum index is %d.\n", _val, _max); }
protected:
	uint32_t _max;
};

class EmptyArray : public GenericException {
public:
	EmptyArray(uint32_t offset):GenericException(offset) {}

	void print() { printf("\nEmptyArray Exception: Attempted to access element %d of an empty array.\n", _val); }
};

#endif

