#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stdio.h>

class Memory {
public:
	Memory(uint32_t);
	virtual ~Memory() { if (_data) free(_data); }

	uint32_t size() { return _bytes; }

protected:
	uint8_t *_data;
	uint32_t _bytes;
};

class Data_Value {};

class Data16_Value : public Data_Value {
public:
	Data16_Value(uint16_t v):_value(v) {}
	~Data16_Value() {}

	operator uint16_t() { return _value; }
	Data16_Value operator =(uint16_t val) { this->_value = val; return *this;}

protected:
	uint16_t _value;
};

class Memory16 : public Memory {
public:
	Memory16(uint32_t size):Memory(size) {}
	virtual ~Memory16() {}

	Data16_Value& operator[](const uint32_t);

protected:
};

class Memory_Module {
};

class RAM_Module : public Memory_Module {
public:
protected:
};

class ROM_Module : public Memory_Module {
public:
	ROM_Module(char *file_name);
	virtual ~ROM_Module() {}

	Data_Value operator[](const uint32_t offset);
protected:
};

#endif

