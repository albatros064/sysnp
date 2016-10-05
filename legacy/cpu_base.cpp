#include "cpu_base.h"

#include <stdint.h>
#include <stdlib.h>

CPU_Base::CPU_Base(uint8_t dbits, uint8_t lbits, uint8_t pbits, uint16_t regs, uint16_t aux, uint64_t mem_size, uint8_t *memory):
 _data_bits         (dbits   ),
 _laddress_bits     (lbits   ),
 _paddress_bits     (pbits   ),
 _register_count    (regs    ),
 _register_count_aux(aux     ),
 _memory_size       (mem_size),
 _memory_file       (memory  ) {
	_register_file = (uint64_t *) malloc(sizeof(uint64_t) * (regs + aux) );
	for (int i = 0; i < regs + aux; i++) {
		_register_file[i] = 0;
	}
}

CPU_Base::~CPU_Base() {
	free(_register_file);
}
