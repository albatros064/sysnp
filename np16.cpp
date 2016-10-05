#include "np16.h"

namespace sysnp {

Np16::Np16(uint8_t *mem, uint64_t mem_size):
 CPU_Base(32, 32, 32, 16, 16, mem_size, mem) {
}

Np16::~Np16() {}

void Np16::clock(NBus &nbus) {
}

uint64_t Np16::step(uint8_t &instruction_length) {
	instruction_length = 2;

	uint32_t ip = _register_file[INSTRUCTION_POINTER];
	uint32_t next_ip = ip + 2;

	uint8_t instr_lo = _memory_file[(ip + 0) % _memory_size];
	uint8_t instr_hi = _memory_file[(ip + 1) % _memory_size];

	uint8_t op[3] = { (0xf0 & instr_lo) >> 4, (0xe0 & instr_hi) >> 5, (0x10 & instr_hi) >> 4 };
	uint8_t re[3] = { (0x0f & instr_lo) >> 0, (0xf0 & instr_hi) >> 4, (0x0f & instr_hi) >> 0 };
	uint8_t im[2] = { (0xff & instr_hi) >> 0, (0x1f & instr_hi) >> 0 };

	uint32_t im32 = im[0];

	uint32_t va[4] = {
		_register_file[re[0] ],
		_register_file[re[1] ],
		_register_file[re[2] ],
		_register_file[   1  ]
	};

	uint32_t mem = va[0] + im[0];
	if (im[0] & 0x80) {
		mem += 0xffffff00;
	}

	switch (op[0]) {
		case 0x0: // addc
			if (_register_file[FLAGS_REGISTER] & FLAG_CARRY) {
				va[1]++;
			}
		case 0x1: // add
			if (re[0]) {
				uint64_t sum = va[1] + va[2];
				// handle carry
				if (sum > (uint64_t) ( (uint32_t) sum) ) {
					_register_file[FLAGS_REGISTER] |= FLAG_CARRY;
				}
				else {
					_register_file[FLAGS_REGISTER] &= ~FLAG_CARRY;
				}
				_register_file[re[0] ] = (uint32_t) sum;

				// Handle overflow
				if (
					( !(sum & 0x80000000) && va[1] & 0x80000000 && va[2] & 0x80000000) ||
					(  (sum & 0x80000000) && !va[1] & 0x80000000 && !va[2] & 0x80000000)
				) {
					fire_exception(EXCEPT_OVERFLOW, 0);
				}
			}
			break;
		case 0x2:
			if (im32 & 0x00000080) {
				im32 |= 0xffffff00;
			}
		case 0x3:
			_register_file[1] = va[0] + im32;
			break;
		case 0x4:  // load
			_register_file[1] =
				(uint32_t) _memory_file[(mem + 3) % _memory_size] << 24 |
				(uint32_t) _memory_file[(mem + 2) % _memory_size] << 16 |
				(uint32_t) _memory_file[(mem + 1) % _memory_size] <<  8 |
				(uint32_t) _memory_file[(mem + 0) % _memory_size] <<  0;
			break;
		case 0x5: // store
			_memory_file[(mem + 3) % _memory_size] = (va[3] & 0xff000000) >> 24;
			_memory_file[(mem + 2) % _memory_size] = (va[3] & 0x00ff0000) >> 16;
		case 0x6: // storeh
			_memory_file[(mem + 1) % _memory_size] = (va[3] & 0x0000ff00) >>  8;
		case 0x7: // storeb
			_memory_file[(mem + 0) % _memory_size] = (va[3] & 0x000000ff) >>  0;
			break;
		case 0x8:
			switch (op[1]) {
				case 0x0: // shl
					_register_file[1] = va[0] << im[1];
					break;
				case 0x1: // shlr
					_register_file[1] = va[0] << va[2];
					break;
				case 0x2: // shr
					_register_file[1] = (uint32_t) ( (int32_t) va[0] >> im[1]);
					break;
				case 0x3: // shrr
					_register_file[1] = (uint32_t) ( (int32_t) va[0] >> va[2]);
					break;
				case 0x4: // shrl
					_register_file[1] = va[0] >> im[1];
					break;
				case 0x5: // shrlr
					_register_file[1] = va[0] >> va[2];
					break;
				case 0x6: // signx
					if (re[0]) {
						if (op[2]) { // signxh
							if (va[2] & 0x00008000) {
								_register_file[re[0] ] = va[2] | 0xffff0000;
							}
							else {
								_register_file[re[0] ] = va[2] & 0x0000ffff;
							}
						}
						else { // signxb
							if (va[2] & 0x00000080) {
								_register_file[re[0] ] = va[2] | 0xffffff00;
							}
							else {
								_register_file[re[0] ] = va[2] & 0x000000ff;
							}
						}
					}
					break;
				case 0x7: // zerox
					if (re[0]) {
						if (op[2]) { // zeroxh
							_register_file[re[0] ] = va[2] & 0x0000ffff;
						}
						else { // zeroxb
							_register_file[re[0] ] = va[2] & 0x000000ff;
						}
					}
					break;
				default:
					break;
			}
			break;
		//case 0x9:
		case 0xa:
			
			break;
		case 0xb: // nor
			if (re[0] != 0) {
				_register_file[re[0] ] = ~(va[1] | va[2]);
			}
			break;
		case 0xc: // j
			next_ip = mem;
			break;
		case 0xd: // jal
			_register_file[RETURN_ADDRESS] = next_ip;
			next_ip = mem;
			break;
		case 0xe: // be
			if (_register_file[1] == 0) {
				next_ip += mem;
			}
			break;
		case 0xf: // bg
			if (_register_file[1] != 0 && !(_register_file[1] & 0x80000000) ) {
				next_ip += mem;
			}
			break;
		default:
			break;
	}

	_register_file[INSTRUCTION_POINTER] = next_ip;
	return ip;
}

void Np16::fire_interrupt(uint8_t interrupt) {
	// TODO: verif interrupt is within range

	uint32_t status_reg = _register_file[STATUS_REGISTER];
	_register_file[CAUSE_REGISTER] |= 1 << interrupt + 8;

	if (status_reg & 1 << interrupt + 8) {
		fire_exception(0, 0);
	}
}

void Np16::fire_exception(uint8_t cause, uint32_t bvaddr) {
	// check that interrupts are enabled
	uint32_t cause_reg  = _register_file[CAUSE_REGISTER ];
	uint32_t status_reg = _register_file[STATUS_REGISTER];
	if (status_reg & 1) {
		_register_file[EPC_REGISTER   ] = _register_file[INSTRUCTION_POINTER];
		_register_file[EFLAGS_REGISTER] = _register_file[FLAGS_REGISTER     ];
		_register_file[BVA_REGISTER   ] = bvaddr;

		uint8_t kernel_status = status_reg << 28 >> 26;
		status_reg = status_reg >> 6 << 6 | kernel_status;

		_register_file[STATUS_REGISTER] = status_reg;
		_register_file[CAUSE_REGISTER ] = cause_reg;
		_register_file[INSTRUCTION_POINTER] = 0x80000080;
	}
}

void Np16::rfe() {
	// restore the kernel status
	uint32_t status_register = _register_file[STATUS_REGISTER];
	uint8_t kernel_status = status_register << 26 >> 28;
	status_register = status_register >> 4 << 4 | kernel_status;
	_register_file[STATUS_REGISTER] = status_register;
	// restore the flags
	_register_file[FLAGS_REGISTER] = _register_file[EFLAGS_REGISTER];
}

char *Np16::decode(uint64_t address) {
	char *name = "<unknown>";
	char *instruction = (char *) malloc(21);



	uint8_t instr_lo = _memory_file[address];
	uint8_t instr_hi = _memory_file[address + 1];

	uint8_t op[3] = { (0xf0 & instr_lo) >> 4, (0xe0 & instr_hi) >> 5, (0x10 & instr_hi) >> 4 };
	uint8_t re[3] = { (0x0f & instr_lo) >> 0, (0xf0 & instr_hi) >> 4, (0x0f & instr_hi) >> 0 };
	uint8_t im[2] = { (0xff & instr_hi) >> 0, (0x1f & instr_hi) >> 0 };

	switch (op[0]) {
		case 0x0:
			name = "addc";

			break;
		case 0x1:
			name = "add";
			break;
		case 0x2: // fall through until we actuall deal with overflow
			name = "addis";
			break;
		case 0x3:
			name = "addi";
			break;
		case 0x4:  // load
			name = "load";
			break;
		case 0x5:
			name = "store";
			break;
		case 0x6:
			name = "storeh";
			break;
		case 0x7:
			name = "storeb";
			break;
		case 0x8:
			switch (op[1]) {
				case 0x0:
					name = "shl";
					break;
				case 0x1:
					name = "shlr";
					break;
				case 0x2:
					name = "shr";
					break;
				case 0x3:
					name = "shrr";
					break;
				case 0x4:
					name = "shrl";
					break;
				case 0x5:
					name = "shrlr";
					break;
				case 0x6:
					if (op[2]) {
						name = "signxh";
					}
					else {
						name = "signxb";
					}
					break;
				case 0x7:
					if (op[2]) {
						name = "zeroxh";
					}
					else {
						name = "zeroxb";
					}
					break;
				default:
					break;
			}
			break;
		//case 0x9:
		//case 0xa:
		case 0xb:
			name = "nor";
			break;
		case 0xc:
			name = "j";
			break;
		case 0xd:
			name = "jal";
			break;
		case 0xe:
			name = "bz";
			break;
		case 0xf:
			name = "bgz";
			break;
		default:
			break;
	}

	return name;
}

}; // namespace
