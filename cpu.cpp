#include "cpu.h"
#include <stdio.h>

CPU::CPU(uint8_t *memory_file, uint32_t memory_size):_memory_file(memory_file), _memory_size(memory_size) {
	_delay_slot = 0;
	_register_file[0] = 0;
}

uint16_t CPU::step() {
	uint16_t instruction;

	uint8_t opcode;
	uint8_t function[2];
	uint8_t registers[2];
	uint8_t immediate[2];

	/* Istruction fetch */
	instruction = _memory_file[_program_counter];
	instruction <<= 8;
	instruction += _memory_file[_program_counter + 1];

	/* Instruction decode */
	opcode       = (instruction & 0xe000) >> 13;
	registers[0] = (instruction & 0x1f00) >>  8;
	registers[1] = (instruction & 0x001f) >>  0;
	immediate[0] = (instruction & 0x00ff) >>  0;
	immediate[1] = (instruction & 0x000f) >>  0;
	function [0] = (instruction & 0x00e0) >>  5;
	function [1] = (instruction & 0x00f0) >>  4;

	// Decide which type of instruction it is, and route accordingly
	switch (opcode) {
		case 0:
			execute_r_0_instruction(function[0], registers[0], registers[1]);
			break;
		case 1:
		case 2:
		case 3:
			execute_i_instruction(opcode, registers[0], immediate[0]);
			break;
		case 4:
			execute_h_instruction(function[1], registers[0], immediate[1]);
			break;
		case 5:
			execute_r_5_instruction(function[0], registers[0], registers[1]);
			break;
		case 6:
			execute_r_6_instruction(function[0], registers[0], registers[1]);
			break;
		case 7:
			execute_r_7_instruction(function[0], registers[0], registers[1]);
			break;
		default:
			break;
	}

	// PC advance
	advance_program_counter();

	return instruction;
}

void CPU::execute_i_instruction(uint8_t opcode, uint8_t reg, uint8_t imm) {
	switch(opcode) {
		case 1:
			// lui $reg, imm
			if (reg == 0) {
				break;
			}
			_register_file[reg] = ((uint16_t) imm) << 8;
			break;
		case 2:
			// addi $reg, imm
			_acc = _register_file[reg] + imm; // TODO: Deal with sign
			break;
		case 3:
			// addiu $reg, imm
			_acc = _register_file[reg] + imm;
			break;
	}
}

void CPU::execute_r_0_instruction(uint8_t function, uint8_t reg_0, uint8_t reg_1) {
	// nop
	if (reg_0 == 0 || reg_0 == reg_1) {
		return;
	}

	switch (function) {
		case 0:
			// move $reg_0, $reg_1 ($reg_1 -> $reg_0)
			_register_file[reg_0] = _register_file[reg_1];
			break;
		case 1:
			// cmp $reg_0, $reg_1 ($reg_1 > reg_0 ? 1 : (reg_1 < reg_0 ? -1 : 0) )
			_acc = reg_1 > reg_0 ? 0x0001 : (reg_1 < reg_0 ? 0xffff : 0x0000);
			break;
		case 2:
			// load $reg_0, [$reg_1]
			_register_file[reg_0] = _memory_file[_register_file[reg_1]];
			break;
		case 3:
			// store $reg_0, [$reg_1]
			_memory_file[_register_file[reg_1]] = (uint8_t) (_register_file[reg_0] >> 8);
			_memory_file[_register_file[reg_1]+1] = (uint8_t) _register_file[reg_0];
			break;
		case 4:
			// add $reg_0, $reg_1
			_acc = _register_file[reg_0] + _register_file[reg_1]; // TODO: Deal with sign
			break;
		case 5:
			// sub $reg_0, $reg_1
			_acc = _register_file[reg_0] - _register_file[reg_1]; // TODO: Deal with sign
			break;
		case 6:
			// addu $reg_0, $reg_1
			_acc = _register_file[reg_0] + _register_file[reg_1];
			break;
		case 7:
			// subu $reg_0, $reg_1
			_acc = _register_file[reg_0] - _register_file[reg_1];
			break;
	}
}

void CPU::execute_r_5_instruction(uint8_t function, uint8_t reg_0, uint8_t reg_1) {
	switch (function) {
		case 0:
			// be $reg_0, $reg_1 (if $reg_1 == 0, PC + $reg_0 -> PC)
			if (_register_file[reg_1] == 0) {
				_delay_slot = 1;
			}
			break;
		case 1:
			// bg $reg_0, $reg_1 (if $reg_1 > 0, PC + $reg_0 -> PC)
			if (_register_file[reg_1] && !(_register_file[reg_1] & 0x8000) ) {
				_delay_slot = 1;
			}
			break;
		case 2:
			// bl $reg_0, $reg_1 (if $reg_1 < 0, PC + $reg_0 -> PC)
			if (_register_file[reg_1] & 0x8000) {
				_delay_slot = 1;
			}
			break;
		case 3:
			// bgeal $reg_0, $reg_1 (if $reg_1 >= 0, PC + 2 -> $ra, PC + $reg_0 -> PC)
			if (!(_register_file[reg_1] & 0x8000) ) {
				_register_file[31] = _program_counter + 2;
				_delay_slot = 1;
			}
			break;
		case 4:
			// bne $reg_0, $reg_1 / bgl $reg_0, $reg_1 (if $reg_1 <> 0, PC + $reg_0 -> PC)
			if (_register_file[reg_1] > 0) {
				_delay_slot = 1;
			}
			break;
		case 5:
			// bng $reg_0, $reg_1 / ble $reg_0, $reg_1 (if $reg_1 <= 0, PC + $reg_0 -> PC)
			if (!_register_file[reg_1] || (_register_file[reg_1] & 0x8000) ) {
				_delay_slot = 1;
			}
			break;
		case 6:
			// bnl $reg_0, $reg_1 / bge $reg_0, $reg_1 (if $reg_1 >= 0, PC + $reg_0 -> PC)
			if (!(_register_file[reg_1] & 0x8000) ) {
				_delay_slot = 1;
			}
			break;
		case 7:
			// blal $reg_0, $reg_1 (if $reg_1 < 0, PC + 2 -> $ra, PC + $reg_0 -> PC)
			if (_register_file[reg_1] & 0x8000) {
				_register_file[31] = _program_counter + 2;
				_delay_slot = 1;
			}
			break;
	}

	if (_delay_slot == 1) {
		_jump_pc = _program_counter + _register_file[reg_0];
	}
}

void CPU::execute_r_6_instruction(uint8_t function, uint8_t reg_0, uint8_t reg_1) {
	switch (function) {
		case 0:
			// and $reg_0, $reg_1
			_acc = _register_file[reg_0] & _register_file[reg_1];
			break;
		case 1:
			// or $reg_0, $reg_1
			_acc = _register_file[reg_0] | _register_file[reg_1];
			break;
		case 2:
			// xor $reg_0, $reg_1
			_acc = _register_file[reg_0] ^ _register_file[reg_1];
			break;
		case 3:
			// nor $reg_0, $reg_1
			_acc = ~(_register_file[reg_0] | _register_file[reg_1]);
			break;
		case 4:
			if (reg_0) {
				_register_file[reg_0] = _register_file[reg_0] & _register_file[reg_1];
			}
			break;
		case 5:
			if (reg_0) {
				_register_file[reg_0] = _register_file[reg_0] | _register_file[reg_1];
			}
			break;
		case 6:
			if (reg_0) {
				_register_file[reg_0] = _register_file[reg_0] ^ _register_file[reg_1];
			}
			break;
		case 7:
			if (reg_0) {
				_register_file[reg_0] = ~(_register_file[reg_0] | _register_file[reg_1]);
			}
			break;
	}
}

void CPU::execute_r_7_instruction(uint8_t function, uint8_t reg_0, uint8_t reg_1) {
	uint8_t aux = reg_1 & 0xf;
	switch (function) {
		case 0:
			// shl $reg, imm
			_acc = _register_file[reg_0] << _register_file[reg_1];
			break;
		case 1:
			// shr $reg, imm
			_acc = _register_file[reg_0] >> _register_file[reg_1];
			break;
		case 2:
			// shrl $reg_0, $reg_1
			_acc = (_register_file[reg_0] >> _register_file[reg_1]) | ( (_register_file[reg_0] & 0x8000) ? 0xffff << (16 - _register_file[reg_1]) : 0 );
			break;
		case 3:
			// exch $reg_0, $reg_1
			if (reg_0 && reg_1) {
				uint16_t tmp = _register_file[reg_0];
				_register_file[reg_0] = _register_file[reg_1];
				_register_file[reg_1] = tmp;
			}
			break;
		case 4:
			_acc = (_register_file[reg_0] << _register_file[reg_1]) | (_register_file[reg_0] >> (16 - _register_file[reg_1]) );
			break;
		case 5:
			_register_file[reg_0] = (_register_file[reg_0] << _register_file[reg_1]) | (_register_file[reg_0] >> (16 - _register_file[reg_1]) );
			break;
		case 6:
			// mtc0 $reg_0, $reg_1{3:0}
			if (reg_1 & 0x10) {

				// start by disallowing everything
				uint16_t mask = 0;

				// set up the modification masks
				if (_status & 0x2) {// are we kernel?
					// yes. let the kernel do whatever, kernel beware
					mask = 0xffff;
				}
				else {
					// no. don't let user-mode do much here
					if (aux == 0) {
						mask = 0; // TODO: fine-tune what usermode can mess with
					}
				}

				// wipe the stuff we're writing
				_register_aux[aux] &= ~mask;
				// and write only those portions
				_register_aux[aux] |= _register_file[reg_0] & mask;
			}
			else {
				// mfc0 $reg_0, $reg_1{3:0}
				if (reg_0) {
					_register_file[reg_0] = _register_aux[aux];
				}
			}
			break;
		case 7:
			execute_privelaged_instruction(reg_0, reg_1);
			break;
	}
}

void CPU::execute_h_instruction(uint8_t function, uint8_t reg, uint8_t imm) {
	if (function & 0x8) {
		execute_h_high_instruction(function & 0x7, reg, imm);
	}
	else {
		execute_h_low_instruction(function & 0x7, reg, imm);
	}
}

void CPU::execute_h_low_instruction(uint8_t function, uint8_t reg, uint8_t imm) {
	switch (function) {
		case 0:
			// shl $reg, imm
			_acc = _register_file[reg] << imm;
			break;
		case 1:
			// shr $reg, imm
			_acc = _register_file[reg] >> imm;
			break;
		case 2:
			// shrl $reg, imm
			_acc = (_register_file[reg] >> imm) | ( (_register_file[reg] & 0x8000) ? 0xffff << (16 - imm) : 0 );
			break;
		case 3:
			break;
		case 4:
			// rotr $reg, imm
			_acc = (_register_file[reg] << imm) | (_register_file[reg] >> (16 - imm) );
			break;
		case 5:
			// rotre $reg, imm
			_register_file[reg] = (_register_file[reg] << imm) | (_register_file[reg] >> (16 - imm) );
			break;
		case 6:
			// not $reg
			_acc = ~_register_file[reg];
			break;
		case 7:
			// note $reg
			_register_file[reg] = ~_register_file[reg];
			break;
	}
}

void CPU::execute_h_high_instruction(uint8_t function, uint8_t reg, uint8_t imm) {
	switch (function) {
		case 0:
			// bal $reg
			_register_file[31] = _program_counter + 2;
			// fall through
		case 1:
			// b $reg
			_jump_pc = _program_counter + _register_file[reg];
			// Since this is a straight branch and not a conditional branch, skip the delay slot.
			_delay_slot = 2;
			break;
		case 2:
			// jal $reg
			_register_file[31] = _program_counter + 2;
			// fall through
		case 3:
			// j $reg
			_jump_pc = _register_file[reg];
			// Since this is a jump and not a conditional branch, skip the delay slot.
			_delay_slot = 2;
			break;
		case 4:
			// shle $reg, imm
			_register_file[reg] = _register_file[reg] << imm;
			break;
		case 5:
			// shre $reg, imm
			_acc = _register_file[reg] >> imm;
			break;
		case 6:
		case 7:
			break;
	}
}

void CPU::execute_privelaged_instruction(uint8_t reg, uint8_t function) {
	switch (function) {
		case 0:
			// syscall
			execute_syscall();
			break;
		case 1:
			// eret

			// restore previous processor privelage level
			_status = (_status & 0xffc0) | ( (_status & 0x3f) >> 2);
			// jump back to where we were before the exception
			_delay_slot = 2;
			_jump_pc = _exception_program_counter;
			break;
		default:
			break;
	}
}

void CPU::execute_syscall() {
	_exception_program_counter = _program_counter + 2;
	_status = (_status & 0xffc0) | ( (_status << 2) & 0x3f);
	_delay_slot = 2;
	_jump_pc = EXCEPTION_ADDRESS;
	_cause = SYSCALL_CAUSE;
}

void CPU::advance_program_counter() {
	if (_delay_slot) {
		if (_delay_slot == 2) {
			_program_counter = _jump_pc;
			_delay_slot = 0;
			return;
		}
		else {
			_delay_slot = 2;
		}
	}
	
	_program_counter += 2;
}

