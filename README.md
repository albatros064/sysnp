sysNP
====
(C) 2011-2023 Nathaniel Powell

## Description
sysNP is a computer system emulation project to feed my CPU instruction set design habit. The goal is to build a device emulator, including memory modules, bus topology, and I/O devices that can be connected to an emulator for any one of said instruction sets.

## Current modules
* Machine operating loop
* NBus: system bus; 16 bits data, 24 bits address, 4 interrupt lines.
* Memory: NBus Device; RAM and ROM facilities.
* N16R: NBus Device; 16-bit RISC-y CPU loosely inspired by MIPS. Instructions are 16 or 32 bits.
