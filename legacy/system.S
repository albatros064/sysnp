.offset 0
	j __kernel_entry

__e_entry:
	move $k0, $at
	move $k1, $acc
	store $k0, __e_reg_store + at
	store $k1, __e_reg_store + acc
	mfc0 $acc, $cause
	j __e_type_jumps($acc)
	
__e_reg_store:
	.words 32

__e_type_jumps:
	# hardware interrupt
	# syscall

