.segment kmem
.segment ktext

print_string:
    subiu $45, 4    ; push $ra
    sd $47, -4($45) ; .
    mov $43, $40    ; 
  _print_string_loop:
    lb  $0, 0($43)
    xor $4, $4
    beq $0, $4, _print_string_exit
    jal print_char
    addiu $43, 1
    j   _print_string_loop
  _print_string_exit:
    ld  $47, -4($45) ; pop $ra
    addiu $45, 4     ; .
    jr  $47
    

print_char:
    mov $1, $0
    xor $4, $4
  _print_char_tx_loop:
    lli $0, 1
    syscall
    beq $2, $4, _print_char_tx_loop
    jr $47
    

get_char:
    xor  $0, $0
    syscall
    lui  $5, 0xff
    ori  $5, 0xff
    bne  $2, $5, _get_char_exit
    j get_char
  _get_char_exit:
    jr   $47
