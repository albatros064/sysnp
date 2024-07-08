.arch n16r

.segment kmem

; KERNEL MEMORY
m_kernel_exception_memory:
    .dword 0 ; $40
    .dword 0 ; $41
    .dword 0 ; $42
    .dword 0 ; $43
    .dword 0 ; $44
    .dword 0 ; $45
    .dword 0 ; $46
    .dword 0 ; $47
    .dword 0 ; $60
    .dword 0 ; $65
    .dword 0 ; $66
    .dword 0 ; $67


; DRIVER MEMORY

.segment ktext
    ; set interrupt handler
    adr $0, $1, exception_handler
    mov $65, $40

    ; serial driver init
    jal driver_serial_init

    ; set the command interpreter's stack pointer
    adr $0, $1, 0x3ffffe
    mov $45, $40
    mov $46, $40

    ; 
    lui $0, 0xff
    ori $0, 0x0c
    mov $21, $0
    eret

    ; move into the command interpreter
    j monitor_enter

exception_handler:
    ; hold $0,$1 and kernel load store address
    xch  $40, $50
    adr  $0, $1, m_kernel_exception_memory
    xch  $45, $40
    ; store first-tier registers so we have breathing room
    sw   $0, 20($45) ; store stack pointer
    sw   $1, 22($45) ; .
    mov  $40, $44
    sw   $0, 16($45) ; store global pointer
    sw   $1, 18($45) ; .
    mov  $40, $46
    sw   $0, 24($45) ; store frame pointer
    sw   $1, 26($45) ; .
    mov  $40, $47
    sw   $0, 28($45) ; store return address
    sw   $1, 30($45) ; .
    mov  $40, $50
    sw   $0,  0($45)
    sw   $1,  2($45)
    sw   $2,  4($45)
    sw   $3,  6($45)
    sw   $4,  8($45)
    sw   $5, 10($45)
    sw   $6, 12($45)
    sw   $7, 14($45)

    mov  $0,  $20   ; grab cause
    andi $0,  0x3c  ;  isolate exception
    xor  $1,  $1
    xch  $40, $47
    ; $47 should now contain an offset into our jump table
    adr  $0, $1, j_exception_vectors
    addu $47, $40
    jr   $47        ;  jump to the target

    ; 16 jump targets for individual exception handlers
  j_exception_vectors:
    j interrupt_handler        ; int
    j exception_handler_exit
    j exception_handler_exit
    j exception_handler_exit
    j exception_handler_exit   ; addrl
    j exception_handler_exit   ; addrs
    j exception_handler_exit   ; ibus
    j exception_handler_exit   ; dbus
    j syscall_handler          ; syscall
    j exception_handler_exit   ; bkpt
    j exception_handler_exit   ; ri
    j exception_handler_exit
    j exception_handler_exit   ; ovf
    j exception_handler_exit
    j exception_handler_exit
    j exception_handler_exit

exception_handler_exit:
    ; restore $2 and $3 if we don't care about
    ; return values
    lw  $2, 4($45)
    lw  $3, 6($45)
exception_handler_final:
    ; restore most top-level registers
    lw  $0, 28($45) ; pull $ra from memory
    lw  $1, 30($45) ; .
    mov $57, $40    ; stash $ra for later
    lw  $0, 20($45) ; pull $sp from memory
    lw  $1, 22($45) ; .
    mov $50, $40    ; stash $sp for later
    lw  $0, 16($45) ; restore $gp
    lw  $1, 18($45) ; .
    mov $44, $40    ; .
    lw  $0, 24($45) ; restore $fp
    lw  $1, 26($45) ; .
    mov $46, $40    ; .
    lw  $0, 0($45)  ; restore $0
    lw  $1, 2($45)  ; restore $1
    ;w  $2, 4($45)  ; restore $2 (handled above)
    ;w  $3, 6($45)  ; restore $3 (handled above)
    lw  $4, 8($45)  ; restore $4
    lw  $5, 10($45) ; restore $5
    lw  $6, 12($45) ; restore $6
    lw  $7, 14($45) ; restore $7
    mov $45, $50    ; finally put $sp back in place
    
    mov $47, $67 ; grab epc
    xch $47, $57 ; put $ra back, set up return jump
    eret $57

interrupt_handler:
    xor  $1, $1
    mov  $0, $20
    mov  $2, $0
    shr  $2, 0x8
    andi $2, 0x8
    bne  $2, $1, driver_serial_interrupt
    j exception_handler_exit

syscall_handler:
    adr  $2, $3, j_syscall_vectors
    lw   $0, 0($45)
    andi $0, 3
    shl  $0, 3
    xor  $1, $1
    addu $41,$40
    xch  $40,$50
    jr   $41
  j_syscall_vectors:
    jal driver_serial_char_rx ; 0: read character
    j   _syscall_cleanup
    jal driver_serial_char_tx ; 1: write character
    j   _syscall_cleanup
    j   _syscall_cleanup      ; ??
    j   _syscall_cleanup
    j   _syscall_cleanup      ; ??
    j   _syscall_cleanup
  _syscall_cleanup:
    mov   $40, $67 ; adjust epc
    addiu $40, 2   ; .
    mov   $67, $40 ; .
    j exception_handler_final

.include "driver/serial.S"

.include "utils.S"
.include "monitor.S"
