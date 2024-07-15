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
    sd   $40, 20($45) ; store $sp
    sd   $44, 16($45) ; store $gp
    sd   $46, 24($45) ; store $fp
    sd   $47, 28($45) ; store $ra
    mov  $40, $50
    sd   $40,  0($45)
    sd   $41,  4($45)
    sd   $42,  8($45)
    sd   $43, 12($45)

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
    ld  $41, 4($45)
exception_handler_final:
    ; restore most top-level registers
    ld  $40, 28($45) ; pull $ra from memory
    mov $57, $40    ; stash $ra for later

    ld  $40,  0($45) ; restore $0/$1
    ;d  $41,  4($45) ; restore $2/$3 (above)
    ld  $42,  8($45) ; restore $4/$5
    ld  $43, 12($45) ; restore $6/$7

    ld  $44, 16($45) ; restore $gp
    ld  $46, 24($45) ; restore $fp
    ld  $45, 20($45) ; restore $sp
    
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
