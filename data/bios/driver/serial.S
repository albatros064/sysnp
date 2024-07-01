.segment kmem

  m_driver_serial:
    .byte 0  ; bool     out_ready
    .byte 0  ; reserved
    .byte 0  ; uint8_t  in_buffer_head
    .byte 0  ; uint8_t  in_buffer_tail
    .dword 0 ; char[16] in_buffer
    .dword 0 ; .
    .dword 0 ; .
    .dword 0 ; .

.segment ktext

driver_serial_init:
    adr $2, $3, m_driver_serial
    xor $0, $0
    sw  $0, 2($41) ; in_buffer_head = in_buffer_tail = 0
    ori $0, 1
    sb  $0, 0($41) ; out_ready = 1
    jr  $47

driver_serial_interrupt:
    adr  $2, $3, m_driver_serial
    adr  $0, $1, 0x00f00010 ; i/o address
    lw   $0, 0($40) ; read serial status word
    mov  $1, $0     ; extract input status
    shr  $1, 8      ; .
    andi $1, 1      ; .
    xor  $4, $4
    ; if no input byte, move on
    beq  $1, $4, _driver_serial_interrupt_output
    mov  $1, $0
    andi $1, 0xff   ; isolate input byte
    lw   $4, 2($41) ; read buffer head/tail
    mov  $5, $4
    shr  $5, 8      ; isolate head
    andi $4, 0xf    ; isolate tail
    mov  $6, $4
    addi $6, 1      ; increment head
    andi $6, 0xf    ; and wrap it to the buffer size
    ; if buffer is full, move on
    beq  $6, $5, _driver_serial_interrupt_output
    addi $4, 4
    sb   $1,$4($41)
    sb   $6, 2($41)
  _driver_serial_interrupt_output:
    shr  $0, 9      ; extract "output busy" status
    nor  $0, $0     ; invert for "output ready"
    andi $0, 1
    sb   $0, 0($41) ; and write it

    j exception_handler_exit

driver_serial_char_rx:
    lui   $2, 0xff
    ori   $2, 0xff
    adr   $4, $5, m_driver_serial
    lw    $0, 2($42)    ; load buffer head/tail
    mov   $1, $0        ; 
    shr   $1, 8         ; isolate head
    andi  $0, 0xf       ; isolate tail
    beq   $0, $1, _driver_serial_char_rx_return
    mov   $0, $1
    addi  $1, 4
    lb    $2, $1($42)
    addiu $0, 1
    andi  $0, 0xf
    sb    $0, 3($42)
  _driver_serial_char_rx_return:
    jr   $47

driver_serial_char_tx:
    adr  $4, $5, m_driver_serial
    lb   $2, 0($42)
    xor  $0, $0     ; 
    beq  $2, $0, _driver_serial_char_tx_return
    adr  $2, $3, 0x00f00010
    sb   $1, 0($41) ; send byte to serial device
    sb   $0, 0($42) ; out_ready := 0
    addi $0, 1      ; increment our return
  _driver_serial_char_tx_return:
    mov  $2, $0
    jr   $47

