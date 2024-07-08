.segment kmem

.segment ktext

monitor_enter:
    adr $0, $1, sz_monitor_welcome_message
    jal print_string
    adr $0, $1, sz_monitor_prompt
    jal print_string
  _monitor_echo_loop:
    jal get_char
    mov $0, $2
    jal print_char
    j _monitor_echo_loop
    
sz_monitor_welcome_message:
    .stringz "Welcome!\n"
sz_monitor_prompt:
    .stringz "> "

