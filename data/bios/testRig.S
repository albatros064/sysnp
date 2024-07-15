.arch n16r

.segment kmem
    .dword 0
m_test1:
    .dword 0
    .dword 0
    .dword 0
    .dword 0

.segment ktext
    adr $0,$1, m_test1
    sd $40, 0($40)
    sd $40, 1($40)

foo:
    j foo
