objects = main.o machine.o memory.o cpu.o

sysnp: $(objects)
	g++ -o sysnp $(objects)

main.o: cpu.h machine.h
machine.o: cpu.h machine.h
memory.o: memory.h exceptions.h
cpu.o: cpu.h

.PHONY: clean
clean:
	rm sysnp $(objects)
