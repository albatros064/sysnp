objects = main.o machine.o memory.o nbus.o

CPPFLAGS=-std=c++11

sysnp: $(objects)
	g++ -o sysnp $(objects) $(CPPFLAGS) -lconfig++ -lvncserver

main.o: machine.h
machine.o: nbus.h machine.h
memory.o: memory.h
nbus.o: nbus.h

nbus.h: device.h
memory.h: nbus.h

.PHONY: clean
clean:
	rm sysnp $(objects)
