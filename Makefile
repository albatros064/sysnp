out=sysnp
objects = main.o machine.o memory.o nbus.o util.o

CXX=clang++-3.8
CPPFLAGS=-std=c++11
LDFLAGS=-lconfig++ -lvncserver

$(out): $(objects)
	$(CXX) -o $(out) $(objects) $(CPPFLAGS) $(LDFLAGS)

main.o: machine.h
machine.o: nbus.h machine.h util.h
memory.o: memory.h
nbus.o: nbus.h
util.o: util.h

nbus.h: device.h
memory.h: nbus.h util.h

.PHONY: clean
clean:
	rm sysnp $(objects)
