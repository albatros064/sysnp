objects = main.o machine.o cpu_base.o np16_2.o

sysnp: $(objects)
	g++ -o sysnp $(objects)

main.o: cpu_base.h machine.h np16_2.h
machine.o: cpu_base.h machine.h
cpu_base.o: cpu_base.h
np16_2.o: np16_2.h

np16_2.h: cpu_base.h

.PHONY: clean
clean:
	rm sysnp $(objects)
