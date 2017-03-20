all: main.o convolve_base.o convolve.o convolve_sse_3.o
	gcc -g -Wall ./*.o -o convtest -losmocore

clean:
	rm -f ./*.o
	rm -f ./convtest

main.o: main.c
	gcc -g -Wall -c main.c

convolve_base.o: ../../Transceiver52M/common/convolve_base.c
	gcc -std=c99 -c ../../Transceiver52M/common/convolve_base.c

convolve.o: ../../Transceiver52M/x86/convolve.c
	gcc -std=c99 -c ../../Transceiver52M/x86/convolve.c -I ../../Transceiver52M/common/ -msse3 -DHAVE_SSE3

convolve_sse_3.o: ../../Transceiver52M/x86/convolve_sse_3.c
	gcc -std=c99 -c ../../Transceiver52M/x86/convolve_sse_3.c -I ../../Transceiver52M/common/ -msse3 -DHAVE_SSE3
