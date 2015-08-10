CC = g++ 

CFLAGS = -o3

simulator: main.o simulator.o matching.o
	${CC} ${CFLAGS} main.o simulator.o matching.o -o simulator

main.o: main.cpp simulator.o
	${CC} ${CFLAGS} -c main.cpp -o main.o

simulator.o: simulator.cpp matching.o matching.h
	${CC} ${CFLAGS} -c simulator.cpp -o simulator.o

matching.o: matching.cpp matching.h
	${CC} ${CFLAGS} -c matching.cpp -o matching.o


clean:
	rm -f simulator.o matching.o simulator
