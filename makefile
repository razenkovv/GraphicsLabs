OBJ = main.o functions.o
TARGET = task

all: $(OBJ) link

main.o: main.cpp
	g++ -c main.cpp -o main.o

functions.o: functions.cpp
	g++ -c functions.cpp -o functions.o

run:
	./$(TARGET)

link: main.o
	g++ main.o functions.o -o $(TARGET) -lsfml-graphics -lsfml-window -lsfml-system

clean:
	rm -rf $(TARGET) *.o