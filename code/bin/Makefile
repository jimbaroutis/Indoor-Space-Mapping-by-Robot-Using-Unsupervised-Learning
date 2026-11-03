INCL=../incl

CXX=g++
CXXFLAGS=-O3 -std=c++17 -Wall -Wextra -pthread

all: bigscale_msp

bigscale_msp: bigscale_msp.o human_robot_movement.o
        $(CXX) $(CXXFLAGS) -o bigscale_msp bigscale_msp.o human_robot_movement.o

run:
        ./bigscale_msp

clean:
        rm -f bigscale_msp *.o

human_robot_movement.o: ../src/human_robot_movement.cpp
        $(CXX) $(CXXFLAGS) -c ../src/human_robot_movement.cpp -I$(INCL)

bigscale_msp.o: ../src/bigscale_msp.cpp
        $(CXX) $(CXXFLAGS) -c ../src/bigscale_msp.cpp -I$(INCL)



