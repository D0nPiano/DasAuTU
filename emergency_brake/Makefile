# Vorsicht vor O3 wegen Out-of-Order Execution
CXXFLAGS = -Wall -Wextra -O2 -fopenmp -std=c++11

all: main

main: emergency_brake.cpp emergency_brake.h laserscan.h
	g++ $(CXXFLAGS) -o $@ emergency_brake.cpp 
