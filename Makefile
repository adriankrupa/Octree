CXX ?= clang++

INCLUDES = -I .
CXXFLAGS = -std=c++11 -ffast-math -pthread -Wall -g

test: tests.o gtest-all.o
	$(CXX) $(CXXFLAGS) tests.o gtest-all.o -o Octree.out

tests.o: tests.cpp Octree.h
	$(CXX) -c $(CXXFLAGS) -DPOINTS=${POINTS} $(INCLUDES) $< -o $@

gtest-all.o: gtest/gtest-all.cc
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $@

run:
	./Octree.out

clear:
	rm -rf Octree.out tests.o gtest-all.o
