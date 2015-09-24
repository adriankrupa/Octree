CXX ?= g++-4.9
POINTS ?= 100
INCLUDES = -I .
CXXFLAGS = $(CXXFLAGS) -std=c++11 -pthread -Wall -g -Wno-unknown-pragmas

test: tests.o gtest-all.o
	$(CXX) $(CXXFLAGS) tests.o gtest-all.o -o Octree.out

tests.o: tests.cpp Octree.h
	$(CXX) -c $(CXXFLAGS) -DPOINTS=${POINTS} $(INCLUDES) $< -o $@

gtest-all.o: gtest/gtest-all.cc
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $@

run:
	./Octree.out

valgrind:
	valgrind --leak-check=full --track-origins=yes --dsymutil=yes ./Octree.out

clear:
	rm -rf Octree.out tests.o gtest-all.o
