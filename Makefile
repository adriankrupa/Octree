CXX ?= g++-4.9
POINTS ?= 100000
INCLUDES = -I .
CXXFLAGS = -std=c++11 -pthread -Wall -Wno-unknown-pragmas
CXXFLAGS_DEBUG = -g -O0 --coverage

test: tests.o gtest-all.o
	$(CXX) --version
	$(CXX) $(CXXFLAGS) $(CXXFLAGS_DEBUG) tests.o gtest-all.o -o Octree.out

tests.o: tests.cpp Octree.h
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_DEBUG) -DPOINTS=${POINTS} $(INCLUDES) $< -o $@

gtest-all.o: gtest/gtest-all.cc
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_DEBUG) $(INCLUDES) $< -o $@

run:
	./Octree.out

valgrind:
	valgrind --leak-check=full --track-origins=yes --dsymutil=yes --show-reachable=yes --error-exitcode=1 ./Octree.out

clear:
	rm -rf Octree.out tests.o gtest-all.o *.gc*
