CXX ?= g++-4.9
POINTS ?= 100000
INCLUDES = -I .
CXXFLAGS = -std=c++11 -pthread -Wall -Wno-unknown-pragmas
CXXFLAGS_DEBUG = -g -O0 --coverage --pedantic
CXXFLAGS_RELEASE = -O3

test: tests.o gtest-all.o
	$(CXX) --version
	$(CXX) $(CXXFLAGS) $(CXXFLAGS_DEBUG) tests.o gtest-all.o -o Octree.out

testR: testsR.o gtest-allR.o
	$(CXX) --version
	$(CXX) $(CXXFLAGS) $(CXXFLAGS_RELEASE) testsR.o gtest-allR.o -o Octree.out

tests.o: tests.cpp Octree.h
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_DEBUG) -DPOINTS=${POINTS} $(INCLUDES) $< -o $@

gtest-all.o: gtest/gtest-all.cc
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_DEBUG) $(INCLUDES) $< -o $@

testsR.o: tests.cpp Octree.h
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_RELEASE) -DPOINTS=${POINTS} $(INCLUDES) $< -o $@

gtest-allR.o: gtest/gtest-all.cc
	$(CXX) -c $(CXXFLAGS) $(CXXFLAGS_RELEASE) $(INCLUDES) $< -o $@

run:
	./Octree.out

valgrind:
	valgrind --leak-check=full --errors-for-leak-kinds=definite --track-origins=yes --dsymutil=yes --show-reachable=yes --error-exitcode=1 ./Octree.out

clear:
	rm -rf Octree.out *.o *.gc*

cppCheck:
	cppcheck --enable=all --force --language=c++ --xml --verbose -f tests.cpp Octree.h  2> err.xml
	cppcheck-htmlreport --file err.xml --report-dir="CodeAnalysisReport" --title="Octree"
