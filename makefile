all: testDQ libbvhdq.so distanceTest

CXX=icpc
CXXFLAGS= -O3 -xAVX -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC -openmp
#CXXFLAGS=-static-libstdc++ -O3 -xAVX -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC -openmp

bvh.o: bvh.cpp bvh.h distanceQueries.h

distanceQueries.o: distanceQueries.cpp bvh.h distanceQueries.h

libbvhdq.so: bvh.o distanceQueries.o
	icpc -shared -o $@ bvh.o distanceQueries.o

testDQ: example.cpp libbvhdq.so
	icpc -o $@ $<  -std=c++11 -openmp -L. -lbvhdq 

distanceTest.o: distanceTest.cpp

distanceTest: distanceTest.o
	icpc -o $@ $<  -std=c++11 -openmp -L. -lbvhdq
