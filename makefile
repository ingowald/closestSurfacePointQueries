all: aosExample soaExample benchmark


OPENMP=-openmp -DUSE_OPENMP=1

CXX=icpc
CXXFLAGS= -O3 -march=native -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC
# ${OPENMP}

bvh.o: bvh.cpp bvh.h distanceQueries.h

distanceQueries.o: distanceQueries.cpp bvh.h distanceQueries.h

libbvhdq.a: bvh.o distanceQueries.o
	ar cr libbvhdq.a bvh.o distanceQueries.o

aosExample: aosExample.cpp libbvhdq.a
	icpc -o $@ $<   ${CXXFLAGS} libbvhdq.a

soaExample: soaExample.cpp libbvhdq.a
	icpc -o $@ $<   ${CXXFLAGS} libbvhdq.a

benchmark: benchmark.cpp libbvhdq.a
	icpc -o $@ $<   ${CXXFLAGS} libbvhdq.a

clean:
	-rm benchmark benchmark.o aosExample soaExample bvh.o distanceQueries.o libbvhdq.so libbvhdq.a
