all: aosExample soaExample benchmark


OPENMP=-openmp -DUSE_OPENMP=1

#CXX=icpc
#CXXFLAGS= -O3 -march=native -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC

CXX=g++
CXXFLAGS= -O3 -march=native -DNDEBUG -std=c++11 -fPIC

# ${OPENMP}



bvh.o: src/bvh.cpp src/bvh.h src/*.h
	${CXX} ${CXXFLAGS} -c -o $@ $<

distanceQueries.o: src/distanceQueries.cpp src/*.h
	${CXX} ${CXXFLAGS} -c -o $@ $<

libbvhdq.a: bvh.o distanceQueries.o
	ar cr libbvhdq.a bvh.o distanceQueries.o

aosExample: examples/aosExample.cpp libbvhdq.a
	${CXX} ${CXXFLAGS} -o $@ $< libbvhdq.a

soaExample: examples/soaExample.cpp libbvhdq.a
	${CXX} ${CXXFLAGS} -o $@ $< libbvhdq.a

benchmark: examples/benchmark.cpp libbvhdq.a
	${CXX} ${CXXFLAGS} -o $@ $< libbvhdq.a

clean:
	-rm benchmark aosExample soaExample bvh.o distanceQueries.o libbvhdq.a
