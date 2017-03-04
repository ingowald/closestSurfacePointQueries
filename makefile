all: aosExample soaExample benchmark


OPENMP=-openmp -DUSE_OPENMP=1

CXX=icpc
CXXFLAGS= -O3 -xAVX -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC ${OPENMP}
#CXXFLAGS=-static-libstdc++ -O3 -xAVX -g -no-inline-max-total-size -inline-factor=1000 -DNDEBUG -restrict -std=c++11 -fPIC -openmp

bvh.o: bvh.cpp bvh.h distanceQueries.h

distanceQueries.o: distanceQueries.cpp bvh.h distanceQueries.h

libbvhdq.so: bvh.o distanceQueries.o
	icpc -shared -o $@ bvh.o distanceQueries.o

aosExample: aosExample.cpp libbvhdq.so
	icpc -o $@ $<  -std=c++11 ${OPENMP} -L. -lbvhdq 

soaExample: soaExample.cpp libbvhdq.so
	icpc -o $@ $<  -std=c++11 ${OPENMP} -L. -lbvhdq 

benchmark.o: benchmark.cpp

benchmark: benchmark.o
	icpc -o $@ $<  -std=c++11 ${OPENMP} -L. -lbvhdq

clean:
	-rm benchmark benchmark.o aosExample soaExample bvh.o distanceQueries.o libbvhdq.so
