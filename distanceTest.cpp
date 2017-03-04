#include "distanceQueries.h"
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <string.h>
#include <math.h>

std::vector<double> point_x;
std::vector<double> point_y;
std::vector<double> point_z;

std::vector<double> vertex_x;
std::vector<double> vertex_y;
std::vector<double> vertex_z;
std::vector<int> index_x;
std::vector<int> index_y;
std::vector<int> index_z;

std::vector<double> result_x;
std::vector<double> result_y;
std::vector<double> result_z;
std::vector<double> result_dist;

std::vector<double> reference_dist;
std::vector<double> reference_x;
std::vector<double> reference_y;
std::vector<double> reference_z;

size_t numTriangles,numVertices;


#define DBG(a) /* nothing */

void loadTriangles(const std::string &fileName)
{
  FILE *file = fopen(fileName.c_str(),"r");
  if (!file) throw std::runtime_error("could not open "+fileName);

  char line[1000];

  if (fscanf(file,"NSURFPTS=%12li NTRI=%12li\n",&numVertices,&numTriangles)!=2) 
    throw std::runtime_error("could not parse num vertices/num trianlges from "+fileName);
  std::cout << "num vertices " << numVertices << std::endl;
  std::cout << "num triangles " << numTriangles << std::endl;
  fgets(line,1000,file);

  double x,y,z;
  int ID,i,j,k;

  // parse vertices
  for (int vtxID=0;vtxID<numVertices;vtxID++) {
    if (fscanf(file,"%i %lf %lf %lf\n",&ID,&x,&y,&z)!=4) 
      throw std::runtime_error("couldn't read vertex no "+std::to_string(vtxID));
    if (ID != vtxID+1)
      throw std::runtime_error("invalid vertex line "+std::to_string(vtxID));
    // printf(" vtx %i -> %f %f %f\n",ID,x,y,z);
    vertex_x.push_back(x);
    vertex_y.push_back(y);
    vertex_z.push_back(z);
  }
  std::cout << "loaded " << vertex_x.size() << " vertices" << std::endl;

  // parse indices
  for (int idxID=0;idxID<numTriangles;idxID++) {
    if (fscanf(file,"%i %i %i %i\n",&ID,&i,&j,&k)!=4) 
      throw std::runtime_error("couldn't read triangles indices for tri no "+std::to_string(idxID)
                               +" in "+fileName);
    if (ID != idxID+1)
      throw std::runtime_error("invalid vertex line "+std::to_string(idxID));
    // printf(" idx %i -> %i %i %i\n",idxID,i,j,k);
    index_x.push_back(i-1);
    index_y.push_back(j-1);
    index_z.push_back(k-1);
  }
  std::cout << "loaded " << index_x.size() << " triangles" << std::endl;

  fclose(file);
}

void loadReference(const std::string &fileName)
{
  FILE *file = fopen(fileName.c_str(),"r");
  if (!file) throw std::runtime_error("could not open "+fileName);

  for (int i=0;i<point_x.size();i++) {
    double x,y,z,s;
    if (fscanf(file,"%lf %lf %lf %lf\n",&x,&y,&z,&s) != 4)
      throw std::runtime_error("could not read reference result from "+fileName);
    reference_x.push_back(x);
    reference_y.push_back(y);
    reference_z.push_back(z);
    reference_dist.push_back(s);
  }
  std::cout << "loaded " << reference_dist.size() << " reference distances" << std::endl;
  fclose(file);
}

void loadPoints(const std::string &fileName)
{
  FILE *file = fopen(fileName.c_str(),"r");
  if (!file) throw std::runtime_error("could not open "+fileName);

  char line[1000];

  while (fgets(line,1000,file) && !feof(file) && strstr(line,"Zone")) {
    std::cout << "loading zone " << line;
    double x,y,z;
    
    while (fscanf(file,"%lf %lf %lf\n",&x,&y,&z)==3) {
      point_x.push_back(x);
      point_y.push_back(y);
      point_z.push_back(z);
    }
  }
  std::cout << "loaded total of " << point_x.size() << " points across all zones" << std::endl;
  fclose(file);
}

int main(int ac, char **av)
{
  if (ac != 2)
    throw std::runtime_error("usage: ./distanceTest <directory_with_dumps>");
  const std::string dir = av[1];

  loadTriangles(dir+"/triang_input.txt");
  loadPoints(dir+"/point_input.txt");
  loadReference(dir+"/tim.xyzs.txt");
  
  auto begin = std::chrono::system_clock::now();
  std::cout << "creating embree query object" << std::endl;
  distance_query_scene scene
    = rtdqNewTriangleMeshdi(vertex_x.data(),vertex_y.data(),vertex_z.data(),1,
                            index_x.data(),index_y.data(),index_z.data(),1,
                            index_x.size());
  auto done_build = std::chrono::system_clock::now();

  result_dist.resize(point_x.size());
  result_x.resize(point_x.size());
  result_y.resize(point_x.size());
  result_z.resize(point_x.size());


//   std::cout << "computing closest _VERTEX_ to query point 0" << std::endl;
//   float qpx = point_x[30];
//   float qpy = point_y[30];
//   float qpz = point_z[30];
//   float closestDist = 1000;
//   printf("query point %f %f %f\n",qpx,qpy,qpz);
//   for (int i=0;i<numVertices;i++) {
//     float px = vertex_x[i];
//     float py = vertex_y[i];
//     float pz = vertex_z[i];
//     float dx=qpx-px;
//     float dy=qpy-py;
//     float dz=qpz-pz;

//     float dist = sqrtf(dx*dx+dy*dy+dz*dz);
//     if (dist < closestDist) {
//       closestDist = dist;
//       printf("**** point %f %f %f dist %f, idx %i\n",px,py,pz,dist,i);
//     }
//   }

#if 0
  std::cout << "calling query (threaded)" << std::endl;
  rtdqComputeClosestPointsdi_threaded(scene,
                             // no position:
                             NULL,NULL,NULL,0,
                             // distance:
                             result_dist.data(),1,
                             // no prim IDs
                             NULL,0,
                             // input: the query points
                             point_x.data(),point_y.data(),point_z.data(),1,
                             // num points:
                             point_x.size());
#else
  std::cout << "calling query (scalar)" << std::endl;
  rtdqComputeClosestPointsdi(scene,
                             // no position:
# if 1
                             result_x.data(),
                             result_y.data(),
                             result_z.data(),
                             1,
# else
                             NULL,NULL,NULL,0,
# endif
                             // distance:
                             result_dist.data(),1,
                             // no prim IDs
                             NULL,0,
                             // input: the query points
                             point_x.data(),point_y.data(),point_z.data(),1,
                             // num points:
                             point_x.size());
#endif
  auto done_all = std::chrono::system_clock::now();
  
  size_t numPoints = point_x.size();
  size_t numTriangles = index_x.size();

  std::chrono::duration<double> buildTime = done_build - begin;
  std::chrono::duration<double> queryTime = done_all   - done_build;
  std::cout << "time to build tree " << (1000.f*buildTime.count()) << "ms" << std::endl;
  std::cout << "time to query " << numPoints << " points: " << queryTime.count() << "s" << std::endl;
  std::cout << "(this is " << (1000000.f * queryTime.count()/numPoints) << " micro seconds/query)" << std::endl;

  DBG(
  std::cout << "echcking ..." << std::endl;
  for (int i=0;i<point_x.size();i++) {
    if (result_dist[i] > reference_dist[i]+1e-7f) {
      std::cout << "fail for point " << i << "  : " << point_x[i] << " " << point_y[i] << " " << point_z[i]  << " " << std::endl;
      std::cout << "   we found  " << result_x[i] << " " << result_y[i] << " " << result_z[i] << " " << " dist " << result_dist[i] << std::endl;
      std::cout << "   reference " << reference_x[i]  << " " << reference_y[i] << " " << reference_z[i] << " " << " dist " << reference_dist[i] << std::endl;
    }


    if (result_dist[i] +1e-7f < reference_dist[i]) {
      std::cout << "***** CLOSER **** for point " << i << "  : " << point_x[i] << " " << point_y[i] << " " << point_z[i]  << " " << std::endl;
      std::cout << "   we found  " << result_x[i] << " " << result_y[i] << " " << result_z[i] << " " << " dist " << result_dist[i] << std::endl;
      std::cout << "   reference " << reference_x[i]  << " " << reference_y[i] << " " << reference_z[i] << " " << " dist " << reference_dist[i] << std::endl;
    }
  }
      )

  rtdqDestroy(scene);
}
