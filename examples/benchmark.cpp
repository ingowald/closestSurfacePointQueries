#include "../src/distanceQueries.h"
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
std::vector<double> dennis_dist;

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

void loadDennis(const std::string &fileName)
{
  FILE *file = fopen(fileName.c_str(),"r");
  if (!file) throw std::runtime_error("could not open "+fileName);

  for (int i=0;i<point_x.size();i++) {
    double s;
    if (fscanf(file,"%lf\n",&s) != 1)
      throw std::runtime_error("could not read dennis result from "+fileName);
    dennis_dist.push_back(s);
  }
  std::cout << "loaded " << dennis_dist.size() << " reference distances" << std::endl;
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

  bool couldLoadReference = true;
  try {
//     loadReference(dir+"/dennis.dist.txt");
    loadReference(dir+"/tim.xyzs.txt");
    loadDennis(dir+"/dennis.dist.txt");
  } catch (std::runtime_error e) {
    couldLoadReference = false;
    printf("********************** could NOT load reference results **********************\n");
  }

  
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

  std::cout << "calling query (scalar)" << std::endl;
  rtdqComputeClosestPointsdi(scene,
                             // closest point:
                             result_x.data(),
                             result_y.data(),
                             result_z.data(),
                             1,
                             // distance:
                             result_dist.data(),1,
                             // no prim IDs
                             NULL,0,
                             // input: the query points
                             point_x.data(),point_y.data(),point_z.data(),1,
                             // num points:
                             point_x.size());
  auto done_all = std::chrono::system_clock::now();
  
  size_t numPoints = point_x.size();
  size_t numTriangles = index_x.size();

  std::chrono::duration<double> buildTime = done_build - begin;
  std::chrono::duration<double> queryTime = done_all   - done_build;
  std::cout << "time to build tree " << (1000.f*buildTime.count()) << "ms" << std::endl;
  std::cout << "time to query " << numPoints << " points: " << queryTime.count() << "s" << std::endl;
  std::cout << "(this is " << (1000000.f * queryTime.count()/numPoints) << " micro seconds/query)" << std::endl;

  if (couldLoadReference) {
  double most_further_tim_mag = 0.f;
  double most_further_tim_myDist;
  double most_further_tim_hisDist;
  double most_further_tim_input[3];
  double most_further_tim_result[3];

  double most_closer_tim_mag = 0.f;
  double most_closer_tim_myDist;
  double most_closer_tim_hisDist;
  double most_closer_tim_input[3];
  double most_closer_tim_result[3];


  double most_further_dennis_mag = 0.f;
  double most_further_dennis_myDist;
  double most_further_dennis_hisDist;
  double most_further_dennis_input[3];
  double most_further_dennis_result[3];

  double most_closer_dennis_mag = 0.f;
  double most_closer_dennis_myDist;
  double most_closer_dennis_hisDist;
  double most_closer_dennis_input[3];
  double most_closer_dennis_result[3];


  std::cout << "echcking ..." << std::endl;
  for (int i=0;i<point_x.size();i++) {

    printf("results for %8i: mine %18.16f    tim %18.16f    dennis %18.16f\n",i,result_dist[i],reference_dist[i],dennis_dist[i]);
    

    float closer_dennis = dennis_dist[i] - result_dist[i];
    if (closer_dennis > most_closer_dennis_mag) {
      most_closer_dennis_mag = closer_dennis;
      most_closer_dennis_input[0] = point_x[i];
      most_closer_dennis_input[1] = point_y[i];
      most_closer_dennis_input[2] = point_z[i];
      most_closer_dennis_result[0] = result_x[i];
      most_closer_dennis_result[1] = result_y[i];
      most_closer_dennis_result[2] = result_z[i];
      most_closer_dennis_myDist    = result_dist[i];
      most_closer_dennis_hisDist    = dennis_dist[i];
    }

    float further_dennis = result_dist[i] - dennis_dist[i];
    if (further_dennis > most_further_dennis_mag) {
      most_further_dennis_mag = further_dennis;
      most_further_dennis_input[0] = point_x[i];
      most_further_dennis_input[1] = point_y[i];
      most_further_dennis_input[2] = point_z[i];
      most_further_dennis_result[0] = result_x[i];
      most_further_dennis_result[1] = result_y[i];
      most_further_dennis_result[2] = result_z[i];
      most_further_dennis_myDist    = result_dist[i];
      most_further_dennis_hisDist    = dennis_dist[i];
    }

    float closer_tim = reference_dist[i] - result_dist[i];
    if (closer_tim > most_closer_tim_mag) {
      most_closer_tim_mag = closer_tim;
      most_closer_tim_input[0] = point_x[i];
      most_closer_tim_input[1] = point_y[i];
      most_closer_tim_input[2] = point_z[i];
      most_closer_tim_result[0] = result_x[i];
      most_closer_tim_result[1] = result_y[i];
      most_closer_tim_result[2] = result_z[i];
      most_closer_tim_myDist    = result_dist[i];
      most_closer_tim_hisDist    = reference_dist[i];
    }

    float further_tim = result_dist[i] - reference_dist[i];
    if (further_tim > most_further_tim_mag) {
      most_further_tim_mag = further_tim;
      most_further_tim_input[0] = point_x[i];
      most_further_tim_input[1] = point_y[i];
      most_further_tim_input[2] = point_z[i];
      most_further_tim_result[0] = result_x[i];
      most_further_tim_result[1] = result_y[i];
      most_further_tim_result[2] = result_z[i];
      most_further_tim_myDist    = result_dist[i];
      most_further_tim_hisDist    = reference_dist[i];
    }


  }
  printf("biggest dist (to tim) where we're closer: input %g %g %g ours %g %g %g dist %g his-dist %g\n",
         most_closer_tim_input[0],
         most_closer_tim_input[1],
         most_closer_tim_input[2],
         most_closer_tim_result[0],
         most_closer_tim_result[1],
         most_closer_tim_result[2],
         most_closer_tim_myDist,
         most_closer_tim_hisDist);
  printf("biggest dist (to tim) where we're further: input %g %g %g ours %g %g %g dist %g his-dist %g\n",
         most_further_tim_input[0],
         most_further_tim_input[1],
         most_further_tim_input[2],
         most_further_tim_result[0],
         most_further_tim_result[1],
         most_further_tim_result[2],
         most_further_tim_myDist,
         most_further_tim_hisDist);
         
  printf("biggest dist (to dennis) where we're closer: input %g %g %g ours %g %g %g dist %g his-dist %g\n",
         most_closer_dennis_input[0],
         most_closer_dennis_input[1],
         most_closer_dennis_input[2],
         most_closer_dennis_result[0],
         most_closer_dennis_result[1],
         most_closer_dennis_result[2],
         most_closer_dennis_myDist,
         most_closer_dennis_hisDist);
  printf("biggest dist (to dennis) where we're further: input %g %g %g ours %g %g %g dist %g his-dist %g\n",
         most_further_dennis_input[0],
         most_further_dennis_input[1],
         most_further_dennis_input[2],
         most_further_dennis_result[0],
         most_further_dennis_result[1],
         most_further_dennis_result[2],
         most_further_dennis_myDist,
         most_further_dennis_hisDist);
         
  }
  rtdqDestroy(scene);
}
