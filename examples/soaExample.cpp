// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../src/distanceQueries.h"
#include "../src/vec.h"

#include <vector>
#include <chrono>

namespace embree {

  typedef ospcommon::vec3f vec3f;
  typedef ospcommon::vec3i vec3i;
  
  extern "C" int main(int ac, char **av)
  {
    srand48(290374);

    int numTriangles = 1000000;
    float maxEdgeLen = 1.f/(powf(numTriangles,1.f/3.f));
    int numPoints    = 1000000;
    
    // the triangles we're querying to
    std::vector<float>   vertex_x;
    std::vector<float>   vertex_y;
    std::vector<float>   vertex_z;
    std::vector<int32_t> index_x;
    std::vector<int32_t> index_y;
    std::vector<int32_t> index_z;
    
    for (int i=0;i<numTriangles;i++) {
      const vec3f P(drand48(),drand48(),drand48());
      const vec3f e0 = maxEdgeLen * vec3f(drand48(),drand48(),drand48());
      const vec3f e1 = maxEdgeLen * vec3f(drand48(),drand48(),drand48());
      const int vIdx = vertex_x.size();
      vertex_x.push_back(P.x);
      vertex_x.push_back(P.x+e0.x);
      vertex_x.push_back(P.x+e1.x);

      vertex_y.push_back(P.y);
      vertex_y.push_back(P.y+e0.y);
      vertex_y.push_back(P.y+e1.y);

      vertex_z.push_back(P.z);
      vertex_z.push_back(P.z+e0.z);
      vertex_z.push_back(P.z+e1.z);

      index_x.push_back(vIdx+0);
      index_y.push_back(vIdx+1);
      index_z.push_back(vIdx+2);
    }
    
    // the points we're query'ing
    std::vector<vec3f> queryPoint;
    for (int i=0;i<numPoints;i++) {
      int triID = int(drand48()*numTriangles);
      float u = drand48();
      float v = drand48();
      const vec3f A(vertex_x[index_x[triID]],
                    vertex_y[index_x[triID]],
                    vertex_z[index_x[triID]]);
      const vec3f B(vertex_x[index_y[triID]],
                    vertex_y[index_y[triID]],
                    vertex_z[index_y[triID]]);
      const vec3f C(vertex_x[index_z[triID]],
                    vertex_y[index_z[triID]],
                    vertex_z[index_z[triID]]);
      if (u+v >= 1.f) { u = 1-u; v = 1-v; }
      const vec3f pointOnTri = A+u*(B-A)+v*(C-A);
      queryPoint.push_back(pointOnTri + float(0.1f*drand48()*maxEdgeLen)*vec3f(drand48(),drand48(),drand48()));
    }

    // the result of our queries
    std::vector<vec3f>   result_pos;    result_pos.resize(queryPoint.size());
    std::vector<int32_t> result_primID; result_primID.resize(queryPoint.size());
    std::vector<float>   result_dist;   result_dist.resize(queryPoint.size());

    auto begin = std::chrono::system_clock::now();
    // create the actual scene:
    distance_query_scene scene
      = rtdqNewTriangleMeshfi(&vertex_x[0],&vertex_y[0],&vertex_z[0],1,
                              &index_x[0],&index_y[0],&index_z[0],1,
                              index_x.size());
    auto done_build = std::chrono::system_clock::now();

    // perform the queries - all together, in a single thread
    rtdqComputeClosestPointsfi(scene,
                             &result_pos[0].x,&result_pos[0].y,&result_pos[0].z,3,
                             &result_dist[0],1,
                             &result_primID[0],1,
                             &queryPoint[0].x,&queryPoint[0].y,&queryPoint[0].z,3,
                             numPoints);
    auto done_all = std::chrono::system_clock::now();

    std::chrono::duration<double> buildTime = done_build - begin;
    std::chrono::duration<double> queryTime = done_all   - done_build;
    std::cout << "time to build tree " << buildTime.count() << "s" << std::endl;
    std::cout << "time to query " << numPoints << " points: " << queryTime.count() << "s" << std::endl;
    std::cout << "(this is " << (queryTime.count()/numPoints) << " seconds/prim)" << std::endl;

    rtdqDestroy(scene);
  }

}
