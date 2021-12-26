# ======================================================================== #
# Copyright 2021-2021 Ingo Wald                                            #
#                                                                          #
# Licensed under the Apache License, Version 2.0 (the "License");          #
# you may not use this file except in compliance with the License.         #
# You may obtain a copy of the License at                                  #
#                                                                          #
#     http://www.apache.org/licenses/LICENSE-2.0                           #
#                                                                          #
# Unless required by applicable law or agreed to in writing, software      #
# distributed under the License is distributed on an "AS IS" BASIS,        #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. #
# See the License for the specific language governing permissions and      #
# limitations under the License.                                           #
# ======================================================================== #

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif()

option(BUILD_CUDA "Build CUDA Version" ON)

if (BUILD_CUDA)
  project(closestSurfacePointQueries VERSION 0.1.1 LANGUAGES C CXX CUDA)
  cmake_minimum_required(VERSION 2.17)
  find_package(CUDA)
  option(CMAKE_FORCE_CUDA_ARCHITECTURE "Choose a single CUDA architecture?" ON)
  set(CUDA_SEPARABLE_COMPILATION ON)
  enable_language(CUDA)
  if (CMAKE_FORCE_CUDA_ARCHITECTURE)
    # option 1: let USER choose the arch(s)
    set(CMAKE_FORCED_CUDA_ARCHITECTURE "86" CACHE STRING "Chosen arch to compile to?" )
    set(CMAKE_CUDA_ARCHITECTURES ${CMAKE_FORCED_CUDA_ARCHITECTURE})
    message(STATUS "just chosen CMAKE_CUDA_ARCHITECTURES = ${CMAKE_CUDA_ARCHITECTURES}")
  else()
    # option 2: choose default archs
    if(CUDA_VERSION VERSION_LESS 11)
      # 2a) old cuda, can only to up to 75
      set(CMAKE_CUDA_ARCHITECTURES 70 75)
    else()
      # 2b) new cuda, can also do ampere
      set(CMAKE_CUDA_ARCHITECTURES 70 75 86)
    endif()
  endif()
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.17)
    # nothing to do, setting CMAKE_CUDA_ARCHITECTURES is hte way to go
  else()
    foreach (arch IN ITEMS ${CMAKE_CUDA_ARCHITECTURES})
      #    message("status adding arch=${arch}")
      set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_${arch},code=sm_${arch}")
    endforeach()
  endif()
else()
  project(closestSurfacePointQueries VERSION 0.1.1 LANGUAGES C CXX)
endif()



include(cmake/configure_build_type.cmake)

add_library(bvhdq STATIC
  src/vec.h
  src/box.h
  src/bvh.h
  src/bvh.cpp
  )
if (BUILD_CUDA)
  set_target_properties(bvhdq PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON)
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.17)
    find_package(CUDAToolkit REQUIRED)
    target_link_libraries(bvhdq PUBLIC CUDA::cuda_driver CUDA::cudart_static)
  else()
    target_link_libraries(bvhdq PUBLIC cuda cudart)
    find_path(cuda_h_dir NAMES cuda.h HINTS /usr/local/cuda/include)
    target_include_directories(bvhdq PUBLIC ${cuda_h_dir})
  endif()
endif()




