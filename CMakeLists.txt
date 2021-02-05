cmake_minimum_required(VERSION 2.8.3)

project(riparian_hydrology_pcl)

include_directories(include "/usr/local")

# Point Cloud Library (PCL)
find_package(PCL 1.09 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Computational Geometry Algorithms Library (CGAL)
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL QUIET COMPONENTS)
if(NOT CGAL_FOUND)
  message(STATUS "This project requires CGAL and will not be compiled.")
  return()
endif()

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

# PROJ 
find_package(GDAL REQUIRED)



add_executable(flow_dist_extraction src/flow_dist_extraction.cpp)
target_link_libraries(flow_dist_extraction ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})



install(TARGETS flow_dist_extraction
        DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")