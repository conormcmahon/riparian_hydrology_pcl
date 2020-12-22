
#include <fstream>
#include <stdio.h>

#include <proj.h>

#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <dirt_or_leaf/las_point_types.h>

#include <lidar_raster_stats/point_cloud_raster.hpp>

struct StreamPoint
{
    pcl::PointXYZ coords;
    float order;
};

void load_csv_network(std::string filename, std::vector<StreamPoint>& points, bool strip_header=true);