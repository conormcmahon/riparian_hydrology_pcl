
#include "riparian_hydrology_pcl/flowlines_pcl.hpp"
#include "riparian_hydrology_pcl/point_channel.h"
#include <dirt_or_leaf/point_2d_ground.h>
#include <dirt_or_leaf/las_tin.h>

int main(int argc, char *argv[])
{
    int num_expected_arguments = 8;
    if(argc != num_expected_arguments)
    {
        std::cout << "ERROR: Unexpected number of arguments." << std::endl;
        return -1;
    }
    std::string flowlines_filename_input = argv[1];
    std::string TIN_filename = argv[2];
    std::string flowlines_filename_output = argv[3];
    int EPSG_in = atoi(argv[4]);
    int EPSG_out = atoi(argv[5]);
    float z_scale = atof(argv[6]);
    float z_score_filter = atof(argv[7]);

    // Load DEM cloud
    pcl::PointCloud<pcl::Point2DGround>::Ptr ground(new pcl::PointCloud<pcl::Point2DGround>);   
    if (pcl::io::loadPCDFile<pcl::Point2DGround> (TIN_filename, *ground) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << TIN_filename << std::endl;
        return -1;
    }
    else 
        std::cout << "Read an input cloud with size " << ground->points.size() << std::endl;
    // For my test case, I need to reproject and scale the TIN cloud:
    if(EPSG_in != EPSG_out)
    {
        OGRSpatialReference in_SRS;
        in_SRS.importFromEPSG(EPSG_in);
        OGRSpatialReference out_SRS;
        out_SRS.importFromEPSG(EPSG_out);
        OGRCoordinateTransformation *transformer = OGRCreateCoordinateTransformation(&in_SRS, &out_SRS);
        
        for(int i=0; i<ground->points.size(); i++)
        {
            // Get reprojected XYZ coordinates
            double x = ground->points[i].x;
            double y = ground->points[i].y;
            double z = ground->points[i].z;
            transformer->Transform(1, &x, &y, &z);
            ground->points[i].x = x;
            ground->points[i].y = y;
            ground->points[i].z = z*z_scale;
        }
    }
    // Build TIN out of ground points
    LAS_TIN<pcl::Point2DGround> TIN;
    TIN.setInputCloud(ground);
    TIN.generateTIN();
    std::cout << "  Setting up a tree for ground cloud..." << std::endl;
    pcl::KdTreeFLANN<pcl::Point2DGround>::Ptr ground_tree(new pcl::KdTreeFLANN<pcl::Point2DGround>);
    ground_tree->setInputCloud(ground);
    std::cout << "  Tree Build." << std::endl;
    pcl::Point2DGround template_point;

    // Extract Flowlines Data and FIlter (to include only points interpolated by TIN)
    FlowlinesPCL<pcl::PointChannel> flowlines;
    flowlines.loadFromPCD(flowlines_filename_input, template_point, ground, ground_tree, 100);
    flowlines.populateHeightFromTIN(TIN);
    pcl::PointCloud<pcl::PointChannel>::Ptr new_channels(new pcl::PointCloud<pcl::PointChannel>);
    flowlines.getFlowlinesCloudCopy(new_channels);
    pcl::PointCloud<pcl::PointChannel>::Ptr new_channels_subset(new pcl::PointCloud<pcl::PointChannel>);
    float mean_height = 0;
    for(int i=0; i<new_channels->points.size(); i++)
        if(new_channels->points[i].z != flowlines.default_stream_height_)
        {
            new_channels_subset->points.push_back(new_channels->points[i]);
            mean_height += new_channels->points[i].z;
        }
    mean_height /= new_channels->points.size();
    float std_dev = 0;
    for(int i=0; i<new_channels_subset->points.size(); i++)
        std_dev += pow(new_channels->points[i].z - mean_height,2);
    std_dev = sqrt(std_dev/new_channels->points.size());
    new_channels_subset->height = 1;
    new_channels_subset->width = new_channels_subset->points.size();
    std::cout << "After filtering to include only interpolated points, flowlines cloud has a total of " << new_channels_subset->points.size() << " points, down from " << new_channels->points.size() << " points." << std::endl;

    // Load input cloud
    pcl::PointCloud<pcl::PointChannel>::Ptr old_channels(new pcl::PointCloud<pcl::PointChannel>);
    if (pcl::io::loadPCDFile<pcl::PointChannel> (flowlines_filename_output, *old_channels) == -1) //* load the file
    {
        std::cout << "File " << flowlines_filename_output << " doesn't exist yet. Continuing with an empty existing file." << std::endl;
    }
    else 
        std::cout << "Read an input cloud with size " << old_channels->points.size() << std::endl;
    
    // If new points are NOT the same as existing points, add them to the total cloud
    pcl::KdTreeFLANN<pcl::PointChannel> tree;
    pcl::PointCloud<pcl::PointChannel>::Ptr all_channels(new pcl::PointCloud<pcl::PointChannel>);
    if(old_channels->points.size() > 0)
    {
        *all_channels = *old_channels;
        tree.setInputCloud(old_channels);
        for(int i=0; i<new_channels_subset->points.size(); i++)
        {
            // Skip points which seem like they're incorrectly interpolated (statistical outliers)
            if(fabs(new_channels_subset->points[i].z - mean_height) > z_score_filter*std_dev)
                continue;
            std::vector<int> nearest_ind;
            std::vector<float> dist_squared;
            tree.nearestKSearch(new_channels_subset->points[i], 1, nearest_ind, dist_squared);
            // if points are not BOTH co-located and with the same order, we retain them 
            if(pcl::pointDistance2D(new_channels_subset->points[i], old_channels->points[nearest_ind[0]]) != 0 ||
               new_channels_subset->points[i].stream_order != old_channels->points[nearest_ind[0]].stream_order) 
                all_channels->push_back(new_channels_subset->points[i]);
        }
    }
    else
        for(int i=0; i<new_channels_subset->points.size(); i++)
        {
            // Skip points which seem like they're incorrectly interpolated (statistical outliers)
            if(fabs(new_channels_subset->points[i].z - mean_height) > z_score_filter*std_dev)
                continue;
            all_channels->push_back(new_channels_subset->points[i]);
        }

    pcl::PCDWriter writer;
    writer.write<pcl::PointChannel>(flowlines_filename_output, *all_channels, true);
    std::cout << "After filtering, added " << all_channels->points.size()-old_channels->points.size() << " points to the cloud, which now has " << all_channels->points.size() << " points in total." << std::endl;
}