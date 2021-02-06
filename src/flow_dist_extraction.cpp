
#include "riparian_hydrology_pcl/flow_dist_extraction.h"

int main(int argc, char *argv[])
{
    FlowlinesPCL<pcl::PointChannel> flowline_generator;

    OGRFlowlinesSettings settings;
    settings.flowlines_filename_ = "/mnt/d/serdp/data/pendleton/LiDAR/hydrology/flowlines/flowlines_santa_marg_lower.shp";
    settings.layer_name_ = "flowlines_santa_marg_lower";
    settings.shp_name_field_ = "GNIS_Nm";
    settings.shp_order_field_ = "StrmOrd";
    settings.filter_by_order_ = true;
    settings.filter_by_name_ = false;
    settings.channel_order_threshold_ = 3;
    settings.pcl_order_field_ = "stream_order";
    settings.linear_point_density_ = 1;

    // Generate XY values from Shapefile Flow Network
    flowline_generator.loadFromSHP(settings);

    // Generate Z values by interpolating DEM TIN
    LAS_TIN<pcl::PointChannel> TIN;
    std::string TIN_filename = "/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/62152035_ground_filtered.pcd";
    pcl::PointCloud<pcl::PointChannel>::Ptr DEM_cloud(new pcl::PointCloud<pcl::PointChannel>);
    // Load input cloud
    if (pcl::io::loadPCDFile<pcl::PointChannel> (TIN_filename, *DEM_cloud) == -1) //* load the file
    {
        std::cout << "Couldn't read file " << TIN_filename << std::endl;
    }
    else 
        std::cout << "Read an input cloud with size " << DEM_cloud->points.size() << std::endl;
    TIN.setInputCloud(DEM_cloud);
    TIN.generateTIN();
    flowline_generator.populateHeightFromTIN(TIN);


    flowline_generator.writeFlowlinesCloud("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/flowlines/flowlines_santa_margarita_lower.pcd");

} 