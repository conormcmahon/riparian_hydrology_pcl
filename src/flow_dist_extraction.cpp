
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

    flowline_generator.loadFromSHP(settings);
    flowline_generator.writeFlowlinesCloud("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/flowlines/flowlines_santa_margarita_lower.pcd");

} 