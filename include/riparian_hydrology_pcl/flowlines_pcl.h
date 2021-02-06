
#ifndef FLOWLINES_PCL_
#define FLOWLINES_PCL_

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_spatialref.h>
#include <gdal/ogrsf_frmts.h>

#include <dirt_or_leaf/las_tin.hpp>
#include <dirt_or_leaf/runtime_field_selection.hpp>
#include <dirt_or_leaf/las_arithmetic.hpp>

struct OGRFlowlinesSettings
{
    // Shapefile Internal Structure
    std::string flowlines_filename_;
    std::string layer_name_;
    std::string shp_name_field_;
    std::string shp_order_field_;
    // SHP Filtration Settings
    float channel_order_threshold_;
    std::string channel_name_;
    bool filter_by_order_;
    bool filter_by_name_;
    // PCL Point Type Internal Structure
    std::string pcl_order_field_;
    std::string pcl_name_field_;
    // Sampling Information
    float linear_point_density_;
    // CRS Reprojection (optional)
    int EPSG_in_;
    int EPSG_reproj_;
};


template <typename ChannelPointType>
class FlowlinesPCL
{
public:
    typedef typename pcl::PointCloud<ChannelPointType> CC;
    typedef typename pcl::PointCloud<ChannelPointType>::Ptr CCP;
    typedef typename pcl::KdTreeFLANN<ChannelPointType> TreeC;
    typedef typename pcl::KdTreeFLANN<ChannelPointType>::Ptr TreeCP;

    FlowlinesPCL();

    void loadFromSHP(const OGRFlowlinesSettings &settings);
    void loadFromPCD(std::string filename, float distance_threshold=-999);
    void loadFromCloud(CCP cloud);
    void writeFlowlinesCloud(std::string filename, bool binary=true);
    TreeC getSearchTree();
    CC getFlowlinesCloudCopy();
    template <typename TINType>
    void populateHeightFromTIN(const TINType &TIN);
    template <typename TargetPointType>
    Eigen::Vector2f getPointDistance(TargetPointType target_point);

private:
    CCP flowlines_;
    TreeCP flowlines_search_tree_;

    int getFieldIndexByName(std::string field_name, OGRFeature &segment);
    void sampleOGRSegmentToCloud(const OGRFlowlinesSettings &settings, OGRFeature &segment, int shp_order_index=-1);
    ChannelPointType convertOGRPointToPCL(const OGRFlowlinesSettings &settings, OGRPoint &point, OGRFeature &segment, int shp_order_index);
    void fillLineSegmentPCLPoints(const OGRFlowlinesSettings &settings, ChannelPointType point_start, ChannelPointType point_end, int shp_order_index);

    bool checkChannelName(const OGRFlowlinesSettings &settings, OGRFeature &segment, int index);
    bool checkChannelOrder(const OGRFlowlinesSettings &settings, OGRFeature &segment, int index);
};

#endif //FLOWLINES_PCL_