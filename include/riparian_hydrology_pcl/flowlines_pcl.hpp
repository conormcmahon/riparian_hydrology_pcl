
#ifndef FLOWLINES_PCL_HPP_
#define FLOWLINES_PCL_HPP_

#include "riparian_hydrology_pcl/flowlines_pcl.h"

template <typename ChannelPointType>
FlowlinesPCL<ChannelPointType>::FlowlinesPCL()
{
    flowlines_.reset(new CC);
    flowlines_search_tree_.reset(new TreeC);
    GDALAllRegister();
}


template <typename ChannelPointType>
void FlowlinesPCL<ChannelPointType>::loadFromSHP(const OGRFlowlinesSettings &settings)
{
    std::cout << "Attempting to read channel network from .shpfile at " << settings.flowlines_filename_ << std::endl;
    // Load Dataset from .shpfile
    GDALDataset *flowlines_dataset;
    OGRLayer *flowlines_layer;
    OGRFeature *segment;
    OGRField *flowline_field;
    flowlines_dataset = (GDALDataset*) GDALOpenEx(settings.flowlines_filename_.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(flowlines_dataset == NULL)
    {
        printf("ERROR: GDAL Shapefile Open Failed.");
        exit(-1);
    }
    else 
        std::cout << "  Successfully loaded channel network from .shpfile" << std::endl;
    // Get Flowlines Layer
    flowlines_layer = flowlines_dataset->GetLayerByName(settings.layer_name_.c_str()); 
    // Count and print number of segments in the network
    int num_segments = 0;
    for( auto& flowline_segment_counter: flowlines_layer )
    {
        num_segments++;
    }  
    flowlines_layer->ResetReading();
    std::cout << "  Input channel network contains " << num_segments << " total segments." << std::endl;

    // Get first stream segment (just to have attribute names)
    segment = flowlines_layer->GetNextFeature();
    flowlines_layer->ResetReading();
    // Count and print number of attributes 
    int num_attributes = 0;
    for( auto&& flowline_field: *segment )
    { 
        num_attributes++;
    }   
    std::cout << "  Input channel network contains " << num_attributes << " attributes in each segment." << std::endl;
    
    int shp_name_index = getFieldIndexByName(settings.shp_name_field_, *segment);
    int shp_order_index = getFieldIndexByName(settings.shp_order_field_, *segment);

    for( auto& current_segment: flowlines_layer )
    {
        // Skip features which don't match user-specified attribute filters
        if(settings.filter_by_name_ && shp_name_index > -1)
            if(!checkChannelName(settings, *current_segment, shp_name_index))
                continue;
        if(settings.filter_by_order_ && shp_order_index > -1)
            if(!checkChannelOrder(settings, *current_segment, shp_order_index))
                continue;
        sampleOGRSegmentToCloud(settings, *current_segment, shp_order_index);
    }
    flowlines_->height = 1;
    flowlines_->width = flowlines_->points.size();
    std::cout << "Finished generating a PCL flowline point cloud from input OGR vector format. " << std::endl; 
    flowlines_search_tree_->setInputCloud(flowlines_);
    std::cout << "  Finished generating a KD search tree on flowlines cloud." << std::endl; 
}


template <typename ChannelPointType>
void FlowlinesPCL<ChannelPointType>::sampleOGRSegmentToCloud(const OGRFlowlinesSettings &settings, OGRFeature &segment, int shp_order_index)
{
    OGRGeometry *input_geometry;
    input_geometry = segment.GetGeometryRef(); 

    // Reformat Geometry and get Point Count 
    OGRLineString *input_geometry_string = ( OGRLineString * )input_geometry;
    int number_of_vertices = input_geometry_string ->getNumPoints();
    std::cout << "Beginning to process a segment with " << number_of_vertices << " points." << std::endl;
    // Iterate over all points in segment, generate new data structures in memory
    std::vector<ChannelPointType> point_list(number_of_vertices);
    for (int i=0; i<number_of_vertices; i++)
    {
        OGRPoint ptTemp;
        input_geometry_string->getPoint(i, &ptTemp);
        point_list[i] = convertOGRPointToPCL(settings, ptTemp, segment, shp_order_index);
        flowlines_->push_back(point_list[i]);
    }
    for(int i=1; i<number_of_vertices; i++)
    {
        fillLineSegmentPCLPoints(settings, point_list[i-1], point_list[i], shp_order_index);
    }
}

template <typename ChannelPointType>
ChannelPointType FlowlinesPCL<ChannelPointType>::convertOGRPointToPCL(const OGRFlowlinesSettings &settings, OGRPoint &point_ogr, OGRFeature &segment, int shp_order_index)
{
    ChannelPointType point_pcl;
    // X and Y values come from vector format
    point_pcl.x = point_ogr.getX();
    point_pcl.y = point_ogr.getY();
    point_pcl.z = 0;
    // If the point type includes fields for it, and if we have the field names generated, fill in the stream order and stream name fields        
    if(shp_order_index > -1)
        if(pcl::checkFieldType<ChannelPointType, int>(settings.pcl_order_field_))
            pcl::assignValueToField( point_pcl, 
                                     settings.pcl_order_field_, 
                                     ((segment)[shp_order_index]).GetInteger() );
    return point_pcl;
}

template <typename ChannelPointType> 
void FlowlinesPCL<ChannelPointType>::fillLineSegmentPCLPoints(const OGRFlowlinesSettings &settings, ChannelPointType point_start, ChannelPointType point_end, int shp_order_index)
{
    float horz_dist = pcl::pointDistance2D(point_start, point_end);
    int required_subpoints = floor(horz_dist / settings.linear_point_density_);

    // Interpolate points 
    for(int i=0; i<required_subpoints; i++)
    {
        ChannelPointType intermediate_point = point_start;  // this will copy all other fields, e.g. including segment stream order and channel name
        intermediate_point.x = (float(i)/required_subpoints) * point_start.x + (1 - float(i)/required_subpoints) * point_end.x;
        intermediate_point.y = (float(i)/required_subpoints) * point_start.y + (1 - float(i)/required_subpoints) * point_end.y;
        intermediate_point.z = (float(i)/required_subpoints) * point_start.z + (1 - float(i)/required_subpoints) * point_end.z;
        flowlines_->points.push_back(intermediate_point);
    }
}


template <typename ChannelPointType>
void FlowlinesPCL<ChannelPointType>::writeFlowlinesCloud(std::string filename, bool binary)
{
    if(flowlines_->points.size() < 1)
    {
        std::cout << "WARNING: asked to save an empty flowlines cloud. No cloud will be saved." << std::endl;
        return;
    }
    std::cout << "Saving a flowlines cloud with " << flowlines_->points.size() << " points to the file " << filename << std::endl;
    pcl::PCDWriter writer;
    writer.write<ChannelPointType>(filename, *flowlines_, binary);
    std::cout << "  File saved." << std::endl;
}



template <typename ChannelPointType>
bool FlowlinesPCL<ChannelPointType>::checkChannelName(const OGRFlowlinesSettings &settings, OGRFeature &segment, int index)
{
    if( settings.channel_name_.compare(((segment)[index]).GetAsString()) == 0 )
        return true;
    return false;
}
template <typename ChannelPointType>
bool FlowlinesPCL<ChannelPointType>::checkChannelOrder(const OGRFlowlinesSettings &settings, OGRFeature &segment, int index)
{
    if( ((segment)[index]).GetInteger() >= settings.channel_order_threshold_ )
        return true;
    return false;
}

// Search through OGR Shapefile architecture for an attribute with name FIELD_NAME
//   Returns the index of the attribute if found
//   Otherwise, returns -1
template <typename ChannelPointType>
int FlowlinesPCL<ChannelPointType>::getFieldIndexByName(std::string field_name, OGRFeature &segment)
{
    int target_index = -1;
    std::cout << "Searching for Shapefile field index for channel name field " << field_name << std::endl;
    
    int current_index = 0;
    for( auto&& flowline_field: segment )
    { 
        // NOTE str::compare() returns 0 if the strings are equal. Disgusting, I know
        if(field_name.compare(flowline_field.GetName()) == 0)
        {
            target_index = current_index;
            break;
        }
        current_index++;
    }   
    if(target_index == -1)
        std::cout << "  WARNING: requested attribute (" << field_name << ") was not found." << std::endl;
    else   
        std::cout << "  Found requested attribute at index " << target_index << std::endl;
    return target_index;
}



#endif //FLOWLINES_PCL_HPP_