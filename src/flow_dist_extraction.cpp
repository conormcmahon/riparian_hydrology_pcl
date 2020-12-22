
#include "riparian_hydrology_pcl/flow_dist_extraction.h"

struct stream_point
{
    pcl::PointXYZI point;
    
    float stream_dist_vert; 
    float stream_dist_horz;
    int stream_order;
};

void loadCSVNetwork(std::string filename, pcl::PointCloud<pcl::PointXYZI>::Ptr stream, int stream_order_threshold, bool strip_header)
{
    std::cout << "Loading file with name " << filename << std::endl;

    // Open CSV file containing stream network
    std::ifstream network_file(filename);
    if(!network_file.is_open())
        throw std::runtime_error("Failed to open network CSV file.");
    // Build Stream Points from file
    // Get column names
    std::string line, word;
    if(network_file.good())
    {
        // Optionally remove first line (headers)
        if(strip_header)
            std::getline(network_file, line);
    }
    else 
        std::runtime_error("Something is wrong with CSV input file.");
    // Extract stream data
    while(std::getline(network_file, line))
    {
        pcl::PointXYZI point;
        // Current line
        std::stringstream ss(line);
        // First column - point ID (skip)
        std::getline(ss, word, ',');
        // Second column - easting
        std::getline(ss, word, ',');
        point.x = std::stof(word);
        // Third column - northing
        std::getline(ss, word, ',');
        point.y = std::stof(word);
        // Fourth column - stream order  
        std::getline(ss, word, ',');
        point.intensity = std::stof(word);
        // Fifth Column - elevation
        std::getline(ss, word);
        point.z = std::stof(word); 
        if(point.intensity < stream_order_threshold)
            continue;
        stream->points.push_back(point);
    }
    network_file.close();
}

void streamDistances(pcl::PointCloud<pcl::PointXYZI>::Ptr stream, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree, stream_point& target)
{
    // Default Values
    target.stream_dist_vert = -999;
    target.stream_dist_horz = -999;
    target.stream_order = -999;

    if(target.point.z > 10e8)
        return;

    std::vector<int> nearest_indices;
    std::vector<float> nearest_dists;

    tree->nearestKSearch(target.point, 1, nearest_indices, nearest_dists);

    if(nearest_indices[0] > stream->points.size() | nearest_indices[0] < 1)
        return;

    target.stream_dist_vert = target.point.z - stream->points[nearest_indices[0]].z;
    target.stream_dist_horz = sqrt(pow(float(target.point.x - stream->points[nearest_indices[0]].x),2) + pow(float(target.point.y - stream->points[nearest_indices[0]].y),2));
    target.stream_order = stream->points[nearest_indices[0]].intensity;
}

int main(int argc, char *argv[])
{
    std::string stream_filename = argv[1];
    std::string dem_filename = argv[2];
    std::string output_filename = argv[3];
    int stream_order_threshold = atoi(argv[4]);

    
    // Load Stream CSV data
    pcl::PointCloud<pcl::PointXYZI>::Ptr stream(new pcl::PointCloud<pcl::PointXYZI>);
    loadCSVNetwork(stream_filename, stream, stream_order_threshold, true);
    std::cout << " Finished loading stream data! There are " << stream->points.size() << " points." << std::endl;


    // Load DEM from .tif using GDAL
    std::cout << "Loading DEM from file " << dem_filename << std::endl;
    GDALDataset  *demDataset;
    GDALAllRegister();
    demDataset = (GDALDataset *) GDALOpen(dem_filename.c_str(), GA_ReadOnly );
    if( demDataset == NULL )
    {
        std::cout << "ERROR: empty dataset!" << std::endl;
    }
    std::cout << "Successfully loaded file with size " << demDataset->GetRasterXSize() << " x " << demDataset->GetRasterYSize() << std::endl;
    double rasterGeoTransform[6];
    if( demDataset->GetGeoTransform( rasterGeoTransform ) == CE_None )
    {
        std::cout << "Origin = " << rasterGeoTransform[0] << ", " << rasterGeoTransform[3] << std::endl;
        std::cout << "Pixel Size = " << rasterGeoTransform[1] << ", " << rasterGeoTransform[5] << std::endl;
    }
    // Load DEM data from first (only) band
    GDALRasterBand  *demBand;
    demBand = demDataset->GetRasterBand( 1 );

    /*
    // Create Output File to hold new records
    //    Set up TIF driver, make sure it is supported
    const char *pszFormat = "GTiff";
    GDALDriver *poDriver;
    poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
    if( poDriver == NULL )
        exit(1);
    char **papszMetadata = poDriver->GetMetadata();
    if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATE, FALSE ) )
        printf( "Driver %s supports Create() method.\n", pszFormat );
    if( CSLFetchBoolean( papszMetadata, GDAL_DCAP_CREATECOPY, FALSE ) )
        printf( "Driver %s supports CreateCopy() method.\n", pszFormat );
    // Create new file
    //   Create Dataset
    GDALDataset *outputDataset;
    char **papszOptions = NULL;
    outputDataset = poDriver->Create( output_filename.c_str(), demDataset->GetRasterXSize(), demDataset->GetRasterYSize(), 1, GDT_Byte, papszOptions );
    //   Write Metadata
    OGRSpatialReference oSRS;
    char *pszSRS_WKT = NULL;
    GDALRasterBand *streamVertDist;
    GDALRasterBand *streamHorzDist;
    GDALRasterBand *streamOrder;
    //GByte [demDataset->GetRasterXSize()*demDataset->GetRasterYSize()];
    //poBand = poDataset->GetRasterBand( 1 );
    outputDataset->SetGeoTransform( rasterGeoTransform );
    oSRS.Set  */

    // Generate Search Tree
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    tree->setInputCloud(stream);

    // Set up CSV output file header
    std::ofstream output_file(output_filename);
    output_file << "x,y,z,dist_vert,dist_horz,order\n";

    // Set up reprojection from raster frame to stream frame 
    PJ_CONTEXT *C;
    PJ *P;
    PJ* P_for_GIS;
    PJ_COORD point_init, point_proj;
    C = proj_context_create();
    P = proj_create_crs_to_crs (C,
                                "EPSG:32611",
                                "EPSG:3500", 
                                NULL);

    // Get all search distances and heights
    float *pafScanline;
    int nXSize = demBand->GetXSize();
    for(int j=0; j<demBand->GetYSize(); j++) // iterate over Y
    {
        std::cout << " Working on line " << j << " out of " << demBand->GetYSize() << std::endl;
        pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize);
        demBand->RasterIO( GF_Read, 0, j, nXSize, 1,
                          pafScanline, nXSize, 1, GDT_Float32,
                          0, 0 );
        for(int i=0; i<demBand->GetXSize(); i++) // iterate over X
        {
            // NOTE for EPSG-defined projections, the order is latitude, longitude
            // initial point
            point_init = proj_coord(rasterGeoTransform[0] + i*rasterGeoTransform[1], 
                                    rasterGeoTransform[3] + j*rasterGeoTransform[5], 
                                    0, 0);
            // reprojected point in ESRI:103243
            point_proj = proj_trans(P, PJ_FWD, point_init);
            
            stream_point target;
            target.point.x = point_proj.enu.e;
            target.point.y = point_proj.enu.n;
            target.point.z = pafScanline[i];

            streamDistances(stream, tree, target);

            output_file << std::setprecision(8) << target.point.x << ",";
            output_file << std::setprecision(8) << target.point.y << ",";
            output_file << std::setprecision(5) << target.point.z << ",";
            output_file << target.stream_dist_vert << ",";
            output_file << target.stream_dist_horz << ",";
            output_file << target.stream_order;
            output_file << "\n";
        }
    }
    
    // Close Filestreams
    output_file.close();
    GDALClose(demDataset);

}