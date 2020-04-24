#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

int main(int argc, char** argv) {
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0){
		return -1;
	}

    float max_z,max_y,max_x;
    float min_z,min_y,min_x;
    min_z = min_y = min_x = 1000.0;
    max_z = max_y = max_x = 0.0;
    for (std::size_t i = 0; i < cloud->points.size (); ++i){
        
        min_x = (cloud->points[i].x < min_x) ? cloud->points[i].x : min_x; 
        min_y = (cloud->points[i].y < min_y) ? cloud->points[i].y : min_y; 
        min_z = (cloud->points[i].z < min_z) ? cloud->points[i].z : min_z; 

        max_x = (cloud->points[i].x > max_x) ? cloud->points[i].x : max_x; 
        max_y = (cloud->points[i].y > max_y) ? cloud->points[i].y : max_y; 
        max_z = (cloud->points[i].z > max_z) ? cloud->points[i].z : max_z; 
    }
    
    // std::cout << "inc x: " << max_x - min_x << " inc y: " << max_y - min_y << " inc z: " << max_z - min_z << std::endl; 
    // std::vector<float> incs = {max_x - min_x,max_x - min_x,max_x - min_x};
    // int maxAxis = std::max_element(incs.begin(),incs.end()) - incs.begin();
    
    std::string filterAxis = "z";
    float filterMaxLimit = max_z - (max_z - min_z)/2.0; 

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);

    // Cut in Z axis between 0.0 and max - inc/2
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, filterMaxLimit);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);


    std::cout << "Cloud filtered has " << cloud_filtered->points.size() << " points." << std::endl;
    
    // Clean pointcloud
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_out, indices);
    std::cout << "size: " << cloud_out->points.size () << std::endl;

    std::string fileName = "cloud_filtered.pcd";
    pcl::io::savePCDFileASCII(fileName, *cloud_filtered);
    
    return -1;
}