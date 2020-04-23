#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <iostream>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
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

    std::cout << "max x: " << max_x << " max y: " << max_y << " max z: " << max_z << std::endl; 

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, max_z - (max_z - min_z)/2.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);


    std::cout << "Cloud filtered has " << cloud_filtered->points.size() << " points." << std::endl;
    std::string fileName = "cloud_filtered.pcd";
    pcl::io::savePCDFileASCII(fileName, *cloud_filtered);
    
    return -1;
}