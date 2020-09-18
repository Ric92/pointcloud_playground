#include <iostream>
#include <chrono>

#include <pcl/io/pcd_io.h>

// The GPU specific stuff here

#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>


int main(int argc, char** argv) {

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	
    // Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
    // Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0){
		return -1;
	}
    
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(cloud->points);
    
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    auto tt0 = std::chrono::system_clock::now();
    // If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    // On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. 
    if (argc > 2){
        gec.setClusterTolerance (atof(argv[2])); 
    }else{
        gec.setClusterTolerance (0.02); // 2cm
    }
    gec.setMinClusterSize (500);
    gec.setMaxClusterSize (10000);
    gec.setSearchMethod (octree_device);
    gec.setHostCloud( cloud);
    gec.extract (cluster_indices_gpu);

    auto tt1 = std::chrono::system_clock::now();
    auto clusteringTime = std::chrono::duration_cast<std::chrono::milliseconds>(tt1-tt0).count();
    std::cout << "Time spend clustering GPU: " << clusteringTime << " ms\n";
    // std::cout << "INFO: stopped with the GPU version" << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster_gpu->points.push_back (cloud->points[*pit]); //*
        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

        std::cout << "Cluster " << j << " has " << cloud_cluster_gpu->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + std::to_string(j) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cloud_cluster_gpu);
        j++;
    }

    return -1;
}