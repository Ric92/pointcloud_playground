#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0){
		return -1;
	}

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
    // On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. 
    if (argc > 2){
        ec.setClusterTolerance (atof(argv[2])); 
    }else{
        ec.setClusterTolerance (0.02); // 2cm
    }
    
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);

    auto tt0 = std::chrono::system_clock::now();
    ec.extract (cluster_indices);
    auto tt1 = std::chrono::system_clock::now();
    auto clusteringTime = std::chrono::duration_cast<std::chrono::milliseconds>(tt1-tt0).count();

    std::cout << "Time spend clustering: " << clusteringTime << " ms\n";

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "Cluster " << j << " has " << cloud_cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + std::to_string(j) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cloud_cluster);
        j++;
    }
    return -1;
}