#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/min_cut_segmentation.h>
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

	// Min-cut clustering object.
	pcl::MinCutSegmentation<pcl::PointXYZ> clustering;
	clustering.setInputCloud(cloud);
	// Create a cloud that lists all the points that we know belong to the object
	// (foreground points). We should set here the object's center.
	pcl::PointCloud<pcl::PointXYZ>::Ptr foregroundPoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointXYZ point;
	point.x = atof(argv[2]);
	point.y = atof(argv[3]);
	point.z = atof(argv[4]);
	foregroundPoints->points.push_back(point);
	clustering.setForegroundPoints(foregroundPoints);
	// Set sigma, which affects the smooth cost calculation. It should be
	// set depending on the spacing between points in the cloud (resolution).
	clustering.setSigma(0.05);
	// Set the radius of the object we are looking for.
	clustering.setRadius(0.4);
	// Set the number of neighbors to look for. Increasing this also increases
	// the number of edges the graph will have.
	clustering.setNumberOfNeighbours(1);
	// Set the foreground penalty. It is the weight of the edges
	// that connect clouds points with the source vertex.
	clustering.setSourceWeight(0.4);

	std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

	std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;

	// For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + std::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}