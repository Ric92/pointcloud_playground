#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent& event, void* viewer_void) {
    std::cout << "[INOF] Point picking event occurred." << std::endl;

    float x, y, z;
    if (event.getPointIndex() == -1) {
        return;
    }
    event.getPoint(x, y, z);
    std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z
              << ")" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Please provide as arguments \n 1-Path to rgb image \n 2-"
                  << std::endl;
        std::cout << "1-Path to rgb image" << std::endl;  // argv[1]
        std::cout << "2-Path to depth image" << std::endl;
        std::cout << "3-Path to pointcloud" << std::endl;
        return 0;
    }
    std::cout << "Color image path: " << argv[1] << std::endl;
    std::cout << "Depth image path: " << argv[2] << std::endl;
    cv::Mat rgb;
    rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   

    cv::Mat depth;
    depth = cv::imread(argv[2], CV_LOAD_IMAGE_ANYDEPTH);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(argv[3], *cloud);

     
    
    // visualize pointcloud
    // pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // viewer.addPointCloud(cloud, "cloud");
    // viewer.registerPointPickingCallback(pointPickingEventOccurred,
    //                                     (void*)&viewer);
    // viewer.spin();
    
    // visualize images
    cv::namedWindow( "rgb window");// Create a window for rgb.
    cv::namedWindow( "depth window");// Create a window for depth.
    while(true){
      cv::imshow( "depth window", depth );
      cv::imshow( "rgb window", rgb ); 
      cv::waitKey(1);
    }
    return 0;
}