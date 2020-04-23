#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "[INFO] Point picking event occurred." << std::endl;

  float x, y, z;
  if (event.getPointIndex () == -1)
  {
     return;
  }
  event.getPoint(x, y, z);
  std::cout << "[INFO] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}
    
int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr body (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (argv[1], *body);
    viewer.addPointCloud (body,"body");
    viewer.registerPointPickingCallback (pointPickingEventOccurred, (void*)&viewer);
    viewer.spin();
    return 0;
}