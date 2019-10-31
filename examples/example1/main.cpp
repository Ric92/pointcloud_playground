//---------------------------------------------------------------------------------------------------------------------
//  POINTCLOUD_PLAYGROUND
//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <iostream>
#include <string>

#include <pointcloud_playground/Visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << argv[1] << "\n";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << argv[1] << std::endl;

  
    Visualizer<pcl::PointXYZRGB> vis;


    Eigen::Matrix4f pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    vis.drawPointCloud(cloud, "firstPC", pose);

    while(true){
        vis.spinOnce();
    }

    return 0;
}
