//---------------------------------------------------------------------------------------------------------------------
//  POINTCLOUD_PLAYGROUND
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


#ifndef POINTCLOUD_PLAYGROUND_VISUALIZER_H_
#define POINTCLOUD_PLAYGROUND_VISUALIZER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <thread>
#include <string>

#include <pcl/octree/octree_pointcloud_occupancy.h>

template <typename PointType_>
class Visualizer
{
public:
    Visualizer();

    void cleanAll()
    {
        if (!mViewer)
            return;

        mViewer->removeAllPointClouds();
        mViewer->removeAllCoordinateSystems();
        mViewer->removeAllShapes();
    }

    void drawPointCloud(typename pcl::PointCloud<PointType_>::Ptr &_pointCloud, std::string _name, Eigen::Matrix4f _pose);
    void updateCloud(std::string _name, Eigen::Matrix4f &_newPose);

    void pause();
    void spinOnce();
    void keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data);
    void mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *viewer_void);
    void pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void *viewer_void);
    std::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;

    typedef std::function<void(const pcl::visualization::KeyboardEvent &, void *)> CustomCallbackType;
    void addCustomKeyCallback(CustomCallbackType _callback);

private:
    float x, y, z;
    bool mPause = false;

    std::vector<CustomCallbackType> mCustomCallbacks;
    std::map<std::string, bool> mExistingCloud;
};

#include "Visualizer.inl"

#endif // SLAMMARKI_VISUALIZER_H_