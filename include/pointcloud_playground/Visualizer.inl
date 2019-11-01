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


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline Visualizer<PointType_>::Visualizer()
{
    mViewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
    mViewer->setBackgroundColor(100, 100, 100, 0);
    mViewer->addCoordinateSystem(0.05, "base", 0);
    mViewer->addCoordinateSystem(0.02, "current_pose", 0);
    mViewer->registerKeyboardCallback(&Visualizer::keycallback, *this, (void *)&mViewer);
    mViewer->registerMouseCallback(&Visualizer::mouseEventOccurred, *this, (void *)&mViewer);
    mViewer->registerPointPickingCallback(&Visualizer::pointPickedCallback, *this, (void *)&mViewer);
    mViewer->setCameraPosition(1.59696, 0.285761, -3.40482, -0.084178, -0.989503, -0.117468);
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::drawPointCloud(typename pcl::PointCloud<PointType_>::Ptr &_pointCloud, std::string _name, Eigen::Matrix4f _pose)
{
    if (!mViewer)
    {
        return;
    }

    if (mExistingCloud.find(_name) != mExistingCloud.end())
    {
        mViewer->removeCoordinateSystem(_name);
        mViewer->removeText3D(_name);
        mViewer->removePointCloud(_name);
    }

    //mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_pose), _name);
    //pcl::PointXYZ position(_pose(0, 3), _pose(1, 3), _pose(2, 3));

    //mViewer->addText3D(_name, position, 0.015, 1, 0, 0, _name);
    mViewer->addPointCloud<PointType_>(_pointCloud, _name);
    mViewer->updatePointCloudPose(_name, Eigen::Affine3f(_pose));

    //mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, _name);
    //mViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0,_name);

    mExistingCloud[_name] = true;
    mViewer->spinOnce(10, true);
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::updateCloud(std::string _name, Eigen::Matrix4f &_newPose)
{
    if (!mViewer)
        return;

    if (mViewer->contains(_name))
    {
        mViewer->removeCoordinateSystem(_name);
        mViewer->addCoordinateSystem(0.03, Eigen::Affine3f(_newPose), _name);

        mViewer->removeText3D(_name);
        pcl::PointXYZ position(_newPose(0, 3), _newPose(1, 3), _newPose(2, 3));
        mViewer->addText3D(_name, position, 0.015, 1, 0, 0, _name);

        mViewer->updatePointCloudPose(_name, Eigen::Affine3f(_newPose));
    }
}

// //---------------------------------------------------------------------------------------------------------------------
// template <typename PointType_>
// inline bool Visualizer<PointType_>::updateCurrentPose(Eigen::Matrix4f &_pose){
//     if(!mViewer)
//         return false;

//     mViewer->removeCoordinateSystem("current_pose");
//     mViewer->addCoordinateSystem(0.02, Eigen::Affine3f(_pose), "current_pose");
//     mViewer->removeText3D("current_pose_text");
//     pcl::PointXYZ position(_pose(0, 3), _pose(1, 3), _pose(2, 3));
//     mViewer->addText3D("current_pose", position, 0.01, 1.0, 0.2, 0.2, "current_pose_text");
//     return true;
// }

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::pause()
{
    if (!mViewer)
        return;

    mPause = true;
    while (mPause)
    {
        mViewer->spinOnce(10, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
};

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::spinOnce()
{
    if (!mViewer)
        return;

    mViewer->spinOnce(10, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::keycallback(const pcl::visualization::KeyboardEvent &_event, void *_data)
{
    for (auto &callback : mCustomCallbacks)
    {
        callback(_event, _data);
    }

    if (_event.keyDown() && _event.getKeySym() == "z")
    {
        std::cout << "[Visualizer] Toogle pause" << std::endl;
        if (mPause == true)
            mPause = false;
        else
            pause();
    }
    else if (_event.keyDown() && _event.getKeySym() == "v")
    {
    }
};

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *viewer_void)
{
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
    }
};

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::pointPickedCallback(const pcl::visualization::PointPickingEvent &event, void *viewer_void)
{
    event.getPoint(x, y, z);
    std::cout << "Point clicked at position (" << x << ", " << y << ", " << z << ")" << std::endl;
};

//---------------------------------------------------------------------------------------------------------------------
template <typename PointType_>
inline void Visualizer<PointType_>::addCustomKeyCallback(CustomCallbackType _callback)
{
    mCustomCallbacks.push_back(_callback);
}
