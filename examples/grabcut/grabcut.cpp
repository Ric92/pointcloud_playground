#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat rgb;
cv::Mat depth;
cv::Mat mask;

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

// Function onMouse displays cursor values
void onMouseDepth( int event, int x, int y, int, void* )
{
    if ( event != CV_EVENT_LBUTTONDOWN )
        return;

    cv::Point pt = cv::Point(x,y);
    std::cout<<"("<<pt.x<<", "<<pt.y<<") ...... "<<(depth.at<unsigned short int>(y,x))/255 << '\n';
}

// Function onMouse displays cursor values
void onMouseMask( int event, int x, int y, int, void* )
{
    if ( event != CV_EVENT_LBUTTONDOWN )
        return;

    cv::Point pt = cv::Point(x,y);
    std::cout<<"("<<pt.x<<", "<<pt.y<<") ...... "<<(mask.at<uint16_t>(y,x)) << '\n';
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
    rgb = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    depth = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(argv[3], *cloud);


    //-----------------------------------------------------------------------------------------------------------//
    const int erosion_size = 3;
    cv::Mat erode_less = cv::getStructuringElement(cv::MORPH_RECT,
                                     cv::Size(erosion_size + 1, erosion_size + 1),
                                     cv::Point(erosion_size, erosion_size));
    const int dou_erosion_size = 3 * 2;
    cv::Mat erode_more = cv::getStructuringElement(cv::MORPH_RECT,
                                     cv::Size(dou_erosion_size + 1, dou_erosion_size + 1),
                                     cv::Point(dou_erosion_size, dou_erosion_size));

    // The following operation is taking grayscale image,
    // performs threashold on it, closes small holes and erodes the white area
    auto create_mask_from_depth = [&](cv::Mat& depth, int thresh,
                                      cv::ThresholdTypes type) {
        cv::threshold(depth, depth, thresh, 255, type);
        cv::dilate(depth, depth, erode_less);
        cv::erode(depth, depth, erode_more);
    };
    //-----------------------------------------------------------------------------------------------------------//


    // Generate "near" mask image:
    auto near = depth.clone();
    cv::normalize(near, near, 0, 255, cv::NORM_MINMAX);
    // Take just values within range [180-255]
    // These will roughly correspond to near objects due to histogram equalization
    create_mask_from_depth(near, 180, cv::THRESH_BINARY);

    // Generate "far" mask image:
    auto far = depth.clone();
    cv::normalize(far, far, 0, 255, cv::NORM_MINMAX);
    far.setTo(255, far == 0); // Note: 0 value does not indicate pixel near the camera, and requires special attention 
    create_mask_from_depth(far, 100, cv::THRESH_BINARY_INV);

    // GrabCut algorithm needs a mask with every pixel marked as either:
    // BGD, FGB, PR_BGD, PR_FGB
    mask.create(near.size(), CV_8UC1); 
    mask.setTo(cv::Scalar::all(cv::GC_BGD)); // Set "background" as default guess
    mask.setTo(cv::GC_PR_BGD, far == 0); // Relax this to "probably background" for pixels outside "far" region
    mask.setTo(cv::GC_FGD, near == 255); // Set pixels within the "near" region to "foreground"



    // Run Grab-Cut algorithm:
    cv::Mat bgModel, fgModel; 
    cv::grabCut(rgb, mask, cv::Rect(), bgModel, fgModel, 1, cv::GC_INIT_WITH_MASK);

    // Extract foreground pixels based on refined mask from the algorithm
    cv::Mat3b foreground = cv::Mat3b::zeros(rgb.rows, rgb.cols);
    rgb.copyTo(foreground, (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD));
    



    // visualize pointcloud
    // pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    // viewer.addPointCloud(cloud, "cloud");
    // viewer.registerPointPickingCallback(pointPickingEventOccurred,
    //                                     (void*)&viewer);
    // viewer.spin();

    // visualize images
    cv::namedWindow("rgb window");    // Create a window for rgb.
    cv::namedWindow("depth window");  // Create a window for depth.
    cv::namedWindow("near");  // Create a window for depth.
    cv::namedWindow("far");  // Create a window for depth.
    cv::setMouseCallback( "depth window", onMouseDepth, 0 );
    cv::setMouseCallback( "foreground", onMouseMask, 0 );
    while (true) {
        cv::imshow("depth window", depth);
        cv::imshow("rgb window", rgb);
        cv::imshow("foreground", foreground);
        cv::imshow("near", near);
        cv::imshow("far", far);
        cv::waitKey(1);
    }
    return 0;
}