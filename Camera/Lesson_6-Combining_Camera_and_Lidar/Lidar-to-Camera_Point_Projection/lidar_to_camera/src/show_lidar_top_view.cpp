#include <iostream>
#include <numeric>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    float z_min{std::numeric_limits<float>::max()};
    float z_max{std::numeric_limits<float>::min()};

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor
        float zw = (*it).z+1.73f; // world position in m with z facing top from sensor

        z_min = zw < z_min ? zw : z_min;
        z_max = zw > z_max ? zw : z_max;

        // TODO: 
        // 2. Remove all Lidar points on the road surface while preserving 
        // measurements on the obstacles in the scene.
        float min_height_for_non_road_surface_obects{0.2f};
        if (zw < min_height_for_non_road_surface_obects){
            continue;
        }

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;
        
        // TODO: 
        // 1. Change the color of the Lidar points such that 
        // X=0.0m corresponds to red while X=20.0m is shown as green.
        int green_part{static_cast<int>((255*xw)/worldSize.height)};
        int red_part{static_cast<int>((255*(worldSize.height - xw))/worldSize.height)};
        cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green_part, red_part), -1);  // BGR  
    }

    std::cout << "z_min = " << z_min << ", z_max = " << z_max << std::endl;

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}