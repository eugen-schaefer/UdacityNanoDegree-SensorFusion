/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool render_scene = false;
  bool render_rays = false;
  bool render_cloud = true;
  bool render_segmentation = false;
  bool render_cluster = true;
  std::vector<Car> cars = initHighway(render_scene, viewer);

  // TODO:: Create lidar sensor
  Lidar *lidar = new Lidar(cars, 0.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  point_cloud = lidar->scan();
  if (render_rays) {
    renderRays(viewer, lidar->position, point_cloud);
  }
  if (render_cloud) {
    renderPointCloud(viewer, point_cloud, "some name");
  }

  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> point_processor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentCloud = point_processor.PCLSegmentPlane(point_cloud, 100, 0.2);
  if (render_segmentation) {
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  }

  // Euclidean PCLClustering with PCL
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      point_processor.PCLClustering(segmentCloud.first, 1.0, 3, 30);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

  for (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster : cloudClusters) {
    std::cout << "cluster size ";
    point_processor.numPoints(cluster);
    if (render_cluster) {
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId]);
      Box box = point_processor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }
    ++clusterId;
  }
}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> &point_processor_I,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  auto filtered_cloud = point_processor_I.FilterCloud(
      input_cloud, 0.05f, Eigen::Vector4f(-10, -5, -2, 1),
      Eigen::Vector4f(30, 6.5, 1, 1));

  //  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
  //  pcl::PointCloud<pcl::PointXYZI>::Ptr>
  //      segmented_cloud = point_processor_I.PCLSegmentPlane(
  //      filtered_cloud, 100, 0.2);

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmented_cloud =
          point_processor_I.StudentSegmentPlane(filtered_cloud, 100, 0.2);

  renderPointCloud(viewer, segmented_cloud.second, "planeCloud",
                   Color(0, 1, 0));

  // Euclidean Clustering - PCL version
  //    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters =
  //    point_processor_I.PCLClustering(
  //        segmented_cloud.first, 0.5, 50, 20000);

  // Euclidean Clustering - student version
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters =
      point_processor_I.StudentClustering(segmented_cloud.first, 0.5, 50);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0),       Color(1, 1, 0),
                               Color(0, 0, 1),       Color(1, 0, 0.5),
                               Color(0.2, 0.6, 0.8), Color(0.1, 0.9, .5)};

  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster :
       obstacle_clusters) {
    std::cout << "cluster size ";
    point_processor_I.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = point_processor_I.BoundingBox(cluster);
    // renderBox(viewer, box, clusterId);

    ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 0, 1, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  // simpleHighway(viewer);

  ProcessPointClouds<pcl::PointXYZI> point_processor_I{};

  std::vector<boost::filesystem::path> stream =
      point_processor_I.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    input_cloud = point_processor_I.loadPcd((*streamIterator).string());
    cityBlock(viewer, point_processor_I, input_cloud);

    streamIterator++;
    if (streamIterator == stream.end()) streamIterator = stream.begin();

    viewer->spinOnce();
  }
}