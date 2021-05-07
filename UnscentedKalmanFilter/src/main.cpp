#include "highway.h"

int main(int argc, char** argv)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
	}

	// Write NIS into file
	std::vector<std::string> sensors{"Radar", "Lidar"};
    ofstream file;
    for (auto const &car : highway.traffic){
      std::vector<UKF::NIS_TYPE> nis_list{car.ukf.radar_nis, car.ukf.lidar_nis};
      std::string car_name = car.name;
      for (int sensor_idx{0}; sensor_idx < sensors.size(); ++sensor_idx){
        auto sensor_nis = nis_list[sensor_idx];
        std::string file_name = sensors[sensor_idx] + "_NIS_" + car_name + ".txt";
        file.open (file_name);
        file << "Time" << std::setw(30) << sensors[sensor_idx] + "-NIS" << std::endl;
        for (int idx{0}; idx < sensor_nis.time_stamp_second.size(); ++idx){
          double time = sensor_nis.time_stamp_second[idx];
          double nis = sensor_nis.nis[idx];
          file << time << std::setw(30) << nis << std::endl;
        }
        file.close();
      }
    }
}
