#include<ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<pcl_ros/transforms.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv;
void check_files(ifstream& in_file,string& in_name){
	if(!in_file.is_open()){
		cerr<< "Cannot open trajectory file"<< in_name<< endl;
		exit(EXIT_FAILURE);
	}	
}



int main(int argc, char * argv[]){

	string plan_file_name;
	ros::init(argc,argv,"planning");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");
	
	bool offline_homography = false;
	home.getParam("offline_homography", offline_homography);

	//string H_file = "/homography/H.yml";
	//H_file = ros::package::getPath("iros18_vision")+H_file;
        //FileStorage fs(H_file.c_str(), FileStorage::READ);

	Mat offline_H;
	//fs["Homography"] >> offline_H;


	int loop_frequency = 10;
	ros::Rate loop_rate(loop_frequency);	

	//ros::Publisher plan_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/plan",10);
	ros::Publisher filtered_traj_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> > ("/filtered_tissue_traj",1);
	trajectory_msgs::JointTrajectory plan;
	trajectory_msgs::JointTrajectoryPoint point;
	for (int i = 0; i < 6; ++i){
		point.positions.push_back(0.0);
		point.velocities.push_back(0.0);
		point.accelerations.push_back(0.0);
	}
	// getting the name of the trajecotry parameter file and reading it (it should be in a .txt file)
	home.getParam("plan_file_name", plan_file_name);
	plan_file_name = ros::package::getPath("kuka_control") + plan_file_name;
	ifstream plan_file(plan_file_name.c_str(), ifstream::in);
	check_files(plan_file,plan_file_name);
	string line;
	
	// for now only processes the last data set in the file as with the start and stop data
	int ctr = 0;
	pcl::PointCloud<pcl::PointXYZI> filtered_output_traj;
	while(getline(plan_file, line)){
		istringstream iss(line);
		
		if (ctr == 0){
			double dump;
			for (int i = 0; i < 18; ++i)
				iss >> dump;
			ctr ++;
		}else{
			// read the positions
			for (int i = 0; i < 6; ++i)
				iss >> point.positions[i];
			pcl::PointXYZI xyzi;
			xyzi.x = point.positions[0];
			xyzi.y = point.positions[1];
			xyzi.z = point.positions[2];
			xyzi.intensity = 0;
			filtered_output_traj.points.push_back(xyzi);
			// read the velocities
			for (int i = 0; i < 6; ++i)
				iss >> point.velocities[i];
			// read the accelerations
			for (int i = 0; i < 6; ++i)
				iss >> point.accelerations[i];
			// for now we do not specify durations

			if(offline_homography){		
				vector<Point2f> scene_wps;
				vector<Point2f> way_point;
				Point2f tmp;
				tmp.x = point.positions[0];
				tmp.y = point.positions[1];
				way_point.push_back(tmp);
				perspectiveTransform( way_point, scene_wps, offline_H);
				point.positions[0] = scene_wps[0].x;
				point.positions[1] = scene_wps[0].y;
			}	
			plan.points.push_back(point);
		}
	}
	
	while(ros::ok()){
		//plan.header.stamp = ros::Time::now();
		//plan_pub.publish(plan);
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("world");
		filtered_output_traj.header = pcl_conversions::toPCL(header);
	 	filtered_traj_pub.publish(filtered_output_traj);
		ros::spinOnce();
		loop_rate.sleep();
	}
	plan_file.close();
	return 0;
}

