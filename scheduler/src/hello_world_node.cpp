#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

nav_msgs::MapMetaData g_map_data;
tf2::Quaternion q;   

geometry_msgs::Pose getPixel2Pose(float goal_x, float goal_y, float goal_theta)
{

  geometry_msgs::Pose pose;

  // origin(0, 0) = (g_map_data.origin.position.x,  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution)
  pose.position.x =  g_map_data.origin.position.x + goal_x * g_map_data.resolution ;
  pose.position.y =  g_map_data.origin.position.y + g_map_data.height *g_map_data.resolution - goal_y * g_map_data.resolution ;
  pose.position.z = 0.0;

  q.setRPY(0, 0, goal_theta);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "hello_world_node");
	ros::NodeHandle nh;
	ros::Publisher init_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
	ros::Rate loop_rate(0.2);
	int count = 0;
	float th = 0.0;
	geometry_msgs::PoseWithCovarianceStamped gInitPose;
	while (ros::ok()){
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world!" << count;
		msg.data = ss.str();
		ROS_INFO("%s %f",msg.data.c_str(),th);
	
		++count;
		boost::array<float,36> init_cov_arr = {
		0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
		}; 


		gInitPose.header.seq = 1;
		gInitPose.header.frame_id = "map";
		gInitPose.header.stamp = ros::Time(0);
    	gInitPose.pose.pose = getPixel2Pose(0.0,0.0,th);
		for(int i = 0; i<36; i++)
		{
		gInitPose.pose.covariance[i] = init_cov_arr[i];
		}
			
		init_pose_pub.publish(gInitPose);
		ROS_INFO("initPose");


		th= th+3.14;
		loop_rate.sleep();
	}
	return 0;
}
