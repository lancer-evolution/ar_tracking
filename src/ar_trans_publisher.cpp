#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <ros/subscriber.h>
//#include <ros/publisher.h>

#include <tf2_msgs/TFMessage.h>

#define _DEG2RAD 0.01745331111
#define _RAD2DEG 57.2957184819

using namespace std;

class trans_calculater
{  
public:
  ros::NodeHandle nh_;
  ros::Publisher tf_publisher_;

  std::string topic_name, base_link, odom_link, scaled_maker_link, marker_link, marker_link1, unscaled_cam_link, scaled_cam_link;

  float origin2x,origin2y, origin2z, roll, pitch, yaw;
  bool first_msg;
  
  tf::Transform marker_tf, odom_tf;
  tf::StampedTransform transform, transform_scaled;
  tf::TransformListener listener;
  tf::TransformBroadcaster br;

  trans_calculater():
	nh_("~"),
	first_msg(true)
  {
	nh_.param<float>("x", origin2x, 0.0);
	nh_.param<float>("y", origin2y, 0.6);
	nh_.param<float>("z", origin2z, -0.6);
	nh_.param<float>("roll", roll, 0);
	nh_.param<float>("pitch", pitch, 0);
	nh_.param<float>("yaw", yaw, 0);

	nh_.param("base_link", base_link, std::string("velodyne"));
	nh_.param("unscaled_cam_link", unscaled_cam_link, std::string("orb_pose_unscaled_cam"));
	nh_.param("scaled_cam_link", scaled_cam_link, std::string("orb_pose_scaled_cam"));
	nh_.param("odom", odom_link, std::string("odom"));
	nh_.param("marker_link", marker_link, std::string("marker_link"));
	nh_.param("marker_link1", marker_link1, std::string("ar_marker_1"));
	nh_.param("scaled_marker_link", scaled_maker_link, std::string("scaled_marker_link"));

	marker_tf =
	  tf::Transform(
					tf::createQuaternionFromRPY(roll * _DEG2RAD, pitch * _DEG2RAD, yaw * _DEG2RAD),
					tf::Vector3(origin2x, origin2y, origin2z));

	// world to odom
	odom_tf = tf::Transform(
							tf::Quaternion(0,0,0,1),
							tf::Vector3(0, 0.0, 0.0));
	// cam to marker
	transform_scaled.setData(tf::Transform(
										   tf::Quaternion(0,0,0,1),
										   tf::Vector3(0, 0.0, 0.0))
							 );
	tf_publisher_ = nh_.advertise<tf2_msgs::TFMessage>( "path/path", 1 );
  }

  void send_marker(){
	ros::Time ros_time;

	try{
	  ros_time = ros::Time(0);
	  //ros_time = ros::Time::now();
	  // listener.waitForTransform(marker_link1, odom_link,
	  // 					  now, ros::Duration(2.0));
	  // listener.waitForTransform(marker_link, marker_link1,
	  // 					  now, ros::Duration(2.0));
	  listener.waitForTransform(unscaled_cam_link, marker_link1,
	  							ros_time, ros::Duration(1.0));
	  listener.lookupTransform(unscaled_cam_link, marker_link1,
							   ros_time, transform_scaled);
	  listener.waitForTransform(scaled_cam_link, odom_link,
	  							transform_scaled.stamp_, ros::Duration(1.0));
	  listener.lookupTransform(scaled_cam_link, odom_link,		
							   transform_scaled.stamp_, transform);
		
	  //ROS_INFO_STREAM("ar_marker found!");
	  //odom_tf = marker_tf * transform;
	  odom_tf = marker_tf * transform_scaled.inverse() * transform;
	  ros_time = transform.stamp_;
	  // cout << "trans_scaled stamp: " << fixed<< transform_scaled.stamp_.toSec()<< endl;
	  // cout << "trans stamp:" << transform.stamp_.toSec() << endl;
	  
	  // scale_cam to virtual marker
	  br.sendTransform(tf::StampedTransform(transform_scaled, ros_time, scaled_cam_link, scaled_maker_link));
	}
	catch(tf::TransformException &ex){
	  //ROS_WARN("%s",ex.what());
	  //ROS_WARN_STREAM("ar_marker not found. ");
	  //ros::Duration(1.0).sleep();
	  ros_time = ros::Time::now();

	}

	// base_link to odom
	br.sendTransform(tf::StampedTransform(odom_tf, ros_time, base_link, odom_link));
	// base_link to setted marker
	br.sendTransform(tf::StampedTransform(marker_tf, ros_time, base_link, marker_link));
	
	  
  }
  
  
};


int main( int argc, char** argv )
{
  ros::init( argc, argv, "ar_trans_publisher");
  
  trans_calculater tc;
  ros::Rate loop_rate(1);
  while(ros::ok()){
	
	tc.send_marker();
	ros::spinOnce();
	//loop_rate.sleep();
  }
}
