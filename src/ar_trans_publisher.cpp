#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
//#include <ros/subscriber.h>
//#include <ros/publisher.h>

#include <tf2_msgs/TFMessage.h>

#define _DEG2RAD 0.01745331111
#define _RAD2DEG 57.2957184819

class trans_calculater
{  
public:
  ros::NodeHandle nh_;
  ros::Publisher tf_publisher_;

  std::string topic_name, base_link, odom_link, marker_link, marker_link1;

  float origin2x,origin2y, origin2z, roll, pitch, yaw;
  bool first_msg;
  
  tf::Transform marker_tf, odom_tf;
  tf::StampedTransform transform;
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
	nh_.param("odom", odom_link, std::string("odom"));
	nh_.param("marker_link", marker_link, std::string("marker_link"));
	nh_.param("marker_link1", marker_link1, std::string("ar_marker_1"));
	

	marker_tf =
	  tf::Transform(
					tf::createQuaternionFromRPY(roll * _DEG2RAD, pitch * _DEG2RAD, yaw * _DEG2RAD),
					tf::Vector3(origin2x, origin2y, origin2z));

	// world to odom
	odom_tf = tf::Transform(
							tf::Quaternion(0,0,0,1),
							tf::Vector3(0, 0.0, 0.0));
	
	tf_publisher_ = nh_.advertise<tf2_msgs::TFMessage>( "path/path", 1 );
  }

  void send_marker(){
	if(first_msg){
	  
	  try{
		ros::Time now = ros::Time(0);
		// listener.waitForTransform(marker_link1, odom_link,
	  	// 					  now, ros::Duration(2.0));
		// listener.waitForTransform(marker_link, marker_link1,
	  	// 					  now, ros::Duration(2.0));
		listener.lookupTransform(marker_link1, odom_link,		
								 now, transform);
		ROS_INFO_STREAM("ar_marker found!");
		odom_tf = marker_tf * transform;
		//first_msg = false;
	  }
	  catch(tf::TransformException &ex){
		//ROS_WARN("%s",ex.what());
		ROS_WARN_STREAM("ar_marker not found. ");
		//ros::Duration(1.0).sleep();
	  }
	}

	br.sendTransform(tf::StampedTransform(marker_tf, ros::Time::now(), base_link, marker_link));
	br.sendTransform(tf::StampedTransform(odom_tf, ros::Time::now(), base_link, odom_link));
	
	
  }
  
  
};


int main( int argc, char** argv )
{
  ros::init( argc, argv, "ar_trans_publisher");
  
  trans_calculater tc;
  ros::Rate loop_rate(10);
  while(ros::ok()){
	
	tc.send_marker();
	ros::spinOnce();
	loop_rate.sleep();
  }
}
