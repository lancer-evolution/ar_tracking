#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <iostream>

using namespace std;

class ScaleEstimator{
public:
  ros::Subscriber orb_sub;
  ros::Subscriber ar_sub;
  ros::Publisher scale_pub;
  ros::NodeHandle nh_;
  
  // Create the twist message
  geometry_msgs::Twist vel_msg;
  geometry_msgs::Twist sub_msg;

 
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform pre_transform;

  deque<geometry_msgs::PoseStamped> orb_data_queue_;
  deque<tf::StampedTransform> ar_data_queue_;

  bool fixed_scale_;

  ScaleEstimator():
	fixed_scale_(false)
  {
	vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0;
	vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;

    orb_sub = nh_.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orbCb, this );
	//ar_sub = nh_.subscribe("", 10, )
	// publish scale
    scale_pub = nh_.advertise<std_msgs::Float32> ( "/scale_estimator/scale", 1 );
    
  }

  
  void orbCb(geometry_msgs::PoseStamped msg){
	orb_data_queue_.push_front(msg);
  }
  

  void setup(){

  }

  bool hasFixedScale() {return fixed_scale_;}

  void processQueue(){
	if(orb_data_queue_.size() < 1){ // データが何もなかったら
	  return;
	}

	while(not orb_data_queue_.empty()){
	  geometry_msgs::PoseStamped orb_msg = orb_data_queue_.back();
	  orb_data_queue_.pop_back();
	  
	  double t_orb = orb_msg.header.stamp.toSec();

	  if(0){						// first_orb_msg_
	  }

	  while(not ar_data_queue_.empty()){
		double t_ar = ar_data_queue_.back().header.stamp.toSec();

		if(t_ar > t_orb){		// arの方がorbよりも時間が進んでいたら
		  break;
		}

		tf::StampedTransform transform = ar_data_queue_.back();
		ar_data_queue_.pop_back();

		if(0){					// first_ar_msg_
		}

		
	  }
	  
	}
	
  }

  void estimate(){
	ar_data_queue_.push_front ( transform );

	if ( orb_data_queue_.size() <1 ) {
	  return;
	}
	
	

  }
  void pid_cmd(){
	//vel_msg.linear.x = pid_x.kp * pid_x.linear_err + pid_x.ki * pid_x.linear_integral_err;
	//vel_msg.linear.y = pid_y.kp * pid_y.linear_err + pid_y.ki * pid_y.linear_integral_err;
	//vel_msg.linear.z = pid_z.kp * pid_z.linear_err + pid_z.ki * pid_z.linear_integral_err;
	// cout << "error_x:" << err_x << endl;
	// cout << "value_x:" << vel_msg.linear.x << endl;
	// cout << "error_y:" << err_y << endl;
	// cout << "value_y:" << vel_msg.linear.y << endl;

  }


};


int main(int argc, char** argv){
  ros::init(argc, argv, "ar_pid");
  ros::NodeHandle nh_;

  std::string camera_link;
  std::string marker_link;
  nh_.param("camera_link", camera_link, std::string("/orb/pose_unscaled_cam"));
  nh_.param("marker_link", marker_link, std::string("ar_marker_1"));

  ScaleEstimator scale_est;
  ros::Rate rate(10.0);
  tf::TransformListener listener;
  
  
  //scale_est.setup();
  
  while (scale_est.nh_.ok()){
	ros::spinOnce();

	tf::StampedTransform transform;
	try{
	  ros::Time now = ros::Time::now();
	  listener.waitForTransform(camera_link, marker_link,
								now, ros::Duration(1.5));
	  scale_est.listener.lookupTransform(camera_link, marker_link,				
										 now, transform);
	  scale_est.ar_data_queue_.push_front(transform);
	}
	catch (tf::TransformException &ex){
	  ROS_WARN("%s",ex.what());
	  //ros::Duration(1.0).sleep();
	  ROS_WARN("Change Manual Mode");
	  continue;
	}

	if(not scale_est.hasFixedScale() ){
	  scale_est.processQueue();     //doesn't do anything if queue size less than 50
	  //scale_est.estimateScale();
	}else {// scaleのパブリッシュ
	  std_msgs::Float32 scale_for_publish;
	  //scale_for_publish.data = scale_est.getScale();
	  //scale_pub.publish ( scale_for_publish );
	}
	  
   
	//rate.sleep();
  }
  return 0;
}
