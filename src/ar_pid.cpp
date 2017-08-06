#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <iostream>

using namespace std;

class PID{
public:
  double linear_err,linear_integral_err,linear_derivative_err;
  double max,min;
  // PID gains
  double kp;
  double ki;

  PID()
	:linear_err(0),
	 linear_integral_err(0),
	 linear_derivative_err(0)
  {
	max = 1;
	min = -1;

	kp = 0.01;
	ki = 0.01;
  }
  void pid_set(float err){
	linear_err = err;
	linear_integral_err += err;
	// cout << "err_r" << endl;
  }
  
  
};

class TeleopDronePID{
public:
  ros::Subscriber joy_sub;
  ros::Subscriber cmd_sub;
  ros::Publisher pub_vel;
  ros::NodeHandle nh_;
  
  // Create the twist message
  geometry_msgs::Twist vel_msg;
  geometry_msgs::Twist sub_msg;

  
  sensor_msgs::JoyConstPtr joy_data;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  float err_x,err_y,err_z;
  float target_z;
  
  bool is_flying;
  // button 8 (L2): dead man switch
  bool track_toggle;
  bool track_toggle_last_msg;
  bool track_mode;
  // button 11 (R1): switch emergeny state 
  bool emergency_toggle_pressed;
  
  PID pid_x,pid_y,pid_z;

  TeleopDronePID(){
	vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0;
	vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;

	target_z = 0.8;

	is_flying = false;
	emergency_toggle_pressed = false;
	track_toggle = false;
	track_toggle_last_msg = false;
	track_mode = false;

	joy_sub = nh_.subscribe("/joy", 1, &TeleopDronePID::joyCb, this);
	cmd_sub = nh_.subscribe("/ar_pid/cmd_vel", 1, &TeleopDronePID::cmdCb, this);
	pub_vel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  }
  void joyCb(const sensor_msgs::JoyConstPtr joy_msg){
	joy_data = joy_msg;
	track_toggle = joy_msg->buttons.at(8);
	emergency_toggle_pressed = joy_msg->buttons.at(11);
	//cout << joy_msg->buttons.at(8) << endl;
  }
  void cmdCb(const geometry_msgs::Twist cmd_msg){
	//sub_msg = cmd_msg;
	vel_msg = cmd_msg;
  }
  void setup(){
	pid_x.kp = 0.5;
	pid_x.ki = 0.1;
	pid_y.kp = 0.5;
	pid_y.ki = 0.1;
	pid_z.kp = 1;
	pid_z.ki = 0.1;
  }
  void judge_ar(){
	if(!track_toggle_last_msg && track_toggle){
	  //cout << "before:"<<track_mode << endl;
	  track_mode = !track_mode;
	  //cout << "after:"<<track_mode << endl;
	  if(track_mode){
		ROS_INFO("Tracking Mode");
	  }else
		ROS_INFO("Manual Mode");
	  track_toggle_last_msg = track_toggle;
	}
	if(!track_toggle)
	  track_toggle_last_msg = track_toggle;
  }
  void pid_calc(){
	  err_x = -transform.getOrigin().y();
	  err_y = transform.getOrigin().x();
	  err_z = target_z - transform.getOrigin().z();
	  
	  pid_x.pid_set(err_x);
	  pid_y.pid_set(err_y);
	  pid_z.pid_set(err_z);
	  pid_cmd();

  }
  void pid_cmd(){
	//vel_msg.linear.x = pid_x.kp * pid_x.linear_err + pid_x.ki * pid_x.linear_integral_err;
	//vel_msg.linear.y = pid_y.kp * pid_y.linear_err + pid_y.ki * pid_y.linear_integral_err;
	vel_msg.linear.z = pid_z.kp * pid_z.linear_err + pid_z.ki * pid_z.linear_integral_err;
	// cout << "error_x:" << err_x << endl;
	// cout << "value_x:" << vel_msg.linear.x << endl;
	// cout << "error_y:" << err_y << endl;
	// cout << "value_y:" << vel_msg.linear.y << endl;
	cout << "error_z:" << err_z << endl;
	cout << "value_z:" << vel_msg.linear.z << endl;
  }
  void send_cmd_vel(){
    pub_vel.publish(vel_msg);
  }

};


int main(int argc, char** argv){
  ros::init(argc, argv, "ar_pid");

  //ros::NodeHandle node;

  // ros::service::waitForService("spawn");
  // ros::ServiceClient add_turtle = 
  //   node.serviceClient<turtlesim::Spawn>("spawn");
  // turtlesim::Spawn srv;
  // add_turtle.call(srv);

  // ros::Publisher drone_vel = 
  //   node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

  TeleopDronePID teleop;
  ros::Rate rate(10.0);
  tf::TransformListener listener;
  
  
  teleop.setup();
  
  while (teleop.nh_.ok()){
	ros::spinOnce();
	// ARトラッキングするかどうかの判定
	teleop.judge_ar();
	
	if(teleop.track_mode){
	  tf::StampedTransform transform;
	  try{
		ros::Time now = ros::Time::now();
		listener.waitForTransform("ardrone_base_bottomcam", "ar_marker_1",
								  now, ros::Duration(1.5));
		teleop.listener.lookupTransform("ardrone_base_bottomcam", "ar_marker_1",									   
										now, teleop.transform);
	  }
	  catch (tf::TransformException &ex){
		ROS_WARN("%s",ex.what());
		//ros::Duration(1.0).sleep();
		ROS_WARN("Change Manual Mode");
		teleop.track_mode = false;
		//teleop.vel_msg = teleop.sub_msg;
		continue;
	  }
	  
	  teleop.pid_calc();
	  
	}
	// else{
	//   teleop.vel_msg = teleop.sub_msg;
	// }
		
	teleop.send_cmd_vel();
	
    rate.sleep();
  }
  return 0;
};
