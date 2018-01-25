#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <math.h>

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
  geometry_msgs::PoseStamped pre_msg;

  std::string marker_link,camera_link, world_link;
 
  tf::TransformListener listener;
  tf::StampedTransform transform;
  tf::StampedTransform pre_transform;
  tf::StampedTransform transform_orb, pre_transform_orb;
  tf::StampedTransform transform_move;

  deque<geometry_msgs::PoseStamped> orb_data_queue_;
  deque<tf::StampedTransform> ar_data_queue_;

  int counter;
  bool fixed_scale_;
  bool first_msg, skip_process;
  double msg_dist, ar_dist,
	scale, scale_mean, scale_mean_pre, scale_var, scale_dev, scale_coef4var;
  double scale_thresh;
  
  ScaleEstimator():
	fixed_scale_(false),
	first_msg(true),
	skip_process(true),
	scale(1)
  {

    //orb_sub = nh_.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orbCb, this );
    scale_pub = nh_.advertise<std_msgs::Float32> ( "/scale_estimator/scale", 1 );
    
  }

  
  void orbCb(geometry_msgs::PoseStamped msg){
	//orb_data_queue_.push_front(msg);

	if(not fixed_scale_){
	  try{
		// ros::Time now = ros::Time::now();
		// ros::Time msg_time = msg.header.stamp;
		// //double msg_time = msg.header.stamp;
		// ros::Time past = now - ros::Duration(now - msg_time);
		ros::Time past = ros::Time::now();
		listener.waitForTransform(marker_link, camera_link,
								  msg.header.stamp, ros::Duration(1.5));
		listener.lookupTransform(marker_link, camera_link,		
								 msg.header.stamp, transform);
		// cout << "cam stamp: " << fixed<< msg_time.toSec()<< endl;
		// cout << "marker stamp:" << transform.stamp_.toSec() << endl;
	  }
	  catch (tf::TransformException &ex){
		//ROS_WARN("%s",ex.what());
		//ros::Duration(1.0).sleep();
		//continue;
	  }

	  double diff_time = msg.header.stamp.toSec() - transform.stamp_.toSec();


	  if(fabs(diff_time) > 0.001){
	  	// 差が大きすぎる場合はスルー
	  }else if(first_msg){
	  	cout << "first" << endl;
	  	pre_transform = transform;
	  	pre_msg = msg;
	  	first_msg = false;
	  	counter = scale_mean = scale_mean_pre = scale_var = 0;
	  }else if(1){ // 移動量があるか
	  	msg_dist = 
	  	  sqrt(
	  		   pow(msg.pose.position.x - pre_msg.pose.position.x, 2)
	  		   + pow(msg.pose.position.y - pre_msg.pose.position.y, 2)
	  		   + pow(msg.pose.position.z - pre_msg.pose.position.z, 2)
	  		   );
	  
	  	// metric
	  	ar_dist =
	  	  sqrt(
	  		   pow(transform.getOrigin().x() - pre_transform.getOrigin().x(), 2)
	  		   + pow(transform.getOrigin().y() - pre_transform.getOrigin().y(), 2)
	  		   + pow(transform.getOrigin().z() - pre_transform.getOrigin().z(), 2)
	  		   );
		
		//cout << msg_dist << ", " << ar_dist << endl;
		//cout << msg_dist << endl;
	    
		// 移動量があるか
		if(ar_dist < 0.1 && ar_dist > 0.03){			// 5cm以上
		  scale = msg_dist/ar_dist;
		  
		  //cout << fixed << diff_time << "sec." << endl;
		  // 平均値
		  scale_mean = (counter*scale_mean_pre + scale)/(counter + 1);
		  // 分散
		  scale_var = (counter*scale_var + (scale-scale_mean_pre)*(scale-scale_mean))
			/(counter + 1);
		  scale_mean_pre = scale_mean;
		  // 標準偏差deviation
		  scale_dev = sqrt(scale_var);
		  // 変動係数
		  scale_coef4var = scale_dev/scale_mean;
		  counter++;

		  cout << endl;
		  cout << "scale counter:" << counter+1 << endl;
		  cout << "orb: " << msg_dist << endl;
		  cout << "ar : " << ar_dist << endl;
		  cout << "scale    : " << scale << endl;
		  cout << "scale Ave: " << scale_mean << endl;
		  cout << "scale var: " << scale_var << endl;
		  cout << "scale coef4var: " << scale_coef4var << endl;

		  if(scale_coef4var < scale_thresh && counter > 10){
			std_msgs::Float32 scale_for_publish;
			scale_for_publish.data = scale_mean;
			scale_pub.publish(scale_for_publish);
			fixed_scale_ = true;
			cout << "scale fixed!" << endl;
			cout << "published scale:" << scale_mean << endl;
		  }
		
		  pre_transform = transform;
		  pre_msg = msg;
		}else if (ar_dist > 0.1){	// 移動量が多すぎる場合は初期化
		  pre_transform = transform;
		  pre_msg = msg;
		}
	  
	  }
	}

  }

  void estimate(){
	//orb_data_queue_.push_front(msg);

	if(not fixed_scale_)
	  {
		try{
		  // ros::Time now = ros::Time::now();
		  // ros::Time msg_time = msg.header.stamp;
		  // //double msg_time = msg.header.stamp;
		  // ros::Time past = now - ros::Duration(now - msg_time);
		  ros::Time past = ros::Time(0);
		
		  listener.waitForTransform(marker_link, camera_link,
									past, ros::Duration(5.5));
		  listener.lookupTransform(marker_link, camera_link,	
								   past, transform);
		  listener.waitForTransform(world_link, camera_link,
									transform.stamp_, ros::Duration(5.5));
		  listener.lookupTransform(world_link, camera_link,		
								   transform.stamp_, transform_orb);
		  // cout << "cam stamp: " << fixed<< msg_time.toSec()<< endl;
		  // cout << "marker stamp:" << transform.stamp_.toSec() << endl;
		  if(first_msg){
			cout << "first" << endl;
			pre_transform = transform;
			pre_transform_orb = transform_orb;
			first_msg = false;
			counter = scale_mean = scale_mean_pre = scale_var = 0;
		  }else{ // 移動量があるか
			msg_dist = 
			  sqrt(
				   pow(transform_orb.getOrigin().x() - pre_transform_orb.getOrigin().x(), 2)
				   + pow(transform_orb.getOrigin().y() - pre_transform_orb.getOrigin().y(), 2)
				   + pow(transform_orb.getOrigin().z() - pre_transform_orb.getOrigin().z(), 2)
				   );
	  
			// metric
			ar_dist =
			  sqrt(
				   pow(transform.getOrigin().x() - pre_transform.getOrigin().x(), 2)
				   + pow(transform.getOrigin().y() - pre_transform.getOrigin().y(), 2)
				   + pow(transform.getOrigin().z() - pre_transform.getOrigin().z(), 2)
				   );
		
			//cout << msg_dist << ", " << ar_dist << endl;
			//cout << msg_dist << endl;
	    
			// 移動量があるか
			if(ar_dist < 0.1 && ar_dist > 0.03){			// 5cm以上
			  scale = msg_dist/ar_dist;
		  
			  //cout << fixed << diff_time << "sec." << endl;
			  // 平均値
			  scale_mean = (counter*scale_mean_pre + scale)/(counter + 1);
			  // 分散
			  scale_var = (counter*scale_var + (scale-scale_mean_pre)*(scale-scale_mean))
				/(counter + 1);
			  scale_mean_pre = scale_mean;
			  // 標準偏差deviation
			  scale_dev = sqrt(scale_var);
			  // 変動係数
			  scale_coef4var = scale_dev/scale_mean;
			  counter++;

			  cout << endl;
			  cout << "scale counter:" << counter+1 << endl;
			  cout << "orb: " << msg_dist << endl;
			  cout << "ar : " << ar_dist << endl;
			  cout << "scale    : " << scale << endl;
			  cout << "scale Ave: " << scale_mean << endl;
			  cout << "scale var: " << scale_var << endl;
			  cout << "scale coef4var: " << scale_coef4var << endl;

			  if(scale_coef4var < scale_thresh && counter > 10){
				std_msgs::Float32 scale_for_publish;
				scale_for_publish.data = scale_mean;
				scale_pub.publish(scale_for_publish);
				fixed_scale_ = true;
				cout << "scale fixed!" << endl;
				cout << "published scale:" << scale_mean << endl;
			  }
		
			  pre_transform = transform;
			  pre_transform_orb = transform_orb;
			}else if (ar_dist > 0.1){	// 移動量が多すぎる場合は初期化
			  pre_transform = transform;
			  pre_transform_orb = transform_orb;
			}
		  }
		}
		catch (tf::TransformException &ex){
		  //ROS_WARN("%s",ex.what());
		  //ros::Duration(1.0).sleep();
		  //continue;
		}
	  }
  }
  
  bool hasFixedScale() {return fixed_scale_;}

};


int main(int argc, char** argv){
  ros::init(argc, argv, "ar_pid");
  ros::NodeHandle nh_("~");

  ScaleEstimator scale_est;
  //std::string camera_link;
  //std::string marker_link;
  nh_.param("camera_link", scale_est.camera_link, std::string("/orb_pose_unscaled_cam"));
  nh_.param("marker_link", scale_est.marker_link, std::string("/ar_marker_1"));
  nh_.param("world_link", scale_est.world_link, std::string("/odom"));
  nh_.param("scale_thresh", scale_est.scale_thresh, 0.07);

  
  ros::Rate rate(5.0);
  tf::TransformListener listener;
  
  
  //scale_est.setup();
  
  while (scale_est.nh_.ok()){
	ros::spinOnce();
	scale_est.estimate();
	// if(not scale_est.hasFixedScale() ){
	//   scale_est.processQueue();     //doesn't do anything if queue size less than 50
	//   //scale_est.estimateScale();
	// }else {// scaleのパブリッシュ
	//   std_msgs::Float32 scale_for_publish;
	//   //scale_for_publish.data = scale_est.getScale();
	//   //scale_pub.publish ( scale_for_publish );
	// }
	  
   
	//rate.sleep();
  }
  return 0;
}
