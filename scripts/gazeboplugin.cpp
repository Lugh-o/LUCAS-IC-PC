#include <ros/ros.h>
#include "/home/ebrenna8/gazebo_downloads/src/gazebo_ros_pkgs/gazebo_ros/include/gazebo_ros/gazebo_ros_api_plugin.h"
#include <gazebo_msgs/SpawnModel.h>
#include <iostream>
#include <fstream>
#include "gazebo/physics/physics.hh"
#include <gazebo/gazebo.hh>
#include "gazebo/msgs/MessageTypes.hh"
#include "std_msgs/String.h"
#include <exception>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/msgs/msgs.hh>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/SetModelState.h>
#include "/opt/ros/indigo/include/tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <math.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include "ros/callback_queue.h"
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <thread>
#include <cmath>

#include <limits>
#include <vector>

namespace gazebo {

	class CallSpawnDeleteWorldPlugin:public WorldPlugin {
		printf("Hello World!\n");
		//
		printf("before anything happens:...");

		printf("end");

		private: physics::WorldPtr world;
		private: sdf::ElementPtr sdf;
		private:ros::NodeHandle *rosNode;
		private:tf2_ros::Buffer tfBuffer; //tf2_ros

		private:tf::StampedTransform transformLeft;
		private:tf::StampedTransform transformRight;
		private:tf::StampedTransform transform;

		private:float prev_x_right = -0.0;
		private:float prev_y_right = 0.035;
		private:float prev_theta_right = 0.005;

		private:float prev_x_left = 0.0;
		private:float prev_y_left = 0.035;
		private:float prev_theta_left = -0.003;
		private:float angular_vel_right = 0.0;
		private:float angular_vel_left = 0.0;
		private:float linear_vel_right = 0.0;
		private:float linear_vel_left = 0.0;
		private:float linear_vel_right_flat_tire = 0.0;
		private:float wheel_radius_left = 0.0;
		private:float wheel_radius_right = 0.0;
		private:double theta = 0.0;
		private:float x_star_left = 0.0;
		private:float y_star_left = 0.0;
		private:float x_star_right = 0.0;
		private:float y_star_right = 0.0;
		private:tf2_msgs::TFMessage t;
		private:float current_x_right = 0.0;
		private:float current_y_right = 0.0;
		private:double current_theta_right = 0.0;
		private:float current_x_left = 0.0;
		private:float current_y_left = 0.0;

		private:double current_theta_left = 0.0;
		private:float x_prime = 0.0;
		private:float y_prime = 0.0;
		private:float theta_prime = 0.0;
		private:float x_prime_flat_tire = 0.0;
		private:float y_prime_flat_tire = 0.0;
		private:float theta_prime_flat_tire = 0.0;
		private:float current_x = 0.0;
		private:float current_y = 0.0;
		private:float current_z = 0.0;
		private:double current_theta = 0.0;
		private:double current_tf_theta = 0.0;
		private:double prev_x = 0.0;
		private:double prev_y = 0.0;
		private:double prev_theta = 0.0;
		private:double prev_x_2 = 0.0;
		private:double prev_y_2 = 0.0;
		private:double prev_theta_2 = 0.0;
		private:double x_motion_update = 0.0;
		private:double y_motion_update = 0.0;
		private:double theta_motion_update = 0.0;
		private:double prev_x_motion_update = 0.0;
		private:double prev_y_motion_update = 0.0;
		private:double prev_theta_motion_update = 0.0;
		private:ros::Publisher kidnap_pub;
		private:std_msgs::String kidnap_msg;

		private: ros::CallbackQueue callbackQueue;
		private: GazeboRosPtr gazeboRos;
		private: ros::Subscriber joySub;
		private: std::unique_ptr<ros::NodeHandle> rosNodeTF;
		private: std::thread rosQueueThread;
		private:ros::ServiceClient client;

		private:ros::ServiceClient Getclient;
		private:ros::ServiceClient GlobalLocalizationClient;
		public: CallSpawnPlugin() {} //brief constructor
		private:double count;
		private: std::ofstream myGazeboFile;
		private: std::ofstream myTFFile;
		private:ros::Timer timer;

		public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

			this->world = _world;
			this->sdf = _sdf;
			//
			//
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init(_world->GetName());
			printf("haven't died yet");
			std::cout<<_world->GetName();
			//Good through here.
			std::string topicname = "/tf";
			this->rosNode = new ros::NodeHandle(_world->GetName());
			//ros::SubscribeOptions subOpts =
			ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf", 1, boost::bind(&CallSpawnDeleteWorldPlugin::OnMsgTest, this, _1), ros::VoidPtr(), &this->callbackQueue);
			kidnap_pub = rosNode->advertise<std_msgs::String>("kidnap_event",1000);
			//Creating a timer.
			//this->timer = this->rosNode->createTimer(ros::Duration(1),
			boost::bind(&CallSpawnDeleteWorldPlugin::timerCallback, this, _1));//this creates a continuous localization disturbance
			//this->timer = this->rosNode->createTimer(ros::Duration(45),
			boost::bind(&CallSpawnDeleteWorldPlugin::timerCallbackMinor, this, _1));//for orientation
			//this->timer = this->rosNode->createTimer(ros::Duration(1),
			boost::bind(&CallSpawnDeleteWorldPlugin::timerCallbackRelocationUnitTest, this, _1));
			this->timer = this->rosNode->createTimer(ros::Duration(35),
			boost::bind(&CallSpawnDeleteWorldPlugin::timerCallbackMajor, this, _1)); //45 (Major 1-10?11?) and 65.
			//this->timer = this->rosNode->createTimer(ros::Duration(35),
			boost::bind(&CallSpawnDeleteWorldPlugin::timerCallbackMajorWithGlobalLocalization, this,_1)); //45 (Major 1-10? 11?) and 65.
			//Explanation: You have to do the Boost::bind ...s stuff to pass the function in the right form.
			//Subscribe to TF
			//
			this->joySub = this->rosNode->subscribe(subOpts);
			this->rosQueueThread=std::thread(std::bind(&CallSpawnDeleteWorldPlugin::QueueThread, this));
			this->client = this->rosNode->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
			this->Getclient = this->rosNode->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
			this->GlobalLocalizationClient = this->rosNode->serviceClient<std_srvs::Empty>("/global_localization");
			this->count = 0;
			this->myGazeboFile.open("/home/ebrenna8/Gazebo.txt");
			this->myTFFile.open("/home/ebrenna8/TF.txt");
			this->myGazeboFile<<"RosTime\tprev_x_2\tprev_y_2\tprev_theta_2\tprev_x\tprev_y\tprev_theta\tcurrent_x\tcurrent_y\tcurrent_theta\tNewX\tNewY\tNewTheta";

			printf("Subscribed to my_topic");

		}

		private: void QueueThread() {
			static const double timeout = 0.10;
			while (this->rosNode->ok()) {
			this ->callbackQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
		private:double roll, pitch, yaw;
		private:geometry_msgs::Pose newPose;
		private:std::string s;
		private:bool found_base_footprint = false;
		private:bool found_base_link = false;
		private:bool found_wheel_right_link = false;
		private:bool found_wheel_left_link = false;
		private:tf::Transformer TFtransformer;

		private:double vel_direction = 0;
		private:double vel_direction_left = 0;
		private:double vel_direction_right = 0;


		private:gazebo::common::Time current_time;
		private:gazebo::common::Time prev_time;
		private:double d;
		private:double prev_predicted_theta = 0.0;
		private:double prev_predicted_x = 0.0;
		private:double prev_predicted_y = 0.0;
		private:double current_tf_x = 0.0;
		private:double current_tf_y = 0.0;
		private:double slip_x_motion_update = 0.0;
		private:double slip_y_motion_update = 0.0;
		private:double slip_theta_motion_update = 0.0;
		private:double slip_x_prime = 0.0;
		private:double slip_y_prime = 0.0;
		private:double slip_theta_prime = 0.0;
		private:double prev_slip_x_motion_update = 0.0;
		private:double prev_slip_y_motion_update = 0.0;
		private:double prev_slip_theta_motion_update = 0.0;
		private:double *intx_inty =NULL;
		private:double *xc_yc_mainradius = NULL;
		private:double *newX_newY_newTheta=NULL;
		private:ros::Time callback_time;
		private:int icount=0;
		//Function for Timer-controller callback:
		void timerCallback(const ros::TimerEvent& event){
			//private:void timerCallback(void){
			//private:void timerCallback(const ros::TimerEvent& event);
			//ROS_INFO("INSIDE TIMER CALLBACK!");
			callback_time = ros::Time::now();
			ros::Time five_seconds(5.0);
			ros::Time lower_seconds(40.0);
			ros::Time upper_seconds(50.0);
			if (ros::Time::now() > five_seconds) { //This is to prevetn the callback from inhibiting the inital load of Gazebo.

				if (ros::Time::now() >= lower_seconds && ros::Time::now() <=upper_seconds){//Begin execute only if meets time
					this->world->SetPaused(true);
					//Gazebo model pose:
					gazebo_msgs::GetModelState getmodelstate;
					getmodelstate.request.model_name = "mobile_base";
					this->Getclient.call(getmodelstate);
					current_x = getmodelstate.response.pose.position.x;
					current_y = getmodelstate.response.pose.position.y;
					current_z = getmodelstate.response.pose.position.z;
					tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z, getmodelstate.response.pose.orientation.w);
					tf::Matrix3x3 m(q);
					m.getRPY(roll, pitch, yaw);
					current_theta = yaw;
					//If it's the first time, set prev_x to current pos
					if (prev_x == 0){
					prev_x = current_x;
					prev_y = current_y;
					prev_theta = current_theta;
					prev_x_2 = prev_x;
					prev_y_2 = prev_y;
					prev_theta_2 = prev_theta;
					}
					//NEW ARC-BASED APPROACH!!!
					//RESETTING X_PRIME, Y_PRIME, AND THETA_PRIME
					ROS_INFO("Inputting:");
					ROS_INFO("{%f, %f, %f, %f, %f, %f, %f, %f, %f};", prev_x_2, prev_y_2, prev_theta_2, prev_x, prev_y, prev_theta, current_x, current_y, current_theta);
					//0.0002 seems to be a good metric for moving vs not moving
					if ((std::abs(current_x - prev_x) > 0.0002) && (std::abs(current_y-prev_y) > 0.0002)){ //if there's been sufficient movement
						icount = icount + 1;
						this->myGazeboFile<<"\nKidnapped Pose!";
						this->myGazeboFile<<"\n"<<callback_time<<"\t"<<prev_x_2<<"\t"<<prev_y_2<<"\t"<<prev_theta_2<<"\t"<<prev_x<<"\t"<<prev_y<<"\t"<<prev_theta<<"\t"<<current_x<<"\t"<<current_y<<"\t"<<current_theta;
						std::vector<double> r(3);
						r = calculate_new_pose(prev_theta_2, prev_x_2, prev_y_2, prev_theta, prev_x, prev_y, current_theta, current_x, current_y);
						this->myGazeboFile<<"\t"<<r[0]<<"\t"<<r[1]<<"\t"<<r[2];
						//this->myTFFile<<"\t"<<r[0]<<"\t"<<r[1]<<"\t"<<r[2];
						ROS_INFO("Calculated pose:");
						ROS_INFO("%f %f %f", r[0], r[1], r[2]);
						x_prime = r[0];
						y_prime = r[1];
						theta_prime = r[2];
						//END OF ARC-BASED APPROACH!!!
						//Prepare to publish
						newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
						newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
						//Unroll theta_prime into a quaternion
						tf::Quaternion quat;
						quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH *YAW*
						newPose.orientation.x = quat.x();
						newPose.orientation.y = quat.y();
						newPose.orientation.z = quat.z();
						newPose.orientation.w = quat.w();

						//call gazebo set model state:
						gazebo_msgs::ModelState modelstate;
						modelstate.model_name = (std::string) "mobile_base";
						modelstate.reference_frame = (std::string) "world";
						modelstate.pose = newPose;
						gazebo_msgs::SetModelState setmodelstate;
						setmodelstate.request.model_state = modelstate;
						if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) || std::isnan(modelstate.pose.position.z) ||std::isnan(modelstate.pose.orientation.x) ||std::isnan(modelstate.pose.orientation.y) ||std::isnan(modelstate.pose.orientation.z) || std::isnan(modelstate.pose.orientation.w)||std::isnan(modelstate.twist.linear.x) || std::isnan(modelstate.twist.linear.y) ||std::isnan(modelstate.twist.linear.z) ||std::isnan(modelstate.twist.angular.x) ||std::isnan(modelstate.twist.angular.y) |std::isnan(modelstate.twist.angular.z)){
						ROS_INFO("There's a nan here...1");
						this->world->SetPaused(false);
						prev_x_2 = prev_x;
						prev_y_2 = prev_y;
						prev_theta_2 = prev_theta;
						prev_x = current_x;
						prev_y = current_y;
						prev_theta = current_theta;
						}
						else{
							if(std::isnan(r[0]) || std::isnan(r[1]) || std::isnan(r[2])) {
							ROS_INFO("************* Missed a NAN*****************\r\n");
							}
							//ROS_INFO("%f", ros::Time::now().toSec());
							this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
							//ROS_INFO("%f", ros::Time::now().toSec());
							prev_x_2 = prev_x;
							prev_y_2 = prev_y;
							prev_theta_2 = prev_theta;
							prev_x = x_prime;//current_x;
							prev_y = y_prime;//current_y;
							prev_theta = theta_prime;//current_theta;
						}
					}//end of if there was sufficient movement
					else{
						this->myGazeboFile<<"\n"<<callback_time<<"\t"<<prev_x_2<<"\t"<<prev_y_2<<"\t"<<prev_theta_2<<"\t"<<prev_x<<"\t"<<prev_y<<"\t"<<prev_theta<<"\t"<<current_x<<"\t"<<current_y<<"\t"<<current_theta<<"\tInsufficient Movement\tInsufficient Movement\tInsufficientMovement";
						prev_x_2 = prev_x;
						prev_y_2 = prev_y;
						prev_theta_2 = prev_theta;
						prev_x = current_x;
						prev_y = current_y;
						prev_theta = current_theta;
					}
					this->world->SetPaused(false);
				}//End of time constraints for what happens
			}//end of 5 seconds thing
		}//end of timerCallback
		void timerCallbackMinor(const ros::TimerEvent& event){
			callback_time = ros::Time::now();
			this->world->SetPaused(true);
			this->timer.stop();//so that it only fires once
			//Gazebo model pose:
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name = "mobile_base";
			this->Getclient.call(getmodelstate);
			current_x = getmodelstate.response.pose.position.x;
			current_y = getmodelstate.response.pose.position.y;
			current_z = getmodelstate.response.pose.position.z;
			tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);
			current_theta = yaw;
			x_prime = current_x;// + 0.1;
			y_prime = current_y;// + 0.1;
			ROS_INFO("Minor Kidnapping. Moved the robot from %f, %f to %f, %f", current_x,current_y, x_prime, y_prime);
			//theta_prime = current_theta;// + 4.08;//10deg = 0.17 rad//original value
			//theta_prime = current_theta + 2.08; //120 degrees
			theta_prime = current_theta + 3.3; //190 degrees
			newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
			newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
			//Unroll theta_prime into a quaternion
			tf::Quaternion quat;
			quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH *YAW*
			newPose.orientation.x = quat.x();
			newPose.orientation.y = quat.y();
			newPose.orientation.z = quat.z();
			newPose.orientation.w = quat.w();
			//call gazebo set model state:
			gazebo_msgs::ModelState modelstate;
			modelstate.model_name = (std::string) "mobile_base";
			modelstate.reference_frame = (std::string) "world";
			modelstate.pose = newPose;
			gazebo_msgs::SetModelState setmodelstate;
			setmodelstate.request.model_state = modelstate;
			if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
				ROS_INFO("There's a nan here...1");
				this->world->SetPaused(false);
				prev_x_2 = prev_x;
				prev_y_2 = prev_y;
				prev_theta_2 = prev_theta;
				prev_x = current_x;
				prev_y = current_y;
				prev_theta = current_theta;
			}
			else{
				//ROS_INFO("%f", ros::Time::now().toSec());
				this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
				ROS_INFO("%f", ros::Time::now().toSec());
				ROS_INFO("CHANGED THETA FROM %f to %f", current_theta, theta_prime);
				this->myGazeboFile<<callback_time<<std::endl;
				prev_x_2 = prev_x;
				prev_y_2 = prev_y;
				prev_theta_2 = prev_theta;
				prev_x = x_prime;//current_x;
				prev_y = y_prime;//current_y;
				prev_theta = theta_prime;//current_theta;
				//Send a message that the event occurred.
				kidnap_msg.data = "KIDNAP OCCURRED.";
				kidnap_pub.publish(kidnap_msg);
			}
			this->world->SetPaused(false);
			//
		}//End of time constraints for what happens
		}//end of timerCallbackMinor
		void timerCallbackMajorOffMap(const ros::TimerEvent& event){
		ROS_INFO("INSIDE MAJOR EVENT TIMER CALLBACK!");
		callback_time = ros::Time::now();
		this->world->SetPaused(true);
		this->timer.stop();//so that it only fires once
		//Gazebo model pose:
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name = "mobile_base";
		this->Getclient.call(getmodelstate);
		current_x = getmodelstate.response.pose.position.x;
		current_y = getmodelstate.response.pose.position.y;
		current_z = getmodelstate.response.pose.position.z;
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		current_theta = yaw;
		kidnap_pub.publish(kidnap_msg);
		}
		this->world->SetPaused(false);
		//
		}//End of time constraints for what happens
		}//end of timerCallbackMinor
		void timerCallbackMajorOffMap(const ros::TimerEvent& event){
			ROS_INFO("INSIDE MAJOR EVENT TIMER CALLBACK!");
			callback_time = ros::Time::now();
			this->world->SetPaused(true);
			this->timer.stop();//so that it only fires once
			//Gazebo model pose:
			gazebo_msgs::GetModelState getmodelstate;
			getmodelstate.request.model_name = "mobile_base";
			this->Getclient.call(getmodelstate);
			current_x = getmodelstate.response.pose.position.x;
			current_y = getmodelstate.response.pose.position.y;
			current_z = getmodelstate.response.pose.position.z;
			tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);
			current_theta = yaw;
			187x_prime = current_x +1000;//+ 0.2;
			y_prime = current_y+1000;//+0.2;
			theta_prime = current_theta + 0.5;//10deg = 0.17 rad
			//x_prime = 0;
			//y_prime = 0;
			//theta_prime = 3.14;
			newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
			newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
			//Unroll theta_prime into a quaternion
			tf::Quaternion quat;
			quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH*YAW*
			newPose.orientation.x = quat.x();
			newPose.orientation.y = quat.y();
			newPose.orientation.z = quat.z();
			newPose.orientation.w = quat.w();
			//call gazebo set model state:
			gazebo_msgs::ModelState modelstate;
			modelstate.model_name = (std::string) "mobile_base";
			modelstate.reference_frame = (std::string) "world";
			modelstate.pose = newPose;
			gazebo_msgs::SetModelState setmodelstate;
			setmodelstate.request.model_state = modelstate;
			if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
				ROS_INFO("There's a nan here...1");
				kidnap_pub.publish(kidnap_msg);
			}
			this->world->SetPaused(false);
			//
		}//End of time constraints for what happens
		}//end of timerCallbackMinor
		void timerCallbackMajorOffMap(const ros::TimerEvent& event){
		ROS_INFO("INSIDE MAJOR EVENT TIMER CALLBACK!");
		callback_time = ros::Time::now();
		this->world->SetPaused(true);
		this->timer.stop();//so that it only fires once
		//Gazebo model pose:
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name = "mobile_base";
		this->Getclient.call(getmodelstate);
		current_x = getmodelstate.response.pose.position.x;
		current_y = getmodelstate.response.pose.position.y;
		current_z = getmodelstate.response.pose.position.z;
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		current_theta = yaw;
		187x_prime = current_x +1000;//+ 0.2;
		y_prime = current_y+1000;//+0.2;
		theta_prime = current_theta + 0.5;//10deg = 0.17 rad
		//x_prime = 0;
		//y_prime = 0;
		//theta_prime = 3.14;
		newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
		newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
		//Unroll theta_prime into a quaternion
		tf::Quaternion quat;
		quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH*YAW*
		newPose.orientation.x = quat.x();
		newPose.orientation.y = quat.y();
		newPose.orientation.z = quat.z();
		newPose.orientation.w = quat.w();
		//call gazebo set model state:
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (std::string) "mobile_base";
		modelstate.reference_frame = (std::string) "world";
		modelstate.pose = newPose;
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
			ROS_INFO("There's a nan here...1");
			this->world->SetPaused(false);
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = current_x;
			prev_y = current_y;
			prev_theta = current_theta;
		}
		else{
			//ROS_INFO("%f", ros::Time::now().toSec());
			this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
			//ROS_INFO("%f", ros::Time::now().toSec());
			ROS_INFO("Changed X and y from %f , %f to %f , %f",current_x, current_y,x_prime, y_prime);
			ROS_INFO("CHANGED THETA FROM %f to %f", current_theta, theta_prime);
			this->myGazeboFile<<callback_time<<std::endl;
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = x_prime;//current_x;
			prev_y = y_prime;//current_y;
			prev_theta = theta_prime;//current_theta;
			//Send a message that the event occurred.
			kidnap_msg.data = "KIDNAP OCCURRED.";
			ROS_INFO("SENDING MESSAGE");
			kidnap_pub.publish(kidnap_msg);
		}
		this->world->SetPaused(false);
		//}//End of time constraints for what happens
		}//end of timerCallbackMajorOffMap


		void timerCallbackMajorWithGlobalLocalization(const ros::TimerEvent& event){
		ROS_INFO("INSIDE MAJOR EVENT TIMER CALLBACK!");
		callback_time = ros::Time::now();
		this->world->SetPaused(true);
		this->timer.stop();//so that it only fires once
		//Gazebo model pose:
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name = "mobile_base";
		this->Getclient.call(getmodelstate);
		current_x = getmodelstate.response.pose.position.x;
		current_y = getmodelstate.response.pose.position.y;
		current_z = getmodelstate.response.pose.position.z;
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,
		getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,
		getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		current_theta = yaw;
		x_prime = -1;//current_x +1000;//+ 0.2;
		y_prime =-2;//current_y+1000;//+0.2;
		theta_prime = current_theta;// + 0.5;//10deg = 0.17 rad
		//x_prime = 0;
		//y_prime = 0;
		//theta_prime = 3.14;
		newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
		newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
		//Unroll theta_prime into a quaternion
		tf::Quaternion quat;
		quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH*YAW*
		newPose.orientation.x = quat.x();
		newPose.orientation.y = quat.y();
		newPose.orientation.z = quat.z();
		newPose.orientation.w = quat.w();
		//call gazebo set model state:
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (std::string) "mobile_base";
		modelstate.reference_frame = (std::string) "world";
		modelstate.pose = newPose;
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
			ROS_INFO("There's a nan here...1");
			this->world->SetPaused(false);
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = current_x;
			prev_y = current_y;
			prev_theta = current_theta;
		}
		else{
			//ROS_INFO("%f", ros::Time::now().toSec());
			this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
			std_srvs::Empty emptyArg;
			ROS_INFO("Calling global localizaiton");
			this->GlobalLocalizationClient.call(emptyArg);
			//ROS_INFO("%f", ros::Time::now().toSec());
			ROS_INFO("Changed X and y from %f , %f to %f , %f",current_x, current_y,x_prime, y_prime);
			ROS_INFO("CHANGED THETA FROM %f to %f", current_theta, theta_prime);
			this->myGazeboFile<<callback_time<<std::endl;
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = x_prime;//current_x;
			prev_y = y_prime;//current_y;
			prev_theta = theta_prime;//current_theta;
			//Send a message that the event occurred.
			kidnap_msg.data = "KIDNAP OCCURRED.";
			ROS_INFO("SENDING MESSAGE");
			kidnap_pub.publish(kidnap_msg);
		}
		this->world->SetPaused(false);
		//}//End of time constraints for what happens
		}//end of timerCallbackMajorWithGlobalLocalization
		void timerCallbackMajor(const ros::TimerEvent& event){
		ROS_INFO("INSIDE MAJOR EVENT TIMER CALLBACK!");
		callback_time = ros::Time::now();
		this->world->SetPaused(true);
		this->timer.stop();//so that it only fires once

		//Gazebo model pose:
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name = "mobile_base";
		this->Getclient.call(getmodelstate);
		current_x = getmodelstate.response.pose.position.x;
		current_y = getmodelstate.response.pose.position.y;
		current_z = getmodelstate.response.pose.position.z;
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		current_theta = yaw;
		x_prime = -1 ;//current_x +1000;//+ 0.2;
		y_prime =-2;//current_y+1000;//+0.2;
		theta_prime = current_theta;// + 0.5;//10deg = 0.17 rad
		//x_prime = 0;
		//y_prime = 0;
		//theta_prime = 3.14;
		newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
		newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
		//Unroll theta_prime into a quaternion
		tf::Quaternion quat;
		quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH*YAW*
		newPose.orientation.x = quat.x();
		newPose.orientation.y = quat.y();
		newPose.orientation.z = quat.z();
		newPose.orientation.w = quat.w();
		//call gazebo set model state:
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (std::string) "mobile_base";
		modelstate.reference_frame = (std::string) "world";
		modelstate.pose = newPose;
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
			ROS_INFO("There's a nan here...1");
			this->world->SetPaused(false);
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = current_x;
			prev_y = current_y;
			prev_theta = current_theta;
		}
		else{
			//ROS_INFO("%f", ros::Time::now().toSec());
			this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
			//
			//
			std_srvs::Empty emptyArg;
			ROS_INFO("Calling global localizaiton");
			this->GlobalLocalizationClient.call(emptyArg);
			//ROS_INFO("%f", ros::Time::now().toSec());
			ROS_INFO("Changed X and y from %f , %f to %f , %f",current_x, current_y,x_prime, y_prime);
			ROS_INFO("CHANGED THETA FROM %f to %f", current_theta, theta_prime);
			this->myGazeboFile<<callback_time<<std::endl;
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = x_prime;//current_x;
			prev_y = y_prime;//current_y;
			prev_theta = theta_prime;//current_theta;
			//Send a message that the event occurred.
			kidnap_msg.data = "KIDNAP OCCURRED.";
			ROS_INFO("SENDING MESSAGE");
			kidnap_pub.publish(kidnap_msg);
		}
		this->world->SetPaused(false);
		//}//End of time constraints for what happens
		}//end of timerCallbackMajor
		//This method collects data for evaluating the performance of the kidnapping mechanism.
		void timerCallbackRelocationUnitTest(const ros::TimerEvent& event) {
		callback_time = ros::Time::now();
		this->world->SetPaused(true);
		//Gazebo model pose:
		gazebo_msgs::GetModelState getmodelstate;
		getmodelstate.request.model_name = "mobile_base";
		this->Getclient.call(getmodelstate);
		current_x = getmodelstate.response.pose.position.x;
		current_y = getmodelstate.response.pose.position.y;
		current_z = getmodelstate.response.pose.position.z;
		tf::Quaternion q(getmodelstate.response.pose.orientation.x,getmodelstate.response.pose.orientation.y, getmodelstate.response.pose.orientation.z,getmodelstate.response.pose.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		current_theta = yaw;
		x_prime = 5.5;//+ 0.2;
		y_prime = 5.5;//+0.2;
		theta_prime = 0.5;//10deg = 0.17 rad
		newPose.position.x =x_prime;//prev_predicted_x; //_flat_tire;
		newPose.position.y =y_prime;//prev_predicted_y; //_flat_tire;
		//Unroll theta_prime into a quaternion
		tf::Quaternion quat;
		quat.setRPY(0, 0, theta_prime);//theta_prime);//prev_predicted_theta);//ROLL, PITCH*YAW*
		newPose.orientation.x = quat.x();
		newPose.orientation.y = quat.y();
		newPose.orientation.z = quat.z();
		newPose.orientation.w = quat.w();
		float beforeMoveTime;
		//call gazebo set model state:
		gazebo_msgs::ModelState modelstate;
		modelstate.model_name = (std::string) "mobile_base";
		modelstate.reference_frame = (std::string) "world";
		modelstate.pose = newPose;
		gazebo_msgs::SetModelState setmodelstate;
		setmodelstate.request.model_state = modelstate;
		if (std::isnan(modelstate.pose.position.x) || std::isnan(modelstate.pose.position.y) ||std::isnan(modelstate.pose.position.z) || std::isnan(modelstate.pose.orientation.x)||std::isnan(modelstate.pose.orientation.y) || std::isnan(modelstate.pose.orientation.z) ||std::isnan(modelstate.pose.orientation.w) ||std::isnan(modelstate.twist.linear.x) ||std::isnan(modelstate.twist.linear.y) || std::isnan(modelstate.twist.linear.z)||std::isnan(modelstate.twist.angular.x) || std::isnan(modelstate.twist.angular.y)||std::isnan(modelstate.twist.angular.z)){
			ROS_INFO("There's a nan here...1");
			this->world->SetPaused(false);
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = current_x;
			prev_y = current_y;
			prev_theta = current_theta;
		}
		else{
			//ROS_INFO("%f", ros::Time::now().toSec());
			beforeMoveTime = ros::Time::now().toSec();
			this->client.call(setmodelstate);//coment out if you DO NOT want to PUBLISH
			ROS_INFO("%f,%f,%f,%f,%f,%f,%f,%f",ros::Time::now().toSec(),ros::Time::now().toSec() - beforeMoveTime,current_x, current_y,current_z,x_prime,y_prime,theta_prime);
			//ROS_INFO("%f", ros::Time::now().toSec());
			this->myGazeboFile<<callback_time<<std::endl;
			prev_x_2 = prev_x;
			prev_y_2 = prev_y;
			prev_theta_2 = prev_theta;
			prev_x = x_prime;//current_x;
			prev_y = y_prime;//current_y;
			prev_theta = theta_prime;//current_theta;
			//Send a message that the event occurred.
			kidnap_msg.data = "KIDNAP OCCURRED.";
			kidnap_pub.publish(kidnap_msg);
		}
		this->world->SetPaused(false);
		//}//End of time constraints for what happens
		}//end of relocationUnitTest

		std::vector<double> calculate_new_pose(double theta0, double x0, double y0, double theta1,double x1, double y1, double theta2, double x2, double y2){
			double tx = x2, ty = y2, tth = theta2;
			//ROS_INFO("%f %f %f", theta0, theta1, theta2);
			std::vector<double> vintx_inty(2);
			std::vector<double> vxc_yc_mainradius(3);
			std::vector<double> vxc_yc_mainradiusTest;
			std::vector<double> vnewX_newY_newTheta(3);
			if((std::abs(theta0 - theta1) < .00174) || (std::abs(theta2 - theta1) < .00174)) {//angles are close enough to assume straight line - 1.74e-6 rad <-> .1 deg
				ROS_INFO("************Straight line!*********");
				vnewX_newY_newTheta[0] = (x2 -x1) + x2;
				vnewX_newY_newTheta[1] = (y2 - y1) + y2;
				vnewX_newY_newTheta[2] = theta2 + (theta2 -theta1);//*(3.14/180);//keep it the same
				vnewX_newY_newTheta[0] = x1 + (x1-x0);
				vnewX_newY_newTheta[1] = y1 + (y1-y0);
				vnewX_newY_newTheta[2] = theta1 + (theta1-theta0);
				vnewX_newY_newTheta[0] = x2;
				vnewX_newY_newTheta[1] = y2;
				vnewX_newY_newTheta[2] = theta2;

			}
			else{
				//Logic expects degrees. Convert rads to degrees
				theta0=theta0*(180/3.14);
				theta1 = theta1 * (180/3.14);
				theta2 = theta2*(180/3.14);
				vintx_inty = find_intersection(theta1, x1, y1, theta2, x2, y2);
				double intx = vintx_inty[0];
				double inty=vintx_inty[1];
				vxc_yc_mainradius=draw_circles(theta1,x1, y1, theta2, x2, y2, intx, inty);
				//Orig method
				//Draw Arcs
				vnewX_newY_newTheta=draw_arcs(theta1, x1, y1, theta2, x2, y2,vxc_yc_mainradius[0], vxc_yc_mainradius[1],vxc_yc_mainradius[2]);
			}
			return vnewX_newY_newTheta;
		}
		std::vector<double> draw_circles(double angleA, double posxA, double posyA, double angleB,double posxB, double posyB, double angleC, double posxC, double posyC){
			//This is EB's version:
			double xc = 0.0;
			double yc = 0.0;
			double yDelta_a = posyB-posyA;
			double xDelta_a = posxB-posxA;
			double yDelta_b = posyC-posyB;
			double xDelta_b = posxC-posxB;
			double aSlope = yDelta_b/xDelta_a;
			double bSlope = yDelta_b/xDelta_b;
			double AB_Mid_x = (posxA+posxB)/2;
			double AB_Mid_y = (posyA+posyB)/2;
			double BC_Mid_x = (posxB+posxC)/2;
			double BC_Mid_y = (posyB+posyC)/2;
			if (yDelta_a ==0){
				//aSlope == 0
				xc = AB_Mid_x;
				if (xDelta_b == 0){ //bSlope == Infinity
					yc = BC_Mid_y;
				}
				else{
					yc = BC_Mid_y + (BC_Mid_x - xc)/bSlope;
				}
			}
			else if (yDelta_b == 0){
				//bSlope == 0
				xc = BC_Mid_x;
				if (xDelta_a ==0){
					//aSlope == infinity
					yc = AB_Mid_y;
				}
				else {
					yc = AB_Mid_y + (AB_Mid_x - xc)/aSlope;
				}
			}
			else if (xDelta_a == 0){
				//aSlope == infinity
				yc = AB_Mid_y;
				xc = bSlope*(BC_Mid_y -yc) + BC_Mid_x;
			}
			else if (xDelta_b == 0){
				//bslope == infiinty
				yc = BC_Mid_y;
				xc = aSlope * (AB_Mid_y - yc) + AB_Mid_x;
			}
			else {
				xc = (aSlope * bSlope * (AB_Mid_y - BC_Mid_y) - aSlope * BC_Mid_x + bSlope*AB_Mid_x)/(bSlope -aSlope);
				yc = AB_Mid_y - (xc - AB_Mid_x)/aSlope;
			}

			double main_radius = sqrt(pow(posxA - xc, 2) + pow(posyA - yc,2));

			double ret[] = {xc,yc, main_radius};
			std::vector<double> v(ret, ret+sizeof(ret)/sizeof(double));
			return v;
		}
		std::vector<double> find_intersection(double angle1, double posx1, double posy1, double angle2, double posx2, double posy2){
			double ang1, ang2, px12, py12, px22, py22, x1, y1, x2, y2, x3, x4, y3, y4, den, intx, inty;
			double pi=3.14;
			//Calculate the angle at a 90degree angle to each point's angle
			ang1 = angle1 + 90;
			ang2 = angle2 + 90;
			px12 = posx1 + 10 * cos(ang1*2*pi/360);
			py12 = posy1 + 10 * sin(ang1*2*pi/360);
			px22 = posx2 + 10 * cos(ang2*2*pi/360);

			py22 = posy2 + 10 * sin(ang2*2*pi/360);
			x1 = posx1;
			y1 = posy1;
			x2 = px12;
			y2 = py12;
			x3 = posx2;
			y3 = posy2;
			x4 = px22;
			y4 = py22;
			//Calculating the _____. No idea.
			den = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));
			//if den is not 0
			if (std::abs(den) > .0001){
				intx = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)) / den;
				inty = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)) / den;
			}
			else{
				intx = std::numeric_limits<double>::quiet_NaN();
				inty = std::numeric_limits<double>::quiet_NaN();
			}
			std::vector<double> v={intx,inty};
			//v[0] = intx;
			//v[1] = inty;
			return v;
		}//end of find_intersection
		std::vector<double> draw_circles(double angle1,double posx1, double posy1, double angle2, double posx2, double posy2, double intx, double inty){
			//double intx=5;
			//double inty=5;
			//double* pointer;
			double radius, d, a, h, x0, y0, x1, y1, x2, y2, x3_1, y3_1, x3_2, y3_2, xc_a, yc_a, xc_b, yc_b, dist_a, dist_b, xc, yc,main_radius;
			//Calculate teh radius based on the distance formula between teh 2 points
			radius = sqrt(pow(posx1 - posx2, 2) + pow(posy1 - posy2,2));
			//find crossing points
			d = sqrt(pow(posx1 - posx2, 2) + pow(posy1 - posy2, 2));
			x0 = posx1;
			y0 = posy1;
			x1 = posx2;
			y1 = posy2;
			//middle point between the x's
			x2 = (x0 + x1)/2;
			//Interested in the +'s
			//middle point between the y's
			y2 = (y0 + y1)/2;
			//half of the radius
			a = d/2;
			//distance? btw radius and half the radius
			h = sqrt(pow(radius, 2) - pow(a,2));
			//midpoint of x's + dist btw radius and midpoint * midpoint of y's, divided by radius?
			//Is this chord-y stuff?
			x3_1 = x2 + h * ( y1 - y0 ) / d;
			y3_1 = y2 - h * ( x1 - x0 ) / d;
			x3_2 = x2 - h * ( y1 - y0 ) / d; //minusing the h*... this time. Was prev. +
			y3_2 = y2 + h * ( x1 - x0 ) / d;
			xc_a = (x3_1 + intx)/2;
			yc_a = (y3_1 + inty)/2;
			dist_a = sqrt(pow(x3_1 - xc_a, 2) + pow(y3_1 - yc_a, 2));
			xc_b = (x3_2 + intx)/2;
			yc_b = (y3_2 + inty)/2;
			dist_b = sqrt(pow(x3_2 - xc_b,2) + pow(y3_2 - yc_b,2));
			if (dist_a < dist_b){
				xc = xc_a;
				yc = yc_a;
			}
			else{
				xc = xc_b;
				yc = yc_b;
			}
			main_radius = sqrt(pow(posx1 - xc,2) + pow(posy1 - yc,2));
			xc=intx;
			yc=inty;
			main_radius = sqrt(pow(posx1 - xc,2) + pow(posy1-yc,2));
			//std::vector<double> v = {xc, yc, main_radius};
			/*v[0] = xc;
			v[1] = yc;
			v[2] = main_radius;
			*/
			double ret[] = {xc,yc, main_radius};
			std::vector<double> v(ret, ret+sizeof(ret)/sizeof(double));
			return v;
		}
		std::vector<double> draw_arcs(double angle1, double posx1, double posy1, double angle2,
		double posx2, double posy2, double centX, double centY, double radius){
		double* pointer;
		double projX1, projY1, projX2, projY2, ang1, ang2, px, py, dir, arc_deg, arc_len, new_rad, tmpang, new_arc_deg,new_centX, new_centY, newX, newY, newTheta, s1, s2;
		double pi = 3.14;
		projX1 = posx1 - centX;
		projY1 = posy1 - centY;
		projX2 = posx2 - centX;
		projY2 = posy2 - centY;
		ang1 = atan2(projY1,projX1);
		ang2 = atan2(projY2,projX2);
		//force angle to 0-2pi
		if (ang1 < 0){
			ang1 = ang1 + 2 * pi;
		}
		if (ang2 < 0){
			ang2 = ang2 + 2 * pi;
		}
		/*
		% use cross product to determine if the arc opens counter clockwise or
		% clockwise
		% a x b = ||a|| ||b|| sin(theta) n
		% a x b = [(i j k); (u1 u2 u3); (v1 v2 v3)];
		% a x b = [(i j k); (centX-posx1 centY-posy1 0); (1, tan(angle1) 0)]
		% a x b = posx1*pyk + posy1*px*(-k) % with above zeroz factored in
		*/
		px = cos(angle1*2*pi/360);
		py = sin(angle1*2*pi/360);
		dir = (centX-posx1)*py - (centY-posy1)*px; //in the k direction
		if (dir < 0){
			arc_deg = ang2 - ang1;
		}
		else{
			arc_deg = ang1 - ang2;
		}
		if (arc_deg < 0){
			arc_deg = arc_deg + 2*pi;
		}
		if (arc_deg > 2*pi){
			arc_deg = arc_deg - 2*pi;
		}
		//% arc length
		arc_len = arc_deg/(2*pi) * radius;
		//% radius = (rad_right + rad_left) / 2;
		//% radius ~ wheel_speed_right / wheel_speed_left;
		//% 10% change in one wheel speed results in 10% change in radius it's a 10% change in raidus? or is it 5?
		//Are we sure
		//%based on direction
		//%negative dir means turning left
		//%positive dir means turning right
		new_rad = radius * 0.8;//Change radius here! This determines the severity of the kidnapping orthe wheel slip event and symbolizes the reduction of the motion achieved
		new_arc_deg = arc_len * (2*pi) / (new_rad);
		tmpang = atan2((centY-posy1),(centX-posx1));
		new_centX = centX + (new_rad-radius)*cos(tmpang);
		new_centY = centY + (new_rad-radius)*sin(tmpang);
		/*
		%there is lots of room to play with the above math
		%depending on if the change is multiplicative or additive
		*/
		//%new robot pos
		if (dir < 0){
			newX = new_centX+new_rad*cos(ang1+new_arc_deg);
			newY = new_centY+new_rad*sin(ang1+new_arc_deg);
			newTheta = angle1*(2*pi/360) + new_arc_deg;
		}
		else{
			newX = new_centX+new_rad*cos(ang1-new_arc_deg);
			newY = new_centY+new_rad*sin(ang1-new_arc_deg);
			newTheta = angle1*(2*pi/360) - new_arc_deg;
		}
		// the original angle change + the change in angle change
		s1 = tan(angle1*2*pi/360); //What am I missing with this 2*pi/360?
		s2 = tan(angle2*2*pi/360);
		//find points to plot
		//Question: why are plots to point different than newX, newY?
		px = newX + 2 * cos(newTheta);
		py = newY + 2 * sin(newTheta);
		//std::vector<double> v = {newX, newY, newTheta};
		/*
		v[0] = newX;
		v[1] = newY;
		v[2] = newTheta;
		*/
		double ret[] = {newX,newY, newTheta};
		std::vector<double> v(ret, ret+sizeof(ret)/sizeof(double));
		return v;
		}
		private: transport::NodePtr node;
		private: transport::SubscriberPtr sub;
	};//end class
	//register plugin
	GZ_REGISTER_WORLD_PLUGIN(CallSpawnDeleteWorldPlugin)
}//end namespace
//END OF PLUGIN
