/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: car_controller_key_plugin.cpp
* Author: Mahadevaprabhu N , Vijaymeyapan V
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: controls car using keyboard input
* List of functions used: 
* Revisers Name:
* Date:
* Customer Bug No./ CMF No. :
* Brief description of the fix/enhancement: Replacement of constant numbers with macros. Deleting commented code.
* Created by Tata Elxsi Ltd., < Automotive Group >
* Copyright <Year>Tata Elxsi Ltd.
* All rights reserved.
This code contains information that is proprietary to Tata Elxsi Ltd.
No part of this document/code may be reproduced or used in whole or
part in any form or by any means - graphic, electronic or mechanical
without the written permission of Tata Elxsi Ltd
**********************************************************************/
#define FRICTION 700.00

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <thread>
#include <gazebo/common/common.hh>
#include <simulation/SimJoystickAndKeyboard.h>
#include <math.h>
#include <algorithm>
#include <gazebo/physics/JointWrench.hh>
#include <stdlib.h>

using namespace std;
double g_dKeyVelFrontLeft=0, g_dKeyVelFrontRight=0, g_dKeySteerAngleLeft=0, g_dKeySteerAngleRight = 0, g_dKeyTorqueUp=0, g_dKeyTorqueDown=0;
double g_dCarLength = 3.61;
double g_dCarWidth = 2;
double g_dWmax = 4000;
double g_dTmax = 1000;
double g_dBeta = 0.2;
double g_dRadius = 0.45;
int nIterator;
double g_adGearRatio[] = {4.24,2.52,1.66,1.22,1};
double g_dW,g_dKeyTorque=0,g_dKeyVelocityAverage = 0;
double up=0 ,down = 0;
double g_dBrakeAxisScaled=0;
double g_dPreviousAngle=0,angle =0;
double g_dMomentOfInertia = 0.4+5137;
double g_dKeySteeringAngle=0;

std::vector<gazebo::physics::ModelPtr> model_list;
std::vector<gazebo::physics::ModelPtr> ped_list;

//gazebo plugin class 
namespace gazebo
{
  
class CCarControllerPluginKey : public ModelPlugin
{
		public: CCarControllerPluginKey() 
     	{
			/*checking for ros init*/
			if (!ros::isInitialized())
    		{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_clien_key",ros::init_options::NoSigintHandler);
    		}

	    	ROS_INFO("Ros initialized");
        }
	
		/*subscriber part : topic name /bmw/vel_cmd_vel*/
		public: void velSub(const simulation::SimJoystickAndKeyboard::ConstPtr &_msg)
		{		

   			g_dKeySteeringAngle = _msg->key_steering_angle;
			//calculating average velocity
			g_dKeyVelocityAverage = sqrt(pow(this->model->GetRelativeLinearVel().x,2)+pow(this->model->GetRelativeLinearVel().y,2)+pow(this->model->GetRelativeLinearVel().z,2))+0.01;
			//depending on velocity setting the nIterator
			if (g_dKeyVelocityAverage < 10 && g_dKeyVelocityAverage > 0)
			nIterator=0;
			else if (g_dKeyVelocityAverage < 20 && g_dKeyVelocityAverage > 10)
			nIterator=1;
			else if (g_dKeyVelocityAverage < 30 && g_dKeyVelocityAverage > 20)
			nIterator=2;
			else if (g_dKeyVelocityAverage < 48 && g_dKeyVelocityAverage > 30)
			nIterator=3;
			else if (g_dKeyVelocityAverage > 48)
			nIterator=4;

                        if(_msg->up >= 0)
                        {
			   g_dW= ((g_adGearRatio[nIterator]*g_dKeyVelocityAverage)/g_dRadius);
			   g_dKeyTorqueUp =(_msg->up * (g_dTmax*(1-(g_dBeta*(pow(((g_dW/g_dWmax)-1),2)) ))) );
                        }
                        else
                        {
			   nIterator=0;
			   g_dW= ((g_adGearRatio[nIterator]*g_dKeyVelocityAverage)/g_dRadius);
			   g_dKeyTorqueUp = (_msg->up * (g_dTmax*(1-(g_dBeta*(pow(((g_dW/g_dWmax)-1),2)) ))) );
                        }
	    	        //ROS_INFO("Torque = %lf\n",g_dKeyTorqueUp);
			g_dBrakeAxisScaled = _msg->down;
			
			//g_dKeyTorqueDown = ((torque_fl+ torque_fr)/2)*_msg->down;
			g_dKeyTorque = g_dKeyTorqueUp;
			g_dKeyVelFrontLeft=_msg->mult_fl2*g_dKeyTorque ;
			g_dKeyVelFrontRight=_msg->mult_fr2*g_dKeyTorque;
			g_dKeySteerAngleLeft =_msg->steering_left_2;
			g_dKeySteerAngleRight= _msg->steering_right_2;
               
		}

		/*update rate in ros*/
		private: void queue_thread()
		{
			static const double timeout = 0.1;
			while (this->ros_node->ok())
			{
				this->ros_queue.callAvailable(ros::WallDuration(timeout));
			}
		}
   		public: void onUpdate ( const common::UpdateInfo & _info ) 
		{	//cout<<"-----------------------"<<g_dKeyVelFrontLeft<<"\n";	
			this->update_period = 0.1;
			common::Time current_time = this->world->GetSimTime();
			double seconds_since_last_update = ( current_time - last_update_time ).Double();
			if ( seconds_since_last_update > update_period ) 
			{
				wheelState();
				last_update_time+= common::Time ( update_period );
			}
			this->model->GetJointController()->SetPositionPID(this->steer_joint1_key->GetScopedName(), this->pid);
			this->model->GetJointController()->SetPositionPID(this->steer_joint2_key->GetScopedName(), this->pid);
		        this->model->GetJointController()->SetPositionTarget(this->steer_joint1_key->GetScopedName(),g_dKeySteerAngleLeft);
			this->model->GetJointController()->SetPositionTarget(this->steer_joint2_key->GetScopedName(),g_dKeySteerAngleRight);
			this->joint0_key->SetForce(0, 1*g_dKeyTorque);//g_dKeyVelFrontLeft);
			this->joint1_key->SetForce(0, 1*g_dKeyTorque);//g_dKeyVelFrontRight);
			if (g_dBrakeAxisScaled != 0 && this->joint2_key->GetVelocity(0) >= 0 && this->joint3_key->GetVelocity(0) >= 0)
		        {
				this->joint2_key->SetForce(0,FRICTION*g_dBrakeAxisScaled );
				this->joint3_key->SetForce(0,FRICTION*g_dBrakeAxisScaled ); 
				this->joint0_key->SetForce(0,FRICTION*g_dBrakeAxisScaled );
				this->joint1_key->SetForce(0,FRICTION*g_dBrakeAxisScaled ); 
			}
                        else if(g_dBrakeAxisScaled != 0 && this->joint2_key->GetVelocity(0) < 0 && this->joint3_key->GetVelocity(0) < 0)
                        {
				this->joint2_key->SetForce(0,-1*FRICTION*g_dBrakeAxisScaled );
				this->joint3_key->SetForce(0,-1*FRICTION*g_dBrakeAxisScaled ); 
				this->joint0_key->SetForce(0,-1*FRICTION*g_dBrakeAxisScaled );
				this->joint1_key->SetForce(0,-1*FRICTION*g_dBrakeAxisScaled ); 
                        }
		}
		public: void wheelState ()
		{
			msgs.data =  g_dKeySteeringAngle ;
			wheel_publisher.publish (msgs);
		}

		/*plugin body*/
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			/*checking for availability of joints*/
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "no joints available in bmw\n";
				return;
	        	}
			this->model = _model;
			this->world = _model->GetWorld();

			model_list.push_back(_model);	

			/*creating joint pointers*/  
			physics::JointPtr steering_joints_key[6];
			steering_joints_key[0] = _model->GetJoint("EKF::hyundai::back_left_wheel_joint");
			steering_joints_key[1] = _model->GetJoint("EKF::hyundai::back_right_wheel_joint");
			steering_joints_key[2] = _model->GetJoint("EKF::hyundai::front_left_steering_joint");
			steering_joints_key[3] = _model->GetJoint("EKF::hyundai::front_right_steering_joint");
			steering_joints_key[4] = _model->GetJoint("EKF::hyundai::front_left_wheel_joint");
			steering_joints_key[5] = _model->GetJoint("EKF::hyundai::front_right_wheel_joint");

			this->joint0_key = steering_joints_key[4];
			this->joint1_key = steering_joints_key[5];
			this->joint2_key = steering_joints_key[0];
			this->joint3_key = steering_joints_key[1];
			this->steer_joint1_key = steering_joints_key[2];
			this->steer_joint2_key = steering_joints_key[3];
			cout<< this->joint0_key->GetScopedName()<<endl;
			cout<< this->joint1_key->GetScopedName()<<endl;
			cout<< this->steer_joint1_key->GetScopedName()<<endl;
			cout<< this->steer_joint2_key->GetScopedName()<<endl;
			cout<< this->joint2_key->GetScopedName()<<endl;
			cout<< this->joint3_key->GetScopedName()<<endl;

			/*setting p controller (not needed)*/
			this->pid = common::PID(100000, 0, 0);	  
		

			/*creating subscriber in ros*/
			this->ros_node.reset(new ros::NodeHandle("gazebo_clien_key"));

			ros::SubscribeOptions so =ros::SubscribeOptions::create<simulation::SimJoystickAndKeyboard>
				("/bmw/vel_cmd_key",1,boost::bind(&CCarControllerPluginKey::velSub, this, _1),ros::VoidPtr(), &this->ros_queue);
			this->ros_sub = this->ros_node->subscribe(so);


			this->ros_queue_thread =std::thread(std::bind(&CCarControllerPluginKey::queue_thread, this));

			this->ros_node1 = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( "my_carvel1" ) );
			this->wheel_publisher = ros_node1->advertise<std_msgs::Float32> ( "/wheel_data",1000 );

			this->last_update_time = this->world->GetSimTime();
			//calls subsriber for every onUpdate event
			this->update_connection = event::Events::ConnectWorldUpdateBegin (boost::bind 
				( &CCarControllerPluginKey::onUpdate, this, _1 ) );

		}
		private: physics::ModelPtr model;
		private: event::ConnectionPtr update_connection;
		private: physics::WorldPtr world;
		private: physics::JointPtr joint0_key,joint1_key,joint2_key,joint3_key,steer_joint1_key,steer_joint2_key; //joint pointers
		private: physics::JointWrench jointwr; 
		private: common::PID pid;
		private: std::unique_ptr<ros::NodeHandle> ros_node;
		private: ros::Subscriber ros_sub; //ROS subsriber
		private: ros::CallbackQueue ros_queue;
		private: std::thread ros_queue_thread;
		private: boost::shared_ptr<ros::NodeHandle> ros_node1;
		private: std_msgs::Float32 msgs;
		private: simulation::SimJoystickAndKeyboard msgs1;
		private: ros::Publisher wheel_publisher;
		private: double update_rate;
		private: double update_period;
		private: common::Time last_update_time;
		private: physics::JointController * j2_controller;

  	};

 //registering our model plugin
  GZ_REGISTER_MODEL_PLUGIN(CCarControllerPluginKey)
}

