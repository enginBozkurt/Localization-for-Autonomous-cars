/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: wheel_plugin.cpp
* Author: Vijaymeyapan V
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: This file get the raw wheel speed data from simulator and publish through ros messages. 
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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
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
#include <simulation/wheelData.h>
#define WHEEL_RADIUS 0.3

using namespace std;


namespace gazebo
{
  
  class C_WheelPlugin : public ModelPlugin
  {
   
        public: C_WheelPlugin() 
        {
			/*checking for ros init*/
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "sc_vehiclecan_sim",ros::init_options::NoSigintHandler);
			}

		//ROS_INFO("ros initialized2222");
        }
		public: void onUpdate ( const common::UpdateInfo & _info ) 
		{
			/* looprate = 10hz */
			this->update_period = 0.1;
			common::Time current_time = this->world->GetSimTime();
			double seconds_since_last_update = ( current_time - last_update_time ).Double();
			if ( seconds_since_last_update > update_period ) 
			{
				/* function call for publishing wheel data from gazebo */
				wheelState();
				last_update_time+= common::Time ( update_period );
			}
		}
		public: void wheelState ()
		{
                        ros::Time current_time = ros::Time::now();
			msgs.header.frame_id = "wheel_base";
			msgs.header.stamp = ros::Time::now();
			msgs.header.seq = seq++;
#if 0
			ros::Time current_time = ros::Time::now();
			msgs.wheel_speed_sensor_time = ros::Time::now().toSec();
			msgs.left_front_wheel_speed = this->joint_FL->GetVelocity(0)*(0.3);
			msgs.right_front_wheel_speed = this->joint_FR->GetVelocity(0)*(0.3);
			msgs.left_rear_wheel_speed = this->joint_BL->GetVelocity(0)*(0.3);
			msgs.right_rear_wheel_speed = this->joint_BR->GetVelocity(0)*(0.3);
#endif
                        msgs.header.stamp = ros::Time::now();
			//msgs.left_front_wheel_speed_float_ = this->joint_FL->GetVelocity(0)*(WHEEL_RADIUS);
			//msgs.right_front_wheel_speed_float_ = this->joint_FR->GetVelocity(0)*(WHEEL_RADIUS);
			//msgs.left_rear_wheel_speed_float_ = this->joint_BL->GetVelocity(0)*(WHEEL_RADIUS);
			//msgs.right_rear_wheel_speed_float_ = this->joint_BR->GetVelocity(0)*(WHEEL_RADIUS);
			msgs.velocity =  sqrt(pow(this->model->GetRelativeLinearVel().x,2)+pow(this->model->GetRelativeLinearVel().y,2)+pow(this->model->GetRelativeLinearVel().z,2));			
			msgs.velocity_data_active =1;
			wheel_publisher.publish (msgs);
		}
		/*plugin body*/
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{

			/*checking for availability of joints*/
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "no joints available in vehicle\n";
				return;
			}
			this->model = _model;
			this->world = _model->GetWorld();
			/*creating joint pointers*/  
			physics::JointPtr wheel_joints[4];



			wheel_joints[0] = _model->GetJoint("EKF::hyundai::front_left_wheel_joint");
			wheel_joints[1] = _model->GetJoint("EKF::hyundai::front_right_wheel_joint");
			wheel_joints[2] = _model->GetJoint("EKF::hyundai::back_left_wheel_joint");
			wheel_joints[3] = _model->GetJoint("EKF::hyundai::back_right_wheel_joint");
			this->joint_FL = wheel_joints[0];
			this->joint_FR = wheel_joints[1];
			this->joint_BL = wheel_joints[2];
			this->joint_BR = wheel_joints[3];
			cout<< this->joint_FL->GetScopedName()<<endl;
			cout<< this->joint_FR->GetScopedName()<<endl;
			cout<< this->joint_BL->GetScopedName()<<endl;
			cout<< this->joint_BR->GetScopedName()<<endl;

			/*publisher initialization*/
			this->rosnode1 = boost::shared_ptr<ros::NodeHandle> (new ros::NodeHandle ("my_carvel1"));
                        //To be enabled for autonomai code
			//this->wheel_publisher = rosnode1->advertise<avp_vehicle_state_estimator::VseSensorMessages> ( "wheel_data",1000 );
                        this->wheel_publisher = rosnode1->advertise<simulation::wheelData> ( "wheel_data",1000 );

			this->last_update_time = this->world->GetSimTime();

			this->updateConnection = event::Events::ConnectWorldUpdateBegin (boost::bind 
				( &C_WheelPlugin::onUpdate, this, _1 ) );

		}
	
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		private: physics::WorldPtr world;
		private: physics::JointPtr joint_FL,joint_FR,joint_BL,joint_BR;
		private: boost::shared_ptr<ros::NodeHandle> rosnode1;
                private: simulation::wheelData msgs;
		private: ros::Publisher wheel_publisher;
		private: double update_rate;
		private: double update_period;
		private: common::Time last_update_time;
                private: double seq;
	

  };

  /*registering our model plugin*/
  GZ_REGISTER_MODEL_PLUGIN(C_WheelPlugin)
}

