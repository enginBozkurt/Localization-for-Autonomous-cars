#ifndef GAZEBO_ROS_IMU_H
#define GAZEBO_ROS_IMU_H
/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: gazebo_ros_imu.h
* Author: Vijaymeyapan V
* Version: 1.0
* Date: 
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: publishes sonar values
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
// #define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/common/Plugin.hh>
#include<tf/transform_datatypes.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <simulation/SetBias.h>
#include <sensor_model.h>
#include <update_timer.h>
#include "simulation/imuData.h"
//To be enabled if autonomai code is used
//#include "avp_vehicle_state_estimator/VseSensorMessages.h"
#include <dynamic_reconfigure/server.h>

namespace gazebo
{
	class GazeboRosIMU : public ModelPlugin
	{
	public:
		/// \brief Constructor
		GazeboRosIMU();

		/// \brief Destructor
		virtual ~GazeboRosIMU();

	protected:
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual void Reset();
		virtual void Update();

	private:
		/// \brief The parent World
		physics::WorldPtr world;

		/// \brief The link referred to by this plugin
		physics::LinkPtr link;

		/// \brief pointer to ros node
		ros::NodeHandle* node_handle_;
		ros::Publisher pub_,pub1_;
		ros::Publisher bias_pub_;

		/// \brief ros message
                simulation::imuData imuMsg;
                sensor_msgs::Imu defaultImuMsg;
		sensor_msgs::Imu biasMsg;

		/// \brief store link name
		std::string link_name_;

		/// \brief frame id
		std::string frame_id_;
		double seq=0,seq1=0;
		
		/// \brief topic name
		std::string topic_;
		std::string topic_ditectAngleFromImu;
		std::string bias_topic_;
		
		/// \brief yaw sensor selection
		std::string ditectAngleFromImu;

		/// \brief allow specifying constant xyz and rpy offsets
		math::Pose offset_;

		/// \brief Sensor models
		SensorModel3 accelModel;
		SensorModel3 rateModel;
		SensorModel yawModel;

		/// \brief A mutex to lock access to fields that are used in message callbacks
		boost::mutex lock;

		/// \brief save current body/physics state
		math::Quaternion orientation;
		math::Vector3 velocity;
		math::Vector3 accel;
		math::Vector3 rate;
		math::Vector3 gravity;
		math::Vector3 angular_rate;

		/// \brief Gaussian noise generator
		double GaussianKernel(double mu,double sigma);

		/// \brief for setting ROS name space
		std::string namespace_;

		/// \brief call back when using service
		bool ServiceCallback(std_srvs::Empty::Request &req,
						            std_srvs::Empty::Response &res);
		ros::ServiceServer srv_;
		std::string serviceName;

		/// \brief Bias service callbacks
		bool SetAccelBiasCallback(simulation::SetBias::Request &req, simulation::SetBias::Response &res);
		bool SetRateBiasCallback(simulation::SetBias::Request &req, simulation::SetBias::Response &res);
		ros::ServiceServer accelBiasService;
		ros::ServiceServer rateBiasService;

		#ifdef USE_CBQ
		ros::CallbackQueue callback_queue_;
		void CallbackQueueThread();
		boost::thread callback_queue_thread_;
		#endif

		UpdateTimer updateTimer;
		event::ConnectionPtr updateConnection;

		boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_accel_, dynamic_reconfigure_server_rate_, dynamic_reconfigure_server_yaw_;
	};
}

#endif // GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
