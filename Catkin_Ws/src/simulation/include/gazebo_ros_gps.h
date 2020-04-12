#ifndef GAZEBO_ROS_GPS_H
#define GAZEBO_ROS_GPS_H
/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: gazebo_ros_gps.h
* Author: Vijaymeyapan V
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: publishes gps values
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
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "sensor_model.h"
#include "update_timer.h"

//To be enabled if autonomai code is used
//#include "avp_vehicle_state_estimator/VseSensorMessages.h"
#include "simulation/gpsData.h"
#include <dynamic_reconfigure/server.h>
#include <simulation/GNSSConfig.h>

namespace gazebo
{

class GazeboRosGps : public ModelPlugin
{
public:
	GazeboRosGps();
	virtual ~GazeboRosGps();

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	virtual void Reset();
	virtual void Update();

	typedef simulation::GNSSConfig GNSSConfig;
	void dynamicReconfigureCallback(GNSSConfig &config, uint32_t level);

private:
	/// \brief The parent World
	physics::WorldPtr world;

	/// \brief The link referred to by this plugin
	physics::LinkPtr link;

	ros::NodeHandle* node_handle_;
	ros::Publisher fix_publisher_;
	ros::Publisher velocity_publisher_;

        simulation::gpsData fix_;

        //To be enabled if autonomai code is used
	//avp_vehicle_state_estimator::VseSensorMessages fix_;
	geometry_msgs::Vector3Stamped velocity_;

	std::string namespace_;
	std::string link_name_;
	std::string frame_id_;
	std::string fix_topic_;
	std::string velocity_topic_;

	double reference_latitude_;
	double reference_longitude_;
	double reference_heading_;
	double reference_altitude_;
	double update_rate_;

	double radius_north_;
	double radius_east_;

	SensorModel3 position_error_model_;
	SensorModel3 velocity_error_model_;

	UpdateTimer updateTimer;
	event::ConnectionPtr updateConnection;

	boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_position_, dynamic_reconfigure_server_velocity_;
	boost::shared_ptr<dynamic_reconfigure::Server<GNSSConfig> > dynamic_reconfigure_server_status_;
};

} // namespace gazebo

#endif // GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
