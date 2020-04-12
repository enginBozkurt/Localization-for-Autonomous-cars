/**********************************************************************
* Company: Tata Elxsi Limited
* File Name: gazebo_ros_imu.cpp
* Author: Vijaymeyapan V
* Version: 1.0
* Date: 10-05-2017
* Operating Environment: Linux -ubuntu 16.04, ROS Kinetic distro.,gazebo8
* Compiler with Version Number: gcc version 4.8.4
* Description: This file get the raw pose and orientation data from simulator and publish through ros messages.
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
#include <gazebo_ros_imu.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include "simulation/imuData.h"
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>


namespace gazebo
{

// #define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
#include <geometry_msgs/PoseStamped.h>
static ros::Publisher debugPublisher;
#endif // DEBUG_OUTPUT
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
}


// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
	updateTimer.Disconnect(updateConnection);

	dynamic_reconfigure_server_accel_.reset();
	dynamic_reconfigure_server_rate_.reset();
	dynamic_reconfigure_server_yaw_.reset();

	node_handle_->shutdown();
	#ifdef USE_CBQ
	callback_queue_thread_.join();
	#endif
	delete node_handle_;
}


// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	// Get the world name.
	world = _model->GetWorld();

	// load parameters
	if (_sdf->HasElement("robotNamespace"))
		namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
	else
		namespace_.clear();

	if (_sdf->HasElement("bodyName"))
	{
		link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
		link = _model->GetLink(link_name_);
	}
	else
	{
		link = _model->GetLink();
		link_name_ = link->GetName();
	}

	// assert that the body by link_name_ exists
	if (!link)
	{
		ROS_FATAL("GazeboRosIMU plugin error: bodyName: %s does not exist\n", link_name_.c_str());
		return;
	}

	// default parameters
	frame_id_ = link_name_;
	topic_ = "imu";

	if (_sdf->HasElement("frameId"))
		frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

	if (_sdf->HasElement("topicName"))
		topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("biasTopicName"))
		bias_topic_ = _sdf->GetElement("biasTopicName")->GetValue()->GetAsString();
	else
		bias_topic_ = (!topic_.empty() ? topic_ + "/bias" : "");

	if (_sdf->HasElement("serviceName"))
		serviceName = _sdf->GetElement("serviceName")->GetValue()->GetAsString();
	else
		serviceName = topic_ + "/calibrate";
		
	if (_sdf->HasElement("ditectAngleFromImu"))
	{
		ditectAngleFromImu = _sdf->GetElement("ditectAngleFromImu")->GetValue()->GetAsString();
		topic_ditectAngleFromImu = topic_ + "_raw";
	}
	else
		ditectAngleFromImu = "true";
		
		


	accelModel.Load(_sdf, "accel");
	rateModel.Load(_sdf, "rate");
	yawModel.Load(_sdf, "yaw");

	// also use old configuration variables from gazebo_ros_imu
	if (_sdf->HasElement("gaussianNoise")) {
		double gaussianNoise;
		if (_sdf->GetElement("gaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
		accelModel.gaussian_noise = gaussianNoise;
		rateModel.gaussian_noise  = gaussianNoise;
		}
	}

	if (_sdf->HasElement("xyzOffset")) {
		this->offset_.pos = _sdf->Get<math::Vector3>("xyzOffset");
	} 
	else {
		ROS_INFO("imu plugin missing <xyzOffset>, defaults to 0s");
		this->offset_.pos = math::Vector3(0, 0, 0);
	}

	if (_sdf->HasElement("rpyOffset")) {
		this->offset_.rot = _sdf->Get<math::Vector3>("rpyOffset");
	} 
	else {
		ROS_INFO("imu plugin missing <rpyOffset>, defaults to 0s");
		this->offset_.rot = math::Vector3(0, 0, 0);
	}
	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
		return;
	}

	node_handle_ = new ros::NodeHandle(namespace_);

	// if topic name specified as empty, do not publish (then what is this plugin good for?)
	if (!topic_.empty())
        {
	if(ditectAngleFromImu == "true")
        pub_ = node_handle_->advertise<simulation::imuData>(topic_, 10);
        else
	pub1_ = node_handle_->advertise<sensor_msgs::Imu>(topic_ditectAngleFromImu, 10);
	}
	if (!bias_topic_.empty())
	bias_pub_ = node_handle_->advertise<sensor_msgs::Imu>(bias_topic_, 10);

	#ifdef DEBUG_OUTPUT
	debugPublisher = rosnode_->advertise<geometry_msgs::PoseStamped>(topic_ + "/pose", 10);
	#endif // DEBUG_OUTPUT

	// advertise services for calibration and bias setting
	if (!serviceName.empty())
	srv_ = node_handle_->advertiseService(serviceName, &GazeboRosIMU::ServiceCallback, this);

	accelBiasService = node_handle_->advertiseService(topic_ + "/set_accel_bias", &GazeboRosIMU::SetAccelBiasCallback, this);
	rateBiasService  = node_handle_->advertiseService(topic_ + "/set_rate_bias", &GazeboRosIMU::SetRateBiasCallback, this);

	// setup dynamic_reconfigure servers
	if (!topic_.empty()) {
		dynamic_reconfigure_server_accel_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/accel")));
		dynamic_reconfigure_server_rate_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/rate")));
		dynamic_reconfigure_server_yaw_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/yaw")));
		dynamic_reconfigure_server_accel_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &accelModel, _1, _2));
		dynamic_reconfigure_server_rate_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &rateModel, _1, _2));
		dynamic_reconfigure_server_yaw_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &yawModel, _1, _2));
	}

	#ifdef USE_CBQ
	// start custom queue for imu
	callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
	#endif

	Reset();

	// connect Update function
	updateTimer.Load(world, _sdf);
	updateConnection = updateTimer.Connect(boost::bind(&GazeboRosIMU::Update, this));
}

	void GazeboRosIMU::Reset()
	{
		updateTimer.Reset();

		orientation = math::Quaternion();
		velocity = 0.0;
		accel = 0.0;

		accelModel.reset();
		rateModel.reset();
		yawModel.reset();
	}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
	boost::mutex::scoped_lock scoped_lock(lock);
	rateModel.reset(math::Vector3(0.0, 0.0, 0.0));
	return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(simulation::SetBias::Request &req, simulation::SetBias::Response &res)
{
	boost::mutex::scoped_lock scoped_lock(lock);
	accelModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
	return true;
}

bool GazeboRosIMU::SetRateBiasCallback(simulation::SetBias::Request &req, simulation::SetBias::Response &res)
{
	boost::mutex::scoped_lock scoped_lock(lock);
	rateModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
	return true;
}


// Update the controller
void GazeboRosIMU::Update()
{
	// Get Time Difference dt
	common::Time cur_time = world->GetSimTime();
	double dt = updateTimer.getTimeSinceLastUpdate().Double();
	boost::mutex::scoped_lock scoped_lock(lock);

	// Get Pose/Orientation
	math::Pose pose = link->GetWorldPose();
	// math::Vector3 pos = pose.pos + this->offset_.pos;
	math::Quaternion rot = this->offset_.rot * pose.rot;
	rot.Normalize();

	// get Gravity
	gravity = world->GetPhysicsEngine()->GetGravity();
	double gravity_length = gravity.GetLength();
	ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);

	// get Acceleration and Angular Rates
	// the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
	//accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
	math::Vector3 temp = link->GetWorldLinearVel(); // get velocity in world frame
	if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt - gravity);
	velocity = temp;

	// calculate angular velocity from delta quaternion
	// note: link->GetRelativeAngularVel() sometimes return nan?
	// rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
	math::Quaternion delta = this->orientation.GetInverse() * rot;
	this->orientation = rot;
	if (dt > 0.0) {
	rate = this->offset_.rot.GetInverse()
	* (2.0 * acos(std::max(std::min(delta.w, 1.0), -1.0)) * math::Vector3(delta.x, delta.y, delta.z).Normalize() / dt);
	}


		//ROS_INFO("############################################################");
		//ROS_INFO("activate_yaw_rate : %s",activate_yaw_rate);
	//std::cout<<"############################################################"<<"\n";	
	//std::cout<<"activate_yaw_rate : "<<activate_yaw_rate<<"\n";	
	
	
	
	
	// update sensor models
	accel = accelModel(accel, dt);
	rate  = rateModel(rate, dt);
	yawModel.update(dt);
	ROS_DEBUG_NAMED("gazebo_ros_imu", "Current bias errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
	accelModel.getCurrentBias().x, accelModel.getCurrentBias().y, accelModel.getCurrentBias().z,
	rateModel.getCurrentBias().x, rateModel.getCurrentBias().y, rateModel.getCurrentBias().z,
	yawModel.getCurrentBias());
	ROS_DEBUG_NAMED("gazebo_ros_imu", "Scale errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
	accelModel.getScaleError().x, accelModel.getScaleError().y, accelModel.getScaleError().z,
	rateModel.getScaleError().x, rateModel.getScaleError().y, rateModel.getScaleError().z,
	yawModel.getScaleError());


	// apply accelerometer and yaw drift error to orientation (pseudo AHRS)
	math::Vector3 accelDrift = pose.rot.RotateVector(accelModel.getCurrentBias());
	double yawError = yawModel.getCurrentBias();
	math::Quaternion orientationError(
	math::Quaternion(cos(yawError/2), 0.0, 0.0, sin(yawError/2)) *                                         // yaw error
	math::Quaternion(1.0, 0.5 * accelDrift.y / gravity_length, 0.5 * -accelDrift.x / gravity_length, 0.0)  // roll and pitch error
	);
	orientationError.Normalize();
	rot = orientationError * rot;

	// copy data into pose message
	biasMsg.header.frame_id = frame_id_;
	biasMsg.header.stamp.sec = cur_time.sec;
	biasMsg.header.stamp.nsec = cur_time.nsec;

	// orientation quaternion
	tf::Quaternion quat(rot.x,rot.y,rot.z,rot.w);
	double roll_b, pitch_b, yaw_b;
	tf::Matrix3x3(quat).getRPY(roll_b, pitch_b, yaw_b);
	angular_rate  = link->GetWorldAngularVel();
	


	if(ditectAngleFromImu == "true")
	{
	imuMsg.header.frame_id = frame_id_;
	imuMsg.header.stamp.sec = cur_time.sec;
	imuMsg.header.stamp.nsec = cur_time.nsec;
	imuMsg.header.seq = seq++;
	imuMsg.roll=roll_b;
	imuMsg.pitch=pitch_b;
	imuMsg.yaw=yaw_b;
	// pass accelerations
	imuMsg.imu.linear_acceleration.x     = accel.x;
	imuMsg.imu.linear_acceleration.y    = accel.y;
	imuMsg.imu.linear_acceleration.z    = accel.z;
	
	imuMsg.imu_active=1;
	imuMsg.imu_setteled=1;
	// publish to ros
	pub_.publish(imuMsg);
	//std::cout<<"angle"<<"\n";
	
	}
	else
	{	
	
	defaultImuMsg.header.frame_id = frame_id_;
	defaultImuMsg.header.stamp.sec = cur_time.sec;
	defaultImuMsg.header.stamp.nsec = cur_time.nsec;
	defaultImuMsg.header.seq = seq1++;
	defaultImuMsg.orientation.x = rot.x;
	defaultImuMsg.orientation.y = rot.y;
	defaultImuMsg.orientation.z = rot.z;
	defaultImuMsg.orientation.w = rot.w;
	defaultImuMsg.angular_velocity.x = angular_rate.x;
	defaultImuMsg.angular_velocity.y = angular_rate.y;
	defaultImuMsg.angular_velocity.z = angular_rate.z;
	defaultImuMsg.linear_acceleration.x = accel.x;
	defaultImuMsg.linear_acceleration.y = accel.y;
	defaultImuMsg.linear_acceleration.z = accel.z;
	pub1_.publish(defaultImuMsg);
	//std::cout<<"angular_rate : "<<angular_rate.x<<" ,"<<angular_rate.y<<" ,"<<angular_rate.z<<"\n";

		
	}
	ROS_DEBUG_NAMED("gazebo_ros_imu", "Publishing IMU data at t = %f", cur_time.Double());

	// publish bias
	if (bias_pub_) {
	biasMsg.header = biasMsg.header;
	biasMsg.orientation.x = orientationError.x;
	biasMsg.orientation.y = orientationError.y;
	biasMsg.orientation.z = orientationError.z;
	biasMsg.orientation.w = orientationError.w;
	biasMsg.angular_velocity.x = rateModel.getCurrentBias().x;
	biasMsg.angular_velocity.y = rateModel.getCurrentBias().y;
	biasMsg.angular_velocity.z = rateModel.getCurrentBias().z;
	biasMsg.linear_acceleration.x = accelModel.getCurrentBias().x;
	biasMsg.linear_acceleration.y = accelModel.getCurrentBias().y;
	biasMsg.linear_acceleration.z = accelModel.getCurrentBias().z;
	bias_pub_.publish(biasMsg);
	}

	// debug output
	#ifdef DEBUG_OUTPUT
	if (debugPublisher) {
	geometry_msgs::PoseStamped debugPose;
	debugPose.header = biasMsg.header;
	debugPose.header.frame_id = "/map";
	debugPose.pose.orientation.w = rot.w;
	debugPose.pose.orientation.x = rot.x;
	debugPose.pose.orientation.y = rot.y;
	debugPose.pose.orientation.z = rot.z;
	math::Pose pose = link->GetWorldPose();
	debugPose.pose.position.x = pose.pos.x;
	debugPose.pose.position.y = pose.pos.y;
	debugPose.pose.position.z = pose.pos.z;
	debugPublisher.publish(debugPose);
	}
#endif // DEBUG_OUTPUT
}



#ifdef USE_CBQ
void GazeboRosIMU::CallbackQueueThread()
{
	static const double timeout = 0.01;

	while (rosnode_->ok())
	{
		callback_queue_.callAvailable(ros::WallDuration(timeout));
	}
}
#endif

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

} // namespace gazebo
