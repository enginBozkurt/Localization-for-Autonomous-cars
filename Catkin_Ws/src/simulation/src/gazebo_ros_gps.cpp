#include "gazebo_ros_gps.h"
#include <gazebo/physics/physics.hh>

//To be enabled if autonomai code is used
#include "navsat_conversions.h"
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <tf/transform_datatypes.h>

using namespace std;
// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_x  = 379085.84;
static const double DEFAULT_REFERENCE_y  = 6674234.46;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;
int flag_gps_active = 0;
int flag_first =0;
int offset_count = 0;
double reference_x_ =0;
double reference_y_ =0;
string reference_zone_ = "35V";
//string reference_zone_ = "21p";
double seq = 0 ;
double prev_lat = 0,prev_long = 0,current_lat,current_long,gps_heading_angle = 0,prev_gps_heading_angle = 0;
static int i=1;
double RADIAN = (3.1415926535/180);
bool latlongflag = false;

fstream offset;

namespace gazebo {

GazeboRosGps::GazeboRosGps()
{
}
// Destructor
GazeboRosGps::~GazeboRosGps()
{
	updateTimer.Disconnect(updateConnection);

	dynamic_reconfigure_server_position_.reset();
	dynamic_reconfigure_server_velocity_.reset();
	dynamic_reconfigure_server_status_.reset();

	node_handle_->shutdown();
	delete node_handle_;
}
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	world = _model->GetWorld();

	// load parameters
	if (!_sdf->HasElement("robotNamespace"))
		namespace_.clear();
	else
		namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

	if (!_sdf->HasElement("bodyName"))
	{
		link = _model->GetLink();
		link_name_ = link->GetName();
	}
	else {
		link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
		link = _model->GetLink(link_name_);
	}

	if (!link)
	{
		ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
		return;
	}

	// default parameters
	frame_id_ = "/world";
	fix_topic_ = "fix";
	velocity_topic_ = "fix_velocity";
	
	reference_x_  = DEFAULT_REFERENCE_x;
	reference_y_ = DEFAULT_REFERENCE_y;
		
	reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
	reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

	if (_sdf->HasElement("frameId"))
		frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

	if (_sdf->HasElement("topicName"))
		fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("velocityTopicName"))
		velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

	if (_sdf->HasElement("referenceLatitude"))
		_sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_x_);

	if (_sdf->HasElement("referenceLongitude"))
		_sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_y_);

	if (_sdf->HasElement("referenceHeading"))
	if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
	reference_heading_ *= M_PI/180.0;

	if (_sdf->HasElement("referenceAltitude"))
		_sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
	velocity_.header.frame_id = frame_id_;

	position_error_model_.Load(_sdf);
	velocity_error_model_.Load(_sdf, "velocity");

	// Make sure the ROS node for Gazebo has already been initialized
	if (!ros::isInitialized())
	{
		ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
		return;
	}

	node_handle_ = new ros::NodeHandle(namespace_);
	fix_publisher_ = node_handle_->advertise<simulation::gpsData>(fix_topic_, 10);
	velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

	// setup dynamic_reconfigure servers
	dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
	dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
	dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
	dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
	dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));
	dynamic_reconfigure_server_status_->setCallback(boost::bind(&GazeboRosGps::dynamicReconfigureCallback, this, _1, _2));

	Reset();

	// connect Update function
	updateTimer.setUpdateRate(1.0);
	updateTimer.Load(world, _sdf);
	updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::Reset()
{
	updateTimer.Reset();
	position_error_model_.reset();
	velocity_error_model_.reset();
}

void GazeboRosGps::dynamicReconfigureCallback(GazeboRosGps::GNSSConfig &config, uint32_t level)
{
  using sensor_msgs::NavSatStatus;
}


// Update the controller
void GazeboRosGps::Update()
{
	common::Time sim_time = world->GetSimTime();
	double dt = updateTimer.getTimeSinceLastUpdate().Double();

	math::Pose pose = link->GetWorldPose();

	gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
	gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);


	position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());


	fix_.gps_x = reference_x_  +  position.x ;
	fix_.gps_y = reference_y_  +  position.y ;
	
	
	double roll_b, pitch_b, yaw_b;
	math::Quaternion rot  = pose.rot;
	tf::Quaternion quat(rot.x,rot.y,rot.z,rot.w);
	tf::Matrix3x3(quat).getRPY(roll_b, pitch_b, yaw_b);
	
		
	
	
	if(yaw_b < 0)
	{
	yaw_b = yaw_b+3.14+3.14;
	}
	
	
	yaw_b =(3.14+3.14) - yaw_b;  
	
	//cout<<"yaw_b  : "<<yaw_b *(180/3.14)<<endl;
	fix_.gps_heading = yaw_b;
	RobotLocalization::NavsatConversions::UTMtoLL(fix_.gps_y ,fix_.gps_x,
                           reference_zone_, fix_.nav_sat_fix.latitude,  fix_.nav_sat_fix.longitude );
	//cout<<"lat gps "<<fix_.nav_sat_fix.latitude<<"lon gps "<<fix_.nav_sat_fix.longitude<<endl;
	//cout<<"mercator : "<<fix_.latitude<<" ,"<<fix_.longitude<<" , "<<reference_zone_<<endl;
	//cout<<"xy  : "<<fix_.gps_x<<" ,"<<fix_.gps_y<<endl;
	
	fix_.nav_sat_fix.altitude  = reference_altitude_  + position.z;
	fix_.gps_z = 0;
	fix_.utm_zone = reference_zone_;
	velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
	velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
	velocity_.vector.z = velocity.z;
	flag_gps_active=flag_gps_active+1;
	
	
	if(flag_gps_active >= 5)
	{
		flag_gps_active=10;
		fix_.gps_active =1;
		fix_.gps_fixed =1;
		fix_.header.seq = seq++;
	}

	common::Time cur_time = world->GetSimTime();
	fix_.header.frame_id = frame_id_;
	fix_.header.stamp.sec = cur_time.sec;
	fix_.header.stamp.nsec = cur_time.nsec;

	fix_publisher_.publish(fix_);
	velocity_publisher_.publish(velocity_);
}



// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
