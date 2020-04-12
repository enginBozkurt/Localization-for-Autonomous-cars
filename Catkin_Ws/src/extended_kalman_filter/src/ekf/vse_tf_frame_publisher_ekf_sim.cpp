
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include "simulation/gpsData.h"
#include "simulation/imuData.h"
#include "simulation/wheelData.h"
#include "extended_kalman_filter/VSEFusionData.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include "navsat_conversions.hpp"
#include <math.h>
using namespace std;    	
	
typedef struct
	{
	string frame_id;
	double count;
	ros::Time time;
	double X ;
	double Y ;
	double Z ;
	} vse_tf_point;
	
typedef struct
	{
	string frame_id;
	double count;
	ros::Time time;
	double X ;
	double Y ;
	double Z ;
	string utm_zone;
	} UTM_mercator;
	
typedef struct
	{
	string frame_id;
	double count;
	ros::Time time;
	double roll ;
	double pitch ;
	double yaw ;
	} vse_tf_RPY;

double pub_flag =0;
double dImuOffsetData =0;
double g_nImuAngleAvailability =0;
double g_gps_heading =0;


bool fnCreateFrame(tf::TransformBroadcaster &broadcaster,geometry_msgs::PoseStamped &pose, string parent, string child,ros::Time &now)
{

	tf::Quaternion quat(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
	tf::Vector3 position(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z) ;
	now = ros::Time::now();
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat, position), now ,parent, child));
	return 1;
}

bool fnPositionTransform( geometry_msgs::PoseStamped &source, geometry_msgs::PoseStamped &destination,string parent ,string child , string targetFrame, ros::Time &now ,tf::TransformListener &listener_obj)
{

	try 
	{
	listener_obj.waitForTransform(parent, child,now, ros::Duration(0.001));
	listener_obj.transformPose(targetFrame, source, destination);
	} 
	catch (tf::TransformException ex) 
	{
     	ROS_ERROR(" vse_tf_framepublisher.cpp - Position Transformation : %s", ex.what());
   	}
	 ROS_DEBUG_STREAM("Transformation between "<<parent << " to "<<child<<" data : "<<source);
	
	return 1;
		 
}


bool fnVectorTransform( geometry_msgs::Vector3Stamped &source, geometry_msgs::Vector3Stamped &destination,string parent ,string child , string targetFrame, ros::Time &now ,tf::TransformListener &listener_obj)
{
	try 
	{
	listener_obj.waitForTransform(parent, child,now, ros::Duration(0.001));
	listener_obj.transformVector(targetFrame, source, destination);	
	} 
	catch (tf::TransformException ex) 
	{
     ROS_ERROR("Vector Transformation %s", ex.what());
    }	 
	ROS_DEBUG_STREAM("Transformation between "<<parent << " to "<<child<<" data : "<<source);
	 
	return 1;
}
static bool gps_flag=0;
static bool imu_flag =0 ;
void gps_callback(const simulation::gpsData::ConstPtr& msg, UTM_mercator *gps_point, UTM_mercator *gps_initial_point,vse_tf_point *gps_covariance,vse_tf_point *gps_offset_point, vse_tf_RPY *imu_angle,vse_tf_RPY *imu_initial_angle)
{
	gps_point->frame_id = msg->header.frame_id;
	gps_point->count = msg->header.seq;
	gps_point->time = msg -> header.stamp;
	gps_point->X = msg->gps_x + (gps_offset_point->X * cos (-msg->gps_heading)) + (-gps_offset_point->Y * sin (-msg->gps_heading));
	gps_point->Y = msg->gps_y + (gps_offset_point->X * sin (-msg->gps_heading)) + (gps_offset_point->Y * cos (-msg->gps_heading));
	gps_point->Z = 0;
	gps_point->utm_zone = msg->utm_zone;
	gps_covariance->X= msg->nav_sat_fix.position_covariance[0];
	gps_covariance->Y= msg->nav_sat_fix.position_covariance[4];
	gps_covariance->Z= msg->nav_sat_fix.position_covariance[8];
	g_gps_heading = - msg->gps_heading ;

	if(g_nImuAngleAvailability == 0)
	{
		imu_angle->yaw = - msg->gps_heading ;
		imu_angle->count = msg -> header.seq;
	}

	if (msg->gps_active == 1 && gps_flag == 0 )
	{
		gps_flag = 1;
		gps_initial_point->frame_id = msg->header.frame_id;
		gps_initial_point->count = msg->header.seq;
		gps_initial_point->X = msg->gps_x+gps_offset_point->X;
		gps_initial_point->Y = msg->gps_y+gps_offset_point->Y;
		gps_initial_point->Z = 0;
		gps_initial_point->utm_zone = msg->utm_zone;
		if(g_nImuAngleAvailability == 0)
		{
		imu_flag = 1;
		imu_initial_angle->frame_id = msg ->header.frame_id;
		imu_initial_angle->count = msg -> header.seq;
		imu_initial_angle->roll = 0;
		imu_initial_angle->pitch = 0;
		imu_initial_angle->yaw = - msg->gps_heading ;
		}

	}
}

void imu_callback(const simulation::imuData::ConstPtr& msg, vse_tf_point *imu_acc_point, vse_tf_RPY *imu_angle,vse_tf_RPY *imu_initial_angle)
{
	imu_acc_point->frame_id = msg ->header.frame_id;
	imu_acc_point->count = msg -> header.seq;
	imu_acc_point->time = msg -> header.stamp;
	imu_acc_point->X = msg->imu.linear_acceleration.x;
	imu_acc_point->Y = msg->imu.linear_acceleration.y;
	imu_acc_point->Z = 0;

	imu_angle->frame_id = msg ->header.frame_id;
	imu_angle->count = msg -> header.seq;
	imu_angle->time = msg -> header.stamp;
	imu_angle->roll = msg->roll;
	imu_angle->pitch = msg->pitch;
	imu_angle->yaw = msg->yaw ;


	if(msg->imu_active == 1 && imu_flag == 0)
	{
		imu_flag = 1;
		imu_initial_angle->frame_id = msg ->header.frame_id;
		imu_initial_angle->count = msg -> header.seq;
		imu_initial_angle->roll = msg->roll;
		imu_initial_angle->pitch = msg->pitch;
		imu_initial_angle->yaw = msg->yaw ;
	}

}




int main(int argc,char** argv)
{

	ros::init(argc, argv, "tf_frame_publisher123");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
   	
	tf::TransformListener listener_obj(ros::Duration(10));
	tf::TransformBroadcaster broadcaster;	
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
	
	
	UTM_mercator gps_point,gps_initial_point;
	vse_tf_point imu_acc_vector,gps_covariance,gps_offset_point;
	vse_tf_RPY imu_angle,imu_initial_angle;
	vse_tf_point fusion_pos_point,fusion_acc_vector,fusion_vel_vector,imu_offset_point;
	
	
	gps_point.count = 0 ;
	gps_point.X = 0 ;
	gps_point.Y = 0 ;
	gps_point.Z = 0 ;
	imu_angle.count = 0 ;
	gps_initial_point.X = 0;
	gps_initial_point.Y = 0;
	gps_initial_point.Z = 0;
	imu_initial_angle.yaw = 0;
	gps_covariance.X = 0;
	gps_covariance.Y = 0;
	gps_covariance.Z = 0;

	string szGpsTopic,szImuTopic,szWsTopic,szFusionTopic,szImuTransformedTopic,szGpsTransformedTopic,szWsTransformedTopic,szFusionTransformedTopic;
	nh.getParam("gps_topic",szGpsTopic);
	nh.getParam("imu_topic",szImuTopic);
	nh.getParam("imu_offset_data",dImuOffsetData);
	nh.getParam("imu_transformed_topic",szImuTransformedTopic);
	nh.getParam("gps_transformed_topic",szGpsTransformedTopic);
	nh.getParam("imu_angle_availability",g_nImuAngleAvailability);

	nh.getParam("gps_offset_x",gps_offset_point.X);
	nh.getParam("gps_offset_y",gps_offset_point.Y);
	nh.getParam("gps_offset_z",gps_offset_point.Z);
	
	ros::Subscriber sub_gps = nh.subscribe<simulation::gpsData> (szGpsTopic, 1, boost::bind(gps_callback, _1,&gps_point,&gps_initial_point,&gps_covariance,&gps_offset_point,&imu_angle,&imu_initial_angle) );
   	ros::Subscriber sub_imu = nh.subscribe<simulation::imuData> (szImuTopic, 1, boost::bind(imu_callback, _1,&imu_acc_vector,&imu_angle,&imu_initial_angle) );

    ros::Publisher pub_gps = nh.advertise<simulation::gpsData>(szGpsTransformedTopic,1); 		
		
	int i= 0;
	double prev_gps_point_count = 0;
	
	simulation::imuData msg_odom_imu;
	simulation::gpsData msg_odom_gps;	
	
	ofstream file_yaml (argv[1],ios::trunc);
	ros::Time earth_to_odom_time,odom_to_baselink_time,baselink_to_imuframe_time;
	
	tf::Quaternion quat_wrt_earth;
	tf::Quaternion quat_wrt_baselink;
	geometry_msgs::PoseStamped pose_create_frame;
	geometry_msgs::PoseStamped Psource;
	geometry_msgs::PoseStamped Pdestination;
	geometry_msgs::Vector3Stamped Vsource;
	geometry_msgs::Vector3Stamped Vdestination;
	
	geometry_msgs::TransformStamped static_transformStamped;
        int counter = 0;	
	while(ros::ok())
	{
	
		if(gps_point.count > 1 && imu_angle.count > 1)
		{
			
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "earth";
      static_transformStamped.child_frame_id = "odom";
      static_transformStamped.transform.translation.x = gps_initial_point.X;
      static_transformStamped.transform.translation.y = gps_initial_point.Y;
      static_transformStamped.transform.translation.z = gps_initial_point.Z;
      tf2::Quaternion quat_wrt_earth;
      quat_wrt_earth.setRPY(0, 0, imu_initial_angle.yaw);
      static_transformStamped.transform.rotation.x = quat_wrt_earth.x();
      static_transformStamped.transform.rotation.y = quat_wrt_earth.y();
      static_transformStamped.transform.rotation.z = quat_wrt_earth.z();
      static_transformStamped.transform.rotation.w = quat_wrt_earth.w();
      static_broadcaster.sendTransform(static_transformStamped);
	cout<<"listener_obj.frameExists(odom) : "<<listener_obj.frameExists("odom")<<"\n";

		if(file_yaml.is_open())
				{
					double temp =0.0;
					file_yaml <<"initial_yaw : "<<imu_initial_angle.yaw<<"\n";
					file_yaml <<"initial_pitch : "<<temp<<"\n";
					file_yaml <<"initial_roll : "<<temp<<"\n";
					file_yaml <<"initial_position_x : "<< std::setprecision(15)<<(gps_initial_point.X)<<"\n";
					file_yaml <<"initial_position_y : "<< std::setprecision(15)<<(gps_initial_point.Y)<<"\n";
					file_yaml <<"initial_position_z : "<< std::setprecision(15)<<(gps_initial_point.Z)<<"\n";
					nh.setParam("initial_yaw", imu_initial_angle.yaw);
					nh.setParam("initial_pitch", temp);
					nh.setParam("initial_roll", temp);
					nh.setParam("initial_position_x", gps_initial_point.X);
					nh.setParam("initial_position_y", gps_initial_point.Y);
					nh.setParam("initial_position_z", gps_initial_point.Z);
					file_yaml.close();
					cout<<"yaml created \n";
				}
		
		
		}

		
		{
		
			if(gps_point.count>prev_gps_point_count)
			{		
				prev_gps_point_count = gps_point.count;
				Psource.pose.position.x = gps_point.X ;
				Psource.pose.position.y = gps_point.Y ;
				Psource.pose.position.z = gps_point.Z ;
				Psource.pose.orientation.x = 0 ;
				Psource.pose.orientation.y = 0 ;
				Psource.pose.orientation.z = 0 ;
				Psource.pose.orientation.w = 1 ;
				Psource.header.stamp = ros::Time();
				Psource.header.frame_id = "earth";
			   	fnPositionTransform( Psource, Pdestination,"earth" ,"odom" , "odom", earth_to_odom_time, listener_obj);	
			
				msg_odom_gps.header.seq = gps_point.count; 	
				msg_odom_gps.header.stamp = gps_point.time; 
				msg_odom_gps.gps_active = 1 ;
				msg_odom_gps.utm_zone = gps_point.utm_zone;
				msg_odom_gps.gps_x = Pdestination.pose.position.x;
				msg_odom_gps.gps_y = Pdestination.pose.position.y;
				msg_odom_gps.gps_z = Pdestination.pose.position.z;
				msg_odom_gps.nav_sat_fix.position_covariance[0]=gps_covariance.X;
				msg_odom_gps.nav_sat_fix.position_covariance[4]=gps_covariance.Y;
				msg_odom_gps.nav_sat_fix.position_covariance[8]=gps_covariance.Z;

				tf::Quaternion quat_wrt_imuFrame;
				quat_wrt_imuFrame.setRPY(0,0,g_gps_heading);
				Psource.pose.position.x = 0 ;
				Psource.pose.position.y = 0 ;
				Psource.pose.position.z = 0 ;
				Psource.pose.orientation.x = quat_wrt_imuFrame.x() ;
				Psource.pose.orientation.y = quat_wrt_imuFrame.y()  ;
				Psource.pose.orientation.z = quat_wrt_imuFrame.z() ;
				Psource.pose.orientation.w = quat_wrt_imuFrame.w() ;
				Psource.header.stamp = ros::Time();
				Psource.header.frame_id = "earth";
			   	fnPositionTransform( Psource, Pdestination,"earth" ,"odom" , "odom", earth_to_odom_time , listener_obj);

				tf::Quaternion quat_wrt_odom(Pdestination.pose.orientation.x,Pdestination.pose.orientation.y,Pdestination.pose.orientation.z,Pdestination.pose.orientation.w);
				double roll_bsl,pitch_bsl,yaw_bsl;
				tf::Matrix3x3(quat_wrt_odom).getRPY(roll_bsl, pitch_bsl, yaw_bsl);

				msg_odom_gps.gps_heading = yaw_bsl;

				pub_gps.publish(msg_odom_gps);
		    	}
			    else
			    {
			     	msg_odom_gps.gps_active = 0 ;  	
			    }
			
		}		
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

	
	
