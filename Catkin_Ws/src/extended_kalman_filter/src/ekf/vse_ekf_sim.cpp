#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ekf.hpp"
#include <ros/console.h>
#include "simulation/gpsData.h"
#include "simulation/imuData.h"
#include "simulation/wheelData.h"
#include "extended_kalman_filter/VSEFusionData.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <topic_tools/shape_shifter.h>
#include "sensor_msgs/Imu.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include "navsat_conversions.hpp"
#include <random>
#define RADIAN (3.1415926535/180)
using namespace std;



typedef struct
{
bool active;
double count;
ros::Time time;
double X ;
double Y ;
double Z ;
} point;

typedef struct
{
double roll ;
double pitch ;
double yaw ;
} RPY;


point g_sGpsPoint,g_sImuVector1,g_sImuVector2,g_sWsVector,g_sVisualOdometryVector,g_sWsOdomPlot;
double g_dGpsHeading = 0;
double g_dGpsCount = 0;
sensor_msgs::Imu g_ImuRate;
simulation::imuData g_ImuAngle;
string utm_zone;
double dImuOffsetData =0;
int g_nImuAngleAvailability =0;


bool fnPositionTransform( geometry_msgs::PoseStamped &source, geometry_msgs::PoseStamped &destination,string parent ,string child , string targetFrame, ros::Time &now ,tf::TransformListener &listener_object)
{

	try 
	{
	listener_object.waitForTransform(parent, child,now, ros::Duration(0.001));
	listener_object.transformPose(targetFrame, source, destination);
	} 
	catch (tf::TransformException ex) 
	{
     	ROS_ERROR(" Position Transformation : %s", ex.what());
   	}
	 ROS_DEBUG_STREAM("Transformation between "<<parent << " to "<<child<<" data : "<<source);
	
	return 1;
		 
}


bool fnVectorTransform( geometry_msgs::Vector3Stamped &source, geometry_msgs::Vector3Stamped &destination,string parent ,string child , string targetFrame, ros::Time &now ,tf::TransformListener &listener_object)
{
	try 
	{
	listener_object.waitForTransform(parent, child,now, ros::Duration(0.001));
	listener_object.transformVector(targetFrame, source, destination);	
	} 
	catch (tf::TransformException ex) 
	{
     	ROS_ERROR("Vector Transformation %s", ex.what());
    	}	 
	ROS_DEBUG_STREAM("Transformation between "<<parent << " to "<<child<<" data : "<<source);
	 
	return 1;
}



void gps_callback(const simulation::gpsData::ConstPtr& msg,  CExtendedKalmanFilter &o_CFusionObject,RPY *angle)
{

	g_sGpsPoint.active = msg->gps_active;
	g_sGpsPoint.count = msg->header.seq;
	g_sGpsPoint.time = msg->header.stamp;
	g_sGpsPoint.X = msg -> gps_x;
	g_sGpsPoint.Y = msg -> gps_y;
	g_sGpsPoint.Z = msg -> gps_z;
	utm_zone = msg->utm_zone;

	float epx = msg ->nav_sat_fix.position_covariance[0];
	float epy = msg ->nav_sat_fix.position_covariance[4];

	if(g_nImuAngleAvailability == 0)
	{
		angle->roll =0;
		angle->pitch = 0;
		angle->yaw = msg -> gps_heading ;
	}
	o_CFusionObject.setMeasurementNoiseCovarianceGps(epx, epy, 0);
	
	ROS_INFO_STREAM( "gps_data_odom  :  { " <<angle->yaw<<" , "<<g_sGpsPoint.X <<" ,"<< g_sGpsPoint.Y<<" ," <<g_sGpsPoint.count<<"}\n");
}


const double mean_imu = 0.0;
const double stddev_imu = 0.0174;
std::default_random_engine ran_gen_imu;
std::normal_distribution<double> dist(mean_imu,stddev_imu);
void imu_callback(const simulation::imuData::ConstPtr& msg,  CExtendedKalmanFilter &o_CFusionObject ,RPY *angle)
{	
	g_sImuVector1.active = msg->imu_active;
	g_sImuVector1.count = msg->header.seq;
	g_sImuVector1.time = msg->header.stamp;
	g_sImuVector1.X = msg -> imu.linear_acceleration.x;
	g_sImuVector1.Y = msg -> imu.linear_acceleration.y;
	g_sImuVector1.Z = msg -> imu.linear_acceleration.z;
	angle->roll =msg -> roll;
	angle->pitch = msg -> pitch;
	angle->yaw =  msg -> yaw ;
		
	ROS_INFO_STREAM( "imu_data_odom  :  { " <<g_sImuVector1.active<<" , "<<angle->roll <<" ,"<<angle->pitch<<" ," <<angle->yaw<<"}\n");

}


void imu_rate_callback(const sensor_msgs::Imu::ConstPtr&  msg_rate)
{
	g_sImuVector1.active = 1;
	g_ImuRate.header.frame_id = msg_rate->header.frame_id;
	g_ImuRate.header.seq = msg_rate->header.seq;
	g_sImuVector1.count = msg_rate->header.seq;
	g_ImuRate.header.stamp = msg_rate->header.stamp;
	g_sImuVector1.time = msg_rate->header.stamp;
	g_sImuVector1.X = msg_rate->linear_acceleration.x;
	g_sImuVector1.Y = msg_rate->linear_acceleration.y;
	g_sImuVector1.Z = msg_rate->linear_acceleration.z;

	g_ImuRate.angular_velocity.x = msg_rate->angular_velocity.x;
	g_ImuRate.angular_velocity.y = msg_rate->angular_velocity.y;	
	g_ImuRate.angular_velocity.z =  msg_rate->angular_velocity.z;
	g_ImuRate.linear_acceleration.x = msg_rate->linear_acceleration.x;	
	g_ImuRate.linear_acceleration.y = msg_rate->linear_acceleration.y;	
	g_ImuRate.linear_acceleration.z = msg_rate->linear_acceleration.z;
	ROS_INFO_STREAM( "imu rate_data  :  { " <<g_ImuRate.header.seq <<" , "<<g_ImuRate.angular_velocity.x<<" ,"<< g_ImuRate.angular_velocity.y <<" ," <<g_ImuRate.angular_velocity.z<<"}\n");			

}


const double mean_ws = 0.0;
const double stddev_ws = 0.01;
std::default_random_engine ran_gen_ws;
std::normal_distribution<double> dist_ws(mean_ws,stddev_ws);
void ws_callback(const simulation::wheelData::ConstPtr& msg,  CExtendedKalmanFilter &o_CFusionObject)
{

	g_sWsVector.active = 1;
	g_sWsVector.count = msg->header.seq;
	g_sWsVector.time = msg->header.stamp;
	g_sWsVector.X = msg -> velocity ;//+ dist_ws(ran_gen_ws);
	g_sWsVector.Y = msg -> velocity_y;
	g_sWsVector.Z = msg -> velocity_z;
	ROS_INFO_STREAM( "ws_data_odom  :  { " <<g_sWsVector.active<<" , "<<g_sWsVector.X <<" ,"<< g_sWsVector.Y<<" ," <<g_sWsVector.count<<"}\n");
}


int fnReadDataFromParam(ros::NodeHandle &nh, CExtendedKalmanFilter &o_CFusionObject)
{
	int nNoOfSensors;
   	nh.getParam("no_of_sensors" , nNoOfSensors);
	int nSensorsStates = nNoOfSensors * 3;
	o_CFusionObject.m_nControlDimension = nSensorsStates;
	std::vector<double> vQ1List,vP1List,vR1List;
	nh.getParam("Q1_MAT" , vQ1List);
	nh.getParam("P1_MAT" , vP1List);
	nh.getParam("R1_MAT" , vR1List);
	
	MatrixXd Q1(o_CFusionObject.m_nStateDimension,o_CFusionObject.m_nStateDimension);
	MatrixXd H1(o_CFusionObject.m_nControlDimension,o_CFusionObject.m_nStateDimension);
	MatrixXd R1(o_CFusionObject.m_nControlDimension,o_CFusionObject.m_nControlDimension );
	MatrixXd P1(o_CFusionObject.m_nStateDimension,o_CFusionObject.m_nStateDimension);

	
	int nListSize=0;
	for(int i=0; i < o_CFusionObject.m_nStateDimension; i++) 
	{
		for(int j=0; j < o_CFusionObject.m_nStateDimension; j++) 
		{
			Q1(i,j) = vQ1List[nListSize];
			nListSize+=1;	
		}
		
	}
	vQ1List.clear();
	vQ1List.shrink_to_fit();
	

	nListSize=0;
	for(int i=0; i < o_CFusionObject.m_nStateDimension; i++) 
	{
		for(int j=0; j < o_CFusionObject.m_nStateDimension; j++) 
		{
			P1(i,j) = vP1List[nListSize];
			nListSize+=1;	
		}
		
	}
	vP1List.clear();
	vP1List.shrink_to_fit();
	
	nListSize=0;
	for(int i=0; i < o_CFusionObject.m_nControlDimension; i++) 
	{
		for(int j=0; j < o_CFusionObject.m_nControlDimension; j++) 
		{
			R1(i,j) = vR1List[nListSize];
			nListSize+=1;	
		}
		
	}
	vR1List.clear();
	vR1List.shrink_to_fit();
	
    	vector<double>vH1List;
	std::string szKey,szKey1,szKey2,szKey3,szKey4,szKey5;	
	
	if (nh.searchParam("H1_MAT_gps", szKey4))
	{
		vector<double>vH1MatGps;
		nh.getParam("H1_MAT_gps",vH1MatGps);
		nListSize=0;
		for(int j=0; j < vH1MatGps.size(); j++) 
		{
			vH1List.push_back(vH1MatGps[nListSize]);
			nListSize+=1;	
		}
		vH1MatGps.clear();
		vH1MatGps.shrink_to_fit();
	}
	
	if (nh.searchParam("H1_MAT_Wheel", szKey2))
	{
		vector<double>vH1MatWheel;
		nh.getParam("H1_MAT_Wheel",vH1MatWheel);
		nListSize=0;
		for(int j=0; j < vH1MatWheel.size(); j++) 
		{
			vH1List.push_back(vH1MatWheel[nListSize]);
			nListSize+=1;	
		}
		vH1MatWheel.clear();
		vH1MatWheel.shrink_to_fit();
	}
	
/*	if (nh.searchParam("H1_MAT_visual_odometry", szKey3))
	{
		vector<double>vH1MatVisualOdometry;
		nh.getParam("H1_MAT_visual_odometry",vH1MatVisualOdometry);
		nListSize=0;
		for(int j=0; j < vH1MatVisualOdometry.size(); j++) 
		{
			vH1List.push_back(vH1MatVisualOdometry[nListSize]);
			nListSize+=1;	
		}
		vH1MatVisualOdometry.clear();
		vH1MatVisualOdometry.shrink_to_fit();
	}
*/	
	if (nh.searchParam("H1_MAT_IMU_ACC", szKey))
	{
		vector<double>vH1MatImu1;
		nh.getParam("H1_MAT_IMU_ACC",vH1MatImu1);
		nListSize=0;
		for(int j=0; j < vH1MatImu1.size(); j++) 
		{
			vH1List.push_back(vH1MatImu1[nListSize]);
			nListSize+=1;	
		}
		vH1MatImu1.clear();
		vH1MatImu1.shrink_to_fit();

	}
		
	if (nh.searchParam("H1_MAT_IMU_ANGLE", szKey1))
	{
		vector<double>vH1MatImu2;
		nh.getParam("H1_MAT_IMU_ANGLE",vH1MatImu2);
		nListSize=0;
		for(int j=0; j < vH1MatImu2.size(); j++) 
		{
			vH1List.push_back(vH1MatImu2[nListSize]);
			nListSize+=1;	
		}
		vH1MatImu2.clear();
		vH1MatImu2.shrink_to_fit();
	}
	
	if (nh.searchParam("H1_MAT_IMU_RATE", szKey5))
	{
		vector<double>vH1MatImu3;
		nh.getParam("H1_MAT_IMU_RATE",vH1MatImu3);
		nListSize=0;
		for(int j=0; j < vH1MatImu3.size(); j++) 
		{
			vH1List.push_back(vH1MatImu3[nListSize]);
			nListSize+=1;	
		}
		vH1MatImu3.clear();
		vH1MatImu3.shrink_to_fit();

	}


	nListSize=0;
	for(int i=0; i < o_CFusionObject.m_nControlDimension; i++) 
	{
		for(int j=0; j < o_CFusionObject.m_nStateDimension; j++) 
		{
			H1(i,j) = vH1List[nListSize];
			nListSize+=1;	
		}
		
	}
	vH1List.clear();
	vH1List.shrink_to_fit();	
	o_CFusionObject.setInitialMatrix(P1);
	o_CFusionObject.setFixedMatrix(H1,Q1,R1);
	
	P1.resize(0,0);
	H1.resize(0,0);
	Q1.resize(0,0);
	R1.resize(0,0);
	
	return(o_CFusionObject.m_nControlDimension);
	

}


int fnInitializeVisualization (visualization_msgs::Marker &odom_gps,visualization_msgs::Marker &fusion, visualization_msgs::Marker &odom_ws)

{

	fusion.header.frame_id ="odom";
	fusion.header.stamp = ros::Time::now(); 
	fusion.ns = "kalman";
	fusion.action = visualization_msgs::Marker::ADD;
	fusion.pose.orientation.w = 1.0;

	fusion.id = 0;
	fusion.type = visualization_msgs::Marker::CUBE;

	fusion.scale.x = 0.1;
	fusion.scale.y = 0.1;
	fusion.scale.z = 0.1;

	fusion.color.r = 1.0f;
	fusion.color.g = 1.0f;
	fusion.color.b = 1.0f;
	fusion.color.a = 1.0;

	odom_gps.header.frame_id ="odom";
	odom_gps.header.stamp = ros::Time::now(); 
	odom_gps.ns = "odom_gps";
	odom_gps.action = visualization_msgs::Marker::ADD;
	odom_gps.pose.orientation.w = 1.0;

	odom_gps.id = 0;
	odom_gps.type = visualization_msgs::Marker::CUBE;

	odom_gps.scale.x = 0.5;
	odom_gps.scale.y = 0.5;
	odom_gps.scale.z = 0.5;

	odom_gps.color.r = 0.0f;
	odom_gps.color.g = 0.0f;
	odom_gps.color.b = 1.0f;
	odom_gps.color.a = 1.0;	

	odom_ws.header.frame_id ="odom";
	odom_ws.header.stamp = ros::Time::now(); 
	odom_ws.ns = "odom_ws";
	odom_ws.action = visualization_msgs::Marker::ADD;
	odom_ws.pose.orientation.w = 1.0;

	odom_ws.id = 0;
	odom_ws.type = visualization_msgs::Marker::CUBE;

	odom_ws.scale.x = 0.1;
	odom_ws.scale.y = 0.1;
	odom_ws.scale.z = 0.1;

	odom_ws.color.r = 1.0f;
	odom_ws.color.g = 0.0f;
	odom_ws.color.b = 0.0f;
	odom_ws.color.a = 1.0;	
	return 1;


}


void fnPlotGps(visualization_msgs::Marker &odom_gps, double &nMarkerCount)
{
	odom_gps.id = nMarkerCount;
	odom_gps.pose.position.x = g_sGpsPoint.X;
	odom_gps.pose.position.y = g_sGpsPoint.Y;
	odom_gps.pose.position.z = g_sGpsPoint.Z;

}


void fnPlotFusion(visualization_msgs::Marker &fusion,CExtendedKalmanFilter &o_CFusionObject, double &nMarkerCount)
{
	fusion.id = nMarkerCount;
	fusion.pose.position.x = o_CFusionObject.m_dStateMatrix(0,0);
	fusion.pose.position.y = o_CFusionObject.m_dStateMatrix(1,0);
	fusion.pose.position.z = o_CFusionObject.m_dStateMatrix(2,0);

}


void fnPlotWs(visualization_msgs::Marker &odom_ws, double &nMarkerCount ,double yawangle)
{
	odom_ws.id = nMarkerCount;
	g_sWsOdomPlot.X += (g_sWsVector.X*cos(yawangle)  - g_sWsVector.Y*sin(yawangle))* 0.01 ;
	g_sWsOdomPlot.Y += (g_sWsVector.Y*cos(yawangle)  + g_sWsVector.X*sin(yawangle)) * 0.01 ;
	g_sWsOdomPlot.Z += g_sWsVector.Z * 0.01 ; 
	
	odom_ws.pose.position.x = g_sWsOdomPlot.X;
	odom_ws.pose.position.y = g_sWsOdomPlot.Y;
	odom_ws.pose.position.z = 0;

}


void fnVelocityTransform(geometry_msgs::Vector3 velocity, geometry_msgs::Vector3 &transformed_velocity ,CExtendedKalmanFilter &o_CFusionObject)
{

    MatrixXf rotationMatrix(3,3);
    MatrixXf velocityMatrix(3,1);
    velocityMatrix(0,0) = velocity.x;
    velocityMatrix(1,0) = velocity.y;
    velocityMatrix(2,0) = velocity.z;
    rotationMatrix = rotationMatrix.Identity(3,3);

    rotationMatrix(0,0) = cos(o_CFusionObject.m_dStateMatrix(11,0));
    rotationMatrix(0,1) = sin(o_CFusionObject.m_dStateMatrix(11,0));
    rotationMatrix(1,0) = -sin(o_CFusionObject.m_dStateMatrix(11,0));
    rotationMatrix(1,1) = cos(o_CFusionObject.m_dStateMatrix(11,0));

    MatrixXf resultantMatrix(3,1);

    resultantMatrix = rotationMatrix.inverse() * velocityMatrix;

    transformed_velocity.x = resultantMatrix(0,0);
    transformed_velocity.y = resultantMatrix(1,0);
    transformed_velocity.z = resultantMatrix(2,0);
}

int main(int argc,char** argv)
{

	ros::init(argc, argv, "vse_Fusion");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
		
	RPY angle;

	CExtendedKalmanFilter o_CFusionObject;
	fnReadDataFromParam(nh,  o_CFusionObject);
	
	string szImuTransformedTopic,szGpsTransformedTopic,szFusionTransformedTopic,szCalibWheelTopic,szGpsTopic,szImuTopic,szImuTopic_pub,szFusionTopic;
	nh.getParam("imu_transformed_topic",szImuTransformedTopic);
	nh.getParam("gps_transformed_topic",szGpsTransformedTopic);
	nh.getParam("fusion_transformed_topic",szFusionTransformedTopic);
	nh.getParam("calib_wheel_topic",szCalibWheelTopic);
	nh.getParam("gps_topic",szGpsTopic);
	nh.getParam("imu_topic",szImuTopic);
	nh.getParam("imu_angle_availability",g_nImuAngleAvailability);
	nh.getParam("fusion_topic",szFusionTopic);
	nh.getParam("imu_offset_data",dImuOffsetData);

	int nImuEnable,nWsEnable,nGpsEnable;
	nh.getParam("IMU_sensor1",nImuEnable);
	nh.getParam("Wheel_speed_sensor",nWsEnable);
	nh.getParam("global_position_system",nGpsEnable);

		ros::Subscriber sub_gps = nh.subscribe<simulation::gpsData> (szGpsTransformedTopic, 1, boost::bind(gps_callback, _1,o_CFusionObject,&angle) );
    	ros::Subscriber sub_imu = nh.subscribe<simulation::imuData> (szImuTopic, 1, boost::bind(imu_callback, _1,o_CFusionObject,&angle) );
        string szImuRateTopic = szImuTopic+"_raw";
	ros::Subscriber sub_imu_rate = nh.subscribe(szImuRateTopic, 1,imu_rate_callback );

   		ros::Subscriber sub_ws = nh.subscribe<simulation::wheelData> (szCalibWheelTopic, 1, boost::bind(ws_callback, _1,o_CFusionObject) );
        ros::Publisher pub_fusion = nh.advertise<extended_kalman_filter::VSEFusionData>(szFusionTopic,1);  
	  
        ros::Publisher pub_fusion_trans = nh.advertise<extended_kalman_filter::VSEFusionData>(szFusionTransformedTopic,1);    
        extended_kalman_filter::VSEFusionData   msg_fusion;
	extended_kalman_filter::VSEFusionData   msg_fusion_trans;                                       
                           
	tf::TransformListener listener_object(ros::Duration(10));     
	double prev_imu_data_count = 0;
	//double data_count=0,data_count1=0;
	

	/*visualization Rviz*/
	ros::Publisher marker_odom_fusion = nh.advertise<visualization_msgs::Marker>("visualization_marker_fusion", 10);
	ros::Publisher marker_odom_gps = nh.advertise<visualization_msgs::Marker>("visualization_gps_odom", 10);
	ros::Publisher marker_odom_ws = nh.advertise<visualization_msgs::Marker>("visualization_ws_odom", 10);

	visualization_msgs::Marker fusion;
	visualization_msgs::Marker odom_gps;
	visualization_msgs::Marker odom_ws;

	int visualizationenabled = fnInitializeVisualization(odom_gps,fusion,odom_ws);
	double nMarkerCount =0;
	int initial_yaw =0;
	double prev_GpsCount =0;
   	int counter = 0;	
	while(ros::ok())
	{ 	
		ros::spinOnce();


		if(listener_object.frameExists("base_link") && listener_object.frameExists("odom") && g_sGpsPoint.count>1 )
		{
			if( g_sImuVector1.count > prev_imu_data_count)
			{
			
				prev_imu_data_count = g_sImuVector1.count;
				geometry_msgs::PoseStamped Psource;
				geometry_msgs::PoseStamped Pdestination;
				geometry_msgs::Vector3Stamped Vsource;
				geometry_msgs::Vector3Stamped Vdestination;

				Vsource.vector.x = g_sImuVector1.X;
				Vsource.vector.y = g_sImuVector1.Y;
				Vsource.vector.z = g_sImuVector1.Z ;
				Vsource.header.stamp = ros::Time();
				Vsource.header.frame_id = "imu_frame";

				fnVectorTransform( Vsource, Vdestination,"base_link" ,"imu_frame" , "base_link", Vsource.header.stamp , listener_object);

				tf::Quaternion quat_wrt_imuFrame;
				quat_wrt_imuFrame.setRPY(angle.roll,angle.pitch,angle.yaw);

				if(g_nImuAngleAvailability ==1)
				{
				Psource.pose.position.x = 0 ;
				Psource.pose.position.y = 0 ;
				Psource.pose.position.z = 0 ;
				Psource.pose.orientation.x = quat_wrt_imuFrame.x() ;
				Psource.pose.orientation.y = quat_wrt_imuFrame.y()  ;
				Psource.pose.orientation.z = quat_wrt_imuFrame.z()  ;
				Psource.pose.orientation.w = quat_wrt_imuFrame.w()  ;
				Psource.header.stamp = ros::Time();
				Psource.header.frame_id = "imu_frame";
			   	fnPositionTransform( Psource, Pdestination,"base_link" ,"imu_frame" , "base_link", Psource.header.stamp , listener_object);
				
				Psource.pose.position.x = 0 ;
				Psource.pose.position.y = 0 ;
				Psource.pose.position.z = 0 ;
				Psource.pose.orientation.x = Pdestination.pose.orientation.x ;
				Psource.pose.orientation.y = Pdestination.pose.orientation.y  ;
				Psource.pose.orientation.z = Pdestination.pose.orientation.z ;
				Psource.pose.orientation.w = Pdestination.pose.orientation.w  ;
				Psource.header.stamp = ros::Time();
				Psource.header.frame_id = "earth";
			   	fnPositionTransform( Psource, Pdestination,"earth" ,"odom" , "odom", Psource.header.stamp , listener_object);





				tf::Quaternion quat_wrt_odom(Pdestination.pose.orientation.x,Pdestination.pose.orientation.y,Pdestination.pose.orientation.z,Pdestination.pose.orientation.w);
				double roll_bsl,pitch_bsl,yaw_bsl;
				tf::Matrix3x3(quat_wrt_odom).getRPY(roll_bsl, pitch_bsl, yaw_bsl);
				angle.roll = 0;
				angle.pitch = 0;
				angle.yaw = yaw_bsl;
				}
			
				g_sImuVector1.X = Vdestination.vector.x;
				g_sImuVector1.Y = Vdestination.vector.y;
				g_sImuVector1.Z = Vdestination.vector.z;
					
				if(initial_yaw == 0)
				{
					o_CFusionObject.m_dInitialStateMatrix(o_CFusionObject.eYaw,0) = angle.yaw;
					o_CFusionObject.GetCosSinRPY (angle.roll, angle.pitch, angle.yaw);
					cout<<"initial yaw"<<o_CFusionObject.m_dInitialStateMatrix(o_CFusionObject.eYaw,0);
					initial_yaw =1;
				}
				else
				{
					o_CFusionObject.GetCosSinRPY (angle.roll, angle.pitch, o_CFusionObject.m_dStateMatrix(11,0));
				}

			}
						
			o_CFusionObject.predict();
		

			cout<<"angle.yaw : " <<o_CFusionObject.clampRPY(angle.yaw)<<"\n";
			if(g_nImuAngleAvailability ==1)
			{
				cout<<"Inside if"<<endl;
				o_CFusionObject.m_dMeasurementMatrix.resize(o_CFusionObject.m_nControlDimension,1);
	o_CFusionObject.m_dMeasurementMatrix<<g_sGpsPoint.X,g_sGpsPoint.Y,g_sGpsPoint.Z,g_sWsVector.X,g_sWsVector.Y,g_sWsVector.Z,0,0,0,0,0,o_CFusionObject.clampRPY(angle.yaw),0,0,0;         
			}
			else
			{
				o_CFusionObject.m_dMeasurementMatrix.resize(o_CFusionObject.m_nControlDimension,1);

				if(g_sWsVector.X<0.1)
				{
				g_ImuRate.angular_velocity.z = 0;
				}
				else
				{
				}
					o_CFusionObject.m_dMeasurementMatrix<<g_sGpsPoint.X,g_sGpsPoint.Y,g_sGpsPoint.Z,g_sWsVector.X,g_sWsVector.Y,g_sWsVector.Z,0,0,0,0,0,o_CFusionObject.clampRPY(angle.yaw),0,0,g_ImuRate.angular_velocity.z;

				if(g_sWsVector.X>1.5)
				{ 
					if(g_sGpsPoint.count>prev_GpsCount)
					{
						prev_GpsCount = g_sGpsPoint.count;
						o_CFusionObject.setMeasurementAdaptationAngle(1,1);
					}
					else
					{	
						o_CFusionObject.setMeasurementAdaptationAngle(0,1);
					}
				}
				else
				{	
					o_CFusionObject.setMeasurementAdaptationAngle(0,1);
				}
				
				if(fabs(g_ImuRate.angular_velocity.z)<(0.017*5))
				{
					o_CFusionObject.m_dMeasurementNoiseCovarianceMatrix(14,14) = (0.0017 );//0.017
				}
				else
				{
					o_CFusionObject.m_dMeasurementNoiseCovarianceMatrix(14,14) = 0.6;//0.64
				}


			}							

			if(fabs(o_CFusionObject.m_dStateMatrix(4,0)) < 0.0001)	
			{
				o_CFusionObject.setMeasurementAdaptationGps(g_sGpsPoint.active,nGpsEnable);
				g_sGpsPoint.active = 0;
				o_CFusionObject.setMeasurementAdaptationImu(nImuEnable,g_sImuVector1.active);
				g_sImuVector1.active = 0;
				o_CFusionObject.setMeasurementAdaptationWs(g_sWsVector.active,nWsEnable);
				g_sWsVector.active = 0;
			}
			else
			{
				o_CFusionObject.setMeasurementAdaptationGps(1,1);
				o_CFusionObject.setMeasurementAdaptationImu(nImuEnable,g_sImuVector1.active);		
				o_CFusionObject.setMeasurementAdaptationWs(1,1);
			}			
			o_CFusionObject.correct();
			
			if(visualizationenabled == 1)
			{
				nMarkerCount++;
				fnPlotGps(odom_gps,nMarkerCount);
				fnPlotFusion(fusion,o_CFusionObject,nMarkerCount);
				double temp_Angle = angle.yaw;
				fnPlotWs(odom_ws,nMarkerCount,temp_Angle);
		
				marker_odom_fusion.publish(fusion);
				marker_odom_gps.publish(odom_gps);
				marker_odom_ws.publish(odom_ws);
			}


			msg_fusion.header.frame_id = "odom";
geometry_msgs::Vector3 velocity;
            velocity.x =  o_CFusionObject.m_dStateMatrix(3,0);
            velocity.y =  o_CFusionObject.m_dStateMatrix(4,0);
            velocity.z = 0.0;
            geometry_msgs::Vector3 transformed_velocity;
            fnVelocityTransform(velocity, transformed_velocity , o_CFusionObject);

			{
				msg_fusion.header.stamp = g_sImuVector1.time;
			}
			msg_fusion.estimated_position_x = o_CFusionObject.m_dStateMatrix(0,0);
			msg_fusion.estimated_position_y = o_CFusionObject.m_dStateMatrix(1,0);
			msg_fusion.estimated_position_z = o_CFusionObject.m_dStateMatrix(2,0);
			msg_fusion.estimated_velocity_x = transformed_velocity.x;
			msg_fusion.estimated_velocity_y = transformed_velocity.y;
			msg_fusion.estimated_velocity_z = 0;
			msg_fusion.estimated_acceleration_x = g_sImuVector1.X ;
			msg_fusion.estimated_acceleration_y = g_sImuVector1.Y ;
			msg_fusion.estimated_acceleration_z = 0 ;
			msg_fusion.estimated_roll = o_CFusionObject.m_dStateMatrix(9,0);
			msg_fusion.estimated_pitch = o_CFusionObject.m_dStateMatrix(10,0);
			msg_fusion.estimated_yaw = o_CFusionObject.m_dStateMatrix(11,0);
			msg_fusion.FUSION_ACTIVE = 1;

			geometry_msgs::PoseStamped Psource;
			geometry_msgs::PoseStamped Pdestination;
			Psource.pose.position.x = o_CFusionObject.m_dStateMatrix(0,0) ;
			Psource.pose.position.y = o_CFusionObject.m_dStateMatrix(1,0) ;
			Psource.pose.position.z = 0 ;
			Psource.pose.orientation.x = 0 ;
			Psource.pose.orientation.y = 0  ;
			Psource.pose.orientation.z = 0 ;
			Psource.pose.orientation.w = 1  ;
			Psource.header.stamp = ros::Time();
			Psource.header.frame_id = "odom";
		   	fnPositionTransform( Psource, Pdestination,"earth" ,"odom" , "earth", Psource.header.stamp , listener_object);
			msg_fusion_trans.estimated_position_x = Pdestination.pose.position.x;
			msg_fusion_trans.estimated_position_y = Pdestination.pose.position.y;
			msg_fusion_trans.estimated_position_z = 0;

			UTMtoLL(Pdestination.pose.position.y, Pdestination.pose.position.x,utm_zone,msg_fusion.estimated_latitude,msg_fusion.estimated_longitude);	
			msg_fusion_trans.estimated_latitude = msg_fusion.estimated_latitude;
			msg_fusion_trans.estimated_longitude = msg_fusion.estimated_longitude;

			tf::Quaternion quat_wrt_earthFrame;
			quat_wrt_earthFrame.setRPY(o_CFusionObject.m_dStateMatrix(9,0),o_CFusionObject.m_dStateMatrix(10,0),o_CFusionObject.m_dStateMatrix(11,0));

			
			Psource.pose.position.x = 0 ;
			Psource.pose.position.y = 0 ;
			Psource.pose.position.z = 0 ;
			Psource.pose.orientation.x = quat_wrt_earthFrame.x()  ;
			Psource.pose.orientation.y = quat_wrt_earthFrame.y()  ;
			Psource.pose.orientation.z = quat_wrt_earthFrame.z()  ;
			Psource.pose.orientation.w = quat_wrt_earthFrame.w()  ;
			Psource.header.stamp = ros::Time();
			Psource.header.frame_id = "odom";
		   	fnPositionTransform( Psource, Pdestination,"earth" ,"odom" , "earth", Psource.header.stamp , listener_object);


			tf::Quaternion quat_wrt_earth(Pdestination.pose.orientation.x,Pdestination.pose.orientation.y,Pdestination.pose.orientation.z,Pdestination.pose.orientation.w);
			double roll_earth,pitch_earth,yaw_earth;
			tf::Matrix3x3(quat_wrt_earth).getRPY(roll_earth, pitch_earth, yaw_earth);

			msg_fusion_trans.estimated_roll = roll_earth;
			msg_fusion_trans.estimated_pitch = pitch_earth;
			msg_fusion_trans.estimated_yaw = yaw_earth;

			geometry_msgs::Vector3Stamped Vsource;
			geometry_msgs::Vector3Stamped Vdestination;
			Vsource.vector.x = o_CFusionObject.m_dStateMatrix(3,0);
			Vsource.vector.y = o_CFusionObject.m_dStateMatrix(4,0);
			Vsource.vector.z = o_CFusionObject.m_dStateMatrix(5,0) ;
			Vsource.header.stamp = ros::Time();
			Vsource.header.frame_id = "odom";

			fnVectorTransform( Vsource, Vdestination,"earth" ,"odom" , "earth", Vsource.header.stamp , listener_object);

			
			msg_fusion_trans.estimated_velocity_x = Vdestination.vector.x;
			msg_fusion_trans.estimated_velocity_y = Vdestination.vector.y;
			msg_fusion_trans.estimated_velocity_z = Vdestination.vector.z;

			msg_fusion_trans.header = msg_fusion.header ;
			msg_fusion_trans.header.frame_id = "earth" ;
			msg_fusion_trans.FUSION_ACTIVE = 1;

			pub_fusion.publish(msg_fusion);
			++counter;	
			pub_fusion_trans.publish(msg_fusion_trans);	

		}	
				
    loop_rate.sleep();  
	}
	

	return 0;
}

