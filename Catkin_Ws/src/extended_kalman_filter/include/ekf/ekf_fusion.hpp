#include "ros/ros.h"
#include "std_msgs/String.h"
#include "KalmanFilter.hpp"
#include <ros/console.h>
#include "simulation/imuData.h"
#include "simulation/gpsData.h"
#include "simulation/SCAcanData.h"
#include "extended_kalman_filter/VSEFusionData.h"
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <topic_tools/shape_shifter.h>
#include "sensor_msgs/Imu.h"

using namespace std;


/*Structure for position x,y,z */
typedef struct
{
bool active;
double count;
ros::Time time;
double X ;
double Y ;
double Z ;
} vse_point;

/*Structure for roll,pitch and yaw*/
typedef struct
{
double roll ;
double pitch ;
double yaw ;
} vse_RPY;

/*Global variables*/
extern vse_point g_sGpsPoint,g_sImuVector1,g_sImuVector2,g_sWsVector,g_sVisualOdometryVector;
extern double g_dGpsHeading ;
extern double g_dGpsCount ;
extern sensor_msgs::Imu g_ImuRate;
extern simulation::imuData g_ImuAngle;


/* Initializing Filter parameters*/
extern int fnInitialAngleFusionParam(ros::NodeHandle&,CKalmanFilter&);

extern void SetMeasurementMatrixForAngle(bool&,double&,double&,double&,CKalmanFilter&);

extern int fnReadDataFromParam(ros::NodeHandle&, CKalmanFilter&);

extern void SetMeasurementMatrix(ros::NodeHandle&, int ,CKalmanFilter&);

/*Initialize Marker colour , width and frame for visualization */
extern int fnInitializeVisualization (visualization_msgs::Marker&,visualization_msgs::Marker&,visualization_msgs::Marker&,visualization_msgs::Marker&);

/*Assign data  to markers for visualization  */
extern void fnPlotGps(visualization_msgs::Marker&, double&);

extern void fnPlotImu(visualization_msgs::Marker&, double& ,vse_point&,vse_point&);

extern void fnPlotWs(visualization_msgs::Marker&, double& ,vse_point&);

extern void fnPlotFusion(visualization_msgs::Marker&,CKalmanFilter&, double&);


/*ROS callback functions*/
extern void gps_callback(const simulation::gpsData::ConstPtr& ,  CKalmanFilter&);

extern void imu_callback(const simulation::imuData::ConstPtr& ,  CKalmanFilter& ,vse_RPY*);

extern void ws_callback(const simulation::wheelData::ConstPtr& ,  CKalmanFilter&);

extern void gps_angle_callback(const simulation::gpsData::ConstPtr&  );

extern void imu_rate_callback(const sensor_msgs::Imu::ConstPtr&  );

extern void imu_angle_callback(const simulation::imuData::ConstPtr& );


