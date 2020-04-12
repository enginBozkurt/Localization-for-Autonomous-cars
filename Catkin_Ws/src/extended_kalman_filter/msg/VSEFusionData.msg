Header header
#--------------------------------------FUSION--------------------------------------------
# Fusion: Multi model kalman filter
#----------------------------------------------------------------------------------------


#---------------------------------------FLAGS--------------------------------------------
bool FUSION_ACTIVE
#----------------------------------------------------------------------------------------


#--------------------------------------OUTPUT--------------------------------------------

float64 estimated_position_x
float64 estimated_position_y
float64 estimated_position_z

float64 estimated_velocity_x
float64 estimated_velocity_y
float64 estimated_velocity_z

float64 estimated_acceleration_x
float64 estimated_acceleration_y
float64 estimated_acceleration_z

float32 estimated_roll
float32 estimated_pitch
float32 estimated_yaw

#position data after fusion in  LatLong format:

float64 estimated_latitude
float64 estimated_longitude

#----------------------------------------------------------------------------------------


