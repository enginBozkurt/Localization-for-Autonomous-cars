#--------------------------------------Raw Data------------------------------------------
# Available Sensors:
# 1. GPS
# 2. Wheel Speed Sensor 1
# 3. Wheel Speed Sensor 2
# 4. Wheel Speed Sensor 3
# 5. Wheel Speed Sensor 4
# 6. IMU(Accelerometer + Gyroscope + Magnetometer )
#----------------------------------------------------------------------------------------


bool estop
#---------------------------------Wheel Speed Sensors------------------------------------
bool LEFT_FRONT_WHEEL_SPEED_SENSOR_ACTIVE
bool RIGHT_FRONT_WHEEL_SPEED_SENSOR_ACTIVE
bool LEFT_REAR_WHEEL_SPEED_SENSOR_ACTIVE
bool RIGHT_REAR_WHEEL_SPEED_SENSOR_ACTIVE
int16 Left_Front_Wheel_Pulse_Count
int16 Right_Front_Wheel_Pulse_Count
int16 Left_Rear_Wheel_Pulse_Count
int16 Right_Rear_Wheel_Pulse_Count
float64 Left_Front_Wheel_Speed
float64 Right_Front_Wheel_Speed
float64 Left_Rear_Wheel_Speed
float64 Right_Rear_Wheel_Speed
bool VELOCITY_DATA_ACTIVE
float64 Velocity
float64 mult_fl
float64 mult_fr
float64 mult_fl2
float64 mult_fr2
float64 up
float64 down


float64 velocity_right_wheel
float64 velocity_left_wheel
float64 steering_left
float64 steering_right

float64 velocity_right_wheel_1
float64 velocity_left_wheel_1
float64 steering_left_1
float64 steering_right_1
float64 steering_left_2
float64 steering_right_2
float64 key_steering_angle


float32 steering_axis
float64 acc_axis
float64 brake_axis
float64 left_box
float64 down_x
float64 right_o
float64 up_tri
float64 acc_axis_scaled
float64 brake_axis_scaled

float64 velocity_FL
float64 velocity_FR
float64 velocity_BL
float64 velocity_BR
float64 angle_FL
float64 angle_FR
float64 angle_BL
float64 angle_BR
float64 vehicle_velocity_x
float64 vehicle_velocity_y
float64 vehicle_velocity_z
time current_time
#----------------------------------------------------------------------------------------


#----------------------------------------IMU---------------------------------------------
bool IMU_ACTIVE
bool IMU_SETTELED
float32 Magnetometer_x
float32 Magnetometer_y
float32 Magnetometer_z
float32 Gyroscope_x
float32 Gyroscope_y
float32 Gyroscope_z
float32 Accelerometer_x
float32 Accelerometer_y
float32 Accelerometer_z
float32 Yaw
float32 Roll
float32 Pitch
#----------------------------------------------------------------------------------------

