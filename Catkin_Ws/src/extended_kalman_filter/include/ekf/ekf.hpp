
//Include Eigen Library
#include <eigen3/Eigen/Dense>
#include <math.h>
using namespace Eigen;
#define vse_delta 0.01

/*
*	Kalman Filter Class Definition.
*	
*	Matrix Dimension must be:
*
*	A: n x n
*	H: n x n
*	Q: n x n
*	R: n x n
*	I: n x n
*	X: n x 1
*	Z: n x 1
*	P: n x n
*	K: n x n
*
*/

class CExtendedKalmanFilter {

	public:
	
	
		enum VseStateMembers
		{
			  ePose_x = 0,
			  ePose_y,
			  ePose_z,
			  eVelocity_x,
			  eVelocity_y,
			  eVelocity_z,
			  eAcceleration_x,
			  eAcceleration_y,
			  eAcceleration_z,
			  eRoll,
			  ePitch,
			  eYaw,
			  eRoll_rate,
			  ePitch_rate,
			  eYaw_rate
		};

		/* Problem Dimension */
		int m_nStateDimension; //State matrix dimension
		int m_nControlDimension; //Control matrix (input) dimension (if there is not input, set to zero)
		int mMultiModelFusionFlag; //fusion flag
		int mUseCovarianceFromGps; //gps covariance flag
		/* Fixed Matrix */
		MatrixXd m_dSystemDynamicsMatrix; //System dynamics matrix //a
		MatrixXd m_dJacobianMatrix; //System dynamics matrix(jacobian) //J
		MatrixXd m_dMesaurementAdaptationMatrix; //Mesaurement Adaptation matrix //h
		MatrixXd m_dProcessNoiseCovarianceMatrix; //Process Noise Covariance matrix  //q
		MatrixXd m_dMeasurementNoiseCovarianceMatrix; //Measurement Noise Covariance matrix //r
		MatrixXd m_dIdentityMatrix; //Identity matrix  //i

		/* Variable Matrix */
		MatrixXd m_dStateMatrix; //(Current) State vector //x
		MatrixXd m_dStateCovarianceMatrix; //State Covariance //p
		MatrixXd m_dKalmanGainMatrix; //Kalman Gain matrix //k
		MatrixXd m_dMeasurementMatrix; //Measurement matrix //z
		/* Inizial Value */
		MatrixXd m_dInitialStateMatrix; //Initial State vector //x0
		MatrixXd m_dInitialStateCovarianceMatrix; //Initial State Covariance matrix //p0
		
		double m_dCosRoll;
		double m_dSinRoll;
		double m_dCosPitch;
		double m_dSinPitch;
		double m_dCosYaw;
		double m_dSinYaw;
		
		/* 
		* Constructor 
		* _n: state vector dimension ( set to zero)
		* _m: control vector dimension ( set to zero)
		*/
		CExtendedKalmanFilter();


		/* create cosine and sine with roll, pitch ,yaw */
		void GetCosSinRPY (double roll, double pitch, double yaw);
		
		double clampRPY(double rotation);
		
		void SetStateAngles();
		
		/* Set Fixed Matrix  */
		void setFixedMatrix ( MatrixXd _H, MatrixXd _Q, MatrixXd _R );
		
		/* Create SetSystemDynamics Matrix  */
		void SetSystemDynamicsMatrix ( );
		
		/* Create Jacobian matrix*/
		void SetJacobianMatrix();
		
		/* Set MeasurementAdaptation Matrix depends on sensor availability  */
		void setMeasurementAdaptationGps(int , int );
		void setMeasurementAdaptationImu(int , int);
		void setMeasurementAdaptationWs(int , int );
		void setMeasurementAdaptationAngle(int , int );
		void setMeasurementNoiseCovarianceGps(float ,float ,float );
	 	
		
		/* Set Initial Value */
		void setInitialMatrix ( MatrixXd _P0 );
		
		/* Do prediction  */
		void predict ( void );

		/* Do correction */
		void correct ( void );

};





