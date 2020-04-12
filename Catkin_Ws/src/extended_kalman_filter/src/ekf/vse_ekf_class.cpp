#include <iostream>
#include "ekf.hpp"
using namespace std;


CExtendedKalmanFilter::CExtendedKalmanFilter() 
{
	m_nStateDimension = 15;
	m_nControlDimension = 0;

}


void CExtendedKalmanFilter::setFixedMatrix( MatrixXd _H, MatrixXd _Q, MatrixXd _R )
{
	m_dMesaurementAdaptationMatrix = _H;
	m_dProcessNoiseCovarianceMatrix = _Q;
	m_dMeasurementNoiseCovarianceMatrix = _R;
	m_dIdentityMatrix = m_dIdentityMatrix.Identity(m_nStateDimension, m_nStateDimension);

}


void CExtendedKalmanFilter::setInitialMatrix( MatrixXd _P0 )
{
	m_dJacobianMatrix.resize(m_nStateDimension, m_nStateDimension);
	m_dSystemDynamicsMatrix = m_dSystemDynamicsMatrix.Identity(m_nStateDimension, m_nStateDimension);	
 	m_dInitialStateMatrix.resize(m_nStateDimension, 1);
	m_dInitialStateMatrix <<  MatrixXd::Zero(m_nStateDimension, 1) ;
	m_dInitialStateCovarianceMatrix = _P0;	
}



void CExtendedKalmanFilter::GetCosSinRPY (double roll, double pitch, double yaw)
{
	m_dCosRoll = cos (roll);
	m_dSinRoll = sin (roll);
	m_dCosPitch = cos (pitch);
	m_dSinPitch = sin (pitch);
	m_dCosYaw = cos(yaw);
	m_dSinYaw = sin(yaw);
}

double CExtendedKalmanFilter::clampRPY(double rotation)
  {
    cout<<"rotation is "<<rotation<<endl;
    while (rotation > M_PI)
    {
      rotation -= (2*M_PI);
    }

    while (rotation < -M_PI)
    {
      rotation += (2*M_PI);
    }

    return rotation;
  }
  
void CExtendedKalmanFilter::SetStateAngles()
{
    cout<<"Inside set angles roll"<<endl;
    m_dStateMatrix(eRoll,0)  = clampRPY(m_dStateMatrix(eRoll,0));
    cout<<"Inside set angles pitch"<<endl;
    m_dStateMatrix(ePitch,0) = clampRPY(m_dStateMatrix(ePitch,0));
    cout<<"Inside set angles yaw"<<endl;
    m_dStateMatrix(eYaw,0)   = clampRPY(m_dStateMatrix(eYaw,0));
}


void CExtendedKalmanFilter::SetSystemDynamicsMatrix ()
{

	m_dSystemDynamicsMatrix(ePose_x,eVelocity_x) = m_dCosYaw * m_dCosPitch *vse_delta ;
	m_dSystemDynamicsMatrix(ePose_x,eVelocity_y) = ( (m_dCosYaw * m_dSinPitch * m_dSinRoll) - (m_dSinYaw * m_dCosRoll) )*vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_x,eVelocity_z) = ( (m_dCosYaw * m_dSinPitch * m_dCosRoll) + (m_dSinYaw * m_dSinRoll) )*vse_delta ;
	m_dSystemDynamicsMatrix(ePose_x,eAcceleration_x) = 0.5 * m_dSystemDynamicsMatrix(ePose_x,eVelocity_x) *vse_delta ;
	m_dSystemDynamicsMatrix(ePose_x,eAcceleration_y) = 0.5 * m_dSystemDynamicsMatrix(ePose_x,eVelocity_y) *vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_x,eAcceleration_z) = 0.5 * m_dSystemDynamicsMatrix(ePose_x,eVelocity_z) *vse_delta ;
	
	m_dSystemDynamicsMatrix(ePose_y,eVelocity_x) = m_dSinYaw * m_dCosPitch *vse_delta ;
	m_dSystemDynamicsMatrix(ePose_y,eVelocity_y) = ( (m_dSinYaw * m_dSinPitch * m_dSinRoll) + (m_dCosYaw * m_dCosRoll) )*vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_y,eVelocity_z) = ( (m_dSinYaw * m_dSinPitch * m_dCosRoll) - (m_dCosYaw * m_dSinRoll) )*vse_delta ;
	m_dSystemDynamicsMatrix(ePose_y,eAcceleration_x) = 0.5 * m_dSystemDynamicsMatrix(ePose_y,eVelocity_x) *vse_delta ;
	m_dSystemDynamicsMatrix(ePose_y,eAcceleration_y) = 0.5 * m_dSystemDynamicsMatrix(ePose_y,eVelocity_y) *vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_y,eAcceleration_z) = 0.5 * m_dSystemDynamicsMatrix(ePose_y,eVelocity_z) *vse_delta ;
	
	m_dSystemDynamicsMatrix(ePose_z,eVelocity_x) = -1 * m_dSinPitch *vse_delta ;
	m_dSystemDynamicsMatrix(ePose_z,eVelocity_y) = ( m_dCosPitch * m_dSinRoll )*vse_delta ;
	m_dSystemDynamicsMatrix(ePose_z,eVelocity_z) = ( m_dCosPitch * m_dCosRoll )*vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_z,eAcceleration_x) = 0.5 * m_dSystemDynamicsMatrix(ePose_z,eVelocity_x) *vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_z,eAcceleration_y) = 0.5 * m_dSystemDynamicsMatrix(ePose_z,eVelocity_y) *vse_delta ;
	//m_dSystemDynamicsMatrix(ePose_z,eAcceleration_z) = 0.5 * m_dSystemDynamicsMatrix(ePose_z,eVelocity_z) *vse_delta ;
	
	//m_dSystemDynamicsMatrix(eRoll,eRoll_rate) = m_dSystemDynamicsMatrix(ePose_x,eVelocity_x) ;
	//m_dSystemDynamicsMatrix(eRoll,ePitch_rate) = m_dSystemDynamicsMatrix(ePose_x,eVelocity_y) ;
	//m_dSystemDynamicsMatrix(eRoll,eYaw_rate) = m_dSystemDynamicsMatrix(ePose_x,eVelocity_z) ;
	
	//m_dSystemDynamicsMatrix(ePitch,eRoll_rate) = m_dSystemDynamicsMatrix(ePose_y,eVelocity_x) ;
	//m_dSystemDynamicsMatrix(ePitch,ePitch_rate) = m_dSystemDynamicsMatrix(ePose_y,eVelocity_y) ;
	//m_dSystemDynamicsMatrix(ePitch,eYaw_rate) = m_dSystemDynamicsMatrix(ePose_y,eVelocity_z) ;
	
	m_dSystemDynamicsMatrix(eYaw,eRoll_rate) = m_dSystemDynamicsMatrix(ePose_z,eVelocity_x) ;
	m_dSystemDynamicsMatrix(eYaw,ePitch_rate) = m_dSystemDynamicsMatrix(ePose_z,eVelocity_y) ;
	m_dSystemDynamicsMatrix(eYaw,eYaw_rate) = m_dSystemDynamicsMatrix(ePose_z,eVelocity_z) ;
	
	//m_dSystemDynamicsMatrix(eVelocity_x,eAcceleration_x) = vse_delta ;
	//m_dSystemDynamicsMatrix(eVelocity_y,eAcceleration_y) = vse_delta ;
	//m_dSystemDynamicsMatrix(eVelocity_z,eAcceleration_z) = vse_delta ;
	
}  
  
void CExtendedKalmanFilter::SetJacobianMatrix()
{ 
	
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * vse_delta * vse_delta;

    yCoeff = m_dCosYaw * m_dSinPitch * m_dCosRoll + m_dSinYaw * m_dSinRoll;
    zCoeff = -m_dCosYaw * m_dSinPitch * m_dSinRoll + m_dSinYaw * m_dCosRoll;
    double dFx_dR = (yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFR_dR = 1 + (yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    xCoeff = -m_dCosYaw * m_dSinPitch;
    yCoeff = m_dCosYaw * m_dCosPitch * m_dSinRoll;
    zCoeff = m_dCosYaw * m_dCosPitch * m_dCosRoll;
    double dFx_dP = (xCoeff * eVelocity_x + yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (xCoeff * eAcceleration_x + yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFR_dP = (xCoeff * eRoll_rate + yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    xCoeff = -m_dSinYaw * m_dCosPitch;
    yCoeff = -m_dSinYaw * m_dSinPitch * m_dSinRoll - m_dCosYaw * m_dCosRoll;
    zCoeff = -m_dSinYaw * m_dSinPitch * m_dCosRoll + m_dCosYaw * m_dSinRoll;
    double dFx_dY = (xCoeff * eVelocity_x + yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (xCoeff * eAcceleration_x + yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFR_dY = (xCoeff * eRoll_rate + yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    yCoeff = m_dSinYaw * m_dSinPitch * m_dCosRoll - m_dCosYaw * m_dSinRoll;
    zCoeff = -m_dSinYaw * m_dSinPitch * m_dSinRoll - m_dCosYaw * m_dCosRoll;
    double dFy_dR = (yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFP_dR = (yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    xCoeff = -m_dSinYaw * m_dSinPitch;
    yCoeff = m_dSinYaw * m_dCosPitch * m_dSinRoll;
    zCoeff = m_dSinYaw * m_dCosPitch * m_dCosRoll;
    double dFy_dP = (xCoeff * eVelocity_x + yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (xCoeff * eAcceleration_x + yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFP_dP = 1 + (xCoeff * eRoll_rate + yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    xCoeff = m_dCosYaw * m_dCosPitch;
    yCoeff = m_dCosYaw * m_dSinPitch * m_dSinRoll - m_dSinYaw * m_dCosRoll;
    zCoeff = m_dCosYaw * m_dSinPitch * m_dCosRoll + m_dSinYaw * m_dSinRoll;
    double dFy_dY = (xCoeff * eVelocity_x + yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (xCoeff * eAcceleration_x + yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFP_dY = (xCoeff * eRoll_rate + yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    yCoeff = m_dCosPitch * m_dCosRoll;
    zCoeff = -m_dCosPitch * m_dSinRoll;
    double dFz_dR = (yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFY_dR = (yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    xCoeff = -m_dCosPitch;
    yCoeff = -m_dSinPitch * m_dSinRoll;
    zCoeff = -m_dSinPitch * m_dCosRoll;
    double dFz_dP = (xCoeff * eVelocity_x + yCoeff * eVelocity_y + zCoeff * eVelocity_z) * vse_delta +
                    (xCoeff * eAcceleration_x + yCoeff * eAcceleration_y + zCoeff * eAcceleration_z) * oneHalfATSquared;
    double dFY_dP = (xCoeff * eRoll_rate + yCoeff * ePitch_rate + zCoeff * eYaw_rate) * vse_delta;

    m_dJacobianMatrix = m_dSystemDynamicsMatrix;
    
    m_dJacobianMatrix(ePose_x, eRoll) = dFx_dR;
    m_dJacobianMatrix(ePose_x, ePitch) = dFx_dP;
    m_dJacobianMatrix(ePose_x, eYaw) = dFx_dY;
    m_dJacobianMatrix(ePose_y, eRoll) = dFy_dR;
    m_dJacobianMatrix(ePose_y, ePitch) = dFy_dP;
    m_dJacobianMatrix(ePose_y, eYaw) = dFy_dY;
    m_dJacobianMatrix(ePose_z, eRoll) = dFz_dR;
    m_dJacobianMatrix(ePose_z, ePitch) = dFz_dP;
    m_dJacobianMatrix(eRoll, eRoll) = dFR_dR;
    m_dJacobianMatrix(eRoll, ePitch) = dFR_dP;
    m_dJacobianMatrix(eRoll, eYaw) = dFR_dY;
    m_dJacobianMatrix(ePitch, eRoll) = dFP_dR;
    m_dJacobianMatrix(ePitch, ePitch) = dFP_dP;
    m_dJacobianMatrix(ePitch, eYaw) = dFP_dY;
    m_dJacobianMatrix(eYaw, eRoll) = dFY_dR;
    m_dJacobianMatrix(eYaw, ePitch) = dFY_dP;
  
}
  

void CExtendedKalmanFilter::setMeasurementAdaptationAngle(int data_enable,int angle_enable)
{
		if( data_enable == 1 && angle_enable == 1)
		{
		m_dMesaurementAdaptationMatrix(12,eRoll) = 1;
		m_dMesaurementAdaptationMatrix(13,ePitch) = 1;
		m_dMesaurementAdaptationMatrix(14,eYaw) = 1;
		}
		else
		{
		m_dMesaurementAdaptationMatrix(12,eRoll) = 0;
		m_dMesaurementAdaptationMatrix(13,ePitch) = 0;
		m_dMesaurementAdaptationMatrix(14,eYaw) = 0;
		}
	
}


void CExtendedKalmanFilter::setMeasurementAdaptationGps(int data_enable,int gps_enable)
{
		if( data_enable == 1 && gps_enable == 1)
		{
		m_dMesaurementAdaptationMatrix(ePose_x,ePose_x) = 1;
		m_dMesaurementAdaptationMatrix(ePose_y,ePose_y) = 1;
		m_dMesaurementAdaptationMatrix(ePose_z,ePose_z) = 1;
		}
		else
		{
		m_dMesaurementAdaptationMatrix(ePose_x,ePose_x) = 0;
		m_dMesaurementAdaptationMatrix(ePose_y,ePose_y) = 0;
		m_dMesaurementAdaptationMatrix(ePose_z,ePose_z) = 0;
		}
	
}

void CExtendedKalmanFilter::setMeasurementAdaptationImu(int data_enable, int imu_enable)
{
		if( data_enable == 1 && imu_enable == 1)
		{
			m_dMesaurementAdaptationMatrix(9,eAcceleration_x) = 1;
			m_dMesaurementAdaptationMatrix(10,eAcceleration_y) = 1;
			m_dMesaurementAdaptationMatrix(11,eAcceleration_z) = 1;
		}
		else
		{
			m_dMesaurementAdaptationMatrix(9,eAcceleration_x) = 0;
			m_dMesaurementAdaptationMatrix(10,eAcceleration_y) = 0;
			m_dMesaurementAdaptationMatrix(11,eAcceleration_z) = 0;	
		}
		
}


void CExtendedKalmanFilter::setMeasurementAdaptationWs(int data_enable,int ws_enable)
{
		if( data_enable == 1 && ws_enable == 1)
		{
			m_dMesaurementAdaptationMatrix(3,eVelocity_x) = 1;
			m_dMesaurementAdaptationMatrix(4,eVelocity_y) = 1;
			m_dMesaurementAdaptationMatrix(5,eVelocity_z) = 1;
		}
		else
		{
			m_dMesaurementAdaptationMatrix(3,eVelocity_x) = 0;
			m_dMesaurementAdaptationMatrix(4,eVelocity_y) = 0;
			m_dMesaurementAdaptationMatrix(5,eVelocity_z) = 0;
		}
		
}


void CExtendedKalmanFilter::setMeasurementNoiseCovarianceGps(float epx, float epy, float epz)
{

m_dMeasurementNoiseCovarianceMatrix(0,0) = epx;
m_dMeasurementNoiseCovarianceMatrix(1,1) = epy;
m_dMeasurementNoiseCovarianceMatrix(2,2) = epz;
}

void CExtendedKalmanFilter::predict(void)
{
	SetSystemDynamicsMatrix ();
	SetJacobianMatrix ();
	m_dStateMatrix = (m_dSystemDynamicsMatrix * m_dInitialStateMatrix);

	m_dStateCovarianceMatrix = (m_dJacobianMatrix * m_dInitialStateCovarianceMatrix * m_dJacobianMatrix.transpose()) + m_dProcessNoiseCovarianceMatrix;
	SetStateAngles();
}


void CExtendedKalmanFilter::correct (void ) 
{
cout<<"Inside correct"<<endl;
	m_dKalmanGainMatrix = ( m_dStateCovarianceMatrix * m_dMesaurementAdaptationMatrix.transpose() ) * ( m_dMesaurementAdaptationMatrix * m_dStateCovarianceMatrix * m_dMesaurementAdaptationMatrix.transpose() + m_dMeasurementNoiseCovarianceMatrix).inverse();
	
	
	if(m_dMesaurementAdaptationMatrix(0,0) == 1)
	{
	}

	if(m_dMesaurementAdaptationMatrix(3,3) == 1)
	{
	}
	
	Eigen::MatrixXd zhx = (m_dMeasurementMatrix - m_dMesaurementAdaptationMatrix * m_dStateMatrix);

	zhx(12,0) = clampRPY(zhx(12,0));
	zhx(13,0) = clampRPY(zhx(13,0));
	zhx(14,0) = clampRPY(zhx(14,0));

	m_dStateMatrix = m_dStateMatrix + m_dKalmanGainMatrix*(zhx);
	Eigen::MatrixXd ikh = (m_dIdentityMatrix - m_dKalmanGainMatrix * m_dMesaurementAdaptationMatrix);
	m_dStateCovarianceMatrix = ((ikh) * m_dStateCovarianceMatrix * (ikh.transpose())) + (m_dKalmanGainMatrix * m_dMeasurementNoiseCovarianceMatrix *m_dKalmanGainMatrix.transpose());

	SetStateAngles();
	cout<<"Correct over"<<endl;
	m_dInitialStateMatrix = m_dStateMatrix;
	m_dInitialStateCovarianceMatrix = m_dStateCovarianceMatrix;

}


