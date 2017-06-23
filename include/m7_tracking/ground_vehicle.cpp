#include "ground_vehicle.hpp"

GroundVehicle::GroundVehicle(double dt,
									const Eigen::Matrix4d& F,
									const Eigen::Matrix<double, 2, 4>& H,
									const Eigen::Matrix4d& Q,
									const Eigen::Matrix<double, 2, 2>& R,
									const Eigen::Matrix4d& P)
{
	this->F = F;
	this->H = H;
	this->Q = Q;
	this->R = R;
	this->P = P;
	this->dt = dt;
	this->timeStep = dt;
	this->I.setIdentity();
	initialized = false;
}

GroundVehicle::GroundVehicle()
{
	this->timeStep = TIME_STEP;
	dt = TIME_STEP;

	this->F.setIdentity();
		F(0,2) = F(1, 3) = dt;

	this->P.setConstant(10000);
	this->Q.setZero();
		Q(0,0) = Q(1,1) = 0.1*dt;
		Q(2,2) = Q(3,3) = 0.5*dt;

	this->R.setConstant(0.25);
	this->H.setZero();
		H(0,0) = 1; H(1,1) = 1;

	this->I.setIdentity();

	initialized = false;
}

void GroundVehicle::init(double t0, const Eigen::Matrix<double, 4, 1>& x0)
{
	x_hat = x0;
	this->t0 = t0;
	t = t0;
	initialized = true;

}

void GroundVehicle::predict()
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("Not Initialized!");
		return;
	}

//TODO: Check and make sure
	//	F(0,2) = F(1,3) = dt;
	//	Q(0,0) = Q(1,1) = 0.1*dt;
	//  Q(2,2) = Q(3,3) = 0.5*dt;
	x_hat_new = F*x_hat;
	P = F*P*F.transpose() + Q;
	//	dt = dt + timeStep;

}

void GroundVehicle::update(const Eigen::Matrix<double, 2, 1>& y, std_msgs::Header imageHeader)
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("NOT INITIALIZED");
		return;
	}

	Eigen::Matrix<double, 2, 2> S = H*P*H.transpose() + R;
	K = P*H.transpose()*S.inverse();
	x_hat_new += K*(y - H*x_hat_new);
	P = (I - K*H)*P;
	x_hat = x_hat_new;


	t = t+dt;
	dt = timeStep; //----------------
	dataHeader = imageHeader;
}


