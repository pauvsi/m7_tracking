#include "ground_vehicle.h"

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

	//this->P.setConstant(10000);
	P(0,0) = P(1,1) = P(2,2) = P(3,3) = 10000;

	this->Q.setZero();
		Q(0,0) = Q(1,1) = 0.1*dt;
		Q(2,2) = Q(3,3) = 0.5*dt;

	//this->R.setConstant(0.25);
	R(0,0) = R(1,1) = 0.25;

	this->H.setZero();
	H(0,0) = 1; H(1,1) = 1;

	this->I.setIdentity();


	initialized = false;
	firstRun = true;
}

void GroundVehicle::init(double t0, const Eigen::Matrix<double, 4, 1>& x0)
{
	x_hat = x_hat_new = x0;
	this->t0 = t0;
	t = t0;
	initialized = true;

}

void GroundVehicle::predict(std_msgs::Header imageHeader)
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("Not Initialized!");
		return;
	}
	ROS_WARN_STREAM("IN PREDICT");

	double timeDiff = (imageHeader.stamp.toSec() - dataHeader.stamp.toSec());
	ROS_WARN_STREAM("TIMEDIFF:"<<timeDiff);

//TODO: Check and make sure
	F(0,2) = F(1,3) = timeDiff;
	Q(0,0) = Q(1,1) = 0.1*timeDiff;
	Q(2,2) = Q(3,3) = 0.5*timeDiff;
	x_hat_new = F*x_hat;

	ROS_WARN_STREAM("F"<<F);
	ROS_WARN_STREAM("PRED X_HAT_NEW:\n"<<x_hat_new);

//	dt = dt + timeStep;
//	ROS_WARN_STREAM("Dt after predict:"<<dt);
}

void GroundVehicle::update(const Eigen::Matrix<double, 2, 1>& z, std_msgs::Header imageHeader)
{
	if(!initialized)
	{
		ROS_DEBUG_STREAM("NOT INITIALIZED");
		return;
	}


	if(!firstRun)
	{
		double timeDiff = (imageHeader.stamp.toSec() - dataHeader.stamp.toSec());
		F(0,2) = F(1,3) = timeDiff;
		Q(0,0) = Q(1,1) = 0.1*timeDiff;
		Q(2,2) = Q(3,3) = 0.5*timeDiff;
		x_hat_new = F*x_hat;
		P = F*P*F.transpose() + Q;
	}
	else
		firstRun = false;

	ROS_WARN_STREAM("P\n"<<P);
	ROS_WARN_STREAM("R\n"<<R);

	Eigen::Matrix<double, 2, 1> y = z - H*x_hat_new;

	Eigen::Matrix<double, 2, 2> S = H*P*H.transpose() + R;
	ROS_WARN_STREAM("S\n"<<S);
	K = P*H.transpose()*S.inverse();
	ROS_WARN_STREAM("K:\n"<<K);
	x_hat = x_hat_new + K*(y);
	ROS_WARN_STREAM("X_HAT_NEW\n"<<x_hat_new);
	P = (I - K*H)*P;
//	x_hat = x_hat_new;

	ROS_WARN_STREAM("X_HAT: \n"<<x_hat);

	t = t + imageHeader.stamp.toSec() - dataHeader.stamp.toSec() ;
	dataHeader = imageHeader;
}

geometry_msgs::PoseStamped GroundVehicle::getPoseStamped(std_msgs::Header imageHeader)
{
	//TODO: predict then give it, or don't predict?
	this->predict(imageHeader);
	geometry_msgs::PoseStamped pose;// = new geometry_msgs::PoseStamped(x_hat[0], x_hat[1], 0);
	//TODO: Set ros time, to the time of the image
	pose.header = dataHeader;
	pose.header.stamp.sec = imageHeader.stamp.nsec - dataHeader.stamp.nsec;
	pose.header.stamp.nsec = imageHeader.stamp.nsec - dataHeader.stamp.nsec;
	pose.pose.position.x = x_hat_new(0, 0);
	pose.pose.position.y = x_hat_new(1, 0);
	pose.pose.position.z = ROOMBA_HEIGHT;

	//Orientation x&y are x velocity and y velocity
	pose.pose.orientation.x = x_hat_new(2, 0);
	pose.pose.orientation.y = x_hat_new(3, 0);

	return pose;
}

geometry_msgs::PoseWithCovarianceStamped GroundVehicle::getPoseWithCovariance(std_msgs::Header imageHeader)
{
	this->predict(imageHeader);
	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header = dataHeader;
	pose.header.stamp.sec = imageHeader.stamp.sec - dataHeader.stamp.sec;
	pose.header.stamp.nsec = imageHeader.stamp.nsec - dataHeader.stamp.nsec;
	//	pose.header.stamp =ros::Time(ros::Time(0));
	pose.pose.pose.position.x = x_hat_new(0, 0);
	pose.pose.pose.position.y = x_hat_new(1, 0);
	pose.pose.pose.position.z = ROOMBA_HEIGHT;

	//Orientation x&y are x velocity and y velocity
	pose.pose.pose.orientation.x = x_hat_new(2, 0);
	pose.pose.pose.orientation.y = x_hat_new(3, 0);

	boost::array<double, 36ul> covar;
	for(int i=0; i<36; ++i)
		covar[i] = 0.001;
	covar[0] = P(0,0); covar[1] = P(0, 1); covar[6] = P(1, 0); covar[7] = P(1, 1);

	//Not certain about yaw
	covar[5] = covar[11] = covar[17] = covar[23] = covar[29] = covar[35] = 1000;


	pose.pose.covariance = covar;

	return pose;

}




