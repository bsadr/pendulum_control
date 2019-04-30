#include "penController.h"
#define PI 3.14159

//This function wraps an angle to a value between -pi and pi
double wrap(double theta){
    theta = fmod(theta + PI,2*PI);
    if (theta < 0)
        theta += 2*PI;
    return theta - PI;
}

PenController::PenController(ros::NodeHandle *nodehandle):
		Kp_(2.0),
		Kd_(.3),
		angle_ref_(PI),
		nh_(*nodehandle)
{
	nh_.getParam("Kp", Kp_);
	nh_.getParam("Kd", Kd_);
	nh_.getParam("reference", angle_ref_);
	output_publisher_ = nh_.advertise<geometry_msgs::Wrench>("/control_output", 100);
	input_subscriber_ = nh_.subscribe("/control_input", 100, &PenController::inputCallback, this);
}

void PenController::inputCallback(const pendulum_control::CustomMsg &data)
{
	double e = wrap(angle_ref_ - data.angle);
	double de =  -data.vel;
	double u = Kp_*e + Kd_*de;

	geometry_msgs::Wrench wrench;
	wrench.torque.z = u;
	output_publisher_.publish(wrench);
}

PenController::~PenController(){

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pendulumController");
	ros::NodeHandle nh("~");
	PenController pen_controller(&nh);
	ros::spin();
	return 0;
}

