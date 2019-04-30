#include "penSimulator.h"
#define PI 3.14159
#define G 9.81

//This function wraps an angle to a value between -pi and pi
double wrap(double theta){
    theta = fmod(theta + PI,2*PI);
    if (theta < 0)
        theta += 2*PI;
    return theta - PI;
}

PenSimulator::PenSimulator(ros::NodeHandle *nodehandle):
		pen_mass_(.38),
		rod_mass_(.095),
		length_(.43),
		damper_(.1),
		angle_(0.),
		vel_(0.),
		acc_(0.),
		nh_(*nodehandle)
{
	nh_.getParam("M", pen_mass_);
	nh_.getParam("m", rod_mass_);
	nh_.getParam("l", length_);
	nh_.getParam("b", damper_);
	calcMomentofInertia();
	t0_ = ros::Time::now();
	input_publisher_ = nh_.advertise<pendulum_control::CustomMsg>("/control_input", 100);
	viz_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/pendulum_viz", 100);
	output_subscriber_ =nh_.subscribe("/control_output", 100, &PenSimulator::outputCallback, this);
}

void PenSimulator::calcMomentofInertia(){
	cg_ = length_*(pen_mass_+.5*rod_mass_)/(pen_mass_+rod_mass_);
	mI_ = length_*length_*(pen_mass_+rod_mass_/3);
}

void PenSimulator::outputCallback(const geometry_msgs::Wrench &wrench)
{
	ros::Duration dt = ros::Time::now() - t0_;
	// derivations modified from http://ctms.engin.umich.edu/CTMS/index.php?aux=Activities_Pendulum
	acc_ = wrench.torque.z / mI_ - (pen_mass_+rod_mass_) * G * cg_ * sin(angle_) / mI_ - damper_ * vel_ / mI_;
	vel_ = acc_ * dt.toSec() + vel_;
	angle_ = wrap(vel_ * dt.toSec() + angle_);
	t0_ = ros::Time::now();
}

void PenSimulator::publishInput(){
	pendulum_control::CustomMsg custom;
	custom.angle = angle_;
	custom.vel = vel_;
	custom.acc = acc_;
	input_publisher_.publish(custom);
}

void PenSimulator::publishViz(){
	double x_;
	double y_;
	// mass
	x_ = length_ * cos(angle_);
	y_ = length_ * sin(angle_);
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray  markerArray;
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "pendulum";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.pose.position.x = x_;
	marker.pose.position.y = y_;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    markerArray.markers.push_back(marker);
    // rod
	x_ = .5 * length_ * cos(angle_);
	y_ = .5 * length_ * sin(angle_);
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "/world";
	marker2.header.stamp = ros::Time::now();
	marker2.ns = "pendulum";
	marker2.id = 1;
	marker2.type = visualization_msgs::Marker::CYLINDER;
	marker2.pose.position.x = x_;
	marker2.pose.position.y = y_;
	marker2.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(PI/2, 0, PI/2+angle_);
	tf2::convert(q, marker2.pose.orientation);
    marker2.scale.x = .01;
    marker2.scale.y = .01;
    marker2.scale.z = length_;
    marker2.color.r = 0.0f;
    marker2.color.g = 0.0f;
    marker2.color.b = 1.0f;
    marker2.color.a = 1.0;
    marker2.lifetime = ros::Duration();

    markerArray.markers.push_back(marker2);
    viz_publisher_.publish(markerArray);
}

PenSimulator::~PenSimulator(){

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pendulumSimulator");
	ros::NodeHandle nh("~");;
	PenSimulator pen_simluator(&nh);
	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		pen_simluator.publishInput();
		pen_simluator.publishViz();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

