#ifndef _PEN_CONTROLLER_H_
#define _PEN_CONTROLLER_H_

#include "ros/ros.h"  //ROS Library
#include "geometry_msgs/Wrench.h"  //Needed to publish force/torque
#include "pendulum_control/CustomMsg.h"

class PenController //Controller class definition
{
	private:
		ros::NodeHandle nh_;  // Node handle
		ros::Publisher output_publisher_;  // control_output
		ros::Subscriber input_subscriber_;  // control_input

		// Controller parameters
		double angle_ref_;
		double Kp_;
		double Kd_;
		// Methods
		void inputCallback(const pendulum_control::CustomMsg &data);

	public:
		PenController(ros::NodeHandle *nodehandle);
		~PenController();
};

#endif
