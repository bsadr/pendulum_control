#ifndef _PEN_SIMULATOR_H_
#define _PEN_SIMULATOR_H_

#include "ros/ros.h"  //ROS Library
#include "geometry_msgs/Wrench.h"  //Needed to read force/torque
#include "pendulum_control/CustomMsg.h"
#include "visualization_msgs/Marker.h"  	 //Needed for publishing visualization message
#include "visualization_msgs/MarkerArray.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Time.h"
#include <time.h>
#include <math.h>  //Needed for functions like sin and cos

class PenSimulator //Simulator class definition
{
	private:
		ros::NodeHandle nh_;  // Node handle
		ros::Publisher input_publisher_;  // control_input
		ros::Publisher viz_publisher_;  // pendulum_viz
		ros::Subscriber output_subscriber_;  // control_output
		ros::Time t0_;
		double pen_mass_;
		double rod_mass_;
		double length_;
		double cg_;			// center of gravity
		double mI_;			// moment of inertia
		double damper_;		// damper coefficient
		double angle_;		// pendulum angle
		double vel_;	// pendulum velocity
		double acc_;  		// pendulum acceleration
		void outputCallback(const geometry_msgs::Wrench &wrench);
		void calcMomentofInertia();
	public:
		PenSimulator(ros::NodeHandle *nodehandle);
		void publishInput();
		void publishViz();
		~PenSimulator();
};

#endif
