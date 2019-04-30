 
# pendulum_control

This package simulates an inverted pendulum connected to a motor with a fixed arm and controls it using torque into a stable upright position. The considered angle is the angle between the rod and the vertical axis. It is zero at the steady-state free motion of the rod.

## Nodes:
- `simulator`: this node simulates the pendulum motion and send visualization commands to rviz. Here are the parameters of this node
    - `M`: mass of the ball
    - `m`: mass of the rod
    - `l`: length of the rod
    - `b`: damping coefficient considered for the angular friction of the rod
- 'controller': this node controls the pendulum motion usinf a PD controller
    - `Kp`: proportional gain of the controller
    - `Kd`: derivative gain the controller
    - `reference`: reference angle of the rod (default value pi) 
    - `b`: damping coefficient considered for the angular friction of the rod

## Setup:
No non standard libraries used.

## Launch:
'roslaunch pendulum_control pendulum.launch