ATRV Jr drivers and bringup
===========================

Contains rflex driver based off of b21 driver by David V. Lu.
The rflex driver works well for driving. Other functionality (e.g. sonars) might not
work.

In addition to the original b21 driver, the driver has
a dynamic reconfigure that allows to adjust torque.

The stack still includes the b21 driver, which could be removed in the future.

The bringup files are specific for ATRV Jr used at UMass Lowell Robotics Lab, which has front
facing SICK rangefinder and back facing URG.
