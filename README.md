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

# Frontend Interface
For the Unity frontend interface, please see 

https://github.com/uml-robotics/ATRV_JR_Frontend


# STARTUP
You can run 

```ros2 launch atrv.launch.py```

to startup every node, or start them up individually below

## Robot Driver

```ros2 launch atrvjr_bringup rflex.launch.py```

## Sensors

Start all sensors with TF frames with

```ros2 launch atrvjr_bringup sensors.launch.py```

Or start each sensor individually with

```ros2 launch atrvjr_bringup front_camera.launch.py```

```ros2 launch atrvjr_bringup rear_camera.launch.py```

```ros2 launch atrvjr_bringup rear_lidar.launch.py```

```ros2 launch atrvjr_nav2 tf.launch.py```

# Full navigation (supply your map)
```ros2 launch atrvjr_nav2 navigation.launch.py map:=/path/to/your/map.yaml```

# Unity communication
```ros2 launch ros-tcp-endpoint endpoint.py```

# Dependencies
ros-humble-tf-transformations
