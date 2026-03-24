"""
ATRV Jr ROS2 TF Publisher

Static transforms:
  base_footprint -> base_link
  tilt_link      -> front_camera
  base_link      -> rear_camera   (facing backwards, 180° yaw)
  base_link      -> front_laser
  base_link      -> rear_laser    (facing backwards, 180° yaw)

Dynamic transforms (driven by /joint_states):
  base_link -> pan_link   (pan joint  — rotates about Z)
  pan_link  -> tilt_link  (tilt joint — rotates about Y)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
from tf_transformations import quaternion_from_euler


def _make_static(frame_id: str, child_frame_id: str,
                 tx: float, ty: float, tz: float,
                 roll: float, pitch: float, yaw: float) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    q = quaternion_from_euler(roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t


class AtrvjrTfPublisher(Node):
    def __init__(self):
        super().__init__('atrvjr_tf_publisher')

        # ── Parameters ──────────────────────────────────────────────────────
        # base_footprint -> base_link
        self.declare_parameter('base_link_z', 0.30)

        # base_link -> pan_link  (pan-tilt unit mount point)
        self.declare_parameter('pan_mount_x', 0.10)
        self.declare_parameter('pan_mount_y', 0.00)
        self.declare_parameter('pan_mount_z', 0.50)

        # pan_link -> tilt_link  (tilt axis offset from pan axis)
        self.declare_parameter('tilt_offset_x', 0.00)
        self.declare_parameter('tilt_offset_y', 0.00)
        self.declare_parameter('tilt_offset_z', 0.05)

        # tilt_link -> front_camera
        self.declare_parameter('front_camera_x', 0.05)
        self.declare_parameter('front_camera_y', 0.00)
        self.declare_parameter('front_camera_z', 0.03)

        # base_link -> rear_camera  (mounted at rear, facing backwards)
        self.declare_parameter('rear_camera_x', -0.30)
        self.declare_parameter('rear_camera_y',  0.00)
        self.declare_parameter('rear_camera_z',  0.50)

        # base_link -> front_laser  (SICK — matches legacy launch)
        self.declare_parameter('front_laser_x',  0.40)
        self.declare_parameter('front_laser_y',  0.00)
        self.declare_parameter('front_laser_z',  0.10)

        # base_link -> rear_laser   (URG — matches legacy launch)
        self.declare_parameter('rear_laser_x', -0.30)
        self.declare_parameter('rear_laser_y',  0.00)
        self.declare_parameter('rear_laser_z',  0.70)

        # Joint names on /joint_states
        self.declare_parameter('pan_joint_name',  'pan_joint')
        self.declare_parameter('tilt_joint_name', 'tilt_joint')

        # ── TF broadcasters ──────────────────────────────────────────────────
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)

        self._publish_static_transforms()

        # ── State ────────────────────────────────────────────────────────────
        self._pan_angle  = 0.0
        self._tilt_angle = 0.0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_states_cb, 10)

        # Publish dynamic transforms at 50 Hz even when joints are not moving
        self.create_timer(0.02, self._publish_dynamic_transforms)

        self.get_logger().info('ATRV Jr TF publisher started.')

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _p(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _s(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    # ── Static transforms ─────────────────────────────────────────────────

    def _publish_static_transforms(self):
        now = self.get_clock().now().to_msg()

        transforms = [
            # base_footprint -> base_link  (robot body above ground)
            _make_static('base_footprint', 'base_link',
                         0.0, 0.0, self._p('base_link_z'),
                         0.0, 0.0, 0.0),

            # tilt_link -> front_camera  (camera on pan-tilt, faces forward)
            _make_static('tilt_link', 'front_camera',
                         self._p('front_camera_x'),
                         self._p('front_camera_y'),
                         self._p('front_camera_z'),
                         0.0, 0.0, 0.0),

            # base_link -> rear_camera  (facing backwards: 180° yaw)
            _make_static('base_link', 'rear_camera',
                         self._p('rear_camera_x'),
                         self._p('rear_camera_y'),
                         self._p('rear_camera_z'),
                         0.0, 0.0, math.pi),

            # base_link -> front_laser  (SICK, faces forward)
            _make_static('base_link', 'front_laser',
                         self._p('front_laser_x'),
                         self._p('front_laser_y'),
                         self._p('front_laser_z'),
                         0.0, 0.0, 0.0),

            # base_link -> rear_laser   (URG, faces backwards: 180° yaw)
            _make_static('base_link', 'rear_laser',
                         self._p('rear_laser_x'),
                         self._p('rear_laser_y'),
                         self._p('rear_laser_z'),
                         0.0, 0.0, math.pi),
        ]

        for t in transforms:
            t.header.stamp = now

        self._static_broadcaster.sendTransform(transforms)

    # ── Dynamic transforms ────────────────────────────────────────────────

    def _joint_states_cb(self, msg: JointState):
        pan_name  = self._s('pan_joint_name')
        tilt_name = self._s('tilt_joint_name')

        for i, name in enumerate(msg.name):
            if name == pan_name and i < len(msg.position):
                self._pan_angle = msg.position[i]
            elif name == tilt_name and i < len(msg.position):
                self._tilt_angle = msg.position[i]

    def _publish_dynamic_transforms(self):
        now = self.get_clock().now().to_msg()

        # base_link -> pan_link  (pan rotates about Z)
        pan_q = quaternion_from_euler(0.0, 0.0, self._pan_angle)
        pan_tf = TransformStamped()
        pan_tf.header.stamp = now
        pan_tf.header.frame_id = 'base_link'
        pan_tf.child_frame_id  = 'pan_link'
        pan_tf.transform.translation.x = self._p('pan_mount_x')
        pan_tf.transform.translation.y = self._p('pan_mount_y')
        pan_tf.transform.translation.z = self._p('pan_mount_z')
        pan_tf.transform.rotation.x = pan_q[0]
        pan_tf.transform.rotation.y = pan_q[1]
        pan_tf.transform.rotation.z = pan_q[2]
        pan_tf.transform.rotation.w = pan_q[3]

        # pan_link -> tilt_link  (tilt rotates about Y)
        tilt_q = quaternion_from_euler(0.0, self._tilt_angle, 0.0)
        tilt_tf = TransformStamped()
        tilt_tf.header.stamp = now
        tilt_tf.header.frame_id = 'pan_link'
        tilt_tf.child_frame_id  = 'tilt_link'
        tilt_tf.transform.translation.x = self._p('tilt_offset_x')
        tilt_tf.transform.translation.y = self._p('tilt_offset_y')
        tilt_tf.transform.translation.z = self._p('tilt_offset_z')
        tilt_tf.transform.rotation.x = tilt_q[0]
        tilt_tf.transform.rotation.y = tilt_q[1]
        tilt_tf.transform.rotation.z = tilt_q[2]
        tilt_tf.transform.rotation.w = tilt_q[3]

        self._dynamic_broadcaster.sendTransform([pan_tf, tilt_tf])


def main(args=None):
    rclpy.init(args=args)
    node = AtrvjrTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
