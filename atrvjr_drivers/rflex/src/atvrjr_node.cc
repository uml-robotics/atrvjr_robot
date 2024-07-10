#define BOOST_BIND_NO_PLACEHOLDERS
#include <string>
#include <memory>
#include <boost/bind.hpp>
#include <angles/angles.h>

using std::placeholders::_1;

#include "rflex/atrvjr_driver.h"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <sensor_msgs/msg/point_cloud.hpp>

class atrv_jr_node : public rclcpp::Node
{
    protected:
        ATRVJR* driver;
        bool initialized = false;
        double first_bearing;
        bool isSonarOn;
        double last_distance, last_bearing;
        double x_odo, y_odo, a_odo;
        int prev_bumps;

        

        //Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_accel_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sonar_power_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_brake_power_sub;

        //Publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sonar_power_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_power_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plugged_in_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        // rclcpp::Publisher<> sonar_cloud_pub;

        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

        void cmd_vel_callback(const geometry_msgs::msg::Twist &msg) 
        {
            RCLCPP_INFO(this->get_logger(), "I heard: \n\t'Linear Velocity: %f\n\tAngular Velocity: %f'", msg.linear.x, msg.angular.z);
            driver->setVelocity(msg.linear.x, msg.angular.z);
        }

        void cmd_accel_callback(const std_msgs::msg::Float32 &msg) 
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.data);
            this->driver->config.setTransAcc(msg.data);
        }

        void cmd_sonar_power_callback(const std_msgs::msg::Bool &msg) 
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
            isSonarOn=msg.data;
            this->driver->setSonarPower(msg.data);
            
            this->driver->sendSystemStatusCommand();
        }

        void cmd_brake_power_callback(const std_msgs::msg::Bool &msg) 
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg.data);
            driver->setBrakePower(msg.data);
        }

        int initialize_driver(const std::string port) 
        {
            int ret = driver->initialize(port.c_str());
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize driver");
                return ret;
            }
            driver->setOdometryPeriod(100000);
            driver->setDigitalIoPeriod(100000);
            driver->motionSetDefaults();
            return 0;
        }



        void system_status_callback() {
            // Publish the sonar power status
            std_msgs::msg::Bool sonar_power;
            sonar_power.data = false;//driver->getSonarPower();
            sonar_power_pub->publish(sonar_power);

            // Publish the brake power status
            std_msgs::msg::Bool brake_power;
            brake_power.data = driver->getBrakePower();
            brake_power_pub->publish(brake_power);

            // Publish the voltage
            std_msgs::msg::Float32 voltage;
            voltage.data = driver->getVoltage();
            voltage_pub->publish(voltage);

            // Publish the plugged in status
            std_msgs::msg::Bool plugged_in;
            plugged_in.data = driver->isPluggedIn();
            plugged_in_pub->publish(plugged_in);
        }

        void motor_update_callback() {
            if (driver->isOdomReady()) {
                // get all of the odom data 
                double distance = driver->getDistance();
                double true_bearing = angles::normalize_angle(driver->getBearing());

                if(!this->initialized) {
                    this->initialized = true;
                    first_bearing = true_bearing;
                    x_odo = 0;
                    y_odo = 0;
                    a_odo = 0*true_bearing;
                } else {
                    double delta_distance = distance - last_distance;
                    double delta_bearing = true_bearing - last_bearing;
                    double delta_x = delta_distance * cos(a_odo);
                    double delta_y = delta_distance * sin(a_odo);
                    x_odo += delta_x;
                    y_odo += delta_y;
                    a_odo += delta_bearing;
                    a_odo = angles::normalize_angle(a_odo);
                }

                last_distance = distance;
                last_bearing = true_bearing - first_bearing;
                 tf2::Quaternion q;
                q.setRPY(0,0,last_bearing);
                geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

                geometry_msgs::msg::TransformStamped odom_trans;
                odom_trans.header.stamp = this->now();
                odom_trans.header.frame_id = this->get_parameter("odometry_frame_id").as_string();
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = x_odo;
                odom_trans.transform.translation.y = y_odo;
                odom_trans.transform.rotation = odom_quat;

                broadcaster->sendTransform(odom_trans);

                nav_msgs::msg::Odometry odometry;
                odometry.header.frame_id = this->get_parameter("odometry_frame_id").as_string();
                odometry.pose.pose.position.x = x_odo;
                odometry.pose.pose.position.y = y_odo;
                odometry.pose.pose.orientation = odom_quat;
                odometry.header.stamp = this->now();

                for(int i = 0; i < 6; i++) {
                    odometry.pose.covariance[i * 7] = 1;
                }

                odometry.child_frame_id = "base_link";
                double tvel = driver->getTransVelocity();
                odometry.twist.twist.linear.x = tvel*cos(a_odo);
                odometry.twist.twist.linear.y = tvel*sin(a_odo);
                odometry.twist.twist.angular.z = driver->getRotVelocity();

                odom_pub->publish(odometry);

                sensor_msgs::msg::JointState joint_state;
                joint_state.header.stamp = this->now();
                joint_state.name.resize(1);
                joint_state.position.resize(1);
                joint_state.name[0] = "lewis_twist";
                joint_state.position[0] = true_bearing;

                state_pub->publish(joint_state);
            }
        }

        void sonar_update_callback() {
            // Publish the sonar data
            //sensor_msgs::msg::PointCloud sonar_cloud;
            //sonar_cloud.header.stamp = this->now();
            //sonar_cloud.header.frame_id = "base_link";
            //driver->getBaseSonarPoints(&sonar_cloud);
            //this->publish("sonar_cloud_base", sonar_cloud);
        }

    public:
        atrv_jr_node()
        : Node("atrv_jr_node")
        {
            // Declare parameters
            RCLCPP_INFO(this->get_logger(), "Initializing params\n");
            this->declare_parameter("odo_distance_conversion",93810);
            this->declare_parameter("odo_angle_conversion", 38500);
            this->declare_parameter("trans_acceleration", 0.7);
            this->declare_parameter("trans_torque", 0.3);
            this->declare_parameter("rot_acceleration", 2.6);
            this->declare_parameter("rot_torque", 0.9);
            this->declare_parameter("power_offset", 1.2);
            this->declare_parameter("plugged_threshold", 1.2);
            this->declare_parameter("odometry_frame_id", "odom");
            this->declare_parameter("port", "/dev/ttyUSB0");

            driver = new ATRVJR(this->get_clock());

            // Get parameters
            RCLCPP_INFO(this->get_logger(), "Getting params\n");
            if(this->initialize_driver(this->get_parameter("port").as_string())) {
                throw std::runtime_error("Failed to initialize driver");
            }

            driver->config.setOdoDistanceConversion(this->get_parameter("odo_distance_conversion").as_int());
            driver->config.setOdoAngleConversion(this->get_parameter("odo_angle_conversion").as_int());
            driver->config.setTransAcc(this->get_parameter("trans_acceleration").as_double());
            driver->config.setTransTorque(this->get_parameter("trans_torque").as_double());
            driver->config.setRotAcc(this->get_parameter("rot_acceleration").as_double());
            driver->config.setRotTorque(this->get_parameter("rot_torque").as_double());

            RCLCPP_INFO(this->get_logger(), "Got params\n Creating subs\n");

            // Node Subscribers
            cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&atrv_jr_node::cmd_vel_callback, this, _1));
            cmd_accel_sub = this->create_subscription<std_msgs::msg::Float32>("cmd_accel", 10, std::bind(&atrv_jr_node::cmd_accel_callback, this, _1));
            cmd_sonar_power_sub = this->create_subscription<std_msgs::msg::Bool>("cmd_sonar_power", 10, std::bind(&atrv_jr_node::cmd_sonar_power_callback, this, _1));
            cmd_brake_power_sub = this->create_subscription<std_msgs::msg::Bool>("cmd_brake_power", 10, std::bind(&atrv_jr_node::cmd_brake_power_callback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Created subs\n Creating pubs\n");
            //Node Publishers
            //this->create_publisher<sensor_msgs::msg::PointCloud>("sonar_cloud_base", 50);
            sonar_power_pub = this->create_publisher<std_msgs::msg::Bool>("sonar_power", 1);
            brake_power_pub = this->create_publisher<std_msgs::msg::Bool>("brake_power", 1);
            voltage_pub = this->create_publisher<std_msgs::msg::Float32>("voltage", 1);
            odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);
            plugged_in_pub = this->create_publisher<std_msgs::msg::Bool>("plugged_in", 1);
            state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_state",1);

            RCLCPP_INFO(this->get_logger(), "Creating tf broadcaster\n");

            broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            RCLCPP_INFO(this->get_logger(), "Created pubs\n Initializing Driver\n");
            //timer_ = this->create_wall_timer(
            //std::chrono::milliseconds(500), std::bind(&atrv_jr_node::timer_callback, this));

            driver->systemStatusUpdateSignal.set(boost::bind(&atrv_jr_node::system_status_callback, this));
            driver->motorUpdateSignal.set(boost::bind(&atrv_jr_node::motor_update_callback, this));

            RCLCPP_INFO(this->get_logger(), "Driver Initialized\n Ready for use\n");
            //driver->sonarUpdateSignal.set(boost::bind(&atrv_jr_node::sonar_update_callback, this));

        }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<atrv_jr_node>());
    rclcpp::shutdown();
    return 0;
}