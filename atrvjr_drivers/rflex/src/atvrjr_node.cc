#include <string>
#include <boost/bind.hpp>


#include "rflex/atrvjr_driver.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
//#include <sensor_msgs/msg/point_cloud.hpp>

class atvr_jr_node : public rclcpp::Node
{
    protected:
        ATVRJR driver;
        bool initialized = false;
        double first_bearing;
        bool isSonarOn;
        double last_distance, last_bearing;
        double x_odo, y_odo, a_odo;
        int prev_bumps;

        tf::TransformBroadcaster broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        dynamic_reconfigure::Server<rflex::AtrvjrParamsConfig> reconfigure_srv_;

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
            driver->setVelocity(msg->linear.x, msg->angular.z);
        }

        void cmd_accel_callback(const std_msgs::msg::Float32::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
            driver->setTransAcc(msg->data);
        }

        void cmd_sonar_power_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
            driver->setSonarPower(msg->data);
            driver->sendSystemStatusCommand();
        }

        void cmd_brake_power_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
            driver->setBrakePower(msg->data);
        }

        int initialize_driver(int port) {
            int ret = driver.initialize(port);
            if (ret < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize driver");
                return ret;
            }
            driver.setOdomPeriod(100000);
            driver.setDigitalIoPeriod(100000);
            driver.motionSetDefaults();
            return 0;
        }

        /*void timer_callback() {

        }*/

        void system_status_callback() {
            // Publish the sonar power status
            std_msgs::msg::Bool sonar_power;
            sonar_power.data = driver->getSonarPower();
            this->publish("sonar_power", sonar_power);

            // Publish the brake power status
            std_msgs::msg::Bool brake_power;
            brake_power.data = driver->getBrakePower();
            this->publish("brake_power", brake_power);

            // Publish the voltage
            std_msgs::msg::Float32 voltage;
            voltage.data = driver->getVoltage();
            this->publish("voltage", voltage);

            // Publish the plugged in status
            std_msgs::msg::Bool plugged_in;
            plugged_in.data = driver->isPluggedIn();
            this->publish("plugged_in", plugged_in);
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
                    double delta_x = delta_distance * cos(a_odo + delta_a/2);
                    double delta_y = delta_distance * sin(a_odo + delta_a/2);
                    x_odo += delta_x;
                    y_odo += delta_y;
                    a_odo += delta_bearing;
                    a_odo = angles::normalize_angle(a_odo);
                }

                last_distance = distance;
                last_bearing = true_bearing - first_bearing;
                geometry_msgs::msg::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(last_bearing);

                geomerty_msgs::msg::TransformStamped odom_trans;
                odom_trans.header.stamp = this->now();
                odom_trans.header.frame_id = this->get_parameter("odometry_frame_id").as_string();
                odom_trans.header.child_frame_id = "base_link";

                odom_trans.transform.translation.x = x_odo;
                odom_trans.transform.translation.y = y_odo;
                odom_trans.transform.rotation = odom_quat;

                broadcaster->sendTransform(odom_trans)

                nav_msgs::msg::Odometry odom;
                odom.header.frame_id = this->get_parameter("odometry_frame_id").as_string();
                odom.pose.pose.position.x = x_odom;
                odom.pose.pose.position.y = y_odom;
                odom.pose.pose.orientation = odom_quat;
                odom.header.stamp = this->now();

                for(int i = 0; i < 6; i++) {
                    odom.pose.covariance[i * 7] = 1;
                }

                odom.child_frame_id = "base_link";
                double tvel = driver.getTransVelocity();
                odom.twist.twist.linear.x = tvel*cos(a_odo);
                odom.twist.twist.linear.y = tvel*sin(a_odo);
                odom.twist.twist.angular.z = driver.getRotVelocity();

                this->publish("odom", odom);

                sensor_msgs::msg::JointState joint_state;
                joint_state.header.stamp = this->now();
                joint_state.name.resize(1);
                joint_state.position.resize(1);
                joint_state.name[0] = "lewis_twist";
                joint_state.position[0] = true_bearing;

                this->publish("joint_states", joint_state);
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
    atvr_jr_node()
    : Node("atvr_jr_node")
    {
        // Declare parameters
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

        // Get parameters

        if(this->initialize_driver(this->get_parameter("port").as_string())) {
            throw std::runtime_error("Failed to initialize driver");
        }

        driver.config.setOdoDistanceConversion(this->get_parameter("odo_distance_conversion").as_int());
        driver.config.setOdoAngleConversion(this->get_parameter("odo_angle_conversion").as_int());
        driver.config.setTransAcc(this->get_parameter("trans_acceleration").as_double());
        driver.config.setTransTorque(this->get_parameter("trans_torque").as_double());
        driver.config.setRotAcc(this->get_parameter("rot_acceleration").as_double());
        driver.config.setRotTorque(this->get_parameter("rot_torque").as_double());



        // Node Subscribers
        this->create_subscriber<geometry_msgs::msg::Twist>("cmd_vel", this->cmd_vel_callback,10);
        this->create_subscriber<std_msgs::msg::Float32>("cmd_accel", this->cmd_accel_callback, 10);
        this->create_subscriber<std_msgs::msg::Bool>("cmd_sonar_power", this->cmd_sonar_power_callback, 10);
        this->create_subscriber<std_msgs::msg::Bool>("cmd_brake_power", this->cmd_brake_power_callback, 10);

        //Node Publishers
        //this->create_publisher<sensor_msgs::msg::PointCloud>("sonar_cloud_base", 50);
        this->create_publisher<std_msgs::msg::Bool>("sonar_power", 1);
        this->create_publisher<std_msgs::msg::Bool>("brake_power", 1);
        this->create_publisher<std_msgs::msg::Float32>("voltage", 1);
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
        this->create_publisher<std_msgs::msg::Bool>("plugged_in", 1);
        this->create_publisher<sensor_msgs::msg::JointState>("state",1);


        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&atvr_jr_node::timer_callback, this));

        driver.systemStatusUpdateSignal.set(boost::bind(&atvr_jr_node::system_status_callback, this));
        driver.motorUpdateSignal.set(boost::bind(&atvr_jr_node::motor_update_callback, this));
        //driver.sonarUpdateSignal.set(boost::bind(&atvr_jr_node::sonar_update_callback, this));

    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<atvr_jr_node>());
    rclcpp::shutdown();
    return 0;
}