#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pronto_msgs/msg/joint_state_with_acceleration.hpp>

// using namespace std::chrono_literals;
// rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
// using A = pronto_msgs::msg::JointStateWithAcceleration;

using std::placeholders::_1;


class JointStateRepublisher : public rclcpp::Node
{
    public : JointStateRepublisher()
        : Node("joint_state_with_acceleration_republisher")
        {
            sub = this->create_subscription<pronto_msgs::msg::JointStateWithAcceleration>(
            "joint_states_with_acceleration", 10, std::bind(& JointStateRepublisher::jointStateWithAccelerationCallback, this, _1));

            pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        }

    private :
        void jointStateWithAccelerationCallback(const pronto_msgs::msg::JointStateWithAcceleration::SharedPtr msg) const
        {
            sensor_msgs::msg::JointState out;
            out.header = msg->header;
            out.name = msg->name;
            out.position = msg->position;
            out.velocity = msg->velocity;
            out.effort = msg->effort;

            pub->publish(out);
        }

        rclcpp::Subscription<pronto_msgs::msg::JointStateWithAcceleration>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateRepublisher>());
    rclcpp::shutdown();
    return 0;
}
