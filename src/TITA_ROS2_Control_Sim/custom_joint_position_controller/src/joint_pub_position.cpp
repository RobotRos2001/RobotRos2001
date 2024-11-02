#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <control_msgs/msg/multi_dof_command.hpp>
#include <geometry_msgs/msg/transform.hpp>  // 添加 Transform 的导入

using namespace std::chrono_literals;

class JointPositionController : public rclcpp::Node
{
public:
    JointPositionController()
        : Node("joint_pub_position")
    {
        // 初始化发布器和订阅器
        position_publisher_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>("/tita/pid_controller/reference", 10);
        target_position_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tita/actions", 10, std::bind(&JointPositionController::targetPositionCallback, this, std::placeholders::_1));
        control_timer_ = this->create_wall_timer(
            2ms, std::bind(&JointPositionController::controlLoop, this));
    }

private:
    std::vector<double> target_positions_;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr position_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        target_positions_.assign(msg->data.begin(), msg->data.end());
    }

    void controlLoop()
    {
        // if (target_positions_.empty())
        //     return;

        // 创建并初始化 MultiDOFCommand 消息
        auto command_msg = control_msgs::msg::MultiDOFCommand();

        // 指定关节名称
        const std::vector<std::string> joint_names = {
            "joint_left_leg_1",
            "joint_left_leg_2",
            "joint_left_leg_3",
            "joint_right_leg_1",
            "joint_right_leg_2",
            "joint_right_leg_3"
        };

        // 设置 degrees of freedom (DoF) 的名称
        command_msg.dof_names = joint_names;

        // 设置 values 为目标位置
        command_msg.values = target_positions_;

        // 设置 values_dot 为 0
        command_msg.values_dot.resize(joint_names.size(), 0.0); // 将速度设为0

        // 发布消息
        position_publisher_->publish(command_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPositionController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
