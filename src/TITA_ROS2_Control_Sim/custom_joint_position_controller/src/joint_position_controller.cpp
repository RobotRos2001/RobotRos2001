#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;

class JointPositionController : public rclcpp::Node
{
public:
    JointPositionController()
        : Node("joint_position_controller")
    {
        // 声明并获取 PID 参数
        this->declare_parameter("kp", 80.0);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 1.5);
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();

        // 初始化发布器和订阅器
        effort_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita/effort_controller/commands", 10);
        target_position_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tita/actions", 10, std::bind(&JointPositionController::targetPositionCallback, this, std::placeholders::_1));
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/tita/joint_states", 10, std::bind(&JointPositionController::jointStateCallback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            2ms, std::bind(&JointPositionController::controlLoop, this));
    }

private:
    double kp_, kd_, ki_;
    std::vector<double> integral_error_;
    std::vector<double> target_positions_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_position_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void targetPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        target_positions_.assign(msg->data.begin(), msg->data.end());
        if (integral_error_.size() != target_positions_.size())
        {
            integral_error_.resize(target_positions_.size(), 0.0);
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 定义所需的关节排序顺序
        std::vector<std::string> desired_order = {
            "joint_left_leg_1",
            "joint_left_leg_2",
            "joint_left_leg_3",
            "joint_right_leg_1",
            "joint_right_leg_2",
            "joint_right_leg_3"};

        // 创建用于排序后的 position 和 velocity 数据的向量
        std::vector<double> sorted_position(desired_order.size());
        std::vector<double> sorted_velocity(desired_order.size());

        // 遍历所需顺序中的每个关节名，找到在传入消息（msg）中的对应索引
        for (size_t i = 0; i < desired_order.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), desired_order[i]);
            if (it != msg->name.end())
            {
                size_t index = std::distance(msg->name.begin(), it);
                sorted_position[i] = msg->position[index];
                sorted_velocity[i] = msg->velocity[index];
            }
        }

        // 将排序后的关节数据保存到类成员中
        current_positions_ = sorted_position;
        current_velocities_ = sorted_velocity;
    }

    void controlLoop()
    {
        if (target_positions_.empty() || current_positions_.empty())
            return;

        std_msgs::msg::Float64MultiArray effort_msg;
        effort_msg.data.resize(target_positions_.size());

        for (size_t i = 0; i < target_positions_.size(); ++i)
        {
            double position_error = target_positions_[i] - current_positions_[i];
            integral_error_[i] = 0; // 计算积分误差（假设循环频率为 10 ms）
            // integral_error_[i] += position_error * 0.01;       // 计算积分误差（假设循环频率为 10 ms）
            double derivative_error = -current_velocities_[i]; // 导数误差基于关节速度

            double effort = kp_ * position_error + ki_ * integral_error_[i] + kd_ * derivative_error;
            effort_msg.data[i] = effort;
        }


        // 发布力矩到控制器
        effort_publisher_->publish(effort_msg);
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
