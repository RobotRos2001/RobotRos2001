#include <iostream>
#include <vector>
#include <algorithm>

class TitaPointFootNode
{
public:
    std::vector<float> convertActionsToDesiredPositions_1(const std::vector<float> &actions);
    std::vector<float> convertActionsToDesiredPositions_2(const std::vector<float> &actions);

    // 初始化所需的参数
    std::vector<float> current_obs_ = std::vector<float>(18, 0.0f);                   // 18个关节状态，初始化为0
    std::vector<float> default_joint_angles_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 默认关节角度
    float damping_ = 1.5f;
    float stiffness_ = 80.0f;
    float user_torque_limit_ = 80.0f;
    float action_scale_pos_ = 0.1f;

    // 为了便于测试，直接使用 last_actions_ 的值
    std::vector<float> last_actions_;
    std::vector<float> obs_scales_dofpos_ = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};       // 假设的观测缩放
    std::vector<float> obs_scales_dofvel_ = {0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f}; // 假设的速度缩放
};

std::vector<float> TitaPointFootNode::convertActionsToDesiredPositions_1(const std::vector<float> &actions)
{
    std::vector<float> desired_positions(actions.size());

    // 使用最新的观测数据
    std::vector<float> temp_observation = current_obs_;

    for (size_t i = 0; i < actions.size(); i++)
    {
        float joint_position = temp_observation[6 + i] / obs_scales_dofpos_[i] + default_joint_angles_[i];
        float joint_velocity = temp_observation[12 + i] / obs_scales_dofvel_[i];

        float actionMin = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity - user_torque_limit_) / stiffness_;
        float actionMax = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity + user_torque_limit_) / stiffness_;

        float scaled_action = std::max(actionMin / action_scale_pos_, std::min(actionMax / action_scale_pos_, actions[i]));
        desired_positions[i] = scaled_action * action_scale_pos_ + default_joint_angles_[i];
    }

    // 更新上一次的动作
    last_actions_ = actions;

    return desired_positions;
}

std::vector<float> TitaPointFootNode::convertActionsToDesiredPositions_2(const std::vector<float> &actions)
{
    std::vector<float> desired_positions(actions.size());

    // 使用最新的观测数据
    std::vector<float> temp_observation = current_obs_;

    for (size_t i = 0; i < actions.size(); i++)
    {
        float joint_position = temp_observation[6 + i] / obs_scales_dofpos_[i] + default_joint_angles_[i];
        float joint_velocity = temp_observation[12 + i] / obs_scales_dofvel_[i];

        float actionMin = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity - user_torque_limit_) / stiffness_;
        float actionMax = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity + user_torque_limit_) / stiffness_;

        // float scaled_action = std::clamp(actions[i] / action_scale_pos_, actionMin, actionMax);
        float scaled_action = std::max(actionMin / action_scale_pos_, std::min(actionMax / action_scale_pos_, actions[i]));

        desired_positions[i] = scaled_action * action_scale_pos_ + default_joint_angles_[i];
    }

    // 更新上一次动作
    last_actions_ = actions;

    return desired_positions;
}

int main()
{
    TitaPointFootNode node;
    std::vector<float> actions = {1.0f, -0.5f, 0.9f, 0.2f, 0.7f, 0.4f}; // 测试输入动作

    std::vector<float> desired_positions = node.convertActionsToDesiredPositions_2(actions);

    // 输出结果
    std::cout << "Desired Positions: ";
    for (float pos : desired_positions)
    {
        std::cout << pos << " ";
    }
    std::cout << std::endl;

    return 0;
}

// Desired Positions: 0.1 -0.05 0.09 0.02 0.07 0.04
// Desired Positions: 0.1 -0.1 0.1 0.1 0.1 0.1