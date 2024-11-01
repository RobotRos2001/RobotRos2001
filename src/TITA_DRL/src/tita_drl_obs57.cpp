#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <deque>
#include <mutex>

using std::placeholders::_1;

// 定义 TitaPointFootNode 类
class TitaPointFootNode : public rclcpp::Node
{
public:
    TitaPointFootNode()
        : Node("tita_pointfoot_node"),
          env_(ORT_LOGGING_LEVEL_WARNING, "TITAPointfootONNX")
    {
        // 初始化订阅者
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/tita/imu_sensor_broadcaster/imu", 10,
            std::bind(&TitaPointFootNode::imuCallback, this, _1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/tita/joint_states", 10,
            std::bind(&TitaPointFootNode::jointStateCallback, this, _1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TitaPointFootNode::cmdVelCallback, this, _1));

        // 初始化发布者
        action_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/tita_pointfoot/actions", 10);

        // 创建定时器，控制循环以50Hz频率运行
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&TitaPointFootNode::controlLoop, this));

        // 加载参数
        if (!loadParameters())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters.");
            rclcpp::shutdown();
        }

        // 加载 ONNX 模型
        RCLCPP_INFO(this->get_logger(), "Loading ONNX model from: %s", model_path.c_str());
        if (!loadONNXModel(model_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model.");
            rclcpp::shutdown();
        }

        // 初始化历史数据结构
        history_dof_pos_.resize(6);
        history_dof_vel_.resize(6);

        // 初始化默认关节角度
        default_joint_angles_.resize(6);

        // 初始化观测向量
        observation_.resize(observations_size_, 0.0f);
        current_obs_.resize(observations_size_, 0.0f);

        // 初始化上一次动作
        last_actions_.resize(actions_size_, 0.0f);

        // 初始化当前命令
        current_commands_.resize(3, 0.0f);

        // 使用默认值初始化历史队列
        for (size_t i = 0; i < 6; ++i)
        {
            history_dof_pos_[i] = std::deque<float>(history_length_dof_pos_, default_joint_angles_[i]);
            history_dof_vel_[i] = std::deque<float>(history_length_dof_vel_, 0.0f);
        }

        RCLCPP_INFO(this->get_logger(), "TitaPointFootNode initialized successfully.");
    }

    ~TitaPointFootNode()
    {
        if (policy_session_)
        {
            delete policy_session_;
            policy_session_ = nullptr;
        }
    }

private:
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 最新的消息存储
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_;
    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_msg_;

    // 互斥锁
    std::mutex imu_mutex_;
    std::mutex joint_state_mutex_;

    // ONNX模型相关
    Ort::Env env_;
    Ort::Session *policy_session_ = nullptr;
    Ort::SessionOptions session_options_;
    std::vector<const char *> input_names_;
    std::vector<const char *> output_names_;
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;

    // 参数
    std::vector<std::string> joint_names_;
    std::vector<float> default_joint_angles_;
    float obs_scales_angvel_, obs_scales_dofpos_, obs_scales_dofvel_;
    float scaled_commands_x_, scaled_commands_y_, scaled_commands_z_;
    float stiffness_, damping_, action_scale_pos_, user_torque_limit_, decimation_;
    int actions_size_, observations_size_, commands_size_;
    float clip_observations_, clip_actions_;
    std::string model_path;

    int history_length_dof_pos_;
    int history_length_dof_vel_;

    // 数据
    std::vector<float> observation_;
    std::vector<float> last_actions_;
    std::vector<std::deque<float>> history_dof_pos_;
    std::vector<std::deque<float>> history_dof_vel_;
    std::vector<float> current_obs_;

    // 当前速度命令
    std::vector<float> current_commands_;

    // 标志位，指示是否已接收到初始观测
    bool has_received_observation_ = false;

    // 成员函数

    // IMU消息回调
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_msg_ = msg;
    }

    // JointState 消息回调
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        // 定义所需的关节排序顺序
        std::vector<std::string> desired_order = {
            "joint_left_leg_1",
            "joint_left_leg_2",
            "joint_left_leg_3",
            "joint_right_leg_1",
            "joint_right_leg_2",
            "joint_right_leg_3"};

        // 创建用于排序后的 position、velocity 和 effort 数据的向量
        std::vector<double> sorted_position(desired_order.size());
        std::vector<double> sorted_velocity(desired_order.size());
        std::vector<double> sorted_effort(desired_order.size());

        // 遍历所需顺序中的每个关节名，找到在传入消息（msg）中的对应索引
        for (size_t i = 0; i < desired_order.size(); ++i)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), desired_order[i]);
            if (it != msg->name.end())
            {
                size_t index = std::distance(msg->name.begin(), it);
                sorted_position[i] = msg->position[index];
                sorted_velocity[i] = msg->velocity[index];
                sorted_effort[i] = msg->effort[index];
            }
        }

        // 创建一个新的 JointState 消息，使用排序后的数据
        auto sorted_msg = std::make_shared<sensor_msgs::msg::JointState>();
        sorted_msg->header = msg->header;
        sorted_msg->name = desired_order;
        sorted_msg->position = sorted_position;
        sorted_msg->velocity = sorted_velocity;
        sorted_msg->effort = sorted_effort;

        // 将排序后的消息赋值给 latest_joint_state_msg_
        latest_joint_state_msg_ = sorted_msg;
        // 打印 latest_joint_state_msg_ 的数据
        // RCLCPP_INFO(rclcpp::get_logger("joint_state_callback"), "Joint States:");
        // for (size_t i = 0; i < latest_joint_state_msg_->name.size(); ++i)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("joint_state_callback"),
        //                 "Joint: %s, Position: %.6f, Velocity: %.6f, Effort: %.6f",
        //                 latest_joint_state_msg_->name[i].c_str(),
        //                 latest_joint_state_msg_->position[i],
        //                 latest_joint_state_msg_->velocity[i],
        //                 latest_joint_state_msg_->effort[i]);
        // }
    }

    // cmd_vel消息回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Use the user_cmd_scales to scale the velocity commands
        current_commands_[0] = static_cast<float>(msg->linear.x) * scaled_commands_x_;
        current_commands_[1] = static_cast<float>(msg->linear.y) * scaled_commands_y_;
        current_commands_[2] = static_cast<float>(msg->angular.z) * scaled_commands_z_;
    }

    // 加载参数
    bool loadParameters()
    {
        RCLCPP_INFO(this->get_logger(), "Loading parameters directly...");
        // 参数声明
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_left_leg_1", -0.27);
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_left_leg_2", 0.28);
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_left_leg_3", -0.73);
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_right_leg_1", 0.27);
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_right_leg_2", 0.28);
        this->declare_parameter<float>("TITAPointfootCfg.default_joint_angle.joint_right_leg_3", -0.73);

        this->declare_parameter<int>("TITAPointfootCfg.observation.history_length_dof_pos", 3);
        this->declare_parameter<int>("TITAPointfootCfg.observation.history_length_dof_vel", 2);

        this->declare_parameter<float>("TITAPointfootCfg.control.stiffness", 80.0);
        this->declare_parameter<float>("TITAPointfootCfg.control.damping", 1.5);
        this->declare_parameter<float>("TITAPointfootCfg.control.action_scale_pos", 0.1);
        this->declare_parameter<float>("TITAPointfootCfg.control.decimation", 4.0);
        this->declare_parameter<float>("TITAPointfootCfg.control.user_torque_limit", 80.0);

        this->declare_parameter<float>("TITAPointfootCfg.normalization.clip_scales.clip_observations", 100.0);
        this->declare_parameter<float>("TITAPointfootCfg.normalization.clip_scales.clip_actions", 100.0);
        this->declare_parameter<float>("TITAPointfootCfg.normalization.obs_scales.ang_vel", 0.25);
        this->declare_parameter<float>("TITAPointfootCfg.normalization.obs_scales.dof_pos", 1.0);
        this->declare_parameter<float>("TITAPointfootCfg.normalization.obs_scales.dof_vel", 0.05);

        this->declare_parameter<int>("TITAPointfootCfg.size.actions_size", 6);
        this->declare_parameter<int>("TITAPointfootCfg.size.observations_size", 57);
        this->declare_parameter<int>("TITAPointfootCfg.size.commands_size", 3);

        this->declare_parameter<float>("TITAPointfootCfg.user_cmd_scales.lin_vel_x", 1.5);
        this->declare_parameter<float>("TITAPointfootCfg.user_cmd_scales.lin_vel_y", 1.0);
        this->declare_parameter<float>("TITAPointfootCfg.user_cmd_scales.ang_vel_yaw", 0.5);

        this->declare_parameter<std::string>("TITAPointfootCfg.model_path", "/home/server/WS_ROS_humble/tita_ws/src/TITA_DRL/config/policy/policy_57.onnx");

        // 获取参数
        default_joint_angles_.resize(6);
        // 默认角度
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_left_leg_1", default_joint_angles_[0]);
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_left_leg_2", default_joint_angles_[1]);
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_left_leg_3", default_joint_angles_[2]);
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_right_leg_1", default_joint_angles_[3]);
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_right_leg_2", default_joint_angles_[4]);
        this->get_parameter("TITAPointfootCfg.default_joint_angle.joint_right_leg_3", default_joint_angles_[5]);
        // 历史数据长度
        this->get_parameter("TITAPointfootCfg.observation.history_length_dof_pos", history_length_dof_pos_);
        this->get_parameter("TITAPointfootCfg.observation.history_length_dof_vel", history_length_dof_vel_);
        // control参数
        this->get_parameter("TITAPointfootCfg.control.stiffness", stiffness_);
        this->get_parameter("TITAPointfootCfg.control.damping", damping_);
        this->get_parameter("TITAPointfootCfg.control.action_scale_pos", action_scale_pos_);
        this->get_parameter("TITAPointfootCfg.control.decimation", decimation_);
        this->get_parameter("TITAPointfootCfg.control.user_torque_limit", user_torque_limit_);
        // 剪切和归一化参数
        this->get_parameter("TITAPointfootCfg.normalization.clip_scales.clip_observations", clip_observations_);
        this->get_parameter("TITAPointfootCfg.normalization.clip_scales.clip_actions", clip_actions_);
        this->get_parameter("TITAPointfootCfg.normalization.obs_scales.ang_vel", obs_scales_angvel_);
        this->get_parameter("TITAPointfootCfg.normalization.obs_scales.dof_pos", obs_scales_dofpos_);
        this->get_parameter("TITAPointfootCfg.normalization.obs_scales.dof_vel", obs_scales_dofvel_);
        // 网络结构参数
        this->get_parameter("TITAPointfootCfg.size.actions_size", actions_size_);
        this->get_parameter("TITAPointfootCfg.size.observations_size", observations_size_);
        this->get_parameter("TITAPointfootCfg.size.commands_size", commands_size_);
        // 命令缩放参数
        this->get_parameter("TITAPointfootCfg.user_cmd_scales.lin_vel_x", scaled_commands_x_);
        this->get_parameter("TITAPointfootCfg.user_cmd_scales.lin_vel_y", scaled_commands_y_);
        this->get_parameter("TITAPointfootCfg.user_cmd_scales.ang_vel_yaw", scaled_commands_z_);
        // onnx文件路径
        this->get_parameter("TITAPointfootCfg.model_path", model_path);

        RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully.");
        return true;
    }

    // 加载 ONNX 模型
    bool loadONNXModel(const std::string &model_path)
    {
        session_options_.SetIntraOpNumThreads(1);

        try
        {
            policy_session_ = new Ort::Session(env_, model_path.c_str(), session_options_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
            return false;
        }

        // 获取输入和输出的名称及形状
        Ort::AllocatorWithDefaultOptions allocator;
        input_names_.push_back(policy_session_->GetInputName(0, allocator));
        output_names_.push_back(policy_session_->GetOutputName(0, allocator));

        input_shape_ = policy_session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        output_shape_ = policy_session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

        // 调试信息
        std::stringstream ss;
        ss << "Model input shape:";
        for (auto dim : input_shape_)
        {
            ss << " " << dim;
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        ss.str("");
        ss << "Model output shape:";
        for (auto dim : output_shape_)
        {
            ss << " " << dim;
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

        RCLCPP_INFO(this->get_logger(), "ONNX model loaded successfully: %s", model_path.c_str());
        return true;
    }

    // 控制循环
    void controlLoop()
    {
        // 获取最新的 IMU 和 JointState 消息
        sensor_msgs::msg::Imu::SharedPtr imu_msg;
        sensor_msgs::msg::JointState::SharedPtr joint_msg;

        {
            std::lock_guard<std::mutex> imu_lock(imu_mutex_);
            imu_msg = latest_imu_msg_;
        }

        {
            std::lock_guard<std::mutex> joint_lock(joint_state_mutex_);
            joint_msg = latest_joint_state_msg_;
        }

        // 检查是否接收到所有必要的消息
        if (!imu_msg || !joint_msg)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for both IMU and JointState messages.");
            return;
        }

        // 准备临时变量
        std::vector<float> temp_observation(observations_size_, 0.0f);
        std::vector<float> temp_last_actions = last_actions_;
        std::vector<float> temp_current_commands = current_commands_;
        std::vector<std::deque<float>> temp_history_dof_pos = history_dof_pos_;
        std::vector<std::deque<float>> temp_history_dof_vel = history_dof_vel_;

        // 将四元数转换为 ZYX 欧拉角并计算重力投影
        Eigen::Quaterniond q(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
        Eigen::Matrix3d rot_matrix = q.toRotationMatrix();
        Eigen::Vector3d gravity_vector(0, 0, -1);
        Eigen::Vector3d projected_gravity = rot_matrix.transpose() * gravity_vector;

        // 更新观测 - 角速度 (索引 0-2)
        temp_observation[0] = static_cast<float>(imu_msg->angular_velocity.x) * obs_scales_angvel_;
        temp_observation[1] = static_cast<float>(imu_msg->angular_velocity.y) * obs_scales_angvel_;
        temp_observation[2] = static_cast<float>(imu_msg->angular_velocity.z) * obs_scales_angvel_;

        // 更新观测 - 重力投影向量 (索引 3-5)
        temp_observation[3] = static_cast<float>(projected_gravity.x());
        temp_observation[4] = static_cast<float>(projected_gravity.y());
        temp_observation[5] = static_cast<float>(projected_gravity.z());

        // 更新关节状态
        for (size_t i = 0; i < 6; ++i)
        {
            if (i >= joint_msg->position.size() || i >= joint_msg->velocity.size())
            {
                RCLCPP_WARN(this->get_logger(), "JointState message does not have enough data.");
                continue;
            }

            float pos_offset = static_cast<float>(joint_msg->position[i]);
            float vel_scaled = static_cast<float>(joint_msg->velocity[i]);

            // 更新关节位置历史 (未缩放，用于历史数据)
            temp_history_dof_pos[i].push_front(pos_offset);
            if (temp_history_dof_pos[i].size() > static_cast<size_t>(history_length_dof_pos_))
                temp_history_dof_pos[i].pop_back();

            // 更新关节速度历史 (未缩放，用于历史数据)
            temp_history_dof_vel[i].push_front(vel_scaled);
            if (temp_history_dof_vel[i].size() > static_cast<size_t>(history_length_dof_vel_))
                temp_history_dof_vel[i].pop_back();

            // 缩放当前关节位置偏移并更新观测 (索引 6-11)
            temp_observation[6 + i] = (pos_offset - default_joint_angles_[i]) * obs_scales_dofpos_;

            // 缩放当前关节速度并更新观测 (索引 12-17)
            temp_observation[12 + i] = vel_scaled * obs_scales_dofvel_;
        }

        // 更新观测中的上一次动作 (索引 18-23)
        for (size_t i = 0; i < temp_last_actions.size(); ++i)
        {
            if (18 + static_cast<int>(i) >= observations_size_)
                break;
            temp_observation[18 + i] = temp_last_actions[i];
        }

        // 更新观测中的速度命令 (索引 24-26)
        if (observations_size_ >= 27)
        {
            temp_observation[24] = temp_current_commands[0];
            temp_observation[25] = temp_current_commands[1];
            temp_observation[26] = temp_current_commands[2];
        }

        // 缩放并添加关节位置历史 (索引 27-44)
        int idx = 27;
        for (size_t i = 0; i < 6; ++i) // 遍历每个关节
        {
            for (int j = 0; j < history_length_dof_pos_; ++j) // 从最近到最旧的历史
            {
                if (idx >= observations_size_)
                    break;
                float pos_offset = (temp_history_dof_pos[i][j] - default_joint_angles_[i]) * obs_scales_dofpos_;
                temp_observation[idx++] = pos_offset;
            }
        }

        // 缩放并添加关节速度历史 (索引 45-56)
        for (size_t i = 0; i < 6; ++i) // 遍历每个关节
        {
            for (int j = 0; j < history_length_dof_vel_; ++j) // 从最近到最旧的历史
            {
                if (idx >= observations_size_)
                    break;
                float vel_scaled = temp_history_dof_vel[i][j] * obs_scales_dofvel_;
                temp_observation[idx++] = vel_scaled;
            }
        }

        // 更新当前观测向量
        current_obs_ = temp_observation;

        // 更新历史数据
        history_dof_pos_ = temp_history_dof_pos;
        history_dof_vel_ = temp_history_dof_vel;

        // 设置标志位，指示已接收到初始观测
        has_received_observation_ = true;

        // 剪裁观测值
        std::transform(current_obs_.begin(), current_obs_.end(), current_obs_.begin(),
                       [this](float x) -> float
                       { return std::clamp(x, -clip_observations_, clip_observations_); });

        // 运行推理
        std::vector<float> actions = runInference(current_obs_);

        // 剪裁动作
        std::transform(actions.begin(), actions.end(), actions.begin(),
                       [this](float x) -> float
                       { return std::clamp(x, -clip_actions_, clip_actions_); });

        // 将动作转换为期望的位置
        std::vector<float> desired_positions = convertActionsToDesiredPositions(actions);

        // 发布期望的位置
        publishDesiredPositions(desired_positions);
    }

    // 运行推理
    std::vector<float> runInference(const std::vector<float> &input_obs)
    {
        // 检查输入大小
        if (input_obs.size() != static_cast<size_t>(input_shape_[1]))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Input observation size (%lu) does not match model input size (%ld).",
                         input_obs.size(), input_shape_[1]);
            return std::vector<float>(actions_size_, 0.0f);
        }

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        std::vector<Ort::Value> input_tensors;
        std::vector<float> output_values(output_shape_[1]);

        // 创建输入张量
        input_tensors.emplace_back(
            Ort::Value::CreateTensor<float>(memory_info, const_cast<float *>(input_obs.data()),
                                            input_obs.size(), input_shape_.data(), input_shape_.size()));

        // 运行推理
        try
        {
            auto output_tensors = policy_session_->Run(Ort::RunOptions{nullptr}, input_names_.data(),
                                                       input_tensors.data(), 1,
                                                       output_names_.data(), 1);

            // 获取输出结果
            float *output_data = output_tensors[0].GetTensorMutableData<float>();
            for (int64_t i = 0; i < output_shape_[1]; ++i)
            {
                output_values[i] = output_data[i];
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to run inference: %s", e.what());
            return std::vector<float>(actions_size_, 0.0f);
        }

        return output_values;
    }

    // 将动作转换为期望的位置
    std::vector<float> convertActionsToDesiredPositions(const std::vector<float> &actions)
    {
        std::vector<float> desired_positions(actions.size());

        // 使用最新的观测数据
        std::vector<float> temp_observation = current_obs_;

        for (size_t i = 0; i < actions.size(); i++)
        {
            if (6 + i >= temp_observation.size() || 12 + i >= temp_observation.size())
                continue;

            float joint_position = temp_observation[6 + i] / obs_scales_dofpos_ + default_joint_angles_[i];
            float joint_velocity = temp_observation[12 + i] / obs_scales_dofvel_;

            float actionMin = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity - user_torque_limit_) / stiffness_;
            float actionMax = joint_position - default_joint_angles_[i] + (damping_ * joint_velocity + user_torque_limit_) / stiffness_;

            float scaled_action = std::clamp(actions[i] / action_scale_pos_, actionMin, actionMax);
            desired_positions[i] = scaled_action * action_scale_pos_ + default_joint_angles_[i];
        }

        // 更新上一次动作
        last_actions_ = actions;

        return desired_positions;
    }

    // 发布期望的位置
    void publishDesiredPositions(const std::vector<float> &desired_positions)
    {
        auto action_msg = std_msgs::msg::Float32MultiArray();
        action_msg.data = desired_positions;
        action_pub_->publish(action_msg);
    }
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TitaPointFootNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}