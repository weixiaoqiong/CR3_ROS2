/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
//#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>
//#include <ros/ros.h>
//#include <ros/param.h>
#include "dobot_bringup/cr5_robot.h"
#include <sensor_msgs/msg/joint_state.hpp>

//using namespace std::chrono_literals;
//导入头文件，就像xacro中导入xacro macro一样？头文件中申明了一个类，这里对类的成员函数作了定义

using namespace std::placeholders;
using CR5FollowJoint = control_msgs::action::FollowJointTrajectory;
using GoalHandleCR5FollowJoint = rclcpp_action::ServerGoalHandle<CR5FollowJoint>;
using namespace std::chrono_literals;

CR5Robot::CR5Robot(const rclcpp::NodeOptions & options)
: Node("cr5_bringup_node", options)
, goal_{}
, trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));

    std::string ip;
    this->declare_parameter<std::string>("robot_ip_address", "192.168.1.6");
    this->get_parameter("robot_ip_address", ip);
    this->declare_parameter("trajectory_duration", 0.3);
    this->get_parameter("trajectory_duration",trajectory_duration_);
    RCLCPP_INFO(this->get_logger(), "trajectory_duration : %0.2lf", trajectory_duration_);
    RCLCPP_INFO(this->get_logger(), ip);

    /*
    std::string ip = this.param<std::string>("robot_ip_address", "192.168.5.1");
    trajectory_duration_ = this.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);
    */

    commander_ = std::make_shared<CR5Commander>(ip, this);
    std::cout << "Hello";
    commander_->init();
    
    RCLCPP_INFO(this->get_logger(), "commander");

    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::EnableRobot>("/dobot_bringup/srv/EnableRobot", std::bind(&CR5Robot::enableRobot, this, _1, _2)));
    server_tbl_.push_back(
        this->create_service<dobot_bringup_srv::srv::DisableRobot>("/dobot_bringup/srv/DisableRobot", std::bind(&CR5Robot::disableRobot, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ClearError>("/dobot_bringup/srv/ClearError", std::bind(&CR5Robot::clearError, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ResetRobot>("/dobot_bringup/srv/ResetRobot", std::bind(&CR5Robot::resetRobot, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SpeedFactor>("/dobot_bringup/srv/SpeedFactor", std::bind(&CR5Robot::speedFactor, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::User>("/dobot_bringup/srv/User", std::bind(&CR5Robot::user, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Tool>("/dobot_bringup/srv/Tool", std::bind(&CR5Robot::tool, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::RobotMode>("/dobot_bringup/srv/RobotMode", std::bind(&CR5Robot::robotMode, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::PayLoad>("/dobot_bringup/srv/PayLoad", std::bind(&CR5Robot::payload, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::DO>("/dobot_bringup/srv/DO", std::bind(&CR5Robot::DO, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::DOExecute>("/dobot_bringup/srv/DOExecute", std::bind(&CR5Robot::DOExecute, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ToolDO>("/dobot_bringup/srv/ToolDO", std::bind(&CR5Robot::toolDO, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ToolDOExecute>("/dobot_bringup/srv/ToolDOExecute", std::bind(&CR5Robot::toolDOExecute, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::AO>("/dobot_bringup/srv/AO", std::bind(&CR5Robot::AO, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::AOExecute>("/dobot_bringup/srv/AOExecute", std::bind(&CR5Robot::AOExecute, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::AccJ>("/dobot_bringup/srv/AccJ", std::bind(&CR5Robot::accJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::AccL>("/dobot_bringup/srv/AccL", std::bind(&CR5Robot::accL, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SpeedJ>("/dobot_bringup/srv/SpeedJ", std::bind(&CR5Robot::speedJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SpeedL>("/dobot_bringup/srv/SpeedL", std::bind(&CR5Robot::speedL, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Arch>("/dobot_bringup/srv/Arch", std::bind(&CR5Robot::arch, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::CP>("/dobot_bringup/srv/CP", std::bind(&CR5Robot::cp, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::LimZ>("/dobot_bringup/srv/LimZ", std::bind(&CR5Robot::limZ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SetArmOrientation>("/dobot_bringup/srv/SetArmOrientation", std::bind(&CR5Robot::setArmOrientation, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::PowerOn>("/dobot_bringup/srv/PowerOn", std::bind(&CR5Robot::powerOn, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::RunScript>("/dobot_bringup/srv/RunScript", std::bind(&CR5Robot::runScript, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::StopScript>("/dobot_bringup/srv/StopScript", std::bind(&CR5Robot::stopScript, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::PauseScript>("/dobot_bringup/srv/PauseScript", std::bind(&CR5Robot::pauseScript, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ContinueScript>("/dobot_bringup/srv/ContinueScript", std::bind(&CR5Robot::continueScript, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SetSafeSkin>("/dobot_bringup/srv/SetSafeSkin", std::bind(&CR5Robot::setSafeSkin, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SetObstacleAvoid>("/dobot_bringup/srv/SetObstacleAvoid", std::bind(&CR5Robot::setObstacleAvoid, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::SetCollisionLevel>("/dobot_bringup/srv/SetCollisionLevel", std::bind(&CR5Robot::setCollisionLevel, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::EmergencyStop>("/dobot_bringup/srv/EmergencyStop", std::bind(&CR5Robot::emergencyStop, this, _1, _2)));

    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::MovJ>("/dobot_bringup/srv/MovJ", std::bind(&CR5Robot::movJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::MovL>("/dobot_bringup/srv/MovL", std::bind(&CR5Robot::movL, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::JointMovJ>("/dobot_bringup/srv/JointMovJ", std::bind(&CR5Robot::jointMovJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Jump>("/dobot_bringup/srv/Jump", std::bind(&CR5Robot::jump, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::RelMovJ>("/dobot_bringup/srv/RelMovJ", std::bind(&CR5Robot::relMovJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::RelMovL>("/dobot_bringup/srv/RelMovL", std::bind(&CR5Robot::relMovL, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Arc>("/dobot_bringup/srv/Arc", std::bind(&CR5Robot::arc, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Circle>("/dobot_bringup/srv/Circle", std::bind(&CR5Robot::circle, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ServoJ>("/dobot_bringup/srv/ServoJ", std::bind(&CR5Robot::servoJ, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::ServoP>("/dobot_bringup/srv/ServoP", std::bind(&CR5Robot::servoP, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::Sync>("/dobot_bringup/srv/Sync", std::bind(&CR5Robot::sync, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::StartTrace>("/dobot_bringup/srv/StartTrace", std::bind(&CR5Robot::startTrace, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::StartPath>("/dobot_bringup/srv/StartPath", std::bind(&CR5Robot::startPath, this, _1, _2)));
    server_tbl_.push_back(
        this->create_service<dobot_bringup_srv::srv::StartFCTrace>("/dobot_bringup/srv/StartFCTrace", std::bind(&CR5Robot::startFCTrace, this, _1, _2)));
    server_tbl_.push_back(this->create_service<dobot_bringup_srv::srv::MoveJog>("/dobot_bringup/srv/MoveJog", std::bind(&CR5Robot::moveJog, this, _1, _2)));    
    
    RCLCPP_INFO(this->get_logger(), "Service");

    this->action_server_ = rclcpp_action::create_server<CR5FollowJoint>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "CR5Robot",
        std::bind(&CR5Robot::goalHandle, this, _1, _2),
        std::bind(&CR5Robot::cancelHandle, this, _1),
        std::bind(&CR5Robot::acceptHandle, this, _1)
    );     
    RCLCPP_INFO(this->get_logger(), "Action");

    pub_timer = this->create_wall_timer(500ms, std::bind(&CR5Robot::pub_timer_callback, this));
    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 100);  
    robot_status_pub = this->create_publisher<dobot_bringup_msg::msg::RobotStatus>("/dobot_bringup/msg/RobotStatus", 100);  
    tool_vector_pub = this->create_publisher<dobot_bringup_msg::msg::ToolVectorActual>("/dobot_bringup/msg/ToolVectorActual", 100);  


    for (uint32_t i = 0; i < 6; i++)
    {
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
    }       
    
    // this->declare_parameter("JointStatePublishRate", 10.0);
    // double rate_vale;
    // this->get_parameter("JointStatePublishRate", rate_vale);      
}

//头文件cr5_this->h定义了类CR5Robot,这里是在定义其构造函数CR5Robot
/*
CR5Robot::CR5Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , this(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}
*/

CR5Robot::~CR5Robot()
{
    RCLCPP_INFO(this->get_logger(), "~CR5Robot");
    //ROS_INFO("~CR5Robot");//析构函数不承担任何重要的工作，可以将它编写为不执行任何操作的函数
    //这里写个“byebye～”也是一样的，只是能让人看出何时析构函数被调用了
}

void CR5Robot::pub_timer_callback()
{
    // rclcpp::WallRate loop_rate(rate_vale);
    // double position[6];
    // while (rclcpp::ok())
    // {
        //
        // publish joint state
        //
    this->getJointState(position);
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.header.frame_id = "dummy_link";
    for (uint32_t i = 0; i < 6; i++)
        joint_state_msg.position[i] = position[i];
    joint_state_pub->publish(joint_state_msg);

    double val[6];
    this->getToolVectorActual(val);
    tool_vector_actual_msg.x = val[0];
    tool_vector_actual_msg.y = val[1];
    tool_vector_actual_msg.z = val[2];
    tool_vector_actual_msg.rx = val[3];
    tool_vector_actual_msg.ry = val[4];
    tool_vector_actual_msg.rz = val[5];
    tool_vector_pub->publish(tool_vector_actual_msg);

    //
    // publish robot status
    //
    robot_status_msg.is_enable = this->isEnable();
    robot_status_msg.is_connected = this->isConnected();
    robot_status_pub->publish(robot_status_msg);     
}

/*
void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)*/
void CR5Robot::feedbackHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle)                              
{
    control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr feedback;
    //std::make_shared<control_msgs::action::FollowJointTrajectory> feedback;
    //control_msgs::action::FollowJointTrajectory_Feedback feedback; 
    //control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++)
    {
        feedback->joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback->actual.positions.push_back(current_joints[i]);
        feedback->desired.positions.push_back(goal_[i]);
    }

    handle->publish_feedback(feedback);
    //handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle)
{
    double dur = trajectory_duration_ + 1.0;
    rclcpp::Rate loop_rate(dur);
    const auto trajectory = handle->get_goal();
    auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    //control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();
    for(; (index_<trajectory->trajectory.points.size()) && rclcpp::ok(); ++index_)
    {
        //Check if there is a cancel request
        if(handle->is_canceling()){
            handle->canceled(result);
            RCLCPP_INFO(this->get_logger(),"Goal canceled");
            return;
        }
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++)
        {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        commander_->servoJ(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
        index_++;
        CR5Robot::feedbackHandle(handle);
        loop_rate.sleep();
    }
#define OFFSET_VAL 0.01
    //check if goal is done
    double current_joints[6];
    getJointState(current_joints);
    if (rclcpp::ok() &&
        (current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
        (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
        (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
        (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
        (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
        (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL))
    {
        
        /*
        timer_.stop();
        movj_timer_.stop();
        */
        //timer_->cancel();
        //movj_timer_->cancel();
        
        //CR5Robot::feedbackHandle(handle);

        //handle.setSucceeded();
        handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void CR5Robot::acceptHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++)
    {
        goal_[i] = handle->get_goal()->trajectory.points[handle->get_goal()->trajectory.points.size() - 1].positions[i];
        //goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    std::thread{std::bind(&CR5Robot::moveHandle, this, _1), handle}.detach();
}

rclcpp_action::GoalResponse CR5Robot::goalHandle(const rclcpp_action::GoalUUID & uuid,
                                                std::shared_ptr<const CR5FollowJoint::Goal> goal)
{   
    //timer_ = this->create_wall_timer(1000ms, CR5Robot::feedbackHandle);//???
    //movj_timer_ = this->create_wall_timer(trajectory_duration_, CR5Robot::moveHandle);
    //timer_ = this.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    //movj_timer_ = this.createTimer(ros::Duration(trajectory_duration_),
    //
    //                                      boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    /*timer_.start();
    movj_timer_.start();
    handle.setAccepted();?????*/

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CR5Robot::cancelHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle)
{
    /*timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();*/
    //timer_->cancel();
    //movj_timer_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CR5Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CR5Robot::isEnable() const
{
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const
{
    return commander_->isConnected();
}

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::enableRobot(const dobot_bringup_srv::srv::EnableRobot::Request::SharedPtr request, dobot_bringup_srv::srv::EnableRobot::Response::SharedPtr response)
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "wait_for_service");
        commander_->enableRobot();
        RCLCPP_INFO(this->get_logger(), "Enable");
        response->res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response->res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(const dobot_bringup_srv::srv::DisableRobot::Request::SharedPtr request,
                            dobot_bringup_srv::srv::DisableRobot::Response::SharedPtr response)
{
    try
    {
        commander_->disableRobot();
        response->res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response->res = -1;
        return false;
    }
}

bool CR5Robot::clearError(const dobot_bringup_srv::srv::ClearError::Request::SharedPtr request, dobot_bringup_srv::srv::ClearError::Response::SharedPtr response)
{
    try
    {
        commander_->clearError();
        response->res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        commander_->clearError();
        response->res = -1;
        return false;
    }
}

bool CR5Robot::resetRobot(const dobot_bringup_srv::srv::ResetRobot::Request::SharedPtr request, dobot_bringup_srv::srv::ResetRobot::Response::SharedPtr response)
{
    try
    {
        commander_->resetRobot();
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedFactor(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request->ratio);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::user(const dobot_bringup_srv::srv::User::Request::SharedPtr request, dobot_bringup_srv::srv::User::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request->index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tool(const dobot_bringup_srv::srv::Tool::Request::SharedPtr request, dobot_bringup_srv::srv::Tool::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request->index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::robotMode(const dobot_bringup_srv::srv::RobotMode::Request::SharedPtr request, dobot_bringup_srv::srv::RobotMode::Response::SharedPtr response)
{
    try
    {
        const char *cmd = "RobotMode()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::payload(const dobot_bringup_srv::srv::PayLoad::Request::SharedPtr request, dobot_bringup_srv::srv::PayLoad::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request->weight, request->inertia);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::DO(const dobot_bringup_srv::srv::DO::Request::SharedPtr request, dobot_bringup_srv::srv::DO::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request->index, request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::DOExecute(const dobot_bringup_srv::srv::DOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::DOExecute::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request->index, request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDO(const dobot_bringup_srv::srv::ToolDO::Request::SharedPtr request, dobot_bringup_srv::srv::ToolDO::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d, %d)", request->index, request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDOExecute(const dobot_bringup_srv::srv::ToolDOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::ToolDOExecute::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d, %d)", request->index, request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::AO(const dobot_bringup_srv::srv::AO::Request::SharedPtr request, dobot_bringup_srv::srv::AO::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", request->index, request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::AOExecute(const dobot_bringup_srv::srv::AOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::AOExecute::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %0.3f)", request->index, static_cast<float>(request->value));
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accJ(const dobot_bringup_srv::srv::AccJ::Request::SharedPtr request, dobot_bringup_srv::srv::AccJ::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request->r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accL(const dobot_bringup_srv::srv::AccL::Request::SharedPtr request, dobot_bringup_srv::srv::AccL::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request->r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedJ(const dobot_bringup_srv::srv::SpeedJ::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedJ::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request->r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedL(const dobot_bringup_srv::srv::SpeedL::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedL::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request->r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arch(const dobot_bringup_srv::srv::Arch::Request::SharedPtr request, dobot_bringup_srv::srv::Arch::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arch(%d)", request->index);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::cp(const dobot_bringup_srv::srv::CP::Request::SharedPtr request, dobot_bringup_srv::srv::CP::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request->r);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::limZ(const dobot_bringup_srv::srv::LimZ::Request::SharedPtr request, dobot_bringup_srv::srv::LimZ::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request->value);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setArmOrientation(const dobot_bringup_srv::srv::SetArmOrientation::Request::SharedPtr request, dobot_bringup_srv::srv::SetArmOrientation::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request->lorr, request->uord, request->forn, request->config6);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::powerOn(const dobot_bringup_srv::srv::PowerOn::Request::SharedPtr request, dobot_bringup_srv::srv::PowerOn::Response::SharedPtr response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::runScript(const dobot_bringup_srv::srv::RunScript::Request::SharedPtr request, dobot_bringup_srv::srv::RunScript::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request->projectname.c_str());
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::stopScript(const dobot_bringup_srv::srv::StopScript::Request::SharedPtr request, dobot_bringup_srv::srv::StopScript::Response::SharedPtr response)
{
    try
    {
        const char *cmd = "StopScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::pauseScript(const dobot_bringup_srv::srv::PauseScript::Request::SharedPtr request, dobot_bringup_srv::srv::PauseScript::Response::SharedPtr response)
{
    try
    {
        const char *cmd = "PauseScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::continueScript(const dobot_bringup_srv::srv::ContinueScript::Request::SharedPtr request, dobot_bringup_srv::srv::ContinueScript::Response::SharedPtr response)
{
    try
    {
        const char *cmd = "ContinueScript()";
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setSafeSkin(const dobot_bringup_srv::srv::SetSafeSkin::Request::SharedPtr request, dobot_bringup_srv::srv::SetSafeSkin::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setObstacleAvoid(const dobot_bringup_srv::srv::SetObstacleAvoid::Request::SharedPtr request, dobot_bringup_srv::srv::SetObstacleAvoid::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request->status);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setCollisionLevel(const dobot_bringup_srv::srv::SetCollisionLevel::Request::SharedPtr request, dobot_bringup_srv::srv::SetCollisionLevel::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request->level);
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::emergencyStop(const dobot_bringup_srv::srv::EmergencyStop::Request::SharedPtr request, dobot_bringup_srv::srv::EmergencyStop::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}


/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::movJ(const dobot_bringup_srv::srv::MovJ::Request::SharedPtr request, dobot_bringup_srv::srv::MovJ::Response::SharedPtr response)
{
    try
    {
        commander_->movJ(request->x, request->y, request->z, request->a, request->b, request->c);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::movL(const dobot_bringup_srv::srv::MovL::Request::SharedPtr request, dobot_bringup_srv::srv::MovL::Response::SharedPtr response)
{
    try
    {
        commander_->movL(request->x, request->y, request->z, request->a, request->b, request->c);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoJ(const dobot_bringup_srv::srv::ServoJ::Request::SharedPtr request, dobot_bringup_srv::srv::ServoJ::Response::SharedPtr response)
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "servoj");
        commander_->servoJ(request->j1, request->j2, request->j3, request->j4, request->j5, request->j6);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jump(const dobot_bringup_srv::srv::Jump::Request::SharedPtr request, dobot_bringup_srv::srv::Jump::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->offset1, request->offset2, request->offset3,
                request->offset4, request->offset5, request->offset6);
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arc(const dobot_bringup_srv::srv::Arc::Request::SharedPtr request, dobot_bringup_srv::srv::Arc::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->x1,
                request->y1, request->z1, request->rx1, request->ry1, request->rz1, request->x2, request->y2, request->z2,
                request->rx2, request->ry2, request->rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::circle(const dobot_bringup_srv::srv::Circle::Request::SharedPtr request, dobot_bringup_srv::srv::Circle::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)",
                request->count, request->x1, request->y1, request->z1, request->rx1, request->ry1, request->rz1, request->x2,
                request->y2, request->z2, request->rx2, request->ry2, request->rz2);
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoP(const dobot_bringup_srv::srv::ServoP::Request::SharedPtr request, dobot_bringup_srv::srv::ServoP::Response::SharedPtr response)
{
    try
    {
        commander_->servoP(request->x, request->y, request->z, request->a, request->b, request->c);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJ(const dobot_bringup_srv::srv::RelMovJ::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovJ::Response::SharedPtr response)
{
    try
    {
        commander_->relMovJ(request->offset1, request->offset2, request->offset3, request->offset4, request->offset5,
                            request->offset6);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovL(const dobot_bringup_srv::srv::RelMovL::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovL::Response::SharedPtr response)
{
    try
    {
        commander_->relMovL(request->x, request->y, request->z);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jointMovJ(const dobot_bringup_srv::srv::JointMovJ::Request::SharedPtr request, dobot_bringup_srv::srv::JointMovJ::Response::SharedPtr response)
{
    try
    {
        commander_->jointMovJ(request->j1, request->j2, request->j3, request->j4, request->j5, request->j6);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::sync(const dobot_bringup_srv::srv::Sync::Request::SharedPtr request, dobot_bringup_srv::srv::Sync::Response::SharedPtr response)
{
    try
    {
        const char* cmd = "Sync()";
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startTrace(const dobot_bringup_srv::srv::StartTrace::Request::SharedPtr request, dobot_bringup_srv::srv::StartTrace::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request->trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startPath(const dobot_bringup_srv::srv::StartPath::Request::SharedPtr request, dobot_bringup_srv::srv::StartPath::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request->trace_name.c_str(), request->const_val, request->cart);
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startFCTrace(const dobot_bringup_srv::srv::StartFCTrace::Request::SharedPtr request,
                            dobot_bringup_srv::srv::StartFCTrace::Response::SharedPtr response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request->trace_name.c_str());
        commander_->realSendCmd(cmd, strlen(cmd));
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::moveJog(const dobot_bringup_srv::srv::MoveJog::Request::SharedPtr request, dobot_bringup_srv::srv::MoveJog::Response::SharedPtr response)
{
    try
    {
        commander_->moveJog(request->axisid);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_INFO(this->get_logger(), "%s", err.what());
        response->res = -1;
        return false;
    }
}

//RCLCPP_COMPONENTS_REGISTER_NODE(dobot_bringup::Bringup)
