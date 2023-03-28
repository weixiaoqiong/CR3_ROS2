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

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "commander.h"
#include <rclcpp_action/rclcpp_action.hpp>
//#include <actionlib/server/action_server.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
//#include <control_msgs/FollowJointTrajectoryAction.h>

#include "dobot_bringup_msg/msg/robot_status.hpp"
#include "dobot_bringup_msg/msg/tool_vector_actual.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "dobot_bringup_srv/srv/acc_j.hpp"
#include "dobot_bringup_srv/srv/acc_l.hpp"
#include "dobot_bringup_srv/srv/ao.hpp"
#include "dobot_bringup_srv/srv/ao_execute.hpp"
#include "dobot_bringup_srv/srv/arc.hpp"
#include "dobot_bringup_srv/srv/arch.hpp"
#include "dobot_bringup_srv/srv/circle.hpp"
#include "dobot_bringup_srv/srv/clear_error.hpp"
#include "dobot_bringup_srv/srv/continue_script.hpp"
#include "dobot_bringup_srv/srv/cp.hpp"
#include "dobot_bringup_srv/srv/disable_robot.hpp"
#include "dobot_bringup_srv/srv/do.hpp"
#include "dobot_bringup_srv/srv/do_execute.hpp"
#include "dobot_bringup_srv/srv/emergency_stop.hpp"
#include "dobot_bringup_srv/srv/enable_robot.hpp"
#include "dobot_bringup_srv/srv/get_hold_regs.hpp"
#include "dobot_bringup_srv/srv/joint_mov_j.hpp"
#include "dobot_bringup_srv/srv/jump.hpp"
#include "dobot_bringup_srv/srv/lim_z.hpp"
#include "dobot_bringup_srv/srv/move_jog.hpp"
#include "dobot_bringup_srv/srv/mov_j.hpp"
#include "dobot_bringup_srv/srv/mov_l.hpp"
#include "dobot_bringup_srv/srv/pause_script.hpp"
#include "dobot_bringup_srv/srv/pay_load.hpp"
#include "dobot_bringup_srv/srv/power_on.hpp"
#include "dobot_bringup_srv/srv/rel_mov_j.hpp"
#include "dobot_bringup_srv/srv/rel_mov_l.hpp"
#include "dobot_bringup_srv/srv/reset_robot.hpp"
#include "dobot_bringup_srv/srv/robot_mode.hpp"
#include "dobot_bringup_srv/srv/run_script.hpp"
#include "dobot_bringup_srv/srv/servo_j.hpp"
#include "dobot_bringup_srv/srv/servo_p.hpp"
#include "dobot_bringup_srv/srv/set_arm_orientation.hpp"
#include "dobot_bringup_srv/srv/set_collision_level.hpp"
#include "dobot_bringup_srv/srv/set_hold_regs.hpp"
#include "dobot_bringup_srv/srv/set_obstacle_avoid.hpp"
#include "dobot_bringup_srv/srv/set_safe_skin.hpp"
#include "dobot_bringup_srv/srv/speed_factor.hpp"
#include "dobot_bringup_srv/srv/speed_j.hpp"
#include "dobot_bringup_srv/srv/speed_l.hpp"
#include "dobot_bringup_srv/srv/start_fc_trace.hpp"
#include "dobot_bringup_srv/srv/start_path.hpp"
#include "dobot_bringup_srv/srv/start_trace.hpp"
#include "dobot_bringup_srv/srv/stop_script.hpp"
#include "dobot_bringup_srv/srv/sync.hpp"
#include "dobot_bringup_srv/srv/tool.hpp"
#include "dobot_bringup_srv/srv/tool_do.hpp"
#include "dobot_bringup_srv/srv/tool_do_execute.hpp"
#include "dobot_bringup_srv/srv/user.hpp"


//using namespace actionlib;
//using namespace control_msgs;

/**
 * CR5Robot
 */
//class CR5Robot : protected ActionServer<FollowJointTrajectoryAction>
//class CR5Robot : protected rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>
class CR5Robot : public rclcpp::Node
{
private:
    double goal_[6];
    uint32_t index_;
    //rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::TimerBase::SharedPtr movj_timer_;
    //ros::Timer timer_;
    //ros::Timer movj_timer_;
    double trajectory_duration_;
    //rclcpp::Node::SharedPtr control_nh_;
    //ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<rclcpp::ServiceBase::SharedPtr> server_tbl_;
    //std::vector<ros::ServiceServer> server_tbl_;

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;

    double position[6];

    sensor_msgs::msg::JointState joint_state_msg;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    dobot_bringup_msg::msg::RobotStatus robot_status_msg;
    rclcpp::Publisher<dobot_bringup_msg::msg::RobotStatus>::SharedPtr robot_status_pub; 
    dobot_bringup_msg::msg::ToolVectorActual tool_vector_actual_msg;
    rclcpp::Publisher<dobot_bringup_msg::msg::ToolVectorActual>::SharedPtr tool_vector_pub; 
    rclcpp::TimerBase::SharedPtr pub_timer;


public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    CR5Robot(const rclcpp::NodeOptions & options);
    //CR5Robot(ros::NodeHandle& nh, std::string name); //构造函数，没有返回值，也不声明类型，不承担任何重要工作，负责清理
    //用构造函数创建对象后，程序负责跟踪该对象，直到过期为止。对象过期时，程序将自动调用一个特殊的成员函数

    /**
     * CR5Robot
     */
    ~CR5Robot() override; //析构函数，没有参数，

    using CR5FollowJoint = control_msgs::action::FollowJointTrajectory;
    using GoalHandleCR5FollowJoint = rclcpp_action::ServerGoalHandle<CR5FollowJoint>;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool enableRobot(const dobot_bringup_srv::srv::EnableRobot::Request::SharedPtr request, dobot_bringup_srv::srv::EnableRobot::Response::SharedPtr response);
    bool disableRobot(const dobot_bringup_srv::srv::DisableRobot::Request::SharedPtr request, dobot_bringup_srv::srv::DisableRobot::Response::SharedPtr response);
    bool clearError(const dobot_bringup_srv::srv::ClearError::Request::SharedPtr request, dobot_bringup_srv::srv::ClearError::Response::SharedPtr response);
    bool resetRobot(const dobot_bringup_srv::srv::ResetRobot::Request::SharedPtr request, dobot_bringup_srv::srv::ResetRobot::Response::SharedPtr response);
    bool speedFactor(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
    bool user(const dobot_bringup_srv::srv::User::Request::SharedPtr request, dobot_bringup_srv::srv::User::Response::SharedPtr response);
    bool tool(const dobot_bringup_srv::srv::Tool::Request::SharedPtr request, dobot_bringup_srv::srv::Tool::Response::SharedPtr response);
    bool robotMode(const dobot_bringup_srv::srv::RobotMode::Request::SharedPtr request, dobot_bringup_srv::srv::RobotMode::Response::SharedPtr response);
    bool payload(const dobot_bringup_srv::srv::PayLoad::Request::SharedPtr request, dobot_bringup_srv::srv::PayLoad::Response::SharedPtr response);
    bool DO(const dobot_bringup_srv::srv::DO::Request::SharedPtr request, dobot_bringup_srv::srv::DO::Response::SharedPtr response);
    bool DOExecute(const dobot_bringup_srv::srv::DOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::DOExecute::Response::SharedPtr response);
    bool toolDO(const dobot_bringup_srv::srv::ToolDO::Request::SharedPtr request, dobot_bringup_srv::srv::ToolDO::Response::SharedPtr response);
    bool toolDOExecute(const dobot_bringup_srv::srv::ToolDOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::ToolDOExecute::Response::SharedPtr response);
    bool AO(const dobot_bringup_srv::srv::AO::Request::SharedPtr request, dobot_bringup_srv::srv::AO::Response::SharedPtr response);
    bool AOExecute(const dobot_bringup_srv::srv::AOExecute::Request::SharedPtr request, dobot_bringup_srv::srv::AOExecute::Response::SharedPtr response);
    bool accJ(const dobot_bringup_srv::srv::AccJ::Request::SharedPtr request, dobot_bringup_srv::srv::AccJ::Response::SharedPtr response);
    bool accL(const dobot_bringup_srv::srv::AccL::Request::SharedPtr request, dobot_bringup_srv::srv::AccL::Response::SharedPtr response);
    bool speedJ(const dobot_bringup_srv::srv::SpeedJ::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedJ::Response::SharedPtr response);
    bool speedL(const dobot_bringup_srv::srv::SpeedL::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedL::Response::SharedPtr response);
    bool arch(const dobot_bringup_srv::srv::Arch::Request::SharedPtr request, dobot_bringup_srv::srv::Arch::Response::SharedPtr response);
    bool cp(const dobot_bringup_srv::srv::CP::Request::SharedPtr request, dobot_bringup_srv::srv::CP::Response::SharedPtr response);
    bool limZ(const dobot_bringup_srv::srv::LimZ::Request::SharedPtr request, dobot_bringup_srv::srv::LimZ::Response::SharedPtr response);
    bool setArmOrientation(const dobot_bringup_srv::srv::SetArmOrientation::Request::SharedPtr request, dobot_bringup_srv::srv::SetArmOrientation::Response::SharedPtr response);
    bool powerOn(const dobot_bringup_srv::srv::PowerOn::Request::SharedPtr request, dobot_bringup_srv::srv::PowerOn::Response::SharedPtr response);
    bool runScript(const dobot_bringup_srv::srv::RunScript::Request::SharedPtr request, dobot_bringup_srv::srv::RunScript::Response::SharedPtr response);
    bool stopScript(const dobot_bringup_srv::srv::StopScript::Request::SharedPtr request, dobot_bringup_srv::srv::StopScript::Response::SharedPtr response);
    bool pauseScript(const dobot_bringup_srv::srv::PauseScript::Request::SharedPtr request, dobot_bringup_srv::srv::PauseScript::Response::SharedPtr response);
    bool continueScript(const dobot_bringup_srv::srv::ContinueScript::Request::SharedPtr request, dobot_bringup_srv::srv::ContinueScript::Response::SharedPtr response);
//    bool getHoldRegs(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool setHoldRegs(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
    bool setSafeSkin(const dobot_bringup_srv::srv::SetSafeSkin::Request::SharedPtr request, dobot_bringup_srv::srv::SetSafeSkin::Response::SharedPtr response);
    bool setObstacleAvoid(const dobot_bringup_srv::srv::SetObstacleAvoid::Request::SharedPtr request, dobot_bringup_srv::srv::SetObstacleAvoid::Response::SharedPtr response);
//    bool getTraceStartPose(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool getPathStartPose(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool positiveSolution(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool inverseSolution(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
    bool setCollisionLevel(const dobot_bringup_srv::srv::SetCollisionLevel::Request::SharedPtr request, dobot_bringup_srv::srv::SetCollisionLevel::Response::SharedPtr response);
//    bool handleTrajPoints(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool getSixForceData(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool getAngle(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
//    bool getPose(const dobot_bringup_srv::srv::SpeedFactor::Request::SharedPtr request, dobot_bringup_srv::srv::SpeedFactor::Response::SharedPtr response);
    bool emergencyStop(const dobot_bringup_srv::srv::EmergencyStop::Request::SharedPtr request, dobot_bringup_srv::srv::EmergencyStop::Response::SharedPtr response);

    bool movJ(const dobot_bringup_srv::srv::MovJ::Request::SharedPtr request, dobot_bringup_srv::srv::MovJ::Response::SharedPtr response);
    bool movL(const dobot_bringup_srv::srv::MovL::Request::SharedPtr request, dobot_bringup_srv::srv::MovL::Response::SharedPtr response);
    bool jointMovJ(const dobot_bringup_srv::srv::JointMovJ::Request::SharedPtr request, dobot_bringup_srv::srv::JointMovJ::Response::SharedPtr response);
    bool jump(const dobot_bringup_srv::srv::Jump::Request::SharedPtr request, dobot_bringup_srv::srv::Jump::Response::SharedPtr response);
    bool relMovJ(const dobot_bringup_srv::srv::RelMovJ::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovJ::Response::SharedPtr response);
    bool relMovL(const dobot_bringup_srv::srv::RelMovL::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovL::Response::SharedPtr response);
    //bool MovLIO(const dobot_bringup_srv::srv::RelMovL::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovL::Response::SharedPtr response);
    //bool MovJIO(const dobot_bringup_srv::srv::RelMovL::Request::SharedPtr request, dobot_bringup_srv::srv::RelMovL::Response::SharedPtr response);
    bool arc(const dobot_bringup_srv::srv::Arc::Request::SharedPtr request, dobot_bringup_srv::srv::Arc::Response::SharedPtr response);
    bool circle(const dobot_bringup_srv::srv::Circle::Request::SharedPtr request, dobot_bringup_srv::srv::Circle::Response::SharedPtr response);
    bool servoJ(const dobot_bringup_srv::srv::ServoJ::Request::SharedPtr request, dobot_bringup_srv::srv::ServoJ::Response::SharedPtr response);
    bool servoP(const dobot_bringup_srv::srv::ServoP::Request::SharedPtr request, dobot_bringup_srv::srv::ServoP::Response::SharedPtr response);
    bool sync(const dobot_bringup_srv::srv::Sync::Request::SharedPtr request, dobot_bringup_srv::srv::Sync::Response::SharedPtr response);
    bool startTrace(const dobot_bringup_srv::srv::StartTrace::Request::SharedPtr request, dobot_bringup_srv::srv::StartTrace::Response::SharedPtr response);
    bool startPath(const dobot_bringup_srv::srv::StartPath::Request::SharedPtr request, dobot_bringup_srv::srv::StartPath::Response::SharedPtr response);
    bool startFCTrace(const dobot_bringup_srv::srv::StartFCTrace::Request::SharedPtr request, dobot_bringup_srv::srv::StartFCTrace::Response::SharedPtr response);
    bool moveJog(const dobot_bringup_srv::srv::MoveJog::Request::SharedPtr request, dobot_bringup_srv::srv::MoveJog::Response::SharedPtr response);

private:
    void feedbackHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle);//publish feedback
    void moveHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle);//execute
    rclcpp_action::GoalResponse goalHandle(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CR5FollowJoint::Goal> goal);//goal response
    rclcpp_action::CancelResponse cancelHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle);//cancel
    void acceptHandle(const std::shared_ptr<GoalHandleCR5FollowJoint> handle);//accept
    
    void pub_timer_callback();


    
    /*
    void feedbackHandle(const ros::TimerEvent& tm,
                        actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    */
};
