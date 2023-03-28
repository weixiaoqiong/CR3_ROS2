/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

/*
#include <ros/ros.h>
#include <ros/param.h>
#include <dobot_bringup/cr5_robot.h>
#include <sensor_msgs/JointState.h>
#include <dobot_bringup/ToolVectorActual.h>
*/
#include <rclcpp/rclcpp.hpp>
#include "dobot_bringup/cr5_robot.h"
#include <sensor_msgs/msg/joint_state.hpp>
//#include <dobot_bringup_msg/msg/robot_status.hpp>
//#include <dobot_bringup_msg/msg/tool_vector_actual.hpp>
//#include <rclcpp_components/register_node_macro.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout<<"hello hello\n";
    auto cr5_bringup_node = std::make_shared<CR5Robot>(rclcpp::NodeOptions());
    rclcpp::spin(cr5_bringup_node);
    rclcpp::shutdown();

    return 0;
}
//RCLCPP_COMPONENTS_REGISTER_NODE(dobot_bringup::DobotBringup)
