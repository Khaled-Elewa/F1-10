#include "navigation_stack_control_converter.h"
#include <algorithm>
#include <boost/algorithm/clamp.hpp>
#include <eigen3/Eigen/Dense>
#include <math.h> 

using namespace Eigen;

NavigationStackControlConverter::NavigationStackControlConverter()
{
    this->m_command_velocity_subscriber =
        this->m_node_handle.subscribe<geometry_msgs::Twist>(TOPIC_CMD_VEL, 1,
                                                            &NavigationStackControlConverter::convertCallback, this);

    this->m_drive_param_publisher = this->m_node_handle.advertise<drive_msgs::drive_param>(TOPIC_DRIVE_PARAM, 10);
}

void NavigationStackControlConverter::convertCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_message)
{

    double velocity = cmd_vel_message->linear.x;
    double omega = cmd_vel_message->angular.z;
    double r = velocity / omega;
    double steering = atan(0.325*r);
    drive_msgs::drive_param control_message;

    control_message.velocity = velocity ; 
    control_message.angle = steering;
    m_drive_param_publisher.publish(control_message);
}
