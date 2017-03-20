/*********************************************************************
 * tm700_test_node.cpp
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************
 *
 * Author: Howard Chen, ISCI, NCTU
 */

#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/Int32.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


#include "tm_msgs/SetIO.h"
//#include "tm_msgs/SetIORequest.h"
//#include "tm_msgs/SetIOResponse.h"



bool try_move_to_named_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::string& target_name,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setNamedTarget(target_name);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
    return success;
}

bool try_move_to_joint_target(moveit::planning_interface::MoveGroup& group,
                              moveit::planning_interface::MoveGroup::Plan& plan,
                              const std::vector<double>& joint_target,
                              unsigned int max_try_times = 1
                             )
{
    if (!ros::ok()) return false;
    bool success = false;

    for (unsigned int i = 0; i < max_try_times; i++)
    {

        group.setJointValueTarget(joint_target);

        if (group.move())
        {
            success = true;
            break;
        }
        else
        {
            if (!ros::ok()) break;
            sleep(1);
        }
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tm700_stop");
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroup group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    ros::Publisher number_publisher = node_handle.advertise<std_msgs::Int32>("/numbers",0);
    ros::Rate loop_rate(10);

    int stop_message = 0;

    ROS_INFO("Collision detection start!!");

    while(ros::ok())
    {
        std_msgs::Int32 msg;

        if (getchar() && ros::ok())
        {
            group.stop();
            ROS_INFO("trajectory stop!!\n");
            stop_message = 1;
        }

        msg.data = stop_message;
        number_publisher.publish(msg);
        ROS_INFO("stop state : %d\n",msg.data);


        if (getchar() && ros::ok())
        {
            stop_message = 0;
            msg.data = stop_message;
            number_publisher.publish(msg);
            ROS_INFO("trajectory resume!!\n");
        }

        ros::spinOnce();

    }



    return 0;
}
