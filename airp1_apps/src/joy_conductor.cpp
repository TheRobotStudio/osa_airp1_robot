/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file joy_conductor.cpp
 * @author Cyril Jourdan <cyril.jourdan.86@gmail.com>
 * @date Modified on Jul 05, 2019
 * @date Created on Oct 2, 2016
 * @version 0.1.1
 * @brief 
 */

/*** Includes ***/
//ROS
#include <ros/ros.h>
//#include <ros/package.h>
//ROS messages
#include <sensor_msgs/Joy.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
//ROS services
#include "airp1_foldy_base_apps/switchNode.h"
#include "airp1_foldy_base_apps/getSlaveCmdArray.h"

#include "osa_common/enums.h"

/*** Defines ***/
#define LOOP_RATE	50 //HEART_BEAT //50
#define NUMBER_MOTORS_BASE 3
#define NUMBER_MOTORS_ARM_HEAD 6

/*** Variables ***/
bool joy_arrived = false;
sensor_msgs::Joy xboxJoy;
osa_msgs::MotorCmdMultiArray mobile_base_motor_cmd_ma;
osa_msgs::MotorCmdMultiArray sea_arm_motor_cmd_ma;

//booleans
bool switch_node = false; //disable by default
bool mobile_base_teleop_enable = false;
bool sea_arm_teleop_enable = false;

/*** Callback functions ***/
void joy_cb(const sensor_msgs::JoyConstPtr& joy)
{
	xboxJoy = *joy;
	joy_arrived = true;
}

/*** Services ***/
bool switchNode(airp1_foldy_base_apps::switchNode::Request  &req, airp1_foldy_base_apps::switchNode::Response &res)
{
	switch_node = req.state;
	return true;
}

/*** Main ***/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "osa_foldy_joy_conductor_client_node");
	ros::NodeHandle nh;

	ros::Rate r(LOOP_RATE);

	//Publishers
	ros::Publisher pub_setMobileBaseCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/foldy_base/motor_cmd_to_filter", 1);
	ros::Publisher pub_set_sea_arm_cmd = nh.advertise<osa_msgs::MotorCmdMultiArray>("/sea_arm/motor_cmd_to_filter", 1);

	//Subscribers
	ros::Subscriber sub_joy = nh.subscribe ("/joy", 10, joy_cb);
	
	//Services
	ros::ServiceServer srv_switchNode = nh.advertiseService("switch_foldy_joy_conductor", switchNode);
	ros::ServiceClient srvClt_switchMobileBaseManual = nh.serviceClient<airp1_foldy_base_apps::switchNode>("switch_foldy_base_manual_srv");
	ros::ServiceClient srvClt_getMobileBaseManualCmd = nh.serviceClient<airp1_foldy_base_apps::getSlaveCmdArray>("get_foldy_base_manual_cmd_srv");

	airp1_foldy_base_apps::getSlaveCmdArray srv_getSlaveCmdArrayMobileBaseManual;
	airp1_foldy_base_apps::switchNode srv_switchNodeMobileBaseManual;
	srv_switchNodeMobileBaseManual.request.state = false;

	//create the commands multi array
	mobile_base_motor_cmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mobile_base_motor_cmd_ma.layout.dim[0].size = NUMBER_MOTORS_BASE;
	mobile_base_motor_cmd_ma.layout.dim[0].stride = NUMBER_MOTORS_BASE;
	mobile_base_motor_cmd_ma.layout.dim[0].label = "motors";
	mobile_base_motor_cmd_ma.layout.data_offset = 0;
	mobile_base_motor_cmd_ma.motor_cmd.clear();
	mobile_base_motor_cmd_ma.motor_cmd.resize(NUMBER_MOTORS_BASE);

	for(int i=0; i<NUMBER_MOTORS_BASE; i++)
	{
		mobile_base_motor_cmd_ma.motor_cmd[i].node_id = i+1;
		mobile_base_motor_cmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		mobile_base_motor_cmd_ma.motor_cmd[i].value = 0;
	}

	//create the commands multi array
	sea_arm_motor_cmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	sea_arm_motor_cmd_ma.layout.dim[0].size = NUMBER_MOTORS_ARM_HEAD;
	sea_arm_motor_cmd_ma.layout.dim[0].stride = NUMBER_MOTORS_ARM_HEAD;
	sea_arm_motor_cmd_ma.layout.dim[0].label = "motors";
	sea_arm_motor_cmd_ma.layout.data_offset = 0;
	sea_arm_motor_cmd_ma.motor_cmd.clear();
	sea_arm_motor_cmd_ma.motor_cmd.resize(NUMBER_MOTORS_ARM_HEAD);

	for(int i=0; i<NUMBER_MOTORS_ARM_HEAD; i++)
	{
		sea_arm_motor_cmd_ma.motor_cmd[i].node_id = i+1;
		sea_arm_motor_cmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		sea_arm_motor_cmd_ma.motor_cmd[i].value = 0;
	}

	//No conductor above, so switched on by default
	switch_node = true;
	mobile_base_teleop_enable = true;
	sea_arm_teleop_enable = true;

	while(ros::ok())
	{
		//Default base value
		for(int i=0; i<NUMBER_MOTORS_BASE; i++)
		{
			mobile_base_motor_cmd_ma.motor_cmd[i].node_id = i+1;
			mobile_base_motor_cmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
			mobile_base_motor_cmd_ma.motor_cmd[i].value = 0;
		}

		//read the joystick inputs
		ros::spinOnce();

		if(switch_node)
		{
			if(joy_arrived)
			{
				if(srvClt_getMobileBaseManualCmd.call(srv_getSlaveCmdArrayMobileBaseManual))
				{
					//publish to the commandBuilder node
					pub_setMobileBaseCommand.publish(srv_getSlaveCmdArrayMobileBaseManual.response.motorCmdMultiArray);
				}
				else
				{
					ROS_DEBUG("Failed to call service get_mobile_base_manual_cmd");
				}

				joy_arrived = false;
			}
		}
	}//while ros ok

	return 0;
}
