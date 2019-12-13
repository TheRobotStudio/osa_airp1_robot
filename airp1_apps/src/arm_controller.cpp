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
 * @file arm_controller.cpp
 * @author Cyril Jourdan
 * @date Fev 8, 2019
 * @version 0.1.1
 * @brief File for the controller of the SEA Arm and simple hand 
 *
 * Contact: contact@therobotstudio.com
 * Created on : Jan 29, 2019
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <osa_msgs/MotorCmdMultiArray.h>
#include "osa_common/enums.h"

#define NUMBER_MOTORS_ARM 4

using namespace std;

sensor_msgs::JointState joint_state;
osa_msgs::MotorCmdMultiArray armMotorCmd_ma;

bool joint_state_arrived = false;

const double gear_ratio_ec32f = 190/1;
const double enc_tic_per_motor_turn_ec32f = 6*4; //hall sensor
const double enc_tic_per_shaft_turn_ec32f = enc_tic_per_motor_turn_ec32f*gear_ratio_ec32f;

const double gear_ratio_dcx26 = 62/1;
const double enc_tic_per_motor_turn_dcx26 = 128*4; //multiply by 4 for the quadrate encoder
const double enc_tic_per_shaft_turn_dcx26 = enc_tic_per_motor_turn_dcx26*gear_ratio_dcx26;

double p1 = 0.0;
double p2 = 0.0;
double p3 = 0.0;
double p4 = 0.0; 

void jointStatesCallback(const sensor_msgs::JointStateConstPtr &js)
{
	joint_state = *js;
	joint_state_arrived = true;
}

int main(int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "osa_airp1_arm_controller");
	ros::NodeHandle nh("~");

	//Subscribers
	ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);
	//Publishers
	ros::Publisher pub_setMobileBaseCommand = nh.advertise<osa_msgs::MotorCmdMultiArray>("/sea_arm/motor_cmd_to_filter", 1); 

	ros::Rate loop_rate(10); //test up to 50Hz

	//create the commands multi array
	armMotorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	armMotorCmd_ma.layout.dim[0].size = NUMBER_MOTORS_ARM;
	armMotorCmd_ma.layout.dim[0].stride = NUMBER_MOTORS_ARM;
	armMotorCmd_ma.layout.dim[0].label = "motors";
	armMotorCmd_ma.layout.data_offset = 0;
	armMotorCmd_ma.motor_cmd.clear();
	armMotorCmd_ma.motor_cmd.resize(NUMBER_MOTORS_ARM);

	for(int i=0; i<NUMBER_MOTORS_ARM; i++)
	{
		armMotorCmd_ma.motor_cmd[i].node_id = i+1;
		armMotorCmd_ma.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		armMotorCmd_ma.motor_cmd[i].value = 0;
	}

	ROS_INFO("start node");

	while(ros::ok())
	{
		//read the SLAM output
		ros::spinOnce();

		if(joint_state_arrived)
		{
			//reset base value to default
			for(int i=0; i<NUMBER_MOTORS_ARM; i++)
			{
				armMotorCmd_ma.motor_cmd[i].node_id = i+1;
				armMotorCmd_ma.motor_cmd[i].command = SET_TARGET_POSITION;
				armMotorCmd_ma.motor_cmd[i].value = 0;			
			}

			p1 = -joint_state.position[3]*enc_tic_per_shaft_turn_dcx26/(2*M_PI);
			p2 = -joint_state.position[4]*enc_tic_per_shaft_turn_ec32f/(2*M_PI);
			p3 = -joint_state.position[5]*enc_tic_per_shaft_turn_ec32f/(2*M_PI);
			p4 = -joint_state.position[6]*enc_tic_per_shaft_turn_ec32f/(2*M_PI);

			ROS_INFO("p1=%f, p2=%f, p3=%f, p4=%f", p1, p2, p3, p4);

			// Asign result to the msg
			armMotorCmd_ma.motor_cmd[0].value = (int)p1; 
			armMotorCmd_ma.motor_cmd[1].value = (int)p2; 
			armMotorCmd_ma.motor_cmd[2].value = (int)p3;
			armMotorCmd_ma.motor_cmd[3].value = (int)p4;

			//publish to the command filter node
			pub_setMobileBaseCommand.publish(armMotorCmd_ma);

			joint_state_arrived = false;

			loop_rate.sleep();
		}
	}
}
