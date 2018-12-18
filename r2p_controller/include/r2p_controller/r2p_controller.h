/*
 * Copyright (c) 2018, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice, this
 *        list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
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
 * @file r2p_controller.h
 * @author Cyril Jourdan <cyril.jourdan@therobotstudio.com>
 * @date Modified on Dec 13, 2018
 * @date Created on Dec 13, 2018
 * @version 0.1.1
 * @brief Header file for class R2PController
 */

#ifndef R2P_ROBOT_R2P_CONTROLLER_H
#define R2P_ROBOT_R2P_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>


namespace r2p_controller
{

class R2PControllerClass: public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
	bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
	void update(const ros::Time& time, const ros::Duration& period);
	void starting(const ros::Time& time);
	void stopping(const ros::Time& time);

private:
	hardware_interface::JointHandle joint_;
	double init_pos_;
};
}//r2p_controller_ns namespace

#endif // R2P_ROBOT_R2P_CONTROLLER_H
