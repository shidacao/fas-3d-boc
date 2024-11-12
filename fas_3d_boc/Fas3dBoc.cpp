/****************************************************************************
 *
 *   Copyright (c) 2013-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Fas3dBoc.cpp
 *
 * Fully-actuated-system-based three-dimentional bearing-only circumnavigation task
 *
 * This program is the corresponding simulation part of the paper
 * “Three-dimensional bearing-only circumnavigation based on fully actuated system approach”
 *
 * This is a process-oriented style program
 *
 * @author Shida Cao (曹世达) <shida_cao@163.com>
 * The author of this program is from Center for Control and Guidance Technology,
 * School of Astronautics, Harbin Institute of Technology
 * (哈尔滨工业大学航天学院控制理论与制导技术研究中心)
 */

/**
 * Abbreviations in this program:
 *  veh: vehicle
 *  pos: position
 *  vel: velocity
 *  acc: acceleration
 *  traj: trajectory
 *  sp: setpoint
 *  circum: circumnavigation
 */

#include <commander/px4_custom_mode.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <cstdio>
#include <matrix/math.hpp>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <iomanip>

namespace fas_3d_boc{

const double RAD2DEG = 180.0 / M_PI;
const float RAD2DEGf = 180.0f / M_PIf;
// const double DEG2RAD = M_PI / 180.0;
const float DEG2RADf = M_PIf / 180.0f;

using namespace time_literals;

////////// configuration /////////////////////////////////////////
const float period = 0.1f;                            		//
const hrt_abstime period_abstime = 100_ms;            		//
const float hold_height = 4.0f;                       		//
const hrt_abstime hold_duration = 1_s;                		//
const std::string figure_dir_name{"FAS-3D-BOC-figure"};     	//
                                        	      		//
struct CircumParam                                    		//
{                                                     		//
	const matrix::Vector3f x{3,0,-6};             		//
	const matrix::Vector3f n{0,0,1};              		//
	const float arho0 = 1;                        		//
	const float arho1 = 1;                        		//
	const float av2 = 1;                          		//
	const float al0 = 1;                          		//
	const float al1 = 1;                          		//
	const float d = 3;                            		//
	const float alpha = 2;                        		//
}circum_param;                                        		//
//////////////////////////////////////////////////////////////////

struct CircumState
{
	float time;
	float rho;
	float rhohat;
	matrix::Vector3f tau1;
	matrix::Vector3f tau2;
	matrix::Vector3f tau3;
	float v1;
	float v2;
	float v3;
	float omega2;
	float omega3;
	float theta;
	float l;
}circum_state;

struct DroneState
{
	matrix::Vector3f pos;
	matrix::Vector3f vel;
	matrix::Vector3f acc;
	float yaw;
}drone_state;

// declarations
float getRhohat(matrix::Vector3f y_now, matrix::Vector3f tau1_now, matrix::Vector3f tau23_now, float v1_now);
matrix::Vector3f getAccThenRecord(const matrix::Vector3f &y, const matrix::Vector3f &ydot, const CircumParam & param, CircumState &record);
int fas_3d_boc_run(int argc, char **argv);
void print(const DroneState &state);
void print(const CircumState &state);
void recordToFiles();
void recordScalarToFile(float time, float variable, std::string filename);
void recordVectorToFile(float time, matrix::Vector3f variable, std::string filename);

// main function
extern "C" __EXPORT int fas_3d_boc_main(int argc, char *argv[])
{
	PX4_INFO("fas_3d_boc is started.");

	px4_task_spawn_cmd("fas_3d_boc", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024, fas_3d_boc_run, nullptr);

	return 0;
}

// actual main function
int fas_3d_boc_run(int argc, char **argv)
{
	// vehicle position in local frame
	uORB::Subscription veh_local_pos_sub{ORB_ID(vehicle_local_position)};
	vehicle_local_position_s veh_local_pos_msg;

	// vehicle state
	uORB::Subscription veh_status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s veh_status_msg;

	// trajectory setpoint
	uORB::Publication<trajectory_setpoint_s> traj_sp_pub{ORB_ID(trajectory_setpoint)};
	trajectory_setpoint_s traj_sp_msg;
	memset(&traj_sp_msg, 0, sizeof(trajectory_setpoint_s));

	// offboard control mode
	uORB::Publication<offboard_control_mode_s> offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	offboard_control_mode_s offboard_control_mode_msg;
	memset(&offboard_control_mode_msg, 0, sizeof(offboard_control_mode_s));

	// vehicle command
	uORB::Publication<vehicle_command_s> veh_command_pub{ORB_ID(vehicle_command)};
	vehicle_command_s veh_command_msg;
	memset(&veh_command_msg, 0, sizeof(vehicle_command_s));

	while (true)
	{
		// judge whether armed and switched to offboard mode
		veh_status_sub.update(&veh_status_msg);
		bool is_armed = veh_status_msg.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? true : false;
		bool has_changed_to_offboard = veh_status_msg.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD ? true : false;
		if (is_armed && has_changed_to_offboard)
		{
			PX4_INFO("armed and switched to offboard mode");
			break;
		}

		// publish trajectory setpoint
		traj_sp_msg.timestamp = hrt_absolute_time();
		veh_local_pos_sub.update(&veh_local_pos_msg);
		traj_sp_msg.position[0] = veh_local_pos_msg.x;
		traj_sp_msg.position[1] = veh_local_pos_msg.y;
		traj_sp_msg.position[2] = veh_local_pos_msg.z;
		traj_sp_pub.publish(traj_sp_msg);

		// publish offboard_control_mode message
		offboard_control_mode_msg.timestamp = hrt_absolute_time();
		offboard_control_mode_msg.position = true;
		offboard_control_mode_pub.publish(offboard_control_mode_msg);

		// publish arm command and set mode command
		veh_command_msg.target_system = veh_status_msg.system_id;
		veh_command_msg.target_component = veh_status_msg.component_id;

		veh_command_msg.timestamp = hrt_absolute_time();
		veh_command_msg.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
		veh_command_msg.param1 = 1.0f;
		veh_command_pub.publish(veh_command_msg);

		usleep(10_ms);

		veh_command_msg.timestamp = hrt_absolute_time();
		veh_command_msg.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		veh_command_msg.param1 = 1.0f;
		veh_command_msg.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		veh_command_pub.publish(veh_command_msg);
	}

	typedef enum _TaskState{BEGIN, TAKEOFF, HOLD, CIRCUM} TaskState;
	TaskState task_state = TaskState::BEGIN;

	while (true)
	{
		veh_local_pos_sub.update(&veh_local_pos_msg);

		// record drone state then print
		drone_state.pos(0) = veh_local_pos_msg.x;
		drone_state.pos(1) = veh_local_pos_msg.y;
		drone_state.pos(2) = veh_local_pos_msg.z;
		drone_state.vel(0) = veh_local_pos_msg.vx;
		drone_state.vel(1) = veh_local_pos_msg.vy;
		drone_state.vel(2) = veh_local_pos_msg.vz;
		drone_state.acc(0) = veh_local_pos_msg.ax;
		drone_state.acc(1) = veh_local_pos_msg.ay;
		drone_state.acc(2) = veh_local_pos_msg.az;
		drone_state.yaw = veh_local_pos_msg.heading;
		print(drone_state);

		switch (task_state)
		{
			case TaskState::BEGIN:
			{
				const float begin_x = veh_local_pos_msg.x;
				const float begin_y = veh_local_pos_msg.y;
				const float begin_z = veh_local_pos_msg.z;

				traj_sp_msg.position[0] = begin_x;
				traj_sp_msg.position[1] = begin_y;
				traj_sp_msg.position[2] = begin_z - hold_height;
				offboard_control_mode_msg.position = true;

				task_state = TaskState::TAKEOFF;// BEGIN state exists only once
				PX4_INFO("task begins");
				break;
			}
			case TaskState::TAKEOFF:
			{
				PX4_INFO("takeing off, now position is (%7.3f, %7.3f, %7.3f)",
					 (double)veh_local_pos_msg.x,
					 (double)veh_local_pos_msg.y,
					 (double)veh_local_pos_msg.z);

				bool is_takeoff_finished;
				is_takeoff_finished =
				matrix::Vector3f{veh_local_pos_msg.x - traj_sp_msg.position[0],
				     veh_local_pos_msg.y - traj_sp_msg.position[1],
				     veh_local_pos_msg.z - traj_sp_msg.position[2]}.norm() < 0.2f; // 20cm
				if (is_takeoff_finished)
				{
					task_state = TaskState::HOLD;
					PX4_INFO("takeoff finished, now start to hold");
				}
				break;
			}
			case TaskState::HOLD:
			{
				static const hrt_abstime hold_start_time = hrt_absolute_time();
				hrt_abstime hold_time = hrt_absolute_time() - hold_start_time;
				if (hold_time > hold_duration)
					task_state = TaskState::CIRCUM;
				else
					PX4_INFO("holding for %.3fs", static_cast<double>(hold_time) * 1e-6);
				break;
			}
			case TaskState::CIRCUM:
			{
				PX4_INFO("doing circumnavigation work");

				traj_sp_msg.position[0] = NAN;
				traj_sp_msg.position[1] = NAN;
				traj_sp_msg.position[2] = NAN;

				traj_sp_msg.velocity[0] = NAN;
				traj_sp_msg.velocity[1] = NAN;
				traj_sp_msg.velocity[2] = NAN;

				matrix::Vector3f pos_measured, vel_measured, acc_tobe;
				pos_measured(0) = veh_local_pos_msg.x;
				pos_measured(1) = veh_local_pos_msg.y;
				pos_measured(2) = veh_local_pos_msg.z;
				vel_measured(0) = veh_local_pos_msg.vx;
				vel_measured(1) = veh_local_pos_msg.vy;
				vel_measured(2) = veh_local_pos_msg.vz;

				acc_tobe = getAccThenRecord(pos_measured, vel_measured, circum_param, circum_state);
				print(circum_state);

				traj_sp_msg.acceleration[0] = acc_tobe(0);
				traj_sp_msg.acceleration[1] = acc_tobe(1);
				traj_sp_msg.acceleration[2] = acc_tobe(2);

				traj_sp_msg.yaw = 0.0f * DEG2RADf;

				offboard_control_mode_msg.position = false;
				offboard_control_mode_msg.velocity = false;
				offboard_control_mode_msg.acceleration = true;
				break;
			}
			default:
				break;
		}

		// publish
		offboard_control_mode_msg.timestamp = hrt_absolute_time();
		offboard_control_mode_pub.publish(offboard_control_mode_msg);

		veh_status_sub.update(&veh_status_msg);
		bool is_now_offboard = veh_status_msg.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
		if (is_now_offboard)
		{
			traj_sp_msg.timestamp = hrt_absolute_time();
			traj_sp_pub.publish(traj_sp_msg);
		}

		usleep(period_abstime);
	}

	return 0;
}

// controller
matrix::Vector3f getAccThenRecord(const matrix::Vector3f &y, const matrix::Vector3f &ydot, const CircumParam &param, CircumState &record)
{
	static u_int num = 0;

	static const hrt_abstime circum_start_time_abstime = hrt_absolute_time();
	float circum_time = num == 0 ? 0.0f : (hrt_absolute_time() - circum_start_time_abstime) * 1e-6f;

	const matrix::Vector3f x = param.x;
	const matrix::Vector3f n = param.n;
	const float arho0 = param.arho0;
	const float arho1 = param.arho1;
	const float av2 = param.av2;
	const float al0 = param.al0;
	const float al1 = param.al1;
	const float d = param.d;
	const float alpha = param.alpha;

	float  v1, v2, v3, omega2, omega3, rho, theta, l, ldot, erhohat, ev2, rhohat;
	matrix::Vector3f tau1, tau2, tau3, tau23, f, w, yddot;
	matrix::Matrix<float,3,3> Binv, R, Temp;

	rho = (x - y).norm();

	tau1 = (x - y) / (x - y).norm();
	tau2 = n.cross(tau1) / n.cross(tau1).norm();
	tau3 = tau1.cross(tau2);

	v1 = tau1.dot(ydot);
	v2 = tau2.dot(ydot);
	v3 = tau3.dot(ydot);

	omega2 = v2 / rho;
	omega3 = v3 / rho;

	theta = std::acos(n.dot(tau1)) - M_PIf  / 2.0f;

	l = rho * theta;
	static float l_last = l;

	tau23 = std::sqrt(v2 * v2 + v3 * v3) < 0.2f ? matrix::Vector3f{0,0,0} :
		(v2 * tau2 + v3 * tau3) / std::sqrt(v2 * v2 + v3 * v3);

	rhohat = getRhohat(y, tau1, tau23, v1);

	R(0,0) = tau1(0);
	R(1,0) = tau1(1);
	R(2,0) = tau1(2);

	R(0,1) = tau2(0);
	R(1,1) = tau2(1);
	R(2,1) = tau2(2);

	R(0,2) = tau3(0);
	R(1,2) = tau3(1);
	R(2,2) = tau3(2);

	Temp(0,0) = -1;
	Temp(1,1) = Temp(2,2) = 1;
	Temp(0,1) = Temp(0,2) = Temp(1,0) = Temp(1,2) = Temp(2,1) = 0;

	Binv = R * Temp;

	f(0) = v2 * omega2 + v3 * omega3;
	f(1) = v1 * omega2 + v3 * omega2 * std::tan(theta);
	f(2) = v2 * omega2 * theta + v3 * omega3 * theta - v2 * omega2 * std::tan(theta);

	erhohat = rhohat - d;
	ev2 = v2 - alpha;
	ldot = (l - l_last) / period;

	w(0) = - arho1 * v1 + arho0 * erhohat;
	w(1) = av2 * ev2;
	w(2) = al1 * ldot + al0 * l;

	yddot = - Binv * (f + w);

	l_last = l;

	// record
	record.time = circum_time;
	record.rho = rho;
	record.rhohat = rhohat;
	record.tau1 = tau1;
	record.tau2 = tau2;
	record.tau3 = tau3;
	record.v1 = v1;
	record.v2 = v2;
	record.v3 = v3;
	record.omega2 = omega2;
	record.omega3 = omega3;
	record.theta = theta;
	record.l = l;

	// record to file
	recordScalarToFile(circum_time, rho, "rho.txt");
	recordScalarToFile(circum_time, rhohat, "rhohat.txt");
	float rhotilde = rho - rhohat;
	recordScalarToFile(circum_time, rhotilde, "rhotilde.txt");
	float erho = rho - d;
	recordScalarToFile(circum_time, erho, "erho.txt");
	recordScalarToFile(circum_time, ev2, "ev2.txt");
	recordScalarToFile(circum_time, l, "l.txt");
	recordVectorToFile(circum_time, y, "y.txt");

	num++;

	return yddot;
}

// estimator
float getRhohat(matrix::Vector3f y_now, matrix::Vector3f tau1_now, matrix::Vector3f tau23_now, float v1_now)
{
	static u_int num = 0;

	static const float rhohat0 = 1, k = 1;
	static matrix::Vector3f y_last,	y_last_last, tau1_last,	tau1_last_last,	tau23_last;
	static float rhohat_last, rhohat_last_last, v1_last;

	matrix::Vector3f xhat_last, xhat_last_last, xhatdot_last_last;
	float rhohat_now, rhohatdot_last;

	if (num == 0)
	{
		rhohat_now = rhohat0;
	}

	if (num == 1)
	{
		rhohatdot_last = - v1_last;
		rhohat_now = rhohat0 + period * rhohatdot_last;
	}

	if (num >= 2)
	{
		xhat_last_last = y_last_last + rhohat_last_last * tau1_last_last;
		xhat_last = y_last + rhohat_last * tau1_last;
		xhatdot_last_last = (xhat_last - xhat_last_last) / period;

		rhohatdot_last = - v1_last + k * tau23_last.dot(xhatdot_last_last);

		rhohat_now = rhohat_last + period * rhohatdot_last;
	}

	rhohat_last_last = rhohat_last;
	rhohat_last = rhohat_now;

	y_last_last = y_last;
	y_last = y_now;

	tau1_last_last = tau1_last;
	tau1_last = tau1_now;

	tau23_last = tau23_now;

	v1_last = v1_now;

	num++;

	return rhohat_now;
}

void print(const DroneState &state)
{
	PX4_INFO("here is quadrotor state");
	printf("\tposition:     %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.pos(0)),
		static_cast<double>(state.pos(1)),
		static_cast<double>(state.pos(2)));
	printf("\tvelocity:     %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.vel(0)),
		static_cast<double>(state.vel(1)),
		static_cast<double>(state.vel(2)));
	printf("\tacceleration: %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.acc(0)),
		static_cast<double>(state.acc(1)),
		static_cast<double>(state.acc(2)));
	printf("\tyaw:          %7.3f (deg)\n", static_cast<double>(state.yaw) * RAD2DEG);
}

void print(const CircumState &state)
{
	PX4_INFO("here is circumnavigation state");
	printf("\ttime:   %10.6f (s)\n", static_cast<double>(state.time));
	printf("\trho:    %7.3f (m)\n", static_cast<double>(state.rho));
	printf("\trhohat: %7.3f (m)\n", static_cast<double>(state.rhohat));
	printf("\ttau1:   %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.tau1(0)),
		static_cast<double>(state.tau1(1)),
		static_cast<double>(state.tau1(2)));
	printf("\ttau2:   %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.tau2(0)),
		static_cast<double>(state.tau2(1)),
		static_cast<double>(state.tau2(2)));
	printf("\ttau3:   %7.3f, %7.3f, %7.3f (m)\n",
		static_cast<double>(state.tau3(0)),
		static_cast<double>(state.tau3(1)),
		static_cast<double>(state.tau3(2)));
	printf("\tv1:     %7.3f (m/s)\n", static_cast<double>(state.v1));
	printf("\tv2:     %7.3f (m/s)\n", static_cast<double>(state.v2));
	printf("\tv3:     %7.3f (m/s)\n", static_cast<double>(state.v3));
	printf("\tomega2: %7.3f (m/s^2)\n", static_cast<double>(state.omega2));
	printf("\tomega3: %7.3f (m/s^2)\n", static_cast<double>(state.omega3));
	printf("\ttheta:  %7.3f (deg)\n", static_cast<double>(state.theta));
	printf("\tl:      %7.3f (m)\n", static_cast<double>(state.l));
}

void recordScalarToFile(float time, float variable, std::string filename)
{
	mkdir(figure_dir_name.c_str(), 0777);
	filename = figure_dir_name + "/" + filename;
	std::ofstream file;
	file.open(filename, time < 0.000001f ? std::ios::ate : std::ios::app);
	file << std::left << std::setw(20) << time
	     << std::left << std::setw(20) << variable << std::endl;
	file.close();
}

void recordVectorToFile(float time, matrix::Vector3f variable, std::string filename)
{
	mkdir(figure_dir_name.c_str(), 0777);
	filename = figure_dir_name + "/" + filename;
	std::ofstream file;
	file.open(filename, time < 0.000001f ? std::ios::ate : std::ios::app);
	file << std::left << std::setw(20) << variable(0)
	     << std::left << std::setw(20) << variable(1)
	     << std::left << std::setw(20) << variable(2) << std::endl;
	file.close();
}

} // namespace fas_3d_boc
