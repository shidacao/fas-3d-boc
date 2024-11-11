/*
*  abbreviation
*  veh:  vehicle
*  pos:  position
*  vel:  velocity
*  acc:  acceleration
*  traj: trajectory
*  sp:   setpoint
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

#include <iostream>/////////////////

#define RAD2DEG  (180.0 / M_PI)
#define RAD2DEGf (180.0f / M_PIf)

using namespace time_literals;

const float period = 0.05f;
const hrt_abstime period_abstime = 50_ms;

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

		std::cout << "it: " << k * tau23_last.dot(xhatdot_last_last) << std::endl;

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

matrix::Vector3f getAcc(matrix::Vector3f y, matrix::Vector3f ydot)
{
	const matrix::Vector3f x{3,0,-6};
	const matrix::Vector3f n{0,0,1};

	const float arho0 = 1, arho1 = 1, av2 = 1, al0 = 1, al1 = 1, d = 3, alpha = 2;
	float  v1, v2, v3, omega2, omega3, rho, theta, l, ldot, erhohat, ev2, rhohat;
	matrix::Vector3f tau1, tau2, tau3, tau23, f, w, yddot;
	matrix::Matrix<float,3,3> Binv, R, Temp;

	rho = (x - y).norm();

	std::cout << "rho: " << rho << std::endl;

	tau1 = (x - y) / (x - y).norm();
	tau2 = n.cross(tau1) / n.cross(tau1).norm();
	tau3 = tau1.cross(tau2);

	std::cout << "tau1: " << tau1(0) << ' ' << tau1(1) << ' ' << tau1(2) << std::endl;
	std::cout << "tau2: " << tau2(0) << ' ' << tau2(1) << ' ' << tau2(2) << std::endl;
	std::cout << "tau3: " << tau3(0) << ' ' << tau3(1) << ' ' << tau3(2) << std::endl;

	v1 = tau1.dot(ydot);
	v2 = tau2.dot(ydot);
	v3 = tau3.dot(ydot);

	std::cout << "v1: " << v1 << std::endl;
	std::cout << "v2: " << v2 << std::endl;
	std::cout << "v3: " << v3 << std::endl;

	omega2 = v2 / rho;
	omega3 = v3 / rho;

	std::cout << "omega2: " << omega2 << std::endl;
	std::cout << "omega3: " << omega3 << std::endl;

	theta = std::acos(n.dot(tau1)) - M_PIf  / 2.0f;

	std::cout << "theta: " << theta * RAD2DEGf << " degree" << std::endl;

	l = rho * theta;
	static float l_last = l;

	std::cout << "l: " << l << std::endl;

	tau23 = std::sqrt(v2 * v2 + v3 * v3) < 0.2f ? matrix::Vector3f{0,0,0} :
		(v2 * tau2 + v3 * tau3) / std::sqrt(v2 * v2 + v3 * v3);

	rhohat = getRhohat(y, tau1, tau23, v1);

	std::cout << "rhohat: " << rhohat << std::endl;

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

	return yddot;
}

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
		if (is_armed && has_changed_to_offboard) break;

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

	PX4_INFO("armed and switched to offboard mode");

	typedef enum _TaskState{BEGIN, TAKEOFF, HOLD, DO} TaskState;
	TaskState task_state = TaskState::BEGIN;

	while (true)
	{
		veh_local_pos_sub.update(&veh_local_pos_msg);

		std::printf("\n");
		PX4_INFO("position:     %7.3f, %7.3f, %7.3f", (double)veh_local_pos_msg.x, (double)veh_local_pos_msg.y, (double)veh_local_pos_msg.z);
		PX4_INFO("velocity:     %7.3f, %7.3f, %7.3f", (double)veh_local_pos_msg.vx, (double)veh_local_pos_msg.vy, (double)veh_local_pos_msg.vz);
		PX4_INFO("acceleration: %7.3f, %7.3f, %7.3f", (double)veh_local_pos_msg.ax, (double)veh_local_pos_msg.ay, (double)veh_local_pos_msg.az);
		PX4_INFO("yaw:          %7.3f degree", static_cast<double>(veh_local_pos_msg.heading) * RAD2DEG);
		std::printf("\n");

		switch (task_state)
		{
			case TaskState::BEGIN:
			{
				const float begin_x = veh_local_pos_msg.x;
				const float begin_y = veh_local_pos_msg.y;
				const float begin_z = veh_local_pos_msg.z;

				traj_sp_msg.position[0] = begin_x;
				traj_sp_msg.position[1] = begin_y;
				traj_sp_msg.position[2] = begin_z - 4.0f;
				offboard_control_mode_msg.position = true;

				task_state = TaskState::TAKEOFF;// BEGIN state exists only once
				PX4_INFO("task begins");
				break;
			}
			case TaskState::TAKEOFF:
			{
				PX4_INFO("takeing off, now position is (%.3f, %.3f, %.3f)",
					 (double)veh_local_pos_msg.x, (double)veh_local_pos_msg.y, (double)veh_local_pos_msg.z);

				bool is_takeoff_finished;
				is_takeoff_finished =
				matrix::Vector3f{veh_local_pos_msg.x - traj_sp_msg.position[0],
				     veh_local_pos_msg.y - traj_sp_msg.position[1],
				     veh_local_pos_msg.z - traj_sp_msg.position[2]}.norm() < 0.2f; // 20cm
				if (is_takeoff_finished)
				{
					task_state = TaskState::HOLD;
					PX4_INFO("takeoff finished, now start to holding");
				}
				break;
			}
			case TaskState::HOLD:
			{
				static const hrt_abstime hold_start_time = hrt_absolute_time();
				hrt_abstime hold_time = hrt_absolute_time() - hold_start_time;
				if (hold_time > 5_s)
					task_state = TaskState::DO;
				else
					PX4_INFO("holding for %.3fs", static_cast<double>(hold_time) / 1000000.0);
				break;
			}
			case TaskState::DO:
			{
				PX4_INFO("doing boc3d work");

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
				acc_tobe = getAcc(pos_measured, vel_measured);

				traj_sp_msg.acceleration[0] = acc_tobe(0);
				traj_sp_msg.acceleration[1] = acc_tobe(1);
				traj_sp_msg.acceleration[2] = acc_tobe(2);

				traj_sp_msg.yaw = 0.0f / 180.0f * M_PIf;

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

extern "C" __EXPORT int fas_3d_boc_main(int argc, char *argv[])
{
	PX4_INFO("Star is started.");

	px4_task_spawn_cmd("star", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
			   1024, fas_3d_boc_run, nullptr);

	return 0;
}


