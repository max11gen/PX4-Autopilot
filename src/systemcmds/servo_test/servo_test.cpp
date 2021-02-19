/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Holger Steinhaus <hsteinhaus@gmx.de>
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
 * @file
 *
 * Tool to test servo reactions to flight control inputs
 *
 */

#include "servo_test.h"

extern "C" __EXPORT int servo_test_main(int argc, char *argv[]);

int ServoTest::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ServoTest::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test the servos' reaction to flight control input.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("servo_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("run", "Publish specific values to actuator_control_0 topic to see how the servos react to it");
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, -100, 100, "Roll input (-100...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, -100, 100, "Pitch input (-100...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('y', 0, -100, 100, "Yaw input (-100...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0,    0, 100, "Thrust input (0...100)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Return all servos to neutral position");

	return 0;
}

int ServoTest::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

int ServoTest::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("servo_test",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ServoTest *ServoTest::instantiate(int argc, char *argv[])
{
	int8_t roll_in{-128},  pitch_in{-128},  yaw_in{-128},  thrust_in{-128};
	float roll_out{0.}, pitch_out{0.}, yaw_out{0.}, thrust_out{0.};

	//////////////////
	// Read options //
	//////////////////
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool error_flag=false;
	while ((ch = px4_getopt(argc, argv, "r:p:y:t:", &myoptind, &myoptarg)) != -1) {
		switch (ch) {

		case 'r':
			roll_in = (int8_t)strtol(myoptarg, nullptr, 0);
			if(roll_in>-101 && roll_in<101) {
				roll_out=float(roll_in)/100.f;
			} else {
				print_usage("Roll value is invalid");
				error_flag=true;
			}
			break;
		case 'p':
			pitch_in = (int8_t)strtol(myoptarg, nullptr, 0);
			if(pitch_in>-101 && pitch_in<101) {
				pitch_out=float(pitch_in)/100.f;
			} else {
				print_usage("Pitch value is invalid");
				error_flag=true;
			}
			break;
		case 'y':
			yaw_in = (int8_t)strtol(myoptarg, nullptr, 0);
			if(yaw_in>-101 && yaw_in<101) {
				yaw_out=float(yaw_in)/100.f;
			} else {
				print_usage("Yaw value is invalid");
				error_flag=true;
			}
			break;
		case 't':
			thrust_in = (int8_t)strtol(myoptarg, nullptr, 0);
			if(thrust_in>-101 && thrust_in<101) {
				thrust_out=float(thrust_in)/100.f;
			} else {
				print_usage("thrust value is invalid");
				error_flag=true;
			}
			break;

		default:
			print_usage("No input provided or unrecognized flag");
			error_flag=true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	ServoTest *instance = new ServoTest(
		roll_out, pitch_out, yaw_out, thrust_out
		);
	/*
	if (myoptind >= 0 && myoptind < argc) {
		if (strcmp("run", argv[myoptind]) == 0) {
			servo_test(roll_out,pitch_out,yaw_out,thrust_out);
		} else if (strcmp("stop", argv[myoptind]) == 0) {
			servo_test(0.,0.,0.,0.);
		} else {
			usage(nullptr);
			return 0;
		}

	}
	*/
	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	PX4_INFO("Roll: %2.2f, Pitch: %2.2f, Yaw: %2.2f, thrust: %2.2f",(double)roll_out,(double)pitch_out,(double)yaw_out,(double)thrust_out);

	return instance;
}

ServoTest::ServoTest(float roll, float pitch, float yaw, float thrust)
	: ModuleParams(nullptr),
	vhcl_ang_accel_sub{ORB_ID(vehicle_angular_acceleration)},
	acturator_pub{ORB_ID(actuator_controls_0)},
	_roll(roll), _pitch(pitch), _yaw(yaw), _thrust(thrust)
{
}

void ServoTest::run()
{
	//int * pub_instance(nullptr);
	vehicle_angular_acceleration_s accels;
	actuator_controls_s actuators;
	actuators.control[actuator_controls_s::INDEX_ROLL]=_roll;
	actuators.control[actuator_controls_s::INDEX_PITCH]=_pitch;
	actuators.control[actuator_controls_s::INDEX_YAW]=_yaw;
	actuators.control[actuator_controls_s::INDEX_THROTTLE]=_thrust;
	actuators.control[actuator_controls_s::INDEX_FLAPS]=0;
	actuators.control[actuator_controls_s::INDEX_SPOILERS]=0;
	actuators.control[actuator_controls_s::INDEX_AIRBRAKES]=0;
	actuators.control[actuator_controls_s::INDEX_LANDING_GEAR]=0;
	actuators.timestamp = hrt_absolute_time();
	actuators.timestamp_sample= hrt_absolute_time();
	orb_advert_t actuator_pub = orb_advertise(ORB_ID(actuator_controls_0),&actuators);
	while (!should_exit()) {
		if(vhcl_ang_accel_sub.update(&accels)) {
			actuators.timestamp=hrt_absolute_time();
			actuators.timestamp_sample=hrt_absolute_time();
			orb_publish(ORB_ID(actuator_controls_0),actuator_pub,&actuators);
		}
	}
}

int servo_test_main(int argc, char *argv[])
{
	return ServoTest::main(argc, argv);
}
