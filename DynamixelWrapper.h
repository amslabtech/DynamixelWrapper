//
//	DynamixelWrapper.h
//
#ifndef _DYNAMIXELWRAPPER_H
#define _DYNAMIXELWRAPPER_H

#include "dynamixel_sdk.h"
#include <math.h>
#include <iostream>
using namespace std;
using namespace dynamixel;


class DynamixelNetwork {

  public:
	enum ADDR {
		ADDR_ID					= 7,
		ADDR_BAUDRATE			= 8,
		ADDR_OPERATING_MODE		= 11,
		ADDR_PROTOCOL_TYPE		= 13,

		// These are not able to control from this SDK
//		ADDR_CURRENT_LIMIT		= 38,
//		ADDR_ACCELERATION_LIMIT	= 40,
//		ADDR_VELOCITY_LIMIT		= 44,
//		ADDR_MAX_POSITION_LIMIT	= 48,
//		ADDR_MIN_POSITION_LIMIT	= 52,

		ADDR_TORQUE_ENABLE		= 512,
		ADDR_LED_RED			= 513,
		ADDR_LED_GREEN			= 514,
		ADDR_LED_BLUE			= 515,

		ADDR_VELOCITY_I_GAIN	= 524,
		ADDR_VELOCITY_P_GAIN	= 526,
		ADDR_POSITION_D_GAIN	= 528,
		ADDR_POSITION_I_GAIN	= 530,
		ADDR_POSITION_P_GAIN	= 532,

		ADDR_PROFILE_ACCELERATION= 556,
		ADDR_PROFILE_VELOCITY	= 560,
		ADDR_GOAL_POSITION		= 564,

		ADDR_PRESENT_POSITION	= 580,
		ADDR_PRESENT_INPUT_VOLTAGE = 592,
		ADDR_PRESENT_TEMPERATURE= 594
	};

	enum BAUDRATE_ID {
		BAUDRATE_9K6 = 0,
		BAUDRATE_57K6,
		BAUDRATE_115K2,
		BAUDRATE_1M,
		BAUDRATE_2M,
		BAUDRATE_3M,
		BAUDRATE_4M,
		BAUDRATE_4M5,
		BAUDRATE_6M,
		BAUDRATE_10M5,
	};

	uint32_t baudrate[10] = {
		9600, 57600, 115200, 1000000, 2000000, 3000000,
		4000000, 4500000, 6000000, 10500000 };

	enum OPERATING_MODE {
		MODE_CURRENT_CONTROL	= 0,
		MODE_VELOCITY_CONTROL	= 1,
		MODE_POSITION_CONTROL	= 3,
		MODE_EXTENDED_POSITION	= 4,
		MODE_PWM_CONTROL		=16
	};

	enum PROTOCOL {
		PROTOCOL2 = 2,
	};

	enum TORQUE {
		TORQUE_DISABLE=0, TORQUE_ENABLE=1
	};

	static DynamixelNetwork* getNetworkPointer() { return netp; }
	static void create(char const * const dev, PROTOCOL p, BAUDRATE_ID b);
	static void destroy();

	int write1b(uint8_t _id, enum ADDR _addr, uint8_t COMMAND);
	int write2b(uint8_t _id, enum ADDR _addr, uint16_t COMMAND);
	int write4b(uint8_t _id, enum ADDR _addr, uint32_t COMMAND);
	int read4b (uint8_t _id, enum ADDR _addr, int32_t* _data);

  protected:
	PortHandler*	portHandler;
	PacketHandler*	packetHandler;

	int comm_result;
	uint8_t error_type;

	// Singleton
	static DynamixelNetwork* netp;
	DynamixelNetwork(char const * const dev, PROTOCOL p, BAUDRATE_ID b);
};


class DynamixelServo {

  protected:
	uint8_t id;
	int32_t position;

	DynamixelNetwork* dxlnet = NULL;

  public:
	DynamixelServo(DynamixelNetwork* _net, uint8_t _id):id(_id),dxlnet(_net){}

//	virtual ~DynamixelServo();
	virtual void showID(void) { cout << "id:" << id << "\n"; }

	// Followings are not able to control from this SDK
	// int acceleration_limit(uint32_t);
	// int velocity_limit(uint32_t);
	// int max_position_limit(int32_t);
	// int min_position_limit(int32_t);

	// Then, followings just calculate and show max values
	virtual void max_position_limit_deg(float _theta) = 0;
	virtual void max_position_limit_rad(float _theta) = 0;
	virtual void min_position_limit_deg(float _theta) = 0;
	virtual void min_position_limit_rad(float _theta) = 0;

	int torque_enable();
	int torque_disable();

	int led(uint8_t, uint8_t, uint8_t);

	int position_i_gain(uint16_t);
	int position_p_gain(uint16_t);

	int profile_acceleration(uint32_t);
	int profile_acceleration(float _rpm2) {return profile_acceleration(uint32_t(_rpm2));}
	int profile_velocity(uint32_t);
	int profile_velocity(float _rpm) {return profile_velocity(uint32_t(_rpm*100)); }

	int goal_position(int32_t);
	virtual int goal_position_rad(float theta) = 0;
	virtual int goal_position_deg(float theta) = 0;

	int present_position();
	virtual float present_position_rad() = 0;
	virtual float present_position_deg() = 0;
};


class Dynamixel_H42P : public DynamixelServo {
  protected:
  public:
	enum CONSTANT {
		ACCELERATION_LIMIT	= 10867,
		VELOCITY_LIMIT		= 2600,
		MAX_POSITION_LIMIT	= 262931,
		MIN_POSITION_LIMIT	=-262931,
		MAX_POSITION		= 263187,	// 180 deg
	};

	Dynamixel_H42P(DynamixelNetwork* _net, uint8_t _id):DynamixelServo(_net,_id) {}
	void showID(void) { cout << "id:" << +id << " H42P\n"; }

	void max_position_limit_deg(float _theta) {
		int32_t limit = _theta * MAX_POSITION/180.0;
		cout << "max_position_limit: " << _theta << "[deg], " << limit << "\n";
	}
	void max_position_limit_rad(float _theta) {
		int32_t limit = _theta * MAX_POSITION/M_PI;
		cout << "max_position_limit: " << _theta << "[rad], " << limit << "\n";
	}

	void min_position_limit_deg(float _theta) {
		int32_t limit = _theta * MAX_POSITION/180.0;
		cout << "max_position_limit: " << _theta << "[deg], " << limit << "\n";
	}
	void min_position_limit_rad(float _theta) {
		int32_t limit = _theta * MAX_POSITION/M_PI;
		cout << "max_position_limit: " << _theta << "[rad], " << limit << "\n";
	}

	int goal_position_rad(float _theta) {
		int32_t goal = _theta * MAX_POSITION/M_PI;
		return goal_position(goal);
	}
	int goal_position_deg(float _theta) {
		int32_t goal = _theta * MAX_POSITION/180.0;
		return goal_position(goal);
	}


	float present_position_deg() {
		present_position();
		float p = DynamixelServo::position;
		p = p/MAX_POSITION*180.0;
		return p;
	}
	float present_position_rad() {
		present_position();
		float p= DynamixelServo::position;
		p= p/MAX_POSITION*M_PI;
		return p;
	}
};

class Dynamixel_H54P : public DynamixelServo {
  protected:
  public:
	enum CONSTANT {
		ACCELERATION_LIMIT	= 9982,
		VELOCITY_LIMIT		= 2900,
		MAX_POSITION_LIMIT	= 501433,
		MIN_POSITION_LIMIT	=-501433,
		MAX_POSITION		= 501923,
	};

	Dynamixel_H54P(DynamixelNetwork* _net, uint8_t _id):DynamixelServo(_net,_id) {}
	void showID(void) { cout << "id:" << +id << " H54P\n"; }

	void max_position_limit_deg(float _theta) {
		int32_t limit = _theta * MAX_POSITION/180.0;
		cout << "max_position_limit: " << _theta << "[deg], " << limit << "\n";
		// return min_position_limit(limit);
	}
	void max_position_limit_rad(float _theta) {
		int32_t limit = _theta * MAX_POSITION/M_PI;
		cout << "max_position_limit: " << _theta << "[rad], " << limit << "\n";
		// return min_position_limit(limit);
	}

	void min_position_limit_deg(float _theta) {
		int32_t limit = _theta * MAX_POSITION/180.0;
		cout << "max_position_limit: " << _theta << "[deg], " << limit << "\n";
		// return min_position_limit(limit);
	}
	void min_position_limit_rad(float _theta) {
		int32_t limit = _theta * MAX_POSITION/M_PI;
		cout << "max_position_limit: " << _theta << "[rad], " << limit << "\n";
		// return min_position_limit(limit);
	}

	int goal_position_rad(float _theta) {
		int32_t goal = _theta * MAX_POSITION/M_PI;
		return goal_position(goal);
	}
	int goal_position_deg(float _theta) {
		int32_t goal = _theta * MAX_POSITION/180.0;
		return goal_position(goal);
	}

	float present_position_deg() {
		present_position();
		float p = DynamixelServo::position;
		p = p/MAX_POSITION*180.0;
		return p;
	}
	float present_position_rad() {
		present_position();
		float p= DynamixelServo::position;
		p= p/MAX_POSITION*M_PI;
		return p;
	}
};

#define CAP 20
class DynamixelRobotSystem {
  protected:
	int size;
	DynamixelServo*	svo[CAP];

  public:
	DynamixelRobotSystem():size(0){}
	int add(DynamixelServo* s) {
		if(size<CAP) {
			cout << "add svo[" << size << "]\n";
			svo[size++] = s;
			return 0;
		} else {
			cout << "[ERROR: Number of servos exceeded " << size << ".\n";
			return -1;
		}
	}
	virtual void run() = 0;
};

#endif	// _DYNAMIXELWRAPPER_H


