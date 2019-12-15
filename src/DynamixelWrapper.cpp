//
//	DynamixelWrapper.cpp
//
//#include "dynamixel_sdk.h"
#include "DynamixelWrapper.hpp"
#include <iostream>
using namespace std;
using namespace dynamixel;

// #define DEBUG

//
//	DynamixelServo methods 
//

int DynamixelServo::torque_enable() {
	cout << "torque_enable " << +id << "\n";
	return dxlnet->write1b(id,
		DynamixelNetwork::ADDR_TORQUE_ENABLE, DynamixelNetwork::TORQUE_ENABLE);
}

int DynamixelServo::torque_disable() {
	cout << "torque_disable " << +id << "\n";
	return dxlnet->write1b(id,
		DynamixelNetwork::ADDR_TORQUE_ENABLE, DynamixelNetwork::TORQUE_DISABLE);
}

int DynamixelServo::led(uint8_t r, uint8_t g, uint8_t b) {
	cout << "LED " << +r << ", " << +g << ", " << +b << "\n";
	dxlnet->write1b(id,	DynamixelNetwork::ADDR_LED_RED, r);
	dxlnet->write1b(id,	DynamixelNetwork::ADDR_LED_GREEN, g);
	dxlnet->write1b(id,	DynamixelNetwork::ADDR_LED_BLUE, b);
	return 0;
}

int DynamixelServo::position_i_gain(uint16_t gain) {
	cout << "id:" << +id << " position i gain:" << gain << "\n";
	return dxlnet->write2b(id,	DynamixelNetwork::ADDR_POSITION_I_GAIN, gain);
}

int DynamixelServo::position_p_gain(uint16_t gain) {
	cout << "id:" << +id << " position p gain:" << gain << "\n";
	return dxlnet->write2b(id,	DynamixelNetwork::ADDR_POSITION_P_GAIN, gain);
}

int DynamixelServo::profile_acceleration(uint32_t acceleration) {
	cout << "id:" << +id << " profile acceleration:" << acceleration << "\n";
	return dxlnet->write4b(id,	DynamixelNetwork::ADDR_PROFILE_ACCELERATION, acceleration);
}

int DynamixelServo::profile_velocity(uint32_t velocity) {
	cout << "id:" << +id << " profile velocity:" << velocity << "\n";
	return dxlnet->write4b(id,	DynamixelNetwork::ADDR_PROFILE_VELOCITY, velocity);
}

int DynamixelServo::goal_position(int32_t position) {
	// cout << "id:" << +id << " goal position:" << position << "\n";
	return dxlnet->write4b(id,	DynamixelNetwork::ADDR_GOAL_POSITION, position);
}

int DynamixelServo::present_position() {
	// cout << "id:" << +id << " present position:" << "\n";
	return dxlnet->read4b(id,
		DynamixelNetwork::ADDR_PRESENT_POSITION, &position);
}


//
//	DynamixelNetwork methods
//

int DynamixelNetwork::write1b(uint8_t _id, enum ADDR _addr, uint8_t _command) {
	// cout << "wreite1b\n";
	if((comm_result = packetHandler->write1ByteTxRx ( portHandler, _id,
							_addr, _command, &error_type)) != COMM_SUCCESS) {
		cout << "id:" << +_id << " " << packetHandler->getTxRxResult(comm_result) << "\n";
	} else if( error_type != 0 ) {
		cout << "id:" << +_id << " " << packetHandler->getRxPacketError(error_type) << "\n";
	} else {
		// cout << "SUCCESS\n";// ok
	}
	return comm_result;
}

int DynamixelNetwork::write2b(uint8_t _id, enum ADDR _addr, uint16_t _command) {
	// cout << "wreite1b\n";
	if((comm_result = packetHandler->write2ByteTxRx ( portHandler, _id,
							_addr, _command, &error_type)) != COMM_SUCCESS) {
		cout << "id:" << +_id << " " << packetHandler->getTxRxResult(comm_result) << "\n";
	} else if( error_type != 0 ) {
		cout << "id:" << +_id << " " << packetHandler->getRxPacketError(error_type) << "\n";
	} else {
		// ok
	}
	return comm_result;
}

int DynamixelNetwork::write4b(uint8_t _id, enum ADDR _addr, uint32_t _command) {
	// cout << "write4b " << +_id << "ADDR:" << _addr << "COM:" << _command << "\n";
	if((comm_result = packetHandler->write4ByteTxRx ( portHandler, _id,
							_addr, _command, &error_type)) != COMM_SUCCESS) {
		cout << "id:" << +_id << " " << packetHandler->getTxRxResult(comm_result) << "\n";
	} else if( error_type != 0 ) {
		cout << "id:" << +_id << " " << packetHandler->getRxPacketError(error_type) << "\n";
	} else {
		// ok
	}
	return comm_result;
}

int DynamixelNetwork::read4b(uint8_t _id, enum ADDR _addr, int32_t* _data) {
	// cout << "write4b " << +_id << "ADDR:" << _addr << "COM:" << _command << "\n";
	if((comm_result = packetHandler->read4ByteTxRx (portHandler, _id,
						_addr, (uint32_t*)_data, &error_type)) != COMM_SUCCESS) {
		cout << "id:" << +_id << " " << packetHandler->getTxRxResult(comm_result) << "\n";
	} else if( error_type != 0 ) {
		cout << "id:" << +_id << " " << packetHandler->getRxPacketError(error_type) << "\n";
	} else {
		// ok
	}
	return comm_result;
}

DynamixelNetwork* DynamixelNetwork::netp = NULL;

void DynamixelNetwork::create(char const * const dev, PROTOCOL p, BAUDRATE_ID b) {
	if( netp == NULL ) {
		netp = new DynamixelNetwork(dev, p, b);
		cout << "CREATE " << dev << ", PROTOCOL:" << p << ", BAUDRATE_ID:" << b << "\n";
	}
}

void DynamixelNetwork::destroy() {
	if( netp != NULL ) {
		delete netp;
		netp = NULL;
	}
}

DynamixelNetwork::DynamixelNetwork(char const * const dev, PROTOCOL p, BAUDRATE_ID b):
	portHandler ( PortHandler::getPortHandler(dev) ),
	packetHandler ( PacketHandler::getPacketHandler(p) ),
	groupSyncWrite (
		new dynamixel::GroupSyncWrite
			(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION) ),
	groupSyncRead (
		new dynamixel::GroupSyncRead
			(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) ),
	comm_result(COMM_TX_FAIL)
{
	cout << "Protocol " << p << "\n";


#if 0
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite
	groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead
	groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
#endif

	cout << "Opening device: " << dev <<"\n";
	if( portHandler->openPort() ) {
		cout << "Succeeded to open port " << dev << "\n";
	} else {
		cout << "Failed to open port " << dev << "\n";
		exit(0);
	}

	cout << "Baudrate " << b << "\n";
	if( portHandler->setBaudRate(baudrate[b]) ) {
		cout << "Succeeded to set baudrate to " << b << "\n";
	} else {
		cout << "Failed to set baudrate.";
		exit(0);
	}
}


#ifdef DEBUG

#include <stdlib.h>
#include <unistd.h>

class CCV : public DynamixelRobotSystem {
  public:
	enum DXLID { DXLID_ROLL=1, DXLID_FORE, DXLID_REAR, DXLID_STEER };
	enum INDEX { ROLL=0, FORE, REAR, STEER };
	void setup();
	void run();
};

void CCV::setup() {
	svo[ROLL ]->profile_acceleration(1800.0F);
	svo[FORE ]->profile_acceleration(1800.0F);
	svo[REAR ]->profile_acceleration(1800.0F);
	svo[STEER]->profile_acceleration(1800.0F);
//	svo[STEER]->position_p_gain(25);
}

void CCV::run() {
	svo[ROLL ]->torque_enable();
	svo[FORE ]->torque_enable();
	svo[REAR ]->torque_enable();
//	svo[STEER]->goal_position_deg(0);

	float goal[] = { 0, 0, 0, 0 };
	dnet->sync_goal_position_deg(goal);

	usleep(1000000);

	while(1) {
		goal[ROLL ] = float(rand()%1400)/100 - 7;
		goal[FORE ] = float(rand()%2000)/100 -10;
		goal[REAR ] =-float(rand()%2000)/100 -10;
		goal[STEER] = float(rand()%4800)/100 -24;

		dnet->sync_goal_position_deg(goal);
//		svo[STEER]->goal_position_deg(theta);
//		svo[STEER]->led(rand()%256,rand()%256,rand()%256);

		usleep(10*1000);
	}
}

int main()
{
	DynamixelNetwork::create
		("/dev/ttyUSB0", DynamixelNetwork::PROTOCOL2, DynamixelNetwork::BAUDRATE_1M);
	DynamixelNetwork* dnet = DynamixelNetwork::getNetworkPointer();
	
	CCV* ccv = new CCV(dnet);
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_ROLL));
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_FORE));
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_REAR));
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_STEER));

	ccv->setup();
	ccv->run();

	dnet->destroy();
}

#endif	// DEBUG


