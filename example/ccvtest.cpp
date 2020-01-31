//
//	main.cpp
//
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <DynamixelWrapper.hpp>
using namespace std;
using namespace dynamixel;



class CCV : public DynamixelRobotSystem {
  public:
	enum DXLID { DXLID_ROLL=1, DXLID_FORE, DXLID_REAR, DXLID_STRR, DXLID_STRL };
	enum INDEX { ROLL=0, FORE, REAR, STRR, STRL };
	void setup();
	void run();
	CCV(DynamixelNetwork* _dnet):
		DynamixelRobotSystem(_dnet){}
};

void CCV::setup() {
	// disable anyway, for safety
	torque_disable();
	profile_acceleration(1000.0F);
//	svo[FORE]->position_p_gain(0);
	torque_enable();
	float goal[] = { 0, 0, 0, 0, 0 };
	goal_position_deg(goal);
	//sync_goal_position_deg(goal);
	usleep(3000*1000);
#if 0
	goal[FORE] = 10;	// lift up
	goal[REAR] = 10;	// lift up
	sync_goal_position_deg(goal);
	usleep(3000*1000);

	goal[FORE] = 0;		// calm down
	goal[REAR] = 0;		// calm down
	sync_goal_position_deg(goal);
	usleep(3000*1000);
#endif

}

void CCV::run() {

	for(int step=0; step<7; step++) {
		svo[ROLL]->led(rand()%256,rand()%256,rand()%256);

		cout << "Step: " << step << "\n";

		
		if(step == 1) {
			float goal[] = { 5,0,0,0,0 };
			goal_position_deg(goal);	// roll right
		} 

		if(step == 2) {
			float goal[] = { 0,0,0,0,0 };
			goal_position_deg(goal);	// roll right
		} 
		
		if(step == 3) {
			float goal[] = { -7,0,0,0,0 };
			goal_position_deg(goal);	// roll right
		}
		if(step == 4) {
			float goal[] = { 0,0,0,0,0 };
			goal_position_deg(goal);	// roll right
		} 
		
		if(step == 5) {
			float goal[] = { 9,0,0,0,0 };
			goal_position_deg(goal);	// roll right
		} 
		
		if(step == 6) {
			float goal[] = { 0,10,-10,0,0 };
			goal_position_deg(goal);	// roll right
		}



		float alpha = float(rand()%4801)/100 -24;	// -24 < alpha < 24
		cout << "Goal position: " << alpha << "\n";
		float goal[] = { 0,10,-10,alpha,alpha};
		goal_position_deg(goal);	// roll right

		for(int i=0; i<100; i++){
			float cp = svo[STRR]->present_position_deg();
			cout << "present position: " << cp << "\n";
			
			if( abs(alpha-cp) < 1 ) break;
			usleep(100000);
		}	

		usleep(3000000);
	}

	float goal[] = { 0,0,0,0,0 };
	goal_position_deg(goal);
	usleep(1000000);
}

int main()
{
	DynamixelNetwork::create
		("/dev/ttyUSB0", DynamixelNetwork::PROTOCOL2, DynamixelNetwork::BAUDRATE_4M);
	DynamixelNetwork* dnet = DynamixelNetwork::getNetworkPointer();
	
	CCV* ccv = new CCV(dnet);
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_ROLL));
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_FORE));
	ccv->add(new Dynamixel_H54P(dnet, CCV::DXLID_REAR));
	ccv->add(new Dynamixel_H42P(dnet, CCV::DXLID_STRR));
	ccv->add(new Dynamixel_H42P(dnet, CCV::DXLID_STRL));

	ccv->setup();
	ccv->run();

	dnet->destroy();
}


