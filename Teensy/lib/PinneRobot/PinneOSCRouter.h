#ifndef PINNE_OSC_ROUTER_H
#define PINNE_OSC_ROUTER_H
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>	
#include <OSCBundle.h>
#include <OSCMessage.h>


class PinneOSCRouter {
	public:
		PinneOSCRouter(PinneRobot *robot);
		void RouteMsg(OSCBundle &bundle);

	private:
		PinneRobot *_robot;

}



#endif
