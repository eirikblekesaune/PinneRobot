#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>	

#include <OSCBundle.h>
#include <OSCBoards.h>

#include <PinneRobot.h>


EthernetUDP Udp;
byte mac[] = {	
	0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // you can find this written on the board of some Arduino Ethernets or shields

IPAddress ip(196, 168, 1, 200);
unsigned int inPort = 8888;
IPAddress outIp(192, 168, 1, 100);
unsigned int outPort = 57120;

PinneRobot *robot;
PinneComm *comm;

void setup() {
	robot = new PinneRobot();
	comm = new PinneComm(robot, ip, inPort, outIp, outPort);
	Serial.begin(9600);
	delay(2000);
	Ethernet.begin(mac,ip);
	Udp.begin(inPort);
}

void routeMotorMsg(OSCMessage &msg, int addressOffset) {
	int matchAddressOffset;
	matchAddressOffset = msg.match("/left", addressOffset);
	if(matchAddressOffset > 0) {
		Serial.println("OSC msg: /motor/left");
		robot->leftMotor->RouteOSCMessage(msg, matchAddressOffset);
		return;
	} 
	matchAddressOffset = msg.match("/right", addressOffset);
	if(matchAddressOffset > 0) {
		Serial.println("OSC msg: /motor/right");
		robot->rightMotor->RouteOSCMessage(msg, matchAddressOffset);
		return;
	}
}

//reads and dispatches the incoming message
void loop(){ 
	OSCBundle bundleIN;
	int size;

	if( (size = Udp.parsePacket())>0)
	{
		while(size--)
			bundleIN.fill(Udp.read());

		if(!bundleIN.hasError()) {
			bundleIN.route("/pinne", routeMotorMsg);
		}
	}
}



///////////////////////////////
/*
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>

#include "PinneAPI.h"
#include "PinneRobot.h"
#include "PinneAPIParser.h"

PinneRobot *robot;
PinneAPIParser *parser;
byte mac[] = {
0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10, 0, 0, 222);
EthernetUDP Udp;

unsigned int localPort = 8888;

extern const int LOOP_UPDATE_RATE = 10;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];	// buffer to hold incoming packet,
char ReplyBuffer[] = "ok baby baby";		// a string to send back


void msgReceive() {
OSCMessage msg;
int size;
if((size = Udp.parsePacket()) > 0) {
while(size--)
msg.fill(Udp.read());
if(!msg.hasError()) {
msg.route("/pinne", handlePinneMsg);
}
}
}

void handlePinneMsg(OSCMessage &msg) {
Serial.println("Got msg");
}

void setup()
{ 
Serial.begin(57600);
while(!Serial);
delay(100);

Ethernet.begin(mac, ip);
Udp.begin(localPort);

robot = new PinneRobot();
parser = new PinneAPIParser(robot);
robot->init();
}

void loop()
{
while(Serial.available() > 0)
{
parser->parseIncomingByte(Serial.read());
}
msgReceive();
robot->update();
delay(LOOP_UPDATE_RATE);
}
 */
