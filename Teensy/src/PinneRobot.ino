/* /1* #include <Arduino.h> *1/ */
/* #include <Ethernet.h> */
/* #include <EthernetUdp.h> */
/* #include <SPI.h> */	

/* #include <OSCBundle.h> */
/* #include <OSCBoards.h> */

/* /1* #include <PinneRobot.h> *1/ */


/* EthernetUDP Udp; */
/* byte mac[] = { */	
/* 	0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED */
/* }; // you can find this written on the board of some Arduino Ethernets or shields */

/* IPAddress ip(196, 168, 38, 200); */
/* unsigned int inPort = 8888; */
/* IPAddress outIp(192, 168, 1, 100); */
/* unsigned int outPort = 57120; */

/* /1* PinneRobot *robot; *1/ */
/* /1* PinneComm *comm; *1/ */

/* void setup() { */
/* 	pinMode(LED_BUILTIN, OUTPUT); */
/* 	/1* robot = new PinneRobot(); *1/ */
/* 	/1* comm = new PinneComm(robot, ip, inPort, outIp, outPort); *1/ */
/* 	Serial.begin(9600); */
/* 	delay(2000); */
/* 	Ethernet.begin(mac,ip); */
/* 	Udp.begin(inPort); */
/* } */

/* /1* void routeMotorMsg(OSCMessage &msg, int addressOffset) { *1/ */
/* /1* 	int matchAddressOffset; *1/ */
/* /1* 	matchAddressOffset = msg.match("/left", addressOffset); *1/ */
/* /1* 	if(matchAddressOffset > 0) { *1/ */
/* /1* 		Serial.println("OSC msg: /motor/left"); *1/ */
/* /1* 		robot->leftMotor->RouteOSCMessage(msg, matchAddressOffset); *1/ */
/* /1* 		return; *1/ */
/* /1* 	} *1/ */ 
/* /1* 	matchAddressOffset = msg.match("/right", addressOffset); *1/ */
/* /1* 	if(matchAddressOffset > 0) { *1/ */
/* /1* 		Serial.println("OSC msg: /motor/right"); *1/ */
/* /1* 		robot->rightMotor->RouteOSCMessage(msg, matchAddressOffset); *1/ */
/* /1* 		return; *1/ */
/* /1* 	} *1/ */
/* /1* } *1/ */

/* //reads and dispatches the incoming message */
/* void loop(){ */ 
/* 	OSCBundle bundleIN; */
/* 	int size; */
/* 	/1* delay(200); *1/ */
/* 	/1* digitalWrite(LED_BUILTIN, HIGH); *1/ */
/* 	/1* delay(200); *1/ */
/* 	/1* digitalWrite(LED_BUILTIN, LOW); *1/ */

/* 	if( (size = Udp.parsePacket())>0) */
/* 	{ */
/* 		while(size--) */
/* 			bundleIN.fill(Udp.read()); */

/* 		if(!bundleIN.hasError()) { */
/* 			Serial.println("Got msg"); */
/* 			/1* bundleIN.route("/pinne", routeMotorMsg); *1/ */
/* 		} */
/* 	} */
/* } */



///////////////////////////////
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SD.h>
#include <SPI.h>
#include <PinneRobot.h>

byte mac[] = {
	0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
EthernetUDP Udp;
IPAddress *ip;
IPAddress *outIp;
unsigned int outPort;
unsigned int inPort;

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
	OSCMessage reply("/helloMyeLove");
	reply.add(99);
	reply.add(88.88888);
	reply.add("hello");
	Udp.beginPacket(*outIp, outPort);
	reply.send(Udp);
	Udp.endPacket();
	reply.empty();
}

String readLineFromFile(File& file, int lineNum) {
	file.seek(0); //rewind
	int l = 0;
	char c = file.read();
	while(l != lineNum) {
		if(c == '\n') {
			l++;
		}
		c = file.read();
	}
	String result;
	result += c;
	c = file.read();
	while(c != '\n') {
		result += c;
		c = file.read();
	}
	return result;
}

unsigned long blinkInterval;
unsigned long lastBlinkTime = 0;
int blinkValue = LOW;
bool initSuccess = false;

void setup()
{ 
	delay(100);
	pinMode(LED_BUILTIN, OUTPUT);

	OSCMessage hello("/booted");
	if(SD.begin(BUILTIN_SDCARD)) {
		hello.add("yes");
		File settings_file;
		settings_file = SD.open("pinne.txt", FILE_READ);
		if(settings_file) {
			hello.add("file_opened");
			ip = new IPAddress();
			ip->fromString(readLineFromFile(settings_file, 1).c_str());
			inPort = readLineFromFile(settings_file, 2).toInt();
			outIp = new IPAddress();
			outIp->fromString(readLineFromFile(settings_file, 3).c_str());
			outPort = readLineFromFile(settings_file, 4).toInt();
			hello.add(readLineFromFile(settings_file, 0).c_str());
			hello.add(readLineFromFile(settings_file, 1).c_str());
			hello.add(readLineFromFile(settings_file, 2).c_str());
			hello.add(readLineFromFile(settings_file, 3).c_str());
			hello.add(readLineFromFile(settings_file, 4).c_str());
			Ethernet.begin(mac, *ip);
			Udp.begin(inPort);
			delay(200);

			Udp.beginPacket(*outIp, outPort);
			hello.send(Udp);
			Udp.endPacket();
			hello.empty();
			initSuccess = true;
		}
	}

	if(!initSuccess) {
		blinkInterval = 100;
	} else {
		blinkInterval = 1000;
	}
}

void loop()
{
	//Heartbeat
	if((millis() - lastBlinkTime) > blinkInterval) {
		blinkValue = !blinkValue;
		digitalWrite(LED_BUILTIN, blinkValue);
		lastBlinkTime = millis();
	} 
	if(initSuccess) {
		msgReceive();
		delay(LOOP_UPDATE_RATE);
	}
}
