#include <PinneComm.h>

PinneComm::PinneComm(PinneRobot *robot, IPAddress ip, int inPort, IPAddress outIp, int outPort) :
	_robot(robot),
	_ip(ip)
	_inPort(inPort)
	_outIp(outIp)
	_outPort(outPort)
{

}


void PinneComm::Reply(const char *) {}

void PinneComm::SendMsg(OSCMessage &msg) {}

void PinneComm::ReturnGetValue(command_t command, address_t address, int value) {}

void PinneComm::NotifyStateChange(stateChange_t stateChange, address_t address) {}

void PinneComm::DebugUnitPrint(address_t address, const char *) {}

void PinneComm::DebugUnitPrint(address_t address, int val) {}

void PinneComm::DebugPrint( int val ) {}

void PinneComm::DebugPrint( float val) {}

void PinneComm::DebugPrint( const char *) {}

void PinneComm::DebugMessagePrint(command_t command, address_t address, setGet_t setGet, int value) {}
