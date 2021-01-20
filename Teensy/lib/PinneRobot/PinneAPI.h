#ifndef PINNE_API_H
#define PINNE_API_H
#include <map>
#include <string>

static void Reply(const char* str)
{
	OSCMessage msg("/reply");
	msg.add(str);
	Udp.beginPacket(outIp, outPort);
	msg.send(Udp);
	Udp.endPacket();
	msg.empty();
}

static void SendMsg(OSCMessage &msg) {
	Udp.beginPacket(outIp, outPort);
	msg.send(Udp);
	Udp.endPacket();
}

static void ReturnGetValue(command_t command, address_t address, int value)
{
	std::string path = std::string("/");
	path.append(AddressMap.at(address));
	path.append("/");
	path.append(CommandMap.at(command));
	OSCMessage msg(path.c_str());
	msg.add(value);
	SendMsg(msg);
	msg.empty();
}

static void NotifyStateChange(stateChange_t stateChange, address_t address)
{
	std::string path = std::string("/");
	path.append(AddressMap.at(address));
	path.append("/");
	path.append(CommandMap.at(CMD_STATE_CHANGE));
	OSCMessage msg(path.c_str());
	msg.add(StateChangeMap.at(stateChange).c_str());
	SendMsg(msg);
	msg.empty();
}

static void DebugUnitPrint(address_t address, const char* val)
{
	std::string path = std::string("/");
	path.append(AddressMap.at(address));
	path.append("/");
	path.append(CommandMap.at(CMD_INFO));
	OSCMessage msg(path.c_str());
	msg.add(val);
	SendMsg(msg);
	msg.empty();
}

static void DebugUnitPrint(address_t address, int val)
{
	std::string path = std::string("/");
	path.append(AddressMap.at(address));
	path.append("/");
	path.append(CommandMap.at(CMD_INFO));
	OSCMessage msg(path.c_str());
	msg.add(val);
	SendMsg(msg);
	msg.empty();
}

static void DebugPrint( int val ) {
	OSCMessage msg("/debug");
	msg.add(val);
	SendMsg(msg);
	msg.empty();
}

static void DebugPrint( float val ) {
	OSCMessage msg("/debug");
	msg.add(val);
	SendMsg(msg);
	msg.empty();
}

static void DebugPrint(const char * val) {
	OSCMessage msg("/debug");
	msg.add(val);
	SendMsg(msg);
	msg.empty();
}

static void DebugMessagePrint(command_t cmd, address_t addr, setGet_t setGet, int value)
{
	OSCMessage msg("/debug");
	msg.add("cmd");
	msg.add(cmd);
	msg.add("addr");
	msg.add(addr);
	msg.add("setGet");
	msg.add(setGet);
	msg.add("value");
	msg.add(value);
	SendMsg(msg);
	msg.empty();
}

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(x) DebugPrint(x)
#define DEBUG_NL
#else
#define DEBUG_PRINT(x)
#define DEBUG_NL
#endif


#endif
