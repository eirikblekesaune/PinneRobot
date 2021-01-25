#include <PinneComm.h>

PinneComm::PinneComm(PinneSettings *settings) {
  Ethernet.begin(mac, *ip);
  Udp.begin(inPort);
  delay(200);
  Udp.beginPacket(*outIp, outPort);
  hello.send(Udp);
  Udp.endPacket();
  hello.empty();
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

void PinneComm::msgReceive() {
  OSCMessage msg;
  int size;
  if ((size = Udp.parsePacket()) > 0) {
    while (size--)
      msg.fill(Udp.read());
    if (!msg.hasError()) {
      msg.route("/pinne", handlePinneMsg);
    }
  }
}

void PinneComm::handlePinneMsg(OSCMessage &msg) {
  OSCMessage reply("/helloMyeLove");
  reply.add(99);
  reply.add(88.88888);
  reply.add("hello");
  Udp.beginPacket(*outIp, outPort);
  reply.send(Udp);
  Udp.endPacket();
  reply.empty();
}
