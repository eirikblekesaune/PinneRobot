#include <PinneComm.h>

PinneComm::PinneComm(PinneSettings *settings) {
  // get the MAC address from the Teensy registry,
  // as suggested in https://forum.pjrc.com/threads/60857-T4-1-Ethernet-Library
  uint32_t m1 = HW_OCOTP_MAC1;
  uint32_t m2 = HW_OCOTP_MAC0;
  _mac = new uint8_t[6];
  _mac[0] = (uint8_t)(m1 >> 8);
  _mac[1] = (uint8_t)(m1 >> 0);
  _mac[2] = (uint8_t)(m2 >> 24);
  _mac[3] = (uint8_t)(m2 >> 16);
  _mac[4] = (uint8_t)(m2 >> 8);
  _mac[5] = (uint8_t)(m2 >> 0);

  _name = settings->name;

  // construct IPAddress and port fields
  _ip = new IPAddress();
  if (!_ip->fromString(settings->hostname)) {
    initResult |= invalidHostname;
  }
  _port = settings->port;

  _targetIp = new IPAddress();
  if (!_targetIp->fromString(settings->targetHostname)) {
    initResult |= invalidTargetHostname;
  }
  _targetPort = settings->targetPort;
  _broadcastIp = new IPAddress(255, 255, 255, 255);

  // start Enthernet and Udp
  Ethernet.begin(_mac, *_ip);
  _Udp.begin(_port);
  // small delay to allow it to init
  delay(200);
  OSCMessage *msg;
  if (initResult == validSettings) {
    msg = new OSCMessage("/hello");
  } else {
    msg = new OSCMessage("/initFailed");
  }
  msg->add(settings->name.c_str());
  msg->add(settings->hostname.c_str());
  msg->add(settings->port);
  msg->add(settings->targetHostname.c_str());
  msg->add(settings->targetPort);
  _Udp.beginPacket(*_broadcastIp, _targetPort);
  msg->send(_Udp);
  _Udp.endPacket();
  msg->empty();
  free(msg);
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
  if ((size = _Udp.parsePacket()) > 0) {
    while (size--)
      msg.fill(_Udp.read());
    if (!msg.hasError()) {
      Serial.println("Got msg");
      /* msg.route("/pinne", this->handlePinneMsg); */
    }
  }
}

void PinneComm::handlePinneMsg(OSCMessage &msg) {
  /* OSCMessage reply("/helloMyeLove"); */
  /* reply.add(99); */
  /* reply.add(88.88888); */
  /* reply.add("hello"); */
  /* Udp.beginPacket(*outIp, outPort); */
  /* reply.send(Udp); */
  /* Udp.endPacket(); */
  /* reply.empty(); */
}