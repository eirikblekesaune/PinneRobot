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

void PinneComm::SendInfo() {
  OSCMessage *msg;
  if (initResult == validSettings) {
    msg = new OSCMessage("/pinne/info");
  } else {
    msg = new OSCMessage("/pinne/initFailed");
  }
  msg->add(_name.c_str());
  /* msg->add(_hostname.c_str()); */
  msg->add(_port);
  /* msg->add(_targetHostname.c_str()); */
  msg->add(_targetPort);
  _Udp.beginPacket(*_broadcastIp, _targetPort);
  msg->send(_Udp);
  _Udp.endPacket();
  msg->empty();
  free(msg);
}

void PinneComm::Reply(const char *) {}

void PinneComm::ReturnGetValue(command_t command, address_t address,
                               int value) {}

void PinneComm::ReturnQueryValue(command_t command, address_t address,
                                 OSCMessage &replyMsg) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(AddressMap.at(address));
  replyAddress.append("/");
  replyAddress.append(CommandMap.at(command));
  replyAddress.append("/:reply");
  replyMsg.setAddress(replyAddress.c_str());
  _Udp.beginPacket(*_targetIp, _targetPort);
  replyMsg.send(_Udp);
  _Udp.endPacket();
}

void PinneComm::ReturnQueryValue(command_t command, OSCMessage &replyMsg) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(CommandMap.at(command));
  replyAddress.append("/:reply");
  replyMsg.setAddress(replyAddress.c_str());
  _Udp.beginPacket(*_targetIp, _targetPort);
  replyMsg.send(_Udp);
  _Udp.endPacket();
}

bool PinneComm::HasQueryAddress(OSCMessage &msg, int initialOffset) {
  int offset;
  offset = msg.match("/:query", initialOffset);
  return offset > 0;
}

void PinneComm::NotifyMotorStateChange(motorState_t stateChange,
                                       address_t address) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(AddressMap.at(address));
  replyAddress.append("/stateChange");
  OSCMessage msg(replyAddress.c_str());
  String stateStr = String(MotorStateChangeMap.at(stateChange));
  msg.add(stateStr.c_str());
  _Udp.beginPacket(*_targetIp, _targetPort);
  msg.send(_Udp);
  _Udp.endPacket();
}

void PinneComm::NotifyTargetPositionMoverStateChange(
    targetPositionMoverState_t stateChange, address_t address) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(AddressMap.at(address));
  replyAddress.append("/goToTargetPosition/moverState");
  OSCMessage msg(replyAddress.c_str());
  String stateStr = String(TargetPositionMoverStateChangeMap.at(stateChange));
  msg.add(stateStr.c_str());
  SendOSCMessage(msg);
}

// FIXME: too little time for DRY
void PinneComm::SendTargetPositionMoverProgress(float progress, address_t address) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(AddressMap.at(address));
  replyAddress.append("/goToTargetPosition/moverProgress");
  OSCMessage msg(replyAddress.c_str());
  msg.add(progress);
  SendOSCMessage(msg);
}

// FIXME: too little time for DRY
void PinneComm::SendParkingProceduresState(parkingProcedureState_t state, address_t address) {
  String replyAddress = String();
  replyAddress.reserve(64);
  replyAddress.append("/pinne/");
  replyAddress.append(AddressMap.at(address));
  replyAddress.append("/goToParkingPosition/procedureState");
  OSCMessage msg(replyAddress.c_str());
  String stateStr = String(ParkingStateMap.at(state));
  msg.add(stateStr.c_str());
  SendOSCMessage(msg);
}

void PinneComm::DebugUnitPrint(address_t address, const char *) {}

void PinneComm::DebugUnitPrint(address_t address, int val) {}

void PinneComm::DebugPrint(int val) {}

void PinneComm::DebugPrint(float val) {}

void PinneComm::DebugPrint(const char *) {}

void PinneComm::SendOSCMessage(OSCMessage &msg) {
  _Udp.beginPacket(*_targetIp, _targetPort);
  msg.send(_Udp);
  _Udp.endPacket();
}

void PinneComm::DebugMessagePrint(command_t command, address_t address,
                                  setGet_t setGet, int value) {}

void PinneComm::msgReceive() {
  OSCMessage msg;
  int size;
  if ((size = _Udp.parsePacket()) > 0) {
    while (size--)
      msg.fill(_Udp.read());
    if (!msg.hasError()) {
      int offset = msg.match("/pinne", 0);
      if (offset) {
        _robot->routeOSC(msg, offset);
      } else {
        if(msg.match("/hello")) {
          SendInfo();
        }
        Serial.println("Unmatched OSc message");
      }
    }
  }
}
