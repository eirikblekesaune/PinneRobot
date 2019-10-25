
#include "PinneAPI.h"
#include "PinneRobot.h"
#include "PinneAPIParser.h"

PinneRobot *robot;
PinneAPIParser *parser;

extern const int LOOP_UPDATE_RATE = 10;

void setup()
{ 
  Serial1.begin(9600);
  while(!Serial1);
  delay(100);
  robot = new PinneRobot();
  parser = new PinneAPIParser(robot);
  robot->init();
}

void loop()
{
  if(Serial1.available())
  {
    parser->parseIncomingByte(Serial1.read());
  }
  robot->update();
  //delay(LOOP_UPDATE_RATE);
}
