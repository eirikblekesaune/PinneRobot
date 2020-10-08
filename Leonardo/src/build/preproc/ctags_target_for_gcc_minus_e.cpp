# 1 "/home/eirik/git/eirikblekesaune/PinneRobot/src/PinneRobot/L293Driver.cpp"
# 2 "/home/eirik/git/eirikblekesaune/PinneRobot/src/PinneRobot/L293Driver.cpp" 2

const int L293Driver::SPEED_MIN = 0;
const int L293Driver::SPEED_MAX = 512;

L293Driver::L293Driver(unsigned char INA, unsigned char INB, unsigned char PWM) :
 _INA(INA),
 _INB(INB),
 _PWM(PWM)
{
}
void L293Driver::init()
{
 pinMode(_INA, 1);
 pinMode(_INB, 1);
 pinMode(_PWM, 1);
 TCCR3A = 0b10000000;
 TCCR3B = 0b00010001;
 ICR3 = SPEED_MAX;
 SetDirection(0);
 SetSpeed(0);
}

void L293Driver::SetSpeed(speed_t speed)
{
 if (speed < SPEED_MIN)
  speed = SPEED_MIN;
 if (speed > SPEED_MAX) // Max PWM dutycycle
  speed = SPEED_MAX;
 _speed = speed;
 OCR3A = _speed;
 UpdateDirection();
}

void L293Driver::SetDirection(int direction)
{
 if(direction < 0)
  direction = 0;
 if(direction > 1)
  direction = 1;
 _direction = direction;
 UpdateDirection();
}

void L293Driver::UpdateDirection()
{
 if (!_direction)
 {
  digitalWrite(_INA, 0);
  digitalWrite(_INB, 1);
 }
 else
 {
  digitalWrite(_INA, 1);
  digitalWrite(_INB, 0);
 }
}

void L293Driver::SetBrake(speed_t brake)
{
 _brake = brake;
}
# 1 "/home/eirik/git/eirikblekesaune/PinneRobot/src/PinneRobot/ThothRobot.ino"


# 4 "/home/eirik/git/eirikblekesaune/PinneRobot/src/PinneRobot/ThothRobot.ino" 2
# 5 "/home/eirik/git/eirikblekesaune/PinneRobot/src/PinneRobot/ThothRobot.ino" 2

PinneRobot *robot;
PinneAPIParser *parser;

extern const int LOOP_UPDATE_RATE = 10;

void setup()
{
  Serial1.begin(57600);
  while(!Serial1);
  delay(100);
  robot = new PinneRobot();
  parser = new PinneAPIParser(robot);
  robot->init();
}

void loop()
{
  while(Serial1.available() > 0)
  {
    parser->parseIncomingByte(Serial1.read());
  }
  robot->update();
  //delay(LOOP_UPDATE_RATE);
}
