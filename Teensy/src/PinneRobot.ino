#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SD.h>
#include <SPI.h>
#include <PinneComm.h>
#include <PinneRobot.h>

const int LOOP_UPDATE_RATE = 10;

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

PinneComm *comm;
PinneRobot *robot;

void setup()
{ 
	delay(100);
	pinMode(LED_BUILTIN, OUTPUT);
	analogWriteResolution(12);
	Serial.begin(57600);
	/* while(!Serial); */
	delay(1000);
	OSCMessage hello("/booted");
	if(SD.begin(BUILTIN_SDCARD)) {
		hello.add("yes");
		File settings_file;
		settings_file = SD.open("pinne.txt", FILE_READ);
		if(settings_file) {
			PinneSettings settings = {
				readLineFromFile(settings_file, 0),
				readLineFromFile(settings_file, 1),
				readLineFromFile(settings_file, 2).toInt(),
				readLineFromFile(settings_file, 3),
				readLineFromFile(settings_file, 4).toInt()
			};
			comm = new PinneComm(&settings);
			if(comm->initResult == comm->validSettings) {
				robot = new PinneRobot(comm);
				robot->init();
				comm->setRobot(robot);
				initSuccess = true;
			} else {
				Serial.print("Invalid settings: ");
				Serial.print(comm->initResult);
				Serial.println();
			}
		}
	}
	if(initSuccess) {
		blinkInterval = 1000;
	} else {
		blinkInterval = 100;
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
		comm->msgReceive();
		robot->update();
		/* delay(LOOP_UPDATE_RATE); */
	}
}
