#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <OSCBundle.h>
#include <OSCBoards.h>
#include <SD.h>
#include <SPI.h>
#include <PinneComm.h>
#include <PinneRobot.h>
#include <EEPROM.h>

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
bool nocard = true;
int readSettingsFromSDCardPin = 32;

PinneComm *comm;
PinneRobot *robot;

bool readSettingsFromSDCard(PinneSettings *settings) {
	bool didLoad = false;
	if(SD.begin(BUILTIN_SDCARD)) {
		File settings_file;
		Serial.println("Has SD card");
		settings_file = SD.open("pinne.txt", FILE_READ);
		if(settings_file) {
			settings->name = readLineFromFile(settings_file, 0);
			settings->hostname = readLineFromFile(settings_file, 1);
			settings->port = readLineFromFile(settings_file, 2).toInt();
			settings->targetHostname = readLineFromFile(settings_file, 3);
			settings->targetPort = readLineFromFile(settings_file, 4).toInt();
			didLoad = true;
		}
	}
	return didLoad;
}

void writeSettingsToEEPROM(PinneSettings &settings) {
	PinneSettingsRaw rawSettings;
	settings.name.toCharArray(rawSettings.name, 16);
	settings.hostname.toCharArray(rawSettings.hostname, 16);
	rawSettings.port = settings.port;
	settings.targetHostname.toCharArray(rawSettings.targetHostname, 16);
	rawSettings.targetPort = settings.targetPort;
	EEPROM.put(0, rawSettings);
	Serial.println("wrote settings to EEPROM");
}

bool readSettingsFromEEPROM(PinneSettings *settings) {
	PinneSettingsRaw rawSettings;
	EEPROM.get(0, rawSettings);
	settings->name = String(rawSettings.name);
	settings->hostname = String(rawSettings.hostname);
	settings->port = rawSettings.port;
	settings->targetHostname = String(rawSettings.targetHostname);
	settings->targetPort = rawSettings.targetPort;
	return true;
}

void setup()
{ 
	delay(100);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(readSettingsFromSDCardPin, INPUT_PULLUP);
	analogWriteResolution(12);
	/* Serial.begin(57600); */
	delay(1000);
	/* while(!Serial); */
	Serial.println("Hello my friend~!");
	//if an SD card is inserted we will read the settings from the inserted SD card
	//from the file 'pinne.txt'.
	//If not SD card inserted we read the settings from the EEPROM
	bool settingsLoaded = false;
	PinneSettings settings;
	Serial.println("Feeling is a nice!");
	if(SD.begin(BUILTIN_SDCARD)) {
		Serial.println("Attempt reading settings from SD card");
		settingsLoaded = readSettingsFromSDCard(&settings);
		Serial.println("Loaded settings from SD card");
	} else {
		Serial.println("Attempt reading settings from EEPROM");
		settingsLoaded = readSettingsFromEEPROM(&settings);
		Serial.println("Loaded settings from EEPROM");
	}
	Serial.print("settingsLoaded: ");
	Serial.println(settingsLoaded);
	if(settingsLoaded) {
		Serial.println(settings.name);
		Serial.println(settings.hostname);
		Serial.println(settings.port);
		Serial.println(settings.targetHostname);
		Serial.println(settings.targetPort);
		comm = new PinneComm(&settings);
		if(comm->initResult == comm->validSettings) {
			robot = new PinneRobot(comm);
			robot->init();
			comm->setRobot(robot);
			initSuccess = true;

			//if pin 32 is pulled LOW we will write the loaded settings
			//to the mcu EEPROM
			if(digitalRead(readSettingsFromSDCardPin) == LOW) {
				Serial.print("Writing settings to EEPROM");
				writeSettingsToEEPROM(settings);
			}
		} else {
			Serial.print("Invalid settings: ");
			Serial.print(comm->initResult);
			Serial.println();
		}
	}
	if(initSuccess) {
		blinkInterval = 1000;
	} else {
		blinkInterval = 200;
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
