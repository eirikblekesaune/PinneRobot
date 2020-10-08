~pinner = [
	SerialPort("/dev/tty.usbserial-FT3EET44B", 9600, crtscts:true,xonxoff:true),// pinne en
	SerialPort("/dev/tty.usbserial-FT3EET4IA", 9600, crtscts:true,xonxoff:true),// pinne to
	SerialPort("/dev/tty.usbserial-FT3EET4IB", 9600, crtscts:true,xonxoff:true),// pinne tre
	SerialPort("/dev/tty.usbserial-FT3EET52A", 9600, crtscts:true,xonxoff:true)// pinne fire
];

~makeDataBytes = {|val|
	[(val >> 7) & 0x7F, val & 0x7F];
};
~parseDataBytes = {|bytes|
	(bytes[0] << 7).bitOr(bytes[1]);
};
~addressMasks = TwoWayIdentityDictionary[
	\left -> 2r00000000,
	\right -> 2r00010000,
	\rotation -> 2r00100000,
	\global -> 2r00110000
];
~commandMasks = TwoWayIdentityDictionary[
	\stop -> 2r0000,
	\speed -> 2r0001,
	\direction -> 2r0010,
	\targetPosition -> 2r0011,
	\currentPosition -> 2r0100,
	\brake -> 2r0101,
	\stateChange -> 2r0110,
	\info -> 2r0111,
	\minPosition -> 2r1000,
	\maxPosition -> 2r1001,
	\goToParkingPosition -> 2r1010,
	\goToTargetPosition -> 2r1011,
	\goToSpeedRampUp -> 2r1100,//argument is ramp up percent of halfway point
	\goToSpeedRampDown -> 2r1101,//ramp down time will take effect after halfway point
	\goToSpeedScaling -> 2r1110
];
~setGetMasks = TwoWayIdentityDictionary[
	\set -> 2r00000000,
	\get -> 2r01000000
];
~stateChanges = TwoWayIdentityDictionary[
	\stopped ->   2r00000000,
	\goingDown ->   2r00000001,
	\goingUp -> 2r00000010,
	\stoppedAtTarget -> 2r00000011,
	\goingToTarget -> 2r00000100,
	\blockedByTopSensor -> 2r00000101,
	\blockedBySlackSensor -> 2r00000110,
	\blockedByMinPosition -> 2r00000111,
	\blockedByMaxPosition -> 2r00001000,
	\blockedByAbsMinPosition -> 2r00001001,
	\motorDriverFault -> 2r00001010
];
~buildMessage = {arg address, setGet, command, value;
	var msg;
	msg = [
		0x80,
		~addressMasks[address],
		~setGetMasks[setGet],
		~commandMasks[command],
	].reduce(\bitOr).asArray;
	if(setGet == \set, {
		msg = msg ++ ~makeDataBytes.value(value);
	});
	msg;
};
OSCdef(\pinneResponder, {arg msg, time, addr, port;
	var index, motor, val, key;
	index = msg[1].asInteger - 1;
	motor = msg[2].asSymbol;
	key = msg[3].asSymbol;
	val = msg[4].asInteger;
	"pinne: num: % motor: % key: % val: %\n".postf(index, motor, key, val);
	~pinner[index].putAll(~buildMessage.value(motor, \set, key, val));
}, '/robot/pinne');

OSCdef(\pinneParking, {arg msg, time, addr, port;
	~pinner.do({arg item;
		item.putAll(~buildMessage.value(\global, \set, \goToParkingPosition, 0));
	});
}, '/robot/pinne/park');