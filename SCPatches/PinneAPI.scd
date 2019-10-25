SerialPort.devices.do(_.postln)
~pinne1 = SerialPort('/dev/tty.usbserial-FT3EET44B', 2400, crtscts: true);
~pinne1.put(33)

~pinne2 = SerialPort('/dev/tty.usbserial-FT3EET4IA', 2400, crtscts: true);
~pinne2.put(33)

~pinne3 = SerialPort('/dev/tty.usbserial-FT3EET4IB', 2400, crtscts: true);
~pinne3.put(22)

~pinne4 = SerialPort('/dev/tty.usbserial-FT3EET52A', 2400, crtscts: true);
~pinne4.put(22)
SerialPort.devices
(
p = SerialPort("/dev/tty.usbserial-FT5QT6JNA", 9600, crtscts:true);
//p = SerialPort("/dev/tty.usbserial-FT5QT6TQA", 9600, crtscts:true,xonxoff:true);//EN
//p = SerialPort("/dev/tty.usbserial-FT3EET44B", 9600, crtscts:true,xonxoff:true);// pinne en
//p = SerialPort("/dev/tty.usbserial-FT3EET4IA", 9600, crtscts:true,xonxoff:true);// pinne to
//p = SerialPort("/dev/tty.usbserial-FT3EET4IB", 9600, crtscts:true,xonxoff:true);// pinne tre
//p = SerialPort("/dev/tty.usbserial-FT5QT6JNA", 9600, crtscts:true,xonxoff:true);// pinne fire
Tdef(\listenToSerial, {
	loop{
		var byte;
		byte = p.read;
		//byte.post;
		~pinneParser.putByte(byte);
		//p.read.asBinaryString.postln;
	}
}).play;
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
~pinneParser = (
	parserStates: [
		\waitingForCommandByte,
		\waitingForDataByte,
		\waitingForStateByte,
		\waitingForInfoByte
	],
	dataBytesReceived: 0,
	state: \waitingForCommandByte,
	currentCommand: \none,
	currentAddress: \none,
	currentSetGet: \none,
	valueBytes: Array.newClear(2),
	infoBytes: Array.new,
	putByte: {arg ev, byte;
		if(byte.bitAnd(128) > 0 and: {ev[\state] == \waitingForCommandByte}, {//if first bit is 1
			var command, nextParserState, address, setGet;
			//byte is a command byte
			nextParserState = \waitingForDataByte;
			command = ~commandMasks.getID(byte.bitAnd(2r00001111));//keep the lower 4 bits, and get command symbol
			address = ~addressMasks.getID(byte.bitAnd(2r00110000));//get address symbol
			setGet = ~setGetMasks.getID(byte.bitAnd(2r01000000));//get set or get command
			switch(command,
				\stop, {"Received stop command".postln},
				\stateChange, {
					//"Receive state change command".postln;
					nextParserState = \waitingForStateByte;
				},
				\speed, {"Received speed command".postln},
				\direction, {"Received direction command".postln},
				\targetPosition, {"Received targetPosition command".postln},
				\currentPosition, {"Receive currentPosition command".postln;},
				\brake, {"Received brake command".postln;},
				\minPosition, {"Receive min position command".postln;},
				\maxPosition, {"Receive max position command".postln;},
				\info, {
					nextParserState = \waitingForInfoByte;
				}
			);
			ev[\state] = nextParserState;
			ev[\currentCommand] = command;
			ev[\currentAddress] = address;
			ev[\currentSetGet] = setGet;
			}, {
				if(ev[\state] == \waitingForDataByte, {
					//"Data byte received".postln;
					switch(ev[\dataBytesReceived],
						0, {
							ev[\valueBytes][0] = byte;
							ev[\dataBytesReceived] = 1
						},
						1, {
							ev[\valueBytes][1] = byte;
							ev[\doCommand].value(ev);
						}
					)
				});
				if(ev[\state] == \waitingForStateByte, {
					//"State byte received".postln;
					ev[\valueBytes][0] = byte;
					ev[\doStateCommand].value(ev);
				});
				if(ev[\state] == \waitingForInfoByte, {
					if(byte != 4, {//4 is end of transimssion byte according to ASCII
						ev[\infoBytes] = ev[\infoBytes].add(byte);
						}, {
							"INFO: [%]: ".postf(ev[\currentAddress]);
							String.newFrom(ev[\infoBytes].collect(_.asAscii)).postln;
							ev[\infoBytes] = Array.new;
							ev[\reset].value(ev);
					});
				});
		});
	},
	doCommand: {|ev|
		"doing command: %, address: %, setGet: %, value: %\n".format(
			ev[\currentCommand], ev[\currentAddress], ev[\currentSetGet], ~parseDataBytes.value(ev[\valueBytes])
		).postln;
		ev[\reset].value(ev);
	},
	doStateCommand: {|ev|
		var state;
		state = ~stateChanges.getID(ev[\valueBytes][0]);
		"doing state command - address: % state: %\n".postf(ev[\currentAddress], state);
		ev[\reset].value(ev);
	},
	reset: {|ev|
		ev.use {
			~state = \waitingForCommandByte;
			~currentCommand = \none;
			~currentSetGet = \none;
			~currentAddres = \none;
			~dataBytesReceived = 0;
			~valueBytes.fill(nil);
		}
	},
	dump: {arg ev;
		ev.postln;
	}
);
)




p.putAll(~buildMessage.value(\left, \set, \goToTargetPosition, 10));
p.putAll(~buildMessage.value(\left, \set, \targetPosition, 150));
(
p.putAll(~buildMessage.value(\left, \set, \goToSpeedRampUp, 1000));
p.putAll(~buildMessage.value(\left, \get, \goToSpeedRampUp));
p.putAll(~buildMessage.value(\left, \set, \goToSpeedRampDown, 2000));
p.putAll(~buildMessage.value(\left, \get, \goToSpeedRampDown));
p.putAll(~buildMessage.value(\left, \set, \goToSpeedScaling, 500));
p.putAll(~buildMessage.value(\left, \get, \goToSpeedScaling));
)
(
p.putAll(~buildMessage.value(\right, \set, \goToSpeedRampUp, 10));
p.putAll(~buildMessage.value(\right, \get, \goToSpeedRampUp));
p.putAll(~buildMessage.value(\right, \set, \goToSpeedRampDown, 2000));
p.putAll(~buildMessage.value(\right, \get, \goToSpeedRampDown));
p.putAll(~buildMessage.value(\right, \set, \goToSpeedScaling, 99));
p.putAll(~buildMessage.value(\right, \get, \goToSpeedScaling));

p.putAll(~buildMessage.value(\right, \set, \targetPosition, 10));
p.putAll(~buildMessage.value(\right, \set, \goToTargetPosition, 0));
)

p.putAll(~buildMessage.value(\rotation, \set, \goToTargetPosition, 3000));
p.putAll(~buildMessage.value(\rotation, \set, \targetPosition, 650));
(
p.putAll(~buildMessage.value(\rotation, \set, \goToSpeedRampUp, 3000));
p.putAll(~buildMessage.value(\rotation, \get, \goToSpeedRampUp));
p.putAll(~buildMessage.value(\rotation, \set, \goToSpeedRampDown, 10));
p.putAll(~buildMessage.value(\rotation, \get, \goToSpeedRampDown));
p.putAll(~buildMessage.value(\rotation, \set, \goToSpeedScaling, 1000));
p.putAll(~buildMessage.value(\rotation, \get, \goToSpeedScaling));
p.putAll(~buildMessage.value(\rotation, \get, \speed));
)

p.putAll(~buildMessage.value(\right, \set, \goToSpeedRampDown, 20));
p.putAll(~buildMessage.value(\right, \get, \goToSpeedRampDown));

p.putAll(~buildMessage.value(\rotation, \set, \goToSpeedRampDown, 30));
p.putAll(~buildMessage.value(\rotation, \get, \goToSpeedRampDown));




p.putAll(~buildMessage.value(\rotation, \set, \goToTargetPosition, 0));
p.putAll(~buildMessage.value(\rotation, \set, \pidPValue, 11));
p.putAll(~buildMessage.value(\rotation, \set, \pidIValue, 22));
p.putAll(~buildMessage.value(\rotation, \set, \pidDValue, 33));
p.putAll(~buildMessage.value(\rotation, \get, \pidPValue));
p.putAll(~buildMessage.value(\rotation, \get, \pidIValue));
p.putAll(~buildMessage.value(\rotation, \get, \pidDValue));

(
p.putAll(~buildMessage.value(\right, \set, \targetPosition, 200));
p.putAll(~buildMessage.value(\right, \set, \goToTargetPosition, 0));
)
(
p.putAll(~buildMessage.value(\right, \set, \pidPValue, 3000));
p.putAll(~buildMessage.value(\right, \set, \pidIValue, 1));
p.putAll(~buildMessage.value(\right, \set, \pidDValue, 1000));
)
p.putAll(~buildMessage.value(\right, \get, \pidPValue));
p.putAll(~buildMessage.value(\right, \get, \pidIValue));
p.putAll(~buildMessage.value(\right, \get, \pidDValue));

(
p.putAll(~buildMessage.value(\left, \set, \speed, 100));
p.putAll(~buildMessage.value(\right, \set, \speed, 100));
p.putAll(~buildMessage.value(\left, \set, \speed, 0));
)
p.putAll(~buildMessage.value(\left, \set, \stop, 100));
p.putAll(~buildMessage.value(\left, \get, \stop));
(
var dir = 0;
p.putAll(~buildMessage.value(\left, \set, \direction, dir));
p.putAll(~buildMessage.value(\right, \set, \direction,dir));
)
(
p.putAll(~buildMessage.value(\left, \get, \currentPosition));
p.putAll(~buildMessage.value(\right, \get, \currentPosition));
)
(
p.putAll(~buildMessage.value(\left, \get, \maxPosition));
p.putAll(~buildMessage.value(\right, \get, \maxPosition));
)

p.putAll(~buildMessage.value(\global, \set, \stop));
p.putAll(~buildMessage.value(\left, \set, \minPosition, 100));
p.putAll(~buildMessage.value(\right, \set, \minPosition, 100));
p.putAll(~buildMessage.value(\left, \set, \maxPosition, 800));
(
p.putAll(~buildMessage.value(\left, \set, \direction, 0));
p.putAll(~buildMessage.value(\right, \set, \direction, 1));
)
(
p.putAll(~buildMessage.value(\left, \set, \targetPosition, 300));
p.putAll(~buildMessage.value(\right, \set, \targetPosition, 300));
)

p.putAll(~buildMessage.value(\rotation, \set, \targetPosition, 350));
p.putAll(~buildMessage.value(\left, \set, \maxPosition, 500));
p.putAll(~buildMessage.value(\right, \set, \maxPosition, 500));

(
p.putAll(~buildMessage.value(\left, \get, \maxPosition));
p.putAll(~buildMessage.value(\right, \get, \maxPosition));
)
(
p.putAll(~buildMessage.value(\left, \set, \currentPosition, 700));
p.putAll(~buildMessage.value(\right, \set, \currentPosition, 700));
)


//doesn't stop on slack sensor on the way down?
//autmatic 0 position set wen top sensor hit


p.putAll(~buildMessage.value(\left, \set, \stop));
p.putAll(~buildMessage.value(\left, \get, \speed));
p.putAll(~buildMessage.value(\global, \set, \stop));

p.putAll(~buildMessage.value(\left, \get, \direction));
p.putAll(~buildMessage.value(\left, \set, \direction, 1));
p.putAll(~buildMessage.value(\left, \set, \minPosition, 10));
p.putAll(~buildMessage.value(\left, \set, \maxPosition, 700));
p.putAll(~buildMessage.value(\left, \get, \currentPosition));
p.putAll(~buildMessage.value(\left, \set, \currentPosition, 0));
p.putAll(~buildMessage.value(\left, \set, \targetPosition, 100));
p.putAll(~buildMessage.value(\left, \set, \targetPosition, 300));

p.putAll(~buildMessage.value(\left, \set, \targetPosition, 0));
p.putAll(~buildMessage.value(\left, \get, \targetPosition));
p.putAll(~buildMessage.value(\left, \set, \direction, 1));

p.putAll(~buildMessage.value(\rotation, \set, \speed, 600));
p.putAll(~buildMessage.value(\rotation, \set, \speed, 0));
p.putAll(~buildMessage.value(\rotation, \get, \speed));
p.putAll(~buildMessage.value(\rotation, \set, \direction, 1));
p.putAll(~buildMessage.value(\rotation, \set, \direction, 0));
p.putAll(~buildMessage.value(\rotation, \get, \direction));
p.putAll(~buildMessage.value(\rotation, \set, \targetPosition, 700));
p.putAll(~buildMessage.value(\rotation, \get, \targetPosition));
p.putAll(~buildMessage.value(\rotation, \get, \currentPosition));


p.putAll(~buildMessage.value(\right, \set, \speed, 120));
p.putAll(~buildMessage.value(\right, \set, \stop));
p.putAll(~buildMessage.value(\right, \set, \direction, 1));
p.putAll(~buildMessage.value(\right, \get, \direction));
p.putAll(~buildMessage.value(\right, \get, \speed));
p.putAll(~buildMessage.value(\right, \set, \currentPosition, 800));
p.putAll(~buildMessage.value(\right, \set, \minPosition, 100));
p.putAll(~buildMessage.value(\right, \set, \maxPosition, 300));
p.putAll(~buildMessage.value(\right, \get, \currentPosition));


p.putAll(~buildMessage.value(\left, \set, \currentPosition, hh));
p.putAll(~buildMessage.value(\left, \get, \currentPosition));
p.putAll(~buildMessage.value(\right, \set, \currentPosition, jdjd));
p.putAll(~buildMessage.value(\right, \get, \currentPosition));

p.putAll(~buildMessage.value(\left, \set, \direction, 0));
p.putAll(~buildMessage.value(\left, \set, \direction, 1));
p.putAll(~buildMessage.value(\left, \get, \direction));
p.putAll(~buildMessage.value(\right, \set, \direction, 1));
p.putAll(~buildMessage.value(\right, \set, \direction, 0));
p.putAll(~buildMessage.value(\right, \get, \direction));

p.putAll(~buildMessage.value(\left, \set, \brake, 0));
p.putAll(~buildMessage.value(\left, \get, \brake));
p.putAll(~buildMessage.value(\right, \set, \brake, 0));
p.putAll(~buildMessage.value(\right, \get, \brake));

p.putAll(~buildMessage.value(\left, \set, \targetPosition, 11));
p.putAll(~buildMessage.value(\left, \get, \targetPosition));
p.putAll(~buildMessage.value(\right, \set, \targetPosition, 24));
p.putAll(~buildMessage.value(\right, \get, \targetPosition));

p.putAll(~buildMessage.value(\left, \set, \minPosition, 100));
p.putAll(~buildMessage.value(\left, \get, \minPosition));
p.putAll(~buildMessage.value(\right, \set, \minPosition, 322));
p.putAll(~buildMessage.value(\right, \get, \minPosition));

p.putAll(~buildMessage.value(\left, \set, \maxPosition, 400));
p.putAll(~buildMessage.value(\left, \get, \maxPosition));
p.putAll(~buildMessage.value(\right, \set, \maxPosition, 51));
p.putAll(~buildMessage.value(\right, \get, \maxPosition));


(0x09).asBinaryString
fork{ (400..0).do {|i| p.putAll(~buildMessage.value(\right, \set, \speed, i)); 0.01.wait};}
fork{ (0..400).do {|i| p.putAll(~buildMessage.value(\right, \set, \speed, i)); 0.1.wait};}

~buildMessage.value(\left, \set, \stop)[0].asAscii

SerialPort.devices


Tdef(\listenToSerial).stop;p.close


Tdef(\listenToSerial).clear


"L0240".ascii

16000000 / 1 / 2 / 800

s.boot
{SinOsc.ar(440)}.play


//sjekk motor range 0 - 1023
//parkerings funksjon
//spearate parker for rot and left right
//check what happens when robot starts with top sensor or slack sensor in/out

(
var win, leftView, rightView, rotationView;
win = Window.new("Pinne Robot Control", 200@600);
win.addFlowLayout(0@0, 0@0);
leftView = FlowView.new(win, 100@250, 0@0, 0@0, "Left pinne");
rightView = FlowView.new(win, 100@250, 0@0, 0@0, "Right pinne");
rotationView = FlowView.new(win, 200@100, 0@0, 0@0, "Rotation");
['left', 'right'].do{arg item, i;
	var view = [leftView, rightView][i];
	Slider(view, 100@200).action_({arg slid;
		var speed, direction, val;
		val = slid.value.linlin(0.0, 1.0, -512.0, 512.0);
		speed = val.abs.asInteger;
		direction = val.isPositive.asInteger;
		"speed: %, direction: %\n".postf(speed, direction);
		p.putAll(~buildMessage.value(item, \set, \direction, direction));
		p.putAll(~buildMessage.value(item, \set, \speed, speed));
	}).value_(0.5);
	Button(view,100@50)
	.action_({arg butt;p.putAll(~buildMessage.value(item, \set, \stop, 0))})
	.states_([["STOP:" ++ item.asString]]);
};
Slider(rotationView, 200@20).action_({arg slid;
	var speed, direction, val;
	val = slid.value.linlin(0.0, 1.0, -512, 512);
	speed = val.abs.asInteger;
	direction = val.isNegative.asInteger;
	"speed: %, direction: %\n".postf(speed, direction);
	p.putAll(~buildMessage.value(\rotation, \set, \direction, direction));
	p.putAll(~buildMessage.value(\rotation, \set, \speed, speed));

}).value_(0.5);
Button(rotationView,200@50)
	.action_({arg butt;p.putAll(~buildMessage.value(\rotation, \set, \stop, 0))})
	.states_([["STOP Rot"]]);
Button(win, 200@50).action_({arg butt;
//	p.putAll(~buildMessage.value(\global, \set, \goToParkingPosition, 0));
}).states_([["Parkering"]]);
win.front;
)
p.putAll(~buildMessage.value(\right, \set, \goToParkingPosition, 0));
p.putAll(~buildMessage.value(\left, \set, \goToParkingPosition, 0));
p.putAll(~buildMessage.value(\rotation, \set, \goToParkingPosition, 0));
p.putAll(~buildMessage.value(\global, \set, \goToParkingPosition, 0));

p.putAll(~buildMessage.value(\left, \set, \speed, 100));


p.putAll(~buildMessage.value(\rotation, \get, \currentPosition));
p.putAll(~buildMessage.value(\rotation, \set, \targetPosition, 300));

p.putAll(~buildMessage.value(\left, \set, \currentPosition, 10));

p.putAll(~buildMessage.value(\left, \set, \minPosition, 100));
p.putAll(~buildMessage.value(\left, \set, \maxPosition, 200));
(
Tdef(\checkPos, {
	loop {
		// p.putAll(~buildMessage.value(\rotation, \get, \currentPosition));
		// p.putAll(~buildMessage.value(\left, \get, \currentPosition));
		p.putAll(~buildMessage.value(\right, \get, \currentPosition));
		1.0.wait;
	}
}).play;
)
Tdef(\checkPos).stop

//LEAP motion controller