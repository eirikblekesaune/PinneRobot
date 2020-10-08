///Pinner
SerialPort.listDevices;
(
~pinne1 = VTPinneRobot("/dev/ttyUSB7");// pinne en
~pinne2 = VTPinneRobot("/dev/ttyUSB6");// pinne to
~pinne3 = VTPinneRobot("/dev/ttyUSB5");// pinne tre
~pinne4 = VTPinneRobot("/dev/ttyUSB4");// pinne fire
~pinne5 = VTPinneRobot("/dev/ttyUSB3");// pinne fem
~pinne6 = VTPinneRobot("/dev/ttyUSB2");// pinne seks
~pinne7 = VTPinneRobot("/dev/ttyUSB1");// pinne syv
~pinne8 = VTPinneRobot("/dev/ttyUSB0");// pinne aatte
);
(
//~pinne1 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ0", reverseMount: true);// pinne en
~pinne = ~pinne2;
)
~pinne.stopRefreshTask;
~pinne.startRefreshTask;
~pinne.stop;
~pinne.leftMotor.speed_(512);
~pinne.leftMotor.speed_(0);
~pinne.leftMotor.direction_(\down);
~pinne.leftMotor.direction_(\up);
~pinne.rightMotor.stop(1000);
~pinne.rightMotor.speed_(140);
~pinne.rightMotor.speed_(0);
~pinne.rightMotor.direction_(\down);
~pinne.rightMotor.direction_(\up);
~pinne.rotationMotor.speed_(800);
~pinne.rotationMotor.speed_(0);
~pinne.rotationMotor.direction_(\left);
~pinne.rotationMotor.direction_(\right);
~pinne.leftMotor.currentPosition;
~pinne.rightMotor.currentPosition;
~pinne.rotationMotor.currentPosition;

~pinne.leftMotor.maxPosition_(2000)

~pinne.echoMessages_(true);
~pinne.echoMessages_(false);

~pinne.leftMotor.speed_(120);
~pinne.rightMotor.speed_(120);
~pinne.leftMotor.targetPosition_(200);
~pinne.rightMotor.targetPosition_(200);
~pinne.leftMotor.goToTargetPosition(100);
~pinne.rightMotor.goToTargetPosition(10);
~pinne.leftMotor.speedScaling_(200);
~pinne.rightMotor.speedScaling_(200);

~pinne.leftMotor.addDependant({|...args|"LEFT: %".format(args).postln;});
~pinne.rightMotor.addDependant({|...args|"RIGHT: %".format(args).postln;});

"{cmd: 15,addr: 48,setGet: 0,value: 1}".parseYAML["addr"];
(
SimpleController.new(~pinne.leftMotor).put(\state, {
	"Left: %".format(~pinne.leftMotor.state).postln;
});
SimpleController.new(~pinne.rightMotor).put(\state, {
	"Right: %".format(~pinne.rightMotor.state).postln;
});
)

//test equal speed
(
~pinne.leftMotor.direction_(\down);
~pinne.rightMotor.direction_(\down);
~pinne.leftMotor.speed_(150);
~pinne.rightMotor.speed_(150);
)
(
~pinne.leftMotor.stop(0);
~pinne.rightMotor.stop(0);
)
(
~pinne.leftMotor.direction_(\up);
~pinne.rightMotor.direction_(\up);
~pinne.leftMotor.speed_(190);
~pinne.rightMotor.speed_(190);
)
~pinne.leftMotor.bipolarSpeed_(-100)
~pinne.leftMotor.bipolarSpeed_(0)
~pinne.rightMotor.bipolarSpeed_(-100)
~pinne.rightMotor.bipolarSpeed_(0)



~pinne.leftMotor.goToParkingPosition;
~pinne.rightMotor.goToParkingPosition;
~pinne.rotationMotor.goToParkingPosition;

f = fork{ loop { ~pinne.rotationMotor.currentPosition.postln; 0.5.wait; } };
f.stop;
~pinneRanges = (
	\1: (
		rotation: (minPosition: 200, maxPosition: 800),
		left: (minPosition: 0, maxPosition: 690),
		right: (minPosition: 0, maxPosition: 690)//traad maa forlenges
	),
	\2: (
		rotation: (minPosition: 200, maxPosition: 800),
		left: (minPosition: 0, maxPosition: 590),
		right: (minPosition: 0, maxPosition: 590)
	),
	\3: (
		rotation: (minPosition: 367, maxPosition: 576),
		left: (minPosition: 0, maxPosition: 580),
		right: (minPosition: 0, maxPosition: 580)
	),
	\4: (
		rotation: (minPosition: 200, maxPosition: 800),
		left: (minPosition: 0, maxPosition: 650),
		right: (minPosition: 0, maxPosition: 650)
	),
	\5: (
		rotation: (minPosition: 485, maxPosition: 550),
		left: (minPosition: 0, maxPosition: 600),
		right: (minPosition: 0, maxPosition: 600)//noe rart med rot pot feste her
	),
	\6: (
		rotation: (minPosition: 406, maxPosition: 722),
		left: (minPosition: 0, maxPosition: 600),
		right: (minPosition: 0, maxPosition: 600)//noe rart med rot pot feste her
	),
	\7: (
		rotation: (minPosition: 300, maxPosition: 700),//rot pot funker ikke
		left: (minPosition: 0, maxPosition: 600),
		right: (minPosition: 0, maxPosition: 600)
	),
	\8: (
		rotation: (minPosition: 300, maxPosition: 700),//rot pot funker ikke
		left: (minPosition: 0, maxPosition: 600),
		right: (minPosition: 0, maxPosition: 600)
	)
)



