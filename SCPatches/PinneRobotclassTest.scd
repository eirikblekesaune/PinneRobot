///Pinner
SerialPort.listDevices;
(
~pinne1 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ0");// pinne en
~pinne2 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ1");// pinne to
~pinne3 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TZ0");// pinne tre
~pinne4 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TZ1");// pinne fire
~pinne5 = VTPinneRobot("/dev/tty.usbserial-FT5QT6UD0");// pinne fem
~pinne6 = VTPinneRobot("/dev/tty.usbserial-FT5QT6UD1");// pinne seks
~pinne7 = VTPinneRobot("/dev/tty.usbserial-FT5QT6UL0");// pinne syv
~pinne8 = VTPinneRobot("/dev/tty.usbserial-FT5QT6UL1");// pinne aatte
);
(
~pinne1 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ0");// pinne en
~pinne = ~pinne1;
)
~pinne = ~pinneApp.modules['pinne.1'].envir[\robot];
~pinne.stopRefreshTask;
~pinne.startRefreshTask;
~pinne.stop;
~pinne.leftMotor.speed_(220);
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
SimpleController.new(~pinne.leftMotor).put(\currentPosition, {
	"Left: %".format(~pinne.leftMotor.currentPosition).postln;
});
SimpleController.new(~pinne.rightMotor).put(\currentPosition, {
	"Right: %".format(~pinne.rightMotor.currentPosition).postln;
});
)
~pinne.leftMotor.dependants
~pinne.rightMotor.dependants
~pinne.rightMotor.changed(\currentPosition)

//test equal speed
(
~pinne.leftMotor.direction_(\down);
~pinne.rightMotor.direction_(\down);
~pinne.leftMotor.speed_(100);
~pinne.rightMotor.speed_(100);
)
(
~pinne.leftMotor.stop(0);
~pinne.rightMotor.stop(0);
)
(
~pinne.leftMotor.direction_(\up);
~pinne.rightMotor.direction_(\up);
~pinne.leftMotor.speed_(120);
~pinne.rightMotor.speed_(120);
)
~pinne.leftMotor.bipolarSpeed_(100)
~pinne.leftMotor.bipolarSpeed_(0)
Slider().action_({|slid| ~pinne.leftMotor.bipolarSpeed_(slid.value.linlin(0.0,1.0, -511, 511).asInteger)}.inEnvir).value_(0.5).front;

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



