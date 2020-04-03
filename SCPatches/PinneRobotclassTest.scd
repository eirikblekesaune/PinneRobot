///Pinner
SerialPort.listDevices;
(
~pinner = Dictionary.new;
~pinner[1] = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ0", reverseMount: true);// pinne en
~pinner[2] = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ1", reverseMount: true);// pinne to
~pinner[3] = VTPinneRobot("/dev/tty.usbserial-FT5QT6TZ0");// pinne tre
~pinner[4] = VTPinneRobot("/dev/tty.usbserial-FT5QT6TZ1");// pinne fire
~pinner[5] = VTPinneRobot("/dev/tty.usbserial-FT5QT6UD0", reverseMount: true);// pinne fem
~pinner[6] = VTPinneRobot("/dev/tty.usbserial-FT5QT6UD1", reverseMount: true);// pinne seks
~pinner[7] = VTPinneRobot("/dev/tty.usbserial-FT5QT6UL0");// pinne syv
~pinner[8] = VTPinneRobot("/dev/tty.usbserial-FT5QT6UL1");// pinne aatte
);
(
var speed = 0;
~pinner.do({arg it;
	it.leftMotor.bipolarSpeed_(speed);
	it.rightMotor.bipolarSpeed_(speed);
});
)
(
~pinner[6].leftMotor.bipolarSpeed_(0);
~pinner[6].rightMotor.bipolarSpeed_(0);
)

(
~pinner[6].leftMotor.bipolarSpeed_(-100);
~pinner[6].rightMotor.bipolarSpeed_(-100);
)
~pinner[8].leftMotor.currentPosition;
~pinner[8].rightMotor.currentPosition;

(
~pinner[8].leftMotor.goToParkingPosition;
~pinner[8].rightMotor.goToParkingPosition;
)

1600 /6.0

(
var speed = 0;
~pinner.do({arg it;
	it.leftMotor.goToParkingPosition;
	it.rightMotor.goToParkingPosition;
});
)
(
var ctrlLayouts;
(1..8).do({arg num;
	ctrlLayouts = ctrlLayouts.add(
		HLayout(
			VLayout(
				Slider().action_({|slid|
					~pinner[num].leftMotor.bipolarSpeed_(slid.value.linlin(0.0,1.0, -511, 511).asInteger)}.inEnvir).value_(0.5),
				Button().action_({|slid|
					~pinner[num].leftMotor.stop;
				}).states_([["STOP"]])
			),
			VLayout(
				Slider().action_({|slid|
					~pinner[num].rightMotor.bipolarSpeed_(slid.value.linlin(0.0,1.0, -511, 511).asInteger)}.inEnvir).value_(0.5),
				Button().action_({|slid|
					~pinner[num].rightMotor.stop;
				}).states_([["STOP"]])
			)
		)
	);
});
Window().layout_(
	HLayout(*ctrlLayouts)
).front
)
(
var num = 8;
Window().layout_(
	HLayout(
		VLayout(
			Slider().action_({|slid|
				~pinner[num].leftMotor.bipolarSpeed_(slid.value.linlin(0.0,1.0, -511, 511).asInteger)}.inEnvir).value_(0.5),
			Button().action_({|slid|
				~pinner[num].leftMotor.stop;
			}).states_([["STOP"]])
		),
		VLayout(
			Slider().action_({|slid|
				~pinner[num].rightMotor.bipolarSpeed_(slid.value.linlin(0.0,1.0, -511, 511).asInteger)}.inEnvir).value_(0.5),
			Button().action_({|slid|
				~pinner[num].rightMotor.stop;
			}).states_([["STOP"]])
		)
	)
).front
)

(
~pinne1 = VTPinneRobot("/dev/tty.usbserial-FT5QT6TQ0", reverseMount: true);// pinne en
~pinne = ~pinne1;
)

~pinne = ~pinne1;
// ~pinne = ~pinneApp.modules['pinne.1'].envir[\robot];
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



