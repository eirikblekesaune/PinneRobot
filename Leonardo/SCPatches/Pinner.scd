~pinner = [
	VTPinneRobot("/dev/tty.usbserial-FT5QT6TQA"),// pinne en
	VTPinneRobot("/dev/tty.usbserial-FT5QT6TQB"),// pinne to
	VTPinneRobot("/dev/tty.usbserial-FT5QT6TZA"),// pinne tre
	VTPinneRobot("/dev/tty.usbserial-FT5QT6TZB"),// pinne fire
	VTPinneRobot("/dev/tty.usbserial-FT5QT6UDA"),// pinne fem
	VTPinneRobot("/dev/tty.usbserial-FT5QT6UDB"),// pinne seks
	VTPinneRobot("/dev/tty.usbserial-FT5QT6ULA"),// pinne syv
	VTPinneRobot("/dev/tty.usbserial-FT5QT6ULB")// pinne aatte
];

OSCdef(\pinneClassResponder, {arg msg, time, addr, port;
	var index, motor, val, key, pinne, pinneMotor;
	index = msg[1].asInteger - 1;
	motor = msg[2].asSymbol;
	key = msg[3].asSymbol;
	switch(key,
		\direction, {val = msg[4];},
		\goToPosition, {val = msg[4..];},
		{val = msg[4].asInteger;}
	);
	pinne = ~pinner[index];
	"pinne class: num: % motor: % key: % val: %\n".postf(index, motor, key, val);

	pinneMotor = switch(motor,
		\left, pinne.leftMotor,
		\right, pinne.rightMotor,
		\rotation, pinne.rotationMotor
	);
	"MOTOR: %".format([pinneMotor, key, val]).postln;
	pinneMotor.perform(key.asSetter, *val);
}.inEnvir, '/robot/pinne');

OSCdef(\pinneClassStopper, {arg msg, time, addr, port;
	~pinner.do({arg item;
		item.stop;
	});
}, '/robot/pinne/stop');

OSCdef(\getPinnePositions, {arg msg, time, addr, port;
	var packet = ();
	~pinner.do({arg item, i;
		var result = ();
		result.put(\left, item.leftMotor.currentPosition);
		result.put(\right, item.rightMotor.currentPosition);
		result.put(\rotation, item.rotationMotor.currentPosition);
		packet.put((i + 1).asSymbol, result);
	});
	NetAddr("1.2.3.111", 50010).sendMsg('/pinneData', JSON.stringify(packet));
}, '/robot/getPinneData');

OSCdef(\getPinnePosition, {arg msg, time, addr, port;
	var num, pinne;
	var packet = ();
	num = msg[1];
	pinne = ~pinner[num - 1];
	packet.put(\left, pinne.leftMotor.currentPosition);
	packet.put(\right, pinne.rightMotor.currentPosition);
	packet.put(\rotation, pinne.rotationMotor.currentPosition);
	NetAddr("1.2.3.111", 50010).sendMsg("/pinneData/pinne.%".format(num).asSymbol, JSON.stringify(packet));
}, '/robot/pinne/getData');

/////////////////////////////////////////////////////////
//Sound attachment responders
/////////////////////////////////////////////////////////
~pinneSoundData = Dictionary.new;
(1..8).do{arg pinneNumber;
	~pinneSoundData.put(pinneNumber, ());
	[\left, \right, \rotation].do{arg motorName;
		~pinneSoundData[pinneNumber].put(motorName, ());
		OSCdef("attachSoundPinne_%_%".format(pinneNumber, motorName).asSymbol, {
			arg msg, time, addr, port;
			var soundCue, soundCueKey, soundModuleKey, lydserver;
			var soundModulePath, incomingCue;
			incomingCue = Dictionary.newFrom( msg[1].asString.parseYAML);
			"The sound cue: [%]%".format(incomingCue.class, incomingCue).postln;
			"Why: %".format(incomingCue.respondsTo(\put)).postln;
			incomingCue.put('name', \bufferplayer);
			"Attaching sound to pinne % - %".format(pinneNumber, motorName);
			lydserver = NetAddr("1.2.3.57", 57120);
			soundCue = ().put(
				"pinne%".format(pinneNumber).asSymbol,
				().put(
					\source,
					().put("%Sound".format(motorName).asSymbol, incomingCue)
				)
			);
			soundModulePath = "/pinne%/source/%Sound".format(pinneNumber, motorName);
			lydserver.sendMsg('/cue/json', JSON.stringify(soundCue));
			if(~pinneSoundData[pinneNumber][motorName][\soundController].isNil, {
				~pinneSoundData.at(pinneNumber).at(motorName).put(
					\soundController,
					SimpleController.new(
						~pinner[pinneNumber - 1].perform("%Motor".format(motorName).asSymbol)
					).put(\bipolarSpeed, {arg theChanged, what;
						lydserver.sendMsg(
							"%/controlValue".format(soundModulePath).asSymbol,
							theChanged.bipolarSpeed.abs.linlin(0.0, 512, 0.0, 1.0);
						);
					});
				);
			});
		}.inEnvir, "/robot/pinne%/%/attachSound".format(pinneNumber, motorName).asSymbol);
		OSCdef("detachSoundPinne%_%".format(pinneNumber, motorName).asSymbol, {arg msg, time, addr, port;
			var soundCuePath;
			"Detaching sound from pinne % - %".format(pinneNumber, motorName).postln;
			soundCuePath = "/pinne%/source/%Sound".format(pinneNumber,motorName).asSymbol;
			NetAddr("1.2.3.57", 57120).sendMsg('/cue/module/free', soundCuePath);
			~pinneSoundData[pinneNumber][motorName].removeAt('soundController').remove(\bipolarSpeed);
		}.inEnvir, "/robot/pinne%/%/detachSound".format(pinneNumber, motorName).asSymbol);

	};
};