VTPinneRobot {
	var parser, <online = false;
	var <leftMotor, <rightMotor, <rotationMotor;
	var refreshTask, <>refreshInterval = 0.01;//refresh is when cached values are sent to the robot
	var updateTask, <>updateInterval = 1.0;//update is when values are requested from the robot
	var semaphore;
	var <reverseMount;

	*new{arg path, doConnect = true, reverseMount = false;
		^super.new.init(path, doConnect, reverseMount);
	}

	init{arg path_, doConnect = true, reverseMount_;
		reverseMount = reverseMount_;
		if(reverseMount, {
			leftMotor = VTPinneRobotMotor.new(this, \right);
			rightMotor = VTPinneRobotMotor.new(this, \left);
		}, {
			leftMotor = VTPinneRobotMotor.new(this, \left);
			rightMotor = VTPinneRobotMotor.new(this, \right);
		});
		rotationMotor = VTPinneRotationMotor.new(this, \rotation);
		if(doConnect, {
			this.connect(path_);
			refreshTask = Task({
				loop{
					this.refresh;
					refreshInterval.wait;
				}
			}).play;
			updateTask = Task({
				this.requestValuesFromRobot([\currentPosition]);
				loop{
					// this.requestValuesFromRobot([\currentPosition]);
					updateInterval.wait;
				}
			}).play;
			this.sendMsg(\right, \get, \direction);
			this.sendMsg(\left, \get, \direction);
			this.sendMsg(\rotation, \get, \direction);

		});
		if(this.connected.not, {"PinneRobot is offline".warn});
	}

	connect{arg path;
		parser = VTPinneRobotParser(this, path);
	}

	connected{
		^(parser.notNil and: {parser.connected});
	}

	disconnect{
		parser.disconnect;
	}

	free{
		this.stopRefreshTask;
		this.disconnect;
	}

	startRefreshTask{
		refreshTask.play;
		updateTask.play;
	}

	stopRefreshTask{
		refreshTask.stop;
		updateTask.stop;
	}

	sendMsg{arg address, setGet, command, val;
		parser.sendMsg(address, setGet, command, val);
	}

	refresh{
		[leftMotor, rightMotor, rotationMotor].do(_.refresh);
	}

	stop{
		[leftMotor, rightMotor, rotationMotor].do(_.stop);
	}

	requestValuesFromRobot{arg which;//a list of keys to be updated
		[leftMotor, rightMotor, rotationMotor].do(_.requestValuesFromRobot(which));
	}

	echoMessages_{arg val;
		parser.sendMsg('global', 'set', 'echoMessages', val.booleanValue.asInteger);
	}

	//x and y are decimals from -1.0 to 1.0
	joystickSpeed_{arg x, y, invertFactor;
		var weights;
		var values;
		var invertScales;
		//use x for weighting betwen left and right
		weights = [
			x.linlin(-1.0, 1.0, 2.0, 0.0).min(1.0),
			x.linlin(-1.0, 1.0, 0.0, 2.0).min(1.0),
		];
		invertScales = [
			invertFactor.linlin(-1.0, 1.0, -1.0, 3.0).min(1.0),
			invertFactor.linlin(-1.0, 1.0, 3.0, -1.0).min(1.0),
		];
		values = ((y * 511) ! 2) * (weights * invertScales);
		leftMotor.bipolarSpeed_(values[0]);
		rightMotor.bipolarSpeed_(values[1]);
	}

	update{arg theChanged, theChanger, address, key, value;
		//		"Robot update: %".format([theChanged, theChanger, address, key, value]).postln;
		if(theChanger !== this, {

			switch(address,
				\left, {this.leftMotor.perform(key.asSetter, value, sync: false);},
				\right, {this.rightMotor.perform(key.asSetter, value, sync: false);},
				\rotation, {this.rotationMotor.perform(key.asSetter, value, sync: false);},
			);
		});
	}
}

VTPinneRobotMotor{
	var robot;
	var <speed = 0;
	var <direction = \down; //symbol up or down
	var <targetPosition = 0;
	var <currentPosition = 0;
	var <minPosition = 0;
	var <maxPosition = 4096;
	var <state = \goingDown;
	var <speedRampUp = 0;
	var <speedRampDown = 0;
	var <speedScaling = 0;
	var <stopTime = 0;
	var <specs;
	var <directionEnum;
	var <address;
	var needsRefresh;//parameters that need to be sent to the robot
	var outputCache;//parameter values to be sent to the robot
	var needsUpdate;//parameter values to be update from the robot
	var commandOrder;

	*new{arg robot, address;
		^super.new.init(robot, address);
	}

	init{arg robot_, address_;
		robot = robot_;
		address = address_;
		directionEnum = TwoWayIdentityDictionary[\down -> 0, \up -> 1];
		specs = (
			speed: ControlSpec(0, 512,step:1,default:0),
			targetPosition: ControlSpec(0, 4096, step: 1),
			currentPosition: ControlSpec(0, 4096, step: 1),
			minPosition: ControlSpec(0, 4096, step: 1),
			maxPosition: ControlSpec(0, 4096, step: 1),
			speedRampUp: ControlSpec(0, 5000, step: 1, default: 0),
			speedRampDown: ControlSpec(0, 5000, step: 1, default: 0),
			speedScaling: ControlSpec(0.0, 1.0, step: 0.001, default: 0),
			stopTime: ControlSpec(0, 5000, step:1, default:0),
			goToTargetPosition: ControlSpec(0, 5000, \lin, 1, default: 1000)
		);
		commandOrder = (
			'targetPosition': 1,
			'direction': 2,
			'speed': 3,
			'currentPosition': 4,
			'brake': 5,
			'minPosition': 6,
			'maxPosition': 7,
			'goToParkingPosition': 9,
			'goToTargetPosition': 10,
			'goToSpeedRampUp': 11,
			'goToSpeedRampDown': 12,
			'goToSpeedScaling': 13,
			'echoMessages': 14,
			'stop': 15,
			'stateChange': 16,
			'info': 17
		);

		needsRefresh = Order.new;
		//needsRefresh.add(\direction);
		needsUpdate = IdentitySet.newFrom(specs.keys);
		outputCache = IdentityDictionary.new;

	}

	isReverseMounted{
		^robot.reverseMount;
	}

	prInvalidateParameter{arg key, val;
		var commandPlace;
		commandPlace = commandOrder[key];
		if(commandOrder.notNil, {
			outputCache.put(key, val);
			needsRefresh.put(commandPlace, key);
		}, {
			"Unknown command order place: %".format(key).warn;
		});
	}

	speed_{arg val, sync = true;
		speed = specs.at(\speed).constrain(val);
		if(sync, { this.prInvalidateParameter(\speed, speed)});
	}

	bipolarSpeed{
		if(direction == 0, {^speed.neg;}, {^speed});
	}

	bipolarSpeed_{arg val;
		if(val.isNegative, {
			this.direction_(\down);
		});
		if(val.isPositive, {
			this.direction_(\up);
		});
		this.speed_(val.abs);
		this.changed(\bipolarSpeed);
	}

	getOppositeMotor{
		var result;
		if(address == \left, {
			result = robot.rightMotor;
		}, {
			result = robot.leftMotor;
		});
		^result;
	}

	getLeftMotor{
		^robot.leftMotor;
	}

	getRightMotor{
		^robot.rightMotor;
	}

	stop{
		//this.set(\stop, val);
		//speed = 0;
		//this.changed(\stop);
		this.bipolarSpeed_(0);
	}

	direction_{arg newDir, sync = true;
		if(newDir.isKindOf(SimpleNumber),
			{
				direction = directionEnum.getID(newDir.clip(0, 1));
			},
			{
				direction = directionEnum.at(newDir);
			}
		);
		if(sync, { this.prInvalidateParameter(\direction, direction)});
	}

	targetPosition_{arg val, sync = true;
		targetPosition = specs.at(\targetPosition).constrain(val);
		if(sync, { this.prInvalidateParameter(\targetPosition, targetPosition)});
	}

	currentPosition_{arg val, sync = true;
		var oldVal = currentPosition;
		currentPosition = specs.at(\currentPosition).constrain(val);
		if(oldVal != currentPosition, {
			this.changed(\currentPosition, currentPosition);
		});
		if(sync, { this.prInvalidateParameter(\currentPosition, currentPosition)});
	}

	minPosition_{arg val, sync = true;
		minPosition = specs.at(\minPosition).constrain(val);
		if(sync, { this.prInvalidateParameter(\minPosition, minPosition)});
	}

	maxPosition_{arg val, sync = true;
		maxPosition = specs.at(\maxPosition).constrain(val);
		if(sync, { this.prInvalidateParameter(\maxPosition, maxPosition)});
	}

	goToPosition_{arg val, speed;
		"Going to position".postln;
		this.changed(\goToPosition);
		this.targetPosition_(val);
		this.speed_(speed);
	}

	// stop{arg val = 0, sync = true;
	// 	stopTime = specs.at(\stopTime).constrain(val);
	// 	if(sync, { this.prInvalidateParameter(\stop, stopTime.asInteger)});
	// }

	goToParkingPosition{
		this.prInvalidateParameter(\goToParkingPosition, 0);
		direction = \up;//this is dangerous
	}

	goToParkingPosition_{//temp hack
		this.prInvalidateParameter(\goToParkingPosition, 0);
		direction = \up;//this is dangerous
	}

	goToTargetPosition{arg duration;
		duration = duration ? specs.at(\goToTargetPosition).constrain(duration);
		this.prInvalidateParameter(\goToTargetPosition, duration);
	}

	goToTargetPosition_{arg duration;
		duration = duration ? specs.at(\goToTargetPosition).constrain(duration);
		this.prInvalidateParameter(\goToTargetPosition, duration);
	}

	speedRampUp_{arg val;
		speedRampUp = specs.at(\speedRampUp).constrain(val);
		this.prInvalidateParameter(\speedRampUp, speedRampUp);
	}

	speedRampDown_{arg val;
		speedRampDown = specs.at(\speedRampDown).constrain(val);
		this.prInvalidateParameter(\speedRampDown, speedRampDown);
	}

	speedScaling_{arg val;
		speedScaling = specs.at(\speedScaling).constrain(val);
		this.prInvalidateParameter(\speedScaling, speedScaling);
	}

	state_{arg newState, sync = true;
		if(newState != state, {
			state = newState;
			this.changed(\state);
		});
	}

	refresh{arg forceRefresh = false;
		var cachedValue, cachedKey, cacheStream, val;
		if(needsRefresh.notEmpty, {
			cacheStream = needsRefresh.deepCopy.iter;
			needsRefresh.clear;
			cachedKey = cacheStream.next;
			while({cachedKey.notNil}, {
				val = outputCache.removeAt(cachedKey);
				if(val.notNil, {
					robot.sendMsg(address, \set, cachedKey, val);
				}, {
					"Did not find value for cached key: %".format(cachedKey).postln;
				});
				cachedKey = cacheStream.next;
			});
		});
	}

	requestValuesFromRobot{arg which;
		if(which.isNil, {
			//"reqeusting all".postln;
			specs.keys.do({arg item;
				//robot.sendMsg(address, \get, item);
			});
		}, {
			//"Requesting: %\n".postf(which);
			which.do({arg item;
				robot.sendMsg(address, \get, item, 0);
			})
		});
	}

	//this is a push style udpate function. The changed message must send the new value as an argument
	//
	update{arg theChanged, theChanger, what, value;
		"Motor:Update: $".format(theChanged, theChanger, what).postln;
		if(theChanger !== this, {
			this.setParameter(what, value, sync: false);
		});
	}
}

VTPinneRotationMotor : VTPinneRobotMotor {
	*new{arg robot, address;
		^super.new(robot, address);
	}

	init{arg robot_, address_;
		direction = \right;
		super.init(robot_, address_);
		directionEnum = TwoWayIdentityDictionary[\right -> 0, \left -> 1];
	}

	bipolarSpeed_{arg val;
		if(val.isNegative and: {direction != \right}, {
			this.direction_(\right);
		});
		if(val.isPositive and: {direction != \left}, {
			this.direction_(\left);
		});
		this.speed_(val.abs);
		this.changed(\bipolarSpeed);
	}
}


VTPinneRobotParser{
	var state = \waitingForCommandByte;
	var dataBytesReceived = 0;
	var robot;
	var <serialPort, readTask, serialPortLock;
	var currentCommand;
	var currentAddress;
	var currentSetGet;
	var infoBytes;
	var valueBytes;
	var valueBytesBuffer;
	var messageQueue;
	var hasOutgoingMessages;
	var messageSender;

	classvar <addressMasks, <commandMasks, <setGetMasks, <stateChangeMasks;

	*initClass{
		addressMasks = TwoWayIdentityDictionary[
			\left -> 2r00000000,
			\right -> 2r00010000,
			\rotation -> 2r00100000,
			\global -> 2r00110000
		];
		commandMasks = TwoWayIdentityDictionary[
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
			\goToTargetPosition -> 2r1011,//0x0B
			\goToSpeedRampUp -> 2r1100,//argument is ramp up percent of halfway point
			\goToSpeedRampDown -> 2r1101,//ramp down time will take effect after halfway point
			\goToSpeedScaling -> 2r1110,
			\echoMessages -> 2r1111
		];
		setGetMasks = TwoWayIdentityDictionary[
			\set -> 2r00000000,
			\get -> 2r01000000
		];
		stateChangeMasks = TwoWayIdentityDictionary[
			\stopped -> 2r00000000,
			\goingDown -> 2r00000001,
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
	}

	*new{arg robot, path;
		^super.new.init(robot, path);
	}

	init{arg robot_, path_;
		robot = robot_;
		serialPortLock = Semaphore(1);
		this.connect(path_);
		this.addDependant(robot);
		messageQueue = LinkedList.new;
		hasOutgoingMessages = Condition.new(false);
		messageSender = Task({
			loop{
				hasOutgoingMessages.wait;
				serialPort.putAll(messageQueue.popFirst);
				if(messageQueue.isEmpty, {
					hasOutgoingMessages.test = false;
				});
			};
		}).play;
		readTask = Task({
			loop{
				var byte;
				byte = serialPort.read;
				this.parseByte(byte);
			}
		}).play;
		serialPort.doneAction = {
			readTask.stop;
		};
	}

	reset{
		valueBytes = nil;
		state = \waitingForCommandByte;
	}

	parseByte{arg byte;
		//		"Praser state: %".format([state, byte]).postln;
		if(byte.bitAnd(128) > 0 and: {state == \waitingForCommandByte}, {//if first bit is 1
			var command, nextParserState, address, setGet;
			//byte is a command byte
			dataBytesReceived = 0;
			valueBytesBuffer = nil;
			nextParserState = \waitingForDataByte;
			command = this.class.commandMasks.getID(byte.bitAnd(2r00001111));//keep the lower 4 bits, and get command symbol
			address = this.class.addressMasks.getID(byte.bitAnd(2r00110000));//get address symbol
			setGet = this.class.setGetMasks.getID(byte.bitAnd(2r01000000));//get set or get command
			switch(command,
				\stop, {"Received stop command".postln},
				\stateChange, {
					//"Receive state change command".postln;
					nextParserState = \waitingForStateByte;
				},
				\speed, {/*"Received speed command".postln*/},
				\direction, {/*"Received direction command".postln*/},
				\targetPosition, {"Received targetPosition command".postln},
				\currentPosition, {/*"Receive currentPosition command".postln;*/},
				\brake, {"Received brake command".postln;},
				\minPosition, {"Receive min position command".postln;},
				\maxPosition, {"Receive max position command".postln;},
				\info, {
					nextParserState = \waitingForInfoByte;
				}
			);
			state = nextParserState;
			currentCommand = command;
			currentAddress = address;
			currentSetGet = setGet;
		}, {
			if(state == \waitingForDataByte, {
				switch(dataBytesReceived,
					0, {
						valueBytesBuffer = valueBytesBuffer.add(byte);
						dataBytesReceived = 1;
					},
					1, {
						valueBytes = valueBytesBuffer.add(byte).copy;
						this.doCommand;
					}
				)
			});
			if(state == \waitingForStateByte, {
				//"State byte received".postln;
				this.doStateCommand(byte);
			});
			if(state == \waitingForInfoByte, {
				if(byte != 4, {//4 is end of transmission byte according to ASCII
					infoBytes = infoBytes.add(byte);
				}, {
					try{
						var str;
						str = String.newFrom(infoBytes.collect(_.asAscii));
						if(str.beginsWith("ECHO "), {
							str = str.copyRange(5, str.size);
							str = str.parseYAML;
							"ECHO-> cmd: '%' addr: '%' setGet: '%' value: '%'".format(
								commandMasks.getID(str["cmd"].asInteger),
								addressMasks.getID(str["addr"].asInteger),
								setGetMasks.getID(str["setGet"].asInteger),
								str["value"]
							).postln;
						}, {
							"INFO: [%]: ".postf(currentAddress);
							str.postln;
						});
					} {
						"Failed parsing info bytes: %".format(infoBytes).warn;
					};
					infoBytes = Array.new;
					this.reset;
				});
			});
		});
	}

	doCommand{
		var value;
		try{
			value = this.class.parseDataBytes(valueBytes);
		} {
			"Parsing valueBytes failed: %".format(valueBytes).warn;
		};
		try{
			robot.update(robot, this, currentAddress, currentCommand, value);
		} {
			"Failed doing command from robot: addr: '%' command: '%' value: '%'".format(
				currentAddress, currentCommand, value
			).warn;
		};
		this.reset;
	}

	doStateCommand{arg newState;
		//		"State change: %".format(this.class.stateChangeMasks.getID(newState)).postln;
		robot.update(robot, this, currentAddress, \state, this.class.stateChangeMasks.getID(newState));
		this.reset;
	}

	prBuildMessage{ arg address, setGet, command, value;
		var msg, checksum = 0;
		msg = [
			128,//2r10000000 signifies a command byte
			this.class.addressMasks[address],
			this.class.setGetMasks[setGet],
			this.class.commandMasks[command],
		].reduce(\bitOr).asArray;
		if(setGet == \set, {
			msg = msg ++ this.class.makeDataBytes(value);
		});
		msg.do({|it|
			checksum = (checksum + it).bitAnd(0xFF);
		});

		checksum = 0xFF - checksum;
		msg = msg ++ checksum;
		^msg;
	}

	sendMsg{arg addr, setGet, command, value;
		//"Sending message: %".format([addr, setGet, command, value]).postln;
		// forkIfNeeded{
		// serialPortLock.wait;
		messageQueue.add(this.prBuildMessage(addr, setGet, command, value));
		hasOutgoingMessages.test = true;
		hasOutgoingMessages.signal;
		// serialPort.putAll(this.prBuildMessage(addr, setGet, command, value));
		// serialPortLock.signal;
		// }
	}

	connect{arg path;
		this.disconnect;
		serialPort = SerialPort.new(path, 57600, crtscts:true);
	}

	disconnect{
		if(serialPort.notNil, { serialPort.close; });
	}

	connected{//should ping robot device instead
		^serialPort.isOpen;
	}

	*makeDataBytes{arg value;
		value = value.asInteger;
		^[value.rightShift(7).bitAnd(127), value.bitAnd(127)];
	}

	*parseDataBytes{arg bytes;
		^bytes.at(0).leftShift(7).bitOr(bytes.at(1));
	}
}
