Pinne API is separates between data and command byte, much like MIDI, and supports 14-bit value ranges.


* bit7: 1=command byte, 0=value byte. i.e. 1nnn nnnn is a command byte, 0nnn nnnn is a data byte
* bit6: 0=set message, 1=get message
* bit4-5: motor address, 
	* 00: left motor
	* 01: right motor
	* 10: rotation motor
	* 11: global command
* bit0-3: Which command:
	* 0000: stop
	* 0001: speed
	* 0010: direction
	* 0011: targetPosition
	* 0100: currentPosition
	* 0101: brake
	* 0110: state change
	* 0111: info (debug)
	* 1000: min position
	* 1001: max position

Det første som kommer inn er alltid en kommando byte med bit8==1. Hvis beskjeden har tilhørende data, f.eks. sett posisjon i etterfølgende bytes, så er bit7==1. Beskjeden vil i så tilfelle bli etterfulgt av data bytes hvor bit8==0.

Data bytes varierer med hvilken kommando som er sendt.
State data byte: Value byte for state er bygd opp slik: 
```
Command byte:
bit 0: 1 Since it is a command byte
bit 1: 0 Since this is a set command
bit 2-3: nn (Which motor(address) the state is concerning
bit 4-8: 0001, signifies a state value message

Value byte:
bit 0: 0 (Since it is a value byte)
bit 1 - 7:
x0000000: stopped
x0000001: going up
x0000010: going down
x0000011: stopped at target
x0000100: blocked by top sensor
x0000101: blocked by slack sensor
x0000110: blocked by min position
x0000111: blocked by max position
x0001000: motor driver fault 
```

Telleverk/Target position/speed data bytes:
0<bits:20,19,18,17,16,15,14> 0<bits:13,12,11,10,9,8,7> 0<bits:6,5,4,3,2,1,0>
Dette utgjør en unsigned 32 bit type, selv om oppløsning kun går til 21-bits verdi. Dette er overkill for fart, servoposisjon og servofart, men jeg tror det er lurt å ha mer å gå på for telleverk/posisjon. Greit at databytes alltid er tre bytes med denne formateringen, slik at du kun trenger èn rutine for å parse de.


Vi får følgende beskjeder sendt til pinnerobot: ('x' betyr at bit ikke er viktig, men settes til 0 pr default, '->>' indikerer hva som pinnerobot skal returnere)

* Non-replying commands: 
    * Left motor go up:                     11010001 0xxxxx01 = D1 01 0D 0A
    * Left motor go down:                 11010001 0xxxxx11 = D1 03 0D 0A
    * Left motor stop:                       10010000                = 90 0D 0A
    * Left motor set speed:               11011000 0nnnnnnn 0nnnnnnn = D8 nn nn nn 0D 0A
    * Left motor set current position: 11010100 0nnnnnnn 0nnnnnnn  = D4 nn nn nn 0D 0A
    * Left motor set target position:   11010010 0nnnnnnn 0nnnnnnn  = D2 nn nn nn 0D 0A
  
   * Right motor go up:                     11110001 00000001 = F1 01 0D 0A
   * Right motor go down:                 11110001 00000011 = F1 03 0D 0A
   * Right motor stop:                       10110000                = B0 0D 0A
   * Right motor set speed:               11111000 0nnnnnnn 0nnnnnnn  = F8 nn nn nn 0D 0A
   * Right motor set current position: 11110100 0nnnnnnn 0nnnnnnn  = F4 nn nn nn 0D 0A
   * Right motor set target position:   11110010 0nnnnnnn 0nnnnnnn  = F2 nn nn nn 0D 0A
  
   * Rotation servo set position:        10x10101 0nnnnnnn 0nnnnnnn  = 95 nn nn nn 0D 0A
   * Rotation servo set speed:           10x11100 0nnnnnnn 0nnnnnnn  = 9C nn nn nn 0D 0A

* Replying commands:
   * Left motor get state:                  10000001 = 81 0D 0A 
      * ->> (returnerer) 11000001 0xxxxx<direction><on|off> = C1 nn 0D 0A
   * Left motor get target position:        10000010 = 82 0D 0A
      * ->> 11000010 0nnnnnnn 0nnnnnnn = C2 nn nn nn 0D 0A
   * Left motor get current position:     10000100 = 84 0D 0A
      * ->> 11000100 0nnnnnnn 0nnnnnnn = C4 nn nn nn 0D 0A
   * Left motor get speed:                 10001000 = 88 0D 0A
      * ->> 11001000 0nnnnnnn 0nnnnnnn = C8 nn nn nn 0D 0A
  
   * Right motor get state:                  10100001 = A1 0D 0A 
      * ->> (returnerer) 11100001 0xxxxx<direction><on|off> = E1 nn 0D 0A
   * Right motor get target position:        10100010 = A2 0D 0A
      * ->> 11100010 0nnnnnnn 0nnnnnnn = E2 nn nn nn 0D 0A
   * Right motor get current position:     10100100 = A4 0D 0A
      * ->> 11100100 0nnnnnnn 0nnnnnnn = E4 nn nn nn 0D 0A
   * Right motor get speed:                 10101000 = A8 0D 0A
      * ->> 11101000 0nnnnnnn 0nnnnnnn = E8 nn nn nn 0D 0A

   * Servo motor get speed:               10x01100 = 8C 0D 0A
      * ->> 11x01100 0nnnnnnn 0nnnnnnn = CC nn nn nn 0D 0A
   * Servo motor get position:            10x00101 = 85 0D 0A
      * ->> 11x00101 0nnnnnnn 0nnnnnnn = C5 nn nn nn 0D 0A

