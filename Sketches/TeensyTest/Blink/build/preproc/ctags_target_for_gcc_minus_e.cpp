# 1 "/home/eirik/git/eirikblekesaune/PinneRobot/src/TeensyTest/Blink/Blink.pde"
/* LED Blink, Teensyduino Tutorial #1
   http://www.pjrc.com/teensy/tutorial.html
 
   This example code is in the public domain.
*/

// Teensy 2.0 has the LED on pin 11
// Teensy++ 2.0 has the LED on pin 6
// Teensy 3.x / Teensy LC have the LED on pin 13
const int ledPin = 13;

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.
  pinMode(ledPin, 1);
}

// the loop() methor runs over and over again,
// as long as the board has power

int time = 50;
void loop() {
  digitalWrite(ledPin, 1); // set the LED on
  delay(time); // wait for a second
  digitalWrite(ledPin, 0); // set the LED off
  delay(time); // wait for a second
}
