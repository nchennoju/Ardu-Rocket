/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo ms1;
Servo ms2;
Servo ms3;
Servo ms4;

int pos = 0;    // variable to store the servo position

void setup() {
  ms1.attach(9);
  ms2.attach(10);
  ms3.attach(11); //done
  ms4.attach(12); //done
}

void loop() {
  ms1.write(81);
  ms2.write(97);
  ms3.write(83);
  ms4.write(90);
  /*for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }*/
}

