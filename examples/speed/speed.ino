/* speed
 by BARRAGAN <http://barraganstudio.com> This example code is in the public domain.
 modified 28 May 2015 by Michael C. Miller
 modified 8 Nov 2013 by Scott Fitzgerald
 modifed 29 Jul 2018 by LLeo Kaganov lleo@lleo.me
*/

#include <ServoLLeo.h> 
 
Servo myservo0;
Servo myservo1;
Servo myservo2;
 
void setup() { 
  myservo0.attach(2); // servo №0 - GPIO02
  myservo1.attach(4); // servo №1
  myservo1.attach(5); // servo №2

  myservo1.write(90); // Old references are supported
  myservo2.write(110,2); // Go to 110 degree, speed = 1 (1 ... 100)

  delay(2000);

  ServoSetSpeed(0,5); // Set default speed 5 for servos 0,1,2
  ServoSetSpeed(1,5);
  ServoSetSpeed(2,5);

} 
 
void loop() {

    for(var i=0;i<3;i++) ServoWrite(i,10); // go to 10 degree slowly (default speed was set 5)
    delay(2000);

    for(var i=0;i<3;i++) ServoWrite(i, 170, 50); // go to 170 degree quickly: speed 50
    delay(2000);

}
