/*
Copyright (c) 2015 Michael C. Miller. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#if defined(ESP8266)

#include <Arduino.h>
#include <ServoLLeo.h>

#define INVALID_SERVO         255     // flag indicating an invalid servo index

const uint32_t c_CycleCompensation = 4;  // compensation us to trim adjust for digitalWrite delays



#define INVALID_PIN           63    // flag indicating never attached servo

struct ServoInfo {
    uint8_t pin : 6;             // a pin number from 0 to 62, 63 reserved
    uint8_t isActive : 1;        // true if this channel is enabled, pin not pulsed if false
    uint8_t isDetaching : 1;     // true if this channel is being detached, maintains pulse integrity
    uint8_t degree : 8;   // position in degree
    uint16_t speed : 16;    // speed
    uint16_t defSpeed : 16;    // speedStep
    uint16_t _minUs : 16;
    uint16_t _maxUs : 16;
    uint16_t waitDefault : 16;
    uint16_t wait : 16;
};

struct ServoState {
    ServoInfo info;
    volatile uint16_t usPulse;
    volatile uint16_t usPulseDestination;
};

#if !defined (SERVO_EXCLUDE_TIMER0)
ServoTimer0 s_servoTimer0;
#endif

#if !defined (SERVO_EXCLUDE_TIMER1)
ServoTimer1 s_servoTimer1;
#endif

static ServoState s_servos[MAX_SERVOS];     // static array of servo structures

static uint8_t s_servoCount = 0;            // the total number of attached s_servos

// inconvenience macros
#define SERVO_INDEX_TO_TIMER(servoIndex) ((ServoTimerSequence)(servoIndex / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX(timerId, channel) ((timerId * SERVOS_PER_TIMER) + channel)     // macro to access servo index by timer and channel

// similiar to map but will have increased accuracy that provides a more
// symetric api (call it and use result to reverse will provide the original value)
// 
int improved_map(int value, int minIn, int maxIn, int minOut, int maxOut) {
    const int rangeIn = maxIn - minIn;
    const int rangeOut = maxOut - minOut;
    const int deltaIn = value - minIn;
    // fixed point math constants to improve accurancy of divide and rounding
    const int fixedHalfDecimal = 1;
    const int fixedDecimal = fixedHalfDecimal * 2;

    return ((deltaIn * rangeOut * fixedDecimal) / (rangeIn) + fixedHalfDecimal) / fixedDecimal + minOut;
}

//------------------------------------------------------------------------------
// Interrupt handler template method that takes a class that implements
// a standard set of methods for the timer abstraction
//------------------------------------------------------------------------------
template <class T>
static void Servo_Handler(T* timer) ICACHE_RAM_ATTR;

template <class T>
static void Servo_Handler(T* timer) {
    uint8_t servoIndex;
    timer->ResetInterrupt(); // clear interrupt

    if(timer->isEndOfCycle()) timer->StartCycle();
    else {
        servoIndex = SERVO_INDEX(timer->timerId(), timer->getCurrentChannel());
        if(servoIndex < s_servoCount && s_servos[servoIndex].info.isActive) { // OK
            digitalWrite(s_servos[servoIndex].info.pin, LOW); // pulse this channel low if activated
            if(s_servos[servoIndex].info.isDetaching) { s_servos[servoIndex].info.isActive = false; s_servos[servoIndex].info.isDetaching = false; }
        }
        timer->nextChannel();
    }

    servoIndex = SERVO_INDEX(timer->timerId(), timer->getCurrentChannel());

    if(servoIndex < s_servoCount && timer->getCurrentChannel() < SERVOS_PER_TIMER) { // < 12 = first timer

	if(s_servos[servoIndex].info.speed) timer->SetPulseCompare(timer->usToTicks(s_servos[servoIndex].usPulse) - c_CycleCompensation);
        else timer->SetPulseCompare(timer->usToTicks(s_servos[servoIndex].usPulseDestination) - c_CycleCompensation);

        if(s_servos[servoIndex].info.isActive) {
            if(s_servos[servoIndex].info.isDetaching) { s_servos[servoIndex].info.isActive = false; s_servos[servoIndex].info.isDetaching = false; } // it was active, reset state and leave low
            else {

	
		// cycle High
		// -----------------------------------------------------------------------------
	    
		if(s_servos[servoIndex].info.speed) { // SPEED defined ?
			if(s_servos[servoIndex].info.wait) { // before stop
			    s_servos[servoIndex].info.wait--;
			    if(!s_servos[servoIndex].info.wait) s_servos[servoIndex].info.isDetaching = true;
			}

			if(s_servos[servoIndex].usPulse < s_servos[servoIndex].usPulseDestination) { // +
				s_servos[servoIndex].usPulse+=s_servos[servoIndex].info.speed;
				if(s_servos[servoIndex].usPulse >= s_servos[servoIndex].usPulseDestination) {
				    s_servos[servoIndex].usPulse = s_servos[servoIndex].usPulseDestination;
				    s_servos[servoIndex].info.wait=s_servos[servoIndex].info.waitDefault;
				}
			} else if(s_servos[servoIndex].usPulse > s_servos[servoIndex].usPulseDestination) { // -
				s_servos[servoIndex].usPulse-=s_servos[servoIndex].info.speed;
				if(s_servos[servoIndex].usPulse <= s_servos[servoIndex].usPulseDestination) {
				    s_servos[servoIndex].usPulse = s_servos[servoIndex].usPulseDestination;
				    s_servos[servoIndex].info.wait=s_servos[servoIndex].info.waitDefault;
				}
			}

		}

		// -----------------------------------------------------------------------------
	
		digitalWrite(s_servos[servoIndex].info.pin, HIGH); // its an active channel so pulse it high
	    }
        }

    } else {

        if(!isTimerActive(timer->timerId())) finISR(timer->timerId()); // no active running channels on this timer, stop the ISR

        else { // finished all channels so wait for the refresh period to expire before starting over allow a few ticks to ensure the next match is not missed
            uint32_t refreshCompare = timer->usToTicks(REFRESH_INTERVAL);
            if ((timer->GetCycleCount() + c_CycleCompensation * 2) < refreshCompare) timer->SetCycleCompare(refreshCompare - c_CycleCompensation);
            else timer->SetCycleCompare(timer->GetCycleCount() + c_CycleCompensation * 2); // at least REFRESH_INTERVAL has elapsed
        }
        timer->setEndOfCycle();

    }
}

static void handler0() ICACHE_RAM_ATTR;
static void handler0() { Servo_Handler<ServoTimer0>(&s_servoTimer0); }

static void handler1() ICACHE_RAM_ATTR;
static void handler1() { Servo_Handler<ServoTimer1>(&s_servoTimer1); }

static void initISR(ServoTimerSequence timerId) {
#if !defined (SERVO_EXCLUDE_TIMER0)
    if (timerId == ServoTimerSequence_Timer0) s_servoTimer0.InitInterrupt(&handler0);
#endif
#if !defined (SERVO_EXCLUDE_TIMER1)
    if (timerId == ServoTimerSequence_Timer1) s_servoTimer1.InitInterrupt(&handler1);
#endif
}

static void finISR(ServoTimerSequence timerId) ICACHE_RAM_ATTR;
static void finISR(ServoTimerSequence timerId) {
#if !defined (SERVO_EXCLUDE_TIMER0)
    if (timerId == ServoTimerSequence_Timer0) s_servoTimer0.StopInterrupt();
#endif
#if !defined (SERVO_EXCLUDE_TIMER1)
    if (timerId == ServoTimerSequence_Timer1) s_servoTimer1.StopInterrupt();
#endif
}

// returns true if any servo is active on this timer
static boolean isTimerActive(ServoTimerSequence timerId) ICACHE_RAM_ATTR;
static boolean isTimerActive(ServoTimerSequence timerId) {
    for (uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) { if (s_servos[SERVO_INDEX(timerId, channel)].info.isActive) return true; }
    return false;
}

//-------------------------------------------------------------------
// Servo non-class methods

uint8_t ServoAttach(uint8_t n, int pin) { return ServoAttach(n,pin,s_servos[n].info.defSpeed, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t ServoAttach(uint8_t n, int pin, uint16_t speed) { return ServoAttach(n, pin, speed, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t ServoAttach(uint8_t n, int pin, uint16_t speed, uint16_t minUs, uint16_t maxUs) {
    ServoTimerSequence timerId;

    if(n >= MAX_SERVOS) return n;

    if(s_servos[n].info.pin == INVALID_PIN) {
        pinMode(pin, OUTPUT);       // set servo pin to output
        digitalWrite(pin, LOW);
        s_servos[n].info.pin = pin;
    }

    // keep the min and max within 200-3000 us, these are extreme ranges and should support extreme servos while maintaining reasonable ranges
    s_servos[n].info._maxUs = max((uint16_t)250, min((uint16_t)3000, maxUs));
    s_servos[n].info._minUs = max((uint16_t)200, min(s_servos[n].info._maxUs, minUs));

    // initialize the timerId if it has not already been initialized
    timerId = SERVO_INDEX_TO_TIMER(n);
    if(!isTimerActive(timerId)) initISR(timerId);

    s_servos[n].info.defSpeed = speed; // default Speed
    s_servos[n].info.isDetaching = false;
    s_servos[n].info.isActive = true; // this must be set after the check for isTimerActive
}

void ServoDetach(uint8_t n) { if(s_servos[n].info.isActive) s_servos[n].info.isDetaching = true; }

void ServoWrite(uint8_t n, int degree) { ServoWrite(n,degree,s_servos[n].info.defSpeed); }
void ServoWrite(uint8_t n, int degree, int speed) {
    if(degree >= MIN_PULSE_WIDTH) { ServoWriteMicroseconds(n,degree,255,speed); return; } // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    int value = constrain(degree,0,180); // assumed to be 0-180 degrees servo
    // writeMicroseconds will contrain the calculated value for us for any user defined min and max, but we must use default min max
    value = improved_map(value,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
    ServoWriteMicroseconds(n,value,degree,speed);
}

void ServoWriteMicroseconds(uint16_t n, int value,int degree,int speed) {  // ensure channel is valid
    if((n < MAX_SERVOS)) {
        if(!ServoAttached(n)) ServoAttach(n,s_servos[n].info.pin); // s_servos[_servoIndex].info.pin = INVALID_PIN;
        value = constrain(value, s_servos[n].info._minUs,s_servos[n].info._maxUs); // ensure pulse width is valid
	s_servos[n].info.wait = 0;
        s_servos[n].usPulseDestination = value;
        s_servos[n].info.degree = degree;
        s_servos[n].info.speed = speed;
    }
}

void ServoSetSpeed(uint8_t n,int speed) { s_servos[n].info.defSpeed=speed; }

void ServoWaitDefault(uint8_t n, uint16_t wait) { s_servos[n].info.waitDefault = (wait?wait:DEFAULT_WAIT_BEFORE_STOP); } // set timeout before stop

bool ServoAttached(uint8_t n) { return s_servos[n].info.isActive; }

int ServoReadMicrosecondsReal(uint8_t n) { return (n != INVALID_SERVO ? s_servos[n].usPulse : 0 ); }
int ServoReadMicroseconds(uint8_t n) { return (n != INVALID_SERVO ? s_servos[n].usPulseDestination : 0 ); }

int ServoRead(uint8_t n) { if(n == INVALID_SERVO) return 0;
    if(s_servos[n].info.degree!=255) return s_servos[n].info.degree;
    return improved_map(ServoReadMicroseconds(n), MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180); // calculate angle for an assumed 0-180
}

int ServoReadReal(uint8_t n) { return (n == INVALID_SERVO ? 0 : improved_map(ServoReadMicrosecondsReal(n), MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 180) ); }

//-------------------------------------------------------------------
// Servo class methods

Servo::Servo() {
    if(s_servoCount < MAX_SERVOS) {
        _servoIndex = s_servoCount++; // assign a servo index to this instance
        s_servos[_servoIndex].usPulse = DEFAULT_PULSE_WIDTH; // store default values
        s_servos[_servoIndex].usPulseDestination = DEFAULT_PULSE_WIDTH; // store default values

        s_servos[_servoIndex].info.waitDefault = DEFAULT_WAIT_BEFORE_STOP; // wait before stop
        s_servos[_servoIndex].info.wait = 0;

        // set default _minUs and _maxUs incase write() is called before attach()
        s_servos[_servoIndex].info._minUs = MIN_PULSE_WIDTH;
        s_servos[_servoIndex].info._maxUs = MAX_PULSE_WIDTH;

        s_servos[_servoIndex].info.isActive = false;
        s_servos[_servoIndex].info.isDetaching = false;
        s_servos[_servoIndex].info.pin = INVALID_PIN;
        s_servos[_servoIndex].info.speed = 0;
        s_servos[_servoIndex].info.defSpeed = 0;
        s_servos[_servoIndex].info.degree = 0;
    }
    else _servoIndex = INVALID_SERVO;  // too many servos
}

uint8_t Servo::attach(int pin) { return ServoAttach(_servoIndex,pin, s_servos[_servoIndex].info.defSpeed, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t Servo::attach(int pin, uint16_t speed) { return ServoAttach(_servoIndex,pin, speed, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t Servo::attach(int pin, uint16_t minUs, uint16_t maxUs) { return ServoAttach(_servoIndex,pin, s_servos[_servoIndex].info.defSpeed , MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }
uint8_t Servo::attach(int pin, uint16_t speed, uint16_t minUs, uint16_t maxUs) { return ServoAttach(_servoIndex,pin, speed, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH); }

void Servo::detach() { ServoDetach(_servoIndex); }

void Servo::write(int degree) { ServoWrite(_servoIndex,degree,s_servos[_servoIndex].info.defSpeed); }
void Servo::write(int degree,int speed) { ServoWrite(_servoIndex,degree,speed); }

void Servo::writeMicroseconds(int value) { ServoWriteMicroseconds(_servoIndex,value,255,s_servos[_servoIndex].info.defSpeed); }
void Servo::writeMicroseconds(int value,int speed) { ServoWriteMicroseconds(_servoIndex,value,255,speed); }
void Servo::writeMicroseconds(int value,int degree,int speed) { ServoWriteMicroseconds(_servoIndex,value,degree,speed); }

void Servo::setSpeed(int speed) { ServoSetSpeed(_servoIndex,speed); }

int Servo::read() { return ServoRead(_servoIndex); }
int Servo::readReal() { return ServoReadReal(_servoIndex); }

int Servo::readMicroseconds() { return ServoReadMicroseconds(_servoIndex); }
int Servo::readMicrosecondsReal() { return ServoReadMicrosecondsReal(_servoIndex); }

bool Servo::attached() { return ServoAttached(_servoIndex); }

#endif

