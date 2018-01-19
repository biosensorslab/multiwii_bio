#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);


/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR1A = ((motor[0]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1A = (motor[0] - 1000) >> 2; //  pin 9
      #elif defined(EXT_MOTOR_4KHZ)
        OCR1A = (motor[0] - 1000) << 1;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR1A = (motor[0] - 1000) << 3;
      #else
        OCR1A = motor[0]>>3; //  pin 9
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR1B = ((motor[1]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1B = (motor[1] - 1000) >> 2; //  pin 10
      #elif defined(EXT_MOTOR_4KHZ)
        OCR1B = (motor[1] - 1000) << 1;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR1B = (motor[1] - 1000) << 3;
      #else
        OCR1B = motor[1]>>3; //  pin 10
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR2A = ((motor[2]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      #elif defined(EXT_MOTOR_4KHZ)
        OCR2A = (motor[2] - 1000) >> 2;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR2A = (motor[2] - 1000) >> 2;
      #else
        OCR2A = motor[2]>>3; //  pin 11
      #endif
    #endif
    OCR2B = motor[3]>>3; //  pin 3

  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
	/****************            mark all PWM pins as Output             ******************/
	for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
		pinMode(PWM_PIN[i],OUTPUT);
	}
	TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
	TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
	TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
	TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
  
	writeAllMotors(MINCOMMAND);
	delay(300);
}


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

  /****************                   main Mix Table                ******************/
  //20170826
  motor[0] = PIDMIX(-1,+1,-1); //REAR_R
  motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
  motor[2] = PIDMIX(+1,+1,+1); //REAR_L
  motor[3] = PIDMIX(+1,-1,-1); //FRONT_L

  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++)
    {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;

      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
    	  motor[i] = conf.minthrottle;
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
}

