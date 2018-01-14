#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  #if !defined(HWPWM6)
    #if defined(TEENSY20)
      uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
    #elif defined(A32U4_4_HW_PWM_SERVOS)
      uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
    #else
    uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #endif
  #else
    #if defined(TEENSY20)
      uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
    #elif defined(A32U4_4_HW_PWM_SERVOS)
      uint8_t PWM_PIN[8] = {6,9,10,11,5,13,SW_PWM_P3,SW_PWM_P4};   //
    #else
      uint8_t PWM_PIN[8] = {9,10,5,6,11,13,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #endif
  #endif
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H flat for promini
    volatile uint8_t atomicPWM_PIN5_lowState;
    volatile uint8_t atomicPWM_PIN5_highState;
    volatile uint8_t atomicPWM_PIN6_lowState;
    volatile uint8_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on promini
    volatile uint8_t atomicPWM_PINA2_lowState;
    volatile uint8_t atomicPWM_PINA2_highState;
    volatile uint8_t atomicPWM_PIN12_lowState;
    volatile uint8_t atomicPWM_PIN12_highState;
  #endif
#else
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X/HEX6H and for Promicro
    volatile uint16_t atomicPWM_PIN5_lowState;
    volatile uint16_t atomicPWM_PIN5_highState;
    volatile uint16_t atomicPWM_PIN6_lowState;
    volatile uint16_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on Promicro
    volatile uint16_t atomicPWM_PINA2_lowState;
    volatile uint16_t atomicPWM_PINA2_highState;
    volatile uint16_t atomicPWM_PIN12_lowState;
    volatile uint16_t atomicPWM_PIN12_highState;
  #endif
#endif

#if defined(SERVO)
  #if defined(HW_PWM_SERVOS)
    // hw servo pwm does not need atomicServo[]
  #elif defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
    #if defined(AIRPLANE) || defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
    #else
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    #endif
  #else
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320};
    #else
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
    #endif
  #endif
#endif

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
  #if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
    #if PRI_SERVO_FROM < SEC_SERVO_FROM
      #define SERVO_START PRI_SERVO_FROM
    #else
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #else
    #if defined(PRI_SERVO_FROM)
      #define SERVO_START PRI_SERVO_FROM
    #endif
    #if defined(SEC_SERVO_FROM)
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #endif
  #if defined(PRI_SERVO_TO) && defined(SEC_SERVO_TO)
    #if PRI_SERVO_TO > SEC_SERVO_TO
      #define SERVO_END PRI_SERVO_TO
    #else
      #define SERVO_END SEC_SERVO_TO
    #endif
  #else
    #if defined(PRI_SERVO_TO)
      #define SERVO_END PRI_SERVO_TO
    #endif
    #if defined(SEC_SERVO_TO)
      #define SERVO_END SEC_SERVO_TO
    #endif
  #endif


#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    #if defined(PRI_SERVO_FROM) && !defined(HW_PWM_SERVOS)   // write primary servos
      for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
        #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
          atomicServo[i] = (servo[i]-1000)>>2;
        #else
          atomicServo[i] = (servo[i]-1000)<<4;
        #endif
      }
    #endif
    #if defined(SEC_SERVO_FROM) && !defined(HW_PWM_SERVOS)  // write secundary servos
      #if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)) && defined(MMSERVOGIMBAL)
        // Moving Average Servo Gimbal by Magnetron1
        static int16_t mediaMobileServoGimbalADC[3][MMSERVOGIMBALVECTORLENGHT];
        static int32_t mediaMobileServoGimbalADCSum[3];
        static uint8_t mediaMobileServoGimbalIDX;
        uint8_t axis;

        mediaMobileServoGimbalIDX = ++mediaMobileServoGimbalIDX % MMSERVOGIMBALVECTORLENGHT;
        for (axis=(SEC_SERVO_FROM-1); axis < SEC_SERVO_TO; axis++) {
          mediaMobileServoGimbalADCSum[axis] -= mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX] = servo[axis];
          mediaMobileServoGimbalADCSum[axis] += mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)>>2;
          #else
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)<<4;
          #endif
        }
      #else
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
    // write HW PWM servos for the mega
    #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        OCR5C = servo[0];
      #endif 
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        OCR5B = servo[1];
      #endif 
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        OCR5A = servo[2];
      #endif 
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        OCR1A = servo[3];
      #endif 
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        OCR1B = servo[4];
      #endif 
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        OCR4A = servo[5];
      #endif 
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        OCR4B = servo[6];
      #endif 
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        OCR4C = servo[7];
      #endif
    #endif
    // write HW PWM servos for the promicro
    #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7)
        OCR1A = servo[6];// Pin 9
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5)
        OCR1B = servo[4];// Pin 10
      #endif
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6)
        OCR3A = servo[5];// Pin 5
      #endif
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4)
        OCR1C = servo[3];// Pin 11
      #endif
    #endif
  #endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  /****************  Specific PWM Timers & Registers for the MEGA's   *******************/
  #if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3
      #else
        OCR3C = ((motor[0]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5
      #else
        OCR3A = ((motor[1]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6
      #else
        OCR4A = ((motor[2]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2
      #else
        OCR3B = ((motor[3]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #ifndef EXT_MOTOR_RANGE 
        OCR4B = motor[4]<<3; //  pin 7
        OCR4C = motor[5]<<3; //  pin 8
      #else
        OCR4B = ((motor[4]<<4) - 16000);
        OCR4C = ((motor[5]<<4) - 16000);
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[6]>>3; //  pin 9
        OCR2A = motor[7]>>3; //  pin 10
      #else
        OCR2B = (motor[6]>>2) - 250;
        OCR2A = (motor[7]>>2) - 250;
      #endif
    #endif
  #endif

  /******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    uint16_t Temp2;
    Temp2 = motor[3] - 1000;
    #if (NUMBER_MOTOR > 0)
      #if defined(A32U4_4_HW_PWM_SERVOS)
        // write motor0 to pin 6
        // Timer 4 A & D [1000:2000] => [1000:2000]
        #ifndef EXT_MOTOR_RANGE
          TC4H = motor[0]>>8; OCR4D = (motor[0]&0xFF); //  pin 6
        #else
          TC4H = (((motor[0]-1000)<<1)+16)>>8; OCR4D = ((((motor[0]-1000)<<1)+16)&0xFF); //  pin 6
        #endif
      #else
        // Timer 1 A & B [1000:2000] => [8000:16000]
        #ifdef EXT_MOTOR_RANGE
          OCR1A = ((motor[0]<<4) - 16000) + 128;
        #elif defined(EXT_MOTOR_64KHZ)
          OCR1A = (motor[0] - 1000) >> 2; // max = 255
        #elif defined(EXT_MOTOR_32KHZ)
          OCR1A = (motor[0] - 1000) >> 1; // max = 511
        #elif defined(EXT_MOTOR_16KHZ)
          OCR1A = motor[0] - 1000;        //  pin 9
        #elif defined(EXT_MOTOR_8KHZ)
          OCR1A = (motor[0]-1000) << 1;   //  pin 9
        #else
          OCR1A = motor[0]<<3; //  pin 9
        #endif
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifdef EXT_MOTOR_RANGE 
        OCR1B = ((motor[1]<<4) - 16000) + 128;
      #elif defined(EXT_MOTOR_64KHZ)
        OCR1B = (motor[1] - 1000) >> 2;
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1B = (motor[1] - 1000) >> 1;
      #elif defined(EXT_MOTOR_16KHZ)
        OCR1B = motor[1] - 1000;        //  pin 10
      #elif defined(EXT_MOTOR_8KHZ)
        OCR1B = (motor[1]-1000) << 1;   //  pin 10
      #else
        OCR1B = motor[1]<<3; //  pin 10
      #endif
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        #ifndef EXT_MOTOR_RANGE 
          TC4H = (2047-motor[2])>>8; OCR4A = ((2047-motor[2])&0xFF); //  pin 5
        #else
          TC4H = 2047-(((motor[2]-1000)<<1)+16)>>8; OCR4A = (2047-(((motor[2]-1000)<<1)+16)&0xFF); //  pin 5
        #endif
      #else
        #ifdef EXT_MOTOR_RANGE 
          OCR3A = ((motor[2]<<4) - 16000) + 128;
        #elif defined(EXT_MOTOR_64KHZ)
          OCR3A = (motor[2] - 1000) >> 2;
        #elif defined(EXT_MOTOR_32KHZ)
          OCR3A = (motor[2] - 1000) >> 1;
        #elif defined(EXT_MOTOR_16KHZ)
          OCR3A = motor[2] - 1000;        //  pin 5
        #elif defined(EXT_MOTOR_8KHZ)
          OCR3A = (motor[2]-1000) << 1;   //  pin 5
        #else
          OCR3A = motor[2]<<3; //  pin 5
        #endif
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifdef EXT_MOTOR_RANGE 
        TC4H = (((motor[3]-1000)<<1)+16)>>8; OCR4D = ((((motor[3]-1000)<<1)+16)&0xFF); //  pin 6
      #elif defined(EXT_MOTOR_64KHZ)
        Temp2 = Temp2 >> 2;
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_32KHZ)
        Temp2 = Temp2 >> 1;
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_16KHZ)
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #elif defined(EXT_MOTOR_8KHZ)
        TC4H = Temp2 >> 8;
        OCR4D = Temp2 & 0xFF;           //  pin 6
      #else
        TC4H = motor[3]>>8; OCR4D = (motor[3]&0xFF); //  pin 6
      #endif
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        #if (NUMBER_MOTOR == 6) && !defined(SERVO)
          atomicPWM_PIN5_highState = motor[4]<<3;
          atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = motor[5]<<3;
          atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;      
        #else
          atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
          atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
          atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;        
        #endif
      #else
        #ifndef EXT_MOTOR_RANGE 
          OCR1C = motor[4]<<3; //  pin 11
          TC4H = motor[5]>>8; OCR4A = (motor[5]&0xFF); //  pin 13  
        #else
          OCR1C = ((motor[4]<<4) - 16000) + 128;
          TC4H = (((motor[5]-1000)<<1)+16)>>8; OCR4A = ((((motor[5]-1000)<<1)+16)&0xFF); //  pin 13       
        #endif  
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if !defined(HWPWM6)
        atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
        atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
        atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
      #else
        atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
        atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
        atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;     
      #endif
    #endif
  #endif

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

