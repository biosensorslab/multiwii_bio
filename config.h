#define CONFIG_H_

/*************************************************************************************************/
/****           CONFIGURABLE PARAMETERS                                                       ****/
/*************************************************************************************************/

/* this file consists of several sections
* to create a working combination you must at least make your choices in section 1.
* 1 - BASIC SETUP - you must select an option in every block.
*      this assumes you have 4 channels connected to your board with standard ESCs and servos.
* 2 - COPTER TYPE SPECIFIC OPTIONS - you likely want to check for options for your copter type
* 3 - RC SYSTEM SETUP
* 4 - ALTERNATE CPUs & BOARDS - if you have
* 5 - ALTERNATE SETUP - select alternate RX (SBUS, PPM, etc.), alternate ESC-range, etc. here
* 6 - OPTIONAL FEATURES - enable nice to have features here (FlightModes, LCD, telemetry, battery monitor etc.)
* 7 - TUNING & DEVELOPER - if you know what you are doing; you have been warned
*     - (ESCs calibration, Dynamic Motor/Prop Balancing, Diagnostics,Memory savings.....)
* 8 - DEPRECATED - these features will be removed in some future release
*/

/* Notes:
* 1. parameters marked with (*) in the comment are stored in eeprom and can be changed via serial monitor or LCD.
* 2. parameters marked with (**) in the comment are stored in eeprom and can be changed via the GUI
*/


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  1 - BASIC SETUP                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/**************************    The type of multicopter    ****************************/
#define QUADX


/****************************    Motor minthrottle    *******************************/
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
This is the minimum value that allow motors to run at a idle speed  */
#define MINTHROTTLE 1180 // (*) (**)

/****************************    Motor maxthrottle    *******************************/
/* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
#define MAXTHROTTLE 1800//Throttle Stick 최대일 때의 출력 상한 설정.

/****************************    Mincommand          *******************************/
/* this is the value for the ESCs when they are not armed
in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
#define MINCOMMAND  1000

/**********************************  I2C speed for old WMP config (useless config for other sensors)  *************/
#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
#define LOOP_TIME 2800

/**************************************************************************************/
/*****************          boards and sensor definitions            ******************/
/**************************************************************************************/

/***************************    Combined IMU Boards    ********************************/
/* if you use a specific sensor board:
please submit any correction to this list.
Note from Alex: I only own some boards, for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
#define GY_521          // Chinese 6  DOF with  MPU6050, LLC

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  2 - COPTER TYPE SPECIFIC OPTIONS                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/
/********************************  PID Controller *********************************/
/* choose one of the alternate PID control algorithms
* 1 = evolved oldschool algorithm (similar to v2.2)
* 2 = new experimental algorithm from Alex Khoroshko - unsupported - http://www.multiwii.com/forum/viewtopic.php?f=8&t=3671&start=10#p37387
* */
#define PID_CONTROLLER 1

/* NEW: not used anymore for servo coptertypes  <== NEEDS FIXING - MOVE TO WIKI */
#define YAW_DIRECTION 1
//#define YAW_DIRECTION -1 // if you want to reverse the yaw correction direction

#define ONLYARMWHENFLAT //prevent the copter from arming when the copter is tilted

/********************************    ARM/DISARM    *********************************/
/* optionally disable stick combinations to arm/disarm the motors.
* In most cases one of the two options to arm/disarm via TX stick is sufficient */
#define ALLOW_ARM_DISARM_VIA_TX_YAW
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL

/********************************    SERVOS      *********************************/
/* info on which servos connect where and how to setup can be found here
* http://www.multiwii.com/wiki/index.php?title=Config.h#Servos_configuration
*/

/* Do not move servos if copter is unarmed
* It is a quick hack to overcome feedback tail wigglight when copter has a flexibile
* landing gear
*/
//#define DISABLE_SERVOS_WHEN_UNARMED


/* if you want to preset min/middle/max values for servos right after flashing, because of limited physical
* room for servo travel, then you must enable and set all three following options */

/***********************          Cam Stabilisation             ***********************/
/* The following lines apply only for a pitch/roll tilt stabilization system. Uncomment the first or second line to activate it */

/* camera trigger function : activated via Rc Options in the GUI, servo output=A2 on promini */
// trigger interval can be changed via (*GUI*) or via AUX channel
#define CAM_TIME_HIGH 1000   // the duration of HIGH state servo expressed in ms

/***********************          Airplane                       ***********************/
//#define USE_THROTTLESERVO // For use of standard 50Hz servo on throttle.
#define FLAPPERON_EP   { 1500, 1700 } // Endpooints for flaps on a 2 way switch else set {1020,2000} and program in radio.
#define FLAPPERON_INVERT { -1, 1 }    // Change direction om flapperons { Wing1, Wing2 }

/***********************      Common for Heli & Airplane         ***********************/

/* Governor: attempts to maintain rpm through pitch and voltage changes
* predictive approach: observe input signals and voltage and guess appropriate corrections.
* (the throttle curve must leave room for the governor, so 0-50-75-80-80 is ok, 0-50-95-100-100 is _not_ ok.
* Can be toggled via aux switch.
*/
/* tail precomp from collective */
#define YAW_COLL_PRECOMP 10           // (*) proportional factor in 0.1. Higher value -> higher precomp effect. value of 10 equals no/neutral effect
#define YAW_COLL_PRECOMP_DEADBAND 120 // (*) deadband for collective pitch input signal around 0-pitch input value

/***********************          Heli                           ***********************/
/* Channel to control CollectivePitch */
#define COLLECTIVE_PITCH      THROTTLE

/* Limit the range of Collective Pitch. 100% is Full Range each way and position for Zero Pitch */
#define COLLECTIVE_RANGE { 80, 0, 80 }// {Min%, ZeroPitch offset from 1500, Max%}.
#define YAWMOTOR                 0       // If a motor is used as YAW Set to 1 else set to 0.

/* Servo mixing for heli 120
{Coll,Nick,Roll} */
#define SERVO_NICK   { +10, -10,  0 }
#define SERVO_LEFT   { +10, +5, +10 }
#define SERVO_RIGHT  { +10, +5, -10 }

/* Limit Maximum controll for Roll & Nick  in 0-100% */
#define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }



/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  3 - RC SYSTEM SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/* note: no need to uncomment something in this section if you use a standard receiver */

/****************************    EXTENDED AUX STATES    ***********************************/
/* If you uncomment this line, you can use six states for each of the aux channels (AUX1-AUX4)
to control your copter.
Channel values
1000-1230
1231-1360
1361-1490
1491-1620
1621-1749
1750-
At this moment you can use this function only with WinGUI 2.3 release. MultiWiiConf does not support it yet
*/


/**************************************************************************************/
/********                       special receiver types             ********************/
/**************************************************************************************/
/*******************************    SBUS RECIVER    ************************************/
/* The following line apply only for Futaba S-Bus Receiver on MEGA boards or PROMICRO boards.
You have to invert the S-Bus-Serial Signal e.g. with a Hex-Inverter like IC SN74 LS 04 */
#define SBUS_MID_OFFSET 988 //SBUS Mid-Point at 1500

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  5 - ALTERNATE SETUP                                            *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/******                Serial com speed    *********************************/
/* This is the speed of the serial interfaces */
#define SERIAL0_COM_SPEED 115200
#define SERIAL1_COM_SPEED 115200
#define SERIAL2_COM_SPEED 115200
#define SERIAL3_COM_SPEED 115200

/* when there is an error on I2C bus, we neutralize the values during a short time. expressed in microseconds
it is relevent only for a conf with at least a WMP */
#define NEUTRALIZE_DELAY 100000


/********                          Failsafe settings                 ********************/
/* Failsafe check pulses on four main control channels CH1-CH4. If the pulse is missing or bellow 985us (on any of these four channels)
the failsafe procedure is initiated. After FAILSAFE_DELAY time from failsafe detection, the level mode is on (if ACC is avaliable),
PITCH, ROLL and YAW is centered and THROTTLE is set to FAILSAFE_THROTTLE value. You must set this value to descending about 1m/s or so
for best results. This value is depended from your configuration, AUW and some other params.  Next, after FAILSAFE_OFF_DELAY the copter is disarmed,
and motors is stopped. If RC pulse coming back before reached FAILSAFE_OFF_DELAY time, after the small quard time the RC control is returned to normal. */
//#define FAILSAFE                                // uncomment  to activate the failsafe function
#define FAILSAFE_DELAY     10                     // Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAFE_OFF_DELAY 200                    // Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

#define FAILSAFE_DETECT_TRESHOLD  985



/**************************************************************************************/
/***********************                  GPS                **************************/
/**************************************************************************************/

/* GPS using a SERIAL port
if enabled, define here the Arduino Serial port number and the UART speed
note: only the RX PIN is used in case of NMEA mode, the GPS is not configured by multiwii
in NMEA mode the GPS must be configured to output GGA and RMC NMEA sentences (which is generally the default conf for most GPS devices)
at least 5Hz update rate. uncomment the first line to select the GPS serial port of the arduino */

// avoid using 115200 baud because with 16MHz arduino the 115200 baudrate have more than 2% speed error (57600 have 0.8% error)
#define GPS_BAUD   57600       // GPS_BAUD will override SERIALx_COM_SPEED for the selected port


/* indicate a valid GPS fix with at least 5 satellites by flashing the LED  - Modified by MIS - Using stable LED (YELLOW on CRIUS AIO) led work as sat number indicator
- No GPS FIX -> LED blink at speed of incoming GPS frames
- Fix and sat no. bellow 5 -> LED off
- Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ... */
#define GPS_LED_INDICATOR

//Enables the MSP_WP command set , which is used by WinGUI for displaying an setting up navigation
//#define USE_MSP_WP

// HOME position is reset at every arm, uncomment it to prohibit it (you can set home position with GyroCalibration)
//#define DONT_RESET_HOME_AT_ARM

/* GPS navigation can control the heading */
// copter faces toward the navigation point, maghold must be enabled for it
#define NAV_CONTROLS_HEADING       1    //(**)
// true - copter comes in with tail first
#define NAV_TAIL_FIRST             0    //(**)
// true - when copter arrives to home position it rotates it's head to takeoff direction
#define NAV_SET_TAKEOFF_HEADING    1    //(**)

/* Get your magnetic declination from here : http://magnetic-declination.com/
Convert the degree+minutes into decimal degree by ==> degree+minutes*(1/60)
Note the sign on declination it could be negative or positive (WEST or EAST)
Also note, that maqgnetic declination changes with time, so recheck your value every 3-6 months */
#define MAG_DECLINATION  4.02f   //(**)

// Adds a forward predictive filterig to compensate gps lag. Code based on Jason Short's lead filter implementation
#define GPS_LEAD_FILTER               //(**)

// add a 5 element moving average filter to GPS coordinates, helps eliminate gps noise but adds latency comment out to disable
// use it with NMEA gps only 
//#define GPS_FILTERING                 //(**)

// if we are within this distance to a waypoint then we consider it reached (distance is in cm)
#define GPS_WP_RADIUS              100      //(**)

// Safe WP distance, do not start mission if the first wp distance is larger than this number (in meters)
// Also aborts mission if the next waypoint distance is more than this number
#define SAFE_WP_DISTANCE           500      //(**)

//Maximu allowable navigation altitude (in meters) automatic altitude control will not go above this height
#define MAX_NAV_ALTITUDE           100     //(**)

// minimum speed when approach waypoint
#define NAV_SPEED_MIN              100    // cm/sec //(**)
// maximum speed to reach between waypoints
#define NAV_SPEED_MAX              400    // cm/sec //(**)
// Slow down to zero when reaching waypoint (same as NAV_SPEED_MIN = 0)
#define NAV_SLOW_NAV               0      //(**)
// Weight factor of the crosstrack error in navigation calculations (do not touch)
#define CROSSTRACK_GAIN            .4     //(**)
// Maximum allowable banking than navigation outputs
#define NAV_BANK_MAX 3000                 //(**)

//Defines the RTH altitude. 0 means keep current alt during RTH (in meters)
#define RTH_ALTITUDE               15        //(**)
//Wait to reach RTH alt before start moving to home (0-no, 1-yes)
#define WAIT_FOR_RTH_ALT           1         //(**)

//Navigation engine will takeover BARO mode control
#define NAV_TAKEOVER_BARO          1         //(**)

//Throttle stick input will be ignored  (only in BARO)
#define IGNORE_THROTTLE            1         //(**)

//If FENCE DISTANCE is larger than 0 then copter will switch to RTH when it farther from home
//than the defined number in meters
#define FENCE_DISTANCE      600

//This governs the descent speed during landing. 100 is equals approc 50cm/sec
#define LAND_SPEED          100
/**************************************************************************************/
/***********************        LCD/OLED - display settings       *********************/
/**************************************************************************************/

/* http://www.multiwii.com/wiki/index.php?title=Extra_features#LCD_.2F_OLED */

/*****************************   The type of LCD     **********************************/
/* choice of LCD attached for configuration and telemetry, see notes below */
//#define LCD_DUMMY       // No Physical LCD attached.  With this & LCD_CONF defined, TX sticks still work to set gains, by watching LED blink.
//#define LCD_SERIAL3W    // Alex' initial variant with 3 wires, using rx-pin for transmission @9600 baud fixed
//#define LCD_TEXTSTAR    // SERIAL LCD: Cat's Whisker LCD_TEXTSTAR Module CW-LCD-02 (Which has 4 input keys for selecting menus)
//#define LCD_VT100       // SERIAL LCD: vt100 compatible terminal emulation (blueterm, putty, etc.)
//#define LCD_TTY         // SERIAL LCD: useful to tweak parameters over cable with arduino IDE 'serial monitor'
//#define LCD_ETPP        // I2C LCD: Eagle Tree Power Panel LCD, which is i2c (not serial)
//#define LCD_LCD03       // I2C LCD: LCD03, which is i2c
//#define LCD_LCD03S      // SERIAL LCD: LCD03 whit serial 9600 baud comunication enabled.
//#define OLED_I2C_128x64 // I2C LCD: OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350
//#define OLED_DIGOLE     // I2C OLED from http://www.digole.com/index.php?productID=550

/******************************   Display settings   ***********************************/
#define LCD_SERIAL_PORT 0    // must be 0 on Pro Mini and single serial boards; Set to your choice on any Mega based board

//#define SUPPRESS_OLED_I2C_128x64LOGO  // suppress display of OLED logo to save memory

/* double font height for better readability. Reduces visible #lines by half.
* The lower part of each page is accessible under the name of shifted keyboard letter :
* 1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
* You must add both to your lcd.telemetry.* sequences
*/

/********************************    Navigation     ***********************************/
/* keys to navigate the LCD menu */
#define LCD_MENU_PREV 'p'
#define LCD_MENU_NEXT 'n'
#define LCD_VALUE_UP 'u'
#define LCD_VALUE_DOWN 'd'

#define LCD_MENU_SAVE_EXIT 's'
#define LCD_MENU_ABORT 'x'


/********************************************************************/
/****           battery voltage monitoring                       ****/
/********************************************************************/
/* for V BAT monitoring
after the resistor divisor we should get [0V;5V]->[0;1023] on analog V_BATPIN
with R1=33k and R2=51k
vbat = [0;1023]*16/VBATSCALE
must be associated with #define BUZZER ! */

#define VBATSCALE       131 // (*) (**) change this value if readed Battery voltage is different than real voltage
#define VBATNOMINAL     126 // 12,6V full battery nominal voltage - only used for lcd.telemetry
#define VBATLEVEL_WARN1 107 // (*) (**) 10,7V
#define VBATLEVEL_WARN2  99 // (*) (**) 9.9V
#define VBATLEVEL_CRIT   93 // (*) (**) 9.3V - critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define NO_VBAT          16 // Avoid beeping without any battery
#define VBAT_OFFSET       0 // offset in 0.1Volts, gets added to voltage value  - useful for zener diodes

/* for V BAT monitoring of individual cells
* enable both VBAT and VBAT_CELLS
*/
//#define VBAT_CELLS
#define VBAT_CELLS_NUM 0 // set this to the number of cells you monitor via analog pins
#define VBAT_CELLS_PINS {A0, A1, A2, A3, A4, A5 } // set this to the sequence of analog pins
#define VBAT_CELLS_OFFSETS {0, 50, 83, 121, 149, 177 } // in 0.1 volts, gets added to voltage value  - useful for zener diodes
#define VBAT_CELLS_DIVS { 75, 122,  98, 18, 30, 37 } // divisor for proportional part according to resistors - larger value here gives smaller voltage

/********************************************************************/
/****           powermeter (battery capacity monitoring)         ****/
/********************************************************************/

/* enable monitoring of the power consumption from battery (think of mAh)
allows to set alarm value in GUI or via LCD
Full description and howto here http://www.multiwii.com/wiki/index.php?title=Powermeter
Two options:
1 - hard: - (uses hardware sensor, after configuration gives very good results)
2 - soft: - (good results +-5% for plush and mystery ESCs @ 2S and 3S, not good with SuperSimple ESC)    */
#define PSENSORNULL 510 /* (*) hard only: set to analogRead() value for zero current; for I=0A my sensor
gives 1/2 Vss; that is approx 2.49Volt; */
#define PINT2mA 132     /* (*) hard: one integer step on arduino analog translates to mA (example 4.9 / 37 * 1000) ;
soft: use fictional value, start with 100.
for hard and soft: larger PINT2mA will get you larger value for power (mAh equivalent) */

/********************************************************************/
/****           altitude hold                                    ****/
/********************************************************************/

/* defines the neutral zone of throttle stick during altitude hold, default setting is
+/-50 uncommend and change the value below if you want to change it. */
#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE    50
/* uncomment to disable the altitude hold feature.
* This is useful if all of the following apply
* + you have a baro
* + want altitude readout and/or variometer
* + do not use altitude hold feature
* + want to save memory space */

/********************************************************************/
/****           altitude variometer                              ****/
/********************************************************************/

/* enable to get audio feedback upon rising/falling copter/plane.
* Requires a working baro.
* For now, Output gets sent to an enabled vt100 terminal program over the serial line.
* choice of two methods (enable either one or both)
* method 1 : use short term movement from baro ( bigger code size)
* method 2 : use long term observation of altitude from baro (smaller code size)
*/

/********************************************************************/
/****           board naming                                     ****/
/********************************************************************/

/*
* this name is displayed together with the MultiWii version number
* upon powerup on the LCD.
* If you are without a DISPLAYD then You may enable LCD_TTY and
* use arduino IDE's serial monitor to view the info.
*
* You must preserve the format of this string!
* It must be 16 characters total,
* The last 4 characters will be overwritten with the version number.
*/
#define BOARD_NAME "MultiWii   V-.--"


/*************      Support multiple configuration profiles in EEPROM     ************/
//#define MULTIPLE_CONFIGURATION_PROFILES

/*************      do no reset constants when change of flashed program is detected ***********/
#define NO_FLASH_CHECK

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  7 - TUNING & DEVELOPER                                  **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

#define VBAT_PRESCALER 16 // set this to 8 if vbatscale would exceed 255


/**************************************************************************************/
/***********************     motor, servo and other presets     ***********************/
/**************************************************************************************/
/* motors will not spin when the throttle command is in low position
this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/***********************         Servo Refreshrates            ***********************/
/* Default 50Hz Servo refresh rate*/
#define SERVO_RFR_50HZ

/***********************             HW PWM Servos             ***********************/
/* HW PWM Servo outputs for Arduino Mega.. moves:
Pitch   = pin 44
Roll    = pin 45
CamTrig = pin 46
SERVO4  = pin 11 (aileron left for fixed wing or TRI YAW SERVO)
SERVO5  = pin 12 (aileron right for fixed wing)
SERVO6  = pin 6   (rudder for fixed wing)
SERVO7  = pin 7   (elevator for fixed wing)
SERVO8  = pin 8   (motor for fixed wing)       */
#define MEGA_HW_PWM_SERVOS

/* HW PWM Servo outputs for 32u4 NanoWii, MicroWii etc. - works with either the variable SERVO_RFR_RATE or
* one of the 3 fixed servo.refresh.rates *
* Tested only for heli_120, i.e. 1 motor + 4 servos, moves..
* motor[0] = motor       = pin  6
* servo[3] = nick  servo = pin 11
* servo[4] = left  servo = pin 10
* servo[5] = yaw   servo = pin  5
* servo[6]  = right servo= pin  9
*/
#define SERVO_RFR_RATE  50    // In Hz, you can set it from 20 to 400Hz, used only in HW PWM mode for mega and 32u4


/********************************************************************/
/****           Memory savings                                   ****/
/********************************************************************/

/* options to counter the general shortage of both flash and ram memory, like with leonardo m32u4 and others */

/**** suppress handling of serial commands.***
* This does _not_ affect handling of RXserial, Spektrum or GPS. Those will not be affected and still work the same.
* Enable either one or both of the following options  */

/* Remove handling of all commands of the New MultiWii Serial Protocol.
* This will disable use of the GUI, winGUI, android apps and any other program that makes use of the MSP.
* You must find another way (like LCD_CONF) to tune the parameters or live with the defaults.
* If you run a LCD/OLED via i2c or serial/Bluetooth, this is safe to use */
//#define SUPPRESS_ALL_SERIAL_MSP // saves approx 2700 bytes

/* Remove handling of other serial commands.
* This includes navigating via serial the lcd.configuration menu, lcd.telemetry and permanent.log .
* Navigating via stick inputs on tx is not affected and will work the same.  */
//#define SUPPRESS_OTHER_SERIAL_COMMANDS // saves  approx 0 to 100 bytes, depending on features enabled

/**** suppress keeping the defaults for initial setup and reset in the code.
* This requires a manual initial setup of the PIDs etc. or load and write from defaults.mwi;
* reset in GUI will not work on PIDs
*/
//#define SUPPRESS_DEFAULTS_FROM_GUI

//#define DISABLE_SETTINGS_TAB  // Saves ~400bytes on ProMini

/********************************************************************/
/****           diagnostics                                      ****/
/********************************************************************/

/* to log values like max loop time and others to come
logging values are visible via LCD config
set to 1, enable 'R' option to reset values, max current, max altitude
set to 2, adds min/max cycleTimes
set to 3, adds additional powerconsumption on a per motor basis (this uses the big array and is a memory hog, if POWERMETER <> PM_SOFT) */

/* Permanent logging to eeprom - survives (most) upgrades and parameter resets.
* used to track number of flights etc. over lifetime of controller board.
* Writes to end of eeprom - should not conflict with stored parameters yet.
* Logged values: accumulated lifetime, #powercycle/reset/initialize events, #arm events, #disarm events, last armedTime,
*                #failsafe@disarm, #i2c_errs@disarm
* Enable one or more options to show the log
*/

/* to add debugging code
not needed and not recommended for normal operation
will add extra code that may slow down the main loop or make copter non-flyable */
/* Use this to trigger LCD configuration without a TX - only for debugging - do NOT fly with this activated */
/* Use this to trigger telemetry without a TX - only for debugging - do NOT fly with this activated */
/* Enable string transmissions from copter to GUI */
#define DEBUGMSG


/********************************************************************/
/****           ESCs calibration                                 ****/
/********************************************************************/

/* to calibrate all ESCs connected to MWii at the same time (useful to avoid unplugging/re-plugging each ESC)
Warning: this creates a special version of MultiWii Code
You cannot fly with this special version. It is only to be used for calibrating ESCs
Read How To at http://code.google.com/p/multiwii/wiki/ESCsCalibration */
#define ESC_CALIB_LOW  MINCOMMAND
#define ESC_CALIB_HIGH 1290

/****           internal frequencies                             ****/
/* frequenies for rare cyclic actions in the main loop, depend on cycle time
time base is main loop cycle time - a value of 6 means to trigger the action every 6th run through the main loop
example: with cycle time of approx 3ms, do action every 6*3ms=18ms
value must be [1; 65535] */
#define LCD_TELEMETRY_FREQ 23       // to send telemetry data over serial 23 <=> 60ms <=> 16Hz (only sending interlaced, so 8Hz update rate)
#define LCD_TELEMETRY_AUTO_FREQ  967// to step to next telemetry page 967 <=> 3s
#define PSENSOR_SMOOTH 16           // len of averaging vector for smoothing the PSENSOR readings; should be power of 2; set to 1 to disable
#define VBAT_SMOOTH 16              // len of averaging vector for smoothing the VBAT readings; should be power of 2; set to 1 to disable
#define RSSI_SMOOTH 16              // len of averaging vector for smoothing the RSSI readings; should be power of 2; set to 1 to disable

/********************************************************************/
/****           Dynamic Motor/Prop Balancing                     ****/
/********************************************************************/
/*                   !!! No Fly Mode !!!                            */

/********************************************************************/
/****           Regression testing                               ****/
/********************************************************************/

/* for development only:
to allow for easier and reproducable config sets for test compiling, different sets of config parameters are kept
together. This is meant to help detecting compile time errors for various features in a coordinated way.
It is not meant to produce your flying firmware
To use:
- do not set any options in config.h,
- enable with #define COPTERTEST 1, then compile
- if possible, check for the size
- repeat with other values of 2, 3, 4 etc.
*/

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  SECTION  8 - DEPRECATED                                                 *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

/* these features will be removed in the unforseeable future. Do not build new products or
* functionality based on such features. The default for all such features is OFF.
*/

/**************************    WMP power pin     *******************************/
/* disable use of the POWER PIN (allready done if the option RCAUXPIN12 is selected) */
#define DISABLE_POWER_PIN
