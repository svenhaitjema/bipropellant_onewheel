/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "sensorcoms.h"
#include "flashaccess.h"
#include "protocolfunctions.h"
#include "bldc.h"
#include "hallinterrupts.h"
#include "softwareserial.h"
//#include "hd44780.h"
#include "pid.h"
#include "flashcontent.h"

#include "deadreckoner.h"
#include "control_structures.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
void BldcController_Init();

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

extern volatile int64_t bldc_counter;
int cmd1, cmd2;                      // normalized input values. -1000 to 1000
double cmd1_ADC, cmd2_ADC;           // ADC input values
double adcrFiltered, adctFiltered;

// used if set in setup.c
int autoSensorBaud2 = 0; // in USART2_IT_init
int autoSensorBaud3 = 0; // in USART3_IT_init

bool ADCcontrolActive = false;

int sensor_control = 0;



#define DRIVING_MODE_NORMAL 0
#define DRIVING_MODE_PUSHBACK 1
#define DRIVING_MODE_ELEVATED 2

uint8_t current_driving_mode = DRIVING_MODE_NORMAL;
uint8_t configured_driving_mode = DRIVING_MODE_NORMAL;

#define DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS 0
#define DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS -2 // 9000 == maximum tilt (20 deg), 450 = 5% elevation of maximum
#define DRIVING_MODE_ELEVATED_GYRO_ANGLE_BIAS -2 // 9000 == maximum tilt (20 deg), 450 = 5% elevation of maximum
short current_driving_mode_gyro_bias = DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;





#define THROTTLE_MAX 800 // 600 = max pwm
float throttle = 0;
bool step_up = true;


#define STEP_UP_ALLOWED_ANGLE_DEVIATION 0.5

#define DRIVE_MODE_NORMAL 0
#define DRIVE_MODE_PUSHBACK 1
#define DRIVE_MODE_ELEVATED 2
#define DRIVE_MODE_TIGHT 3


#define STATE_IDLE 0
#define STATE_STEP_UP 1
#define STATE_DRIVING 2
#define STATE_PUSHBACK 3
#define STATE_SHUT_OFF 4

#define POWER_BUTTON_STATE_NOT_PRESSED 0 
#define POWER_BUTTON_STATE_SHORT_PRESS 1 // >100 ms && < 2 sec
#define POWER_BUTTON_STATE_MEDIUM_PRESS 2 // > 2 sec && < 4 sec
#define POWER_BUTTON_STATE_LONG_PRESS 3 // > 6 sec && < 10 sec

#define MAX_STEP_UP_ANGLE 5

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control 
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate board own weight distribution)

#define KP 2//0.32       
#define KD 800//0.35//0.050     
#define KP_THROTTLE 2//0.051//0.080 
#define KI_THROTTLE 0//0.01//0.1 

#define KP_POSITION 0.06  
#define KD_POSITION 0.45  


uint16_t foot_off_pad_delay_ticks = 0;
#define FOOT_OFF_PAD_SWITCH_DELAY 50

float motor_pwms = 0;

float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;

float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;


uint8_t current_state = STATE_IDLE;
uint8_t new_state = STATE_IDLE;


/* MANDATORY TO CHANGE ALL ACCORDINGLY, OTHERWISE UNDEFINED BEHAVIOUR */
uint8_t drive_mode = DRIVE_MODE_NORMAL;
uint8_t previous_drive_mode = DRIVE_MODE_NORMAL;
float driving_mode_current_angle=DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;
/* ******************************************************************* */

uint16_t acceleration_in_pwm_units = 0;
float current_hub_speed_in_mm_per_second = 0;
float current_tire_speed_in_m_per_second = 0;
float current_tire_speed_in_km_per_hour = 0;

float wanted_hub_speed_in_mm_per_second = 0;

uint16_t current_angle = 0;
uint16_t wanted_angle = 0;


float stability_setPointOld=0;
float stability_PID_errorOld=0;

int32_t speed_PID_errorSum = 0;

float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = CLAMP((setPoint - stability_setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - stability_PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  stability_PID_errorOld = input;  // error for Kd is only the input component
  stability_setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  speed_PID_errorSum += CLAMP(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  speed_PID_errorSum = CLAMP(speed_PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = (Kp * error) + (Ki * speed_PID_errorSum) * DT; // DT is in miliseconds...
  return (output);
}

float map_val(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// the main thing qwhich determines how we are controlled from protocol
int control_type = CONTROL_TYPE_NONE;

#ifdef CONTROL_SENSOR
SENSOR_DATA last_sensor_data[2];
int sensor_stabilise = 0;

#endif
uint8_t disablepoweroff = 0;
int powerofftimer = 0;


extern volatile unsigned int timerval;
extern volatile unsigned int ssbits;

uint8_t button1, button2, button1_ADC, button2_ADC;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
int buzzerLen = 0;

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t input_timeout_counter; // global variable for input timeout
extern float batteryVoltage; // global variable for battery voltage

float average_battery_voltage[10];
uint8_t n_battery_measurements = 0;
uint8_t average_battery_index = 0;




uint32_t inactivity_timeout_counter;
uint32_t debug_counter = 0;

extern uint8_t nunchuck_data[6];

int milli_vel_error_sum = 0;

DEADRECKONER *deadreconer;
INTEGER_XYT_POSN xytPosn;

typedef struct tag_power_button_info {
  int startup_button_held;      // indicates power button was active at startup
  int button_prev;              // last value of power button
  unsigned int button_held_ms;  // ms for which the button has been held down
} POWER_BUTTON_INFO;
void check_power_button();

POWER_BUTTON_INFO power_button_info;

void poweroff() {
    if (ABS(speed) < 20) {
        buzzerPattern = 0;
        enable = 0;
        for (int i = 0; i < 8; i++) {
            buzzerFreq = i;
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);

        // if we are powered from sTLink, this bit allows the system to be started again with the button.
        while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}

        while (1){
          if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)){
            HAL_NVIC_SystemReset();
          }
        }
    }
}

// actually 'power'
int pwms[2] = {0, 0};

// unused, but keep ascii from erroring
int dspeeds[2] = {0,0};



/////////////////////////////////////////
// variables stored in flash
// from flashcontent.h
FLASH_CONTENT FlashContent;
const FLASH_CONTENT FlashDefaults = {
  .magic = CURRENT_MAGIC,
  .PositionKpx100 = 50,
  .PositionKix100 = 50,
  .PositionKdx100 = 0,
  .PositionPWMLimit = 1000,
  .SpeedKpx100 = 20,
  .SpeedKix100 = 10,
  .SpeedKdx100 = 0,
  .SpeedPWMIncrementLimit = 20,
  .MaxCurrLim = 1500,
  .HoverboardEnable = 1,
  .calibration_0 = 0,
  .calibration_1 = 0,
  .HoverboardPWMLimit = 1000,
  .adc.adc1_mult_neg = ADC1_MULT_NEG,
  .adc.adc1_mult_pos = ADC1_MULT_POS,
  .adc.adc1_min = ADC1_MIN,
  .adc.adc1_zero = ADC1_ZERO,
  .adc.adc1_max = ADC1_MAX,
  .adc.adc2_min = ADC2_MIN,
  .adc.adc2_zero = ADC2_ZERO,
  .adc.adc2_max = ADC2_MAX,
  .adc.adc2_mult_neg = ADC2_MULT_NEG,
  .adc.adc2_mult_pos = ADC2_MULT_POS,
  .adc.adc_off_start = ADC_OFF_START,
  .adc.adc_off_end = ADC_OFF_END,
  .adc.adc_off_filter = ADC_OFF_FILTER,
  .adc.adc_switch_channels = ADC_SWITCH_CHANNELS,
  .adc.adc_reverse_steer = ADC_REVERSE_STEER,
  .adc.adc_tankmode = ADC_TANKMODE,
};


typedef struct tag_PID_FLOATS{
    float in;
    float set;
    float out;

    int count; // - used in averaging speed between pid loops
} PID_FLOATS;

// setup pid control for left and right speed.
pid_controller  PositionPid[2];
// temp floats
PID_FLOATS PositionPidFloats[2] = {
  { 0, 0, 0,   0 },
  { 0, 0, 0,   0 }
};
pid_controller  SpeedPid[2];
// temp floats
PID_FLOATS SpeedPidFloats[2] = {
  { 0, 0, 0,   0 },
  { 0, 0, 0,   0 }
};

void init_PID_control(){
  memset(&PositionPid, 0, sizeof(PositionPid));
  memset(&SpeedPid, 0, sizeof(SpeedPid));
  for (int i = 0; i < 2; i++){
    PositionPidFloats[i].in = 0;
    PositionPidFloats[i].set = 0;
    pid_create(&PositionPid[i], &PositionPidFloats[i].in, &PositionPidFloats[i].out, &PositionPidFloats[i].set,
      (float)FlashContent.PositionKpx100/100.0,
      (float)FlashContent.PositionKix100/100.0,
      (float)FlashContent.PositionKdx100/100.0);

    // maximum pwm outputs for positional control; limits speed
  	pid_limits(&PositionPid[i], -FlashContent.PositionPWMLimit, FlashContent.PositionPWMLimit);
  	pid_auto(&PositionPid[i]);
    SpeedPidFloats[i].in = 0;
    SpeedPidFloats[i].set = 0;
    pid_create(&SpeedPid[i], &SpeedPidFloats[i].in, &SpeedPidFloats[i].out, &SpeedPidFloats[i].set,
      (float)FlashContent.SpeedKpx100/100.0,
      (float)FlashContent.SpeedKix100/100.0,
      (float)FlashContent.SpeedKdx100/100.0);

    // maximum increment to pwm outputs for speed control; limits changes in speed (accelleration)
  	pid_limits(&SpeedPid[i], -FlashContent.SpeedPWMIncrementLimit, FlashContent.SpeedPWMIncrementLimit);
  	pid_auto(&SpeedPid[i]);
  }
}

void change_PID_constants(){
  for (int i = 0; i < 2; i++){
    pid_tune(&PositionPid[i],
      (float)FlashContent.PositionKpx100/100.0,
      (float)FlashContent.PositionKix100/100.0,
      (float)FlashContent.PositionKdx100/100.0);
  	pid_limits(&PositionPid[i], -FlashContent.PositionPWMLimit, FlashContent.PositionPWMLimit);

    pid_tune(&SpeedPid[i],
      (float)FlashContent.SpeedKpx100/100.0,
      (float)FlashContent.SpeedKix100/100.0,
      (float)FlashContent.SpeedKdx100/100.0);
  	pid_limits(&SpeedPid[i], -FlashContent.SpeedPWMIncrementLimit, FlashContent.SpeedPWMIncrementLimit);
  }
}

void init_flash_content(){
  FLASH_CONTENT FlashRead;
  int len = readFlash( (unsigned char *)&FlashRead, sizeof(FlashRead) );

  if ((len != sizeof(FlashRead)) || (FlashRead.magic != CURRENT_MAGIC)){
    memcpy(&FlashRead, &FlashDefaults, sizeof(FlashRead));
    writeFlash( (unsigned char *)&FlashRead, sizeof(FlashRead) );
    consoleLog("Flash initiailised\r\n");
  }
  memcpy(&FlashContent, &FlashRead, sizeof(FlashContent));
}

void buzz(uint8_t freq,uint8_t pattern,uint16_t len_ms){
  buzzerFreq = freq;
  buzzerPattern = pattern;
  buzzerLen = len_ms;
}

bool footOnPad(void){
  if(sensor_data[0].sensor_ok || sensor_data[1].sensor_ok){
    return true;
  }
  return false;
}


int main(void) {
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  #define WHEELBASE_MM 525.0
  deadreconer = DeadReckoner(
    & HallData[0].HallPosn,
    & HallData[1].HallPosn,
    HALL_POSN_PER_REV,
    (DEFAULT_WHEEL_SIZE_INCHES*25.4),
    WHEELBASE_MM, 1);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
    UART_Init();
  #endif

  memset((void*)&electrical_measurements, 0, sizeof(electrical_measurements));

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  #ifdef CONTROL_SENSOR
  if (USART2_BAUD_SENSE) {
    autoSensorBaud2 = getSensorBaudRate(0);
  }
  if (USART3_BAUD_SENSE) {
    autoSensorBaud3 = getSensorBaudRate(1);
  }
  #endif

  #ifdef SERIAL_USART2_IT
  USART2_IT_init();
  #endif
  #ifdef SERIAL_USART3_IT
  USART3_IT_init();
  #endif
  #ifdef SOFTWARE_SERIAL
  SoftwareSerialInit();
  #endif

  init_flash_content();

  init_PID_control();

  #ifdef CONTROL_SENSOR
  // initialise to 9 bit interrupt driven comms on USART 2 & 3
  sensor_init();
  #endif

  if (0 == FlashContent.MaxCurrLim) {
    FlashContent.MaxCurrLim = DC_CUR_LIMIT*100;
  }
  electrical_measurements.dcCurLim = MIN(DC_CUR_LIMIT*100, FlashContent.MaxCurrLim);



  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  //int lastspeeds[2] = {0, 0};

  #ifdef CONTROL_SENSOR
  // things we use in main loop for sensor control
  consoleLog("power on\n");

  #endif

    // enables interrupt reading of hall sensors for dead reconing wheel position.
    HallInterruptinit();




      // sets up serial ports, and enables protocol on selected ports
    #if defined(SOFTWARE_SERIAL) && (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)
        setup_protocol(&sSoftwareSerial);
    #endif
    #if defined(SERIAL_USART2_IT) && (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)
        setup_protocol(&sUSART2);
    #endif
    #if defined(SERIAL_USART3_IT) && (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2) && !defined(CONTROL_SENSOR)
        setup_protocol(&sUSART3);
    #endif

  int last_control_type = CONTROL_TYPE_NONE;


  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float board_temp_deg_c;

  enable = 1;  // enable motors

  // ####### POWER-BUTTON startup conditions #######
  power_button_info.startup_button_held = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
  power_button_info.button_prev = power_button_info.startup_button_held;
  power_button_info.button_held_ms = 0;

  if (power_button_info.startup_button_held) {
    consoleLog("Power button down at startup\r\n");
  } else {
    consoleLog("Power button up at startup\r\n");
  }


  timeStats.hclkFreq = HAL_RCC_GetHCLKFreq();

  timeStats.now_us = HallGetuS();
  timeStats.now_ms = HAL_GetTick();
  timeStats.nominal_delay_us = (DELAY_IN_MAIN_LOOP * 1000);
  timeStats.start_processing_us = timeStats.now_us + timeStats.nominal_delay_us;
  timeStats.start_processing_ms = timeStats.now_ms + DELAY_IN_MAIN_LOOP;

  int64_t start_bldc_counter = bldc_counter;
  HAL_Delay(200);
  int64_t bldc_counter_200ms = bldc_counter;

  int bldc_in_200ms = (int)(bldc_counter_200ms - start_bldc_counter);
  timeStats.bldc_freq = bldc_in_200ms * 5;

  // uses timeStats.bldc_freq
  BldcControllerParams.callFrequency = timeStats.bldc_freq;
  BldcController_Init();

  uint8_t current_state = STATE_IDLE;
  uint8_t last_state = STATE_IDLE;

  input_timeout_counter=0;

  while(1) {
    timeStats.time_in_us = timeStats.now_us;
    timeStats.time_in_ms = timeStats.now_ms;

    if (timeStats.start_processing_us < timeStats.now_us) {
      timeStats.us_lost += timeStats.now_us - timeStats.start_processing_us;
      timeStats.main_late_count++;
      timeStats.start_processing_us = timeStats.now_us + 1000;  // at least 1ms of delay
    }

    // delay until we should start processing
    while (timeStats.now_us < timeStats.start_processing_us){
      #if (INCLUDE_PROTOCOL == INCLUDE_PROTOCOL2)
        #ifdef SOFTWARE_SERIAL
          while ( softwareserial_available() > 0 ) {
            protocol_byte( &sSoftwareSerial, (unsigned char) softwareserial_getrx() );
          }
          protocol_tick( &sSoftwareSerial );
        #endif

        #if defined(SERIAL_USART2_IT) && defined(CONTROL_SENSOR)
          // if we enabled USART2 as protocol from power button at startup
          if (USART2ProtocolEnable) {
            while ( serial_usart_buffer_count(&usart2_it_RXbuffer) > 0 ) {
              protocol_byte( &sUSART2, (unsigned char) serial_usart_buffer_pop(&usart2_it_RXbuffer) );
            }
            protocol_tick( &sUSART2 );
          }
        #endif
      #endif
      timeStats.now_us = HallGetuS();
      timeStats.now_ms = HAL_GetTick();
    }

    // move out '5ms' trigger on by 5ms
    timeStats.processing_in_us = timeStats.now_us;
    timeStats.processing_in_ms = timeStats.now_ms;

    timeStats.bldc_us = (1000*timeStats.bldc_cycles)/(timeStats.hclkFreq/1000);

    // read last DMAed ADC values, moved from bldc interrupt to non interrupt.
    readADCs();
    sensor_read_data();

    switch(current_state){
      case STATE_IDLE:
        if(current_state != last_state){
          inactivity_timeout_counter=0;
          last_state = current_state;
          pwms[0]=0; // disable motors
          enable = 0;
          break;
        }

        if (power_button_info.startup_button_held){
          break; // wait for startup processes to finish
        }

          // if batt ok & feet on pad -> goto state stepup
        inactivity_timeout_counter++;
        // inactivity 10s warning; 1s bleeping
        if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 50 * 1000) / DELAY_IN_MAIN_LOOP) &&
            (buzzerFreq == 0)) {
          buzz(3,1,1000);
        }
        // inactivity 5s warning; 1s bleeping
        if ((inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 55 * 1000) / DELAY_IN_MAIN_LOOP) &&
            (buzzerFreq == 0)) {
          buzz(2,1,1000);
        }
        // power off after ~60s of inactivity
        if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / DELAY_IN_MAIN_LOOP ) {  // rest of main loop needs maybe 1ms
          inactivity_timeout_counter = 0;
          consoleLog("power off by 60s inactivity\r\n");
          poweroff();
        }

        if (electrical_measurements.charging){
          sensor_set_flash(0, 3);
          break;
        } else {
          sensor_set_flash(0, 0);
        }

        if(footOnPad()){
          current_state = STATE_STEP_UP; 
        }


      break;
      case STATE_STEP_UP:
        if(current_state != last_state){
          last_state = current_state;
          pwms[0]=0; // disable motors
          enable = 0;
          break;
        }

        if(!footOnPad()){
          current_state = STATE_IDLE;
          break;
        }  

        int16_t step_up_angle = 0 ;
        switch(drive_mode){
          case DRIVE_MODE_NORMAL:
            step_up_angle = DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;
          break;
          case DRIVE_MODE_PUSHBACK:
            step_up_angle = DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS;
          break;
          case DRIVE_MODE_ELEVATED:
            step_up_angle = DRIVING_MODE_ELEVATED_GYRO_ANGLE_BIAS;
          break;
        }

        float mapped_stepup_angle = CLAMP(sensor_data[0].complete.Angle,-9000,9000);
        mapped_stepup_angle = map_val(mapped_stepup_angle,-9000,9000,-45,45);

        
        if(mapped_stepup_angle > ((float)step_up_angle-STEP_UP_ALLOWED_ANGLE_DEVIATION) && mapped_stepup_angle < ((float)step_up_angle+STEP_UP_ALLOWED_ANGLE_DEVIATION)){ // activate board
          current_state = STATE_DRIVING;
          buzz(1,1,1000);
          driving_mode_current_angle=step_up_angle;
          break;
        } 
        

        // if feet on pad & angle ~ 0
      break;
      case STATE_DRIVING:
        if(current_state != last_state){
          last_state = current_state;
          enable = 1;
          pwmr = 0; 
          pwml = 0;
          motor_pwms=0; 

          stability_setPointOld=0;
          stability_PID_errorOld=0;
          speed_PID_errorSum = 0;
          throttle=0;
          foot_off_pad_delay_ticks=0;
          break;
        }

        if(!footOnPad()){
          foot_off_pad_delay_ticks+=1; // safety feature, we don't want a dead stop / PID reinitialize if we are rolling due to a glitch
          if(foot_off_pad_delay_ticks >= FOOT_OFF_PAD_SWITCH_DELAY){
            current_state = STATE_IDLE;
            break;
          }  
        }    
        else
        {
          foot_off_pad_delay_ticks=0;
        }
        sensor_send_lights();
        //int dirs[2] = {-1, 1};
        //pwms[0] = CLAMP((dirs[0]*((sensor_data[0].complete.Angle) - sensor_data[0].Center)/3), -FlashContent.HoverboardPWMLimit, FlashContent.HoverboardPWMLimit);
        //pwmr = -pwms[0]; 
        //pwml = pwms[0];

        
        float hub_speed_kmh = (float)HallData[0].HallSpeed_mm_per_s*-0.0036; // forward is positive speed
        float mapped_angle = CLAMP(sensor_data[0].complete.Angle,-9000,9000);
        mapped_angle = map_val(mapped_angle,-9000,9000,-45,45); // negative angle is nose up
        


        float speed_control_acceleration = speedPIControl(5, hub_speed_kmh, throttle, Kp_thr, Ki_thr);

        speed_control_acceleration = CLAMP(speed_control_acceleration, -FlashContent.HoverboardPWMLimit,FlashContent.HoverboardPWMLimit); // limited output // negative angle is nose up
        
        #define SPEED_CONTROL_ACCELERATION_GAIN -0.01
        #define THROTTLE_ANGLE_ERROR_GAIN 0.01
        speed_control_acceleration = speed_control_acceleration * SPEED_CONTROL_ACCELERATION_GAIN;
 
        float desired_angle = 0;
        /*
        switch(drive_mode){
          case DRIVE_MODE_NORMAL:
            desired_angle = DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;
            if(ABS((int16_t)motor_pwms)>900){
              drive_mode=DRIVE_MODE_PUSHBACK;
              previous_drive_mode=DRIVE_MODE_NORMAL;
              desired_angle = DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS;
            }
          break;
          case DRIVE_MODE_PUSHBACK:
            speed_control_acceleration=0;
            throttle=0;
            speed_PID_errorSum=0;
            
            desired_angle = DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS;
            buzz(2,1,1000);
            if(ABS((int16_t)motor_pwms)<750){
              drive_mode=previous_drive_mode;
              desired_angle = DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;
            }
          break;
          case DRIVE_MODE_ELEVATED:
          
            desired_angle = DRIVING_MODE_ELEVATED_GYRO_ANGLE_BIAS;
            if(ABS((int16_t)motor_pwms)>800){
              previous_drive_mode=DRIVE_MODE_ELEVATED;
              drive_mode=DRIVE_MODE_PUSHBACK;
              desired_angle = DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS;
            }
          break;
        }
        */
        if(ABS(desired_angle-driving_mode_current_angle) <= 0.001){
          driving_mode_current_angle=desired_angle;
        }
        else if(desired_angle > driving_mode_current_angle) driving_mode_current_angle+=0.01;
        else if(desired_angle < driving_mode_current_angle) driving_mode_current_angle-=0.01;


        float delta_angle = mapped_angle-((float)0);

        if(hub_speed_kmh>0){ // we are going forward
          if(delta_angle > (float)0.1){
            throttle+=delta_angle*THROTTLE_ANGLE_ERROR_GAIN;//0.05; // add more speed
          }
          else if(delta_angle <-1){
            throttle+=delta_angle*THROTTLE_ANGLE_ERROR_GAIN;//0.05; // remove speed
          }
        }
        else{
          if(delta_angle < -(float)0.1){
            throttle+=delta_angle*THROTTLE_ANGLE_ERROR_GAIN;//0.05; // add more speed
          }
          else if(delta_angle > 1){
            throttle+=delta_angle*THROTTLE_ANGLE_ERROR_GAIN;//0.05; // remove speed
          }          
        }



        throttle = CLAMP(throttle,-40,40);


        motor_pwms += stabilityPDControl(5,mapped_angle , driving_mode_current_angle, Kp, Kd);
        motor_pwms+=(int16_t)roundf(speed_control_acceleration);
        motor_pwms = CLAMP(motor_pwms, -FlashContent.HoverboardPWMLimit, FlashContent.HoverboardPWMLimit); // Limit max output from control
        
        int16_t t_motor_pwm = (int16_t)roundf(motor_pwms);//+(int16_t)roundf(speed_control_acceleration);

        

        t_motor_pwm = CLAMP(t_motor_pwm,-FlashContent.HoverboardPWMLimit, FlashContent.HoverboardPWMLimit);
  
        pwmr = -(t_motor_pwm); 
        pwml = t_motor_pwm;     

        // if  feet on pad & within limits, stay here, else idle or pushback

        // limits:
        // -batt > 3.6
        // -dutycycle <= 80%
        // -angle<45 deg
        // 
      break;
      case STATE_PUSHBACK:
        // if batt ok & feet on pad & outside of limits, stay here, else 
        // - stepoff -> idle
        // - wihtin limit -> driving
        // - over max limit -> shutoff
      break;
      case STATE_SHUT_OFF:
        // nothing to check here, just bluntly turn off
      break;
    }

    if ((debug_counter++) % 100 == 0) {
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;

      electrical_measurements.board_temp_raw = adc_buffer.temp;
      electrical_measurements.board_temp_filtered = board_temp_adc_filtered;
      electrical_measurements.board_temp_deg_c = board_temp_deg_c;
      electrical_measurements.charging = !(CHARGER_PORT->IDR & CHARGER_PIN);
    }

    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && ABS(speed) < 20){  // poweroff before mainboard burns OR low bat 3
      consoleLog("power off by temp\r\n");
      poweroff();
    }

    if (electrical_measurements.batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && ABS(speed) < 20) {  // poweroff before mainboard burns OR low bat 3
      consoleLog("power off by low voltage\r\n");
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      buzz(4,1,1000);
    } else if (electrical_measurements.batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && electrical_measurements.batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      buzz(5,42,1000);
    } else if (electrical_measurements.batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && electrical_measurements.batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      buzz(5,6,1000);
    } else if (BEEPS_BACKWARD && speed < -50) {  // backward beep
      buzz(5,1,1000);
    } else {  // do not beep
      if (buzzerLen > 0){
        buzzerLen--;
      } else {
        buzzerFreq = 0;
        buzzerPattern = 0;
      }
    }
    /*
    if(configured_driving_mode!=current_driving_mode){
      current_driving_mode=configured_driving_mode;
      if(configured_driving_mode==DRIVING_MODE_NORMAL){
        current_driving_mode_gyro_bias = DRIVING_MODE_NORMAL_GYRO_ANGLE_BIAS;
      }
      else if(configured_driving_mode==DRIVING_MODE_PUSHBACK){
        current_driving_mode_gyro_bias = DRIVING_MODE_PUSHBACK_GYRO_ANGLE_BIAS;
      }
      else if(configured_driving_mode==DRIVING_MODE_ELEVATED){
        current_driving_mode_gyro_bias = DRIVING_MODE_ELEVATED_GYRO_ANGLE_BIAS;
      }    
    }
    */
    check_power_button();

    ////////////////////////////////
    // take stats
    timeStats.now_us = HallGetuS();
    timeStats.now_ms = HAL_GetTick();

    timeStats.main_interval_us = timeStats.now_us - timeStats.time_in_us;
    timeStats.main_interval_ms = timeStats.now_ms - timeStats.time_in_ms;
    timeStats.main_delay_us = timeStats.processing_in_us - timeStats.time_in_us;
    timeStats.main_delay_ms = timeStats.processing_in_ms - timeStats.time_in_ms;
    timeStats.main_processing_us = timeStats.now_us - timeStats.processing_in_us;
    timeStats.main_processing_ms = timeStats.now_ms - timeStats.processing_in_ms;

    // maybe average main_dur as a stat?
    if (timeStats.main_interval_ms == 0){
      timeStats.main_interval_ms = ((float)timeStats.main_interval_us)/1000;
      timeStats.main_processing_ms = ((float)timeStats.main_processing_us)/1000.0;
    }
    timeStats.f_main_interval_ms = timeStats.f_main_interval_ms * 0.99;
    timeStats.f_main_interval_ms = timeStats.f_main_interval_ms + (((float)timeStats.main_interval_us)/1000.0)*0.01;

    timeStats.f_main_processing_ms = timeStats.f_main_processing_ms * 0.99;
    timeStats.f_main_processing_ms = timeStats.f_main_processing_ms + (((float)timeStats.main_processing_us)/1000.0)*0.01;

    // select next loop start point
    // move out '5ms' trigger on by 5ms
    timeStats.start_processing_us = timeStats.start_processing_us + timeStats.nominal_delay_us;
  }
}


//////////////////////////////////////////////////////
// check and do things when we press the power button
void check_power_button(){

  // if power button is currently down
  if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    // increment how long it has been down for.
    power_button_info.button_held_ms += DELAY_IN_MAIN_LOOP;

    // if power button was down at start
    if (power_button_info.startup_button_held) {
      if (power_button_info.button_held_ms > 5000)
      {
        // provisional startup mode - do something if button held for 5s at startup
      }
    } else {

      // if we have seen the state released since startup
      if (power_button_info.button_prev == 0){
        power_button_info.button_prev = 1;
        // reset the time pressed to zero
        power_button_info.button_held_ms = 0;
      } else {
        // button remains pressed
        // indicate how long it has been pressed by colour
        if ((power_button_info.button_held_ms > 100) &&
            (power_button_info.button_held_ms < 2000) )
        {
        #if defined CONTROL_SENSOR
          // indicate with 1 flash that this would power off
          sensor_set_flash(0, 1);
          sensor_set_flash(1, 1);
        #endif
        }
        if ((power_button_info.button_held_ms >= 2000) &&
            (power_button_info.button_held_ms < 5000) )
        {
        #if defined CONTROL_SENSOR
          // indicate with 2 flashes that we would not power off, and not calibrate
          sensor_set_flash(0, 2);
          sensor_set_flash(1, 2);
        #endif
        }
        if ((power_button_info.button_held_ms > 5000) &&
            (power_button_info.button_held_ms < 10000) )
        {
        #if defined CONTROL_SENSOR
          // indicate with 3 flashes that we would calibrate
          sensor_set_flash(0, 3);
          sensor_set_flash(1, 3);
        #endif
        }
        if ((power_button_info.button_held_ms > 10000) &&
            (power_button_info.button_held_ms < 15000) )
        {
        #if defined CONTROL_SENSOR
          // indicate with 4 flashes that we would turn USART2 into control
          sensor_set_flash(0, 4);
          sensor_set_flash(1, 4);
        #endif
        }
        if ((power_button_info.button_held_ms > 15000) &&
            (power_button_info.button_held_ms < 100000) )
        {
        #if defined CONTROL_SENSOR
          // indicate with 4 flashes that we would NOT calibrate
          sensor_set_flash(0, 4);
          sensor_set_flash(1, 4);
        #endif
        }
      }
    }
  } else {

    // ONLY take action if the startup button was NOT down when we started up, or has been released since
    if (!power_button_info.startup_button_held) {
      // if this is a button release
      if (power_button_info.button_prev) {
        // power button held for < 100ms or > 10s -> nothing
        if ((power_button_info.button_held_ms >= 15000) || (power_button_info.button_held_ms < 100))
        {
          // no action taken
        }

        // power button held for between 5s and 10s -> HB angle calibration
        // (only if it had been released since startup)
        if ((power_button_info.button_held_ms >= 10000) &&
            (power_button_info.button_held_ms < 15000))
        {
          buzzerPattern = 0;
          enable = 0;

        #if defined CONTROL_SENSOR
          // indicate we accepted calibrate command
          sensor_set_flash(0, 8);
          sensor_set_flash(1, 8);
        #endif

          // buz to indicate we are calibrating
          for (int i = 0; i < 20; i++) {
            buzzerFreq = i & 3;
            HAL_Delay(100);
          }

        #if defined CONTROL_SENSOR
          setUSART2ToControl();
          consoleLog("*** Write Flash Calibration data\r\n");
        #else
          consoleLog("*** Not a hoverboard, not modifyiing USART2\r\n");
        #endif

        }

        
        if ((power_button_info.button_held_ms >= 2000) &&
            (power_button_info.button_held_ms < 5000))
        {
          if(configured_driving_mode==DRIVING_MODE_ELEVATED){
              configured_driving_mode=DRIVING_MODE_NORMAL;
          } 
          else if(configured_driving_mode==DRIVING_MODE_NORMAL) {
            configured_driving_mode=DRIVING_MODE_ELEVATED;
          }
          buzzerPattern = 0;
          for (int i = 0; i < 20; i++) {
            buzzerFreq = i & 2;
            HAL_Delay(100);
          }
        }
        // power button held for between 5s and 10s -> HB angle calibration
        // (only if it had been released since startup)
        if ((power_button_info.button_held_ms >= 10000) &&
            (power_button_info.button_held_ms < 15000))
        {
          buzzerPattern = 0;
          enable = 0;

        #if defined CONTROL_SENSOR
          // indicate we accepted calibrate command
          sensor_set_flash(0, 8);
          sensor_set_flash(1, 8);
        #endif

          // buz to indicate we are calibrating
          for (int i = 0; i < 20; i++) {
            buzzerFreq = i & 2;
            HAL_Delay(100);
          }

        #if defined CONTROL_SENSOR
          // read current board angles, and save to flash as center
          FlashContent.calibration_0 = sensor_data[0].Center_calibration = sensor_data[0].complete.Angle;
          FlashContent.calibration_1 = sensor_data[1].Center_calibration = sensor_data[1].complete.Angle;
          writeFlash( (unsigned char *)&FlashContent, sizeof(FlashContent) );
          consoleLog("*** Write Flash Calibration data\r\n");
        #else
          consoleLog("*** Not a hoverboard, no calibrarion done\r\n");
        #endif

        }

        // power button held for >100ms < 2s -> power off
        // (only if it had been released since startup)
        if ((power_button_info.button_held_ms >= 100) &&
            (power_button_info.button_held_ms < 2000)) {
          enable = 0;
          //while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN));
          consoleLog("power off by button\r\n");
          poweroff();
        }

        // always stop flash after done/released regardless
      #if defined CONTROL_SENSOR
        sensor_set_flash(0, 0);
        sensor_set_flash(1, 0);
      #endif

      } // end of if power button was previous down
    }

    // always mark the button as released
    power_button_info.button_held_ms = 0;
    power_button_info.button_prev = 0;
    power_button_info.startup_button_held = 0;
  } // end of else power button must be up
}



/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  // enable the DWT counter for cycle timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

}
