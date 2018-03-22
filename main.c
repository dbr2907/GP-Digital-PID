/*This work is licensed under a
Creative Commons Attribution-NonCommmercial-ShareAlike 4.0 International

This license lets others remix, tweak, and build upon this work non-commercially,
as long as they credit the AUTHOR and license their new creations under the identical terms.
*/

/*
Ivan Rene Morales Argueta (2014)
ivan[at]fisica[dot]usac[dot]edu[dot]gt
Universidad de San Carlos de Guatemala
EE School
*/

/*
Modified by David Barrientos (2015)
for applications using Embedded C that require float data
d[dot]b[dot]gt[at]ieee[dot]org
Universidad de San Carlos de Guatemala
EE School
*/

/*************************************Librerias**************************************/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
/************************************Definiciones**********************************************/
#define Clock 80000000
#define WheelMotorsFreq 1000
/*************************************Variables************************************************/
///-----------------PID/-----------------//
volatile float Kp, Ki, Kd, minInt, maxInt;
volatile float setPoint;
volatile float derivator, integrator;
volatile float error;
///-----------------PWM-----------------//
volatile uint32_t Load;
volatile uint32_t PWMClock;
volatile uint32_t DutyC1;
///-----------------Control-----------------//
volatile uint32_t SensorVariable;
/************************************PID*********************************************************/

//**********************Inicialización*************************//
void PIDInit(float p, float i, float d, float minI, float maxI){
//-----------------Parametros PID------------------------------//
   Kp = p; Ki = i; Kd = d;
//-----------------Limites del Integrador----------------------//
   minInt = minI; maxInt = maxI;
//-----------------Condiciones Iniciales-----------------------//
   derivator = 0; integrator = 0;
   error = 0;
}

//**************************Set Point*************************//
void PIDSetPoint(float set){
  setPoint = set;
  derivator = 0;
  integrator = 0;
}

//****************Update (Output Generation)*******************//
float PIDUpdate(float currentValue){
  float pValue, iValue, dValue, PID;
  error = setPoint - currentValue;
  //-----------------Proportional---------------------//
  pValue = Kp*error;
  //-----------------Integral-------------------------//
  integrator = integrator + error;
  if (integrator > maxInt){integrator = maxInt;}
  else if(integrator < minInt){integrator = minInt;}
  iValue = integrator*Ki;
  //-----------------Derivative-----------------------//
  dValue = Kd*(error - derivator);
  derivator = error;
  //-----------------Output----------------------//
  PID = pValue + iValue + dValue;

//****************Specific to Application*******************//
  //------------Motor Rotation Change------------//
  if(PID>0){GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x10);}
  else{GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 0x20);}
  //--------Hysteresis Control for PWM---------//
  if(PID>100){PID=100;}
  else if(PID<-100){PID=-100;}
  if(PID<0){PID=(-1)*PID;}
//***************************************************************//
   return PID;
}

/************************************PWM*********************************************************/
void PWM_Init(void){
    PWMClock = Clock/64;
    Load = (PWMClock / WheelMotorsFreq) - 1;
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, Load);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

/************************************Main*********************************************************/
void main(void){
    /************************************CLOCK********************************************************/
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_XTAL_16MHZ);
    /****************************PWM *************************************/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    PWM_Init();

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Load*DutyC1/100);

    PIDInit(10, 2, 10, -5, 5);
    PIDSetPoint(45);

    while(1){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Load*DutyC1/100);

        DutyC1 = (unsigned int)PIDUpdate(SensorVariable);
    }
}

