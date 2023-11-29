#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include "main.h"
#include "math.h"

// Define Port
#define A1_PORT	GPIOA
#define B1_PORT	GPIOA
#define A2_PORT GPIOA
#define B2_PORT	GPIOA
// Define Pin
#define A1_PIN	GPIO_PIN_0
#define	B1_PIN	GPIO_PIN_1
#define A2_PIN	GPIO_PIN_2
#define	B2_PIN	GPIO_PIN_3
// Define step motor
#define SET_ARR						4096						// SET_ARR must be equal to ARR you set on CubeMX
#define MODE							24							// The mode you want to run, I recommend MODE don't greater than 64

// If you use other TIMER then change htim
// Example: I use htim3 instead of htim1
// Then: extern TIM_HandleTypeDef htim3;	
// And go to file StepMotor.c to change
extern TIM_HandleTypeDef htim1;						
extern TIM_HandleTypeDef htim2;

extern float PWM_increase[MODE];
extern float PWM_decrease[MODE];

// If you use other Channel of PWM, go to file StepMotor.c to change
int Pulse_Per_Round(int angle_rotation);
int Time_Delay_Tau(int pulse_per_round, float velocity);
int Pulse(int angle_rotation, int number_of_rotation);
void delay_us(uint32_t time_delay);
void WritePWM_Increase(float* array, uint8_t mode);
void WritePWM_Decrease(float* array, uint8_t mode);
void RunMotorClockwise(uint32_t time_delay_tau, int pulse);
void RunMotorCounter_clockwise(uint32_t time_delay_tau, int pulse);
void ResetPinState(void);
#endif
