#include "StepMotor.h"

uint8_t clock_wise[8] = 				{0x0A,0x02,0x06,0x04,0x05,0x01,0x09,0x08};			// 2 phase ON is even , 1 phase ON is odd
uint8_t counter_clock_wise[8] = {0x0A,0x08,0x09,0x01,0x05,0x04,0x06,0x02};			// 2 phase ON is even , 1 phase ON is odd


float PWM_increase[MODE];
float PWM_decrease[MODE];


float micro_step_angle = 1.8 / MODE;
float theta = (90.0 / (MODE * 1.0)) * 3.14 / 180.0;


int Pulse_Per_Round(int angle_rotation)
{
	int pulse_per_round = angle_rotation / micro_step_angle;
	return pulse_per_round;
}

int Time_Delay_Tau(int pulse_per_round, float velocity)
{
	float pulse_per_sec = (velocity / 60) * (float)pulse_per_round;
	float time_delay_tau = (1.0 / pulse_per_sec) * 1000.0 * 1000.0; // [us]
	return time_delay_tau;
}

int Pulse(int angle_rotation, int number_of_rotation)
{
	int pulse = angle_rotation * number_of_rotation / (micro_step_angle / 2);
	return pulse;
}

// If you use other TIMER change htim2
void delay_us(uint32_t time_delay)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	HAL_TIM_Base_Start(&htim2);
	while(__HAL_TIM_GET_COUNTER(&htim2) < time_delay);
	HAL_TIM_Base_Stop(&htim2);
}

void WritePWM_Increase(float* array, uint8_t mode)									// Find all PWM value in proportion to each increasing theta 
{
	float T_ON;
	for(uint8_t i = 0; i < mode; i++)
	{
		T_ON = sin(theta) * 100;
		*(array + i) = T_ON;
		theta = theta + (90.0 / (MODE * 1.0)) * 3.14 / 180.0;
	}
}

void WritePWM_Decrease(float* array, uint8_t mode)															// Find all PWM value in proportion to each decreasing theta 
{
	for(uint8_t i = 0; i < mode - 1; i++)
	{
		*(array + i) = PWM_increase[31 - i - 1];
	}
	*(array + mode - 1) = 0;
}

// If you use other channel of PWM, then change CCR
// Example: I use channel 1 instead of channel 3
// Then TIM1->CCR1 = ...;
void RunMotorClockwise(uint32_t time_delay_tau, int* counted_pulse) 
{
	for(uint8_t i = 0; i < 8; i = i + 2)
	{
		HAL_GPIO_WritePin(A1_PORT,A1_PIN,(GPIO_PinState)((clock_wise[i] & 0x08) >> 3));
		HAL_GPIO_WritePin(B1_PORT,B1_PIN,(GPIO_PinState)((clock_wise[i] & 0x04) >> 2));
		HAL_GPIO_WritePin(A2_PORT,A2_PIN,(GPIO_PinState)((clock_wise[i] & 0x02) >> 1));
		HAL_GPIO_WritePin(B2_PORT,B2_PIN,(GPIO_PinState)((clock_wise[i] & 0x01) >> 0));
		
		if(i == 0 || i == 4)
		{
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR3 = PWM_increase[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
	
			
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR2 = PWM_decrease[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}	
		}
		
		else if(i == 2 || i == 6)
		{
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR2 = PWM_increase[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
			
			
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR3 = PWM_decrease[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
		}
	}
}

// If you use other channel of PWM, then change CCR
// Example: I use channel 1 instead of channel 3
// Then TIM1->CCR1 = ...;
void RunMotorCounter_clockwise(uint32_t time_delay_tau, int* counted_pulse)
{
	for(uint8_t i = 0; i < 8; i = i + 2)
	{
		HAL_GPIO_WritePin(A1_PORT,A1_PIN,(GPIO_PinState)((counter_clock_wise[i] & 0x08) >> 3));
		HAL_GPIO_WritePin(B1_PORT,B1_PIN,(GPIO_PinState)((counter_clock_wise[i] & 0x04) >> 2));
		HAL_GPIO_WritePin(A2_PORT,A2_PIN,(GPIO_PinState)((counter_clock_wise[i] & 0x02) >> 1));
		HAL_GPIO_WritePin(B2_PORT,B2_PIN,(GPIO_PinState)((counter_clock_wise[i] & 0x01) >> 0));
		
		if(i == 0 || i == 4)
		{
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR3 = PWM_increase[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
			
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR2 = PWM_decrease[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
		}
		
		else if(i == 2 || i == 6)
		{
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR2 = PWM_increase[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
			
			
			for(uint8_t j = 0; j < MODE; j++)
			{
				TIM1->CCR3 = PWM_decrease[j] * SET_ARR;
				delay_us(time_delay_tau / MODE);
				*counted_pulse = *counted_pulse + 1;
			}
		}
	}
}
void ResetPinState()
{
	HAL_GPIO_WritePin(A1_PORT,A1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(B1_PORT,B1_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A2_PORT,A2_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(B2_PORT,B2_PIN,GPIO_PIN_RESET);
}
