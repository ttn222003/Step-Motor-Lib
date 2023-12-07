#include "main.h"
#include "StepMotor.h"

// Create variable calculate
int ppr;
int delay;
int pulse;
int count = 0;

// Create variable control
int angle;
float vel;
int number_of_rotation;

int main(void)
{
    ppr = Pulse_Per_Round(angle);
	delay = Time_Delay_Tau(ppr, vel);
	pulse = Pulse(angle, number_of_rotation);

    ResetPinState();
	
	WritePWM_Increase(PWM_increase,MODE);
	WritePWM_Decrease(PWM_decrease, MODE);
	
    // If you use other TIMER or other channel of PWM then change htimX and TIM_CHANNEL_X, CCRX
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	TIM1->CCR2 = PWM_increase[MODE-1];
	TIM1->CCR3 = PWM_decrease[MODE-1];

    while (1)
    {
        // Run Motor
        RunMotorCounter_clockwise(delay, &count);
    }
}