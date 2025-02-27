/*
 * motors.c
 *
 *  Created on: Oct 27, 2024
 *      Author: xiaofei
 */

#include "motors.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

typedef struct {
    float Kp;      // Proportional gain
    float Ki;      // Integral gain
    float Kd;
    float min_error;
    float max_error;
} PID_Param;

#define PID_TABLE_SIZE 7
PID_Param pid_table[PID_TABLE_SIZE] = {
	{0.005, 0.0000006, 0,6000, 8000},
    {0.01, 0.000002, 0.0001,4000, 6000},
    {0.03, 0.000006,  0.005, 3000.0, 4000},
    {0.08, 0.000003, 0.01, 2000, 3000},
	{0.4, 0.000008, 0.05, 1000, 2000},
    {0.5, 0.000009, 0.06, 500, 1000}  ,
	{0.3, 0.00001, 0.0007, 0, 500}
};


void get_motors_speed();
PID_Param* get_pid_params(float error);



typedef struct
{
	float current_speed;
	float expect_speed;
	int direction;
	float error; //for pid
} Motor;

Motor motor_a={0.0f, 0.0f, 1, 0.0f}, motor_b={0.0f, 0.0f, 1, 0.0f}, motor_c={0.0f, 0.0f, 1, 0.0f}, motor_d={0.0f, 0.0f, 1, 0.0f};

// motor_a encoder PA15、PB3 timer2 CH1,2, motor PE9、PE11 timer1 CH1,2
//motor_b encoder PB4、PB5 timer3 CH1, 2, motor PE13、PE14 timer1 CH3,4
//motor_c encoder PD12、PD13 timer4 CH1,2 ; motor PB14、PB15 timer12 CH1,2
//motor_d encoder PA0、PA1 timer5 CH1,2; motor PE5、PE6 timer9 CH1,2


double duration=500;
float latest_encoder_a=0;
float latest_encoder_b=0;
float latest_encoder_c=0;
float latest_encoder_d=0;
float encoder_value_a;
float encoder_value_b;
float encoder_value_c;
float encoder_value_d;
PID_Param* pid_params;

void get_motors_speed()
{
	latest_encoder_a = __HAL_TIM_GET_COUNTER(&htim2);
	latest_encoder_b = __HAL_TIM_GET_COUNTER(&htim3);
	latest_encoder_c = __HAL_TIM_GET_COUNTER(&htim4);
	latest_encoder_d = __HAL_TIM_GET_COUNTER(&htim5);
	 if (get_systick_ms() >= duration)
	  {
		  encoder_value_a = __HAL_TIM_GET_COUNTER(&htim2);
		  encoder_value_b = __HAL_TIM_GET_COUNTER(&htim3);
		  encoder_value_c = __HAL_TIM_GET_COUNTER(&htim4);
		  encoder_value_d = __HAL_TIM_GET_COUNTER(&htim5);

		  if ((htim2.Instance->CR1 & TIM_CR1_DIR) != 0)
//		  if (motor_a.direction==2) //moving backward
			  encoder_value_a=encoder_value_a-4000;
		  if ((htim3.Instance->CR1 & TIM_CR1_DIR) != 0)
//		  if (motor_b.direction==2)
			  encoder_value_b=encoder_value_b-4000;
		  if ((htim4.Instance->CR1 & TIM_CR1_DIR) != 0)
//		  if (motor_c.direction==2)
			  encoder_value_c=encoder_value_c-4000;
		  if ((htim5.Instance->CR1 & TIM_CR1_DIR) != 0)
//		  if (motor_d.direction==2)
			  encoder_value_d=encoder_value_d-4000;



		  if( encoder_value_a>-4000 && encoder_value_a<=4000 )
			  motor_a.current_speed=(encoder_value_a)/duration*1000;
		  if( encoder_value_b>-4000 && encoder_value_b<=4000 )
			  motor_b.current_speed=(encoder_value_b)/duration*1000;
		  if( encoder_value_c>-4000 && encoder_value_c<=4000 )
			  motor_c.current_speed=(encoder_value_c)/duration*1000;
		  if( encoder_value_d>-4000 && encoder_value_d<=4000 )
			  motor_d.current_speed=(encoder_value_d)/duration*1000;

		  set_systick_ms(0);
		  __HAL_TIM_SET_COUNTER(&htim2, 0);
		  __HAL_TIM_SET_COUNTER(&htim3, 0);
		  __HAL_TIM_SET_COUNTER(&htim4, 0);
		  __HAL_TIM_SET_COUNTER(&htim5, 0);

//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
//		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
//		  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 2000);
//		  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 2000);

	   }

}
void set_motors_speed()
{
	motor_a.expect_speed=2000;
	motor_b.expect_speed=2000;
	motor_c.expect_speed=2000;
	motor_d.expect_speed=2000;

}

PID_Param* get_pid_params(float error)
{
	float abs_error=fabs(error);
    for (int i = 0; i < PID_TABLE_SIZE; i++)
    {
        // Check if the error falls within the defined range
        if (abs_error > pid_table[i].min_error && abs_error <= pid_table[i].max_error) {
            return &pid_table[i]; // Return the parameters for the matching range
        }
    }
    // Return NULL if no match is found
    return NULL;
}

float integral_a;
float integral_out_a;
float derivative_a;
float prev_error_a=0;
float output_a;

float integral_b;
float integral_out_b;
float derivative_b;
float prev_error_b=0;
float output_b;

float integral_c;
float integral_out_c;
float derivative_c;
float prev_error_c=0;
float output_c;

float integral_d;
float integral_out_d;
float derivative_d;
float prev_error_d=0;
float output_d;
void set_motors_output()
{
	//motor_a
	motor_a.error=motor_a.expect_speed-motor_a.current_speed;

	pid_params = get_pid_params(motor_a.error);

	integral_a += motor_a.error;
	integral_out_a=pid_params->Ki*integral_a;

	if (fabs(integral_out_a)>4000)
	{
		integral_a=0;
	}

	output_a= pid_params->Kp*motor_a.error+ integral_out_a;

	derivative_a = (motor_a.error - prev_error_a) / duration *1000;
	output_a = pid_params->Kd * derivative_a+output_a;

	output_a+=motor_a.expect_speed/2;
	if (output_a>4000)
		output_a=4000;
	if (output_a<-4000)
		output_a=-4000;

	if (output_a>0)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, output_a);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	}

	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -output_a);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}

	//motor_b
	motor_b.error=motor_b.expect_speed-motor_b.current_speed;
	pid_params = get_pid_params(motor_b.error);

	integral_b += motor_b.error;
	integral_out_b=pid_params->Ki*integral_b;

	if (fabs(integral_out_b)>4000)
	{
		integral_b=0;
	}

	output_b= pid_params->Kp*motor_b.error+ integral_out_b;

	derivative_b = (motor_b.error - prev_error_b) / duration *1000;
	output_b = pid_params->Kd * derivative_b+output_b;

	output_b+=motor_b.expect_speed/2;
	if (output_b>4000)
		output_b=4000;
	if (output_b<-4000)
		output_b=-4000;

	if (output_b>0)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, output_b);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}

	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -output_b);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	}

	//motor_c
	motor_c.error=motor_c.expect_speed-motor_c.current_speed;
	pid_params = get_pid_params(motor_c.error);

	integral_c += motor_c.error;
	integral_out_c=pid_params->Ki*integral_c;

	if (fabs(integral_out_c)>4000)
	{
		integral_c=0;
	}

	output_c= pid_params->Kp*motor_c.error+ integral_out_c;

	derivative_c = (motor_c.error - prev_error_c) / duration *1000;
	output_c = pid_params->Kd * derivative_c+output_c;

	output_c+=motor_c.expect_speed/2;
	if (output_c>4000)
		output_c=4000;
	if (output_c<-4000)
		output_c=-4000;

	if (output_c>0)
	{
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, output_c);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
	}

	else
	{
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, -output_c);
		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
	}

	//motor_d
	motor_d.error=motor_d.expect_speed-motor_d.current_speed;
	pid_params = get_pid_params(motor_d.error);

	integral_d += motor_d.error;
	integral_out_d=pid_params->Ki*integral_d;

	if (fabs(integral_out_d)>4000)
	{
		integral_d=0;
	}

	output_d= pid_params->Kp*motor_d.error+ integral_out_d;

	derivative_d = (motor_d.error - prev_error_d) / duration *1000;
	output_d = pid_params->Kd * derivative_d+output_d;

	output_d+=motor_d.expect_speed/2;
	if (output_d>4000)
		output_d=4000;
	if (output_d<-4000)
		output_d=-4000;

	if (output_d>0)
	{
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, output_d);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
	}

	else
	{
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -output_d);
		__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	}


	if (get_systick_ms() >= duration)
	{
		prev_error_a=motor_a.error;
		prev_error_b=motor_b.error;
		prev_error_c=motor_c.error;
		prev_error_d=motor_d.error;
	}

}
