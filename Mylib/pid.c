#include "stm32f4xx.h"
float pid_position(volatile pid_struct* p_s){
		p_s->err = p_s->expect - p_s->cur_position;
		float p_t = p_s->K_P * p_s->err;
		float i_t = p_s->K_I * p_s->i;
		float d_t = -p_s->K_D * p_s->cur_speed;
		float out = p_t + i_t + d_t;
		p_s->i = p_s->i + p_s->err;
	  if (out > p_s->out_max) out = p_s->out_max;
    if (out < p_s->out_min) out = p_s->out_min;
		return out;
}
float pid_speed(volatile pid_struct* p_s){
		p_s->err = p_s->expect - p_s->cur_speed;
		float p_t = p_s->K_P * p_s->err;
		float i_t = p_s->K_I * p_s->i;
		float d_t = - p_s->K_D * (p_s->cur_speed - p_s->prev_speed)/p_s->dt;
		float out = p_t + i_t + d_t;
	  p_s->i = p_s->i + p_s->err;
		p_s->prev_speed = p_s->cur_speed;
	  if (out > p_s->out_max) out = p_s->out_max;
    if (out < p_s->out_min) out = p_s->out_min;
		return out;
}
