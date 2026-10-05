#ifndef __PID_H
#define __PID_H

typedef struct pid_struct{
		float K_P,K_I,K_D;
		float dt;
		float out_min,out_max;
		float cur_position;
		float expect;
		float err;
		float cur_speed,prev_speed;
		float i;
}pid_struct;

float pid_position(volatile pid_struct* p_s);
float pid_speed(volatile pid_struct* p_s);

#endif
